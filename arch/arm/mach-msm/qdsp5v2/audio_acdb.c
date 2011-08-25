/* Copyright (c) 2009-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/android_pmem.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/uaccess.h>
#include <linux/msm_audio.h>
#include <linux/slab.h>
#include <mach/dal.h>
#include <mach/qdsp5v2/audio_dev_ctl.h>
#include <mach/qdsp5v2/audpp.h>
#include <mach/qdsp5v2/audpreproc.h>
#include <mach/qdsp5v2/qdsp5audppcmdi.h>
#include <mach/qdsp5v2/qdsp5audpreproccmdi.h>
#include <mach/qdsp5v2/qdsp5audpreprocmsg.h>
#include <mach/qdsp5v2/qdsp5audppmsg.h>
#include <mach/qdsp5v2/afe.h>
#include <mach/qdsp5v2/audio_acdbi.h>
#include <mach/qdsp5v2/acdb_commands.h>
#include <mach/qdsp5v2/audio_acdb_def.h>
#include <mach/debug_mm.h>

/* this is the ACDB device ID */
#define DALDEVICEID_ACDB		0x02000069
#define ACDB_PORT_NAME			"DAL00"
#define ACDB_CPU			SMD_APPS_MODEM
#define ACDB_BUF_SIZE			4096
#define PBE_BUF_SIZE                    (33*1024)

#define ACDB_VALUES_NOT_FILLED  	0
#define ACDB_VALUES_FILLED      	1
#define MAX_RETRY			10

/* rpc table index */
enum {
	ACDB_DalACDB_ioctl = DALDEVICE_FIRST_DEVICE_API_IDX
};

enum {
	CAL_DATA_READY	= 0x1,
	AUDPP_READY	= 0x2,
	AUDREC0_READY	= 0x4,
	AUDREC1_READY	= 0x8,
};


struct acdb_data {
	void *handle;

	u32 phys_addr;
	u8 *virt_addr;

	struct task_struct *cb_thread_task;
	struct auddev_evt_audcal_info *device_info;

	u32 acdb_state;
	struct audpp_event_callback audpp_cb;
	struct audpreproc_event_callback audpreproc_cb;

	struct audpp_cmd_cfg_object_params_pcm *pp_iir;
	struct audpp_cmd_cfg_cal_gain *calib_gain_rx;
	struct audpp_cmd_cfg_pbe *pbe_block;
	struct audpp_cmd_cfg_object_params_mbadrc *pp_mbadrc;
	struct audpreproc_cmd_cfg_agc_params *preproc_agc;
	struct audpreproc_cmd_cfg_iir_tuning_filter_params *preproc_iir;
	struct audpreproc_cmd_cfg_cal_gain *calib_gain_tx;
	struct acdb_mbadrc_block mbadrc_block;

	wait_queue_head_t wait;
	struct mutex acdb_mutex;
	u32 device_cb_compl;
	u32 audpp_cb_compl;
	u32 preproc_cb_compl;
	u8 preproc_stream_id;
	u8 audrec0_applied;
	u8 audrec1_applied;
	u32 multiple_sessions;
	u32 cur_tx_session;
	struct acdb_result acdb_result;
	u16 *pbe_extbuff;
	u16 *pbe_enable_flag;
	struct acdb_pbe_block *pbe_blk;

	spinlock_t dsp_lock;
	int dec_id;
	struct audpp_cmd_cfg_object_params_eqalizer eq;

	/*pmem info*/
	int pmem_fd;
	unsigned long paddr;
	unsigned long kvaddr;
	unsigned long pmem_len;
	struct file *file;
	/* pmem for get acdb blk */
	unsigned long	get_blk_paddr;
	u8		*get_blk_kvaddr;
};

static struct acdb_data		acdb_data;

struct acdb_cache_node {
	u32 node_status;
	s32 stream_id;
	u32 phys_addr_acdb_values;
	u8 *virt_addr_acdb_values;
	struct auddev_evt_audcal_info device_info;
};

/*for RX devices  acdb values are applied based on copp ID so
the depth of tx cache is MAX number of COPP supported in the system*/
struct acdb_cache_node acdb_cache_rx[MAX_COPP_NODE_SUPPORTED];

/*for TX devices acdb values are applied based on AUDREC session and
the depth of the tx cache is define by number of AUDREC sessions supported*/
struct acdb_cache_node acdb_cache_tx[MAX_AUDREC_SESSIONS];



static s32 acdb_set_calibration_blk(unsigned long arg)
{
	struct acdb_cmd_device acdb_cmd;
	s32 result = 0;

	MM_DBG("acdb_set_calibration_blk\n");
	if (copy_from_user(&acdb_cmd, (struct acdb_cmd_device *)arg,
			sizeof(acdb_cmd))) {
		MM_ERR("Failed copy command struct from user in"
			"acdb_set_calibration_blk\n");
		return -EFAULT;
	}
	acdb_cmd.phys_buf = (u32 *)acdb_data.paddr;

	MM_DBG("acdb_cmd.phys_buf %x\n", (u32)acdb_cmd.phys_buf);

	result = dalrpc_fcn_8(ACDB_DalACDB_ioctl, acdb_data.handle,
			(const void *)&acdb_cmd, sizeof(acdb_cmd),
			&acdb_data.acdb_result,
			sizeof(acdb_data.acdb_result));

	if (result < 0) {
		MM_ERR("ACDB=> Device Set RPC failure"
			" result = %d\n", result);
		return -EINVAL;
	} else {
		MM_ERR("ACDB=> Device Set RPC success\n");
		if (acdb_data.acdb_result.result == ACDB_RES_SUCCESS)
			MM_DBG("ACDB_SET_DEVICE Success\n");
		else if (acdb_data.acdb_result.result == ACDB_RES_FAILURE)
			MM_ERR("ACDB_SET_DEVICE Failure\n");
		else if (acdb_data.acdb_result.result == ACDB_RES_BADPARM)
			MM_ERR("ACDB_SET_DEVICE BadParams\n");
		else
			MM_ERR("Unknown error\n");
	}
	return result;
}

static s32 acdb_get_calibration_blk(unsigned long arg)
{
	s32 result = 0;
	struct acdb_cmd_device acdb_cmd;

	MM_DBG("acdb_get_calibration_blk\n");

	if (copy_from_user(&acdb_cmd, (struct acdb_cmd_device *)arg,
			sizeof(acdb_cmd))) {
		MM_ERR("Failed copy command struct from user in"
			"acdb_get_calibration_blk\n");
		return -EFAULT;
	}
	acdb_cmd.phys_buf = (u32 *)acdb_data.paddr;
	MM_ERR("acdb_cmd.phys_buf %x\n", (u32)acdb_cmd.phys_buf);

	result = dalrpc_fcn_8(ACDB_DalACDB_ioctl, acdb_data.handle,
			(const void *)&acdb_cmd, sizeof(acdb_cmd),
			&acdb_data.acdb_result,
			sizeof(acdb_data.acdb_result));

	if (result < 0) {
		MM_ERR("ACDB=> Device Get RPC failure"
			" result = %d\n", result);
		return -EINVAL;
	} else {
		MM_ERR("ACDB=> Device Get RPC Success\n");
		if (acdb_data.acdb_result.result == ACDB_RES_SUCCESS)
			MM_DBG("ACDB_GET_DEVICE Success\n");
		else if (acdb_data.acdb_result.result == ACDB_RES_FAILURE)
			MM_ERR("ACDB_GET_DEVICE Failure\n");
		else if (acdb_data.acdb_result.result == ACDB_RES_BADPARM)
			MM_ERR("ACDB_GET_DEVICE BadParams\n");
		else
			MM_ERR("Unknown error\n");
	}
	return result;
}

static int audio_acdb_open(struct inode *inode, struct file *file)
{
	MM_DBG("%s\n", __func__);
	return 0;
}
static int audio_acdb_release(struct inode *inode, struct file *file)
{
	MM_DBG("%s\n", __func__);
	return 0;
}

static long audio_acdb_ioctl(struct file *file, unsigned int cmd,
					unsigned long arg)
{
	int rc = 0;
	unsigned long flags = 0;
	struct msm_audio_pmem_info info;

	MM_DBG("%s\n", __func__);

	switch (cmd) {
	case AUDIO_SET_EQ:
		MM_DBG("IOCTL SET_EQ_CONFIG\n");
		if (copy_from_user(&acdb_data.eq.num_bands, (void *) arg,
				sizeof(acdb_data.eq) -
				(AUDPP_CMD_CFG_OBJECT_PARAMS_COMMON_LEN + 2))) {
			rc = -EFAULT;
			break;
		}
		spin_lock_irqsave(&acdb_data.dsp_lock, flags);
		acdb_data.dec_id    = 0;
		rc = audpp_dsp_set_eq(acdb_data.dec_id, 1,
			&acdb_data.eq, COPP);
		if (rc < 0)
			MM_ERR("AUDPP returned err =%d\n", rc);
		spin_unlock_irqrestore(&acdb_data.dsp_lock, flags);
		break;
	case AUDIO_REGISTER_PMEM:
		MM_DBG("AUDIO_REGISTER_PMEM\n");
		if (copy_from_user(&info, (void *) arg, sizeof(info))) {
			MM_ERR("Cannot copy from user\n");
			return -EFAULT;
		}
		rc = get_pmem_file(info.fd, &acdb_data.paddr,
					&acdb_data.kvaddr,
					&acdb_data.pmem_len,
					&acdb_data.file);
		if (rc == 0)
			acdb_data.pmem_fd = info.fd;
		break;
	case AUDIO_DEREGISTER_PMEM:
		if (acdb_data.pmem_fd)
			put_pmem_file(acdb_data.file);
		break;
	case AUDIO_SET_ACDB_BLK:
		MM_DBG("IOCTL AUDIO_SET_ACDB_BLK\n");
		rc = acdb_set_calibration_blk(arg);
		break;
	case AUDIO_GET_ACDB_BLK:
		MM_DBG("IOiCTL AUDIO_GET_ACDB_BLK\n");
		rc = acdb_get_calibration_blk(arg);
		break;
	default:
		MM_DBG("Unknown IOCTL%d\n", cmd);
		rc = -EINVAL;
	}
	return rc;
}

static const struct file_operations acdb_fops = {
	.owner = THIS_MODULE,
	.open = audio_acdb_open,
	.release = audio_acdb_release,
	.llseek = no_llseek,
	.unlocked_ioctl = audio_acdb_ioctl
};

struct miscdevice acdb_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "msm_acdb",
	.fops	= &acdb_fops,
};

s32 acdb_get_calibration_data(struct acdb_get_block *get_block)
{
	s32 result = -EINVAL;
	struct acdb_cmd_device acdb_cmd;
	struct acdb_result acdb_result;

	MM_DBG("acdb_get_calibration_data\n");

	acdb_cmd.command_id = ACDB_GET_DEVICE;
	acdb_cmd.network_id = 0x0108B153;
	acdb_cmd.device_id = get_block->acdb_id;
	acdb_cmd.sample_rate_id = get_block->sample_rate_id;
	acdb_cmd.interface_id = get_block->interface_id;
	acdb_cmd.algorithm_block_id = get_block->algorithm_block_id;
	acdb_cmd.total_bytes = get_block->total_bytes;
	acdb_cmd.phys_buf = (u32 *)acdb_data.get_blk_paddr;

	result = dalrpc_fcn_8(ACDB_DalACDB_ioctl, acdb_data.handle,
			(const void *)&acdb_cmd, sizeof(acdb_cmd),
			&acdb_result,
			sizeof(acdb_result));

	if (result < 0) {
		MM_ERR("ACDB=> Device Get RPC failure"
			" result = %d\n", result);
		goto err_state;
	} else {
		MM_DBG("ACDB=> Device Get RPC Success\n");
		if (acdb_result.result == ACDB_RES_SUCCESS) {
			MM_DBG("ACDB_GET_DEVICE Success\n");
			result = 0;
			memcpy(get_block->buf_ptr, acdb_data.get_blk_kvaddr,
					get_block->total_bytes);
		} else if (acdb_result.result == ACDB_RES_FAILURE)
			MM_ERR("ACDB_GET_DEVICE Failure\n");
		else if (acdb_result.result == ACDB_RES_BADPARM)
			MM_ERR("ACDB_GET_DEVICE BadParams\n");
		else
			MM_ERR("Unknown error\n");
	}
err_state:
	return result;
}
EXPORT_SYMBOL(acdb_get_calibration_data);

static u8 check_device_info_already_present(
		struct auddev_evt_audcal_info   audcal_info,
			struct acdb_cache_node *acdb_cache_free_node)
{
	if ((audcal_info.dev_id ==
				acdb_cache_free_node->device_info.dev_id) &&
		(audcal_info.sample_rate ==
				acdb_cache_free_node->device_info.\
				sample_rate) &&
			(audcal_info.acdb_id ==
				acdb_cache_free_node->device_info.acdb_id)) {
		MM_DBG("acdb values are already present\n");
		/*if acdb state is not set for CAL_DATA_READY and node status
		is filled, acdb state should be updated with CAL_DATA_READY
		state*/
		acdb_data.acdb_state |= CAL_DATA_READY;
		/*checking for cache node status if it is not filled then the
		acdb values are not cleaned from node so update node status
		with acdb value filled*/
		if (acdb_cache_free_node->node_status != ACDB_VALUES_FILLED) {
			MM_DBG("device was released earlier\n");
			acdb_cache_free_node->node_status = ACDB_VALUES_FILLED;
			return 2; /*node is presnet but status as not filled*/
		}
		return 1; /*node is present but status as filled*/
	}
	MM_DBG("copying device info into node\n");
	/*as device information is not present in cache copy
	the current device information into the node*/
	memcpy(&acdb_cache_free_node->device_info,
				 &audcal_info, sizeof(audcal_info));
	return 0; /*cant find the node*/
}

static struct acdb_iir_block *get_audpp_irr_block(void)
{
	struct header *prs_hdr;
	u32 index = 0;

	while (index < acdb_data.acdb_result.used_bytes) {
		prs_hdr = (struct header *)(acdb_data.virt_addr + index);
		if (prs_hdr->dbor_signature == DBOR_SIGNATURE) {
			if (prs_hdr->abid == ABID_AUDIO_IIR_RX) {
				if (prs_hdr->iid == IID_AUDIO_IIR_COEFF)
					return (struct acdb_iir_block *)
						(acdb_data.virt_addr + index
						 + sizeof(struct header));
			} else {
				index += prs_hdr->data_len +
						sizeof(struct header);
			}
		} else {
			break;
		}
	}
	return NULL;
}


static s32 acdb_fill_audpp_iir(void)
{
	struct acdb_iir_block *acdb_iir;
	s32 i = 0;

	acdb_iir = get_audpp_irr_block();
	if (acdb_iir == NULL) {
		MM_ERR("unable to find  audpp iir block returning\n");
		return -1;
	}
	memset(acdb_data.pp_iir, 0, sizeof(*acdb_data.pp_iir));

	acdb_data.pp_iir->common.cmd_id = AUDPP_CMD_CFG_OBJECT_PARAMS;
	acdb_data.pp_iir->common.stream = AUDPP_CMD_COPP_STREAM;
	acdb_data.pp_iir->common.stream_id = 0;
	acdb_data.pp_iir->common.obj_cfg = AUDPP_CMD_OBJ0_UPDATE;
	acdb_data.pp_iir->common.command_type = 0;

	acdb_data.pp_iir->active_flag = acdb_iir->enable_flag;
	acdb_data.pp_iir->num_bands = acdb_iir->stage_count;
	for (; i < acdb_iir->stage_count; i++) {
		acdb_data.pp_iir->params_filter.filter_4_params.
			numerator_filter[i].numerator_b0_filter_lsw =
			acdb_iir->stages[i].b0_lo;
		acdb_data.pp_iir->params_filter.filter_4_params.
			numerator_filter[i].numerator_b0_filter_msw =
			acdb_iir->stages[i].b0_hi;
		acdb_data.pp_iir->params_filter.filter_4_params.
			numerator_filter[i].numerator_b1_filter_lsw =
			acdb_iir->stages[i].b1_lo;
		acdb_data.pp_iir->params_filter.filter_4_params.
			numerator_filter[i].numerator_b1_filter_msw =
			acdb_iir->stages[i].b1_hi;
		acdb_data.pp_iir->params_filter.filter_4_params.
			numerator_filter[i].numerator_b2_filter_lsw =
			acdb_iir->stages[i].b2_lo;
		acdb_data.pp_iir->params_filter.filter_4_params.
			numerator_filter[i].numerator_b2_filter_msw =
			acdb_iir->stages[i].b2_hi;
		acdb_data.pp_iir->params_filter.filter_4_params.
			denominator_filter[i].denominator_a0_filter_lsw =
			acdb_iir->stages_a[i].a1_lo;
		acdb_data.pp_iir->params_filter.filter_4_params.
			denominator_filter[i].denominator_a0_filter_msw =
			acdb_iir->stages_a[i].a1_hi;
		acdb_data.pp_iir->params_filter.filter_4_params.
			denominator_filter[i].denominator_a1_filter_lsw =
			acdb_iir->stages_a[i].a2_lo;
		acdb_data.pp_iir->params_filter.filter_4_params.
			denominator_filter[i].denominator_a1_filter_msw =
			acdb_iir->stages_a[i].a2_hi;
		acdb_data.pp_iir->params_filter.filter_4_params.
			shift_factor_filter[i].shift_factor_0 =
			acdb_iir->shift_factor[i];
		acdb_data.pp_iir->params_filter.filter_4_params.pan_filter[i].
			pan_filter_0 = acdb_iir->pan[i];
	}
	return 0;
}

static void extract_mbadrc(u32 *phy_addr, struct header *prs_hdr, u32 *index)
{
	if (prs_hdr->iid == IID_MBADRC_EXT_BUFF) {
		MM_DBG("Got IID = IID_MBADRC_EXT_BUFF\n");
		*phy_addr = acdb_data.phys_addr	+ *index +
					sizeof(struct header);
		memcpy(acdb_data.mbadrc_block.ext_buf,
				(acdb_data.virt_addr + *index +
					sizeof(struct header)), 197*2);
		MM_DBG("phy_addr = %x\n", *phy_addr);
		*index += prs_hdr->data_len + sizeof(struct header);
	} else if (prs_hdr->iid == IID_MBADRC_BAND_CONFIG) {
		MM_DBG("Got IID == IID_MBADRC_BAND_CONFIG\n");
		memcpy(acdb_data.mbadrc_block.band_config, (acdb_data.virt_addr
					+ *index + sizeof(struct header)),
				sizeof(struct mbadrc_band_config_type) *
					 acdb_data.mbadrc_block.parameters.\
						mbadrc_num_bands);
		*index += prs_hdr->data_len + sizeof(struct header);
	} else if (prs_hdr->iid == IID_MBADRC_PARAMETERS) {
		struct mbadrc_parameter *tmp;
		tmp = (struct mbadrc_parameter *)(acdb_data.virt_addr + *index
						+ sizeof(struct header));
		MM_DBG("Got IID == IID_MBADRC_PARAMETERS\n");
		acdb_data.mbadrc_block.parameters.mbadrc_enable =
							tmp->mbadrc_enable;
		acdb_data.mbadrc_block.parameters.mbadrc_num_bands =
							tmp->mbadrc_num_bands;
		acdb_data.mbadrc_block.parameters.mbadrc_down_sample_level =
						tmp->mbadrc_down_sample_level;
		acdb_data.mbadrc_block.parameters.mbadrc_delay =
							tmp->mbadrc_delay;
		*index += prs_hdr->data_len + sizeof(struct header);
	}
}

static void get_audpp_mbadrc_block(u32 *phy_addr)
{
	struct header *prs_hdr;
	u32 index = 0;

	while (index < acdb_data.acdb_result.used_bytes) {
		prs_hdr = (struct header *)(acdb_data.virt_addr + index);

		if (prs_hdr->dbor_signature == DBOR_SIGNATURE) {
			if (prs_hdr->abid == ABID_AUDIO_MBADRC_RX) {
				if ((prs_hdr->iid == IID_MBADRC_EXT_BUFF)
					|| (prs_hdr->iid ==
						IID_MBADRC_BAND_CONFIG)
					|| (prs_hdr->iid ==
						IID_MBADRC_PARAMETERS)) {
					extract_mbadrc(phy_addr, prs_hdr,
								&index);
				}
			} else {
				index += prs_hdr->data_len +
						sizeof(struct header);
			}
		} else {
			break;
		}
	}
}

static s32 acdb_fill_audpp_mbadrc(void)
{
	u32 mbadrc_phys_addr = -1;

	get_audpp_mbadrc_block(&mbadrc_phys_addr);
	if (IS_ERR_VALUE(mbadrc_phys_addr)) {
		MM_ERR("failed to get mbadrc block\n");
		return -1;
	}

	memset(acdb_data.pp_mbadrc, 0, sizeof(*acdb_data.pp_mbadrc));

	acdb_data.pp_mbadrc->common.cmd_id = AUDPP_CMD_CFG_OBJECT_PARAMS;
	acdb_data.pp_mbadrc->common.stream = AUDPP_CMD_COPP_STREAM;
	acdb_data.pp_mbadrc->common.stream_id = 0;
	acdb_data.pp_mbadrc->common.obj_cfg = AUDPP_CMD_OBJ0_UPDATE;
	acdb_data.pp_mbadrc->common.command_type = 0;

	acdb_data.pp_mbadrc->enable = acdb_data.mbadrc_block.\
					parameters.mbadrc_enable;
	acdb_data.pp_mbadrc->num_bands =
				acdb_data.mbadrc_block.\
					parameters.mbadrc_num_bands;
	acdb_data.pp_mbadrc->down_samp_level =
				acdb_data.mbadrc_block.parameters.\
					mbadrc_down_sample_level;
	acdb_data.pp_mbadrc->adrc_delay =
				acdb_data.mbadrc_block.parameters.\
					mbadrc_delay;

	if (acdb_data.mbadrc_block.parameters.mbadrc_num_bands > 1)
		acdb_data.pp_mbadrc->ext_buf_size = (97 * 2) +
			(33 * 2 * (acdb_data.mbadrc_block.parameters.\
					mbadrc_num_bands - 2));

	acdb_data.pp_mbadrc->ext_partition = 0;
	acdb_data.pp_mbadrc->ext_buf_lsw = (u16)(mbadrc_phys_addr\
						 & 0xFFFF);
	acdb_data.pp_mbadrc->ext_buf_msw = (u16)((mbadrc_phys_addr\
						 & 0xFFFF0000) >> 16);
	memcpy(acdb_data.pp_mbadrc->adrc_band, acdb_data.mbadrc_block.\
					band_config,
		sizeof(struct mbadrc_band_config_type) *
			acdb_data.mbadrc_block.parameters.mbadrc_num_bands);
	return 0;
}

static struct acdb_calib_gain_rx *get_audpp_cal_gain(void)
{
	struct header *prs_hdr;
	u32 index = 0;

	while (index < acdb_data.acdb_result.used_bytes) {
		prs_hdr = (struct header *)(acdb_data.virt_addr + index);
		if (prs_hdr->dbor_signature == DBOR_SIGNATURE) {
			if (prs_hdr->abid == ABID_AUDIO_CALIBRATION_GAIN_RX) {
				if (prs_hdr->iid ==
					IID_AUDIO_CALIBRATION_GAIN_RX) {
					MM_DBG("Got audpp_calib_gain_rx"
					" block\n");
					return (struct acdb_calib_gain_rx *)
						(acdb_data.virt_addr + index
						+ sizeof(struct header));
				}
			} else {
				index += prs_hdr->data_len +
					sizeof(struct header);
			}
		} else {
			break;
		}
	}
	return NULL;
}

static s32 acdb_fill_audpp_cal_gain(void)
{
	struct acdb_calib_gain_rx *acdb_calib_gain_rx = NULL;

	acdb_calib_gain_rx = get_audpp_cal_gain();
	if (acdb_calib_gain_rx == NULL) {
		MM_ERR("unable to find  audpp"
			" calibration gain block returning\n");
		return -1;
	}
	MM_DBG("Calibration value"
		" for calib_gain_rx %d\n", acdb_calib_gain_rx->audppcalgain);
	memset(acdb_data.calib_gain_rx, 0, sizeof(*acdb_data.calib_gain_rx));

	acdb_data.calib_gain_rx->common.cmd_id = AUDPP_CMD_CFG_OBJECT_PARAMS;
	acdb_data.calib_gain_rx->common.stream = AUDPP_CMD_COPP_STREAM;
	acdb_data.calib_gain_rx->common.stream_id = 0;
	acdb_data.calib_gain_rx->common.obj_cfg = AUDPP_CMD_OBJ0_UPDATE;
	acdb_data.calib_gain_rx->common.command_type = 0;

	acdb_data.calib_gain_rx->audppcalgain =
				acdb_calib_gain_rx->audppcalgain;
	return 0;
}

static void extract_pbe_block(struct header *prs_hdr, u32 *index)
{
	if (prs_hdr->iid == IID_AUDIO_PBE_RX_ENABLE_FLAG) {
		MM_DBG("Got IID = IID_AUDIO_PBE_RX_ENABLE\n");
		acdb_data.pbe_enable_flag = (u16 *)(acdb_data.virt_addr +
							*index +
							sizeof(struct header));
		*index += prs_hdr->data_len + sizeof(struct header);
	} else if (prs_hdr->iid == IID_PBE_CONFIG_PARAMETERS) {
		MM_DBG("Got IID == IID_PBE_CONFIG_PARAMETERS\n");
		acdb_data.pbe_blk = (struct acdb_pbe_block *)
					(acdb_data.virt_addr + *index
					+ sizeof(struct header));
		*index += prs_hdr->data_len + sizeof(struct header);
	}
}

static s32 get_audpp_pbe_block(void)
{
	struct header *prs_hdr;
	u32 index = 0;
	s32 result = -1;

	while (index < acdb_data.acdb_result.used_bytes) {
		prs_hdr = (struct header *)(acdb_data.virt_addr + index);

		if (prs_hdr->dbor_signature == DBOR_SIGNATURE) {
			if (prs_hdr->abid == ABID_AUDIO_PBE_RX) {
				if ((prs_hdr->iid == IID_PBE_CONFIG_PARAMETERS)
					|| (prs_hdr->iid ==
						IID_AUDIO_PBE_RX_ENABLE_FLAG)) {
					extract_pbe_block(prs_hdr, &index);
					result = 0;
				}
			} else {
				index += prs_hdr->data_len +
					sizeof(struct header);
			}
		} else {
			break;
		}
	}
	return result;
}

static s32 acdb_fill_audpp_pbe(void)
{
	s32 result = -1;

	result = get_audpp_pbe_block();
	if (IS_ERR_VALUE(result))
		return result;
	memset(acdb_data.pbe_block, 0, sizeof(*acdb_data.pbe_block));

	acdb_data.pbe_block->common.cmd_id = AUDPP_CMD_CFG_OBJECT_PARAMS;
	acdb_data.pbe_block->common.stream = AUDPP_CMD_COPP_STREAM;
	acdb_data.pbe_block->common.stream_id = 0;
	acdb_data.pbe_block->common.obj_cfg = AUDPP_CMD_OBJ0_UPDATE;
	acdb_data.pbe_block->common.command_type = 0;
	acdb_data.pbe_block->pbe_enable = *acdb_data.pbe_enable_flag;

	acdb_data.pbe_block->realbassmix = acdb_data.pbe_blk->realbassmix;
	acdb_data.pbe_block->basscolorcontrol =
					acdb_data.pbe_blk->basscolorcontrol;
	acdb_data.pbe_block->mainchaindelay = acdb_data.pbe_blk->mainchaindelay;
	acdb_data.pbe_block->xoverfltorder = acdb_data.pbe_blk->xoverfltorder;
	acdb_data.pbe_block->bandpassfltorder =
					acdb_data.pbe_blk->bandpassfltorder;
	acdb_data.pbe_block->adrcdelay = acdb_data.pbe_blk->adrcdelay;
	acdb_data.pbe_block->downsamplelevel =
					acdb_data.pbe_blk->downsamplelevel;
	acdb_data.pbe_block->comprmstav = acdb_data.pbe_blk->comprmstav;
	acdb_data.pbe_block->expthreshold = acdb_data.pbe_blk->expthreshold;
	acdb_data.pbe_block->expslope = acdb_data.pbe_blk->expslope;
	acdb_data.pbe_block->compthreshold = acdb_data.pbe_blk->compthreshold;
	acdb_data.pbe_block->compslope = acdb_data.pbe_blk->compslope;
	acdb_data.pbe_block->cpmpattack_lsw = acdb_data.pbe_blk->cpmpattack_lsw;
	acdb_data.pbe_block->compattack_msw = acdb_data.pbe_blk->compattack_msw;
	acdb_data.pbe_block->comprelease_lsw =
					acdb_data.pbe_blk->comprelease_lsw;
	acdb_data.pbe_block->comprelease_msw =
					acdb_data.pbe_blk->comprelease_msw;
	acdb_data.pbe_block->compmakeupgain = acdb_data.pbe_blk->compmakeupgain;
	acdb_data.pbe_block->baselimthreshold =
					acdb_data.pbe_blk->baselimthreshold;
	acdb_data.pbe_block->highlimthreshold =
					acdb_data.pbe_blk->highlimthreshold;
	acdb_data.pbe_block->basslimmakeupgain =
					acdb_data.pbe_blk->basslimmakeupgain;
	acdb_data.pbe_block->highlimmakeupgain =
					acdb_data.pbe_blk->highlimmakeupgain;
	acdb_data.pbe_block->limbassgrc = acdb_data.pbe_blk->limbassgrc;
	acdb_data.pbe_block->limhighgrc = acdb_data.pbe_blk->limhighgrc;
	acdb_data.pbe_block->limdelay = acdb_data.pbe_blk->limdelay;
	memcpy(acdb_data.pbe_block->filter_coeffs,
		acdb_data.pbe_blk->filter_coeffs, sizeof(u16)*90);
	acdb_data.pbe_block->extpartition = 0;
	acdb_data.pbe_block->extbuffsize_lsw = PBE_BUF_SIZE;
	acdb_data.pbe_block->extbuffsize_msw = 0;
	acdb_data.pbe_block->extbuffstart_lsw = ((u32)acdb_data.pbe_extbuff
							& 0xFFFF);
	acdb_data.pbe_block->extbuffstart_msw = (((u32)acdb_data.pbe_extbuff
							& 0xFFFF0000) >> 16);
	return 0;
}


static s32 acdb_calibrate_audpp(void)
{
	s32	result = 0;

	result = acdb_fill_audpp_iir();
	if (!IS_ERR_VALUE(result)) {
		result = audpp_dsp_set_rx_iir(acdb_data.device_info->dev_id,
				acdb_data.pp_iir->active_flag,
					acdb_data.pp_iir, COPP);
		if (result) {
			MM_ERR("ACDB=> Failed to send IIR data to postproc\n");
			result = -EINVAL;
			goto done;
		} else
			MM_DBG("AUDPP is calibrated with IIR parameters"
					" for COPP ID %d\n",
						acdb_data.device_info->dev_id);
	}
	result = acdb_fill_audpp_mbadrc();
	if (!IS_ERR_VALUE(result)) {
		result = audpp_dsp_set_mbadrc(acdb_data.device_info->dev_id,
					acdb_data.pp_mbadrc->enable,
					acdb_data.pp_mbadrc, COPP);
		if (result) {
			MM_ERR("ACDB=> Failed to send MBADRC data to"
					" postproc\n");
			result = -EINVAL;
			goto done;
		} else
			MM_DBG("AUDPP is calibrated with MBADRC parameters"
					" for COPP ID %d\n",
					acdb_data.device_info->dev_id);
	}
	result = acdb_fill_audpp_cal_gain();
	if (!(IS_ERR_VALUE(result))) {
		result = audpp_dsp_set_gain_rx(acdb_data.device_info->dev_id,
					acdb_data.calib_gain_rx, COPP);
		if (result) {
			MM_ERR("ACDB=> Failed to send gain_rx"
				" data to postproc\n");
			result = -EINVAL;
			goto done;
		} else
			MM_DBG("AUDPP is calibrated with calib_gain_rx\n");
	}
	result = acdb_fill_audpp_pbe();
	if (!(IS_ERR_VALUE(result))) {
		result = audpp_dsp_set_pbe(acdb_data.device_info->dev_id,
					acdb_data.pbe_block->pbe_enable,
					acdb_data.pbe_block, COPP);
		if (result) {
			MM_ERR("ACDB=> Failed to send pbe block"
				"data to postproc\n");
			result = -EINVAL;
			goto done;
		}
		MM_DBG("AUDPP is calibarted with PBE\n");
	}
done:
	return result;
}

static struct acdb_agc_block *get_audpreproc_agc_block(void)
{
	struct header *prs_hdr;
	u32 index = 0;

	while (index < acdb_data.acdb_result.used_bytes) {
		prs_hdr = (struct header *)(acdb_data.virt_addr + index);
		if (prs_hdr->dbor_signature == DBOR_SIGNATURE) {
			if (prs_hdr->abid == ABID_AUDIO_AGC_TX) {
				if (prs_hdr->iid == IID_AUDIO_AGC_PARAMETERS) {
					MM_DBG("GOT ABID_AUDIO_AGC_TX\n");
					return (struct acdb_agc_block *)
						(acdb_data.virt_addr + index
						 + sizeof(struct header));
				}
			} else {
				index += prs_hdr->data_len +
						sizeof(struct header);
			}
		} else {
			break;
		}
	}
	return NULL;
}

static s32 acdb_fill_audpreproc_agc(void)
{
	struct acdb_agc_block	*acdb_agc;

	acdb_agc = get_audpreproc_agc_block();
	if (!acdb_agc) {
		MM_DBG("unable to find preproc agc parameters winding up\n");
		return -1;
	}
	memset(acdb_data.preproc_agc, 0, sizeof(*acdb_data.preproc_agc));
	acdb_data.preproc_agc->cmd_id = AUDPREPROC_CMD_CFG_AGC_PARAMS;
	acdb_data.preproc_agc->stream_id = acdb_data.preproc_stream_id;
	/* 0xFE00 to configure all parameters */
	acdb_data.preproc_agc->tx_agc_param_mask = 0xFFFF;

	if (acdb_agc->enable_status)
		acdb_data.preproc_agc->tx_agc_enable_flag =
			AUDPREPROC_CMD_TX_AGC_ENA_FLAG_ENA;
	else
		acdb_data.preproc_agc->tx_agc_enable_flag =
			AUDPREPROC_CMD_TX_AGC_ENA_FLAG_DIS;

	acdb_data.preproc_agc->comp_rlink_static_gain =
		acdb_agc->comp_rlink_static_gain;
	acdb_data.preproc_agc->comp_rlink_aig_flag =
		acdb_agc->comp_rlink_aig_flag;
	acdb_data.preproc_agc->expander_rlink_th =
		acdb_agc->exp_rlink_threshold;
	acdb_data.preproc_agc->expander_rlink_slope =
		acdb_agc->exp_rlink_slope;
	acdb_data.preproc_agc->compressor_rlink_th =
		acdb_agc->comp_rlink_threshold;
	acdb_data.preproc_agc->compressor_rlink_slope =
		acdb_agc->comp_rlink_slope;

	/* 0xFFF0 to configure all parameters */
	acdb_data.preproc_agc->tx_adc_agc_param_mask = 0xFFFF;

	acdb_data.preproc_agc->comp_rlink_aig_attackk =
		acdb_agc->comp_rlink_aig_attack_k;
	acdb_data.preproc_agc->comp_rlink_aig_leak_down =
		acdb_agc->comp_rlink_aig_leak_down;
	acdb_data.preproc_agc->comp_rlink_aig_leak_up =
		acdb_agc->comp_rlink_aig_leak_up;
	acdb_data.preproc_agc->comp_rlink_aig_max =
		acdb_agc->comp_rlink_aig_max;
	acdb_data.preproc_agc->comp_rlink_aig_min =
		acdb_agc->comp_rlink_aig_min;
	acdb_data.preproc_agc->comp_rlink_aig_releasek =
		acdb_agc->comp_rlink_aig_release_k;
	acdb_data.preproc_agc->comp_rlink_aig_leakrate_fast =
		acdb_agc->comp_rlink_aig_sm_leak_rate_fast;
	acdb_data.preproc_agc->comp_rlink_aig_leakrate_slow =
		acdb_agc->comp_rlink_aig_sm_leak_rate_slow;
	acdb_data.preproc_agc->comp_rlink_attackk_msw =
		acdb_agc->comp_rlink_attack_k_msw;
	acdb_data.preproc_agc->comp_rlink_attackk_lsw =
		acdb_agc->comp_rlink_attack_k_lsw;
	acdb_data.preproc_agc->comp_rlink_delay =
		acdb_agc->comp_rlink_delay;
	acdb_data.preproc_agc->comp_rlink_releasek_msw =
		acdb_agc->comp_rlink_release_k_msw;
	acdb_data.preproc_agc->comp_rlink_releasek_lsw =
		acdb_agc->comp_rlink_release_k_lsw;
	acdb_data.preproc_agc->comp_rlink_rms_tav =
		acdb_agc->comp_rlink_rms_trav;
	return 0;
}

static struct acdb_iir_block *get_audpreproc_irr_block(void)
{

	struct header *prs_hdr;
	u32 index = 0;

	while (index < acdb_data.acdb_result.used_bytes) {
		prs_hdr = (struct header *)(acdb_data.virt_addr + index);

		if (prs_hdr->dbor_signature == DBOR_SIGNATURE) {
			if (prs_hdr->abid == ABID_AUDIO_IIR_TX) {
				if (prs_hdr->iid == IID_AUDIO_IIR_COEFF)
					return (struct acdb_iir_block *)
						(acdb_data.virt_addr + index
						 + sizeof(struct header));
			} else {
				index += prs_hdr->data_len +
						sizeof(struct header);
			}
		} else {
			break;
		}
	}
	return NULL;
}


static s32 acdb_fill_audpreproc_iir(void)
{
	struct acdb_iir_block	*acdb_iir;


	acdb_iir =  get_audpreproc_irr_block();
	if (!acdb_iir) {
		MM_DBG("unable to find preproc iir parameters winding up\n");
		return -1;
	}
	memset(acdb_data.preproc_iir, 0, sizeof(*acdb_data.preproc_iir));

	acdb_data.preproc_iir->cmd_id =
		AUDPREPROC_CMD_CFG_IIR_TUNING_FILTER_PARAMS;
	acdb_data.preproc_iir->stream_id = acdb_data.preproc_stream_id;
	acdb_data.preproc_iir->active_flag = acdb_iir->enable_flag;
	acdb_data.preproc_iir->num_bands = acdb_iir->stage_count;

	acdb_data.preproc_iir->numerator_coeff_b0_filter0_lsw =
		acdb_iir->stages[0].b0_lo;
	acdb_data.preproc_iir->numerator_coeff_b0_filter0_msw =
		acdb_iir->stages[0].b0_hi;
	acdb_data.preproc_iir->numerator_coeff_b1_filter0_lsw =
		acdb_iir->stages[0].b1_lo;
	acdb_data.preproc_iir->numerator_coeff_b1_filter0_msw =
		acdb_iir->stages[0].b1_hi;
	acdb_data.preproc_iir->numerator_coeff_b2_filter0_lsw =
		acdb_iir->stages[0].b2_lo;
	acdb_data.preproc_iir->numerator_coeff_b2_filter0_msw =
		acdb_iir->stages[0].b2_hi;

	acdb_data.preproc_iir->numerator_coeff_b0_filter1_lsw =
		acdb_iir->stages[1].b0_lo;
	acdb_data.preproc_iir->numerator_coeff_b0_filter1_msw =
		acdb_iir->stages[1].b0_hi;
	acdb_data.preproc_iir->numerator_coeff_b1_filter1_lsw =
		acdb_iir->stages[1].b1_lo;
	acdb_data.preproc_iir->numerator_coeff_b1_filter1_msw =
		acdb_iir->stages[1].b1_hi;
	acdb_data.preproc_iir->numerator_coeff_b2_filter1_lsw =
		acdb_iir->stages[1].b2_lo;
	acdb_data.preproc_iir->numerator_coeff_b2_filter1_msw =
		acdb_iir->stages[1].b2_hi;

	acdb_data.preproc_iir->numerator_coeff_b0_filter2_lsw =
		acdb_iir->stages[2].b0_lo;
	acdb_data.preproc_iir->numerator_coeff_b0_filter2_msw =
		acdb_iir->stages[2].b0_hi;
	acdb_data.preproc_iir->numerator_coeff_b1_filter2_lsw =
		acdb_iir->stages[2].b1_lo;
	acdb_data.preproc_iir->numerator_coeff_b1_filter2_msw =
		acdb_iir->stages[2].b1_hi;
	acdb_data.preproc_iir->numerator_coeff_b2_filter2_lsw =
		acdb_iir->stages[2].b2_lo;
	acdb_data.preproc_iir->numerator_coeff_b2_filter2_msw =
		acdb_iir->stages[2].b2_hi;

	acdb_data.preproc_iir->numerator_coeff_b0_filter3_lsw =
		acdb_iir->stages[3].b0_lo;
	acdb_data.preproc_iir->numerator_coeff_b0_filter3_msw =
		acdb_iir->stages[3].b0_hi;
	acdb_data.preproc_iir->numerator_coeff_b1_filter3_lsw =
		acdb_iir->stages[3].b1_lo;
	acdb_data.preproc_iir->numerator_coeff_b1_filter3_msw =
		acdb_iir->stages[3].b1_hi;
	acdb_data.preproc_iir->numerator_coeff_b2_filter3_lsw =
		acdb_iir->stages[3].b2_lo;
	acdb_data.preproc_iir->numerator_coeff_b2_filter3_msw =
		acdb_iir->stages[3].b2_hi;

	acdb_data.preproc_iir->denominator_coeff_a0_filter0_lsw =
		acdb_iir->stages_a[0].a1_lo;
	acdb_data.preproc_iir->denominator_coeff_a0_filter0_msw =
		acdb_iir->stages_a[0].a1_hi;
	acdb_data.preproc_iir->denominator_coeff_a1_filter0_lsw =
		acdb_iir->stages_a[0].a2_lo;
	acdb_data.preproc_iir->denominator_coeff_a1_filter0_msw =
		acdb_iir->stages_a[0].a2_hi;

	acdb_data.preproc_iir->denominator_coeff_a0_filter1_lsw =
		acdb_iir->stages_a[1].a1_lo;
	acdb_data.preproc_iir->denominator_coeff_a0_filter1_msw =
		acdb_iir->stages_a[1].a1_hi;
	acdb_data.preproc_iir->denominator_coeff_a1_filter1_lsw =
		acdb_iir->stages_a[1].a2_lo;
	acdb_data.preproc_iir->denominator_coeff_a1_filter1_msw =
		acdb_iir->stages_a[1].a2_hi;

	acdb_data.preproc_iir->denominator_coeff_a0_filter2_lsw =
		acdb_iir->stages_a[2].a1_lo;
	acdb_data.preproc_iir->denominator_coeff_a0_filter2_msw =
		acdb_iir->stages_a[2].a1_hi;
	acdb_data.preproc_iir->denominator_coeff_a1_filter2_lsw =
		acdb_iir->stages_a[2].a2_lo;
	acdb_data.preproc_iir->denominator_coeff_a1_filter2_msw =
		acdb_iir->stages_a[2].a2_hi;

	acdb_data.preproc_iir->denominator_coeff_a0_filter3_lsw =
		acdb_iir->stages_a[3].a1_lo;
	acdb_data.preproc_iir->denominator_coeff_a0_filter3_msw =
		acdb_iir->stages_a[3].a1_hi;
	acdb_data.preproc_iir->denominator_coeff_a1_filter3_lsw =
		acdb_iir->stages_a[3].a2_lo;
	acdb_data.preproc_iir->denominator_coeff_a1_filter3_msw =
		acdb_iir->stages_a[3].a2_hi;

	acdb_data.preproc_iir->shift_factor_filter0 =
		acdb_iir->shift_factor[0];
	acdb_data.preproc_iir->shift_factor_filter1 =
		acdb_iir->shift_factor[1];
	acdb_data.preproc_iir->shift_factor_filter2 =
		acdb_iir->shift_factor[2];
	acdb_data.preproc_iir->shift_factor_filter3 =
		acdb_iir->shift_factor[3];

	acdb_data.preproc_iir->pan_of_filter0 =
		acdb_iir->pan[0];
	acdb_data.preproc_iir->pan_of_filter1 =
		acdb_iir->pan[1];
	acdb_data.preproc_iir->pan_of_filter2 =
		acdb_iir->pan[2];
	acdb_data.preproc_iir->pan_of_filter3 =
		acdb_iir->pan[3];
	return 0;
}

static struct acdb_calib_gain_tx *get_audpreproc_cal_gain(void)
{
	struct header *prs_hdr;
	u32 index = 0;

	while (index < acdb_data.acdb_result.used_bytes) {
		prs_hdr = (struct header *)(acdb_data.virt_addr + index);
		if (prs_hdr->dbor_signature == DBOR_SIGNATURE) {
			if (prs_hdr->abid == ABID_AUDIO_CALIBRATION_GAIN_TX) {
				if (prs_hdr->iid ==
					IID_AUDIO_CALIBRATION_GAIN_TX) {
					MM_DBG("Got audpreproc_calib_gain_tx"
					" block\n");
					return (struct acdb_calib_gain_tx *)
						(acdb_data.virt_addr + index
						+ sizeof(struct header));
				}
			} else {
				index += prs_hdr->data_len +
					sizeof(struct header);
			}
		} else {
			break;
		}
	}
	return NULL;
}

static s32 acdb_fill_audpreproc_cal_gain(void)
{
	struct acdb_calib_gain_tx *acdb_calib_gain_tx = NULL;

	acdb_calib_gain_tx = get_audpreproc_cal_gain();
	if (acdb_calib_gain_tx == NULL) {
		MM_ERR("unable to find  audpreproc"
			" calibration block returning\n");
		return -1;
	}
	MM_DBG("Calibration value"
		" for calib_gain_tx %d\n", acdb_calib_gain_tx->audprecalgain);
	memset(acdb_data.calib_gain_tx, 0, sizeof(*acdb_data.calib_gain_tx));

	acdb_data.calib_gain_tx->cmd_id =
					AUDPREPROC_CMD_CFG_CAL_GAIN_PARAMS;
	acdb_data.calib_gain_tx->stream_id = acdb_data.preproc_stream_id;
	acdb_data.calib_gain_tx->audprecalgain =
					acdb_calib_gain_tx->audprecalgain;
	return 0;
}

static struct acdb_rmc_block *get_rmc_blk(void)
{
	struct header *prs_hdr;
	u32 index = 0;

	while (index < acdb_data.acdb_result.used_bytes) {
		prs_hdr = (struct header *)(acdb_data.virt_addr + index);
		if (prs_hdr->dbor_signature == DBOR_SIGNATURE) {
			if (prs_hdr->abid == ABID_AUDIO_RMC_TX) {
				if (prs_hdr->iid ==
					IID_AUDIO_RMC_PARAM) {
					MM_DBG("Got afe_rmc block\n");
					return (struct acdb_rmc_block *)
						(acdb_data.virt_addr + index
						+ sizeof(struct header));
				}
			} else {
				index += prs_hdr->data_len +
					sizeof(struct header);
			}
		} else {
			break;
		}
	}
	return NULL;
}

s32 acdb_calibrate_audpreproc(void)
{
	s32	result = 0;
	struct acdb_rmc_block *acdb_rmc = NULL;

	result = acdb_fill_audpreproc_agc();
	if (!IS_ERR_VALUE(result)) {
		result = audpreproc_dsp_set_agc(acdb_data.preproc_agc, sizeof(
					struct audpreproc_cmd_cfg_agc_params));
		if (result) {
			MM_ERR("ACDB=> Failed to send AGC data to preproc)\n");
			result = -EINVAL;
			goto done;
		} else
			MM_DBG("AUDPREC is calibrated with AGC parameters"
				" for COPP ID %d and AUDREC session %d\n",
					acdb_data.device_info->dev_id,
					acdb_data.preproc_stream_id);
	}
	result = acdb_fill_audpreproc_iir();
	if (!IS_ERR_VALUE(result)) {
		result = audpreproc_dsp_set_iir(acdb_data.preproc_iir,
				sizeof(struct\
				audpreproc_cmd_cfg_iir_tuning_filter_params));
		if (result) {
			MM_ERR("ACDB=> Failed to send IIR data to preproc\n");
			result = -EINVAL;
			goto done;
		} else
			MM_DBG("audpreproc is calibrated with iir parameters"
			" for COPP ID %d and AUREC session %d\n",
					acdb_data.device_info->dev_id,
					acdb_data.preproc_stream_id);
	}
	result = acdb_fill_audpreproc_cal_gain();
	if (!(IS_ERR_VALUE(result))) {
		result = audpreproc_dsp_set_gain_tx(acdb_data.calib_gain_tx,
				sizeof(struct audpreproc_cmd_cfg_cal_gain));
		if (result) {
			MM_ERR("ACDB=> Failed to send calib_gain_tx"
				" data to preproc\n");
			result = -EINVAL;
			goto done;
		} else
			MM_DBG("AUDPREPROC is calibrated"
				" with calib_gain_tx\n");
	}
	acdb_rmc = get_rmc_blk();
	if (acdb_rmc != NULL) {
		result = afe_config_rmc_block(acdb_rmc);
		if (result) {
			MM_ERR("ACDB=> Failed to send rmc"
				" data to afe\n");
			result = -EINVAL;
			goto done;
		} else
			MM_DBG("AFE is calibrated with rmc params\n");
	} else
		MM_DBG("RMC block was not found\n");

done:
	return result;
}

static s32 acdb_send_calibration(void)
{
	s32 result = 0;

	if ((acdb_data.device_info->dev_type & RX_DEVICE) == 1) {
		result = acdb_calibrate_audpp();
		if (result)
			goto done;
	} else if ((acdb_data.device_info->dev_type & TX_DEVICE) == 2) {
		result = acdb_calibrate_audpreproc();
		if (acdb_data.preproc_stream_id == 1)
			acdb_data.audrec1_applied = 1;
		else
			acdb_data.audrec0_applied = 1;
		if (result)
			goto done;
	}
done:
	return result;
}

static void handle_tx_device_ready_callback(void)
{
	u8 i = 0;
	u8 ret = 0;
	acdb_cache_tx[acdb_data.cur_tx_session].node_status =
							ACDB_VALUES_FILLED;
	if (acdb_data.multiple_sessions) {
		for (i = 0; i < MAX_AUDREC_SESSIONS; i++) {
			/*check is to exclude copying acdb values in the
			current node pointed by acdb_data structure*/
			if (acdb_cache_tx[i].phys_addr_acdb_values !=
							acdb_data.phys_addr) {
				ret = check_device_info_already_present(\
							*acdb_data.device_info,
							&acdb_cache_tx[i]);
				if (ret) {
					memcpy((char *)acdb_cache_tx[i].\
						virt_addr_acdb_values,
						(char *)acdb_data.virt_addr,
								ACDB_BUF_SIZE);
					acdb_cache_tx[i].node_status =
							ACDB_VALUES_FILLED;
				}
			}
		}
		acdb_data.multiple_sessions = 0;
	}
	/*check wheather AUDREC enabled before device call backs*/
	if ((acdb_data.acdb_state & AUDREC0_READY) &&
					(!acdb_data.audrec0_applied)) {
		MM_DBG("AUDREC0 already enabled apply acdb values\n");
		acdb_send_calibration();
	}
	if ((acdb_data.acdb_state & AUDREC1_READY) &&
					(!acdb_data.audrec1_applied)) {
		MM_DBG("AUDREC1 already enabled apply acdb values\n");
		acdb_send_calibration();
	}
}

static struct acdb_cache_node *get_acdb_values_from_cache_tx(
						u32 preproc_stream_id)
{
	MM_DBG("searching node with stream_id %d\n", preproc_stream_id);
	if ((acdb_cache_tx[preproc_stream_id].stream_id == preproc_stream_id) &&
			(acdb_cache_tx[preproc_stream_id].node_status ==
					ACDB_VALUES_FILLED)) {
			return &acdb_cache_tx[preproc_stream_id];
	}
	MM_DBG("Error! in finding node\n");
	return NULL;
}

static void update_acdb_data_struct(struct acdb_cache_node *cur_node)
{
	if (cur_node) {
		acdb_data.device_info = &cur_node->device_info;
		acdb_data.virt_addr = cur_node->virt_addr_acdb_values;
		acdb_data.phys_addr = cur_node->phys_addr_acdb_values;
	} else
		MM_ERR("error in curent node\n");
}

static void send_acdb_values_for_active_devices(void)
{
	u32 i = 0;
	for (i = 0; i < MAX_COPP_NODE_SUPPORTED; i++) {
		if (acdb_cache_rx[i].node_status ==
					ACDB_VALUES_FILLED) {
			update_acdb_data_struct(&acdb_cache_rx[i]);
			if (acdb_data.acdb_state & CAL_DATA_READY)
				acdb_send_calibration();
		}
	}
}

static s32 acdb_get_calibration(void)
{
	struct acdb_cmd_get_device_table	acdb_cmd;
	s32					result = 0;
	u32 iterations = 0;

	MM_DBG("acdb state = %d\n", acdb_data.acdb_state);
	acdb_cmd.command_id = ACDB_GET_DEVICE_TABLE;
	acdb_cmd.device_id = acdb_data.device_info->acdb_id;
	acdb_cmd.network_id = 0x0108B153;
	acdb_cmd.sample_rate_id = acdb_data.device_info->sample_rate;
	acdb_cmd.total_bytes = ACDB_BUF_SIZE;
	acdb_cmd.phys_buf = (u32 *)acdb_data.phys_addr;

	do {
		result = dalrpc_fcn_8(ACDB_DalACDB_ioctl, acdb_data.handle,
				(const void *)&acdb_cmd, sizeof(acdb_cmd),
				&acdb_data.acdb_result,
				sizeof(acdb_data.acdb_result));

		if (result < 0) {
			MM_ERR("ACDB=> Device table RPC failure"
				" result = %d\n", result);
			goto error;
		}
		/*following check is introduced to handle boot up race
		condition between AUDCAL SW peers running on apps
		and modem (ACDB_RES_BADSTATE indicates modem AUDCAL SW is
		not in initialized sate) we need to retry to get ACDB
		values*/
		if (acdb_data.acdb_result.result == ACDB_RES_BADSTATE) {
			msleep(500);
			iterations++;
		} else if (acdb_data.acdb_result.result == ACDB_RES_SUCCESS) {
			MM_DBG("Modem query for acdb values is successful"
					" (iterations = %d)\n", iterations);
			acdb_data.acdb_state |= CAL_DATA_READY;
			return result;
		} else {
			MM_ERR("ACDB=> modem failed to fill acdb values,"
					" reuslt = %d, (iterations = %d)\n",
					acdb_data.acdb_result.result,
					iterations);
			goto error;
		}
	} while (iterations < MAX_RETRY);
	MM_ERR("ACDB=> AUDCAL SW on modem is not in intiailized state (%d)\n",
			acdb_data.acdb_result.result);
error:
	result = -EINVAL;
	return result;
}

static s32 initialize_rpc(void)
{
	s32 result = 0;

	result = daldevice_attach(DALDEVICEID_ACDB, ACDB_PORT_NAME,
			ACDB_CPU, &acdb_data.handle);

	if (result) {
		MM_ERR("ACDB=> Device Attach failed\n");
		result = -ENODEV;
		goto done;
	}
done:
	return result;
}

static u32 allocate_memory_acdb_cache_tx(void)
{
	u32 result = 0;
	u32 i = 0;
	u32 err = 0;
	/*initialize local cache */
	for (i = 0; i < MAX_AUDREC_SESSIONS; i++) {
		acdb_cache_tx[i].phys_addr_acdb_values =
					pmem_kalloc(ACDB_BUF_SIZE,
						(PMEM_MEMTYPE_EBI1
						| PMEM_ALIGNMENT_4K));

		if (IS_ERR((void *)acdb_cache_tx[i].phys_addr_acdb_values)) {
			MM_ERR("ACDB=> Cannot allocate physical memory\n");
			result = -ENOMEM;
			goto error;
		}
		acdb_cache_tx[i].virt_addr_acdb_values =
					ioremap(
					acdb_cache_tx[i].phys_addr_acdb_values,
						ACDB_BUF_SIZE);
		if (acdb_cache_tx[i].virt_addr_acdb_values == NULL) {
			MM_ERR("ACDB=> Could not map physical address\n");
			result = -ENOMEM;
			pmem_kfree(acdb_cache_tx[i].phys_addr_acdb_values);
			goto error;
		}
		memset(acdb_cache_tx[i].virt_addr_acdb_values, 0,
						ACDB_BUF_SIZE);
	}
	return result;
error:
	for (err = 0; err < i; err++) {
		iounmap(acdb_cache_tx[i].virt_addr_acdb_values);
		pmem_kfree(acdb_cache_tx[i].phys_addr_acdb_values);

	}
	return result;
}

static u32 allocate_memory_acdb_cache_rx(void)
{
	u32 result = 0;
	u32 i = 0;
	u32 err = 0;

	/*initialize local cache */
	for (i = 0; i < MAX_COPP_NODE_SUPPORTED; i++) {
		acdb_cache_rx[i].phys_addr_acdb_values =
					pmem_kalloc(ACDB_BUF_SIZE,
						(PMEM_MEMTYPE_EBI1
						| PMEM_ALIGNMENT_4K));

		if (IS_ERR((void *)acdb_cache_rx[i].phys_addr_acdb_values)) {
			MM_ERR("ACDB=> Can not allocate physical memory\n");
			result = -ENOMEM;
			goto error;
		}
		acdb_cache_rx[i].virt_addr_acdb_values =
					ioremap(
					acdb_cache_rx[i].phys_addr_acdb_values,
						ACDB_BUF_SIZE);
		if (acdb_cache_rx[i].virt_addr_acdb_values == NULL) {
			MM_ERR("ACDB=> Could not map physical address\n");
			result = -ENOMEM;
			pmem_kfree(acdb_cache_rx[i].phys_addr_acdb_values);
			goto error;
		}
		memset(acdb_cache_rx[i].virt_addr_acdb_values, 0,
						ACDB_BUF_SIZE);
	}
	return result;
error:
	for (err = 0; err < i; err++) {
		iounmap(acdb_cache_rx[i].virt_addr_acdb_values);
		pmem_kfree(acdb_cache_rx[i].phys_addr_acdb_values);

	}
	return result;
}

static u32 allocate_memory_acdb_get_blk(void)
{
	u32 result = 0;
	acdb_data.get_blk_paddr = pmem_kalloc(ACDB_BUF_SIZE,
						(PMEM_MEMTYPE_EBI1
						| PMEM_ALIGNMENT_4K));
	if (IS_ERR((void *)acdb_data.get_blk_paddr)) {
		MM_ERR("ACDB=> Cannot allocate physical memory\n");
		result = -ENOMEM;
		goto error;
	}
	acdb_data.get_blk_kvaddr = ioremap(acdb_data.get_blk_paddr,
					ACDB_BUF_SIZE);
	if (acdb_data.get_blk_kvaddr == NULL) {
		MM_ERR("ACDB=> Could not map physical address\n");
		result = -ENOMEM;
		pmem_kfree(acdb_data.get_blk_paddr);
		goto error;
	}
	memset(acdb_data.get_blk_kvaddr, 0, ACDB_BUF_SIZE);
error:
	return result;
}

static void free_memory_acdb_cache_rx(void)
{
	u32 i = 0;

	for (i = 0; i < MAX_COPP_NODE_SUPPORTED; i++) {
		iounmap(acdb_cache_rx[i].virt_addr_acdb_values);
		pmem_kfree(acdb_cache_rx[i].phys_addr_acdb_values);
	}
}

static void free_memory_acdb_cache_tx(void)
{
	u32 i = 0;

	for (i = 0; i < MAX_AUDREC_SESSIONS; i++) {
		iounmap(acdb_cache_tx[i].virt_addr_acdb_values);
		pmem_kfree(acdb_cache_tx[i].phys_addr_acdb_values);
	}
}

static void free_memory_acdb_get_blk(void)
{
	iounmap(acdb_data.get_blk_kvaddr);
	pmem_kfree(acdb_data.get_blk_paddr);
}

static s32 initialize_memory(void)
{
	s32 result = 0;

	result = allocate_memory_acdb_get_blk();
	if (result < 0) {
		MM_ERR("memory allocation for get blk failed\n");
		goto done;
	}

	result = allocate_memory_acdb_cache_rx();
	if (result < 0) {
		MM_ERR("memory allocation for rx cache is failed\n");
		free_memory_acdb_get_blk();
		goto done;
	}
	result = allocate_memory_acdb_cache_tx();
	if (result < 0) {
		MM_ERR("memory allocation for tx cache is failed\n");
		free_memory_acdb_get_blk();
		free_memory_acdb_cache_rx();
		goto done;
	}
	acdb_data.pp_iir = kmalloc(sizeof(*acdb_data.pp_iir),
		GFP_KERNEL);
	if (acdb_data.pp_iir == NULL) {
		MM_ERR("ACDB=> Could not allocate postproc iir memory\n");
		free_memory_acdb_get_blk();
		free_memory_acdb_cache_rx();
		free_memory_acdb_cache_tx();
		result = -ENOMEM;
		goto done;
	}

	acdb_data.pp_mbadrc = kmalloc(sizeof(*acdb_data.pp_mbadrc), GFP_KERNEL);
	if (acdb_data.pp_mbadrc == NULL) {
		MM_ERR("ACDB=> Could not allocate postproc mbadrc memory\n");
		free_memory_acdb_get_blk();
		free_memory_acdb_cache_rx();
		free_memory_acdb_cache_tx();
		kfree(acdb_data.pp_iir);
		result = -ENOMEM;
		goto done;
	}
	acdb_data.calib_gain_rx = kmalloc(sizeof(*acdb_data.calib_gain_rx),
							GFP_KERNEL);
	if (acdb_data.calib_gain_rx == NULL) {
		MM_ERR("ACDB=> Could not allocate"
			" postproc calib_gain_rx memory\n");
		free_memory_acdb_get_blk();
		free_memory_acdb_cache_rx();
		free_memory_acdb_cache_tx();
		kfree(acdb_data.pp_iir);
		kfree(acdb_data.pp_mbadrc);
		result = -ENOMEM;
		goto done;
	}

	acdb_data.preproc_agc = kmalloc(sizeof(*acdb_data.preproc_agc),
							GFP_KERNEL);
	if (acdb_data.preproc_agc == NULL) {
		MM_ERR("ACDB=> Could not allocate preproc agc memory\n");
		free_memory_acdb_get_blk();
		free_memory_acdb_cache_rx();
		free_memory_acdb_cache_tx();
		kfree(acdb_data.pp_iir);
		kfree(acdb_data.pp_mbadrc);
		kfree(acdb_data.calib_gain_rx);
		result = -ENOMEM;
		goto done;
	}

	acdb_data.preproc_iir = kmalloc(sizeof(*acdb_data.preproc_iir),
							GFP_KERNEL);
	if (acdb_data.preproc_iir == NULL) {
		MM_ERR("ACDB=> Could not allocate preproc iir memory\n");
		free_memory_acdb_get_blk();
		free_memory_acdb_cache_rx();
		free_memory_acdb_cache_tx();
		kfree(acdb_data.pp_iir);
		kfree(acdb_data.pp_mbadrc);
		kfree(acdb_data.calib_gain_rx);
		kfree(acdb_data.preproc_agc);
		result = -ENOMEM;
		goto done;
	}
	acdb_data.calib_gain_tx = kmalloc(sizeof(*acdb_data.calib_gain_tx),
							GFP_KERNEL);
	if (acdb_data.calib_gain_tx == NULL) {
		MM_ERR("ACDB=> Could not allocate"
			" preproc calib_gain_tx memory\n");
		free_memory_acdb_get_blk();
		free_memory_acdb_cache_rx();
		free_memory_acdb_cache_tx();
		kfree(acdb_data.pp_iir);
		kfree(acdb_data.pp_mbadrc);
		kfree(acdb_data.calib_gain_rx);
		kfree(acdb_data.preproc_agc);
		kfree(acdb_data.preproc_iir);
		result = -ENOMEM;
		goto done;
	}
	acdb_data.pbe_block = kmalloc(sizeof(*acdb_data.pbe_block),
						GFP_KERNEL);
	if (acdb_data.pbe_block == NULL) {
		MM_ERR("ACDB=> Could not allocate pbe_block memory\n");
		free_memory_acdb_get_blk();
		free_memory_acdb_cache_rx();
		free_memory_acdb_cache_tx();
		kfree(acdb_data.pp_iir);
		kfree(acdb_data.pp_mbadrc);
		kfree(acdb_data.calib_gain_rx);
		kfree(acdb_data.preproc_agc);
		kfree(acdb_data.preproc_iir);
		kfree(acdb_data.calib_gain_tx);
		result = -ENOMEM;
		goto done;
	}
	acdb_data.pbe_extbuff = (u16 *)(pmem_kalloc(PBE_BUF_SIZE,
					(PMEM_MEMTYPE_EBI1 |
					PMEM_ALIGNMENT_4K)));
	if (IS_ERR((void *)acdb_data.pbe_extbuff)) {
		MM_ERR("ACDB=> Cannot allocate physical memory\n");
		free_memory_acdb_get_blk();
		free_memory_acdb_cache_rx();
		free_memory_acdb_cache_tx();
		kfree(acdb_data.pp_iir);
		kfree(acdb_data.pp_mbadrc);
		kfree(acdb_data.calib_gain_rx);
		kfree(acdb_data.preproc_agc);
		kfree(acdb_data.preproc_iir);
		kfree(acdb_data.calib_gain_tx);
		kfree(acdb_data.pbe_block);
		result = -ENOMEM;
		goto done;
	}
done:
	return result;
}

static u32 free_acdb_cache_node(union auddev_evt_data *evt)
{
	u32 session_id;
	if ((evt->audcal_info.dev_type & TX_DEVICE) == 2) {
		session_id = find_first_bit(
				(unsigned long *)&(evt->audcal_info.sessions),
				sizeof(evt->audcal_info.sessions));
		MM_DBG("freeing node %d for tx device", session_id);
		acdb_cache_tx[session_id].
			node_status = ACDB_VALUES_NOT_FILLED;
	} else {
		if (--(acdb_cache_rx[evt->audcal_info.dev_id].stream_id) <= 0) {
			MM_DBG("freeing rx cache node %d\n",
						evt->audcal_info.dev_id);
			acdb_cache_rx[evt->audcal_info.dev_id].
				node_status = ACDB_VALUES_NOT_FILLED;
			acdb_cache_rx[evt->audcal_info.dev_id].stream_id = 0;
		}
	}
	return 0;
}

static void device_cb(u32 evt_id, union auddev_evt_data *evt, void *private)
{
	struct auddev_evt_audcal_info	audcal_info;
	struct acdb_cache_node *acdb_cache_free_node =  NULL;
	u32 stream_id = 0;
	u8 ret = 0;
	u8 count = 0;
	u8 i = 0;

	if (!((evt_id == AUDDEV_EVT_DEV_RDY) ||
		(evt_id == AUDDEV_EVT_DEV_RLS)) ||
		(evt->audcal_info.acdb_id == PSEUDO_ACDB_ID)) {
		goto done;
	}
	/*if session value is zero it indicates that device call back is for
	voice call we will drop the request as acdb values for voice call is
	not applied from acdb driver*/
	if (!evt->audcal_info.sessions) {
		MM_DBG("no active sessions and call back is for"
				" voice call\n");
		goto done;
	}
	if (evt_id == AUDDEV_EVT_DEV_RLS) {
		MM_DBG("got release command for dev %d\n",
					evt->audcal_info.dev_id);
		acdb_data.acdb_state &= ~CAL_DATA_READY;
		free_acdb_cache_node(evt);
		goto done;
	}
	audcal_info = evt->audcal_info;
	MM_DBG("dev_id = %d\n", audcal_info.dev_id);
	MM_DBG("sample_rate = %d\n", audcal_info.sample_rate);
	MM_DBG("acdb_id = %d\n", audcal_info.acdb_id);
	MM_DBG("sessions = %d\n", audcal_info.sessions);
	MM_DBG("acdb_state = %d\n", acdb_data.acdb_state);
	mutex_lock(&acdb_data.acdb_mutex);
	if (acdb_data.acdb_state & CAL_DATA_READY) {
		if ((audcal_info.dev_id ==
				 acdb_data.device_info->dev_id) &&
			(audcal_info.sample_rate ==
				 acdb_data.device_info->sample_rate) &&
			(audcal_info.acdb_id == acdb_data.device_info->
							acdb_id)) {
			MM_DBG("called for same device type and sample rate\n");
			if ((audcal_info.dev_type & TX_DEVICE) == 2) {
				if (!(acdb_data.acdb_state & AUDREC0_READY))
					acdb_data.audrec0_applied = 0;
				if (!(acdb_data.acdb_state & AUDREC1_READY))
					acdb_data.audrec1_applied = 0;
					acdb_data.acdb_state &= ~CAL_DATA_READY;
					goto update_cache;
			}
		} else
			/* state is updated to querry the modem for values */
			acdb_data.acdb_state &= ~CAL_DATA_READY;
	}
update_cache:
	if ((audcal_info.dev_type & TX_DEVICE) == 2) {
		/*loop is to take care of use case:- multiple Audrec
		sessions are routed before enabling the device in this use
		case we will get the sessions value as bits set for all the
		sessions routed before device enable, so we should take care
		of copying device info to all the sessions*/
		for (i = 0; i < MAX_AUDREC_SESSIONS; i++) {
			stream_id = ((audcal_info.sessions >> i) & 0x01);
			if (stream_id) {
				acdb_cache_free_node = 	&acdb_cache_tx[i];
				ret  = check_device_info_already_present(
							audcal_info,
							acdb_cache_free_node);
				acdb_cache_free_node->stream_id = i;
				acdb_data.cur_tx_session = i;
				count++;
			}
		}
		if (count > 1)
			acdb_data.multiple_sessions = 1;
	} else {
		acdb_cache_free_node = &acdb_cache_rx[audcal_info.dev_id];
		ret = check_device_info_already_present(audcal_info,
						acdb_cache_free_node);
		if (ret == 1) {
			MM_DBG("got device ready call back for another "
					"audplay task sessions on same COPP\n");
			/*stream_id is used to keep track of number of active*/
			/*sessions active on this device*/
			acdb_cache_free_node->stream_id++;
			mutex_unlock(&acdb_data.acdb_mutex);
			goto done;
		}
		acdb_cache_free_node->stream_id++;
	}
	update_acdb_data_struct(acdb_cache_free_node);
	acdb_data.device_cb_compl = 1;
	mutex_unlock(&acdb_data.acdb_mutex);
	wake_up(&acdb_data.wait);
done:
	return;
}


static s32 register_device_cb(void)
{
	s32 result = 0;

	result = auddev_register_evt_listner((AUDDEV_EVT_DEV_RDY
						| AUDDEV_EVT_DEV_RLS),
		AUDDEV_CLNT_AUDIOCAL, 0, device_cb, (void *)&acdb_data);

	if (result) {
		MM_ERR("ACDB=> Could not register device callback\n");
		result = -ENODEV;
		goto done;
	}
done:
	return result;
}

static void audpp_cb(void *private, u32 id, u16 *msg)
{
	MM_DBG("\n");
	if (id != AUDPP_MSG_CFG_MSG)
		goto done;

	if (msg[0] == AUDPP_MSG_ENA_DIS) {
		acdb_data.acdb_state &= ~AUDPP_READY;
		MM_DBG("AUDPP_MSG_ENA_DIS\n");
		goto done;
	}

	acdb_data.acdb_state |= AUDPP_READY;
	acdb_data.audpp_cb_compl = 1;
	wake_up(&acdb_data.wait);
done:
	return;
}


static void audpreproc_cb(void *private, u32 id, void *msg)
{
	struct audpreproc_cmd_enc_cfg_done_msg *tmp;

	if (id != AUDPREPROC_CMD_ENC_CFG_DONE_MSG)
		goto done;

	tmp = (struct audpreproc_cmd_enc_cfg_done_msg *)msg;
	acdb_data.preproc_stream_id = tmp->stream_id;
	MM_DBG("rec_enc_type = %x\n", tmp->rec_enc_type);
	if ((tmp->rec_enc_type & 0x8000) ==
				AUD_PREPROC_CONFIG_DISABLED) {
		if (acdb_data.preproc_stream_id == 0) {
			acdb_data.acdb_state &= ~AUDREC0_READY;
			acdb_data.audrec0_applied = 0;
		} else {
			acdb_data.acdb_state &= ~AUDREC1_READY;
			acdb_data.audrec1_applied = 0;
		}
		MM_DBG("AUD_PREPROC_CONFIG_DISABLED\n");
		goto done;
	}
	if (acdb_data.preproc_stream_id == 0)
		acdb_data.acdb_state |= AUDREC0_READY;
	else
		acdb_data.acdb_state |= AUDREC1_READY;
	acdb_data.preproc_cb_compl = 1;
	wake_up(&acdb_data.wait);
done:
	return;
}

static s32 register_audpp_cb(void)
{
	s32 result = 0;

	acdb_data.audpp_cb.fn = audpp_cb;
	acdb_data.audpp_cb.private = NULL;
	result = audpp_register_event_callback(&acdb_data.audpp_cb);
	if (result) {
		MM_ERR("ACDB=> Could not register audpp callback\n");
		result = -ENODEV;
		goto done;
	}
done:
	return result;
}

static s32 register_audpreproc_cb(void)
{
	s32 result = 0;

	acdb_data.audpreproc_cb.fn = audpreproc_cb;
	acdb_data.audpreproc_cb.private = NULL;
	result = audpreproc_register_event_callback(&acdb_data.audpreproc_cb);
	if (result) {
		MM_ERR("ACDB=> Could not register audpreproc callback\n");
		result = -ENODEV;
		goto done;
	}

done:
	return result;
}

static s32 acdb_initialize_data(void)
{
	s32	result = 0;

	mutex_init(&acdb_data.acdb_mutex);

	result = initialize_rpc();
	if (result)
		goto err;

	result = initialize_memory();
	if (result)
		goto err1;

	result = register_device_cb();
	if (result)
		goto err2;

	result = register_audpp_cb();
	if (result)
		goto err3;

	result = register_audpreproc_cb();
	if (result)
		goto err4;

	return result;

err4:
	result = audpreproc_unregister_event_callback(&acdb_data.audpreproc_cb);
	if (result)
		MM_ERR("ACDB=> Could not unregister audpreproc callback\n");
err3:
	result = audpp_unregister_event_callback(&acdb_data.audpp_cb);
	if (result)
		MM_ERR("ACDB=> Could not unregister audpp callback\n");
err2:
	result = auddev_unregister_evt_listner(AUDDEV_CLNT_AUDIOCAL, 0);
	if (result)
		MM_ERR("ACDB=> Could not unregister device callback\n");
err1:
	daldevice_detach(acdb_data.handle);
	acdb_data.handle = NULL;
err:
	return result;
}

static s32 acdb_calibrate_device(void *data)
{
	s32 result = 0;

	/* initialize driver */
	result = acdb_initialize_data();
	if (result)
		goto done;

	while (!kthread_should_stop()) {
		MM_DBG("Waiting for call back events\n");
		wait_event_interruptible(acdb_data.wait,
					(acdb_data.device_cb_compl
					| acdb_data.audpp_cb_compl
					| acdb_data.preproc_cb_compl));
		mutex_lock(&acdb_data.acdb_mutex);
		if (acdb_data.device_cb_compl) {
			acdb_data.device_cb_compl = 0;
			if (!(acdb_data.acdb_state & CAL_DATA_READY)) {
				result = acdb_get_calibration();
				if (result < 0) {
					mutex_unlock(&acdb_data.acdb_mutex);
					MM_ERR("Not able to get calibration "
						"data continue\n");
					continue;
				}
			}
			MM_DBG("acdb state = %d\n",
					 acdb_data.acdb_state);
			if ((acdb_data.device_info->dev_type & TX_DEVICE) == 2)
				handle_tx_device_ready_callback();
			else {
				acdb_cache_rx[acdb_data.device_info->dev_id]\
						.node_status =
						ACDB_VALUES_FILLED;
				if (acdb_data.acdb_state &
						AUDPP_READY) {
					MM_DBG("AUDPP already enabled "
							"apply acdb values\n");
					goto apply;
				}
			}
		}

		if (!(acdb_data.audpp_cb_compl ||
				acdb_data.preproc_cb_compl)) {
			MM_DBG("need to wait for either AUDPP / AUDPREPROC "
					"Event\n");
			mutex_unlock(&acdb_data.acdb_mutex);
			continue;
		} else {
			MM_DBG("got audpp / preproc call back\n");
			if (acdb_data.audpp_cb_compl) {
				send_acdb_values_for_active_devices();
				acdb_data.audpp_cb_compl = 0;
				mutex_unlock(&acdb_data.acdb_mutex);
				continue;
			} else {
				struct acdb_cache_node *acdb_cached_values;
				acdb_data.preproc_cb_compl = 0;
				acdb_cached_values =
					 get_acdb_values_from_cache_tx(
						acdb_data.preproc_stream_id);
				if (acdb_cached_values == NULL) {
					MM_DBG("ERROR: to get chached"
						" acdb values\n");
					mutex_unlock(&acdb_data.acdb_mutex);
					continue;
				}
				update_acdb_data_struct(acdb_cached_values);
			}
		}
apply:
		if (acdb_data.acdb_state & CAL_DATA_READY)
			result = acdb_send_calibration();

		mutex_unlock(&acdb_data.acdb_mutex);
	}
done:
	return 0;
}

static int __init acdb_init(void)
{

	s32 result = 0;

	memset(&acdb_data, 0, sizeof(acdb_data));
	spin_lock_init(&acdb_data.dsp_lock);
	acdb_data.cb_thread_task = kthread_run(acdb_calibrate_device,
		NULL, "acdb_cb_thread");

	if (IS_ERR(acdb_data.cb_thread_task)) {
		MM_ERR("ACDB=> Could not register cb thread\n");
		result = -ENODEV;
		goto err;
	}

	init_waitqueue_head(&acdb_data.wait);

	return misc_register(&acdb_misc);
err:
	return result;
}

static void __exit acdb_exit(void)
{
	s32	result = 0;
	u32 i = 0;

	result = auddev_unregister_evt_listner(AUDDEV_CLNT_AUDIOCAL, 0);
	if (result)
		MM_ERR("ACDB=> Could not unregister device callback\n");

	result = audpp_unregister_event_callback(&acdb_data.audpp_cb);
	if (result)
		MM_ERR("ACDB=> Could not unregister audpp callback\n");

	result = audpreproc_unregister_event_callback(&acdb_data.\
				audpreproc_cb);
	if (result)
		MM_ERR("ACDB=> Could not unregister audpreproc callback\n");

	result = kthread_stop(acdb_data.cb_thread_task);
	if (result)
		MM_ERR("ACDB=> Could not stop kthread\n");

	free_memory_acdb_get_blk();

	for (i = 0; i < MAX_COPP_NODE_SUPPORTED; i++) {
		if (i < MAX_AUDREC_SESSIONS) {
			iounmap(acdb_cache_tx[i].virt_addr_acdb_values);
			pmem_kfree(acdb_cache_tx[i].phys_addr_acdb_values);
		}
		iounmap(acdb_cache_rx[i].virt_addr_acdb_values);
		pmem_kfree(acdb_cache_rx[i].phys_addr_acdb_values);
	}
	kfree(acdb_data.device_info);
	kfree(acdb_data.pp_iir);
	kfree(acdb_data.pp_mbadrc);
	kfree(acdb_data.preproc_agc);
	kfree(acdb_data.preproc_iir);
	pmem_kfree((int32_t)acdb_data.pbe_extbuff);
	mutex_destroy(&acdb_data.acdb_mutex);
	memset(&acdb_data, 0, sizeof(acdb_data));
}

late_initcall(acdb_init);
module_exit(acdb_exit);

MODULE_DESCRIPTION("MSM 7x30 Audio ACDB driver");
MODULE_LICENSE("GPL v2");
