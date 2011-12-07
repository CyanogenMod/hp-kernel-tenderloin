/* Copyright (c) 2002,2007-2011, Code Aurora Forum. All rights reserved.
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
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/slab.h>

#include "kgsl.h"
#include "kgsl_device.h"
#include "kgsl_yamato.h"
#include "kgsl_log.h"
#include "kgsl_pm4types.h"
#include "kgsl_ringbuffer.h"
#include "kgsl_cmdstream.h"
#include "kgsl_cffdump.h"

#include "yamato_reg.h"

#define VALID_STATUS_COUNT_MAX	10
#define GSL_RB_NOP_SIZEDWORDS				2
/* protected mode error checking below register address 0x800
*  note: if CP_INTERRUPT packet is used then checking needs
*  to change to below register address 0x7C8
*/
#define GSL_RB_PROTECTED_MODE_CONTROL		0x200001F2

#define GSL_CP_INT_MASK \
	(CP_INT_CNTL__SW_INT_MASK | \
	CP_INT_CNTL__T0_PACKET_IN_IB_MASK | \
	CP_INT_CNTL__OPCODE_ERROR_MASK | \
	CP_INT_CNTL__PROTECTED_MODE_ERROR_MASK | \
	CP_INT_CNTL__RESERVED_BIT_ERROR_MASK | \
	CP_INT_CNTL__IB_ERROR_MASK | \
	CP_INT_CNTL__IB2_INT_MASK | \
	CP_INT_CNTL__IB1_INT_MASK | \
	CP_INT_CNTL__RB_INT_MASK)

#define YAMATO_PFP_FW "yamato_pfp.fw"
#define YAMATO_PM4_FW "yamato_pm4.fw"
#define LEIA_PFP_470_FW "leia_pfp_470.fw"
#define LEIA_PM4_470_FW "leia_pm4_470.fw"

/*  ringbuffer size log2 quadwords equivalent */
inline unsigned int kgsl_ringbuffer_sizelog2quadwords(unsigned int sizedwords)
{
	unsigned int sizelog2quadwords = 0;
	int i = sizedwords >> 1;

	while (i >>= 1)
		sizelog2quadwords++;

	return sizelog2quadwords;
}


/* functions */
void kgsl_cp_intrcallback(struct kgsl_device *device)
{
	unsigned int status = 0, num_reads = 0, master_status = 0;
	struct kgsl_yamato_device *yamato_device = KGSL_YAMATO_DEVICE(device);
	struct kgsl_ringbuffer *rb = &yamato_device->ringbuffer;

	kgsl_yamato_regread_isr(device, REG_MASTER_INT_SIGNAL, &master_status);
	while (!status && (num_reads < VALID_STATUS_COUNT_MAX) &&
		(master_status & MASTER_INT_SIGNAL__CP_INT_STAT)) {
		kgsl_yamato_regread_isr(device, REG_CP_INT_STATUS, &status);
		kgsl_yamato_regread_isr(device, REG_MASTER_INT_SIGNAL,
					&master_status);
		num_reads++;
	}
	if (num_reads > 1)
		KGSL_DRV_WARN(device,
			"Looped %d times to read REG_CP_INT_STATUS\n",
			num_reads);
	if (!status) {
		if (master_status & MASTER_INT_SIGNAL__CP_INT_STAT) {
			/* This indicates that we could not read CP_INT_STAT.
			 * As a precaution just wake up processes so
			 * they can check their timestamps. Since, we
			 * did not ack any interrupts this interrupt will
			 * be generated again */
			KGSL_DRV_WARN(device, "Unable to read CP_INT_STATUS\n");
			wake_up_interruptible_all(&device->wait_queue);
		} else
			KGSL_DRV_WARN(device, "Spurious interrput detected\n");
		return;
	}

	if (status & CP_INT_CNTL__RB_INT_MASK) {
		/* signal intr completion event */
		unsigned int enableflag = 0;
		kgsl_sharedmem_writel(&rb->device->memstore,
			KGSL_DEVICE_MEMSTORE_OFFSET(ts_cmp_enable),
			enableflag);
		wmb();
		KGSL_CMD_WARN(rb->device, "ringbuffer rb interrupt\n");
	}

	if (status & CP_INT_CNTL__T0_PACKET_IN_IB_MASK) {
		KGSL_CMD_CRIT(rb->device,
			"ringbuffer TO packet in IB interrupt\n");
		kgsl_yamato_regwrite_isr(rb->device, REG_CP_INT_CNTL, 0);
	}
	if (status & CP_INT_CNTL__OPCODE_ERROR_MASK) {
		KGSL_CMD_CRIT(rb->device,
			"ringbuffer opcode error interrupt\n");
		kgsl_yamato_regwrite_isr(rb->device, REG_CP_INT_CNTL, 0);
	}
	if (status & CP_INT_CNTL__PROTECTED_MODE_ERROR_MASK) {
		KGSL_CMD_CRIT(rb->device,
			"ringbuffer protected mode error interrupt\n");
		kgsl_yamato_regwrite_isr(rb->device, REG_CP_INT_CNTL, 0);
	}
	if (status & CP_INT_CNTL__RESERVED_BIT_ERROR_MASK) {
		KGSL_CMD_CRIT(rb->device,
			"ringbuffer reserved bit error interrupt\n");
		kgsl_yamato_regwrite_isr(rb->device, REG_CP_INT_CNTL, 0);
	}
	if (status & CP_INT_CNTL__IB_ERROR_MASK) {
		KGSL_CMD_CRIT(rb->device,
			"ringbuffer IB error interrupt\n");
		kgsl_yamato_regwrite_isr(rb->device, REG_CP_INT_CNTL, 0);
	}
	if (status & CP_INT_CNTL__SW_INT_MASK)
		KGSL_CMD_INFO(rb->device, "ringbuffer software interrupt\n");

	if (status & CP_INT_CNTL__IB2_INT_MASK)
		KGSL_CMD_INFO(rb->device, "ringbuffer ib2 interrupt\n");

	if (status & (~GSL_CP_INT_MASK))
		KGSL_CMD_WARN(rb->device,
			"bad bits in REG_CP_INT_STATUS %08x\n", status);

	/* only ack bits we understand */
	status &= GSL_CP_INT_MASK;
	kgsl_yamato_regwrite_isr(device, REG_CP_INT_ACK, status);

	if (status & (CP_INT_CNTL__IB1_INT_MASK | CP_INT_CNTL__RB_INT_MASK)) {
		KGSL_CMD_WARN(rb->device, "ringbuffer ib1/rb interrupt\n");
		wake_up_interruptible_all(&device->wait_queue);
		atomic_notifier_call_chain(&(device->ts_notifier_list),
					   KGSL_DEVICE_YAMATO,
					   NULL);
	}
}

static void kgsl_ringbuffer_submit(struct kgsl_ringbuffer *rb)
{
	BUG_ON(rb->wptr == 0);

	GSL_RB_UPDATE_WPTR_POLLING(rb);
	/* Drain write buffer and data memory barrier */
	dsb();
	wmb();

	/* Memory fence to ensure all data has posted.  On some systems,
	* like 7x27, the register block is not allocated as strongly ordered
	* memory.  Adding a memory fence ensures ordering during ringbuffer
	* submits.*/
	mb();
	outer_sync();

	kgsl_yamato_regwrite(rb->device, REG_CP_RB_WPTR, rb->wptr);

	rb->flags |= KGSL_FLAGS_ACTIVE;
}

static int
kgsl_ringbuffer_waitspace(struct kgsl_ringbuffer *rb, unsigned int numcmds,
			  int wptr_ahead)
{
	int nopcount;
	unsigned int freecmds;
	unsigned int *cmds;
	uint cmds_gpu;

	/* if wptr ahead, fill the remaining with NOPs */
	if (wptr_ahead) {
		/* -1 for header */
		nopcount = rb->sizedwords - rb->wptr - 1;

		cmds = (unsigned int *)rb->buffer_desc.hostptr + rb->wptr;
		cmds_gpu = rb->buffer_desc.gpuaddr + sizeof(uint)*rb->wptr;

		GSL_RB_WRITE(cmds, cmds_gpu, pm4_nop_packet(nopcount));

		/* Make sure that rptr is not 0 before submitting
		 * commands at the end of ringbuffer. We do not
		 * want the rptr and wptr to become equal when
		 * the ringbuffer is not empty */
		do {
			GSL_RB_GET_READPTR(rb, &rb->rptr);
		} while (!rb->rptr);

		rb->wptr++;

		kgsl_ringbuffer_submit(rb);

		rb->wptr = 0;
	}

	/* wait for space in ringbuffer */
	do {
		GSL_RB_GET_READPTR(rb, &rb->rptr);

		freecmds = rb->rptr - rb->wptr;

	} while ((freecmds != 0) && (freecmds <= numcmds));

	return 0;
}


static unsigned int *kgsl_ringbuffer_allocspace(struct kgsl_ringbuffer *rb,
					     unsigned int numcmds)
{
	unsigned int	*ptr = NULL;
	int				status = 0;

	BUG_ON(numcmds >= rb->sizedwords);

	GSL_RB_GET_READPTR(rb, &rb->rptr);
	/* check for available space */
	if (rb->wptr >= rb->rptr) {
		/* wptr ahead or equal to rptr */
		/* reserve dwords for nop packet */
		if ((rb->wptr + numcmds) > (rb->sizedwords -
				GSL_RB_NOP_SIZEDWORDS))
			status = kgsl_ringbuffer_waitspace(rb, numcmds, 1);
	} else {
		/* wptr behind rptr */
		if ((rb->wptr + numcmds) >= rb->rptr)
			status  = kgsl_ringbuffer_waitspace(rb, numcmds, 0);
		/* check for remaining space */
		/* reserve dwords for nop packet */
		if ((rb->wptr + numcmds) > (rb->sizedwords -
				GSL_RB_NOP_SIZEDWORDS))
			status = kgsl_ringbuffer_waitspace(rb, numcmds, 1);
	}

	if (status == 0) {
		ptr = (unsigned int *)rb->buffer_desc.hostptr + rb->wptr;
		rb->wptr += numcmds;
	}

	return ptr;
}

static int _load_firmware(struct kgsl_device *device, const char *fwfile,
			  void **data, int *len)
{
	const struct firmware *fw = NULL;
	int ret;

	ret = request_firmware(&fw, fwfile, device->dev);

	if (ret) {
		KGSL_DRV_ERR(device, "request_firmware(%s) failed: %d\n",
			     fwfile, ret);
		return ret;
	}

	*data = kmalloc(fw->size, GFP_KERNEL);

	if (*data) {
		memcpy(*data, fw->data, fw->size);
		*len = fw->size;
	} else
		KGSL_MEM_ERR(device, "kmalloc(%d) failed\n", fw->size);

	release_firmware(fw);
	return (*data != NULL) ? 0 : -ENOMEM;
}

static int kgsl_ringbuffer_load_pm4_ucode(struct kgsl_device *device)
{
	struct kgsl_yamato_device *yamato_device = KGSL_YAMATO_DEVICE(device);
	const char *fwfile;
	int i, ret = 0;

	if (device->chip_id == KGSL_CHIPID_LEIA_REV470)
		fwfile =  LEIA_PM4_470_FW;
	else
		fwfile =  YAMATO_PM4_FW;

	if (yamato_device->pm4_fw == NULL) {
		int len;
		unsigned int *ptr;

		ret = _load_firmware(device, fwfile, (void *) &ptr, &len);
		if (ret)
			goto err;

		/* PM4 size is 3 dword aligned plus 1 dword of version */
		if (len % ((sizeof(uint32_t) * 3)) != sizeof(uint32_t)) {
			KGSL_DRV_ERR(device, "Bad firmware size: %d\n", len);
			ret = -EINVAL;
			goto err;
		}

		yamato_device->pm4_fw_size = len / sizeof(uint32_t);
		yamato_device->pm4_fw = ptr;
	}

	KGSL_DRV_INFO(device, "loading pm4 ucode version: %d\n",
		yamato_device->pm4_fw[0]);

	kgsl_yamato_regwrite(device, REG_CP_DEBUG, 0x02000000);
	kgsl_yamato_regwrite(device, REG_CP_ME_RAM_WADDR, 0);
	for (i = 1; i < yamato_device->pm4_fw_size; i++)
		kgsl_yamato_regwrite(device, REG_CP_ME_RAM_DATA,
				     yamato_device->pm4_fw[i]);
err:
	return ret;
}

static int kgsl_ringbuffer_load_pfp_ucode(struct kgsl_device *device)
{
	struct kgsl_yamato_device *yamato_device = KGSL_YAMATO_DEVICE(device);
	const char *fwfile;
	int i, ret = 0;

	if (device->chip_id == KGSL_CHIPID_LEIA_REV470)
		fwfile =  LEIA_PFP_470_FW;
	else
		fwfile = YAMATO_PFP_FW;

	if (yamato_device->pfp_fw == NULL) {
		int len;
		unsigned int *ptr;

		ret = _load_firmware(device, fwfile, (void *) &ptr, &len);
		if (ret)
			goto err;

		/* PFP size shold be dword aligned */
		if (len % sizeof(uint32_t) != 0) {
			KGSL_DRV_ERR(device, "Bad firmware size: %d\n", len);
			ret = -EINVAL;
			goto err;
		}

		yamato_device->pfp_fw_size = len / sizeof(uint32_t);
		yamato_device->pfp_fw = ptr;
	}

	KGSL_DRV_INFO(device, "loading pfp ucode version: %d\n",
		yamato_device->pfp_fw[0]);

	kgsl_yamato_regwrite(device, REG_CP_PFP_UCODE_ADDR, 0);
	for (i = 1; i < yamato_device->pfp_fw_size; i++)
		kgsl_yamato_regwrite(device, REG_CP_PFP_UCODE_DATA,
				     yamato_device->pfp_fw[i]);
err:
	return ret;
}

int kgsl_ringbuffer_start(struct kgsl_ringbuffer *rb, unsigned int init_ram)
{
	int status;
	/*cp_rb_cntl_u cp_rb_cntl; */
	union reg_cp_rb_cntl cp_rb_cntl;
	unsigned int *cmds, rb_cntl;
	struct kgsl_device *device = rb->device;
	uint cmds_gpu;

	if (rb->flags & KGSL_FLAGS_STARTED)
		return 0;

	if (init_ram) {
		rb->timestamp = 0;
		GSL_RB_INIT_TIMESTAMP(rb);
	}

	kgsl_sharedmem_set(&rb->memptrs_desc, 0, 0,
			   sizeof(struct kgsl_rbmemptrs));

	kgsl_sharedmem_set(&rb->buffer_desc, 0, 0xAA,
			   (rb->sizedwords << 2));

	kgsl_yamato_regwrite(device, REG_CP_RB_WPTR_BASE,
			     (rb->memptrs_desc.gpuaddr
			      + GSL_RB_MEMPTRS_WPTRPOLL_OFFSET));

	/* setup WPTR delay */
	kgsl_yamato_regwrite(device, REG_CP_RB_WPTR_DELAY, 0 /*0x70000010 */);

	/*setup REG_CP_RB_CNTL */
	kgsl_yamato_regread(device, REG_CP_RB_CNTL, &rb_cntl);
	cp_rb_cntl.val = rb_cntl;
	/* size of ringbuffer */
	cp_rb_cntl.f.rb_bufsz =
		kgsl_ringbuffer_sizelog2quadwords(rb->sizedwords);
	/* quadwords to read before updating mem RPTR */
	cp_rb_cntl.f.rb_blksz = rb->blksizequadwords;
	cp_rb_cntl.f.rb_poll_en = GSL_RB_CNTL_POLL_EN; /* WPTR polling */
	/* mem RPTR writebacks */
	cp_rb_cntl.f.rb_no_update =  GSL_RB_CNTL_NO_UPDATE;

	kgsl_yamato_regwrite(device, REG_CP_RB_CNTL, cp_rb_cntl.val);

	kgsl_yamato_regwrite(device, REG_CP_RB_BASE, rb->buffer_desc.gpuaddr);

	kgsl_yamato_regwrite(device, REG_CP_RB_RPTR_ADDR,
			     rb->memptrs_desc.gpuaddr +
			     GSL_RB_MEMPTRS_RPTR_OFFSET);

	/* explicitly clear all cp interrupts */
	kgsl_yamato_regwrite(device, REG_CP_INT_ACK, 0xFFFFFFFF);

	/* setup scratch/timestamp */
	kgsl_yamato_regwrite(device, REG_SCRATCH_ADDR,
			     device->memstore.gpuaddr +
			     KGSL_DEVICE_MEMSTORE_OFFSET(soptimestamp));

	kgsl_yamato_regwrite(device, REG_SCRATCH_UMSK,
			     GSL_RB_MEMPTRS_SCRATCH_MASK);

	/* load the CP ucode */

	status = kgsl_ringbuffer_load_pm4_ucode(device);
	if (status != 0)
		return status;

	/* load the prefetch parser ucode */
	status = kgsl_ringbuffer_load_pfp_ucode(device);
	if (status != 0)
		return status;

	kgsl_yamato_regwrite(device, REG_CP_QUEUE_THRESHOLDS, 0x000C0804);

	rb->rptr = 0;
	rb->wptr = 0;

	/* clear ME_HALT to start micro engine */
	kgsl_yamato_regwrite(device, REG_CP_ME_CNTL, 0);

	/* ME_INIT */
	cmds = kgsl_ringbuffer_allocspace(rb, 19);
	cmds_gpu = rb->buffer_desc.gpuaddr + sizeof(uint)*(rb->wptr-19);

	GSL_RB_WRITE(cmds, cmds_gpu, PM4_HDR_ME_INIT);
	/* All fields present (bits 9:0) */
	GSL_RB_WRITE(cmds, cmds_gpu, 0x000003ff);
	/* Disable/Enable Real-Time Stream processing (present but ignored) */
	GSL_RB_WRITE(cmds, cmds_gpu, 0x00000000);
	/* Enable (2D <-> 3D) implicit synchronization (present but ignored) */
	GSL_RB_WRITE(cmds, cmds_gpu, 0x00000000);

	GSL_RB_WRITE(cmds, cmds_gpu,
		GSL_HAL_SUBBLOCK_OFFSET(REG_RB_SURFACE_INFO));
	GSL_RB_WRITE(cmds, cmds_gpu,
		GSL_HAL_SUBBLOCK_OFFSET(REG_PA_SC_WINDOW_OFFSET));
	GSL_RB_WRITE(cmds, cmds_gpu,
		GSL_HAL_SUBBLOCK_OFFSET(REG_VGT_MAX_VTX_INDX));
	GSL_RB_WRITE(cmds, cmds_gpu,
		GSL_HAL_SUBBLOCK_OFFSET(REG_SQ_PROGRAM_CNTL));
	GSL_RB_WRITE(cmds, cmds_gpu,
		GSL_HAL_SUBBLOCK_OFFSET(REG_RB_DEPTHCONTROL));
	GSL_RB_WRITE(cmds, cmds_gpu,
		GSL_HAL_SUBBLOCK_OFFSET(REG_PA_SU_POINT_SIZE));
	GSL_RB_WRITE(cmds, cmds_gpu,
		GSL_HAL_SUBBLOCK_OFFSET(REG_PA_SC_LINE_CNTL));
	GSL_RB_WRITE(cmds, cmds_gpu,
		GSL_HAL_SUBBLOCK_OFFSET(REG_PA_SU_POLY_OFFSET_FRONT_SCALE));

	/* Vertex and Pixel Shader Start Addresses in instructions
	* (3 DWORDS per instruction) */
	GSL_RB_WRITE(cmds, cmds_gpu, 0x80000180);
	/* Maximum Contexts */
	GSL_RB_WRITE(cmds, cmds_gpu, 0x00000001);
	/* Write Confirm Interval and The CP will wait the
	* wait_interval * 16 clocks between polling  */
	GSL_RB_WRITE(cmds, cmds_gpu, 0x00000000);

	/* NQ and External Memory Swap */
	GSL_RB_WRITE(cmds, cmds_gpu, 0x00000000);
	/* Protected mode error checking */
	GSL_RB_WRITE(cmds, cmds_gpu, GSL_RB_PROTECTED_MODE_CONTROL);
	/* Disable header dumping and Header dump address */
	GSL_RB_WRITE(cmds, cmds_gpu, 0x00000000);
	/* Header dump size */
	GSL_RB_WRITE(cmds, cmds_gpu, 0x00000000);

	kgsl_ringbuffer_submit(rb);

	/* idle device to validate ME INIT */
	status = kgsl_yamato_idle(device, KGSL_TIMEOUT_DEFAULT);

	kgsl_yamato_regwrite(rb->device, REG_CP_INT_CNTL, GSL_CP_INT_MASK);
	if (status == 0)
		rb->flags |= KGSL_FLAGS_STARTED;

	return status;
}

int kgsl_ringbuffer_stop(struct kgsl_ringbuffer *rb)
{
	if (rb->flags & KGSL_FLAGS_STARTED) {
		kgsl_yamato_regwrite(rb->device, REG_CP_INT_CNTL, 0);

		/* ME_HALT */
		kgsl_yamato_regwrite(rb->device, REG_CP_ME_CNTL, 0x10000000);

		rb->flags &= ~KGSL_FLAGS_STARTED;
	}

	return 0;
}

int kgsl_ringbuffer_init(struct kgsl_device *device)
{
	int status;
	struct kgsl_yamato_device *yamato_device = KGSL_YAMATO_DEVICE(device);
	struct kgsl_ringbuffer *rb = &yamato_device->ringbuffer;

	rb->device = device;
	rb->sizedwords = (2 << kgsl_cfg_rb_sizelog2quadwords);
	rb->blksizequadwords = kgsl_cfg_rb_blksizequadwords;

	/* allocate memory for ringbuffer */
	status = kgsl_sharedmem_alloc_coherent(&rb->buffer_desc,
					       (rb->sizedwords << 2));
	if (status != 0) {
		kgsl_ringbuffer_close(rb);
		return status;
	}

	/* allocate memory for polling and timestamps */
	/* This really can be at 4 byte alignment boundry but for using MMU
	 * we need to make it at page boundary */
	status = kgsl_sharedmem_alloc_coherent(&rb->memptrs_desc,
					       sizeof(struct kgsl_rbmemptrs));
	if (status != 0) {
		kgsl_ringbuffer_close(rb);
		return status;
	}

	/* overlay structure on memptrs memory */
	rb->memptrs = (struct kgsl_rbmemptrs *) rb->memptrs_desc.hostptr;

	return 0;
}

int kgsl_ringbuffer_close(struct kgsl_ringbuffer *rb)
{
	struct kgsl_yamato_device *yamato_device = KGSL_YAMATO_DEVICE(
							rb->device);
	if (rb->buffer_desc.hostptr)
		kgsl_sharedmem_free(&rb->buffer_desc);

	if (rb->memptrs_desc.hostptr)
		kgsl_sharedmem_free(&rb->memptrs_desc);

	if (yamato_device->pfp_fw != NULL)
		kfree(yamato_device->pfp_fw);
	if (yamato_device->pm4_fw != NULL)
		kfree(yamato_device->pm4_fw);
	yamato_device->pfp_fw = NULL;
	yamato_device->pm4_fw = NULL;

	memset(rb, 0, sizeof(struct kgsl_ringbuffer));

	return 0;
}

static uint32_t
kgsl_ringbuffer_addcmds(struct kgsl_ringbuffer *rb,
				unsigned int flags, unsigned int *cmds,
				int sizedwords)
{
	unsigned int *ringcmds;
	unsigned int timestamp;
	unsigned int total_sizedwords = sizedwords + 6;
	unsigned int i;
	unsigned int rcmd_gpu;

	/* reserve space to temporarily turn off protected mode
	*  error checking if needed
	*/
	total_sizedwords += flags & KGSL_CMD_FLAGS_PMODE ? 4 : 0;
	total_sizedwords += !(flags & KGSL_CMD_FLAGS_NO_TS_CMP) ? 7 : 0;
	total_sizedwords += !(flags & KGSL_CMD_FLAGS_NOT_KERNEL_CMD) ? 2 : 0;

	ringcmds = kgsl_ringbuffer_allocspace(rb, total_sizedwords);
	rcmd_gpu = rb->buffer_desc.gpuaddr
		+ sizeof(uint)*(rb->wptr-total_sizedwords);

	if (!(flags & KGSL_CMD_FLAGS_NOT_KERNEL_CMD)) {
		GSL_RB_WRITE(ringcmds, rcmd_gpu, pm4_nop_packet(1));
		GSL_RB_WRITE(ringcmds, rcmd_gpu, KGSL_CMD_IDENTIFIER);
	}
	if (flags & KGSL_CMD_FLAGS_PMODE) {
		/* disable protected mode error checking */
		GSL_RB_WRITE(ringcmds, rcmd_gpu,
			pm4_type3_packet(PM4_SET_PROTECTED_MODE, 1));
		GSL_RB_WRITE(ringcmds, rcmd_gpu, 0);
	}

	for (i = 0; i < sizedwords; i++) {
		GSL_RB_WRITE(ringcmds, rcmd_gpu, *cmds);
		cmds++;
	}

	if (flags & KGSL_CMD_FLAGS_PMODE) {
		/* re-enable protected mode error checking */
		GSL_RB_WRITE(ringcmds, rcmd_gpu,
			pm4_type3_packet(PM4_SET_PROTECTED_MODE, 1));
		GSL_RB_WRITE(ringcmds, rcmd_gpu, 1);
	}

	rb->timestamp++;
	timestamp = rb->timestamp;

	/* start-of-pipeline and end-of-pipeline timestamps */
	GSL_RB_WRITE(ringcmds, rcmd_gpu, pm4_type0_packet(REG_CP_TIMESTAMP, 1));
	GSL_RB_WRITE(ringcmds, rcmd_gpu, rb->timestamp);
	GSL_RB_WRITE(ringcmds, rcmd_gpu, pm4_type3_packet(PM4_EVENT_WRITE, 3));
	GSL_RB_WRITE(ringcmds, rcmd_gpu, CACHE_FLUSH_TS);
	GSL_RB_WRITE(ringcmds, rcmd_gpu,
		     (rb->device->memstore.gpuaddr +
		      KGSL_DEVICE_MEMSTORE_OFFSET(eoptimestamp)));
	GSL_RB_WRITE(ringcmds, rcmd_gpu, rb->timestamp);

	if (!(flags & KGSL_CMD_FLAGS_NO_TS_CMP)) {
		/* Conditional execution based on memory values */
		GSL_RB_WRITE(ringcmds, rcmd_gpu,
			pm4_type3_packet(PM4_COND_EXEC, 4));
		GSL_RB_WRITE(ringcmds, rcmd_gpu, (rb->device->memstore.gpuaddr +
			KGSL_DEVICE_MEMSTORE_OFFSET(ts_cmp_enable)) >> 2);
		GSL_RB_WRITE(ringcmds, rcmd_gpu, (rb->device->memstore.gpuaddr +
			KGSL_DEVICE_MEMSTORE_OFFSET(ref_wait_ts)) >> 2);
		GSL_RB_WRITE(ringcmds, rcmd_gpu, rb->timestamp);
		/* # of conditional command DWORDs */
		GSL_RB_WRITE(ringcmds, rcmd_gpu, 2);
		GSL_RB_WRITE(ringcmds, rcmd_gpu,
			pm4_type3_packet(PM4_INTERRUPT, 1));
		GSL_RB_WRITE(ringcmds, rcmd_gpu, CP_INT_CNTL__RB_INT_MASK);
	}

	kgsl_ringbuffer_submit(rb);

	/* return timestamp of issued coREG_ands */
	return timestamp;
}

void
kgsl_ringbuffer_issuecmds(struct kgsl_device *device,
						unsigned int flags,
						unsigned int *cmds,
						int sizedwords)
{
	struct kgsl_yamato_device *yamato_device = KGSL_YAMATO_DEVICE(device);
	struct kgsl_ringbuffer *rb = &yamato_device->ringbuffer;

	if (device->state & KGSL_STATE_HUNG)
		return;
	kgsl_ringbuffer_addcmds(rb, flags, cmds, sizedwords);
}

int
kgsl_ringbuffer_issueibcmds(struct kgsl_device_private *dev_priv,
				struct kgsl_context *context,
				struct kgsl_ibdesc *ibdesc,
				unsigned int numibs,
				uint32_t *timestamp,
				unsigned int flags)
{
	struct kgsl_device *device = dev_priv->device;
	struct kgsl_yamato_device *yamato_device = KGSL_YAMATO_DEVICE(device);
	unsigned int *link;
	unsigned int *cmds;
	unsigned int i;
	struct kgsl_yamato_context *drawctxt = context->devctxt;

	if (device->state & KGSL_STATE_HUNG)
		return -EBUSY;
	if (!(yamato_device->ringbuffer.flags & KGSL_FLAGS_STARTED) ||
	      context == NULL)
		return -EINVAL;

	BUG_ON(ibdesc == 0);
	BUG_ON(numibs == 0);

	if (drawctxt->flags & CTXT_FLAGS_GPU_HANG) {
		KGSL_CTXT_WARN(device, "Context %p caused a gpu hang.."
			" will not accept commands for this context\n",
			drawctxt);
		return -EDEADLK;
	}
	link = kzalloc(sizeof(unsigned int) * numibs * 3, GFP_KERNEL);
	cmds = link;
	if (!link) {
		KGSL_MEM_ERR(device, "Failed to allocate memory for for command"
			" submission, size %x\n", numibs * 3);
		return -ENOMEM;
	}
	for (i = 0; i < numibs; i++) {
		(void)kgsl_cffdump_parse_ibs(dev_priv, NULL,
			ibdesc[i].gpuaddr, ibdesc[i].sizedwords, false);

		*cmds++ = PM4_HDR_INDIRECT_BUFFER_PFD;
		*cmds++ = ibdesc[i].gpuaddr;
		*cmds++ = ibdesc[i].sizedwords;
	}

	kgsl_setstate(device,
		      kgsl_pt_get_flags(device->mmu.hwpagetable,
					device->id));

	kgsl_drawctxt_switch(yamato_device, drawctxt, flags);

	*timestamp = kgsl_ringbuffer_addcmds(&yamato_device->ringbuffer,
					KGSL_CMD_FLAGS_NOT_KERNEL_CMD,
					&link[0], (cmds - link));

	KGSL_CMD_INFO(device, "ctxt %d g %08x numibs %d ts %d\n",
		context->id, (unsigned int)ibdesc, numibs, *timestamp);

	kfree(link);

#ifdef CONFIG_MSM_KGSL_CFF_DUMP
	/*
	 * insert wait for idle after every IB1
	 * this is conservative but works reliably and is ok
	 * even for performance simulations
	 */
	kgsl_yamato_idle(device, KGSL_TIMEOUT_DEFAULT);
#endif

	return 0;
}

int kgsl_ringbuffer_extract(struct kgsl_ringbuffer *rb,
				unsigned int *temp_rb_buffer,
				int *rb_size)
{
	struct kgsl_device *device = rb->device;
	unsigned int rb_rptr;
	unsigned int retired_timestamp;
	unsigned int temp_idx = 0;
	unsigned int value;
	unsigned int val1;
	unsigned int val2;
	unsigned int val3;
	unsigned int copy_rb_contents = 0;
	unsigned int cur_context;
	unsigned int j;

	GSL_RB_GET_READPTR(rb, &rb->rptr);

	retired_timestamp = device->ftbl.device_cmdstream_readtimestamp(
				device, KGSL_TIMESTAMP_RETIRED);
	rmb();
	KGSL_DRV_ERR(device, "GPU successfully executed till ts: %x\n",
			retired_timestamp);
	/*
	 * We need to go back in history by 4 dwords from the current location
	 * of read pointer as 4 dwords are read to match the end of a command.
	 * Also, take care of wrap around when moving back
	 */
	if (rb->rptr >= 4)
		rb_rptr = (rb->rptr - 4) * sizeof(unsigned int);
	else
		rb_rptr = rb->buffer_desc.size -
			((4 - rb->rptr) * sizeof(unsigned int));
	/* Read the rb contents going backwards to locate end of last
	 * sucessfully executed command */
	while ((rb_rptr / sizeof(unsigned int)) != rb->wptr) {
		kgsl_sharedmem_readl(&rb->buffer_desc, &value, rb_rptr);
		rmb();
		if (value == retired_timestamp) {
			rb_rptr = adreno_ringbuffer_inc_wrapped(rb_rptr,
							rb->buffer_desc.size);
			kgsl_sharedmem_readl(&rb->buffer_desc, &val1, rb_rptr);
			rb_rptr = adreno_ringbuffer_inc_wrapped(rb_rptr,
							rb->buffer_desc.size);
			kgsl_sharedmem_readl(&rb->buffer_desc, &val2, rb_rptr);
			rb_rptr = adreno_ringbuffer_inc_wrapped(rb_rptr,
							rb->buffer_desc.size);
			kgsl_sharedmem_readl(&rb->buffer_desc, &val3, rb_rptr);
			rmb();
			/* match the pattern found at the end of a command */
			if ((val1 == 2 &&
				val2 == pm4_type3_packet(PM4_INTERRUPT, 1)
				&& val3 == CP_INT_CNTL__RB_INT_MASK) ||
				(val1 == pm4_type3_packet(PM4_EVENT_WRITE, 3)
				&& val2 == CACHE_FLUSH_TS &&
				val3 == (rb->device->memstore.gpuaddr +
				KGSL_DEVICE_MEMSTORE_OFFSET(eoptimestamp)))) {
				rb_rptr = adreno_ringbuffer_inc_wrapped(rb_rptr,
							rb->buffer_desc.size);
				KGSL_DRV_ERR(device,
					"Found end of last executed "
					"command at offset: %x\n",
					rb_rptr / sizeof(unsigned int));
				break;
			} else {
				if (rb_rptr < (3 * sizeof(unsigned int)))
					rb_rptr = rb->buffer_desc.size -
						(3 * sizeof(unsigned int))
							+ rb_rptr;
				else
					rb_rptr -= (3 * sizeof(unsigned int));
			}
		}

		if (rb_rptr == 0)
			rb_rptr = rb->buffer_desc.size - sizeof(unsigned int);
		else
			rb_rptr -= sizeof(unsigned int);
	}

	if ((rb_rptr / sizeof(unsigned int)) == rb->wptr) {
		KGSL_DRV_ERR(device,
			"GPU recovery from hang not possible because last"
			" successful timestamp is overwritten\n");
		return -EINVAL;
	}
	/* rb_rptr is now pointing to the first dword of the command following
	 * the last sucessfully executed command sequence. Assumption is that
	 * GPU is hung in the command sequence pointed by rb_rptr */
	/* make sure the GPU is not hung in a command submitted by kgsl
	 * itself */
	kgsl_sharedmem_readl(&rb->buffer_desc, &val1, rb_rptr);
	kgsl_sharedmem_readl(&rb->buffer_desc, &val2,
				adreno_ringbuffer_inc_wrapped(rb_rptr,
							rb->buffer_desc.size));
	rmb();
	if (val1 == pm4_nop_packet(1) && val2 == KGSL_CMD_IDENTIFIER) {
		KGSL_DRV_ERR(device,
			"GPU recovery from hang not possible because "
			"of hang in kgsl command\n");
		return -EINVAL;
	}

	/* current_context is the context that is presently active in the
	 * GPU, i.e the context in which the hang is caused */
	kgsl_sharedmem_readl(&device->memstore, &cur_context,
		KGSL_DEVICE_MEMSTORE_OFFSET(current_context));
	while ((rb_rptr / sizeof(unsigned int)) != rb->wptr) {
		kgsl_sharedmem_readl(&rb->buffer_desc, &value, rb_rptr);
		rb_rptr = adreno_ringbuffer_inc_wrapped(rb_rptr,
						rb->buffer_desc.size);
		rmb();
		/* check for context switch indicator */
		if (value == KGSL_CONTEXT_TO_MEM_IDENTIFIER) {
			kgsl_sharedmem_readl(&rb->buffer_desc, &value, rb_rptr);
			rb_rptr = adreno_ringbuffer_inc_wrapped(rb_rptr,
							rb->buffer_desc.size);
			rmb();
			BUG_ON(value != pm4_type3_packet(PM4_MEM_WRITE, 2));
			kgsl_sharedmem_readl(&rb->buffer_desc, &val1, rb_rptr);
			rb_rptr = adreno_ringbuffer_inc_wrapped(rb_rptr,
							rb->buffer_desc.size);
			rmb();
			BUG_ON(val1 != (device->memstore.gpuaddr +
				KGSL_DEVICE_MEMSTORE_OFFSET(current_context)));
			kgsl_sharedmem_readl(&rb->buffer_desc, &value, rb_rptr);
			rb_rptr = adreno_ringbuffer_inc_wrapped(rb_rptr,
							rb->buffer_desc.size);
			rmb();
			BUG_ON((copy_rb_contents == 0) &&
				(value == cur_context));
			/* if context switches to a context that did not cause
			 * hang then start saving the rb contents as those
			 * commands can be executed */
			if (value != cur_context) {
				copy_rb_contents = 1;
				temp_rb_buffer[temp_idx++] = pm4_nop_packet(1);
				temp_rb_buffer[temp_idx++] =
						KGSL_CMD_IDENTIFIER;
				temp_rb_buffer[temp_idx++] = pm4_nop_packet(1);
				temp_rb_buffer[temp_idx++] =
						KGSL_CONTEXT_TO_MEM_IDENTIFIER;
				temp_rb_buffer[temp_idx++] =
					pm4_type3_packet(PM4_MEM_WRITE, 2);
				temp_rb_buffer[temp_idx++] = val1;
				temp_rb_buffer[temp_idx++] = value;
			} else {
				/* if temp_idx is not 0 then we do not need to
				 * copy extra dwords indicating a kernel cmd */
				if (temp_idx)
					temp_idx -= 3;
				copy_rb_contents = 0;
			}
		} else if (copy_rb_contents)
			temp_rb_buffer[temp_idx++] = value;
	}

	*rb_size = temp_idx;
	KGSL_DRV_ERR(device, "Extracted rb contents, size: %x\n", *rb_size);
	for (temp_idx = 0; temp_idx < *rb_size;) {
		char str[80];
		int idx = 0;
		if ((temp_idx + 8) <= *rb_size)
			j = 8;
		else
			j = *rb_size - temp_idx;
		for (; j != 0; j--)
			idx += scnprintf(str + idx, 80 - idx,
				"%8.8X ", temp_rb_buffer[temp_idx++]);
		printk(KERN_ALERT "%s", str);
	}
	return 0;
}

void
kgsl_ringbuffer_restore(struct kgsl_ringbuffer *rb, unsigned int *rb_buff,
			int num_rb_contents)
{
	int i;
	unsigned int *ringcmds;
	unsigned int rcmd_gpu;

	if (!num_rb_contents)
		return;

	if (num_rb_contents > (rb->buffer_desc.size - rb->wptr)) {
		kgsl_yamato_regwrite(rb->device, REG_CP_RB_RPTR, 0);
		rb->rptr = 0;
		BUG_ON(num_rb_contents > rb->buffer_desc.size);
	}
	ringcmds = (unsigned int *)rb->buffer_desc.hostptr + rb->wptr;
	rcmd_gpu = rb->buffer_desc.gpuaddr + sizeof(unsigned int) * rb->wptr;
	for (i = 0; i < num_rb_contents; i++)
		GSL_RB_WRITE(ringcmds, rcmd_gpu, rb_buff[i]);
	rb->wptr += num_rb_contents;
	kgsl_ringbuffer_submit(rb);
}
