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
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/msm_kgsl.h>
#include "kgsl_g12_cmdwindow.h"
#include "kgsl_g12_cmdstream.h"
#include "kgsl_g12_drawctxt.h"
#include "kgsl_g12_vgv3types.h"
#include "kgsl_cmdstream.h"

#include "kgsl.h"
#include "kgsl_device.h"
#include "kgsl_g12.h"
#include "kgsl_g12_drawctxt.h"
#include "kgsl_log.h"

#include "g12_reg.h"

static inline unsigned int rb_offset(unsigned int index)
{
	return index*sizeof(unsigned int)*(KGSL_G12_PACKET_SIZE);
}

int kgsl_g12_cmdstream_init(struct kgsl_device *device)
{
	struct kgsl_g12_device *g12_device = KGSL_G12_DEVICE(device);
	memset(&g12_device->ringbuffer, 0, sizeof(struct kgsl_g12_ringbuffer));
	g12_device->ringbuffer.prevctx = KGSL_G12_INVALID_CONTEXT;
	return kgsl_sharedmem_alloc_coherent(&g12_device->ringbuffer.cmdbufdesc,
					     KGSL_G12_RB_SIZE);
}

static void addmarker(struct kgsl_g12_ringbuffer *rb, unsigned int index)
{
	char *ptr = (char *)(rb->cmdbufdesc.hostptr);
	unsigned int *p = (unsigned int *)(ptr + rb_offset(index));

	*p++ = KGSL_G12_STREAM_PACKET;
	*p++ = (KGSL_G12_MARKER_CMD | 5);
	*p++ = ADDR_VGV3_LAST << 24;
	*p++ = ADDR_VGV3_LAST << 24;
	*p++ = ADDR_VGV3_LAST << 24;
	*p++ = KGSL_G12_STREAM_PACKET;
	*p++ = 5;
	*p++ = ADDR_VGV3_LAST << 24;
	*p++ = ADDR_VGV3_LAST << 24;
	*p++ = ADDR_VGV3_LAST << 24;
}

static void addcmd(struct kgsl_g12_ringbuffer *rb, unsigned int index,
			unsigned int cmd, unsigned int nextcnt)
{
	char * ptr = (char *)(rb->cmdbufdesc.hostptr);
	unsigned int *p = (unsigned int *)(ptr + (rb_offset(index)
			   + (KGSL_G12_MARKER_SIZE * sizeof(unsigned int))));

	*p++ = KGSL_G12_STREAM_PACKET_CALL;
	*p++ = cmd;
	*p++ = KGSL_G12_CALL_CMD | nextcnt;
	*p++ = ADDR_VGV3_LAST << 24;
	*p++ = ADDR_VGV3_LAST << 24;
}

int kgsl_g12_cmdstream_start(struct kgsl_device *device)
{
	struct kgsl_g12_device *g12_device = KGSL_G12_DEVICE(device);
	int result;
	unsigned int cmd = VGV3_NEXTCMD_JUMP << VGV3_NEXTCMD_NEXTCMD_FSHIFT;

	g12_device->timestamp = 0;
	g12_device->current_timestamp = 0;

	addmarker(&g12_device->ringbuffer, 0);

	result = kgsl_g12_cmdwindow_write(device, KGSL_CMDWINDOW_2D,
			ADDR_VGV3_MODE, 4);
	if (result != 0)
		return result;

	result = kgsl_g12_cmdwindow_write(device, KGSL_CMDWINDOW_2D,
			ADDR_VGV3_NEXTADDR,
			g12_device->ringbuffer.cmdbufdesc.gpuaddr);
	if (result != 0)
		return result;

	result = kgsl_g12_cmdwindow_write(device, KGSL_CMDWINDOW_2D,
			ADDR_VGV3_NEXTCMD, cmd | 5);
	if (result != 0)
		return result;

	result = kgsl_g12_cmdwindow_write(device, KGSL_CMDWINDOW_2D,
			ADDR_VGV3_WRITEADDR, device->memstore.gpuaddr);

	if (result != 0)
		return result;

	cmd = (int)(((1) & VGV3_CONTROL_MARKADD_FMASK)
			<< VGV3_CONTROL_MARKADD_FSHIFT);

	result = kgsl_g12_cmdwindow_write(device, KGSL_CMDWINDOW_2D,
			ADDR_VGV3_CONTROL, cmd);

	if (result != 0)
		return result;

	result = kgsl_g12_cmdwindow_write(device, KGSL_CMDWINDOW_2D,
			ADDR_VGV3_CONTROL, 0);
	if (result != 0)
		return result;

	return result;
}

void kgsl_g12_cmdstream_close(struct kgsl_device *device)
{
	struct kgsl_g12_device *g12_device = KGSL_G12_DEVICE(device);
	kgsl_sharedmem_free(&g12_device->ringbuffer.cmdbufdesc);
	memset(&g12_device->ringbuffer, 0, sizeof(struct kgsl_g12_ringbuffer));
	kgsl_cmdstream_close(device);
}

static int room_in_rb(struct kgsl_g12_device *device)
{
	int ts_diff;

	ts_diff = device->current_timestamp - device->timestamp;

	return ts_diff < KGSL_G12_PACKET_COUNT;
}

int
kgsl_g12_cmdstream_issueibcmds(struct kgsl_device_private *dev_priv,
			struct kgsl_context *context,
			struct kgsl_ibdesc *ibdesc,
			unsigned int numibs,
			uint32_t *timestamp,
			unsigned int ctrl)
{
	unsigned int result = 0;
	unsigned int ofs        = PACKETSIZE_STATESTREAM * sizeof(unsigned int);
	unsigned int cnt        = 5;
	unsigned int nextaddr   = 0;
	unsigned int index	= 0;
	unsigned int nextindex;
	unsigned int nextcnt    = KGSL_G12_STREAM_END_CMD | 5;
	struct kgsl_memdesc tmp = {0};
	unsigned int cmd;
	struct kgsl_device *device = dev_priv->device;
	struct kgsl_pagetable *pagetable = dev_priv->process_priv->pagetable;
	struct kgsl_g12_device *g12_device = KGSL_G12_DEVICE(device);
	unsigned int sizedwords;

	if (device->state & KGSL_STATE_HUNG) {
		return -EINVAL;
		goto error;
	}
	if (numibs != 1) {
		KGSL_DRV_ERR("Invalid number of ib's passed for z180,"
				" numibs: %d\n", numibs);
		result = -EINVAL;
		goto error;
	}
	cmd = ibdesc[0].gpuaddr;
	sizedwords = ibdesc[0].sizedwords;

	tmp.hostptr = (void *)*timestamp;

	KGSL_CMD_INFO("ctxt %d ibaddr 0x%08x sizedwords %d",
		      context->id, cmd, sizedwords);
	/* context switch */
	if ((context->id != (int)g12_device->ringbuffer.prevctx) ||
	    (ctrl & KGSL_CONTEXT_CTX_SWITCH)) {
		KGSL_CMD_INFO("context switch %d -> %d",
				context->id, g12_device->ringbuffer.prevctx);
		kgsl_mmu_setstate(device, pagetable);
		cnt = PACKETSIZE_STATESTREAM;
		ofs = 0;
	}
	kgsl_g12_setstate(device, kgsl_pt_get_flags(device->mmu.hwpagetable,
						    device->id));

	result = wait_event_interruptible_timeout(g12_device->wait_timestamp_wq,
				  room_in_rb(g12_device),
				  msecs_to_jiffies(KGSL_TIMEOUT_DEFAULT));
	if (result < 0) {
		KGSL_CMD_ERR("failed waiting for ringbuffer. result %d",
			     result);
		goto error;
	}
	result = 0;

	index = g12_device->current_timestamp % KGSL_G12_PACKET_COUNT;
	g12_device->current_timestamp++;
	nextindex = g12_device->current_timestamp % KGSL_G12_PACKET_COUNT;
	*timestamp = g12_device->current_timestamp;

	g12_device->ringbuffer.prevctx = context->id;

	addcmd(&g12_device->ringbuffer, index, cmd + ofs, cnt);

	/* Make sure the next ringbuffer entry has a marker */
	addmarker(&g12_device->ringbuffer, nextindex);

	nextaddr = g12_device->ringbuffer.cmdbufdesc.gpuaddr
		+ rb_offset(nextindex);

	tmp.hostptr = (void *)(tmp.hostptr +
			(sizedwords * sizeof(unsigned int)));
	tmp.size = 12;

	kgsl_sharedmem_writel(&tmp, 4, nextaddr);
	kgsl_sharedmem_writel(&tmp, 8, nextcnt);

	/*synchronize memory before trigging the hardware to execute more
	 * commands
	 */
	dsb();
	mb();

	cmd = (int)(((2) & VGV3_CONTROL_MARKADD_FMASK)
		<< VGV3_CONTROL_MARKADD_FSHIFT);

	kgsl_g12_cmdwindow_write(device,
				KGSL_CMDWINDOW_2D, ADDR_VGV3_CONTROL, cmd);
	kgsl_g12_cmdwindow_write(device,
				KGSL_CMDWINDOW_2D, ADDR_VGV3_CONTROL, 0);
error:
	return result;
}

unsigned int kgsl_g12_cmdstream_readtimestamp(struct kgsl_device *device,
			     enum kgsl_timestamp_type type)
{
	struct kgsl_g12_device *g12_device = KGSL_G12_DEVICE(device);
	/* get current EOP timestamp */
	return g12_device->timestamp;
}

