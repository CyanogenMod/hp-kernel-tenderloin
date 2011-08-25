/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef __DAI_H__
#define __DAI_H__

struct dai_dma_params {
	u8 *buffer;
	uint32_t src_start;
	uint32_t bus_id;
	int buffer_size;
	int period_size;
	int channels;
};

enum {
	DAI_SPKR = 0,
	DAI_MIC,
	DAI_MI2S,
	DAI_SEC_SPKR,
	DAI_SEC_MIC,
};

/* Function Prototypes */
int dai_open(uint32_t dma_ch);
void dai_close(uint32_t dma_ch);
int dai_start(uint32_t dma_ch);
int dai_stop(uint32_t dma_ch);
int dai_set_params(uint32_t dma_ch, struct dai_dma_params *params);
uint32_t dai_get_dma_pos(uint32_t dma_ch);
void register_dma_irq_handler(int dma_ch,
		irqreturn_t (*callback) (int intrSrc, void *private_data),
		void *private_data);
void unregister_dma_irq_handler(int dma_ch);
void dai_set_master_mode(uint32_t dma_ch, int mode);

#endif
