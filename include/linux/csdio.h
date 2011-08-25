/*
Copyright (c) 2010, Code Aurora Forum. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.
    * Neither the name of Code Aurora Forum, Inc. nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef CSDIO_H
#define CSDIO_H

#include <linux/ioctl.h>

#define CSDIO_IOC_MAGIC  'm'

#define CSDIO_IOC_ENABLE_HIGHSPEED_MODE      _IO(CSDIO_IOC_MAGIC, 0)
#define CSDIO_IOC_SET_DATA_TRANSFER_CLOCKS   _IO(CSDIO_IOC_MAGIC, 1)
#define CSDIO_IOC_SET_OP_CODE                _IO(CSDIO_IOC_MAGIC, 2)
#define CSDIO_IOC_FUNCTION_SET_BLOCK_SIZE    _IO(CSDIO_IOC_MAGIC, 3)
#define CSDIO_IOC_SET_BLOCK_MODE             _IO(CSDIO_IOC_MAGIC, 4)
#define CSDIO_IOC_CONNECT_ISR                _IO(CSDIO_IOC_MAGIC, 5)
#define CSDIO_IOC_DISCONNECT_ISR             _IO(CSDIO_IOC_MAGIC, 6)
#define CSDIO_IOC_CMD52                      _IO(CSDIO_IOC_MAGIC, 7)
#define CSDIO_IOC_CMD53                      _IO(CSDIO_IOC_MAGIC, 8)
#define CSDIO_IOC_ENABLE_ISR                 _IO(CSDIO_IOC_MAGIC, 9)
#define CSDIO_IOC_DISABLE_ISR                _IO(CSDIO_IOC_MAGIC, 10)
#define CSDIO_IOC_SET_VDD                    _IO(CSDIO_IOC_MAGIC, 11)
#define CSDIO_IOC_GET_VDD                    _IO(CSDIO_IOC_MAGIC, 12)

#define CSDIO_IOC_MAXNR   12

struct csdio_cmd53_ctrl_t {
	uint32_t    m_block_mode;   /* data tran. byte(0)/block(1) mode */
	uint32_t    m_op_code;      /* address auto increment flag */
	uint32_t    m_address;
} __attribute__ ((packed));

struct csdio_cmd52_ctrl_t {
	uint32_t    m_write;
	uint32_t    m_address;
	uint32_t    m_data;
	uint32_t    m_ret;
} __attribute__ ((packed));

#endif
