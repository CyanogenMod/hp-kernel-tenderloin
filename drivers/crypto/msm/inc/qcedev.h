/* Qualcomm Crypto Engine driver QCEDEV API
 *
 * Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
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
 */
#ifndef __QCEDEV__H
#define __QCEDEV__H

#include <linux/types.h>
#include <linux/ioctl.h>


#define QCEDEV_MAX_BEARER	31
#define QCEDEV_MAX_KEY_SIZE	32   /* 256 bits of keys. */
#define QCEDEV_MAX_IV_SIZE	32

#define QCEDEV_MAX_BUFFERS      16
#define QCEDEV_MAX_SHA_DIGEST	32

#define QCEDEV_USE_PMEM		1
#define QCEDEV_NO_PMEM		0
/**
*qcedev_oper_enum: Operation types
* @QCEDEV_OPER_ENC:		Encrypt
* @QCEDEV_OPER_DEC:		Decrypt
* @QCEDEV_OPER_ENC_NO_KEY:	Encrypt. Do not need key to be specified by
*				user. Key already set by an external processor.
* @QCEDEV_OPER_DEC_NO_KEY:	Decrypt. Do not need the key to be specified by
*				user. Key already set by an external processor.
*/
enum qcedev_oper_enum {
  QCEDEV_OPER_DEC		= 0,
  QCEDEV_OPER_ENC		= 1,
  QCEDEV_OPER_DEC_NO_KEY	= 2,
  QCEDEV_OPER_ENC_NO_KEY	= 3,
  QCEDEV_OPER_LAST
};

/**
*qcedev_oper_enum: Cipher algorithm types
* @QCEDEV_ALG_DES:		DES
* @QCEDEV_ALG_3DES:		3DES
* @QCEDEV_ALG_AES:		AES
*/
enum qcedev_cipher_alg_enum {
	QCEDEV_ALG_DES		= 0,
	QCEDEV_ALG_3DES		= 1,
	QCEDEV_ALG_AES		= 2,
	QCEDEV_ALG_LAST
};

/**
*qcedev_cipher_mode_enum : AES mode
* @QCEDEV_AES_MODE_CBC:		CBC
* @QCEDEV_AES_MODE_ECB:		ECB
* @QCEDEV_AES_MODE_CTR:		CTR
*/
enum qcedev_cipher_mode_enum {
	QCEDEV_AES_MODE_CBC	= 0,
	QCEDEV_AES_MODE_ECB	= 1,
	QCEDEV_AES_MODE_CTR	= 2,
	QCEDEV_AES_MODE_LAST
};

/**
*enum qcedev_sha_alg_enum : Secure Hashing Algorithm
* @QCEDEV_ALG_SHA1:		Digest returned 20 bytes (160 bits)
* @QCEDEV_ALG_SHA256:		Digest returned 32 bytes (256 bit)
*/
enum qcedev_sha_alg_enum {
	QCEDEV_ALG_SHA1		= 0,
	QCEDEV_ALG_SHA256	= 1,
	QCEDEV_ALG_SHA_ALG_LAST
};

/**
* struct buf_info - Buffer information
* @offset:			Offset from the base address of the buffer
*				(Used when buffer is allocated using PMEM)
* @vaddr:			Virtual buffer address pointer
* @len:				Size of the buffer
*/
struct	buf_info {
	union{
		uint32_t	offset;
		uint8_t		*vaddr;
	};
	uint32_t	len;
};

/**
* struct qcedev_vbuf_info - Source and destination Buffer information
* @src:				Array of buf_info for input/source
* @dst:				Array of buf_info for output/destination
*/
struct	qcedev_vbuf_info {
	struct buf_info	src[QCEDEV_MAX_BUFFERS];
	struct buf_info	dst[QCEDEV_MAX_BUFFERS];
};

struct	qcedev_sha_ctxt{
	uint32_t		auth_data[2];
	uint8_t			digest[QCEDEV_MAX_SHA_DIGEST];
	uint32_t		diglen;
	uint8_t			trailing_buf[64];
	uint32_t		trailing_buf_len;
	uint8_t			first_blk;
	uint8_t			last_blk;
};

/**
* struct qcedev_pmem_info - Stores PMEM buffer information
* @fd_src:			Handle to /dev/adsp_pmem used to allocate
*				memory for input/src buffer
* @src:				Array of buf_info for input/source
* @fd_dst:			Handle to /dev/adsp_pmem used to allocate
*				memory for output/dst buffer
* @dst:				Array of buf_info for output/destination
* @pmem_src_offset:		The offset from input/src buffer
*				(allocated by PMEM)
*/
struct	qcedev_pmem_info{
	int		fd_src;
	struct buf_info	src[QCEDEV_MAX_BUFFERS];
	int		fd_dst;
	struct buf_info	dst[QCEDEV_MAX_BUFFERS];
};

/**
* struct qcedev_cipher_op_req - Holds the ciphering request information
* @use_pmem (IN):	Flag to indicate if buffer source is PMEM
*			QCEDEV_USE_PMEM/QCEDEV_NO_PMEM
* @pmem (IN):		Stores PMEM buffer information.
*			Refer struct qcedev_pmem_info
* @vbuf (IN/OUT):	Stores Source and destination Buffer information
*			Refer to struct qcedev_vbuf_info
* @data_len (IN):	Total Length of input/src and output/dst in bytes
* @in_place_op (IN):	Indicates whether the operation is inplace where
*			source == destination
*			When using PMEM allocated memory, must set this to 1
* @enckey (IN):		128 bits of confidentiality key
*			enckey[0] bit 127-120, enckey[1] bit 119-112,..
*			enckey[15] bit 7-0
* @encklen (IN):	Length of the encryption key(set to 128  bits/16
*			bytes in the driver)
* @iv (IN/OUT):		Initialisation vector data
*			This is updated by the driver, incremented by
*			number of blocks encrypted/decrypted.
* @ivlen (IN):		Length of the IV
* @byteoffset (IN):	Offset in the Cipher BLOCK (applicable and to be set
*			for AES-128 CTR mode only)
* @alg (IN):		Type of ciphering algorithm: AES/DES/3DES
* @mode (IN):		Mode use when using AES algorithm: ECB/CBC/CTR
*			Apllicabel when using AES algorithm only
* @op (IN):		Type of operation: QCEDEV_OPER_DEC/QCEDEV_OPER_ENC or
*			QCEDEV_OPER_ENC_NO_KEY/QCEDEV_OPER_DEC_NO_KEY
*
*If use_pmem is set to 0, the driver assumes that memory was not allocated
* via PMEM, and kernel will need to allocate memory and copy data from user
* space buffer (data_src/dta_dst) and process accordingly and copy data back
* to the user space buffer
*
* If use_pmem is set to 1, the driver assumes that memory was allocated via
* PMEM.
* The kernel driver will use the fd_src to determine the kernel virtual address
* base that maps to the user space virtual address base for the  buffer
* allocated in user space.
* The final input/src and output/dst buffer pointer will be determined
* by adding the offsets to the kernel virtual addr.
*
* If use of hardware key is supported in the target, user can configure the
* key paramters (encklen, enckey) to use the hardware key.
* In order to use the hardware key, set encklen to 0 and set the enckey
* data array to 0.
*/
struct	qcedev_cipher_op_req {
	uint8_t				use_pmem;
	union{
		struct qcedev_pmem_info	pmem;
		struct qcedev_vbuf_info	vbuf;
	};
	uint32_t			entries;
	uint32_t			data_len;
	uint8_t				in_place_op;
	uint8_t				enckey[QCEDEV_MAX_KEY_SIZE];
	uint32_t			encklen;
	uint8_t				iv[QCEDEV_MAX_IV_SIZE];
	uint32_t			ivlen;
	uint32_t			byteoffset;
	enum qcedev_cipher_alg_enum	alg;
	enum qcedev_cipher_mode_enum	mode;
	enum qcedev_oper_enum		op;
};

/**
* struct qcedev_sha_op_req - Holds the hashing request information
* @data (IN):			Array of pointers to the data to be hashed
* @entries (IN):		Number of buf_info entries in the data array
* @data_len (IN):		Length of data to be hashed
* @digest (IN/OUT):		Returns the hashed data information
* @diglen (OUT):		Size of the hashed/digest data
* @alg (IN):			Secure Hash algorithm
* @ctxt (Reserved):		RESERVED: User should not modify this data.
*/
struct	qcedev_sha_op_req {
	struct buf_info			data[QCEDEV_MAX_BUFFERS];
	uint32_t			entries;
	uint32_t			data_len;
	uint8_t				digest[QCEDEV_MAX_SHA_DIGEST];
	uint32_t			diglen;
	enum qcedev_sha_alg_enum	alg;
	struct qcedev_sha_ctxt		ctxt;
};


#define QCEDEV_IOC_MAGIC	0x87

#define QCEDEV_IOCTL_ENC_REQ		\
	_IOWR(QCEDEV_IOC_MAGIC, 1, struct qcedev_cipher_op_req)
#define QCEDEV_IOCTL_DEC_REQ		\
	_IOWR(QCEDEV_IOC_MAGIC, 2, struct qcedev_cipher_op_req)
#define QCEDEV_IOCTL_SHA_INIT_REQ	\
	_IOWR(QCEDEV_IOC_MAGIC, 3, struct qcedev_sha_op_req)
#define QCEDEV_IOCTL_SHA_UPDATE_REQ	\
	_IOWR(QCEDEV_IOC_MAGIC, 4, struct qcedev_sha_op_req)
#define QCEDEV_IOCTL_SHA_FINAL_REQ	\
	_IOWR(QCEDEV_IOC_MAGIC, 5, struct qcedev_sha_op_req)
#define QCEDEV_IOCTL_GET_SHA_REQ	\
	_IOWR(QCEDEV_IOC_MAGIC, 6, struct qcedev_sha_op_req)
#define QCEDEV_IOCTL_LOCK_CE	\
	_IO(QCEDEV_IOC_MAGIC, 7)
#define QCEDEV_IOCTL_UNLOCK_CE	\
	_IO(QCEDEV_IOC_MAGIC, 8)
#endif /* _QCEDEV__H */
