/* Qualcomm Crypto Engine driver API
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


#ifndef __CRYPTO_MSM_QCE_H
#define __CRYPTO_MSM_QCE_H

#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/crypto.h>

#include <crypto/algapi.h>
#include <crypto/aes.h>
#include <crypto/des.h>
#include <crypto/sha.h>
#include <crypto/aead.h>
#include <crypto/authenc.h>
#include <crypto/scatterwalk.h>

/* SHA digest size  in bytes */
#define SHA256_DIGESTSIZE		32
#define SHA1_DIGESTSIZE			20

/* key size in bytes */
#define HMAC_KEY_SIZE			(SHA1_DIGESTSIZE)    /* hmac-sha1 */
#define DES_KEY_SIZE			8
#define TRIPLE_DES_KEY_SIZE		24
#define AES128_KEY_SIZE			16
#define AES192_KEY_SIZE			24
#define AES256_KEY_SIZE			32
#define MAX_CIPHER_KEY_SIZE		AES256_KEY_SIZE

/* iv length in bytes */
#define AES_IV_LENGTH			16
#define DES_IV_LENGTH                   8
#define MAX_IV_LENGTH			AES_IV_LENGTH

/* Maximum number of bytes per transfer */
#define QCE_MAX_OPER_DATA		0x8000

typedef void (*qce_comp_func_ptr_t)(void *areq,
		unsigned char *icv, unsigned char *iv, int ret);

enum qce_cipher_alg_enum {
	CIPHER_ALG_DES = 0,
	CIPHER_ALG_3DES = 1,
	CIPHER_ALG_AES = 2,
	CIPHER_ALG_LAST
};

enum qce_hash_alg_enum {
	QCE_HASH_SHA1   = 0,
	QCE_HASH_SHA256 = 1,
	QCE_HASH_LAST
};

enum qce_cipher_dir_enum {
	QCE_ENCRYPT = 0,
	QCE_DECRYPT = 1,
	QCE_CIPHER_DIR_LAST
};

enum qce_cipher_mode_enum {
	QCE_MODE_CBC = 0,
	QCE_MODE_ECB = 1,
	QCE_MODE_CTR = 2,
	QCE_CIPHER_MODE_LAST
};

enum qce_req_op_enum {
	QCE_REQ_ABLK_CIPHER = 0,
	QCE_REQ_ABLK_CIPHER_NO_KEY = 1,
	QCE_REQ_AEAD = 2,
	QCE_REQ_LAST
};

struct qce_sha_req {
	qce_comp_func_ptr_t qce_cb;
	enum qce_hash_alg_enum alg;
	unsigned char *digest;
	struct scatterlist *src;
	uint32_t  auth_data[2];
	bool first_blk;
	bool last_blk;
	unsigned int size;
	void *areq;
};

struct qce_req {
	enum qce_req_op_enum op;
	qce_comp_func_ptr_t qce_cb;
	void *areq;
	enum qce_cipher_alg_enum   alg;
	enum qce_cipher_dir_enum dir;
	enum qce_cipher_mode_enum mode;
	unsigned char *authkey;
	unsigned int authklen;
	unsigned char *enckey;
	unsigned int encklen;
	unsigned char *iv;
	unsigned int ivsize;
	unsigned int cryptlen;
	unsigned int use_pmem;
	struct qcedev_pmem_info *pmem;
};

void *qce_open(struct platform_device *pdev, int *rc);
int qce_close(void *handle);
int qce_aead_req(void *handle, struct qce_req *req);
int qce_ablk_cipher_req(void *handle, struct qce_req *req);
int qce_ota_support(void *handle);
int qce_hmac_support(void *handle);
int qce_process_sha_req(void *handle, struct qce_sha_req *s_req);

#endif /* __CRYPTO_MSM_QCE_H */
