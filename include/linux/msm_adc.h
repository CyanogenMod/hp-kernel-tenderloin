/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
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

#ifndef __MSM_ADC_H
#define __MSM_ADC_H

#define MSM_ADC_MAX_CHAN_STR 64

/* must be <= to the max buffer size in the modem implementation */
#define MSM_ADC_DEV_MAX_INFLIGHT 9

#define MSM_ADC_IOCTL_CODE		0x90

struct msm_adc_conversion {
	/* hwmon channel number - this is not equivalent to the DAL chan */
	uint32_t chan;
	/* returned result in ms */
	int result;
};

/*
 * Issue a blocking adc conversion request. Once the call returns, the data
 * can be found in the 'result' field of msm_adc_conversion. This call will
 * return ENODATA if there is an invalid result returned by the modem driver.
 */
#define MSM_ADC_REQUEST			_IOWR(MSM_ADC_IOCTL_CODE, 1,	\
					     struct msm_adc_conversion)

/*
 * Issue a non-blocking adc conversion request. The results from this
 * request can be obtained by calling AIO_READ once the transfer is
 * completed. To verify completion, the blocking call AIO_POLL can be used.
 * If there are no slot resources, this call will return an error with errno
 * set to EWOULDBLOCK.
 */
#define MSM_ADC_AIO_REQUEST		_IOWR(MSM_ADC_IOCTL_CODE, 2,	\
					     struct msm_adc_conversion)

/*
 * Same non-blocking semantics as AIO_REQUEST, except this call will block
 * if there are no available slot resources. This call can fail with errno
 * set to EDEADLK if there are no resources and the file descriptor in question
 * has outstanding conversion requests already. This is done so the client
 * does not block on resources that can only be freed by reading the results --
 * effectively deadlocking the system. In this case, the client must read
 * pending results before proceeding to free up resources.
 */
#define MSM_ADC_AIO_REQUEST_BLOCK_RES	_IOWR(MSM_ADC_IOCTL_CODE, 3,	\
					     struct msm_adc_conversion)

/*
 * Returns the number of pending results that are associated with a particular
 * file descriptor. If there are no pending results, this call will block until
 * there is at least one. If there are no requests queued at all on this file
 * descriptor, this call will fail with EDEADLK. This is to prevent deadlock in
 * a single-threaded scenario where POLL would never return.
 */
#define MSM_ADC_AIO_POLL		_IOR(MSM_ADC_IOCTL_CODE, 4,	\
					     uint32_t)

struct msm_adc_aio_result {
	uint32_t chan;
	int result;
};

/*
 * Read the results from an AIO / non-blocking conversion request. AIO_POLL
 * should be used before using this command to verify how many pending requests
 * are available for the file descriptor. This call will fail with errno set to
 * ENOMSG if there are no pending messages to be read at the time of the call.
 * The call will return ENODATA if there is an invalid result returned by the
 * modem driver.
 */
#define MSM_ADC_AIO_READ		_IOR(MSM_ADC_IOCTL_CODE, 5,	\
					     struct msm_adc_aio_result)

struct msm_adc_lookup {
	/* channel name (input) */
	char name[MSM_ADC_MAX_CHAN_STR];
	/* local channel index (output) */
	uint32_t chan_idx;
};

/*
 * Look up a channel name and get back an index that can be used
 * as a parameter to the conversion request commands.
 */
#define MSM_ADC_LOOKUP			_IOWR(MSM_ADC_IOCTL_CODE, 6,	\
					     struct msm_adc_lookup)

struct msm_adc_platform_data {
	uint32_t num_adc;
	uint32_t chan_per_adc;
	char **dev_names;
};

#endif /* __MSM_ADC_H */
