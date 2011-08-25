/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef _MSM_RMNET_H_
#define _MSM_RMNET_H_

/* Bitmap macros for RmNET driver operation mode. */
#define RMNET_MODE_NONE     (0x00)
#define RMNET_MODE_LLP_ETH  (0x01)
#define RMNET_MODE_LLP_IP   (0x02)
#define RMNET_MODE_QOS      (0x04)
#define RMNET_MODE_MASK     (RMNET_MODE_LLP_ETH | \
			     RMNET_MODE_LLP_IP  | \
			     RMNET_MODE_QOS)

#define RMNET_IS_MODE_QOS(mode)  \
	((mode & RMNET_MODE_QOS) == RMNET_MODE_QOS)
#define RMNET_IS_MODE_IP(mode)   \
	((mode & RMNET_MODE_LLP_IP) == RMNET_MODE_LLP_IP)

/* IOCTL command enum
 * Values chosen to not conflict with other drivers in the ecosystem */
enum rmnet_ioctl_cmds_e {
	RMNET_IOCTL_SET_LLP_ETHERNET = 0x000089F1, /* Set Ethernet protocol  */
	RMNET_IOCTL_SET_LLP_IP       = 0x000089F2, /* Set RAWIP protocol     */
	RMNET_IOCTL_GET_LLP          = 0x000089F3, /* Get link protocol      */
	RMNET_IOCTL_SET_QOS_ENABLE   = 0x000089F4, /* Set QoS header enabled */
	RMNET_IOCTL_SET_QOS_DISABLE  = 0x000089F5, /* Set QoS header disabled*/
	RMNET_IOCTL_GET_QOS          = 0x000089F6, /* Get QoS header state   */
	RMNET_IOCTL_GET_OPMODE       = 0x000089F7, /* Get operation mode     */
	RMNET_IOCTL_OPEN             = 0x000089F8, /* Open transport port    */
	RMNET_IOCTL_CLOSE            = 0x000089F9, /* Close transport port   */
	RMNET_IOCTL_MAX
};

/* QMI QoS header definition */
#define QMI_QOS_HDR_S  __attribute((__packed__)) qmi_qos_hdr_s
struct QMI_QOS_HDR_S {
	unsigned char    version;
	unsigned char    flags;
	unsigned long    flow_id;
};

#endif /* _MSM_RMNET_H_ */
