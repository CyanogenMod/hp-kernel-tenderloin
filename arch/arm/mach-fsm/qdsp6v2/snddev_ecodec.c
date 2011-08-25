/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
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
#include <mach/debug_mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <asm/uaccess.h>
#include "snddev_ecodec.h"
#include <mach/qdsp5v2/audio_dev_ctl.h>


/* Context for each external codec device */
struct snddev_ecodec_state {
	struct snddev_ecodec_data *data;
	u32 sample_rate;
	bool enabled;
};

/* Global state for the driver */
struct snddev_ecodec_drv_state {
	struct mutex dev_lock;
	u32 rx_active; /* ensure one rx device at a time */
	u32 tx_active; /* ensure one tx device at a time */
	struct clk *lpa_core_clk;
	struct clk *ecodec_clk;
};

#define ADSP_CTL 1

static struct snddev_ecodec_drv_state snddev_ecodec_drv;

static int snddev_ecodec_open_rx(struct snddev_ecodec_state *ecodec)
{
	struct snddev_ecodec_drv_state *drv = &snddev_ecodec_drv;

	MM_INFO(" snddev_ecodec_open_rx\n");

	if (!drv->tx_active) {
		/* config clocks */
		clk_enable(drv->lpa_core_clk);

		/* enable ecodec clk */
		clk_enable(drv->ecodec_clk);
	}

	ecodec->enabled = 1;
	return 0;
}

static int snddev_ecodec_close_rx(struct snddev_ecodec_state *ecodec)
{
	struct snddev_ecodec_drv_state *drv = &snddev_ecodec_drv;

	/* free GPIO */
	if (!drv->tx_active)
		clk_disable(drv->ecodec_clk);

	ecodec->enabled = 0;

	return 0;
}

static int snddev_ecodec_open_tx(struct snddev_ecodec_state *ecodec)
{
	struct snddev_ecodec_drv_state *drv = &snddev_ecodec_drv;

	MM_INFO(" snddev_ecodec_open_tx \n");

	/* request GPIO */
	if (!drv->rx_active) {
		/* config clocks */
		clk_enable(drv->lpa_core_clk);

		/* enable ecodec clk */
		clk_enable(drv->ecodec_clk);
	}

	ecodec->enabled = 1;
	return 0;
}

static int snddev_ecodec_close_tx(struct snddev_ecodec_state *ecodec)
{
	struct snddev_ecodec_drv_state *drv = &snddev_ecodec_drv;

	/* free GPIO */
	if (!drv->rx_active)
		clk_disable(drv->ecodec_clk);

	ecodec->enabled = 0;

	return 0;
}


static int snddev_ecodec_open(struct msm_snddev_info *dev_info)
{
	int rc = 0;
	struct snddev_ecodec_state *ecodec;
	struct snddev_ecodec_drv_state *drv = &snddev_ecodec_drv;

	if (!dev_info) {
		rc = -EINVAL;
		goto error;
	}

	ecodec = dev_info->private_data;

	if (ecodec->data->capability & SNDDEV_CAP_RX) {
		mutex_lock(&drv->dev_lock);
		if (drv->rx_active) {
			mutex_unlock(&drv->dev_lock);
			rc = -EBUSY;
			goto error;
		}
		rc = snddev_ecodec_open_rx(ecodec);
		if (!IS_ERR_VALUE(rc))
			drv->rx_active = 1;
		mutex_unlock(&drv->dev_lock);
	} else {
		mutex_lock(&drv->dev_lock);
		if (drv->tx_active) {
			mutex_unlock(&drv->dev_lock);
			rc = -EBUSY;
			goto error;
		}
		rc = snddev_ecodec_open_tx(ecodec);
		if (!IS_ERR_VALUE(rc))
			drv->tx_active = 1;
		mutex_unlock(&drv->dev_lock);
	}
error:
	return rc;
}

static int snddev_ecodec_close(struct msm_snddev_info *dev_info)
{
	int rc = 0;
	struct snddev_ecodec_state *ecodec;
	struct snddev_ecodec_drv_state *drv = &snddev_ecodec_drv;
	if (!dev_info) {
		rc = -EINVAL;
		goto error;
	}

	ecodec = dev_info->private_data;

	if (ecodec->data->capability & SNDDEV_CAP_RX) {
		mutex_lock(&drv->dev_lock);
		if (!drv->rx_active) {
			mutex_unlock(&drv->dev_lock);
			rc = -EPERM;
			goto error;
		}
		rc = snddev_ecodec_close_rx(ecodec);
		if (!IS_ERR_VALUE(rc))
			drv->rx_active = 0;
		mutex_unlock(&drv->dev_lock);
	} else {
		mutex_lock(&drv->dev_lock);
		if (!drv->tx_active) {
			mutex_unlock(&drv->dev_lock);
			rc = -EPERM;
			goto error;
		}
		rc = snddev_ecodec_close_tx(ecodec);
		if (!IS_ERR_VALUE(rc))
			drv->tx_active = 0;
		mutex_unlock(&drv->dev_lock);
	}

error:
	return rc;
}

static int snddev_ecodec_set_freq(struct msm_snddev_info *dev_info, u32 rate)
{
	int rc = 0;

	if (!dev_info) {
		rc = -EINVAL;
		goto error;
	}
	return 8000;

error:
	return rc;
}

static int snddev_ecodec_probe(struct platform_device *pdev)
{
	int rc = 0, i;
	struct snddev_ecodec_data *pdata;
	struct msm_snddev_info *dev_info;
	struct snddev_ecodec_state *ecodec;

	if (!pdev || !pdev->dev.platform_data) {
		printk(KERN_ALERT "Invalid caller \n");
		rc = -1;
		goto error;
	}
	pdata = pdev->dev.platform_data;

	ecodec = kzalloc(sizeof(struct snddev_ecodec_state), GFP_KERNEL);
	if (!ecodec) {
		rc = -ENOMEM;
		goto error;
	}

	dev_info = kzalloc(sizeof(struct msm_snddev_info), GFP_KERNEL);
	if (!dev_info) {
		kfree(ecodec);
		rc = -ENOMEM;
		goto error;
	}

	dev_info->name = pdata->name;
	dev_info->copp_id = pdata->copp_id;
	dev_info->acdb_id = pdata->acdb_id;
	dev_info->private_data = (void *) ecodec;
	dev_info->dev_ops.open = snddev_ecodec_open;
	dev_info->dev_ops.close = snddev_ecodec_close;
	dev_info->dev_ops.set_freq = snddev_ecodec_set_freq;
	dev_info->dev_ops.enable_sidetone = NULL;
	dev_info->capability = pdata->capability;
	dev_info->opened = 0;

	msm_snddev_register(dev_info);
	ecodec->data = pdata;
	ecodec->sample_rate = 8000; /* Default to 8KHz */
	 if (pdata->capability & SNDDEV_CAP_RX) {
		for (i = 0; i < VOC_RX_VOL_ARRAY_NUM; i++) {
			dev_info->max_voc_rx_vol[i] =
				pdata->max_voice_rx_vol[i];
			dev_info->min_voc_rx_vol[i] =
				pdata->min_voice_rx_vol[i];
		}
	}
error:
	return rc;
}

static int snddev_ecodec_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver snddev_ecodec_driver = {
	.probe = snddev_ecodec_probe,
	.remove = snddev_ecodec_remove,
	.driver = { .name = "msm_snddev_ecodec" }
};

static int __init snddev_ecodec_init(void)
{
	int rc = 0;
	struct snddev_ecodec_drv_state *ecodec_drv = &snddev_ecodec_drv;

	MM_INFO(" snddev_ecodec_init \n");
	rc = platform_driver_register(&snddev_ecodec_driver);
	if (IS_ERR_VALUE(rc))
		goto error_platform_driver;
	ecodec_drv->ecodec_clk = clk_get(NULL, "ecodec_clk");
	if (IS_ERR(ecodec_drv->ecodec_clk))
		goto error_ecodec_clk;
	ecodec_drv->lpa_core_clk = clk_get(NULL, "lpa_core_clk");
	if (IS_ERR(ecodec_drv->lpa_core_clk))
		goto error_lpa_core_clk;


	mutex_init(&ecodec_drv->dev_lock);
	ecodec_drv->rx_active = 0;
	ecodec_drv->tx_active = 0;
	return 0;

error_lpa_core_clk:
	clk_put(ecodec_drv->ecodec_clk);
error_ecodec_clk:
	platform_driver_unregister(&snddev_ecodec_driver);
error_platform_driver:

	MM_ERR("%s: encounter error\n", __func__);
	return -ENODEV;
}

static void __exit snddev_ecodec_exit(void)
{
	struct snddev_ecodec_drv_state *ecodec_drv = &snddev_ecodec_drv;

	platform_driver_unregister(&snddev_ecodec_driver);
	clk_put(ecodec_drv->ecodec_clk);

	return;
}

module_init(snddev_ecodec_init);
module_exit(snddev_ecodec_exit);

MODULE_DESCRIPTION("ECodec Sound Device driver");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");
