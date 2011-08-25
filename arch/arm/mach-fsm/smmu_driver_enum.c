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
 */

#define pr_fmt(fmt) "%s %i " fmt, __func__, __LINE__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/vcm_mm.h>
#include <linux/clk.h>

#include <mach/smmu_hw-8xxx.h>
#include <mach/smmu_device.h>

struct smmu_ctx_search_result {
	/* input */
	char *name;

	/* output */
	struct smmu_device *smmu_device;
	struct smmu_ctx *ctx_device;
	struct smmu_driver *drv;
};


static struct platform_device *all_smmus;

static int each_ctx(struct device *dev, void *data)
{
	struct smmu_ctx_search_result *res = data;
	struct smmu_ctx *c = dev->platform_data;

	if (!res || !c || !c->name || !res->name)
		return -1;

	if (!strcmp(res->name, c->name)) {
		res->ctx_device = c;
		return 1;
	}
	return 0;
}

static int each_smmu(struct device *dev, void *data)
{
	int found;
	struct smmu_ctx_search_result *res = data;
	struct smmu_device *s = dev->platform_data;
	struct smmu_driver *drv = dev_get_drvdata(dev);

	if (!res || !s || !drv || !res->name)
		return -1;

	found = device_for_each_child(dev, res, each_ctx);

	if (found == 1) {
		res->smmu_device = s;
		res->drv = drv;
	}

	return found;
}

struct smmu_dev *smmu_get_ctx_instance(char *ctx_name)
{
	struct smmu_ctx_search_result res;
	struct smmu_dev *dev = NULL;
	int found;

	if (!all_smmus)	{
		pr_err("no root device!\n");
		return NULL;
	}
	res.name = ctx_name;

	found = device_for_each_child(&all_smmus->dev, &res, each_smmu);

	if (found != 1) {
		pr_err("Warning: Could not find smmu ctx <%s>\n", ctx_name);
		goto fail;
	}

	dev = smmu_ctx_init(res.ctx_device->num);

	if (!dev) {
		pr_err("null device\n");
		goto fail;
	}

	if (smmu_ctx_bind(dev, res.drv) != 0) {
		pr_err("could not bind\n");
		goto fail2;
	}

	return dev;

fail2:
	smmu_ctx_deinit(dev);
fail:
	return NULL;
}
EXPORT_SYMBOL(smmu_get_ctx_instance);


int smmu_free_ctx_instance(struct smmu_dev *dev)
{
	return smmu_ctx_deinit(dev);
}
EXPORT_SYMBOL(smmu_free_ctx_instance);


unsigned long smmu_get_base_addr(struct smmu_dev *dev)
{
	if (!dev) {
		pr_err("null device\n");
		return -1;
	}
	return dev->base;
}



int smmu_driver_probe(struct platform_device *pdev)
{
	static void __iomem *regs_base;
	struct resource *r;
	struct smmu_device *smmu_dev = pdev->dev.platform_data;
	struct clk *smmu_clk = NULL;
	int irq = 0;
	int ret;
	int ncb, nm2v;
	struct smmu_driver *drv = NULL;
	resource_size_t	len;

	if (pdev->id != -1) {
		if (!smmu_dev) {
			pr_err("smmu device lacks platform data\n");
			goto fail_nodev;
		}

		if (smmu_dev->clk) {
			smmu_clk = clk_get(NULL, smmu_dev->clk);

			if (!smmu_clk) {
				pr_err("clock defined but not present\n");
				goto fail_nodev;
			}

			if (smmu_dev->clk_rate != 0)
				clk_set_rate(smmu_clk, smmu_dev->clk_rate);

			clk_enable(smmu_clk);
			clk_put(smmu_clk);
		}

		drv = kzalloc(sizeof(struct smmu_driver), GFP_KERNEL);

		if (!drv) {
			pr_err("could not allocate memory\n");
			goto fail_nomem;
		}

		r = platform_get_resource_byname(pdev,
						 IORESOURCE_MEM,
						 "physbase");
		if (!r) {
			pr_err("could not get resources\n");
			goto fail_nodev;
		}

		len = r->end - r->start + 1;

		r = request_mem_region(r->start, len, r->name);
		if (!r) {
			pr_err("could not request memory region: "
			"start=%p, len=%d\n", (void *) r->start, len);
			goto fail_busy;
		}

		regs_base = ioremap(r->start, len);

		if (!regs_base) {
			pr_err("could not ioremap: start=%p, len=%d\n",
				 (void *) r->start, len);
			goto fail_busy;
		}

		irq = platform_get_irq_byname(pdev, "secure_irq");
		if (irq < 0) {
			pr_err("could not retrieve irq\n");
			goto fail_nodev;
		}

		mb();

		if (GET_IDR((unsigned long) regs_base) == 0) {
			pr_err("IDR reads as 0\n");
			goto fail_nodev;
		}

		ret = smmu_drvdata_init(drv, (unsigned long) regs_base, irq);
		if (ret != 0) {
			pr_err("could not init drvdata: error %d\n", ret);
			goto fail_nodev;
		}

		nm2v = GET_NM2VCBMT((unsigned long) regs_base);
		ncb = GET_NCB((unsigned long) regs_base);

		printk(KERN_INFO "Registered driver for %s at %lx, "
				 "irq %d (%d cb, %d m2v) \n",
		       pdev->name, (unsigned long) regs_base,
		       irq, ncb+1, nm2v+1);

		platform_set_drvdata(pdev, drv);
	} else {
		printk(KERN_INFO "Registered SMMU root device.\n");
		ret = vcm_setup_tex_classes();

		if (ret != 0) {
			pr_err("vcm_setup_tex_classes: returned %d\n", ret);
			goto fail_nodev;
		}
		all_smmus = pdev;
	}

	return 0;

fail_busy:
	kfree(drv);
	return -EBUSY;

fail_nodev:
	kfree(drv);
	return -ENODEV;

fail_nomem:
	return -ENOMEM;
}


static int smmu_driver_remove(struct platform_device *pdev)
{
	struct smmu_driver *drv = NULL;

	drv = platform_get_drvdata(pdev);
	if (drv) {
		if (!list_empty(&drv->list_active)) {
			pr_err("active device still present\n");
			goto fail_busy;
		}

		memset(drv, 0, sizeof(struct smmu_driver));
		kfree(drv);
		platform_set_drvdata(pdev, NULL);
	}
	return 0;
fail_busy:
	return -EBUSY;
}

static int ctx_driver_probe(struct platform_device *pdev)
{
	struct smmu_ctx *c = pdev->dev.platform_data;
	struct smmu_driver *drv = NULL;
	int i;
	if (!c) {
		pr_err("no platform data defined\n");
		goto fail;
	}

	printk(KERN_INFO "Probing SMMU context %s with number %d\n", c->name,
	       c->num);
	if (!pdev->dev.parent) {
		pr_err("context has no parent\n");
		goto fail;
	}

	drv = dev_get_drvdata(pdev->dev.parent);

	if (!drv) {
		pr_err("context's parent has no drvdata\n");
		goto fail;
	}

	printk(KERN_INFO "Programming M2V tables for context at %08x\n",
	       (unsigned int) drv->base);

	/* Program the M2V tables for this context */
	for (i = 0; i < MAX_NUM_MIDS; i++) {
		int mid = c->mids[i];
		if (mid == -1)
			break;

		SET_M2VCBR_N(drv->base, mid, 0);
		SET_CBACR_N(drv->base, c->num, 0);

		/* Set VMID = MID */
		SET_VMID(drv->base, mid, mid);

		/* Set the context number for that MID to this context */
		SET_CBNDX(drv->base, mid, c->num);

		/* Set MID associated with this context bank */
		SET_CBVMID(drv->base, c->num, mid);

		/* Set security bit override to be Non-secure */
		SET_NSCFG(drv->base, mid, 3);
	}

	return 0;
fail:
	return -1;
}

static int ctx_driver_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver smmu_driver = {
	.driver = {
		.name	= "smmu",
	},
	.probe		= smmu_driver_probe,
	.remove		= smmu_driver_remove,
};

static struct platform_driver ctx_driver = {
	.driver = {
		.name	= "ctx",
	},
	.probe		= ctx_driver_probe,
	.remove		= ctx_driver_remove,
};


static int smmu_driver_init(void)
{
	int ret;
	ret = platform_driver_register(&smmu_driver);
	if (ret != 0) {
		pr_err("failed to register the SMMU driver\n");
		goto error;
	}

	ret = platform_driver_register(&ctx_driver);
	if (ret != 0) {
		pr_err("failed to register the CTX driver\n");
		goto error;
	}

	return 0;
error:
	return ret;
}

static void smmu_driver_exit(void)
{
	platform_driver_unregister(&ctx_driver);
	platform_driver_unregister(&smmu_driver);
}

subsys_initcall(smmu_driver_init);
module_exit(smmu_driver_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Zach Pfeffer <zpfeffer@codeaurora.org>");

