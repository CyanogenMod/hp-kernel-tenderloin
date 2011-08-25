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

#include <linux/module.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/elf.h>

#include <asm/uaccess.h>

#include "peripheral-reset.h"

struct pil_device {
	const char *name;
	int count;
	int id;
	struct mutex lock;
	struct platform_device pdev;
};

static struct pil_device peripherals[] = {
	{
		.name = "modem",
		.id = PIL_MODEM,
		.pdev = {
			.name = "pil_modem",
			.id = -1,
		},
	},
	{
		.name = "q6",
		.id = PIL_Q6,
		.pdev = {
			.name = "pil_q6",
			.id = -1,
		},
	},
	{
		.name = "dsps",
		.id = PIL_DSPS,
		.pdev = {
			.name = "pil_dsps",
			.id = -1,
		},
	},
};

#define for_each_pil(p) \
	for (p = peripherals; p < peripherals + ARRAY_SIZE(peripherals); p++)

static struct pil_device *find_peripheral(const char *str)
{
	struct pil_device *pil;

	for_each_pil(pil) {
		if (!strcmp(pil->name, str))
			return pil;
	}
	return NULL;
}

#define IOMAP_SIZE SZ_4M

static int load_segment(struct elf32_phdr *phdr, unsigned num,
		struct pil_device *pil)
{
	int ret, count, paddr;
	char fw_name[30];
	const struct firmware *fw;
	const u8 *data;

	snprintf(fw_name, ARRAY_SIZE(fw_name), "%s.b%02d", pil->name, num);
	ret = request_firmware(&fw, fw_name, &pil->pdev.dev);
	if (ret) {
		dev_err(&pil->pdev.dev, "Failed to locate blob %s\n", fw_name);
		return ret;
	}

	if (fw->size != phdr->p_filesz) {
		dev_err(&pil->pdev.dev, "Blob size %u doesn't match %u\n",
				fw->size, phdr->p_filesz);
		ret = -EPERM;
		goto release_fw;
	}

	/* Load the segment into memory */
	count = phdr->p_filesz;
	paddr = phdr->p_paddr;
	data = fw->data;
	while (count > 0) {
		int size;
		u8 __iomem *buf;

		size = min_t(size_t, IOMAP_SIZE, count);
		buf = ioremap(paddr, size);
		if (!buf) {
			dev_err(&pil->pdev.dev, "Failed to map memory\n");
			ret = -ENOMEM;
			goto release_fw;
		}
		memcpy(buf, data, size);
		iounmap(buf);

		count -= size;
		paddr += size;
		data += size;
	}

	/* Zero out trailing memory */
	count = phdr->p_memsz - phdr->p_filesz;
	while (count > 0) {
		int size;
		u8 __iomem *buf;

		size = min_t(size_t, IOMAP_SIZE, count);
		buf = ioremap(paddr, size);
		if (!buf) {
			dev_err(&pil->pdev.dev, "Failed to map memory\n");
			ret = -ENOMEM;
			goto release_fw;
		}
		memset(buf, 0, size);
		iounmap(buf);

		count -= size;
		paddr += size;
	}

	ret = verify_blob(phdr->p_paddr, phdr->p_memsz);
	if (ret)
		dev_err(&pil->pdev.dev, "Blob %u failed verification\n", num);

release_fw:
	release_firmware(fw);
	return ret;
}

#define HASH_SEGMENT_FLAG	BIT(25)

static int load_image(struct pil_device *pil)
{
	int i, ret, has_hash;
	char fw_name[30];
	struct elf32_hdr *ehdr;
	struct elf32_phdr *phdr;
	const struct firmware *fw;

	snprintf(fw_name, sizeof(fw_name), "%s.mdt", pil->name);
	ret = request_firmware(&fw, fw_name, &pil->pdev.dev);
	if (ret) {
		dev_err(&pil->pdev.dev, "Failed to locate %s\n", fw_name);
		goto out;
	}

	if (fw->size < sizeof(*ehdr)) {
		dev_err(&pil->pdev.dev, "Not big enough to be an elf header\n");
		ret = -EIO;
		goto release_fw;
	}

	ehdr = (struct elf32_hdr *)fw->data;
	if (memcmp(ehdr->e_ident, ELFMAG, SELFMAG)) {
		dev_err(&pil->pdev.dev, "Not an elf header\n");
		ret = -EIO;
		goto release_fw;
	}

	if (ehdr->e_phnum == 0) {
		dev_err(&pil->pdev.dev, "No loadable segments\n");
		ret = -EIO;
		goto release_fw;
	}
	if (ehdr->e_phoff > fw->size) {
		dev_err(&pil->pdev.dev, "Program header beyond size of mdt\n");
		ret = -EIO;
		goto release_fw;
	}

	phdr = (struct elf32_phdr *)(fw->data + ehdr->e_phoff);
	ret = init_image(pil->id, fw->data, fw->size);
	if (ret) {
		dev_err(&pil->pdev.dev, "Invalid firmware metadata\n");
		goto release_fw;
	}

	/* Only load segment 0 if it isn't a hash */
	has_hash = (phdr->p_flags & HASH_SEGMENT_FLAG);
	if (has_hash) {
		i = 1;
		phdr++;
	} else
		i = 0;

	for (; i < ehdr->e_phnum; i++, phdr++) {
		ret = load_segment(phdr, i, pil);
		if (ret) {
			dev_err(&pil->pdev.dev, "Failed to load segment %d\n",
					i);
			goto release_fw;
		}
	}

	ret = auth_and_reset(pil->id);
	if (ret) {
		dev_err(&pil->pdev.dev, "Failed to bring out of reset\n");
		goto release_fw;
	}

release_fw:
	release_firmware(fw);
out:
	return ret;
}

/**
 * pil_get() - Load a peripheral into memory and take it out of reset
 * @name: pointer to a string containing the name of the peripheral to load
 *
 * This function returns a pointer if it succeeds. If an error occurs an
 * ERR_PTR is returned.
 *
 * If PIL is not enabled in the kernel, the value %NULL will be returned.
 */
void *pil_get(const char *name)
{
	int ret;
	struct pil_device *pil;
	void *retval;

	pil = retval = find_peripheral(name);
	if (!pil)
		return ERR_PTR(-ENODEV);

	mutex_lock(&pil->lock);
	if (pil->count) {
		pil->count++;
		goto unlock;
	}

	ret = load_image(pil);
	if (ret) {
		retval = ERR_PTR(ret);
		goto unlock;
	}

	pil->count++;
unlock:
	mutex_unlock(&pil->lock);
	return retval;
}
EXPORT_SYMBOL(pil_get);

/**
 * pil_put() - Inform PIL the peripheral no longer needs to be active
 * @peripheral_handle: pointer from a previous call to pil_get()
 *
 * This doesn't imply that a peripheral is shutdown or in reset since another
 * driver could be using the peripheral.
 */
void pil_put(void *peripheral_handle)
{
	struct pil_device *pil = peripheral_handle;
	if (!pil || IS_ERR(pil)) {
		WARN(1, "Invalid peripheral handle\n");
		return;
	}

	mutex_lock(&pil->lock);
	WARN(!pil->count, "%s: Reference count mismatch\n", __func__);
	/* TODO: Peripheral shutdown support */
	if (pil->count == 1)
		goto unlock;
	if (pil->count)
		pil->count--;
	if (pil->count == 0)
		peripheral_shutdown(pil->id);
unlock:
	mutex_unlock(&pil->lock);
}
EXPORT_SYMBOL(pil_put);

#ifdef CONFIG_DEBUG_FS
int msm_pil_debugfs_open(struct inode *inode, struct file *filp)
{
	filp->private_data = inode->i_private;
	return 0;
}

static ssize_t msm_pil_debugfs_read(struct file *filp, char __user *ubuf,
		size_t cnt, loff_t *ppos)
{
	int r;
	char buf[40];
	struct pil_device *pil = filp->private_data;

	mutex_lock(&pil->lock);
	r = snprintf(buf, sizeof(buf), "%d\n", pil->count);
	mutex_unlock(&pil->lock);
	return simple_read_from_buffer(ubuf, cnt, ppos, buf, r);
}

static ssize_t msm_pil_debugfs_write(struct file *filp,
		const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	struct pil_device *pil = filp->private_data;
	char buf[4];

	if (cnt > sizeof(buf))
		return -EINVAL;

	if (copy_from_user(&buf, ubuf, cnt))
		return -EFAULT;

	if (!strncmp(buf, "get", 3)) {
		if (IS_ERR(pil_get(pil->name)))
			return -EIO;
	} else if (!strncmp(buf, "put", 3))
		pil_put(pil);
	else
		return -EINVAL;

	return cnt;
}

static const struct file_operations msm_pil_debugfs_fops = {
	.open	= msm_pil_debugfs_open,
	.read	= msm_pil_debugfs_read,
	.write	= msm_pil_debugfs_write,
};

static int msm_pil_debugfs_init(void)
{
	struct pil_device *pil;
	struct dentry *base_dir;

	base_dir = debugfs_create_dir("pil", NULL);

	for_each_pil(pil) {
		if (!debugfs_create_file(pil->name, S_IRUGO | S_IWUSR, base_dir,
					pil, &msm_pil_debugfs_fops))
			return -ENOMEM;
	}

	return 0;
}
#else
static int msm_pil_debugfs_init(void) { return 0; }
#endif

static int __init msm_pil_init(void)
{
	int ret;
	struct pil_device *pil;

	for_each_pil(pil) {
		mutex_init(&pil->lock);
		ret = platform_device_register(&pil->pdev);
		if (ret)
			goto fail;
	}

	return msm_pil_debugfs_init();
fail:
	for ( ; pil >= peripherals; pil--) {
		platform_device_unregister(&pil->pdev);
		mutex_destroy(&pil->lock);
	}

	return ret;
}

static void __exit msm_pil_exit(void)
{
	struct pil_device *pil;

	for_each_pil(pil) {
		platform_device_unregister(&pil->pdev);
		mutex_destroy(&pil->lock);
	}
}

arch_initcall(msm_pil_init);
module_exit(msm_pil_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Load peripheral images and bring peripherals out of reset");
