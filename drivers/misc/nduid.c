/* 
 * drivers/misc/nduid.c
 *
 * Copyright (C) 2009 Palm, Inc.
 * Author: Yvonne Yip <y@palm.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/crypto.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/nduid.h>

static void *nduid_start(struct seq_file *m, loff_t *pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}

static void *nduid_next(struct seq_file *m, void *v, loff_t *pos)
{
	++pos;
	return NULL;
}

static void nduid_stop(struct seq_file *m, void *v)
{
	/* nothing to do */
}

static unsigned char nduid[128];

static int nduid_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", nduid);
	return 0;
}

static struct seq_operations nduid_ops = {
	.start = nduid_start,
	.next = nduid_next,
	.stop = nduid_stop,
	.show = nduid_show,
};

static int nduid_proc_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &nduid_ops);
}

static struct file_operations nduid_proc_ops = {
	.open = nduid_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

static int __init nduid_setup(char *str)
{
	strncpy(nduid, str, sizeof(nduid));
	return 0;
}
__setup("nduid=", nduid_setup);

struct nduid_ctxt {
	struct miscdevice mdev;
	struct file_operations fops;
	unsigned char *nduid_binary;
	size_t nduid_binary_len;
};

static struct nduid_ctxt *_nduid_ctxt = NULL;

static ssize_t nduid_mdev_read(struct file *file, char __user *buf,
		size_t count, loff_t *ptr)
{
	struct nduid_ctxt *ctxt;

	ctxt = container_of(file->f_op, struct nduid_ctxt, fops);

	if (count < ctxt->nduid_binary_len) {
		return -EINVAL;
	}

	if (copy_to_user(buf, ctxt->nduid_binary, ctxt->nduid_binary_len)) {
		return -EFAULT;
	}
	return ctxt->nduid_binary_len;
}

static struct file_operations nduid_mdev_fops = {
	.owner = THIS_MODULE,
	.read = nduid_mdev_read,
	.open = nonseekable_open,
};

/*
 * nduid_hex_to_char - This function gets an input string
 * of 2 characters, and converts it into an unsigned
 * character equivalent.
 * Returns -1 on error
 */
static int nduid_hex_to_char (char *source, unsigned char *dest)
{
	unsigned char x, c = 0;
	if ( (NULL == source) || (source+1 == NULL) ) {
		return -1;
	}
	if (NULL == dest) {
		return -1;
	}

	x = source[0];
	// Ignore zeros on the high part, since the nduid
	// would have been padded with zeros, for single
	// characters (e.g 0xa would be 0x0a in the nduid string)

	if ('0' != x) {
		if ( (x >= '0') && (x <= '9') ) {
			c = x - '0';
		} else if ( (x >= 'a') && (x <= 'f') ) {
			c = x - 'a' + 10;
		} else if ( (x >= 'A') && (x <= 'F') ) {
			c = x - 'A' + 10;
		} else {
			return -1;
		}
		c <<= 4;
	}

	x = source[1];

	if ( (x >= '0') && (x <= '9') ) {
		c |= (x - '0');
	} else if ( (x >= 'a') && (x <= 'f') ) {
		c |= (x - 'a' + 10);
	} else if ( (x >= 'A') && (x <= 'F') ) {
		c |= (x - 'A' + 10);
	} else {
		return -1;
	}
	*dest = c;
	return 0;
}


static int nduid_probe(struct platform_device *pdev)
{
	int r = 0, i;
	struct nduid_config *pcfg;
	struct nduid_ctxt *ctxt;
	unsigned char c;
	if (_nduid_ctxt) {
		printk(KERN_ERR "nduid: more than one nduid device\n");
		return -EINVAL;
	}

	pcfg = pdev->dev.platform_data;
	if (!pcfg) {
		printk(KERN_ERR "nduid: no platform data\n");
		return -ENODEV;
	}

	ctxt = kzalloc(sizeof(struct nduid_ctxt), GFP_KERNEL);
	if (!ctxt) {
		printk(KERN_ERR "nduid: can't alloc ctxt\n");
		return -ENOMEM;
	}

	_nduid_ctxt = ctxt;
	platform_set_drvdata(pdev, ctxt);

	// Allocate half the length of nduid, as 2 chars
	// of the string, represent one byte of binary data
	ctxt->nduid_binary_len  = strlen (nduid) / 2;

	ctxt->nduid_binary = kmalloc(ctxt->nduid_binary_len, GFP_KERNEL);
	if (!ctxt->nduid_binary) {
		printk(KERN_ERR "nduid: error allocating nduid binary\n");
		goto nduid_probe_fail;
	}

	// Construct the binary data, from the nduid string
	for (i = 0; i < ctxt->nduid_binary_len; i++) {
		c = 0;
		if (nduid_hex_to_char (nduid + (i*2), &c) ) {
			break;
		}
		ctxt->nduid_binary [i] = c;
	}

	// If any errors in constructing the dev id,
	// zero out the binary
	if (i != ctxt->nduid_binary_len) {
		memset(ctxt->nduid_binary, 0, ctxt->nduid_binary_len);
	}

	/* init misc device */
	memcpy(&ctxt->fops, &nduid_mdev_fops, sizeof(struct file_operations));
	ctxt->mdev.minor = MISC_DYNAMIC_MINOR;
	ctxt->mdev.name = pcfg->dev_name;
	ctxt->mdev.fops = &ctxt->fops;

	/* register misc device */
	r = misc_register(&ctxt->mdev);
	if (r) {
		printk(KERN_ERR "nduid: failed to register misc device\n");
		goto nduid_probe_fail;
	}
	return 0;

nduid_probe_fail:
	if (ctxt) {
		if (ctxt->nduid_binary) {
			kfree(ctxt->nduid_binary);
		}
		kfree(ctxt);
	}
	return -1;
}

static int __devexit nduid_remove(struct platform_device *pdev)
{
	struct nduid_ctxt *ctxt;

	ctxt = platform_get_drvdata(pdev);
	if (!ctxt) {
		return 0;
	}

	misc_deregister(&ctxt->mdev);

	kfree(ctxt->nduid_binary);
	kfree(ctxt);
	return 0;
}


#ifdef CONFIG_PM
static int nduid_suspend(struct platform_device *pdev, pm_message_t state)
{
	/* nothing to do */
	return 0;
}

static int nduid_resume(struct platform_device *pdev)
{
	/* nothing to do */
	return 0;
}
#else
#define nduid_suspend        NULL
#define nduid_resume         NULL
#endif

static struct platform_driver nduid_driver = {
	.driver = {
		.name = "nduid",
	},
	.probe = nduid_probe,
	.remove = __devexit_p(nduid_remove),
	.suspend = nduid_suspend,
	.resume = nduid_resume,
};

static int __init nduid_init(void)
{
	struct proc_dir_entry *entry;

	entry = create_proc_entry("nduid", S_IRUGO, NULL);
	if (entry == NULL) {
		return -1;
	}

	entry->proc_fops = &nduid_proc_ops;

	printk(KERN_INFO "nduid: %s\n", nduid);

	return platform_driver_register(&nduid_driver);
}

static void __exit nduid_exit(void)
{
	platform_driver_unregister(&nduid_driver);
}

module_init(nduid_init);
module_exit(nduid_exit);

MODULE_DESCRIPTION("Nova device UID driver");
MODULE_LICENSE("GPL");
