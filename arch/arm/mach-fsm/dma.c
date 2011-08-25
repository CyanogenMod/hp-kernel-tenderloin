/* linux/arch/arm/mach-msm/dma.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2010, Code Aurora Forum. All rights reserved.
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

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/pm_runtime.h>
#include <mach/dma.h>

#define MODULE_NAME "msm_dmov"

#define MSM_DMOV_CHANNEL_COUNT 16
#define MSM_DMOV_CRCI_COUNT 16

struct msm_dmov_ci_conf {
	int start;
	int end;
	int burst;
};

struct msm_dmov_crci_conf {
	int sd;
	int blk_size;
};

struct msm_dmov_chan_conf {
	int sd;
	int block;
	int priority;
};

struct msm_dmov_conf {
	void *base;
	struct msm_dmov_crci_conf *crci_conf;
	struct msm_dmov_chan_conf *chan_conf;
	int channel_active;
	struct list_head ready_commands[MSM_DMOV_CHANNEL_COUNT];
	struct list_head active_commands[MSM_DMOV_CHANNEL_COUNT];
	unsigned int crci_mask;
	spinlock_t lock;
	unsigned int irq;
};

#ifdef CONFIG_ARCH_MSM8X60

#define DMOV_CHANNEL_DEFAULT_CONF { .sd = 0, .block = 0, .priority = 0 }
#define DMOV_CHANNEL_MODEM_CONF { .sd = 3, .block = 0, .priority = 0 }
#define DMOV_CHANNEL_CONF(secd, blk, pri) \
	{ .sd = secd, .block = blk, .priority = pri }

static struct msm_dmov_chan_conf adm0_chan_conf[] = {
	DMOV_CHANNEL_DEFAULT_CONF,
	DMOV_CHANNEL_DEFAULT_CONF,
	DMOV_CHANNEL_DEFAULT_CONF,
	DMOV_CHANNEL_DEFAULT_CONF,
	DMOV_CHANNEL_DEFAULT_CONF,
	DMOV_CHANNEL_DEFAULT_CONF,
	DMOV_CHANNEL_DEFAULT_CONF,
	DMOV_CHANNEL_DEFAULT_CONF,
	DMOV_CHANNEL_DEFAULT_CONF,
	DMOV_CHANNEL_DEFAULT_CONF,
	DMOV_CHANNEL_MODEM_CONF,
	DMOV_CHANNEL_MODEM_CONF,
	DMOV_CHANNEL_MODEM_CONF,
	DMOV_CHANNEL_MODEM_CONF,
	DMOV_CHANNEL_MODEM_CONF,
	DMOV_CHANNEL_DEFAULT_CONF,
};

static struct msm_dmov_chan_conf adm1_chan_conf[] = {
	DMOV_CHANNEL_DEFAULT_CONF,
	DMOV_CHANNEL_DEFAULT_CONF,
	DMOV_CHANNEL_DEFAULT_CONF,
	DMOV_CHANNEL_DEFAULT_CONF,
	DMOV_CHANNEL_DEFAULT_CONF,
	DMOV_CHANNEL_DEFAULT_CONF,
	DMOV_CHANNEL_DEFAULT_CONF,
	DMOV_CHANNEL_DEFAULT_CONF,
	DMOV_CHANNEL_DEFAULT_CONF,
	DMOV_CHANNEL_DEFAULT_CONF,
	DMOV_CHANNEL_MODEM_CONF,
	DMOV_CHANNEL_MODEM_CONF,
	DMOV_CHANNEL_MODEM_CONF,
	DMOV_CHANNEL_MODEM_CONF,
	DMOV_CHANNEL_MODEM_CONF,
	DMOV_CHANNEL_MODEM_CONF,
};

#define DMOV_CRCI_DEFAULT_CONF { .sd = 0, .blk_size = 0 }
#define DMOV_CRCI_CONF(secd, blk) { .sd = secd, .blk_size = blk }

static struct msm_dmov_crci_conf adm0_crci_conf[] = {
	DMOV_CRCI_DEFAULT_CONF,
	DMOV_CRCI_DEFAULT_CONF,
	DMOV_CRCI_DEFAULT_CONF,
	DMOV_CRCI_DEFAULT_CONF,
	DMOV_CRCI_CONF(0, 1),
	DMOV_CRCI_CONF(0, 1),
	DMOV_CRCI_CONF(0, 0x101),
	DMOV_CRCI_DEFAULT_CONF,
	DMOV_CRCI_DEFAULT_CONF,
	DMOV_CRCI_DEFAULT_CONF,
	DMOV_CRCI_DEFAULT_CONF,
	DMOV_CRCI_DEFAULT_CONF,
	DMOV_CRCI_DEFAULT_CONF,
	DMOV_CRCI_DEFAULT_CONF,
	DMOV_CRCI_DEFAULT_CONF,
	DMOV_CRCI_DEFAULT_CONF,
};

static struct msm_dmov_crci_conf adm1_crci_conf[] = {
	DMOV_CRCI_DEFAULT_CONF,
	DMOV_CRCI_CONF(0, 1),
	DMOV_CRCI_CONF(0, 1),
	DMOV_CRCI_DEFAULT_CONF,
	DMOV_CRCI_CONF(0, 1),
	DMOV_CRCI_CONF(0, 1),
	DMOV_CRCI_CONF(0, 5),
	DMOV_CRCI_DEFAULT_CONF,
	DMOV_CRCI_DEFAULT_CONF,
	DMOV_CRCI_DEFAULT_CONF,
	DMOV_CRCI_DEFAULT_CONF,
	DMOV_CRCI_DEFAULT_CONF,
	DMOV_CRCI_DEFAULT_CONF,
	DMOV_CRCI_DEFAULT_CONF,
	DMOV_CRCI_CONF(0, 1),
	DMOV_CRCI_DEFAULT_CONF,
};

static struct msm_dmov_conf dmov_conf[] = {
	{
		.base = MSM_DMOV_ADM0_BASE,
		.crci_conf = adm0_crci_conf,
		.chan_conf = adm0_chan_conf,
		.lock = __SPIN_LOCK_UNLOCKED(dmov_lock),
		.irq = INT_ADM0_MASTER
	}, {
		.base = MSM_DMOV_ADM1_BASE,
		.crci_conf = adm1_crci_conf,
		.chan_conf = adm1_chan_conf,
		.lock = __SPIN_LOCK_UNLOCKED(dmov_lock),
		.irq = INT_ADM1_MASTER
	}
};
#else
static struct msm_dmov_conf dmov_conf[] = {
	{
		.base = MSM_DMOV_BASE,
		.crci_conf = NULL,
		.chan_conf = NULL,
		.lock = __SPIN_LOCK_UNLOCKED(dmov_lock),
		.irq = INT_ADM_AARM
	}
};
#endif

#define MSM_DMOV_ID_COUNT (MSM_DMOV_CHANNEL_COUNT * ARRAY_SIZE(dmov_conf))
#define DMOV_REG(name, adm)    ((name) + (dmov_conf[adm].base))
#define DMOV_ID_TO_ADM(id)   ((id) / MSM_DMOV_CHANNEL_COUNT)
#define DMOV_ID_TO_CHAN(id)   ((id) % MSM_DMOV_CHANNEL_COUNT)
#define DMOV_CHAN_ADM_TO_ID(ch, adm) ((ch) + (adm) * MSM_DMOV_CHANNEL_COUNT)

#ifdef CONFIG_MSM_ADM3
#define DMOV_IRQ_TO_ADM(irq)   \
({ \
	typeof(irq) _irq = irq; \
	((_irq == INT_ADM1_MASTER) || (_irq == INT_ADM1_AARM)); \
})
#else
#define DMOV_IRQ_TO_ADM(irq) 0
#endif

enum {
	MSM_DMOV_PRINT_ERRORS = 1,
	MSM_DMOV_PRINT_IO = 2,
	MSM_DMOV_PRINT_FLOW = 4
};

unsigned int msm_dmov_print_mask = MSM_DMOV_PRINT_ERRORS;

#define MSM_DMOV_DPRINTF(mask, format, args...) \
	do { \
		if ((mask) & msm_dmov_print_mask) \
			printk(KERN_ERR format, args); \
	} while (0)
#define PRINT_ERROR(format, args...) \
	MSM_DMOV_DPRINTF(MSM_DMOV_PRINT_ERRORS, format, args);
#define PRINT_IO(format, args...) \
	MSM_DMOV_DPRINTF(MSM_DMOV_PRINT_IO, format, args);
#define PRINT_FLOW(format, args...) \
	MSM_DMOV_DPRINTF(MSM_DMOV_PRINT_FLOW, format, args);

#ifndef CONFIG_MSM_ADM3
enum {
	CLK_DIS,
	CLK_TO_BE_DIS,
	CLK_EN
};

static struct clk *msm_dmov_clk;
static unsigned int clk_ctl = CLK_DIS;

static void timer_func(unsigned long func_paramter)
{
	unsigned long irq_flags[ARRAY_SIZE(dmov_conf)];
	int i;
	for (i = 0; i < ARRAY_SIZE(dmov_conf); i++)
		spin_lock_irqsave(&dmov_conf[i].lock, irq_flags[i]);
	if (clk_ctl == CLK_TO_BE_DIS) {
		for (i = 0; i < ARRAY_SIZE(dmov_conf); i++)
			BUG_ON(dmov_conf[i].channel_active);
		clk_disable(msm_dmov_clk);
		clk_ctl = CLK_DIS;
	}
	for (i = 0; i < ARRAY_SIZE(dmov_conf); i++)
		spin_unlock_irqrestore(&dmov_conf[i].lock, irq_flags[i]);
}
DEFINE_TIMER(timer, timer_func, 0, 0);
#endif

void msm_dmov_stop_cmd(unsigned id, struct msm_dmov_cmd *cmd, int graceful)
{
	int adm = DMOV_ID_TO_ADM(id);
	int ch = DMOV_ID_TO_CHAN(id);
	writel((graceful << 31), DMOV_REG(DMOV_FLUSH0(ch), adm));
}
EXPORT_SYMBOL(msm_dmov_stop_cmd);

#define	CRCI_UNUSED   0
#define	CRCI_CONFLICT 1
#define	CRCI_MUX_OFF  2
#define	CRCI_MUX_ON   3

#ifdef CONFIG_MSM_ADM3
static int crci_mask_compare(unsigned int x, unsigned int y)
{
	unsigned int mask;
	int i;
	for (i = 0; i < MSM_DMOV_CRCI_COUNT; i++) {
		mask = (x ^ y) >> (2*i);
		if ((mask & 3) == CRCI_CONFLICT)
			return 1;
	}
	return 0;
}
#endif

static int check_crci_conflict(struct msm_dmov_cmd *cmd, int adm)
{
#ifdef CONFIG_MSM_ADM3
	int i;
	struct msm_dmov_cmd *iter;
	struct list_head *cmd_list;
	unsigned int active_crci_mask = 0;

	for (i = 0; i < MSM_DMOV_CHANNEL_COUNT; i++) {
		cmd_list = &dmov_conf[adm].active_commands[i];
		list_for_each_entry(iter, cmd_list, list) {
			active_crci_mask |= iter->crci_mask;
		}
	}
	return crci_mask_compare(cmd->crci_mask, active_crci_mask);
#endif
	return 0;
}

#define CRCI_MUXSEL(n) (((n) >> 4) & 1)
#define CRCI_NUM(n)    ((n) & 0xF)

unsigned int msm_dmov_build_crci_mask(int n, ...)
{
	unsigned int mask = 0;
#ifdef CONFIG_MSM_ADM3
	int i;
	int crci;
	int crci_num;
	unsigned int crci_muxsel;
	va_list crcis;
	va_start(crcis, n);
	for (i = 0; i < n; i++) {
		crci = va_arg(crcis, int);
		crci_muxsel = CRCI_MUXSEL(crci);
		crci_num = CRCI_NUM(crci);
		mask |= (1 << (2*crci_num + 1));
		mask |= (crci_muxsel << (2*crci_num));
	}
	va_end(crcis);
#endif
	return mask;
}
EXPORT_SYMBOL(msm_dmov_build_crci_mask);


static void set_crci_mask(int crci_mask, int adm)
{
#ifdef CONFIG_MSM_ADM3
	int i;
	int blk_size;
	unsigned int crci_ctl;
	unsigned int tmp_crci_mask;
	unsigned int blank_mask;

	for (i = 0; i < MSM_DMOV_CRCI_COUNT; i++) {
		tmp_crci_mask = (crci_mask >> (2*i)) & 3;
		if (crci_mask_compare(dmov_conf[adm].crci_mask,
				      tmp_crci_mask << (2*i))) {
			blank_mask = ~(3 << (2*i));
			blk_size = dmov_conf[adm].crci_conf[i].blk_size;
			crci_ctl =  DMOV_CRCI_CTL_BLK_SZ(blk_size);
			if (tmp_crci_mask == CRCI_MUX_ON)
				crci_ctl |= DMOV_CRCI_MUX;

			writel(crci_ctl, DMOV_REG(DMOV_CRCI_CTL(i), adm));
			dmov_conf[adm].crci_mask &= blank_mask;
			dmov_conf[adm].crci_mask |= (tmp_crci_mask << (2*i));
		}
	}
#endif
}

static void start_ready_cmds(int adm)
{
#ifdef CONFIG_MSM_ADM3
	int i;
	unsigned int status;
	struct list_head *rdy;
	struct list_head *act;
	struct msm_dmov_cmd *cmd;
	for (i = 0; i < MSM_DMOV_CHANNEL_COUNT; i++) {
		rdy = &dmov_conf[adm].ready_commands[i];
		act = &dmov_conf[adm].active_commands[i];
		cmd = list_entry(rdy->next, typeof(*cmd), list);
		if (!list_empty(rdy) && !check_crci_conflict(cmd, adm)) {
			status = readl(DMOV_REG(DMOV_STATUS(i), adm));
			if (status & DMOV_STATUS_CMD_PTR_RDY) {
				list_del(&cmd->list);
				list_add_tail(&cmd->list, act);
				dmov_conf[adm].channel_active |= (1 << i);
				set_crci_mask(cmd->crci_mask, adm);
				writel(cmd->cmdptr,
				       DMOV_REG(DMOV_CMD_PTR(i), adm));
			}
		}
	}
#endif
}

void msm_dmov_enqueue_cmd_ext(unsigned id, struct msm_dmov_cmd *cmd)
{
	unsigned long irq_flags;
	unsigned int status;
	int adm = DMOV_ID_TO_ADM(id);
	int ch = DMOV_ID_TO_CHAN(id);

	spin_lock_irqsave(&dmov_conf[adm].lock, irq_flags);
#ifndef CONFIG_MSM_ADM3
	if (clk_ctl == CLK_DIS)
		clk_enable(msm_dmov_clk);
	else if (clk_ctl == CLK_TO_BE_DIS)
		del_timer(&timer);
	clk_ctl = CLK_EN;
#endif
	status = readl(DMOV_REG(DMOV_STATUS(ch), adm));
	if ((status & DMOV_STATUS_CMD_PTR_RDY) &&
	    (!check_crci_conflict(cmd, adm))) {
		PRINT_IO("msm_dmov_enqueue_cmd(%d), start command, status %x\n",
			id, status);
		if (cmd->exec_func)
			cmd->exec_func(cmd);
		list_add_tail(&cmd->list, &dmov_conf[adm].active_commands[ch]);
		if (!dmov_conf[adm].channel_active)
			enable_irq(dmov_conf[adm].irq);
		dmov_conf[adm].channel_active |= 1U << ch;
		PRINT_IO("Writing %x exactly to register", cmd->cmdptr);
		set_crci_mask(cmd->crci_mask, adm);
		writel(cmd->cmdptr, DMOV_REG(DMOV_CMD_PTR(ch), adm));
	} else {
#ifndef CONFIG_MSM_ADM3
		if (!dmov_conf[adm].channel_active) {
			clk_ctl = CLK_TO_BE_DIS;
			mod_timer(&timer, jiffies + HZ);
		}
		if (list_empty(&dmov_conf[adm].active_commands[ch]))
			PRINT_ERROR("msm_dmov_enqueue_cmd_ext(%d), stalled, "
				"status %x\n", id, status);
#endif
		PRINT_IO("msm_dmov_enqueue_cmd(%d), enqueue command, status "
		    "%x\n", id, status);
		list_add_tail(&cmd->list, &dmov_conf[adm].ready_commands[ch]);
	}
	spin_unlock_irqrestore(&dmov_conf[adm].lock, irq_flags);
}
EXPORT_SYMBOL(msm_dmov_enqueue_cmd_ext);

void msm_dmov_enqueue_cmd(unsigned id, struct msm_dmov_cmd *cmd)
{
	/* Disable callback function (for backwards compatibility) */
	cmd->exec_func = NULL;

	msm_dmov_enqueue_cmd_ext(id, cmd);
}
EXPORT_SYMBOL(msm_dmov_enqueue_cmd);

void msm_dmov_flush(unsigned int id)
{
	unsigned long irq_flags;
	int ch = DMOV_ID_TO_CHAN(id);
	int adm = DMOV_ID_TO_ADM(id);
	spin_lock_irqsave(&dmov_conf[adm].lock, irq_flags);
	/* XXX not checking if flush cmd sent already */
	if (!list_empty(&dmov_conf[adm].active_commands[ch])) {
		PRINT_IO("msm_dmov_flush(%d), send flush cmd\n", id);
		writel(DMOV_FLUSH_TYPE, DMOV_REG(DMOV_FLUSH0(ch), adm));
	}
	spin_unlock_irqrestore(&dmov_conf[adm].lock, irq_flags);
}
EXPORT_SYMBOL(msm_dmov_flush);

struct msm_dmov_exec_cmdptr_cmd {
	struct msm_dmov_cmd dmov_cmd;
	struct completion complete;
	unsigned id;
	unsigned int result;
	struct msm_dmov_errdata err;
};

static void
dmov_exec_cmdptr_complete_func(struct msm_dmov_cmd *_cmd,
			       unsigned int result,
			       struct msm_dmov_errdata *err)
{
	struct msm_dmov_exec_cmdptr_cmd *cmd = container_of(_cmd, struct msm_dmov_exec_cmdptr_cmd, dmov_cmd);
	cmd->result = result;
	if (result != 0x80000002 && err)
		memcpy(&cmd->err, err, sizeof(struct msm_dmov_errdata));

	complete(&cmd->complete);
}

int msm_dmov_exec_cmd(unsigned id, unsigned int crci_mask, unsigned int cmdptr)
{
	struct msm_dmov_exec_cmdptr_cmd cmd;

	PRINT_FLOW("dmov_exec_cmdptr(%d, %x)\n", id, cmdptr);

	cmd.dmov_cmd.cmdptr = cmdptr;
	cmd.dmov_cmd.crci_mask = crci_mask;
	cmd.dmov_cmd.complete_func = dmov_exec_cmdptr_complete_func;
	cmd.dmov_cmd.exec_func = NULL;
	cmd.id = id;
	init_completion(&cmd.complete);

	msm_dmov_enqueue_cmd(id, &cmd.dmov_cmd);
	wait_for_completion_io(&cmd.complete);

	if (cmd.result != 0x80000002) {
		PRINT_ERROR("dmov_exec_cmdptr(%d): ERROR, result: %x\n", id, cmd.result);
		PRINT_ERROR("dmov_exec_cmdptr(%d):  flush: %x %x %x %x\n",
			id, cmd.err.flush[0], cmd.err.flush[1], cmd.err.flush[2], cmd.err.flush[3]);
		return -EIO;
	}
	PRINT_FLOW("dmov_exec_cmdptr(%d, %x) done\n", id, cmdptr);
	return 0;
}
EXPORT_SYMBOL(msm_dmov_exec_cmd);

static void fill_errdata(struct msm_dmov_errdata *errdata, int ch, int adm)
{
	errdata->flush[0] = readl(DMOV_REG(DMOV_FLUSH0(ch), adm));
	errdata->flush[1] = readl(DMOV_REG(DMOV_FLUSH1(ch), adm));
	errdata->flush[2] = readl(DMOV_REG(DMOV_FLUSH2(ch), adm));
	errdata->flush[3] = readl(DMOV_REG(DMOV_FLUSH3(ch), adm));
	errdata->flush[4] = readl(DMOV_REG(DMOV_FLUSH4(ch), adm));
	errdata->flush[5] = readl(DMOV_REG(DMOV_FLUSH5(ch), adm));
}

static irqreturn_t msm_datamover_irq_handler(int irq, void *dev_id)
{
	unsigned int int_status;
	unsigned int mask;
	unsigned int id;
	unsigned int ch;
	unsigned long irq_flags;
	unsigned int ch_status;
	unsigned int ch_result;
	struct msm_dmov_cmd *cmd;
	int adm = DMOV_IRQ_TO_ADM(irq);

	spin_lock_irqsave(&dmov_conf[adm].lock, irq_flags);

	int_status = readl(DMOV_REG(DMOV_ISR, adm)); /* read and clear isr */
	PRINT_FLOW("msm_datamover_irq_handler: DMOV_ISR %x\n", int_status);

	while (int_status) {
		mask = int_status & -int_status;
		ch = fls(mask) - 1;
		id = DMOV_CHAN_ADM_TO_ID(ch, adm);
		PRINT_FLOW("msm_datamover_irq_handler %08x %08x id %d\n", int_status, mask, id);
		int_status &= ~mask;
		ch_status = readl(DMOV_REG(DMOV_STATUS(ch), adm));
		if (!(ch_status & DMOV_STATUS_RSLT_VALID)) {
			PRINT_FLOW("msm_datamover_irq_handler id %d, "
				"result not valid %x\n", id, ch_status);
			continue;
		}
		do {
			ch_result = readl(DMOV_REG(DMOV_RSLT(ch), adm));
			if (list_empty(&dmov_conf[adm].active_commands[ch])) {
				PRINT_ERROR("msm_datamover_irq_handler id %d, got result "
					"with no active command, status %x, result %x\n",
					id, ch_status, ch_result);
				cmd = NULL;
			} else {
				cmd = list_entry(dmov_conf[adm].
					active_commands[ch].next, typeof(*cmd),
					list);
			}
			PRINT_FLOW("msm_datamover_irq_handler id %d, status %x, result %x\n", id, ch_status, ch_result);
			if (ch_result & DMOV_RSLT_DONE) {
				PRINT_FLOW("msm_datamover_irq_handler id %d, status %x\n",
					id, ch_status);
				PRINT_IO("msm_datamover_irq_handler id %d, got result "
					"for %p, result %x\n", id, cmd, ch_result);
				if (cmd) {
					list_del(&cmd->list);
					cmd->complete_func(cmd, ch_result, NULL);
				}
			}
			if (ch_result & DMOV_RSLT_FLUSH) {
				struct msm_dmov_errdata errdata;

				fill_errdata(&errdata, ch, adm);
				PRINT_FLOW("msm_datamover_irq_handler id %d, status %x\n", id, ch_status);
				PRINT_FLOW("msm_datamover_irq_handler id %d, flush, result %x, flush0 %x\n", id, ch_result, errdata.flush[0]);
				if (cmd) {
					list_del(&cmd->list);
					cmd->complete_func(cmd, ch_result, &errdata);
				}
			}
			if (ch_result & DMOV_RSLT_ERROR) {
				struct msm_dmov_errdata errdata;

				fill_errdata(&errdata, ch, adm);

				PRINT_ERROR("msm_datamover_irq_handler id %d, status %x\n", id, ch_status);
				PRINT_ERROR("msm_datamover_irq_handler id %d, error, result %x, flush0 %x\n", id, ch_result, errdata.flush[0]);
				if (cmd) {
					list_del(&cmd->list);
					cmd->complete_func(cmd, ch_result, &errdata);
				}
				/* this does not seem to work, once we get an error */
				/* the datamover will no longer accept commands */
				writel(0, DMOV_REG(DMOV_FLUSH0(ch), adm));
			}
			ch_status = readl(DMOV_REG(DMOV_STATUS(ch), adm));
#ifndef CONFIG_MSM_ADM3
			PRINT_FLOW("msm_datamover_irq_handler id %d, status %x\n", id, ch_status);
			if ((ch_status & DMOV_STATUS_CMD_PTR_RDY) &&
			    !list_empty(&dmov_conf[adm].ready_commands[ch])) {
				cmd = list_entry(dmov_conf[adm].
					ready_commands[ch].next, typeof(*cmd),
					list);
				list_del(&cmd->list);
				if (cmd->exec_func)
					cmd->exec_func(cmd);
				list_add_tail(&cmd->list,
					&dmov_conf[adm].active_commands[ch]);
				PRINT_FLOW("msm_datamover_irq_handler id %d, start command\n", id);
				writel(cmd->cmdptr, DMOV_REG(DMOV_CMD_PTR(ch),
					adm));
			}
#endif
		} while (ch_status & DMOV_STATUS_RSLT_VALID);
		if (list_empty(&dmov_conf[adm].active_commands[ch]) &&
				list_empty(&dmov_conf[adm].ready_commands[ch]))
			dmov_conf[adm].channel_active &= ~(1U << ch);
		PRINT_FLOW("msm_datamover_irq_handler id %d, status %x\n", id, ch_status);
	}

	start_ready_cmds(adm);
	if (!dmov_conf[adm].channel_active) {
		disable_irq_nosync(dmov_conf[adm].irq);
#ifndef CONFIG_MSM_ADM3
		clk_ctl = CLK_TO_BE_DIS;
		mod_timer(&timer, jiffies + HZ);
#endif
	}

	spin_unlock_irqrestore(&dmov_conf[adm].lock, irq_flags);
	return IRQ_HANDLED;
}

#ifndef CONFIG_MSM_ADM3
static int msm_dmov_suspend_late(struct platform_device *pdev,
			    pm_message_t state)
{
	int i;
	unsigned long irq_flags[ARRAY_SIZE(dmov_conf)];
	for (i = 0; i < ARRAY_SIZE(dmov_conf); i++)
		spin_lock_irqsave(&dmov_conf[i].lock, irq_flags[i]);
	if (clk_ctl == CLK_TO_BE_DIS) {
		for (i = 0; i < ARRAY_SIZE(dmov_conf); i++)
			BUG_ON(dmov_conf[i].channel_active);
		del_timer(&timer);
		clk_disable(msm_dmov_clk);
		clk_ctl = CLK_DIS;
	}
	for (i = 0; i < ARRAY_SIZE(dmov_conf); i++)
		spin_unlock_irqrestore(&dmov_conf[i].lock, irq_flags[i]);
	return 0;
}

static int msm_dmov_runtime_suspend(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: suspending...\n");
	return 0;
}

static int msm_dmov_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: resuming...\n");
	return 0;
}

static int msm_dmov_runtime_idle(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: idling...\n");
	return 0;
}

static struct dev_pm_ops msm_dmov_dev_pm_ops = {
	.runtime_suspend = msm_dmov_runtime_suspend,
	.runtime_resume = msm_dmov_runtime_resume,
	.runtime_idle = msm_dmov_runtime_idle,
};

static struct platform_driver msm_dmov_driver = {
	.suspend = msm_dmov_suspend_late,
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
		.pm = &msm_dmov_dev_pm_ops,
	},
};
#endif


static void config_datamover(int adm)
{
#ifdef CONFIG_MSM_ADM3
	int i;
	for (i = 0; i < MSM_DMOV_CHANNEL_COUNT; i++) {
		struct msm_dmov_chan_conf *chan_conf =
			dmov_conf[adm].chan_conf;
		unsigned conf;
		/* Only configure scorpion channels */
		if (chan_conf[i].sd <= 1) {
			conf = readl(DMOV_REG(DMOV_CONF(i), adm));
			conf &= ~DMOV_CONF_SD(7);
			conf |= DMOV_CONF_SD(chan_conf[i].sd);
			writel(conf | DMOV_CONF_SHADOW_EN,
			       DMOV_REG(DMOV_CONF(i), adm));
		}
	}
#endif
}

/* static int __init */
static int __init msm_init_datamover(void)
{
	int i;
	int j;
	int ret;
	for (j = 0; j < ARRAY_SIZE(dmov_conf); j++) {
		config_datamover(j);
		for (i = 0; i < MSM_DMOV_CHANNEL_COUNT; i++) {
			INIT_LIST_HEAD(&dmov_conf[j].ready_commands[i]);
			INIT_LIST_HEAD(&dmov_conf[j].active_commands[i]);
		}
		for (i = 0; i < MSM_DMOV_CHANNEL_COUNT; i++) {
			writel(DMOV_RSLT_CONF_IRQ_EN
			     | DMOV_RSLT_CONF_FORCE_FLUSH_RSLT,
			       DMOV_REG(DMOV_RSLT_CONF(i), j));
		}
		ret = request_irq(dmov_conf[j].irq, msm_datamover_irq_handler,
			0, "msmdatamover", NULL);
		if (ret) {
			PRINT_ERROR("Requesting ADM%d irq %d failed\n", j,
				dmov_conf[j].irq);
			return ret;
		}
		disable_irq(dmov_conf[j].irq);
	}
#ifndef CONFIG_MSM_ADM3
	msm_dmov_clk = clk_get(NULL, "adm_clk");
	if (IS_ERR(msm_dmov_clk))
		return PTR_ERR(msm_dmov_clk);
	ret = platform_driver_register(&msm_dmov_driver);
	if (ret)
		return ret;
#endif
	return 0;
}
arch_initcall(msm_init_datamover);
