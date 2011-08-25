#include <linux/module.h>
#include <linux/smp_lock.h>
#include <linux/kmod.h>
#include <linux/completion.h>
#include <linux/binfmts.h>
#include <linux/fs.h>
#include <linux/ptrace.h>
#include <linux/freezer.h>
#include <linux/minicore2.h>

#include <asm/param.h>
#include <asm/uaccess.h>
#include <asm/unistd.h>
#include <asm/siginfo.h>

//#define MINICORE_DEBUG
#ifdef MINICORE_DEBUG
#define DBG(...) do {   \
	printk(__VA_ARGS__); \
} while (0)
#else
#define DBG(...)
#endif

#define DEFAULT_MINICORE_TIMEOUT	5	/* default timeout in secs */

char minicore_pattern[CORENAME_MAX_SIZE] = "core";
int minicore_timeout = DEFAULT_MINICORE_TIMEOUT;
int format_corename(char *corename, const char *pattern, long signr, siginfo_t *info);

void ptrace_stop(int exit_code, int nostop_code, siginfo_t *info);

static int call_minicore(long signr, siginfo_t *sig_info);

static void signal_other_threads_sig(struct task_struct *p, int signr)
{
	struct task_struct *t;

	p->signal->group_stop_count = 0;

	if (thread_group_empty(p))
		return;

	for (t = next_thread(p); t != p; t = next_thread(t)) {

		DBG("%d/%d attempt to stop %d\n", current->pid, current->tgid,
			   t->pid);

		/*
		 * Don't bother with already dead threads
		 */
		if (t->exit_state)
			continue;

		force_sig_info(SIGSTOP, SEND_SIG_FORCED, t);

		/* SIGKILL will be handled before any pending SIGSTOP */
		//sigaddset(&t->pending.signal, signr);
		//signal_wake_up(t, 1);
	}
}

/**
 * Called from get_signal_to_deliver (signal.c)
 */
int
minicore_launch(long signr, siginfo_t *info)
{
	DECLARE_COMPLETION_ONSTACK(ptrace_attach_done);
	int ret;
	unsigned long timeleft = 0;

	DBG("%d/%d %s\n", current->pid, current->tgid, __FUNCTION__);

	/* completion tells when the ptrace attach is done. */
	task_lock(current);
	current->ptrace_attach_done = &ptrace_attach_done;
	task_unlock(current);

	/* Spawns userspace minicore app. */
	ret = call_minicore(signr, info);
	if (ret < 0)
	{
		goto end_minicore;
	}

	/* Stop all the other threads of this process */
	signal_other_threads_sig(current, signr);

	printk(KERN_INFO "%s: CRASH! %s(%d) received %ld. "
			"Waiting up to %ds for minicore to attach.\n",
			__FUNCTION__, current->comm, current->tgid, signr, minicore_timeout);

	DBG("%d/%d wait_ptrace\n", current->pid, current->tgid);

	/* Wait for minicore to ptrace attach */
	if (minicore_timeout > 0) {
		timeleft =
			wait_for_completion_timeout(current->ptrace_attach_done, minicore_timeout*HZ);
	} else {
		wait_for_completion(current->ptrace_attach_done);
	}

	task_lock(current);	
	current->ptrace_attach_done = NULL;
	task_unlock(current);	

	if (minicore_timeout > 0 && !timeleft)
	{
		printk("%d/%d minicore timedout\n",
				current->pid, current->tgid);

		ret = -ETIMEDOUT;
		goto end_minicore;
	}

	DBG("%d/%d ptrace attach.\n",
			current->pid, current->tgid);

	spin_lock_irq(&current->sighand->siglock);

	/**
	 * Fake a SIGSTOP b/c gdb is wait4'ing for it
	 * after PTRACE_ATTACH.
	 */
	if (current->ptrace & PT_PTRACED)
	{
		printk("%s: minicore attached!\n", __FUNCTION__);

		ptrace_signal_deliver(regs, cookie);

		DBG("%d/%d ptrace_stop\n",
				current->pid, current->tgid);
		/* Let the debugger run.  */
		ptrace_stop(SIGSTOP, SIGSTOP, info);
	}

	spin_unlock_irq(&current->sighand->siglock);

	DBG("%d/%d done.\n", current->pid, current->tgid);

	ret = 0;
end_minicore:
	return ret;
}

static void
argv_cleanup(struct subprocess_info *info)
{
	argv_free(info->argv);
}

static int
call_minicore(long signr, siginfo_t *sig_info)
{
	char corename[CORENAME_MAX_SIZE + 1];
	char **helper_argv = NULL;
	int helper_argc = 0;
	int is_handled;
	int ret = -1;
	char *delimit;
	struct subprocess_info *info;

	if (minicore_pattern[0] != '|')
	{
		goto fail_nocleanup;
	}

	lock_kernel();
	is_handled = format_corename(corename, minicore_pattern, signr, sig_info);
	unlock_kernel();

	if (!is_handled)
	{
		goto fail_nocleanup;
	}

	helper_argv = argv_split(GFP_KERNEL, corename+1, &helper_argc);
	delimit = strchr(corename, ' ');
	if (delimit)
		*delimit = '\0';

	delimit = strrchr(helper_argv[0], '/');
	if (delimit)
		delimit++;
	else
		delimit = helper_argv[0];       
	if (!strcmp(delimit, current->comm)) {
		printk(KERN_NOTICE "Recursive core dump detected, "
				"aborting\n");
		goto fail_helper_args;
	}

	info = call_usermodehelper_setup(helper_argv[0], helper_argv, NULL, GFP_ATOMIC);
	if (!info)
	{
		goto fail_kmod;
	}

	// NOTE: after this, do NOT call argv_free() as call_usermodehelper_exec
	// will do cleanup for us.
	call_usermodehelper_setfns(info, NULL, argv_cleanup, NULL);

	if ((ret = call_usermodehelper_exec(info, UMH_NO_WAIT))) {

		printk(KERN_ERR "Could not attach %s to crashed %s (%d/%d) "
						"because of %d.\n",
				corename+1, current->comm, current->pid, current->tgid, ret);
		goto fail_nocleanup;
	}

	return 0;

fail_kmod:
fail_helper_args:
	if (helper_argv) argv_free(helper_argv);
fail_nocleanup:
	return ret;
}
