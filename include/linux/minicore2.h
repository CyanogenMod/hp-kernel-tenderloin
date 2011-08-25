#ifndef _MINICORE2_H_
#define _MINICORE2_H_

#include <linux/completion.h>
#include <linux/binfmts.h>
#include <linux/fs.h>

#define CORE_ENV_MAX_ARGS	8

int minicore_launch(long signr, siginfo_t *info);

#endif // _MINICORE2_H_
