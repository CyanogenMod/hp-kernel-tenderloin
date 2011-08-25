/** 
* @file lowmemnotify.h
* 
* Copyright (c) 2008 Palm, Inc. or its subsidiaries.
* All rights reserved.
* 
*/

#define MEMNOTIFY_DEVICE   "memnotify"

#define MEMNOTIFY_INVALID  0xffff
#define MEMNOTIFY_NORMAL   0x0000
#define MEMNOTIFY_MEDIUM   0xcfee
#define MEMNOTIFY_LOW	   0xfaac
#define MEMNOTIFY_CRITICAL 0xdead
#define MEMNOTIFY_REBOOT   0xb00f

int memnotify_threshold(void);

unsigned long memnotify_get_free(void);
