/*
 * Texas Instruments Keystone IPC IRQ chip definitions
 *
 * Copyright (C) 2012 Texas Instruments, Inc.
 * Author: Cyril Chemparathy <cyril@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __LINUX_KEYSTONE_IPC_IRQ_H__
#define __LINUX_KEYSTONE_IPC_IRQ_H__

int __init keystone_ipc_irq_of_init(struct device_node *node,
				    struct device_node *parent);

#endif /* __LINUX_KEYSTONE_IPC_IRQ_H__ */
