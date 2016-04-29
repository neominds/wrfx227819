/*
 * Texas Instruments TCI6614 IRQ chip complex
 *
 * Copyright (C) 2012 Texas Instruments, Inc.
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

#ifndef __LINUX_TCI6614_IRQ_H__
#define __LINUX_TCI6614_IRQ_H__

#include <asm/exception.h>

extern int __init tci6614_of_init_irq(struct device_node *np,
				      struct device_node *parent);
extern asmlinkage void __exception_irq_entry
	tci6614_handle_irq(struct pt_regs *regs);

#endif /* __LINUX_TCI6614_IRQ_H__ */
