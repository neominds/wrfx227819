/*
 * Debugging macro include header
 *
 * Copyright 2010-2012 Texas Instruments, Inc.
 * Copyright (C) 1994-1999 Russell King
 * Moved from linux/arch/arm/kernel/debug.S by Ben Dooks
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/serial_reg.h>

#include <mach/serial.h>

#define UART_SHIFT	2

	.macro	addruart,rp,rv,tmp
	ldr	\rv, =UART_VIRT
	ldr	\rp, =UART_PHYS
	.endm

	.macro	senduart,rd,rx
	str	\rd, [\rx, #UART_TX << UART_SHIFT]
	.endm

	.macro	busyuart,rd,rx
1002:	ldr	\rd, [\rx, #UART_LSR << UART_SHIFT]
	and	\rd, \rd, #UART_LSR_TEMT | UART_LSR_THRE
	teq	\rd, #UART_LSR_TEMT | UART_LSR_THRE
	bne	1002b
	.endm

	.macro	waituart,rd,rx
	.endm
