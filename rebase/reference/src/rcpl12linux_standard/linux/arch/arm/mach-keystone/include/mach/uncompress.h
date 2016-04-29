/*
 * Copyright 2010-2012 Texas Instruments, Inc.
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
#ifndef __MACH_KEYSTONE_UNCOMPRESS_H__
#define __MACH_KEYSTONE_UNCOMPRESS_H__

#include <linux/types.h>
#include <linux/serial_reg.h>

#include <mach/serial.h>

static void putc(char c)
{
#ifdef UART_PHYS
	u32 *uart = (u32 *)UART_PHYS;

	while (!(uart[UART_LSR] & UART_LSR_THRE))
		barrier();
	uart[UART_TX] = c;
#endif
}

static inline void flush(void)
{
#ifdef UART_PHYS
	u32 *uart = (u32 *)UART_PHYS;

	while (!(uart[UART_LSR] & UART_LSR_THRE))
		barrier();
#endif
}

#define arch_decomp_setup()
#define arch_decomp_wdog()

#endif
