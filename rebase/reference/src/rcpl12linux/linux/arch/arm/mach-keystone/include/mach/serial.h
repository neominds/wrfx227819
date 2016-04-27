/*
 * Serial port register definitions
 *
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

#ifndef __MACH_KEYSTONE_SERIAL_H__
#define __MACH_KEYSTONE_SERIAL_H__

#if defined(CONFIG_DEBUG_DAVINCI_TCI6614_UART0)

#define UART_PHYS	0x02540000
#define UART_VIRT	0xfeb40000

#elif defined(CONFIG_DEBUG_DAVINCI_TCI6614_UART1)

#define UART_PHYS	0x02541000
#define UART_VIRT	0xfeb41000

#elif defined(CONFIG_DEBUG_DAVINCI_TCI6638_UART0)

#define UART_PHYS	0x02530c00
#define UART_VIRT	0xfeb30c00

#elif defined(CONFIG_DEBUG_DAVINCI_TCI6638_UART1)

#define UART_PHYS	0x02531000
#define UART_VIRT	0xfeb31000

#elif defined(CONFIG_DEBUG_EARLY_PRINTK)

#warn unsupported early debug configuration

#endif

#endif
