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

#ifndef __KEYSTONE_H__
#define __KEYSTONE_H__

extern struct smp_operations keystone_smp_ops;
extern void secondary_startup(void);

extern int __init keystone_pm_runtime_init(void);

extern int __init tci6614_timer_init(void);

#endif /* __KEYSTONE_H__ */
