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
#ifndef __ASM_MACH_MEMORY_H
#define __ASM_MACH_MEMORY_H

#define MAX_PHYSMEM_BITS	36
#define SECTION_SIZE_BITS	34

#define KEYSTONE_LOW_PHYS_START		0x80000000ULL
#define KEYSTONE_LOW_PHYS_SIZE		0x80000000ULL /* 2G */
#define KEYSTONE_LOW_PHYS_END		(KEYSTONE_LOW_PHYS_START + \
					 KEYSTONE_LOW_PHYS_SIZE - 1)

#define KEYSTONE_HIGH_PHYS_START	0x800000000ULL
#define KEYSTONE_HIGH_PHYS_SIZE		0x400000000ULL	/* 16G */
#define KEYSTONE_HIGH_PHYS_END		(KEYSTONE_HIGH_PHYS_START + \
					 KEYSTONE_HIGH_PHYS_SIZE - 1)
#ifdef CONFIG_ARM_LPAE

#ifndef __ASSEMBLY__

static inline phys_addr_t __virt_to_idmap(unsigned long x)
{
	return (phys_addr_t)(x) - CONFIG_PAGE_OFFSET +
		KEYSTONE_LOW_PHYS_START;
}

#define virt_to_idmap(x)	__virt_to_idmap((unsigned long)(x))

extern unsigned long __arch_dma_pfn_offset;

#define __arch_pfn_to_dma(dev, pfn)				\
	PFN_PHYS(pfn - __arch_dma_pfn_offset)
#define __arch_dma_to_pfn(dev, addr)				\
	(PFN_DOWN(addr) + __arch_dma_pfn_offset)
#define __arch_dma_to_virt(dev, addr)				\
	phys_to_virt(addr + PFN_PHYS(__arch_dma_pfn_offset))
#define __arch_virt_to_dma(dev, addr)				\
	(virt_to_phys(addr) - PFN_PHYS(__arch_dma_pfn_offset))

#endif /* __ASSEMBLY__ */

#endif /* CONFIG_ARM_LPAE */

#endif /* __ASM_MACH_MEMORY_H */
