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
#include <linux/clocksource.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/irqchip.h>
#include <linux/irqchip/tci6614.h>
#include <linux/irqchip/keystone-ipc.h>
#include <linux/platform_data/davinci-clock.h>

#include <asm/setup.h>
#include <asm/smp_plat.h>
#include <asm/mach/map.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/arch_timer.h>

#include "keystone.h"

#define RSTMUX8_OMODE_DEVICE_RESET		5
#define RSTMUX8_OMODE_DEVICE_RESET_SHIFT	1
#define RSTMUX8_OMODE_DEVICE_RESET_MASK		(BIT(1) | BIT(2) | BIT(3))
#define RSTMUX8_LOCK_MASK			BIT(0)

unsigned long __arch_dma_pfn_offset;

#ifdef CONFIG_ZONE_DMA
extern phys_addr_t arm_dma_limit;
extern unsigned long arm_dma_zone_size;
#endif

#define reg_dump(addr, mask) \
		pr_debug("reg %p has value %x\n", (void *)addr, \
				(__raw_readl(addr) & ~mask))

static inline void reg_rmw(u32 addr, u32 mask, u32 val)
{
	u32 read_data, data;
	read_data = __raw_readl(addr);
	data = (val & ~mask) | (read_data & mask);
	__raw_writel(data, addr);
}

#define PCIE_RC_MODE		(BIT(2))
#define PCIE_MODE_MASK		(BIT(1) | BIT(2))

static struct map_desc io_desc[] = {
	{
		.virtual        = 0xfe600000UL,
		.pfn            = __phys_to_pfn(0x02000000UL),
		.length         = 0x800000UL,
		.type           = MT_DEVICE
	},
};

static void __init keystone_map_io(void)
{
	iotable_init(io_desc, ARRAY_SIZE(io_desc));
}

static struct of_device_id keystone_dt_match_table[] __initdata = {
	{ .compatible = "simple-bus",},
	{ .compatible = "ti,davinci-aemif", },
	{}
};

struct serdes_config {
	u32 offset;
	u32 mask;
	u32 val;
};

static struct serdes_config keystone2_serdes_cfg[] = {
	{ 0x000, 0xffff00ff, 0x00000800 },
	{ 0x060, 0xff000000, 0x00041c5c },
	{ 0x064, 0x000000ff, 0x0343c700 },
	{ 0x06c, 0xffffff00, 0x00000012 },
	{ 0x068, 0xff00ffff, 0x00070000 },
	{ 0x078, 0xffff00ff, 0x0000c000 },
	{ 0x200, 0xffffff00, 0x00000000 },
	{ 0x204, 0x00ffff00, 0x5e000080 },
	{ 0x208, 0xffffff00, 0x00000006 },
	{ 0x210, 0xffffff00, 0x00000023 },
	{ 0x214, 0x00ff0000, 0x2e003060 },
	{ 0x218, 0x00ffffff, 0x76000000 },
	{ 0x22c, 0xff00ff00, 0x00100002 },
	{ 0x2a0, 0x0000ffff, 0xffee0000 },
	{ 0x2a4, 0xffffff00, 0x0000000f },
	{ 0x204, 0x00ffffff, 0x5e000000 },
	{ 0x208, 0xffffff00, 0x00000006 },
	{ 0x278, 0xffff00ff, 0x00002000 },
	{ 0x280, 0xff00ff00, 0x00280028 },
	{ 0x284, 0x00000000, 0x2d0f0385 },
	{ 0x250, 0x00ffffff, 0xd0000000 },
	{ 0x284, 0xffffff00, 0x00000085 },
	{ 0x294, 0x00ffffff, 0x20000000 },

	{ 0x400, 0xffffff00, 0x00000000 },
	{ 0x404, 0x00ffff00, 0x5e000080 },
	{ 0x408, 0xffffff00, 0x00000006 },
	{ 0x410, 0xffffff00, 0x00000023 },
	{ 0x414, 0x00ff0000, 0x2e003060 },
	{ 0x418, 0x00ffffff, 0x76000000 },
	{ 0x42c, 0xff00ff00, 0x00100002 },
	{ 0x4a0, 0x0000ffff, 0xffee0000 },
	{ 0x4a4, 0xffffff00, 0x0000000f },
	{ 0x404, 0x00ffffff, 0x5e000000 },
	{ 0x408, 0xffffff00, 0x00000006 },
	{ 0x478, 0xffff00ff, 0x00002000 },
	{ 0x480, 0xff00ff00, 0x00280028 },
	{ 0x484, 0x00000000, 0x2d0f0385 },
	{ 0x450, 0x00ffffff, 0xd0000000 },
	{ 0x494, 0x00ffffff, 0x20000000 },

	{ 0x604, 0xffffff00, 0x00000080 },
	{ 0x600, 0xffffff00, 0x00000000 },
	{ 0x604, 0x00ffffff, 0x5e000000 },
	{ 0x608, 0xffffff00, 0x00000006 },
	{ 0x610, 0xffffff00, 0x00000023 },
	{ 0x614, 0x00ff0000, 0x2e003060 },
	{ 0x618, 0x00ffffff, 0x76000000 },
	{ 0x62c, 0xff00ff00, 0x00100002 },
	{ 0x6a0, 0x0000ffff, 0xffee0000 },
	{ 0x6a4, 0xffffff00, 0x0000000f },
	{ 0x604, 0x00ffffff, 0x5e000000 },
	{ 0x608, 0xffffff00, 0x00000006 },
	{ 0x678, 0xffff00ff, 0x00002000 },
	{ 0x680, 0xff00ff00, 0x00280028 },
	{ 0x684, 0x00000000, 0x2d0f0385 },
	{ 0x650, 0x00ffffff, 0xd0000000 },
	{ 0x694, 0x00ffffff, 0x20000000 },

	{ 0x800, 0xffffff00, 0x00000000 },
	{ 0x804, 0x00ffff00, 0x5e000080 },
	{ 0x808, 0xffffff00, 0x00000006 },
	{ 0x810, 0xffffff00, 0x00000023 },
	{ 0x814, 0x00ff0000, 0x2e003060 },
	{ 0x818, 0x00ffffff, 0x76000000 },
	{ 0x82c, 0xff00ff00, 0x00100002 },
	{ 0x8a0, 0x0000ffff, 0xffee0000 },
	{ 0x8a4, 0xffffff00, 0x0000000f },
	{ 0x804, 0x00ffffff, 0x5e000000 },
	{ 0x808, 0xffffff00, 0x00000006 },
	{ 0x878, 0xffff00ff, 0x00002000 },
	{ 0x880, 0xff00ff00, 0x00280028 },
	{ 0x884, 0x00000000, 0x2d0f0385 },
	{ 0x850, 0x00ffffff, 0xd0000000 },
	{ 0x894, 0x00ffffff, 0x20000000 },

	{ 0xa00, 0xffff00ff, 0x00000100 },
	{ 0xa08, 0xff000000, 0x00e12c08 },
	{ 0xa0c, 0xffffff00, 0x00000081 },
	{ 0xa18, 0xff00ffff, 0x00e80000 },
	{ 0xa30, 0x00ffff00, 0x002f2f00 },
	{ 0xa48, 0xff0000ff, 0x00e3ce00 },
	{ 0xa4c, 0x0000ffff, 0xac820000 },
	{ 0xa54, 0x00ffffff, 0xc0000000 },
	{ 0xa58, 0xffff0000, 0x00001441 },
	{ 0xa84, 0xffff0000, 0x00000301 },

	{ 0xa8c, 0x0000ffff, 0x81030000 },
	{ 0xa90, 0xffff0000, 0x00006001 },
	{ 0xa94, 0x00ffffff, 0x01000000 },
	{ 0xaa0, 0x00ffffff, 0x81000000 },
	{ 0xabc, 0x00ffffff, 0xff000000 },
	{ 0xac0, 0xffffff00, 0x0000008b },

	{ 0x000, 0xffffff00, 0x00000003 },
	{ 0xa00, 0xffffff00, 0x0000009f },
};

static void __init keystone2_pcie_serdes_setup(struct device_node *np)
{
	void __iomem *reg_serdes_base, *devcfg;
	u32 val;
	int i;

	reg_serdes_base = of_iomap(np, 0);
	devcfg = of_iomap(np, 1);

	pr_info("keystone2_pcie_serdes_setup\n");
	if (!reg_serdes_base || !devcfg) {
		pr_warn("pcie device cfg bindings missing\n");
		return;
	}

	for (i = 0; i < ARRAY_SIZE(keystone2_serdes_cfg); i++) {
		reg_rmw(((u32)reg_serdes_base+keystone2_serdes_cfg[i].offset),
			keystone2_serdes_cfg[i].mask,
			keystone2_serdes_cfg[i].val);
		reg_dump(((u32)reg_serdes_base+keystone2_serdes_cfg[i].offset),
			keystone2_serdes_cfg[i].mask);
	}

	/*
	 * Delay for minimum of 3000 serdes input clock cycles = 3000 * 10
	 * nsec at 100MHz
	 */
	udelay(2000);

	/* enable RC mode in devcfg */
	val = __raw_readl(devcfg);
	val &= ~PCIE_MODE_MASK;
	val |= PCIE_RC_MODE;
	__raw_writel(val, devcfg);

	pr_info("keystone2_pcie_serdes_setup done\n");
}

static const struct of_device_id keystone2_pcie_serdes_match[] = {
	{
		.compatible = "ti,keystone2-pcie-cfg",
		.data = keystone2_pcie_serdes_setup,
	},
	{
		/* sentinel */
	}
};

static void __init keystone_init_irq(void)
{
	irqchip_init();
}


static void __init keystone_timer_init(void)
{
	int error;

	davinci_of_clk_init();

	error = tci6614_timer_init();
	if (!error)
		return;

	clocksource_of_init();
}

static bool is_coherent(struct device *dev)
{
	struct device_node *node = of_node_get(dev->of_node);

	while (node) {
		if (of_property_read_bool(node, "dma-coherent")) {
			of_node_put(node);
			return true;
		}
		node = of_get_next_parent(node);
	}
	return false;
}

static int keystone_platform_notifier(struct notifier_block *nb,
				      unsigned long event, void *_dev)
{
	struct device *dev = _dev;

	if (event == BUS_NOTIFY_ADD_DEVICE) {
		dev->dma_mask = kmalloc(sizeof(*dev->dma_mask), GFP_KERNEL);
		dev->coherent_dma_mask = arm_dma_limit;
		if (dev->dma_mask)
			*dev->dma_mask = arm_dma_limit;
		if (is_coherent(dev))
			set_dma_ops(dev, &arm_coherent_dma_ops);
		return NOTIFY_OK;
	} else if (event == BUS_NOTIFY_DEL_DEVICE) {
		kfree(dev->dma_mask);
		return NOTIFY_OK;
	}
	return NOTIFY_DONE;
}

static struct notifier_block keystone_platform_nb;

static void __init keystone_init(void)
{
	struct device_node *node;
	const struct of_device_id *of_id;
	void (*func)(struct device_node *);

	if (keystone_platform_nb.notifier_call)
		bus_register_notifier(&platform_bus_type, &keystone_platform_nb);

	keystone_pm_runtime_init();
	of_platform_populate(NULL, keystone_dt_match_table, NULL, NULL);

	/* Initialize PCIE serdes */
	node = of_find_matching_node(NULL, keystone2_pcie_serdes_match);
	if (node) {
		of_id = of_match_node(keystone2_pcie_serdes_match, node);
		func = of_id->data;
		if (func)
			func(node);
		of_node_put(node);
	}
}

static int __init keystone_wd_rstmux_init(void)
{
	struct device_node *node;
	void __iomem *rstmux8;
	u32 val;

	/*
	 * For WD reset to function, rstmux8 should be configured
	 * so that this will trigger a device reset.
	 */
	node = of_find_compatible_node(NULL, NULL, "ti,keystone-reset");
	if (!node) {
		pr_warn("ti, keystone-reset node undefined\n");
		return -EINVAL;
	}

	/* rstmux8 address is configured in the rstctrl node at index 1 */
	rstmux8 = of_iomap(node, 1);
	if (WARN_ON(!rstmux8)) {
		pr_warn("rstmux8 iomap error\n");
		return -ENODEV;
	}

	val = __raw_readl(rstmux8) & ~RSTMUX8_OMODE_DEVICE_RESET_MASK;
	if (!(val & RSTMUX8_LOCK_MASK)) {
		val |= (RSTMUX8_OMODE_DEVICE_RESET <<
				RSTMUX8_OMODE_DEVICE_RESET_SHIFT);
		__raw_writel(val, rstmux8);
	}
	iounmap(rstmux8);
	return 0;
}
postcore_initcall(keystone_wd_rstmux_init);

static const char *keystone1_match[] __initconst = {
	"ti,tci6614-evm",
	NULL,
};

static const char *keystone2_match[] __initconst = {
	"ti,keystone-evm",
	"ti,tci6638-evm",
	NULL,
};

static void __init keystone_init_meminfo(void)
{
	bool lpae = IS_ENABLED(CONFIG_ARM_LPAE);
	bool pvpatch = IS_ENABLED(CONFIG_ARM_PATCH_PHYS_VIRT);
	phys_addr_t offset = PHYS_OFFSET - KEYSTONE_LOW_PHYS_START;
	phys_addr_t mem_start, mem_end;

	BUG_ON(meminfo.nr_banks < 1);
	mem_start = meminfo.bank[0].start;
	mem_end = mem_start + meminfo.bank[0].size - 1;

	/* nothing to do if we are running out of the <32-bit space */
	if (mem_start >= KEYSTONE_LOW_PHYS_START &&
	    mem_end   <= KEYSTONE_LOW_PHYS_END)
		return;

	if (!lpae || !pvpatch) {
		panic("Enable %s%s%s to run outside 32-bit space\n",
		       !lpae ? __stringify(CONFIG_ARM_LPAE) : "",
		       (!lpae && !pvpatch) ? " and " : "",
		       !pvpatch ? __stringify(CONFIG_ARM_PATCH_PHYS_VIRT) : "");
	}

	if (mem_start < KEYSTONE_HIGH_PHYS_START ||
	    mem_end   > KEYSTONE_HIGH_PHYS_END) {
		panic("Invalid address space for memory (%08llx-%08llx)\n",
		      (u64)mem_start, (u64)mem_end);
	}

	offset += KEYSTONE_HIGH_PHYS_START;
	pr_info("switching to high address space at 0x%llx\n", (u64)offset);
	__pv_phys_offset = offset;
	__pv_offset	= offset - PAGE_OFFSET;

	__arch_dma_pfn_offset = PFN_DOWN(KEYSTONE_HIGH_PHYS_START -
					 KEYSTONE_LOW_PHYS_START);
#ifdef CONFIG_ZONE_DMA
	arm_dma_limit = __pv_phys_offset + arm_dma_zone_size - 1;
	keystone_platform_nb.notifier_call = keystone_platform_notifier;
#endif
}

void keystone_restart(char mode, const char *cmd)
{
	struct device_node *node;
	void __iomem *rstctrl;
	u32 val;

	node = of_find_compatible_node(NULL, NULL, "ti,keystone-reset");
	if (WARN_ON(!node)) {
		pr_warn("ti, keystone-reset node undefined\n");
		return;
	}

	rstctrl = of_iomap(node, 0);
	if (WARN_ON(!rstctrl)) {
		pr_warn("ti, pllctrl-reset iomap error\n");
		return;
	}

	val = __raw_readl(rstctrl);
	val &= 0xffff0000;
	val |= 0x5a69;
	__raw_writel(val, rstctrl);

	val = __raw_readl(rstctrl);
	val &= 0xfffe0000;
	__raw_writel(val, rstctrl);
}

#define KEYSTONE_MACHINE_DEFS				\
	.map_io		= keystone_map_io,		\
	.init_irq	= keystone_init_irq,		\
	.init_time	= &keystone_timer_init,		\
	.init_machine	= keystone_init,		\
	.init_meminfo	= keystone_init_meminfo,	\
	.restart	= keystone_restart,

DT_MACHINE_START(KEYSTONE1, "KeyStone1")
	KEYSTONE_MACHINE_DEFS
	.dt_compat	= keystone1_match,
MACHINE_END

DT_MACHINE_START(KEYSTONE2, "KeyStone2")
	KEYSTONE_MACHINE_DEFS
	.smp		= smp_ops(keystone_smp_ops),
	.dt_compat	= keystone2_match,
#ifdef CONFIG_ZONE_DMA
	.dma_zone_size  = SZ_2G,
#endif
MACHINE_END
