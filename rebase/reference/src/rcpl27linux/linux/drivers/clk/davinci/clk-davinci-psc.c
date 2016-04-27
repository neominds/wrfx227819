/*
 * PSC clk driver for DaVinci devices
 *
 * Copyright (C) 2006-2012 Texas Instruments.
 * Copyright (C) 2008-2009 Deep Root Systems, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/platform_data/clk-davinci-psc.h>
#include <linux/of_address.h>
#include <linux/of.h>
#include <linux/module.h>

/* PSC register offsets */
#define EPCPR		0x070
#define PTCMD		0x120
#define PTSTAT		0x128
#define PDSTAT		0x200
#define PDCTL		0x300
#define MDSTAT		0x800
#define MDCTL		0xA00

/* PSC module states */
#define PSC_STATE_SWRSTDISABLE	0
#define PSC_STATE_SYNCRST	1
#define PSC_STATE_DISABLE	2
#define PSC_STATE_ENABLE	3

#define MDSTAT_STATE_MASK	0x3f
#define PDSTAT_STATE_MASK	0x1f
#define MDCTL_FORCE		BIT(31)
#define MDCTL_LRESET		BIT(8)
#define PDCTL_NEXT		BIT(0)
#define PDCTL_EPCGOOD		BIT(8)

/* PSC flags */
#define PSC_SWRSTDISABLE	BIT(0) /* Disable state is SwRstDisable */
#define PSC_FORCE		BIT(1) /* Force module state transtition */
#define PSC_HAS_EXT_POWER_CNTL	BIT(2) /* PSC has external power control
					* available (for DM6446 SoC) */
#define PSC_LRESET		BIT(3) /* Keep module in local reset */
#define STATE_TRANS_MAX_COUNT	0xffff /* Maximum timeout to bail out state
					* transition for module */
/**
 * struct clk_psc - DaVinci PSC clock
 * @hw: clk_hw for the psc
 * @psc_data: PSC driver specific data
 * @lock: Spinlock used by the driver
 */
struct clk_psc {
	struct clk_hw hw;
	struct clk_davinci_psc_data *psc_data;
	spinlock_t *lock;
};

#define to_clk_psc(_hw) container_of(_hw, struct clk_psc, hw)

/* Enable or disable a PSC domain */
static void clk_psc_config(void __iomem *base, unsigned int domain,
		unsigned int id, bool enable, u32 flags)
{
	u32 epcpr, ptcmd, ptstat, pdstat, pdctl, mdstat, mdctl;
	u32 next_state = PSC_STATE_ENABLE;
	void __iomem *psc_base = base;
	u32 count = STATE_TRANS_MAX_COUNT;
	int err = 0;

	if (!enable) {
		if (flags & PSC_SWRSTDISABLE)
			next_state = PSC_STATE_SWRSTDISABLE;
		else
			next_state = PSC_STATE_DISABLE;
	}

	mdctl = __raw_readl(psc_base + MDCTL + 4 * id);
	mdctl &= ~MDSTAT_STATE_MASK;
	mdctl |= next_state;
	if (flags & PSC_FORCE)
		mdctl |= MDCTL_FORCE;

	if (flags & PSC_LRESET)
		mdctl &= ~MDCTL_LRESET;

	/* For disable, we always put the module in local reset */
	if (!enable)
		mdctl &= ~MDCTL_LRESET;

	__raw_writel(mdctl, psc_base + MDCTL + 4 * id);

	pdstat = __raw_readl(psc_base + PDSTAT + 4 * domain);
	if ((pdstat & PDSTAT_STATE_MASK) == 0) {
		pdctl = __raw_readl(psc_base + PDCTL + 4 * domain);
		pdctl |= PDCTL_NEXT;
		__raw_writel(pdctl, psc_base + PDCTL + 4 * domain);

		ptcmd = 1 << domain;
		__raw_writel(ptcmd, psc_base + PTCMD);

		if (flags & PSC_HAS_EXT_POWER_CNTL) {
			do {
				epcpr = __raw_readl(psc_base + EPCPR);
			} while ((((epcpr >> domain) & 1) == 0));
		}

		pdctl = __raw_readl(psc_base + PDCTL + 4 * domain);
		pdctl |= 0x100;
		__raw_writel(pdctl, psc_base + PDCTL + 4 * domain);

		pdctl = __raw_readl(psc_base + PDCTL + 4 * domain);
		pdctl |= PDCTL_EPCGOOD;
		__raw_writel(pdctl, psc_base + PDCTL + 4 * domain);
	} else {
		ptcmd = 1 << domain;
		__raw_writel(ptcmd, psc_base + PTCMD);
	}

	do {
		ptstat = __raw_readl(psc_base + PTSTAT);
	} while (!(((ptstat >> domain) & 1) == 0) && (count--));

	if (!count)
		goto err;

	count = STATE_TRANS_MAX_COUNT;
	do {
		mdstat = __raw_readl(psc_base + MDSTAT + 4 * id);
	} while (!((mdstat & MDSTAT_STATE_MASK) == next_state) && (count--));
err:
	return;
}

static int clk_psc_is_enabled(struct clk_hw *hw)
{
	struct clk_psc *psc = to_clk_psc(hw);
	struct clk_davinci_psc_data *psc_data = psc->psc_data;
	u32 mdstat;

	mdstat = __raw_readl(psc_data->base + MDSTAT + 4 * psc_data->lpsc);
	/* if clocked, state can be "Enable" or "SyncReset" */
	return (mdstat & BIT(12)) ? 1 : 0;
}

static int clk_psc_enable(struct clk_hw *hw)
{
	struct clk_psc *psc = to_clk_psc(hw);
	struct clk_davinci_psc_data *psc_data = psc->psc_data;
	unsigned long flags = 0;

	if (psc->lock)
		spin_lock_irqsave(psc->lock, flags);

	clk_psc_config(psc_data->base, psc_data->domain, psc_data->lpsc,
			1, psc_data->psc_flags);

	if (psc->lock)
		spin_unlock_irqrestore(psc->lock, flags);

	return 0;
}

static void clk_psc_disable(struct clk_hw *hw)
{
	struct clk_psc *psc = to_clk_psc(hw);
	struct clk_davinci_psc_data *psc_data = psc->psc_data;
	unsigned long flags = 0;

	if (psc->lock)
		spin_lock_irqsave(psc->lock, flags);

	clk_psc_config(psc_data->base, psc_data->domain, psc_data->lpsc,
			0, psc_data->psc_flags);

	if (psc->lock)
		spin_unlock_irqrestore(psc->lock, flags);
}

static const struct clk_ops clk_psc_ops = {
	.enable = clk_psc_enable,
	.disable = clk_psc_disable,
	.is_enabled = clk_psc_is_enabled,
};

/**
 * clk_register_davinci_psc - register davinci psc clock
 * @dev: device that is registering this clock
 * @name: name of this clock
 * @parent_name: name of clock's parent
 * @psc_data: platform data to configure this clock
 * @lock: spinlock used by this clock
 */
struct clk *clk_register_davinci_psc(struct device *dev, const char *name,
			const char *parent_name,
			struct clk_davinci_psc_data *psc_data,
			spinlock_t *lock)
{
	struct clk_init_data init;
	struct clk_psc *psc;
	struct clk *clk;

	psc = kzalloc(sizeof(*psc), GFP_KERNEL);
	if (!psc)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	init.ops = &clk_psc_ops;
	init.flags = psc_data->flags;
	init.parent_names = (parent_name ? &parent_name : NULL);
	init.num_parents = (parent_name ? 1 : 0);

	psc->psc_data = psc_data;
	psc->lock = lock;
	psc->hw.init = &init;

	clk = clk_register(NULL, &psc->hw);
	if (IS_ERR(clk))
		kfree(psc);

	return clk;
}
EXPORT_SYMBOL_GPL(clk_register_davinci_psc);

#ifdef CONFIG_OF
#define NUM_GPSC	2
struct reg_psc {
	u32 phy_base;
	u32 size;
	void __iomem *io_base;
};

static struct reg_psc psc_addr[NUM_GPSC];

/**
 * of_davinci_psc_clk_init - initialize davinci psc clock through DT
 * @node: device tree node for this clock
 * @lock: spinlock used by this clock
 */
void __init of_davinci_psc_clk_init(struct device_node *node, spinlock_t *lock)
{
	const char *parent_name, *status = NULL, *base_flags = NULL;
	struct clk_davinci_psc_data *data;
	const char *clk_name = node->name;
	u32 gpsc = 0, lpsc = 0, pd = 0;
	struct resource res;
	struct clk *clk;
	int rc;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	WARN_ON(!data);

	if (of_address_to_resource(node, 0, &res)) {
		pr_err("psc_clk_init - no reg property defined\n");
		goto out;
	}

	of_property_read_u32(node, "gpsc", &gpsc);
	of_property_read_u32(node, "lpsc", &lpsc);
	of_property_read_u32(node, "pd", &pd);

	if (gpsc >= NUM_GPSC) {
		pr_err("psc_clk_init - no reg property defined\n");
		goto out;
	}

	of_property_read_string(node,
			"clock-output-names", &clk_name);
	parent_name = of_clk_get_parent_name(node, 0);
	WARN_ON(!parent_name);

	/* Expected that same phy_base is used for all psc clocks of
	 * a give gpsc. So ioremap is done only once.
	 */
	if (psc_addr[gpsc].phy_base) {
		if (psc_addr[gpsc].phy_base != res.start) {
			pr_err("Different psc base for same GPSC\n");
			goto out;
		}
	} else {
		psc_addr[gpsc].phy_base = res.start;
		psc_addr[gpsc].io_base =
			ioremap(res.start, resource_size(&res));
	}

	WARN_ON(!psc_addr[gpsc].io_base);
	data->base = psc_addr[gpsc].io_base;
	data->lpsc = lpsc;
	data->gpsc = gpsc;
	data->domain = pd;

	of_property_read_string_index(node, "base-flags", 0, &base_flags);
	if (base_flags && !strcmp(base_flags, "ignore-unused"))
		data->flags = CLK_IGNORE_UNUSED;

	if (of_property_read_bool(node, "ti,psc-lreset"))
		data->psc_flags |= PSC_LRESET;
	if (of_property_read_bool(node, "ti,psc-force"))
		data->psc_flags |= PSC_FORCE;

	clk = clk_register_davinci_psc(NULL, clk_name, parent_name,
				data, lock);

	if (clk) {
		of_clk_add_provider(node, of_clk_src_simple_get, clk);

		rc = of_property_read_string(node, "status", &status);
		if (status && !strcmp(status, "enabled"))
			clk_prepare_enable(clk);
		return;
	}
	pr_err("psc_clk_init - error registering psc clk %s\n", node->name);
out:
	kfree(data);
	return;
}
EXPORT_SYMBOL_GPL(of_davinci_psc_clk_init);
#endif
