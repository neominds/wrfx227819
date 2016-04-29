/*
 * Clock initialization code for DaVinci devices
 *
 * Copyright (C) 2006-2012 Texas Instruments.
 * Copyright (C) 2008-2009 Deep Root Systems, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/init.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/platform_data/clk-davinci-pll.h>
#include <linux/platform_data/clk-keystone-pll.h>
#include <linux/platform_data/clk-davinci-psc.h>
#include <linux/platform_data/davinci-clock.h>
#include <linux/of.h>

static DEFINE_SPINLOCK(_lock);

#ifndef CONFIG_OF
struct clk *davinci_lookup_clk(struct davinci_clk_lookup *clocks,
				const char *con_id)
{
	struct davinci_clk_lookup *c;

	for (c = clocks; c->_clk; c++) {
		if (c->con_id && !strcmp(c->con_id, con_id))
			return c->clk;
	}
	return NULL;
}

#ifdef	CONFIG_CLK_DAVINCI_PLL
static void register_davinci_pll_clk(struct davinci_clk_lookup *c,
			struct clk_davinci_pll_data *pll_data)
{

	WARN_ON(!pll_data->phy_pllm);
	pll_data->pllm = ioremap(pll_data->phy_pllm, 4);
	WARN_ON(!pll_data->pllm);
	if (pll_data->phy_prediv) {
		pll_data->prediv = ioremap(pll_data->phy_prediv, 4);
		WARN_ON(!pll_data->prediv);
	}
	if (pll_data->phy_postdiv) {
		pll_data->postdiv = ioremap(pll_data->phy_postdiv, 4);
		WARN_ON(!pll_data->postdiv);
	}
	c->clk = clk_register_davinci_pll(NULL,
			c->_clk->name, c->_clk->parent->name,
			pll_data);
}
#else
static void register_davinci_pll_clk(struct davinci_clk_lookup *c,
			struct clk_davinci_pll_data *pll_data)
{
	return;
}
#endif

#ifdef	CONFIG_CLK_KEYSTONE_PLL
static void register_keystone_pll_clk(struct davinci_clk_lookup *c,
			struct clk_keystone_pll_data *pll_data)
{
	WARN_ON(!pll_data->phy_pllm);
	pll_data->pllm = ioremap(pll_data->phy_pllm, 4);
	WARN_ON(!pll_data->pllm);
	WARN_ON(!pll_data->phy_main_pll_ctl0);
	pll_data->main_pll_ctl0 =
		ioremap(pll_data->phy_main_pll_ctl0, 4);
	WARN_ON(!pll_data->main_pll_ctl0);
	c->clk = clk_register_keystone_pll(NULL,
			c->_clk->name, c->_clk->parent->name,
			 pll_data);
}
#else
static void register_keystone_pll_clk(struct davinci_clk_lookup *c,
			struct clk_keystone_pll_data *pll_data)
{
	return;
}
#endif

int __init davinci_common_clk_init(struct davinci_clk_lookup *clocks,
				struct davinci_dev_lookup *dev_clk_lookups,
				u8 num_gpscs, u32 *psc_bases)
{
	void __iomem **base = NULL, *reg;
	struct davinci_clk_lookup *c;
	struct davinci_clk *_clk;
	unsigned long rate;
	int i, skip;

	WARN_ON(!num_gpscs);
	WARN_ON(psc_bases == NULL);

	base = kzalloc(sizeof(void __iomem *) * num_gpscs, GFP_KERNEL);
	WARN_ON(!base);
	for (i = 0; i < num_gpscs; i++) {
		base[i] = ioremap(psc_bases[i], SZ_4K);
		WARN_ON(!base[i]);
	}

	for (c = clocks; c->_clk; c++) {
		skip = 0;
		_clk = c->_clk;

		WARN_ON(!_clk->clk_data.data);
		switch (_clk->type) {
		case DAVINCI_FIXED_RATE_CLK:
		{
			struct clk_fixed_rate_data *data = _clk->clk_data.fixed_rate;

			if (data->recalc)
				rate = data->recalc(0);
			else
				rate = data->rate;

			c->clk = clk_register_fixed_rate(NULL, _clk->name,
					NULL, data->flags, rate);
			break;
		}
		case KEYSTONE_MAIN_PLL_CLK:
		{
			struct clk_keystone_pll_data *data =
					_clk->clk_data.keystone_pll;

			register_keystone_pll_clk(c, data);
			break;
		}
		case DAVINCI_MAIN_PLL_CLK:
		{
			struct clk_davinci_pll_data *data =
					_clk->clk_data.davinci_pll;

			register_davinci_pll_clk(c, data);
			break;
		}
		case DAVINCI_PRG_DIV_CLK:
		{
			struct clk_divider_data *data = _clk->clk_data.pll_div;

			/* This is a PLL derived clock with divider specified by
			 * div_reg in pll_div_data.
			 */
			reg = ioremap(data->div_reg, 4);
			WARN_ON(!reg);
			c->clk = clk_register_divider(NULL, _clk->name,
					_clk->parent->name, data->flags,
					reg, data->shift, data->width,
					data->divider_flags, &_lock);
			break;
		}
		case DAVINCI_PSC_CLK:
		{
			struct clk_davinci_psc_data *data = _clk->clk_data.psc;

			WARN_ON(!base);
			WARN_ON(data->gpsc >= num_gpscs);
			data->base = base[data->gpsc];
			c->clk = clk_register_davinci_psc(NULL, _clk->name,
					_clk->parent->name, data, &_lock);
			break;
		}
		case DAVINCI_MUX_CLK:
		{
			struct clk_mux_data *data = _clk->clk_data.mux;

			WARN_ON(!data->phys_base);
			reg = ioremap(data->phys_base, 4);
			WARN_ON(!reg);
			c->clk = clk_register_mux(NULL, _clk->name,
					data->parents, data->num_parents,
					data->flags, reg, data->shift,
					data->width, data->mux_flags, &_lock);
			break;
		}
		case DAVINCI_FIXED_FACTOR_CLK:
		{
			struct clk_fixed_factor_data *data
						 = _clk->clk_data.fixed_factor;

			WARN_ON(!data->mult);
			WARN_ON(!data->div);
			c->clk = clk_register_fixed_factor(NULL, _clk->name,
					_clk->parent->name, data->flags,
					data->mult, data->div);
			break;
		}
		default:
			skip = 1;
			pr_warn("Unknown clock - %s\n", _clk->name);
		}

		if (!skip) {
			WARN_ON(!c->clk);
			if (clk_register_clkdev(c->clk,
					c->con_id, c->dev_id) < 0) {
				pr_err("Error in registering clkdev, %s\n",
					_clk->name);
			}
			/* Enable ALWAYS_ENABLED clocks */
			if (_clk->flags & ALWAYS_ENABLED)
				clk_prepare_enable(c->clk);
		}
	}

	if (dev_clk_lookups) {
		struct davinci_dev_lookup *c;
		struct clk *clk;

		for (c = dev_clk_lookups; c->con_id; c++) {
			/* register the clock lookup table */
			clk = davinci_lookup_clk(clocks, c->con_id);
			WARN_ON(!clk);

			for (i = 0; i < c->num_devs; i++) {
				if (clk_register_clkdev(clk,
					c->lookups[i].con_id,
					c->lookups[i].dev_id) < 0) {
					pr_err("Error in registering clkdev" \
						" for dev_id %s\n",
						c->lookups[i].dev_id);
				}
			}
		}
	}
	return 0;
}
#else

static void __init pll_div_clk_init(struct device_node *node)
{
	const char *parent_name;
	void __iomem *reg;
	u32 shift, width;
	struct clk *clk;

	reg = of_iomap(node, 0);
	WARN_ON(!reg);

	parent_name = of_clk_get_parent_name(node, 0);
	WARN_ON(!parent_name);

	if (of_property_read_u32(node, "shift", &shift)) {
		pr_err("pll_div_clk_init - no shift value defined\n");
		return;
	}

	if (of_property_read_u32(node, "width", &width)) {
		pr_err("pll_div_clk_init - no width value defined\n");
		return;
	}

	clk = clk_register_divider(NULL, node->name, parent_name, 0, reg, shift,
				 width, 0, &_lock);

	if (clk)
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
	else
		pr_err("pll_div_clk_init - error in registering mux clk %s\n",
			node->name);
}

/* These will go away once there is support in clk-mux for DT */
void __init of_davinci_fixed_factor_clk_setup(struct device_node *node)
{
	const char *parent_name;
	struct clk *clk;
	u32 mult, div;

	parent_name = of_clk_get_parent_name(node, 0);
	WARN_ON(!parent_name);

	if (of_property_read_u32(node, "mult", &mult)) {
		pr_err("of_davinci_fixed_factor_clk_setup - no mult value defined\n");
		return;
	}

	if (of_property_read_u32(node, "div", &div)) {
		pr_err("of_davinci_fixed_factor_clk_setup - no div value defined\n");
		return;
	}

	WARN_ON(!mult);
	WARN_ON(!div);
	clk = clk_register_fixed_factor(NULL, node->name, parent_name, 0,
					mult, div);
	if (clk)
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
	else
		pr_err("error registering fixed factor clk %s\n", node->name);
}

/* These will go away once there is support in clk-mux for DT */
static void __init main_pll_mux_clk_init(struct device_node *node)
{
	const char *parents[2];
	void __iomem *reg;
	u32 shift, width;
	struct clk *clk;

	reg = of_iomap(node, 0);
	WARN_ON(!reg);

	parents[0] = of_clk_get_parent_name(node, 0);
	WARN_ON(!parents[0]);
	parents[1] = of_clk_get_parent_name(node, 1);
	WARN_ON(!parents[1]);

	if (of_property_read_u32(node, "shift", &shift)) {
		pr_err("main_pll_mux_clk_init - no shift value defined\n");
		return;
	}

	if (of_property_read_u32(node, "width", &width)) {
		pr_err("main_pll_mux_clk_init - no width value defined\n");
		return;
	}

	clk = clk_register_mux(NULL, node->name, (const char **)&parents,
				ARRAY_SIZE(parents) , 0, reg, shift, width,
				0, &_lock);

	if (clk)
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
	else
		pr_err("main_pll_mux_clk_init - error registe mux clk %s\n",
			 node->name);
}

static void __init of_davinci_psc_clk_setup(struct device_node *node)
{
	of_davinci_psc_clk_init(node, &_lock);
}

static const __initconst struct of_device_id clk_match[] = {
	{ .compatible = "fixed-clock", .data = of_fixed_clk_setup, },
	{ .compatible = "fixed-clock-factor", .data =
					 of_davinci_fixed_factor_clk_setup, },
	{ .compatible = "keystone,main-pll-clk", .data =
					of_keystone_pll_clk_init, },
	{ .compatible = "davinci,main-pll-mux-clk", .data =
						main_pll_mux_clk_init, },
	{ .compatible = "davinci,pll-divider-clk", .data = pll_div_clk_init, },
	{ .compatible = "davinci,psc-clk", .data = of_davinci_psc_clk_setup, },
	{}
};

static const __initconst struct of_device_id clk_psc_match[] = {
	{ .compatible = "davinci,psc-clk", .data = of_davinci_psc_clk_setup, },
	{}
};

void __init davinci_of_clk_init()
{
	/* initialize clk providers from device tree */
	of_clk_init(clk_match);
}

void __init davinci_add_clkdev(struct davinci_clk_lookup *clocks)
{
	struct device_node *np;

	/*
	 * When drivers use device nodes for provider clock info, this will
	 * become obsolete. Until then we need a way to register clk devices.
	 * for dt incompatible device drivers.
	 */
	if (clocks == NULL)
		return;

	for_each_matching_node(np, clk_psc_match) {
		struct davinci_clk_lookup *c;
		struct davinci_clk *_clk;
		const char *clk_name;
		struct clk *clk;

		clk_name = np->name;
		of_property_read_string(np,
			"clock-output-names", &clk_name);
		/* get clk through provider */
		clk = of_clk_get(np, 0);
		if (clk) {
			for (c = clocks; c->_clk; c++) {
				_clk = c->_clk;
				if (!strcmp(_clk->name, clk_name) &&
					(clk_register_clkdev(clk, c->con_id,
						 c->dev_id) < 0)) {
					pr_err("Error register clkdev,%s\n",
						clk_name);
					break;
				}
			}
		} else {
			pr_err("Error getting clk provider info, %s\n",
				clk_name);
		}
	}
}
#endif
