/*
 * Main PLL clk driver for Keystone devices
 *
 * Copyright (C) 2012 Texas Instruments.
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
#include <linux/of_address.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform_data/clk-keystone-pll.h>

/**
 * struct clk_pll - DaVinci Main pll clock
 * @hw: clk_hw for the pll
 * @pll_data: PLL driver specific data
 */
struct clk_pll {
	struct clk_hw hw;
	struct clk_keystone_pll_data *pll_data;
};

#define to_clk_pll(_hw) container_of(_hw, struct clk_pll, hw)

static unsigned long clk_pllclk_recalc(struct clk_hw *hw,
					unsigned long parent_rate)
{
	struct clk_pll *pll = to_clk_pll(hw);
	struct clk_keystone_pll_data *pll_data = pll->pll_data;
	unsigned long rate = parent_rate;
	u32  mult = 0, prediv, postdiv, val;

	/*
	 * get bits 0-5 of multiplier from pllctrl PLLM register
	 * if has_pllctrl is non zero */
	if (pll_data->has_pllctrl) {
		val = __raw_readl(pll_data->pllm);
		mult = (val & pll_data->pllm_lower_mask);
	}

	/* bit6-12 of PLLM is in Main PLL control register */
	val = __raw_readl(pll_data->pll_ctl0);
	mult |= ((val & pll_data->pllm_upper_mask)
			>> pll_data->pllm_upper_shift);
	prediv = (val & pll_data->plld_mask);
	postdiv = pll_data->fixed_postdiv;

	rate /= (prediv + 1);
	rate = (rate * (mult + 1));
	rate /= postdiv;

	if (pll_data->has_pllctrl) {
		pr_notice("main_pll_clk rate is %ld, postdiv = %d, mult = %d," \
				"prediv = %d\n", rate, postdiv, mult, prediv);
	} else {
		pr_notice("pll_clk parent_rate(%ld Hz), rate(%ld Hz)," \
				"postdiv = %d, mult = %d, prediv = %d\n",
				parent_rate, rate, postdiv, mult, prediv);
	}
	return rate;
}

static const struct clk_ops clk_pll_ops = {
	.recalc_rate = clk_pllclk_recalc,
};

struct clk *clk_register_keystone_pll(struct device *dev, const char *name,
			const char *parent_name,
			struct clk_keystone_pll_data *pll_data)
{
	struct clk_init_data init;
	struct clk_pll *pll;
	struct clk *clk;

	if (!pll_data)
		return ERR_PTR(-ENODEV);

	pll = kzalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	init.ops = &clk_pll_ops;
	init.flags = 0;
	init.parent_names = (parent_name ? &parent_name : NULL);
	init.num_parents = (parent_name ? 1 : 0);

	pll->pll_data	= pll_data;
	pll->hw.init = &init;

	clk = clk_register(NULL, &pll->hw);
	if (IS_ERR(clk))
		goto out;

	return clk;
out:
	kfree(pll);
	return NULL;
}
EXPORT_SYMBOL_GPL(clk_register_keystone_pll);

#ifdef CONFIG_OF
void __init of_keystone_pll_clk_init(struct device_node *node)
{
	struct clk_keystone_pll_data *pll_data;
	const char *parent_name;
	struct clk *clk;
	int temp;

	pll_data = kzalloc(sizeof(*pll_data), GFP_KERNEL);
	WARN_ON(!pll_data);

	parent_name = of_clk_get_parent_name(node, 0);

	if (of_find_property(node, "pll_has_pllctrl", &temp)) {
		/* PLL is controlled by the pllctrl */
		pll_data->has_pllctrl = 1;
		pll_data->pllm = of_iomap(node, 0);
		WARN_ON(!pll_data->pllm);

		pll_data->pll_ctl0 = of_iomap(node, 1);
		WARN_ON(!pll_data->pll_ctl0);

		if (of_property_read_u32(node, "pllm_lower_mask",
			&pll_data->pllm_lower_mask))
			goto out;

	} else {
		/* PLL is controlled by the ctrl register */
		pll_data->has_pllctrl = 0;
		pll_data->pll_ctl0 = of_iomap(node, 0);
	}

	if (of_property_read_u32(node, "pllm_upper_mask",
			&pll_data->pllm_upper_mask))
		goto out;

	if (of_property_read_u32(node, "pllm_upper_shift",
			&pll_data->pllm_upper_shift))
		goto out;

	if (of_property_read_u32(node, "plld_mask", &pll_data->plld_mask))
		goto out;

	if (of_property_read_u32(node, "fixed_postdiv",
					&pll_data->fixed_postdiv))
		goto out;

	clk = clk_register_keystone_pll(NULL, node->name, parent_name,
					 pll_data);
	if (clk) {
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
		return;
	}
out:
	pr_err("of_keystone_pll_clk_init - error initializing clk %s\n",
		 node->name);
	kfree(pll_data);
}
EXPORT_SYMBOL_GPL(of_keystone_pll_clk_init);
#endif
