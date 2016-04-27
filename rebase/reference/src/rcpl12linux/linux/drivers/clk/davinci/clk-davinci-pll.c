/*
 * PLL clk driver DaVinci devices
 *
 * Copyright (C) 2006-2012 Texas Instruments.
 * Copyright (C) 2008-2009 Deep Root Systems, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * TODO - Add set_parent_rate()
 */
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/platform_data/clk-davinci-pll.h>

#include <mach/cputype.h>

#define PLLM		0x110
#define PLLM_PLLM_MASK  0xff
#define PREDIV          0x114
#define POSTDIV         0x128
#define PLLDIV_EN       BIT(15)

/**
 * struct clk_davinci_pll - DaVinci Main pll clock
 * @hw: clk_hw for the pll
 * @pll_data: PLL driver specific data
 */
struct clk_davinci_pll {
	struct clk_hw hw;
	struct clk_davinci_pll_data *pll_data;
};

#define to_clk_pll(_hw) container_of(_hw, struct clk_davinci_pll, hw)

static unsigned long clk_pllclk_recalc(struct clk_hw *hw,
					unsigned long parent_rate)
{
	struct clk_davinci_pll *pll = to_clk_pll(hw);
	struct clk_davinci_pll_data *pll_data = pll->pll_data;
	u32 mult = 1, prediv = 1, postdiv = 1;
	unsigned long rate = parent_rate;

	/* If there is a device specific recalc defined invoke it. Otherwise
	 * fallback to default one
	 */
	mult = __raw_readl(pll_data->pllm);
	if (pll_data->pllm_multiplier)
		mult =  pll_data->pllm_multiplier *
				(mult & pll_data->pllm_mask);
	else
		mult = (mult & pll_data->pllm_mask) + 1;

	if (pll_data->flags & CLK_DAVINCI_PLL_HAS_PREDIV) {
		/* pre-divider is fixed, take prediv value from pll_data  */
		if (pll_data->fixed_prediv)
			prediv = pll_data->fixed_prediv;
		else {
			prediv = __raw_readl(pll_data->prediv);
			if (prediv & PLLDIV_EN)
				prediv = (prediv & pll_data->prediv_mask) + 1;
			else
				prediv = 1;
		}
	}

	if (pll_data->flags & CLK_DAVINCI_PLL_HAS_POSTDIV) {
		postdiv = __raw_readl(pll_data->postdiv);
		if (postdiv & PLLDIV_EN)
			postdiv = (postdiv & pll_data->postdiv_mask) + 1;
		else
			postdiv = 1;
	}

	rate /= prediv;
	rate *= mult;
	rate /= postdiv;

	pr_debug("PLL%d: input = %lu MHz [ ",
		 pll_data->num, parent_rate / 1000000);
	if (prediv > 1)
		pr_debug("/ %d ", prediv);
	if (mult > 1)
		pr_debug("* %d ", mult);
	if (postdiv > 1)
		pr_debug("/ %d ", postdiv);
	pr_debug("] --> %lu MHz output.\n", rate / 1000000);
	return rate;
}

static const struct clk_ops clk_pll_ops = {
	.recalc_rate = clk_pllclk_recalc,
};

struct clk *clk_register_davinci_pll(struct device *dev, const char *name,
			const char *parent_name,
			struct clk_davinci_pll_data *pll_data)
{
	struct clk_init_data init;
	struct clk_davinci_pll *pll;
	struct clk *clk;

	if (!pll_data)
		return ERR_PTR(-ENODEV);

	pll = kzalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return ERR_PTR(-ENOMEM);
	init.name = name;
	init.ops = &clk_pll_ops;
	init.flags = pll_data->flags;
	init.parent_names = (parent_name ? &parent_name : NULL);
	init.num_parents = (parent_name ? 1 : 0);

	pll->pll_data	= pll_data;
	pll->hw.init = &init;

	clk = clk_register(NULL, &pll->hw);
	if (IS_ERR(clk))
		kfree(pll);

	return clk;
}
