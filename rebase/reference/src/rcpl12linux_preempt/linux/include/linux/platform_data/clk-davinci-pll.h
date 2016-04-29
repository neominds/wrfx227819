/*
 * TI DaVinci clk-pll driver platform data definitions
 *
 * Copyright (C) 2006-2012 Texas Instruments.
 * Copyright (C) 2008-2009 Deep Root Systems, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __CLK_DAVINCI_PLL_H
#define __CLK_DAVINCI_PLL_H

/* PLL flags */
#define CLK_DAVINCI_PLL_HAS_PREDIV			BIT(0)
#define CLK_DAVINCI_PLL_HAS_POSTDIV			BIT(1)

struct clk_davinci_pll_data {
	/* physical addresses set by platform code */
	u32 phy_pllm;
	/* if PLL has a prediv register this should be non zero */
	u32 phy_prediv;
	/* if PLL has a postdiv register this should be non zero */
	u32 phy_postdiv;
	/* mapped addresses. should be initialized by  */
	void __iomem *pllm;
	void __iomem *prediv;
	void __iomem *postdiv;
	u32 pllm_mask;
	u32 prediv_mask;
	u32 postdiv_mask;
	u32 num;
	/* framework flags */
	u32 flags;
	/* pll flags */
	u32 pll_flags;
       /* use this value for prediv */
	u32 fixed_prediv;
	/* multiply PLLM by this factor. By default most SOC set this to zero
	 * that translates to a multiplier of 1 and incrementer of 1.
	 * To override default, set this factor
	 */
	u32 pllm_multiplier;
};

extern struct clk *clk_register_davinci_pll(struct device *dev,
			const char *name, const char *parent_name,
			struct clk_davinci_pll_data *pll_data);
#endif /* CLK_DAVINCI_PLL_H */
