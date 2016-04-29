/*
 * TI DaVinci Clock definitions -  Contains Macros and Types used for
 * defining various clocks on a DaVinci SoC
 *
 * Copyright (C) 2012 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __DAVINCI_CLOCK_H
#define __DAVINCI_CLOCK_H

#include <linux/types.h>

struct clk;

#define CLK(dev, con, ck)	\
	{			\
		.dev_id = dev,	\
		.con_id = con,	\
		._clk = ck,	\
	}			\

/* general flags: */
#define ALWAYS_ENABLED		BIT(0)

/* configuration data for clk mux */
struct clk_mux_data {
	u32			flags;
	u8			mux_flags;
	u8			shift;
	u8			width;
	u8			num_parents;
	const char		**parents;
	u32			phys_base;
};

/* configuration data for PLL divider clock */
struct clk_divider_data {
	u32			flags;
	u8			divider_flags;
	u32                     div_reg;
	/* H/W supported max rate */
	unsigned long		maxrate;
	/* Width and shift for divider clock register mask */
	u8			shift;
	u8			width;
};

/* configuration data for fixed factor clocks. Some clocks in DaVinci are just
 * duplicate clock pins that can be represented by this using a factor of 1
 * for multiplier and divider
 */
struct clk_fixed_factor_data {
	u32			flags;
	u32			mult;
	u32			div;
};

/* configuration data for fixed ref clock */
struct clk_fixed_rate_data {
	u32			flags;
	unsigned long		rate;
	/* where a register has input clock rate settings this function
	 * is used to read that value from SoC specific code
	 */
	unsigned long		(*recalc)(unsigned long parent_rate);
};

/* forward declaration for other clock drivers */
struct clk_davinci_pll_data;
struct clk_keystone_pll_data;
struct clk_psc_data;

enum davinci_clk_type {
	DAVINCI_MAIN_PLL_CLK,
	KEYSTONE_MAIN_PLL_CLK,
	DAVINCI_FIXED_RATE_CLK,
	/* Use programmable divider */
	DAVINCI_PRG_DIV_CLK,
	/* Use fixed divider and multiplier */
	DAVINCI_FIXED_FACTOR_CLK,
	DAVINCI_PSC_CLK,
	DAVINCI_MUX_CLK,
};

/* struct for defining DaVinci clocks for a SoC. Only one of the data ptr
 * to be valid (non NULL). davinci_clk_init() in drivers/clk/davinci/clock.c
 * check these ptr values to determine what clock register function to call
 * to register a particular clock.
 */
struct davinci_clk {
	const char			*name;
	/* General flag for all drivers */
	u32				flags;
	struct davinci_clk		*parent;
	enum davinci_clk_type		type;
	/* one of these will be present in each SoC */
	union {
		/* root ref clock data */
		struct clk_fixed_rate_data	*fixed_rate;
		struct clk_davinci_pll_data	*davinci_pll;
		struct clk_keystone_pll_data	*keystone_pll;
		struct clk_divider_data		*pll_div;
		struct clk_davinci_psc_data	*psc;
		struct clk_mux_data		*mux;
		struct clk_fixed_factor_data	*fixed_factor;
		void				*data;
	} clk_data;
};

/* struct for the DaVinci clock tables. */
struct davinci_clk_lookup {
	const char		*dev_id;
	const char		*con_id;
	struct davinci_clk      *_clk;
	/* This saves the opaque clk ptr returned by clk_register() and is
	 * saved to use later to register clkdevs for a specific clk node.
	 */
	struct clk		*clk;
};

/* struct for the DaVinci clkdev lookups. When several drivers uses the same
 * clock, this structure is used to define a dev look up table that maps
 * clkdevs to a clks node.
 */
struct davinci_dev_lookup {
	const char		*con_id;
	unsigned short		num_devs;
	struct clk_lookup	*lookups;
};

/* clock init function for a DaVinci SoC (defined in drivers/clk/clock.c).
 * This can also be used on other TI SoCs that has similar clock hardware.
 */
extern int davinci_common_clk_init(struct davinci_clk_lookup *clocks,
				struct davinci_dev_lookup *dev_lookups,
				u8 num_gpscs, u32 *psc_bases);
#ifdef CONFIG_OF
extern void davinci_add_clkdev(struct davinci_clk_lookup *clocks);
extern void davinci_of_clk_init(void);
#endif
#endif
