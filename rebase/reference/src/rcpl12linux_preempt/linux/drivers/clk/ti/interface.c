/*
 * OMAP interface clock support
 *
 * Copyright (C) 2013 Texas Instruments, Inc.
 *
 * Tero Kristo <t-kristo@ti.com>
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

#include <linux/clk-provider.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/clk/ti.h>

static const struct clk_ops omap_interface_clk_ops = {
	.init		= &omap2_init_clk_clkdm,
	.enable		= &omap2_dflt_clk_enable,
	.disable	= &omap2_dflt_clk_disable,
	.is_enabled	= &omap2_dflt_clk_is_enabled,
};

void __init _of_omap_interface_clk_setup(struct device_node *node,
					 const struct clk_hw_omap_ops *ops)
{
	struct clk *clk;
	struct clk_init_data init = { 0 };
	struct clk_hw_omap *clk_hw;
	const char *clk_name = node->name;
	const char *parent_name;
	u32 val;

	clk_hw = kzalloc(sizeof(*clk_hw), GFP_KERNEL);
	if (!clk_hw) {
		pr_err("%s: could not allocate clk_hw_omap\n", __func__);
		return;
	}

	clk_hw->hw.init = &init;
	clk_hw->ops = ops;
	clk_hw->enable_reg = of_iomap(node, 0);

	if (!of_property_read_u32(node, "ti,enable-bit", &val))
		clk_hw->enable_bit = val;

	of_property_read_string(node, "clock-output-names", &clk_name);

	init.name = clk_name;
	init.ops = &omap_interface_clk_ops;
	init.flags = 0;

	parent_name = of_clk_get_parent_name(node, 0);
	if (!parent_name) {
		pr_err("%s: %s must have a parent\n", __func__, clk_name);
		goto cleanup;
	}

	init.num_parents = 1;
	init.parent_names = &parent_name;

	clk = clk_register(NULL, &clk_hw->hw);

	if (!IS_ERR(clk)) {
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
		omap2_init_clk_hw_omap_clocks(clk);
		return;
	}

cleanup:
	kfree(clk_hw);
}

static void __init of_omap_interface_clk_setup(struct device_node *node)
{
	_of_omap_interface_clk_setup(node, &clkhwops_iclk_wait);
}
CLK_OF_DECLARE(omap_interface_clk, "ti,omap3-interface-clock",
	       of_omap_interface_clk_setup);

static void __init of_omap_no_wait_interface_clk_setup(struct device_node *node)
{
	_of_omap_interface_clk_setup(node, &clkhwops_iclk);
}
CLK_OF_DECLARE(omap_no_wait_interface_clk, "ti,omap3-no-wait-interface-clock",
	       of_omap_no_wait_interface_clk_setup);

static void __init
of_omap_hsotgusb_interface_clk_setup(struct device_node *node)
{
	_of_omap_interface_clk_setup(node,
				     &clkhwops_omap3430es2_iclk_hsotgusb_wait);
}
CLK_OF_DECLARE(omap_hsotgusb_interface_clk, "ti,omap3-hsotgusb-interface-clock",
	       of_omap_hsotgusb_interface_clk_setup);

static void __init of_omap_dss_interface_clk_setup(struct device_node *node)
{
	_of_omap_interface_clk_setup(node,
		&clkhwops_omap3430es2_iclk_dss_usbhost_wait);
}
CLK_OF_DECLARE(omap_dss_interface_clk, "ti,omap3-dss-interface-clock",
	       of_omap_dss_interface_clk_setup);

static void __init of_omap_ssi_interface_clk_setup(struct device_node *node)
{
	_of_omap_interface_clk_setup(node, &clkhwops_omap3430es2_iclk_ssi_wait);
}
CLK_OF_DECLARE(omap_ssi_interface_clk, "ti,omap3-ssi-interface-clock",
	       of_omap_ssi_interface_clk_setup);

static void __init of_omap_am35xx_interface_clk_setup(struct device_node *node)
{
	_of_omap_interface_clk_setup(node, &clkhwops_am35xx_ipss_wait);
}
CLK_OF_DECLARE(omap_am35xx_interface_clk, "ti,am35xx-interface-clock",
	       of_omap_am35xx_interface_clk_setup);
