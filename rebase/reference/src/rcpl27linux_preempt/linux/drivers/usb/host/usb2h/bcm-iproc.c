/*
 * Copyright (C) 2013, Broadcom Corporation. All Rights Reserved.
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/suspend.h>
#include <mach/memory.h>
#include <mach/iproc_regs.h>
#include <linux/version.h>

#include "usbh_cfg.h"

#include "bcm_usbh.h"

#ifdef DEBUG
#define dbg_printk(fmt, args...) printk(KERN_INFO "%s: " fmt, __func__, ## args)
#else
#define dbg_printk(fmt, args...)
#endif

#define IPROC_USB2_CLK_CONTROL_ENABLE	  (0x1800C180)
#define IPROC_USB2_CLK_CONTROL_ENABLE_VA   HW_IO_PHYS_TO_VIRT(IPROC_USB2_CLK_CONTROL_ENABLE)
#define IPROC_USB2_CLK_CONTROL_PLL		  (0x1800C164)
#define IPROC_USB2_CLK_CONTROL_PLL_VA	   HW_IO_PHYS_TO_VIRT(IPROC_USB2_CLK_CONTROL_PLL)
#define IPROC_STRAP_SKU_VECTOR			  (0x1810D500)
#define IPROC_STRAP_SKU_VECTOR_VA          HW_IO_PHYS_TO_VIRT(IPROC_STRAP_SKU_VECTOR)
#define IPROC_IDM_USB2_RESET_CONTROL	 (0x18115800)
#define IPROC_IDM_USB2_RESET_CONTROL_VA   HW_IO_PHYS_TO_VIRT(IPROC_IDM_USB2_RESET_CONTROL)

#if defined(CONFIG_MACH_HX4) || defined(CONFIG_MACH_NSP) || defined(CONFIG_MACH_KT2)
#define IPROC_IDM_USB2_IO_CONTROL_DIRECT	USB2_IDM_IDM_IO_CONTROL_DIRECT
#define IPROC_IDM_USB2_IO_CONTROL_DIRECT_VA	HW_IO_PHYS_TO_VIRT(IPROC_IDM_USB2_IO_CONTROL_DIRECT)
#endif
#if (defined(CONFIG_MACH_HX4) || defined(CONFIG_MACH_KT2))
#define IPROC_XGPLL							0x1803fc2c
#define IPROC_XGPLL_VA						HW_IO_PHYS_TO_VIRT(IPROC_XGPLL)
#define IPROC_USB_PHY_CTRL					IPROC_WRAP_USBPHY_CTRL
#define IPROC_USB_PHY_CTRL_VA				HW_IO_PHYS_TO_VIRT(IPROC_USB_PHY_CTRL)
#define IPROC_CLK_NDIV_40					0x80
#define IPROC_CLK_NDIV_20					0x8C
#define USB_CLK_NDIV_MASK					0xFE7FFE00
#define USB_CLK_PLL_RESET_MASK				0xFF7FFE00
#define USB_CLK_PHY_RESET_MASK				0xFFFFFE00
#define USB_CLK_NDIV_40						0x30
#define USB_CLK_NDIV_20						0x60
#define ChipcommonA_GPIOOut_VA				HW_IO_PHYS_TO_VIRT(ChipcommonA_GPIOOut)
#define ChipcommonA_GPIOOutEn_VA			HW_IO_PHYS_TO_VIRT(ChipcommonA_GPIOOutEn)
#define SUPPLY_USBD_POWER					0xfffffffd
#endif

#define IPROC_SKU_STRAP_MASK 0xC

struct usbh_ctrl_regs {
	u32 mode;
#define MODE_ULPI_TTL (1<<0)
#define MODE_ULPI_PHY (1<<1)
#define MODE_UTMI_TTL (1<<2)
#define MODE_UTMI_PHY (1<<3)
#define MODE_PORT_CFG(port, mode) ((mode) << (4 * port))

	u32 strap_q;
#define STRAP_PWR_STATE_VALID                (1 << 7)    /* ss_power_state_valid */
#define STRAP_SIM_MODE                       (1 << 6)    /* ss_simulation_mode */
#define STRAP_OHCI_CNTSEL_SIM                (1 << 5)    /* ohci_0_cntsel_i_n */
#define STRAP_PWR_STATE_NXT_VALID            (1 << 4)    /* ss_nxt_power_state_valid_i */
#define STRAP_PWR_STATE_NXT_SHIFT            2           /* ss_next_power_state_i */
#define STRAP_PWR_STATE_NXT_MASK             (3 << STRAP_PWR_STATE_NXT_SHIFT)
#define STRAP_PWR_STATE_SHIFT                0           /* ss_power_state_i */
#define STRAP_PWR_STATE_MASK                 (3 << STRAP_PWR_STATE_SHIFT)

	u32 framelen_adj_q;
	u32 framelen_adj_qx[USBH_NUM_PORTS];       
	u32 misc;
#define MISC_RESUME_R23_ENABLE               (1 << 4) /* ss_utmi_backward_enb_i */
#define MISC_RESUME_R23_UTMI_PLUS_DISABLE    (1 << 3) /* ss_resume_utmi_pls_dis_i */
#define MISC_ULPI_BYPASS_ENABLE              (1 << 2) /* ulpi_bypass_en_i */
#define MISC_PORT_PWRDWN_OVERCURRENT         (1 << 1) /* ss_autoppd_on_overcur_en_i */
#define MISC_OHCI_CLK_RESTART                (1 << 0) /* app_start_clk_i */

};

struct usbh_priv {
	atomic_t probe_done;
	volatile int init_cnt;
	struct mutex lock;
	struct device *dev;
	struct usbh_cfg hw_cfg;
	struct clk *peri_clk;
	struct clk *ahb_clk;
	struct clk *opt_clk;
	struct usbh_ctrl_regs __iomem *ctrl_regs;
};

static struct usbh_priv usbh_data;

int bcm_usbh_suspend(unsigned int host_index)
{
	return 0;
}
EXPORT_SYMBOL(bcm_usbh_suspend);

int bcm_usbh_resume(unsigned int host_index)
{
	return 0;
}
EXPORT_SYMBOL(bcm_usbh_resume);

/*
 * Function to initialize USB host related low level hardware including PHY,
 * clocks, etc.
 *
 * TODO: expand support for more than one host in the future if needed
 */
int bcm_usbh_init(unsigned int host_index)
{
#ifdef CONFIG_MACH_NS
	int usb2_clk_cntrl, usb2_clk_enable, sku_vect;

	sku_vect = __raw_readl(IPROC_STRAP_SKU_VECTOR_VA);
	if ((sku_vect & IPROC_SKU_STRAP_MASK) != 0x0)
	{
		/* enable clocks */
		__raw_writel(0xEA68, IPROC_USB2_CLK_CONTROL_ENABLE_VA);

		usb2_clk_cntrl = __raw_readl(IPROC_USB2_CLK_CONTROL_ENABLE_VA);
		__raw_writel(0xDD10C3, IPROC_USB2_CLK_CONTROL_PLL_VA);

		usb2_clk_enable = __raw_readl(IPROC_USB2_CLK_CONTROL_PLL_VA);
		__raw_writel(0x0, IPROC_USB2_CLK_CONTROL_ENABLE_VA);

		usb2_clk_cntrl = __raw_readl(IPROC_USB2_CLK_CONTROL_ENABLE_VA);
	}
#endif
	return 0;
}

EXPORT_SYMBOL(bcm_usbh_init);
	
/*
 * Function to terminate USB host related low level hardware including PHY,
 * clocks, etc.
 *
 * TODO: expand support for more than one host in the future if needed
 */
int bcm_usbh_term(unsigned int host_index)
{
	return 0;
}
EXPORT_SYMBOL(bcm_usbh_term);
	
static int usbh_probe(struct platform_device *pdev)
{
	int ret;
	struct resource *iomem, *ioarea;

	memset(&usbh_data, 0, sizeof(usbh_data));

	if (pdev->dev.platform_data == NULL) {
		dev_err(&pdev->dev, "platform_data missing\n");
		ret = -EFAULT;
		goto err_exit;
	}
	memcpy(&usbh_data.hw_cfg, pdev->dev.platform_data,
			sizeof(usbh_data.hw_cfg));
	usbh_data.dev = &pdev->dev;
	
	iomem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!iomem) {
		dev_err(&pdev->dev, "no mem resource\n");
		ret = -ENODEV;
		goto err_exit;
	}

	/* mark the memory region as used */
	ioarea = request_mem_region(iomem->start, resource_size(iomem),
			pdev->name);
	if (!ioarea) {
		dev_err(&pdev->dev, "memory region already claimed\n");
		ret = -EBUSY;
		goto err_exit;
	}

	/* now map the I/O memory */
	usbh_data.ctrl_regs = (struct usbh_ctrl_regs __iomem *)
		ioremap(iomem->start, sizeof(usbh_data.ctrl_regs));
	if (!usbh_data.ctrl_regs) {
		dev_err(&pdev->dev, "failed to remap registers\n");
		ret = -ENOMEM;
		goto err_free_mem_region;
	}

	platform_set_drvdata(pdev, &usbh_data);
	mutex_init(&usbh_data.lock);
	usbh_data.init_cnt = 0;
	atomic_set(&usbh_data.probe_done, 1);

	return 0;

err_free_mem_region:
	release_mem_region(iomem->start, resource_size(iomem));

err_exit:
	memset(&usbh_data, 0, sizeof(usbh_data));
	return ret;
}

static int usbh_remove(struct platform_device *pdev)
{
	struct usbh_priv *drv_data = platform_get_drvdata(pdev);
	struct resource *iomem = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	atomic_set(&drv_data->probe_done, 0);
	platform_set_drvdata(pdev, NULL);
	iounmap(drv_data->ctrl_regs);
	release_mem_region(iomem->start, resource_size(iomem));
	memset(&usbh_data, 0, sizeof(usbh_data));

	return 0;
}

static struct platform_driver usbh_driver = 
{
	.driver = {
		.name = "usbh",
		.owner = THIS_MODULE,
	},
	.probe   = usbh_probe,
	.remove  = usbh_remove,
};

static int __init usbh_init(void)
{
	int usb2_reset_state;
	

#if (defined(CONFIG_MACH_HX4) || defined(CONFIG_MACH_KT2))
	int clk_enable;
	unsigned int iClk, USBClk, usbdgpiopwr;

	/* turn off power for USB device connected to the host */
	usbdgpiopwr = __raw_readl(ChipcommonA_GPIOOut_VA);
	usbdgpiopwr |= 0x2;
	__raw_writel(usbdgpiopwr, ChipcommonA_GPIOOut_VA);
	__raw_writel(0x2, ChipcommonA_GPIOOutEn_VA);

	/* Do USB PHY reset */
	mdelay(100);
	USBClk = __raw_readl(IPROC_USB_PHY_CTRL_VA);
	__raw_writel(USBClk & (~(1<<23)), IPROC_USB_PHY_CTRL_VA);
	clk_enable = __raw_readl(IPROC_IDM_USB2_IO_CONTROL_DIRECT_VA);
	dbg_printk("Initial usb2h clock is: %08x\n", clk_enable);
	clk_enable |= 1;
	__raw_writel(clk_enable, IPROC_IDM_USB2_IO_CONTROL_DIRECT_VA);
	clk_enable = __raw_readl(IPROC_IDM_USB2_IO_CONTROL_DIRECT_VA);
	dbg_printk("Initial usb2h clock now is: %08x\n", clk_enable);
#if defined(CONFIG_MACH_HX4)
	iClk = __raw_readl(IPROC_XGPLL_VA);
	USBClk = __raw_readl(IPROC_USB_PHY_CTRL_VA);
	dbg_printk("iClk = %08x, USBClk = %08x\n", iClk, USBClk);
	if ((iClk & 0xff) == IPROC_CLK_NDIV_40)
	{
		__raw_writel((USBClk & USB_CLK_NDIV_MASK) | USB_CLK_NDIV_40, IPROC_USB_PHY_CTRL_VA);
		udelay(10);
		__raw_writel((USBClk & USB_CLK_PLL_RESET_MASK) | USB_CLK_NDIV_40, IPROC_USB_PHY_CTRL_VA);
		udelay(10);
		__raw_writel((USBClk & USB_CLK_PHY_RESET_MASK) | USB_CLK_NDIV_40, IPROC_USB_PHY_CTRL_VA);
		udelay(10);
		USBClk = __raw_readl(IPROC_USB_PHY_CTRL_VA);
		dbg_printk("iClk = %08x, USBClk = %08x\n", iClk, USBClk);
	}
	else if ((iClk & 0xff) == IPROC_CLK_NDIV_20)
	{
		__raw_writel((USBClk & USB_CLK_NDIV_MASK) | USB_CLK_NDIV_20, IPROC_USB_PHY_CTRL_VA);
		udelay(10);
		__raw_writel((USBClk & USB_CLK_PLL_RESET_MASK) | USB_CLK_NDIV_20, IPROC_USB_PHY_CTRL_VA);
		udelay(10);
		__raw_writel((USBClk & USB_CLK_PHY_RESET_MASK) | USB_CLK_NDIV_20, IPROC_USB_PHY_CTRL_VA);
		udelay(10);
		USBClk = __raw_readl(IPROC_USB_PHY_CTRL_VA);
		dbg_printk("iClk = %08x, USBClk = %08x\n", iClk, USBClk);
	}
#endif
	mdelay(100);
	__raw_writel(USBClk | (1<<23), IPROC_USB_PHY_CTRL_VA);
	udelay(100);
#endif
#if defined(CONFIG_MACH_NSP)
	int clk_enable;
	clk_enable = __raw_readl(IPROC_IDM_USB2_IO_CONTROL_DIRECT_VA);
	dbg_printk("Initial usb2h clock is: %08x\n", clk_enable);
	clk_enable |= 1;
	__raw_writel(clk_enable, IPROC_IDM_USB2_IO_CONTROL_DIRECT_VA);
	clk_enable = __raw_readl(IPROC_IDM_USB2_IO_CONTROL_DIRECT_VA);
	dbg_printk("Initial usb2h clock now is: %08x\n", clk_enable);
#endif

	usb2_reset_state = __raw_readl(IPROC_IDM_USB2_RESET_CONTROL_VA);
	dbg_printk("Initial usb2_reset_state is: %08x\n", usb2_reset_state);
	if ((usb2_reset_state & 1) == 1)
	{
		__raw_writel(0x0, IPROC_IDM_USB2_RESET_CONTROL_VA);
		usb2_reset_state = __raw_readl(IPROC_IDM_USB2_RESET_CONTROL_VA);
		dbg_printk("usb2_reset_state is set and now it is: %08x\n", usb2_reset_state);
	}
#if (defined(CONFIG_MACH_HX4) || defined(CONFIG_MACH_KT2))
	/* supply power for USB device connected to the host */
	mdelay(100);
	usbdgpiopwr = __raw_readl(ChipcommonA_GPIOOut_VA);
	usbdgpiopwr &= SUPPLY_USBD_POWER;
	__raw_writel(usbdgpiopwr, ChipcommonA_GPIOOut_VA);
	__raw_writel(0x2, ChipcommonA_GPIOOutEn_VA);
#endif
	return platform_driver_register(&usbh_driver);
}

static void __exit usbh_exit(void)
{
	platform_driver_unregister(&usbh_driver);
}

module_init(usbh_init);
module_exit(usbh_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom USB host low-level driver");
MODULE_LICENSE("GPL");
