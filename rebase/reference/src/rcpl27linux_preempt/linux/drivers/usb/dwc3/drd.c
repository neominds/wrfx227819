/****
 * drd.c - DesignWare USB3 DRD Controller Dual Role Switch Funciton File
 *
 * Copyright (C) 2014 Advanced Micro Devices, Inc.
 *
 * Author: Huang Rui <ray.huang@... <http://gmane.org/get-address.php?address=ray.huang%2d5C7GfCeVMHo%40public.gmane.org>>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2  of
 * the License as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/composite.h>

#include "core.h"
#include "gadget.h"
#include "io.h"
#include "drd.h"


int dwc3_drd_to_host(struct dwc3 *dwc)
{
	int ret;
	unsigned long flags = 0;

	if (dwc->has_xhci)
		dwc3_host_exit(dwc);
	if (dwc->has_gadget)
		dwc3_gadget_stop_on_switch(dwc);

	ret = dwc3_core_soft_reset(dwc);
	if (ret) {
		dev_err(dwc->dev, "soft reset failed\n");
		goto err0;
	}

	spin_lock_irqsave(&dwc->lock, flags);
	ret = dwc3_event_buffers_setup(dwc);
	if (ret) {
		dev_err(dwc->dev, "failed to setup event buffers\n");
		goto err0;
	}

	dwc3_set_mode(dwc, DWC3_GCTL_PRTCAP_HOST);

	ret = dwc3_host_init(dwc);
	if (ret) {
		dev_err(dwc->dev, "failed to init host\n");
		goto err0;
	}
err0:
	spin_unlock_irqrestore(&dwc->lock, flags);
	return ret;
}

int dwc3_drd_to_device(struct dwc3 *dwc)
{
	int ret;
	unsigned long timeout, flags = 0;
	u32 reg;

	if (dwc->has_xhci)
		dwc3_host_exit(dwc);
	if (dwc->has_gadget)
		dwc3_gadget_stop_on_switch(dwc);

	ret = dwc3_core_soft_reset(dwc);
	if (ret) {
		dev_err(dwc->dev, "soft reset failed\n");
		goto err0;
	}

	spin_lock_irqsave(&dwc->lock, flags);
	dwc3_set_mode(dwc, DWC3_GCTL_PRTCAP_DEVICE);

	/* issue device SoftReset too */
	timeout = jiffies + msecs_to_jiffies(500);
	dwc3_writel(dwc->regs, DWC3_DCTL, DWC3_DCTL_CSFTRST);
	do {
		reg = dwc3_readl(dwc->regs, DWC3_DCTL);
		if (!(reg & DWC3_DCTL_CSFTRST))
			break;

		if (time_after(jiffies, timeout)) {
			dev_err(dwc->dev, "Reset Timed Out\n");
			ret = -ETIMEDOUT;
			goto err0;
		}

		cpu_relax();
	} while (true);

	ret = dwc3_event_buffers_setup(dwc);
	if (ret) {
		dev_err(dwc->dev, "failed to setup event buffers\n");
		goto err0;
	}

	ret = dwc3_gadget_restart(dwc);
	if (ret) {
		dev_err(dwc->dev, "failed to restart gadget\n");
		goto err0;
	}
err0:
	spin_unlock_irqrestore(&dwc->lock, flags);
	return ret;
}

int dwc3_drd_to_otg(struct dwc3 *dwc)
{
	int ret = 0;
	unsigned long flags = 0;

	if (dwc->has_xhci)
		dwc3_host_exit(dwc);
	if (dwc->has_gadget)
		dwc3_gadget_stop_on_switch(dwc);

	ret = dwc3_core_soft_reset(dwc);
	if (ret) {
		dev_err(dwc->dev, "soft reset failed\n");
		goto err0;
	}

	spin_lock_irqsave(&dwc->lock, flags);
	ret = dwc3_event_buffers_setup(dwc);
	if (ret) {
		dev_err(dwc->dev, "failed to setup event buffers\n");
		goto err0;
	}

	dwc3_set_mode(dwc, DWC3_GCTL_PRTCAP_OTG);

	ret = dwc3_host_init(dwc);
	if (ret) {
		dev_err(dwc->dev, "failed to init host\n");
		goto err0;
	}

err0:
	spin_unlock_irqrestore(&dwc->lock, flags);
	return ret;
}
