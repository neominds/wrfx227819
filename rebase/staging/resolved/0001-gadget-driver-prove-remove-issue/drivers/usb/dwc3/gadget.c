/**
 * gadget.c - DesignWare USB3 DRD Controller Gadget Framework Link
 *
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com
 *
 * Authors: Felipe Balbi <balbi@ti.com>,
 *	    Sebastian Andrzej Siewior <bigeasy@linutronix.de>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The names of the above-listed copyright holders may not be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2, as published by the Free
 * Software Foundation.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/dma-mapping.h>

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>

#include "core.h"
#include "gadget.h"
#include "io.h"

/* 55XX A0 D+ workaround */
#define WORKAROUDN_55XXA0_DP_ISSUE    (1)

#if WORKAROUDN_55XXA0_DP_ISSUE
/* 55XX A0 D+ workaround */
#define USB3_PHY_CR_CAP_ADDR      0x04400200
#define USB3_PHY_CR_CAP_DATA      0x04400204
#define USB3_PHY_CR_DATA_IN       0x04400208
#define USB3_PHY_CR_READ          0x0440020C
#define USB3_PHY_CR_WRITE         0x04400210
#define USB3_PHY_CR_ACK           0x04400214
#define USB3_PHY_CR_DATA_OUT      0x04400218

#undef	HNDEBUG		/*DEBUG for HN*/
#define	HNDEBUG	1	/*DEBUG for HN*/

volatile u8	__iomem	*cr_cap_addr;
volatile u8	__iomem	*cr_cap_data;
volatile u8	__iomem	*cr_data_in;
volatile u8     __iomem *cr_read;
volatile u8	__iomem	*cr_write;
volatile u8	__iomem	*cr_ack;
volatile u8     __iomem *cr_data_out;

void phy_cr_addr(u32 address)
{
	writel(address, cr_data_in);
	writel(0x01, cr_cap_addr);
	while (readl(cr_ack) == 0x00) {}
	writel(0x0, cr_cap_addr);
	while (readl(cr_ack) == 0x01) {}
}

static u32 phy_cr_read(u32 addr)
{
        u32 reg;

        phy_cr_addr(addr);
        writel(0x01, cr_read);
        while (readl(cr_ack) == 0x00) {}
        reg = readl(cr_data_out);
        writel(0x00, cr_read);
        while (readl(cr_ack) == 0x01) {}

        return reg;
}

static void phy_cr_write(u32 addr, u32 value)
{
	phy_cr_addr(addr);
	writel(value, cr_data_in);
	writel(0x01, cr_cap_data);
	while (readl(cr_ack) == 0x00) {}
	writel(0x00, cr_cap_data);
	while (readl(cr_ack) == 0x01) {}
	writel(0x01, cr_write);
	while (readl(cr_ack) == 0x00) {}
	writel(0x00, cr_write);
	while (readl(cr_ack) == 0x01) {}
}

void phy_fixed_equalization(u32 value)   // Andy test: test fixed equalization
{
        u32 reg = 0;

        if (value > 7) {
                printk("Wrong equalization value %d. Abort\n", value);
                return;
        }

        reg = phy_cr_read(0x1006);
        reg &= ~0x40;
        phy_cr_write(0x1006, reg);

        reg = phy_cr_read(0x1006);
        reg |= 0x80;
        phy_cr_write(0x1006, reg);

        reg = phy_cr_read(0x1006);
        reg |= (value << 8);
        phy_cr_write(0x1006, reg);

        reg = phy_cr_read(0x1006);
        reg |= 0x0800;
        phy_cr_write(0x1006, reg);

        reg = phy_cr_read(0x1006);
#if 0
        printk("PHY control register 0x1006 is modified as 0x%08X\n", reg);   // Andy test
#endif
}

static void phy_dp_pull_up(u8 on, u8 weak)
{
	/* NOTE: The enable bits [5:4] cannot be read back. */
	switch (on)
	{
	case 0:
		/* Deactivate pull-up first */
		phy_cr_write(0x2014, 0x31);
		/* Disable override */
		phy_cr_write(0x2014, 0x00);
		break;
	case 1:
		if (weak)
			phy_cr_write(0x2014, 0x32);
		else
			phy_cr_write(0x2014, 0x30);
		break;
	default:
		break;
	}
}
#endif	/* WORKAROUDN_55XXA0_DP_ISSUE*/

/**
 * dwc3_gadget_set_test_mode - Enables USB2 Test Modes
 * @dwc: pointer to our context structure
 * @mode: the mode to set (J, K SE0 NAK, Force Enable)
 *
 * Caller should take care of locking. This function will
 * return 0 on success or -EINVAL if wrong Test Selector
 * is passed
 */
int dwc3_gadget_set_test_mode(struct dwc3 *dwc, int mode)
{
	u32		reg;

	reg = dwc3_readl(dwc->regs, DWC3_DCTL);
	reg &= ~DWC3_DCTL_TSTCTRL_MASK;

	switch (mode) {
	case TEST_J:
	case TEST_K:
	case TEST_SE0_NAK:
	case TEST_PACKET:
	case TEST_FORCE_EN:
		reg |= mode << 1;
		break;
	default:
		return -EINVAL;
	}

	dwc3_writel(dwc->regs, DWC3_DCTL, reg);

	return 0;
}

/**
 * dwc3_gadget_get_link_state - Gets current state of USB Link
 * @dwc: pointer to our context structure
 *
 * Caller should take care of locking. This function will
 * return the link state on success (>= 0) or -ETIMEDOUT.
 */
int dwc3_gadget_get_link_state(struct dwc3 *dwc)
{
	u32		reg;

	reg = dwc3_readl(dwc->regs, DWC3_DSTS);

	return DWC3_DSTS_USBLNKST(reg);
}

/**
 * dwc3_gadget_set_link_state - Sets USB Link to a particular State
 * @dwc: pointer to our context structure
 * @state: the state to put link into
 *
 * Caller should take care of locking. This function will
 * return 0 on success or -ETIMEDOUT.
 */
int dwc3_gadget_set_link_state(struct dwc3 *dwc, enum dwc3_link_state state)
{
	int		retries = 10000;
	u32		reg;

#if	0	/*<HN>*/
	dev_vdbg(dwc->dev, "%s <HN>\n",__FUNCTION__);
#endif
	/*
	 * Wait until device controller is ready. Only applies to 1.94a and
	 * later RTL.
	 */
//printk(KERN_ERR "XXXXXX %s entry\n",__FUNCTION__);
	if (dwc->revision >= DWC3_REVISION_194A) {
//printk(KERN_ERR "XXXXXX %s 194A afer\n",__FUNCTION__);
		while (--retries) {
			reg = dwc3_readl(dwc->regs, DWC3_DSTS);
			if (reg & DWC3_DSTS_DCNRD)
				udelay(5);
			else
				break;
		}

		if (retries <= 0)
			return -ETIMEDOUT;
	}

	reg = dwc3_readl(dwc->regs, DWC3_DCTL);
	reg &= ~DWC3_DCTL_ULSTCHNGREQ_MASK;

	/* set requested state */
	reg |= DWC3_DCTL_ULSTCHNGREQ(state);
	dwc3_writel(dwc->regs, DWC3_DCTL, reg);

	/*
	 * The following code is racy when called from dwc3_gadget_wakeup,
	 * and is not needed, at least on newer versions
	 */
	if (dwc->revision >= DWC3_REVISION_194A)
		return 0;

	/* wait for a change in DSTS */
	retries = 10000;
	while (--retries) {
		reg = dwc3_readl(dwc->regs, DWC3_DSTS);

		if (DWC3_DSTS_USBLNKST(reg) == state)
			return 0;

		udelay(5);
	}

	dev_vdbg(dwc->dev, "link state change request timed out\n");

	return -ETIMEDOUT;
}

/**
 * dwc3_gadget_resize_tx_fifos - reallocate fifo spaces for current use-case
 * @dwc: pointer to our context structure
 *
 * This function will a best effort FIFO allocation in order
 * to improve FIFO usage and throughput, while still allowing
 * us to enable as many endpoints as possible.
 *
 * Keep in mind that this operation will be highly dependent
 * on the configured size for RAM1 - which contains TxFifo -,
 * the amount of endpoints enabled on coreConsultant tool, and
 * the width of the Master Bus.
 *
 * In the ideal world, we would always be able to satisfy the
 * following equation:
 *
 * ((512 + 2 * MDWIDTH-Bytes) + (Number of IN Endpoints - 1) * \
 * (3 * (1024 + MDWIDTH-Bytes) + MDWIDTH-Bytes)) / MDWIDTH-Bytes
 *
 * Unfortunately, due to many variables that's not always the case.
 */
int dwc3_gadget_resize_tx_fifos(struct dwc3 *dwc)
{
	int		last_fifo_depth = 0;
	int		ram1_depth;
	int		fifo_size;
	int		mdwidth;
	int		num;
#if	1	/*<HN>*/
	dev_vdbg(dwc->dev, "%s <HN>\n",__FUNCTION__);
#endif

	//printk(KERN_ERR "# %s entry\n",__FUNCTION__);
	if (!dwc->needs_fifo_resize)
		return 0;

//printk(KERN_ERR "# %s execute\n",__FUNCTION__);
	ram1_depth = DWC3_RAM1_DEPTH(dwc->hwparams.hwparams7);
	mdwidth = DWC3_MDWIDTH(dwc->hwparams.hwparams0);

	/* MDWIDTH is represented in bits, we need it in bytes */
	mdwidth >>= 3;

	/*
	 * FIXME For now we will only allocate 1 wMaxPacketSize space
	 * for each enabled endpoint, later patches will come to
	 * improve this algorithm so that we better use the internal
	 * FIFO space
	 */
	for (num = 0; num < dwc->num_in_eps; num++) {
		/* bit0 indicates direction; 1 means IN ep */
		struct dwc3_ep	*dep = dwc->eps[(num << 1) | 1];
		int		mult = 1;
		int		tmp;

		if (!(dep->flags & DWC3_EP_ENABLED))
			continue;

		if (usb_endpoint_xfer_bulk(dep->endpoint.desc)
				|| usb_endpoint_xfer_isoc(dep->endpoint.desc))
			mult = 3;

		/*
		 * REVISIT: the following assumes we will always have enough
		 * space available on the FIFO RAM for all possible use cases.
		 * Make sure that's true somehow and change FIFO allocation
		 * accordingly.
		 *
		 * If we have Bulk or Isochronous endpoints, we want
		 * them to be able to be very, very fast. So we're giving
		 * those endpoints a fifo_size which is enough for 3 full
		 * packets
		 */
		tmp = mult * (dep->endpoint.maxpacket + mdwidth);
		tmp += mdwidth;

		fifo_size = DIV_ROUND_UP(tmp, mdwidth);

		fifo_size |= (last_fifo_depth << 16);

		dev_vdbg(dwc->dev, "%s: Fifo Addr %04x Size %d\n",
				dep->name, last_fifo_depth, fifo_size & 0xffff);

		dwc3_writel(dwc->regs, DWC3_GTXFIFOSIZ(num), fifo_size);

		last_fifo_depth += (fifo_size & 0xffff);
	}

	return 0;
}

void dwc3_gadget_giveback(struct dwc3_ep *dep, struct dwc3_request *req,
		int status)
{
	struct dwc3	*dwc = dep->dwc;
	int		i;
#if	0	/*<HN>*/
	dev_vdbg(dwc->dev, "%s <HN> Q=%d\n",__FUNCTION__,req->queued);
#endif
	if (req->queued) {
		i = 0;
		do {
			dep->busy_slot++;
			/*
			 * Skip LINK TRB. We can't use req->trb and check for
			 * DWC3_TRBCTL_LINK_TRB because it points the TRB we
			 * just completed (not the LINK TRB).
			 */
			if (((dep->busy_slot & DWC3_TRB_MASK) ==
				DWC3_TRB_NUM- 1) &&
				usb_endpoint_xfer_isoc(dep->endpoint.desc))
				dep->busy_slot++;
		} while(++i < req->request.num_mapped_sgs);
		req->queued = false;
	}
	list_del(&req->list);
	req->trb = NULL;

	if (req->request.status == -EINPROGRESS)
		req->request.status = status;

	if (dwc->ep0_bounced && dep->number == 0)
		dwc->ep0_bounced = false;
	else
		usb_gadget_unmap_request(&dwc->gadget, &req->request,
				req->direction);

	dev_dbg(dwc->dev, "request %p from %s completed %d/%d ===> %d\n",
			req, dep->name, req->request.actual,
			req->request.length, status);
#ifdef	HNDEBUG /*<HN>*/
	if (status < 0) {
		switch (status) {
		case -EINPROGRESS:
			dev_dbg(dwc->dev, "Status: EINPROGRESS\n");
			break;
		case -ESHUTDOWN:
			dev_dbg(dwc->dev, "Status: Cannot send after transport endpoint shutdown\n");
			break;
		case -ETIMEDOUT:
			dev_dbg(dwc->dev, "Status: Connection timed out\n");
			break;
		default:
			dev_dbg(dwc->dev, "Status: Others=%0xH\n",status);
			break;
		}
	}
#endif
	spin_unlock(&dwc->lock);
//printk(KERN_ERR "# GADGET # %s call compleate\n",__func__);
	req->request.complete(&dep->endpoint, &req->request);
	spin_lock(&dwc->lock);
}

static const char *dwc3_gadget_ep_cmd_string(u8 cmd)
{
	switch (cmd) {
	case DWC3_DEPCMD_DEPSTARTCFG:
		return "Start New Configuration";
	case DWC3_DEPCMD_ENDTRANSFER:
		return "End Transfer";
	case DWC3_DEPCMD_UPDATETRANSFER:
		return "Update Transfer";
	case DWC3_DEPCMD_STARTTRANSFER:
		return "Start Transfer";
	case DWC3_DEPCMD_CLEARSTALL:
		return "Clear Stall";
	case DWC3_DEPCMD_SETSTALL:
		return "Set Stall";
	case DWC3_DEPCMD_GETEPSTATE:
		return "Get Endpoint State";
	case DWC3_DEPCMD_SETTRANSFRESOURCE:
		return "Set Endpoint Transfer Resource";
	case DWC3_DEPCMD_SETEPCONFIG:
		return "Set Endpoint Configuration";
	default:
		return "UNKNOWN command";
	}
}

static const char *dwc3_gadget_generic_cmd_string(u8 cmd)
{
	switch (cmd) {
	case DWC3_DGCMD_SET_LMP:
		return "Set LMP";
	case DWC3_DGCMD_SET_PERIODIC_PAR:
		return "Set Periodic Parameters";
	case DWC3_DGCMD_XMIT_FUNCTION:
		return "Transmit Function Wake Device Notification";
	case DWC3_DGCMD_SET_SCRATCHPAD_ADDR_LO:
		return "Set Scratchpad Buffer Array Address Lo";
	case DWC3_DGCMD_SET_SCRATCHPAD_ADDR_HI:
		return "Set Scratchpad Buffer Array Address Hi";
	case DWC3_DGCMD_SELECTED_FIFO_FLUSH:
		return "Selected FIFO Flush";
	case DWC3_DGCMD_ALL_FIFO_FLUSH:
		return "All FIFO Flush";
	case DWC3_DGCMD_SET_ENDPOINT_NRDY:
		return "Set Endpoint NRDY";
	case DWC3_DGCMD_RUN_SOC_BUS_LOOPBACK:
		return "Run SoC Bus Loopback Test";
	default:
		return "UNKNOWN";
	}
}

static const char *dwc3_gadget_link_string(enum dwc3_link_state link_state)
{
	switch (link_state) {
	case DWC3_LINK_STATE_U0:
		return "U0";
	case DWC3_LINK_STATE_U1:
		return "U1";
	case DWC3_LINK_STATE_U2:
		return "U2";
	case DWC3_LINK_STATE_U3:
		return "U3";
	case DWC3_LINK_STATE_SS_DIS:
		return "SS.Disabled";
	case DWC3_LINK_STATE_RX_DET:
		return "RX.Detect";
	case DWC3_LINK_STATE_SS_INACT:
		return "SS.Inactive";
	case DWC3_LINK_STATE_POLL:
		return "Polling";
	case DWC3_LINK_STATE_RECOV:
		return "Recovery";
	case DWC3_LINK_STATE_HRESET:
		return "Hot Reset";
	case DWC3_LINK_STATE_CMPLY:
		return "Compliance";
	case DWC3_LINK_STATE_LPBK:
		return "Loopback";
	case DWC3_LINK_STATE_RESET:
		return "Reset";
	case DWC3_LINK_STATE_RESUME:
		return "Resume";
	default:
		return "UNKNOWN link state\n";
	}
}

int dwc3_send_gadget_generic_command(struct dwc3 *dwc, int cmd, u32 param)
{
	u32		timeout = 500;
	u32		reg;

	dwc3_writel(dwc->regs, DWC3_DGCMDPAR, param);
	dwc3_writel(dwc->regs, DWC3_DGCMD, cmd | DWC3_DGCMD_CMDACT);

	do {
		reg = dwc3_readl(dwc->regs, DWC3_DGCMD);
		if (!(reg & DWC3_DGCMD_CMDACT)) {
#ifdef	HNDEBUG /*<HN>*/
			dev_vdbg(dwc->dev, "G-Command Complete --> %d\n",
					DWC3_DGCMD_STATUS(reg));
#else
			dev_vdbg(dwc->dev, "Command Complete --> %d\n",
					DWC3_DGCMD_STATUS(reg));
#endif
			return 0;
		}

		/*
		 * We can't sleep here, because it's also called from
		 * interrupt context.
		 */
		timeout--;
		if (!timeout)
			return -ETIMEDOUT;
		udelay(1);
	} while (1);
}

int dwc3_send_gadget_ep_cmd(struct dwc3 *dwc, unsigned ep,
		unsigned cmd, struct dwc3_gadget_ep_cmd_params *params)
{
	struct dwc3_ep		*dep = dwc->eps[ep];
	u32			timeout = 500;
	u32			reg;

/*	unsigned long			flags;*/
//printk(KERN_ERR "# GADGET # %s Entry ep[%d] cmd '%x'\n",__FUNCTION__,ep,cmd);
	dev_vdbg(dwc->dev, "%s: cmd '%s' params %08x %08x %08x\n",
			dep->name,
			dwc3_gadget_ep_cmd_string(cmd), params->param0,
			params->param1, params->param2);

/*	spin_lock_irqsave(&dwc->lock, flags);	NO-GOOD *//* added 0305 <HN>*/
	dwc3_writel(dwc->regs, DWC3_DEPCMDPAR0(ep), params->param0);
	dwc3_writel(dwc->regs, DWC3_DEPCMDPAR1(ep), params->param1);
	dwc3_writel(dwc->regs, DWC3_DEPCMDPAR2(ep), params->param2);

	dwc3_writel(dwc->regs, DWC3_DEPCMD(ep), cmd | DWC3_DEPCMD_CMDACT);
/*	spin_unlock_irqrestore(&dwc->lock, flags);	NO-GOOD *//* added 0305 <HN>*/
	do {
		reg = dwc3_readl(dwc->regs, DWC3_DEPCMD(ep));
		if (!(reg & DWC3_DEPCMD_CMDACT)) {
//			dev_vdbg(dwc->dev, "Command Complete --> %d\n",
//printk(KERN_ERR "# GADGET # Command Complete --> %d\n",
//					DWC3_DEPCMD_STATUS(reg));
			return 0;
		}

		/*
		 * We can't sleep here, because it is also called from
		 * interrupt context.
		 */
		timeout--;
		if (!timeout)
			return -ETIMEDOUT;

		udelay(1);
	} while (1);
	/*Added for debug should not reach here.<HN>*/
	dev_vdbg(dwc->dev, "Command Complete ???--> %d <HN>\n",DWC3_DEPCMD_STATUS(reg));
	return 0;
}

static dma_addr_t dwc3_trb_dma_offset(struct dwc3_ep *dep,
		struct dwc3_trb *trb)
{
	u32		offset = (char *) trb - (char *) dep->trb_pool;

	return dep->trb_pool_dma + offset;
}

static int dwc3_alloc_trb_pool(struct dwc3_ep *dep)
{
	struct dwc3		*dwc = dep->dwc;

	if (dep->trb_pool)
		return 0;

	if (dep->number == 0 || dep->number == 1)
		return 0;

	dep->trb_pool = dma_alloc_coherent(dwc->dev,
			sizeof(struct dwc3_trb) * DWC3_TRB_NUM,
			&dep->trb_pool_dma, GFP_KERNEL);
	if (!dep->trb_pool) {
		dev_err(dep->dwc->dev, "failed to allocate trb pool for %s\n",
				dep->name);
		return -ENOMEM;
	}

	return 0;
}

static void dwc3_free_trb_pool(struct dwc3_ep *dep)
{
	struct dwc3		*dwc = dep->dwc;

	dma_free_coherent(dwc->dev, sizeof(struct dwc3_trb) * DWC3_TRB_NUM,
			dep->trb_pool, dep->trb_pool_dma);

	dep->trb_pool = NULL;
	dep->trb_pool_dma = 0;
}

static int dwc3_gadget_start_config(struct dwc3 *dwc, struct dwc3_ep *dep)
{
	struct dwc3_gadget_ep_cmd_params params;
	u32			cmd;

#if	0	/*<HN>*/
	dev_vdbg(dwc->dev, "%s <HN>\n",__FUNCTION__);
#endif
	memset(&params, 0x00, sizeof(params));

	if (dep->number != 1) {
		cmd = DWC3_DEPCMD_DEPSTARTCFG;
		/* XferRscIdx == 0 for ep0 and 2 for the remaining */
		if (dep->number > 1) {
			if (dwc->start_config_issued)
				return 0;
			dwc->start_config_issued = true;
			cmd |= DWC3_DEPCMD_PARAM(2);
		}
//printk(KERN_ERR "# GADGET # %s Call EP Command \n",__func__);
		return dwc3_send_gadget_ep_cmd(dwc, 0, cmd, &params);
	}

	return 0;
}

static int dwc3_gadget_set_ep_config(struct dwc3 *dwc, struct dwc3_ep *dep,
		const struct usb_endpoint_descriptor *desc,
		const struct usb_ss_ep_comp_descriptor *comp_desc,
		bool ignore, bool restore)
{
	struct dwc3_gadget_ep_cmd_params params;

	memset(&params, 0x00, sizeof(params));

	params.param0 = DWC3_DEPCFG_EP_TYPE(usb_endpoint_type(desc))
		| DWC3_DEPCFG_MAX_PACKET_SIZE(usb_endpoint_maxp(desc));

	/* Burst size is only needed in SuperSpeed mode */
	if (dwc->gadget.speed == USB_SPEED_SUPER) {
//printk(KERN_ERR "*** %s : USB_SPEED_SUPER\n",__FUNCTION__);
		u32 burst = dep->endpoint.maxburst - 1;
/* yamano debug */
		if(burst < 1)	burst =1;
		params.param0 |= DWC3_DEPCFG_BURST_SIZE(burst);
	}

	if (ignore)
		params.param0 |= DWC3_DEPCFG_IGN_SEQ_NUM;

	if (restore) {
		params.param0 |= DWC3_DEPCFG_ACTION_RESTORE;
		params.param2 |= dep->saved_state;
	}

	params.param1 = DWC3_DEPCFG_XFER_COMPLETE_EN
		| DWC3_DEPCFG_XFER_NOT_READY_EN;

	if (usb_ss_max_streams(comp_desc) && usb_endpoint_xfer_bulk(desc)) {
		params.param1 |= DWC3_DEPCFG_STREAM_CAPABLE
			| DWC3_DEPCFG_STREAM_EVENT_EN;
		dep->stream_capable = true;
	}

	if (usb_endpoint_xfer_isoc(desc))
		params.param1 |= DWC3_DEPCFG_XFER_IN_PROGRESS_EN;

	/*
	 * We are doing 1:1 mapping for endpoints, meaning
	 * Physical Endpoints 2 maps to Logical Endpoint 2 and
	 * so on. We consider the direction bit as part of the physical
	 * endpoint number. So USB endpoint 0x81 is 0x03.
	 */
	params.param1 |= DWC3_DEPCFG_EP_NUMBER(dep->number);

	/*
	 * We must use the lower 16 TX FIFOs even though
	 * HW might have more
	 */
	if (dep->direction)
		params.param0 |= DWC3_DEPCFG_FIFO_NUMBER(dep->number >> 1);

	if (desc->bInterval) {
		params.param1 |= DWC3_DEPCFG_BINTERVAL_M1(desc->bInterval - 1);
		dep->interval = 1 << (desc->bInterval - 1);
	}

//printk(KERN_ERR "# GADGET # %s Call EP Setup Command \n",__func__);
	return dwc3_send_gadget_ep_cmd(dwc, dep->number,
			DWC3_DEPCMD_SETEPCONFIG, &params);
}

static int dwc3_gadget_set_xfer_resource(struct dwc3 *dwc, struct dwc3_ep *dep)
{
	struct dwc3_gadget_ep_cmd_params params;

	memset(&params, 0x00, sizeof(params));

	params.param0 = DWC3_DEPXFERCFG_NUM_XFER_RES(1);

//printk(KERN_ERR "# GADGET # %s Call Set trans resource \n",__func__);
	return dwc3_send_gadget_ep_cmd(dwc, dep->number,
			DWC3_DEPCMD_SETTRANSFRESOURCE, &params);
}

/**
 * __dwc3_gadget_ep_enable - Initializes a HW endpoint
 * @dep: endpoint to be initialized
 * @desc: USB Endpoint Descriptor
 *
 * Caller should take care of locking
 */
static int __dwc3_gadget_ep_enable(struct dwc3_ep *dep,
		const struct usb_endpoint_descriptor *desc,
		const struct usb_ss_ep_comp_descriptor *comp_desc,
		bool ignore, bool restore)
{
	struct dwc3		*dwc = dep->dwc;
	u32			reg;
	int			ret;

	dev_vdbg(dwc->dev, "Enabling %s\n", dep->name);
//	printk(KERN_ERR "Enabling %s\n", dep->name);	/* yamano */

	if (!(dep->flags & DWC3_EP_ENABLED)) {
		ret = dwc3_gadget_start_config(dwc, dep);
		if (ret)
			return ret;
	}

	ret = dwc3_gadget_set_ep_config(dwc, dep, desc, comp_desc, ignore,
			restore);
	if (ret)
		return ret;

	if (!(dep->flags & DWC3_EP_ENABLED)) {
		struct dwc3_trb	*trb_st_hw;
		struct dwc3_trb	*trb_link;

		ret = dwc3_gadget_set_xfer_resource(dwc, dep);
		if (ret)
			return ret;

		dep->endpoint.desc = desc;
		dep->comp_desc = comp_desc;
		dep->type = usb_endpoint_type(desc);
		dep->flags |= DWC3_EP_ENABLED;

		reg = dwc3_readl(dwc->regs, DWC3_DALEPENA);
		reg |= DWC3_DALEPENA_EP(dep->number);
		dwc3_writel(dwc->regs, DWC3_DALEPENA, reg);

		if (!usb_endpoint_xfer_isoc(desc))
			return 0;

		/* Link TRB for ISOC. The HWO bit is never reset */
		trb_st_hw = &dep->trb_pool[0];

		trb_link = &dep->trb_pool[DWC3_TRB_NUM - 1];
		memset(trb_link, 0, sizeof(*trb_link));

		trb_link->bpl = lower_32_bits(dwc3_trb_dma_offset(dep, trb_st_hw));
		trb_link->bph = upper_32_bits(dwc3_trb_dma_offset(dep, trb_st_hw));
		trb_link->ctrl |= DWC3_TRBCTL_LINK_TRB;
		trb_link->ctrl |= DWC3_TRB_CTRL_HWO;
	}

	return 0;
}

static void dwc3_stop_active_transfer(struct dwc3 *dwc, u32 epnum, bool force);
static void dwc3_remove_requests(struct dwc3 *dwc, struct dwc3_ep *dep)
{
	struct dwc3_request		*req;

	if (!list_empty(&dep->req_queued)) {
		dwc3_stop_active_transfer(dwc, dep->number, true);

		/* - giveback all requests to gadget driver */
		while (!list_empty(&dep->req_queued)) {
			req = next_request(&dep->req_queued);

			dwc3_gadget_giveback(dep, req, -ESHUTDOWN);
		}
	}

	while (!list_empty(&dep->request_list)) {
		req = next_request(&dep->request_list);

		dwc3_gadget_giveback(dep, req, -ESHUTDOWN);
	}
}

/**
 * __dwc3_gadget_ep_disable - Disables a HW endpoint
 * @dep: the endpoint to disable
 *
 * This function also removes requests which are currently processed ny the
 * hardware and those which are not yet scheduled.
 * Caller should take care of locking.
 */
static int __dwc3_gadget_ep_disable(struct dwc3_ep *dep)
{
	struct dwc3		*dwc = dep->dwc;
	u32			reg;

	dwc3_remove_requests(dwc, dep);
#if 0
printk(KERN_ERR "## %s entry\n",__func__);
#endif
	/* make sure HW endpoint isn't stalled */
	if (dep->flags & DWC3_EP_STALL)
		__dwc3_gadget_ep_set_halt(dep, 0, false);

	reg = dwc3_readl(dwc->regs, DWC3_DALEPENA);
	reg &= ~DWC3_DALEPENA_EP(dep->number);
	dwc3_writel(dwc->regs, DWC3_DALEPENA, reg);

	dep->stream_capable = false;
	dep->endpoint.desc = NULL;
	dep->comp_desc = NULL;
	dep->type = 0;
	dep->flags = 0;

	return 0;
}

/* -------------------------------------------------------------------------- */

static int dwc3_gadget_ep0_enable(struct usb_ep *ep,
		const struct usb_endpoint_descriptor *desc)
{
	return -EINVAL;
}

static int dwc3_gadget_ep0_disable(struct usb_ep *ep)
{
	return -EINVAL;
}

/* -------------------------------------------------------------------------- */

static int dwc3_gadget_ep_enable(struct usb_ep *ep,
		const struct usb_endpoint_descriptor *desc)
{
	struct dwc3_ep			*dep;
	struct dwc3			*dwc;
	unsigned long			flags;
	int				ret;

	if (!ep || !desc || desc->bDescriptorType != USB_DT_ENDPOINT) {
		pr_debug("dwc3: invalid parameters\n");
/*		printk("dwc3: invalid parameters\n");*/
		return -EINVAL;
	}

	if (!desc->wMaxPacketSize) {
		pr_debug("dwc3: missing wMaxPacketSize\n");
		return -EINVAL;
	}

	dep = to_dwc3_ep(ep);
	dwc = dep->dwc;

	if (dep->flags & DWC3_EP_ENABLED) {
		dev_WARN_ONCE(dwc->dev, true, "%s is already enabled\n",
				dep->name);
		return 0;
	}

	switch (usb_endpoint_type(desc)) {
	case USB_ENDPOINT_XFER_CONTROL:
		strlcat(dep->name, "-control", sizeof(dep->name));
		break;
	case USB_ENDPOINT_XFER_ISOC:
		strlcat(dep->name, "-isoc", sizeof(dep->name));
		break;
	case USB_ENDPOINT_XFER_BULK:
		strlcat(dep->name, "-bulk", sizeof(dep->name));
		break;
	case USB_ENDPOINT_XFER_INT: /*<HN> Interrupt Transfer	*/
		strlcat(dep->name, "-int", sizeof(dep->name));
		break;
	default:
		dev_err(dwc->dev, "invalid endpoint transfer type\n");
	}

#ifdef	HNDEBUG /*<HN>*/
    dev_vdbg(dwc->dev, "dwc3_gadget_ep_enable-2 Type=%xH Name=%s\n", usb_endpoint_type(desc),dep->name);
#endif

	spin_lock_irqsave(&dwc->lock, flags);
	ret = __dwc3_gadget_ep_enable(dep, desc, ep->comp_desc, false, false);
	spin_unlock_irqrestore(&dwc->lock, flags);

	return ret;
}

static int dwc3_gadget_ep_disable(struct usb_ep *ep)
{
	struct dwc3_ep			*dep;
	struct dwc3			*dwc;
	unsigned long			flags;
	int				ret;
#if 0
printk(KERN_ERR "## %s entry\n",__func__);
#endif
	if (!ep) {
		pr_debug("dwc3: invalid parameters\n");
		return -EINVAL;
	}

	dep = to_dwc3_ep(ep);
	dwc = dep->dwc;

	if (!(dep->flags & DWC3_EP_ENABLED)) {
		dev_WARN_ONCE(dwc->dev, true, "%s is already disabled\n",
				dep->name);
		return 0;
	}

	snprintf(dep->name, sizeof(dep->name), "ep%d%s",
			dep->number >> 1,
			(dep->number & 1) ? "in" : "out");

	spin_lock_irqsave(&dwc->lock, flags);
	ret = __dwc3_gadget_ep_disable(dep);
	spin_unlock_irqrestore(&dwc->lock, flags);
#if 0
printk(KERN_ERR "## %s exit\n",__func__);
#endif
	return ret;
}

static struct usb_request *dwc3_gadget_ep_alloc_request(struct usb_ep *ep,
	gfp_t gfp_flags)
{
	struct dwc3_request		*req;
	struct dwc3_ep			*dep = to_dwc3_ep(ep);
	struct dwc3			*dwc = dep->dwc;
#if 0
printk(KERN_ERR "#%s entry number = %d\n",__func__, dep->number);
#endif
	req = kzalloc(sizeof(*req), gfp_flags);
	if (!req) {
		dev_err(dwc->dev, "not enough memory\n");
		return NULL;
	}

	req->epnum	= dep->number;
	req->dep	= dep;

	return &req->request;
}

static void dwc3_gadget_ep_free_request(struct usb_ep *ep,
		struct usb_request *request)
{
	struct dwc3_request		*req = to_dwc3_request(request);
#if 0
printk(KERN_ERR "#%s entry number = %d\n",__func__, req->epnum);
#endif
	kfree(req);
}

/**
 * dwc3_prepare_one_trb - setup one TRB from one request
 * @dep: endpoint for which this request is prepared
 * @req: dwc3_request pointer
 */
static void dwc3_prepare_one_trb(struct dwc3_ep *dep,
		struct dwc3_request *req, dma_addr_t dma,
		unsigned length, unsigned last, unsigned chain, unsigned node)
{
	struct dwc3		*dwc = dep->dwc;
	struct dwc3_trb		*trb;

	dev_vdbg(dwc->dev, "%s: req %p dma %08llx length %d%s%s\n",
			dep->name, req, (unsigned long long) dma,
			length, last ? " last" : "",
			chain ? " chain" : "");


	trb = &dep->trb_pool[dep->free_slot & DWC3_TRB_MASK];

	if (!req->trb) {
		dwc3_gadget_move_request_queued(req);
		req->trb = trb;
		req->trb_dma = dwc3_trb_dma_offset(dep, trb);
		req->start_slot = dep->free_slot & DWC3_TRB_MASK;
	}

	dep->free_slot++;
	/* Skip the LINK-TRB on ISOC */
	if (((dep->free_slot & DWC3_TRB_MASK) == DWC3_TRB_NUM - 1) &&
			usb_endpoint_xfer_isoc(dep->endpoint.desc))
		dep->free_slot++;

	trb->size = DWC3_TRB_SIZE_LENGTH(length);
	trb->bpl = lower_32_bits(dma);
	trb->bph = upper_32_bits(dma);

	switch (usb_endpoint_type(dep->endpoint.desc)) {
	case USB_ENDPOINT_XFER_CONTROL:
		trb->ctrl = DWC3_TRBCTL_CONTROL_SETUP;
		break;

	case USB_ENDPOINT_XFER_ISOC:
		if (!node)
			trb->ctrl = DWC3_TRBCTL_ISOCHRONOUS_FIRST;
		else
			trb->ctrl = DWC3_TRBCTL_ISOCHRONOUS;
		break;

	case USB_ENDPOINT_XFER_BULK:
	case USB_ENDPOINT_XFER_INT:
		trb->ctrl = DWC3_TRBCTL_NORMAL;
		break;
	default:
		/*
		 * This is only possible with faulty memory because we
		 * checked it already :)
		 */
		dev_err(dwc->dev, "!!! BUG  !!! <HN>\n");
		BUG();
	}

	if (!req->request.no_interrupt && !chain)
		trb->ctrl |= DWC3_TRB_CTRL_IOC;

	if (usb_endpoint_xfer_isoc(dep->endpoint.desc)) {
		trb->ctrl |= DWC3_TRB_CTRL_ISP_IMI;
		trb->ctrl |= DWC3_TRB_CTRL_CSP;
	} else if (last) {
		trb->ctrl |= DWC3_TRB_CTRL_LST;
	}

	if (chain)
		trb->ctrl |= DWC3_TRB_CTRL_CHN;

	if (usb_endpoint_xfer_bulk(dep->endpoint.desc) && dep->stream_capable)
		trb->ctrl |= DWC3_TRB_CTRL_SID_SOFN(req->request.stream_id);

	trb->ctrl |= DWC3_TRB_CTRL_HWO;
}

/*
 * dwc3_prepare_trbs - setup TRBs from requests
 * @dep: endpoint for which requests are being prepared
 * @starting: true if the endpoint is idle and no requests are queued.
 *
 * The function goes through the requests list and sets up TRBs for the
 * transfers. The function returns once there are no more TRBs available or
 * it runs out of requests.
 */
static void dwc3_prepare_trbs(struct dwc3_ep *dep, bool starting)
{
	struct dwc3_request	*req, *n;
	u32			trbs_left;
	u32			max;
	unsigned int		last_one = 0;

	BUILD_BUG_ON_NOT_POWER_OF_2(DWC3_TRB_NUM);

	/* the first request must not be queued */
	trbs_left = (dep->busy_slot - dep->free_slot) & DWC3_TRB_MASK;

	/* Can't wrap around on a non-isoc EP since there's no link TRB */
	if (!usb_endpoint_xfer_isoc(dep->endpoint.desc)) {
		max = DWC3_TRB_NUM - (dep->free_slot & DWC3_TRB_MASK);
		if (trbs_left > max)
			trbs_left = max;
	}

	/*
	 * If busy & slot are equal than it is either full or empty. If we are
	 * starting to process requests then we are empty. Otherwise we are
	 * full and don't do anything
	 */
	if (!trbs_left) {
		if (!starting)
			return;
		trbs_left = DWC3_TRB_NUM;
		/*
		 * In case we start from scratch, we queue the ISOC requests
		 * starting from slot 1. This is done because we use ring
		 * buffer and have no LST bit to stop us. Instead, we place
		 * IOC bit every TRB_NUM/4. We try to avoid having an interrupt
		 * after the first request so we start at slot 1 and have
		 * 7 requests proceed before we hit the first IOC.
		 * Other transfer types don't use the ring buffer and are
		 * processed from the first TRB until the last one. Since we
		 * don't wrap around we have to start at the beginning.
		 */
		if (usb_endpoint_xfer_isoc(dep->endpoint.desc)) {
			dep->busy_slot = 1;
			dep->free_slot = 1;
		} else {
			dep->busy_slot = 0;
			dep->free_slot = 0;
		}
	}

	/* The last TRB is a link TRB, not used for xfer */
	if ((trbs_left <= 1) && usb_endpoint_xfer_isoc(dep->endpoint.desc))
		return;

	list_for_each_entry_safe(req, n, &dep->request_list, list) {
		unsigned	length;
		dma_addr_t	dma;
#if	1	/*Original	<HN> 2015-03-05 */
		last_one = false;
#else	/* yamano 0219 added from refer source */
		last_one = true;
#endif
		if (req->request.num_mapped_sgs > 0) {
			struct usb_request *request = &req->request;
			struct scatterlist *sg = request->sg;
			struct scatterlist *s;
			int		i;

			for_each_sg(sg, s, request->num_mapped_sgs, i) {
				unsigned chain = true;

				length = sg_dma_len(s);
				dma = sg_dma_address(s);

				if (i == (request->num_mapped_sgs - 1) ||
						sg_is_last(s)) {
					if (list_is_last(&req->list,
							&dep->request_list))
						last_one = true;
					chain = false;
				}

				trbs_left--;
				if (!trbs_left)
					last_one = true;

				if (last_one)
					chain = false;

				dwc3_prepare_one_trb(dep, req, dma, length,
						last_one, chain, i);

				if (last_one)
					break;
			}
		} else {
			dma = req->request.dma;
			length = req->request.length;
			trbs_left--;

			if (!trbs_left)
				last_one = 1;

			/* Is this the last request? */
			if (list_is_last(&req->list, &dep->request_list))
				last_one = 1;

			dwc3_prepare_one_trb(dep, req, dma, length,
					last_one, false, 0);

			if (last_one)
				break;
		}
	}
}

static int __dwc3_gadget_kick_transfer(struct dwc3_ep *dep, u16 cmd_param,
		int start_new)
{
	struct dwc3_gadget_ep_cmd_params params;
	struct dwc3_request		*req;
	struct dwc3			*dwc = dep->dwc;
	int				ret;
	u32				cmd;

#if	0	/*<HN>*/
	dev_vdbg(dwc->dev, "%s with %d<HN>\n",__FUNCTION__,start_new);
#endif
	if (start_new && (dep->flags & DWC3_EP_BUSY)) {
		dev_vdbg(dwc->dev, "%s: endpoint busy\n", dep->name);
		return -EBUSY;
	}
	dep->flags &= ~DWC3_EP_PENDING_REQUEST;

	/*
	 * If we are getting here after a short-out-packet we don't enqueue any
	 * new requests as we try to set the IOC bit only on the last request.
	 */
	if (start_new) {
		if (list_empty(&dep->req_queued))
			dwc3_prepare_trbs(dep, start_new);

		/* req points to the first request which will be sent */
		req = next_request(&dep->req_queued);
	} else {
		dwc3_prepare_trbs(dep, start_new);

		/*
		 * req points to the first request where HWO changed from 0 to 1
		 */
		req = next_request(&dep->req_queued);
	}
	if (!req) {
		dep->flags |= DWC3_EP_PENDING_REQUEST;
//printk(KERN_ERR "# GADGET # %s Not Start\n",__func__);
		return 0;
	}

	memset(&params, 0, sizeof(params));

	if (start_new) {
		params.param0 = upper_32_bits(req->trb_dma);
		params.param1 = lower_32_bits(req->trb_dma);
		cmd = DWC3_DEPCMD_STARTTRANSFER;
	} else {
		cmd = DWC3_DEPCMD_UPDATETRANSFER;
	}

	cmd |= DWC3_DEPCMD_PARAM(cmd_param);
//if(start_new == 1)
//printk(KERN_ERR "# GADGET # %s Entry \n",__func__);
	ret = dwc3_send_gadget_ep_cmd(dwc, dep->number, cmd, &params);

	if (ret < 0) {
		dev_dbg(dwc->dev, "failed to send STARTTRANSFER command\n");

		/*
		 * FIXME we need to iterate over the list of requests
		 * here and stop, unmap, free and del each of the linked
		 * requests instead of what we do now.
		 */
		usb_gadget_unmap_request(&dwc->gadget, &req->request,
				req->direction);
		list_del(&req->list);
//printk(KERN_ERR "# GADGET # %s start now command failer\n",__func__);
		return ret;
	}

	dep->flags |= DWC3_EP_BUSY;

	if (start_new) {
		dep->resource_index = dwc3_gadget_ep_get_transfer_index(dwc,
				dep->number);
		WARN_ON_ONCE(!dep->resource_index);
	}

//	udelay(5);	/*<HN> Try to sync 0306	*/

	return 0;
}

static void __dwc3_gadget_start_isoc(struct dwc3 *dwc,
		struct dwc3_ep *dep, u32 cur_uf)
{
	u32 uf;

//printk(KERN_ERR "# GADGET # %s Entry \n",__func__);
	if (list_empty(&dep->request_list)) {
		dev_vdbg(dwc->dev, "ISOC ep %s run out for requests.\n",
			dep->name);
		dep->flags |= DWC3_EP_PENDING_REQUEST;
		return;
	}

	/* 4 micro frames in the future */
	uf = cur_uf + dep->interval * 4;

	__dwc3_gadget_kick_transfer(dep, uf, 1);
//printk(KERN_ERR "# GADGET # %s Exit \n",__func__);
}

static void dwc3_gadget_start_isoc(struct dwc3 *dwc,
		struct dwc3_ep *dep, const struct dwc3_event_depevt *event)
{
	u32 cur_uf, mask;

	mask = ~(dep->interval - 1);
	cur_uf = event->parameters & mask;

	__dwc3_gadget_start_isoc(dwc, dep, cur_uf);
}

static int __dwc3_gadget_ep_queue(struct dwc3_ep *dep, struct dwc3_request *req)
{
	struct dwc3		*dwc = dep->dwc;
	int			ret;

	req->request.actual	= 0;
	req->request.status	= -EINPROGRESS;
	req->direction		= dep->direction;
	req->epnum		= dep->number;

#if	0	/*<HN>*/
	dev_vdbg(dwc->dev, "%s <HN>\n",__FUNCTION__);
#endif

	/*
	 * We only add to our list of requests now and
	 * start consuming the list once we get XferNotReady
	 * IRQ.
	 *
	 * That way, we avoid doing anything that we don't need
	 * to do now and defer it until the point we receive a
	 * particular token from the Host side.
	 *
	 * This will also avoid Host cancelling URBs due to too
	 * many NAKs.
	 */
	ret = usb_gadget_map_request(&dwc->gadget, &req->request,
			dep->direction);
	if (ret)
		return ret;

	list_add_tail(&req->list, &dep->request_list);

	/*
	 * There are a few special cases:
	 *
	 * 1. XferNotReady with empty list of requests. We need to kick the
	 *    transfer here in that situation, otherwise we will be NAKing
	 *    forever. If we get XferNotReady before gadget driver has a
	 *    chance to queue a request, we will ACK the IRQ but won't be
	 *    able to receive the data until the next request is queued.
	 *    The following code is handling exactly that.
	 *
	 */
	if (dep->flags & DWC3_EP_PENDING_REQUEST) {
		/*
		 * If xfernotready is already elapsed and it is a case
		 * of isoc transfer, then issue END TRANSFER, so that
		 * you can receive xfernotready again and can have
		 * notion of current microframe.
		 */
		if (usb_endpoint_xfer_isoc(dep->endpoint.desc)) {
			if (list_empty(&dep->req_queued)) {
				dwc3_stop_active_transfer(dwc, dep->number, true);
				dep->flags = DWC3_EP_ENABLED;
			}
			return 0;
		}

		ret = __dwc3_gadget_kick_transfer(dep, 0, true);
		if (ret && ret != -EBUSY)
			dev_dbg(dwc->dev, "%s: failed to kick transfers\n",
					dep->name);
		return ret;
	}

	/*
	 * 2. XferInProgress on Isoc EP with an active transfer. We need to
	 *    kick the transfer here after queuing a request, otherwise the
	 *    core may not see the modified TRB(s).
	 */
	if (usb_endpoint_xfer_isoc(dep->endpoint.desc) &&
			(dep->flags & DWC3_EP_BUSY) &&
			!(dep->flags & DWC3_EP_MISSED_ISOC)) {
		WARN_ON_ONCE(!dep->resource_index);
		ret = __dwc3_gadget_kick_transfer(dep, dep->resource_index,
				false);
		if (ret && ret != -EBUSY)
			dev_dbg(dwc->dev, "%s: failed to kick transfers\n",
					dep->name);
		return ret;
	}

	/*
	 * 4. Stream Capable Bulk Endpoints. We need to start the transfer
	 * right away, otherwise host will not know we have streams to be
	 * handled.
	 */
	if (dep->stream_capable) {
		int	ret;

		ret = __dwc3_gadget_kick_transfer(dep, 0, true);
		if (ret && ret != -EBUSY) {
			struct dwc3	*dwc = dep->dwc;

			dev_dbg(dwc->dev, "%s: failed to kick transfers\n",
					dep->name);
		}
	}

	return 0;
}

static int dwc3_gadget_ep_queue(struct usb_ep *ep, struct usb_request *request,
	gfp_t gfp_flags)
{
	struct dwc3_request		*req = to_dwc3_request(request);
	struct dwc3_ep			*dep = to_dwc3_ep(ep);
	struct dwc3			*dwc = dep->dwc;

	unsigned long			flags;

	int				ret;

	if (!dep->endpoint.desc) {
		dev_dbg(dwc->dev, "trying to queue request %p to disabled %s\n",
				request, ep->name);
		return -ESHUTDOWN;
	}

	dev_vdbg(dwc->dev, "queing request %p to %s length %d\n",
			request, ep->name, request->length);

	spin_lock_irqsave(&dwc->lock, flags);
	ret = __dwc3_gadget_ep_queue(dep, req);
	spin_unlock_irqrestore(&dwc->lock, flags);

	return ret;
}

static int dwc3_gadget_ep_dequeue(struct usb_ep *ep,
		struct usb_request *request)
{
	struct dwc3_request		*req = to_dwc3_request(request);
	struct dwc3_request		*r = NULL;

	struct dwc3_ep			*dep = to_dwc3_ep(ep);
	struct dwc3			*dwc = dep->dwc;

	unsigned long			flags;
	int				ret = 0;

	spin_lock_irqsave(&dwc->lock, flags);

	list_for_each_entry(r, &dep->request_list, list) {
		if (r == req)
			break;
	}

	if (r != req) {
		list_for_each_entry(r, &dep->req_queued, list) {
			if (r == req)
				break;
		}
		if (r == req) {
			/* wait until it is processed */
			dwc3_stop_active_transfer(dwc, dep->number, true);
			goto out1;
		}
		dev_err(dwc->dev, "request %p was not queued to %s\n",
				request, ep->name);
		ret = -EINVAL;
		goto out0;
	}
out1:
	/* giveback the request */
	dwc3_gadget_giveback(dep, req, -ECONNRESET);

out0:
	spin_unlock_irqrestore(&dwc->lock, flags);

	return ret;
}

int __dwc3_gadget_ep_set_halt(struct dwc3_ep *dep, int value, int protocol)
{
	struct dwc3_gadget_ep_cmd_params	params;
	struct dwc3				*dwc = dep->dwc;
	int					ret;

	memset(&params, 0x00, sizeof(params));

	if (value) {
		if (!protocol && ((dep->direction && dep->flags & DWC3_EP_BUSY) ||
				(!list_empty(&dep->req_queued) ||
				 !list_empty(&dep->request_list)))) {
			dev_dbg(dwc->dev, "%s: pending request, cannot halt\n",
					dep->name);
			return -EAGAIN;
		}

		//printk(KERN_ERR "## %s Entry value\n",__func__);
		ret = dwc3_send_gadget_ep_cmd(dwc, dep->number,
			DWC3_DEPCMD_SETSTALL, &params);
		if (ret)
			dev_err(dwc->dev, "failed to %s STALL on %s\n",
					value ? "set" : "clear",
					dep->name);
		else
			dep->flags |= DWC3_EP_STALL;
	} else {
#if 0
printk(KERN_ERR "## %s Entry not value\n",__func__);
#endif
		ret = dwc3_send_gadget_ep_cmd(dwc, dep->number,
			DWC3_DEPCMD_CLEARSTALL, &params);
		if (ret)
			dev_err(dwc->dev, "failed to %s STALL on %s\n",
					value ? "set" : "clear",
					dep->name);
		else
			dep->flags &= ~(DWC3_EP_STALL | DWC3_EP_WEDGE);
	}

	return ret;
}

static int dwc3_gadget_ep_set_halt(struct usb_ep *ep, int value)
{
	struct dwc3_ep			*dep = to_dwc3_ep(ep);
	struct dwc3			*dwc = dep->dwc;

	unsigned long			flags;

	int				ret;

printk(KERN_ERR "## %s Entry\n",__func__);
	spin_lock_irqsave(&dwc->lock, flags);

	if (usb_endpoint_xfer_isoc(dep->endpoint.desc)) {
		dev_err(dwc->dev, "%s is of Isochronous type\n", dep->name);
		ret = -EINVAL;
		goto out;
	}

	ret = __dwc3_gadget_ep_set_halt(dep, value, false);
out:
	spin_unlock_irqrestore(&dwc->lock, flags);

	return ret;
}

static int dwc3_gadget_ep_set_wedge(struct usb_ep *ep)
{
	struct dwc3_ep			*dep = to_dwc3_ep(ep);
	struct dwc3			*dwc = dep->dwc;
	unsigned long			flags;

	spin_lock_irqsave(&dwc->lock, flags);
	dep->flags |= DWC3_EP_WEDGE;
	spin_unlock_irqrestore(&dwc->lock, flags);

	if (dep->number == 0 || dep->number == 1)
		return dwc3_gadget_ep0_set_halt(ep, 1);
	else
		return __dwc3_gadget_ep_set_halt(dep, 1, false);
}

/* -------------------------------------------------------------------------- */

static struct usb_endpoint_descriptor dwc3_gadget_ep0_desc = {
	.bLength	= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bmAttributes	= USB_ENDPOINT_XFER_CONTROL,
};

static const struct usb_ep_ops dwc3_gadget_ep0_ops = {
	.enable		= dwc3_gadget_ep0_enable,
	.disable	= dwc3_gadget_ep0_disable,
	.alloc_request	= dwc3_gadget_ep_alloc_request,
	.free_request	= dwc3_gadget_ep_free_request,
	.queue		= dwc3_gadget_ep0_queue,
	.dequeue	= dwc3_gadget_ep_dequeue,
	.set_halt	= dwc3_gadget_ep0_set_halt,
	.set_wedge	= dwc3_gadget_ep_set_wedge,
};

static const struct usb_ep_ops dwc3_gadget_ep_ops = {
	.enable		= dwc3_gadget_ep_enable,
	.disable	= dwc3_gadget_ep_disable,
	.alloc_request	= dwc3_gadget_ep_alloc_request,
	.free_request	= dwc3_gadget_ep_free_request,
	.queue		= dwc3_gadget_ep_queue,
	.dequeue	= dwc3_gadget_ep_dequeue,
	.set_halt	= dwc3_gadget_ep_set_halt,
	.set_wedge	= dwc3_gadget_ep_set_wedge,
};

/* -------------------------------------------------------------------------- */

static int dwc3_gadget_get_frame(struct usb_gadget *g)
{
	struct dwc3		*dwc = gadget_to_dwc(g);
	u32			reg;

	reg = dwc3_readl(dwc->regs, DWC3_DSTS);
	return DWC3_DSTS_SOFFN(reg);
}

static int dwc3_gadget_wakeup(struct usb_gadget *g)
{
	struct dwc3		*dwc = gadget_to_dwc(g);

	unsigned long		timeout;
	unsigned long		flags;

	u32			reg;

	int			ret = 0;

	u8			link_state;
	u8			speed;

#if	0	/*<HN>*/
	dev_vdbg(dwc->dev, "%s <HN>\n",__FUNCTION__);
#endif
	spin_lock_irqsave(&dwc->lock, flags);

	/*
	 * According to the Databook Remote wakeup request should
	 * be issued only when the device is in early suspend state.
	 *
	 * We can check that via USB Link State bits in DSTS register.
	 */
	reg = dwc3_readl(dwc->regs, DWC3_DSTS);

	speed = reg & DWC3_DSTS_CONNECTSPD;
	if (speed == DWC3_DSTS_SUPERSPEED) {
		dev_dbg(dwc->dev, "no wakeup on SuperSpeed\n");
		ret = -EINVAL;
		goto out;
	}

	link_state = DWC3_DSTS_USBLNKST(reg);

	switch (link_state) {
	case DWC3_LINK_STATE_RX_DET:	/* in HS, means Early Suspend */
	case DWC3_LINK_STATE_U3:	/* in HS, means SUSPEND */
		break;
	default:
		dev_dbg(dwc->dev, "can't wakeup from link state %d\n",
				link_state);
		ret = -EINVAL;
		goto out;
	}

	ret = dwc3_gadget_set_link_state(dwc, DWC3_LINK_STATE_RECOV);
	if (ret < 0) {
		dev_err(dwc->dev, "failed to put link in Recovery\n");
		goto out;
	}

	/* Recent versions do this automatically */
	if (dwc->revision < DWC3_REVISION_194A) {
		/* write zeroes to Link Change Request */
		reg = dwc3_readl(dwc->regs, DWC3_DCTL);
		reg &= ~DWC3_DCTL_ULSTCHNGREQ_MASK;
		dwc3_writel(dwc->regs, DWC3_DCTL, reg);
	}

	/* poll until Link State changes to ON */
/*	timeout = jiffies + msecs_to_jiffies(100);*/
	timeout = jiffies + msecs_to_jiffies(100)*5;/* <HN> added *4*/

	while (!time_after(jiffies, timeout)) {
		reg = dwc3_readl(dwc->regs, DWC3_DSTS);

		/* in HS, means ON */
		if (DWC3_DSTS_USBLNKST(reg) == DWC3_LINK_STATE_U0)
			break;
	}

	if (DWC3_DSTS_USBLNKST(reg) != DWC3_LINK_STATE_U0) {
		dev_err(dwc->dev, "failed to send remote wakeup\n");
		ret = -EINVAL;
	}

out:
	spin_unlock_irqrestore(&dwc->lock, flags);

	return ret;
}

static int dwc3_gadget_set_selfpowered(struct usb_gadget *g,
		int is_selfpowered)
{
	struct dwc3		*dwc = gadget_to_dwc(g);
	unsigned long		flags;

	spin_lock_irqsave(&dwc->lock, flags);
	dwc->is_selfpowered = !!is_selfpowered;
	spin_unlock_irqrestore(&dwc->lock, flags);

	return 0;
}

static int dwc3_gadget_run_stop(struct dwc3 *dwc, int is_on, int suspend)
{
	u32			reg;
	u32			timeout = 500;

#if	0	/*<HN>*/
	dev_vdbg(dwc->dev, "%s <HN>\n",__FUNCTION__);
#endif
#if 0
	printk( KERN_ERR "## %s entry is_on = %d\n",__func__,is_on);
#endif
	reg = dwc3_readl(dwc->regs, DWC3_DCTL);
	if (is_on) {
		if (dwc->revision <= DWC3_REVISION_187A) {
			reg &= ~DWC3_DCTL_TRGTULST_MASK;
			reg |= DWC3_DCTL_TRGTULST_RX_DET;
		}

		if (dwc->revision >= DWC3_REVISION_194A)
			reg &= ~DWC3_DCTL_KEEP_CONNECT;
		reg |= DWC3_DCTL_RUN_STOP;

		if (dwc->has_hibernation)
			reg |= DWC3_DCTL_KEEP_CONNECT;

		dwc->pullups_connected = true;
#if WORKAROUDN_55XXA0_DP_ISSUE
		phy_dp_pull_up(1, 1);
#endif
	} else {
		reg &= ~DWC3_DCTL_RUN_STOP;

		if (dwc->has_hibernation && !suspend)
			reg &= ~DWC3_DCTL_KEEP_CONNECT;

		dwc->pullups_connected = false;
#if WORKAROUDN_55XXA0_DP_ISSUE
		phy_dp_pull_up(0, 1);
#endif
	}

	dwc3_writel(dwc->regs, DWC3_DCTL, reg);

	do {
		reg = dwc3_readl(dwc->regs, DWC3_DSTS);
		if (is_on) {
			if (!(reg & DWC3_DSTS_DEVCTRLHLT))
				break;
		} else {
			if (reg & DWC3_DSTS_DEVCTRLHLT)
				break;
		}
		timeout--;
		if (!timeout)
			return -ETIMEDOUT;
		udelay(1);
	} while (1);

	dev_vdbg(dwc->dev, "gadget %s data soft-%s\n",
			dwc->gadget_driver
			? dwc->gadget_driver->function : "no-function",
			is_on ? "connect" : "disconnect");

#if WORKAROUDN_55XXA0_DP_ISSUE
	/* 55XX A0 D+ workaround */
//printk(KERN_ERR "%s phy_dp_pull_up 1,1 exit\n",__FUNCTION__);
//	phy_dp_pull_up(1, 1);
#endif
#if 0
	printk(KERN_ERR "## %s Normal exit\n",__func__);
#endif
	return 0;
}

static int dwc3_gadget_pullup(struct usb_gadget *g, int is_on)
{
	struct dwc3		*dwc = gadget_to_dwc(g);
	unsigned long		flags;
	int			ret;

	is_on = !!is_on;
#if 0
printk( KERN_ERR "## %s entry\n",__func__);
#endif
	spin_lock_irqsave(&dwc->lock, flags);
	ret = dwc3_gadget_run_stop(dwc, is_on, false);
	spin_unlock_irqrestore(&dwc->lock, flags);

	return ret;
}

static void dwc3_gadget_enable_irq(struct dwc3 *dwc)
{
	u32			reg;

	/* Enable all but Start and End of Frame IRQs */
#if	0	/*	<HN ORG>*/
	reg = (DWC3_DEVTEN_VNDRDEVTSTRCVEDEN |
			DWC3_DEVTEN_EVNTOVERFLOWEN |
			DWC3_DEVTEN_CMDCMPLTEN |
			DWC3_DEVTEN_ERRTICERREN |
			DWC3_DEVTEN_WKUPEVTEN |
			DWC3_DEVTEN_ULSTCNGEN |
			DWC3_DEVTEN_CONNECTDONEEN |
			DWC3_DEVTEN_USBRSTEN |
			DWC3_DEVTEN_DISCONNEVTEN);
#else
	reg = (DWC3_DEVTEN_VNDRDEVTSTRCVEDEN |
			DWC3_DEVTEN_EVNTOVERFLOWEN |
			DWC3_DEVTEN_CMDCMPLTEN |
			DWC3_DEVTEN_ERRTICERREN |
			DWC3_DEVTEN_WKUPEVTEN |
			DWC3_DEVTEN_CONNECTDONEEN |
			DWC3_DEVTEN_USBRSTEN |
			DWC3_DEVTEN_DISCONNEVTEN);
#endif
	dwc3_writel(dwc->regs, DWC3_DEVTEN, reg);
}

static void dwc3_gadget_disable_irq(struct dwc3 *dwc)
{
	/* mask all interrupts */
	dwc3_writel(dwc->regs, DWC3_DEVTEN, 0x00);
}

static irqreturn_t dwc3_interrupt(int irq, void *_dwc);
static irqreturn_t dwc3_thread_interrupt(int irq, void *_dwc);

int dwc3_gadget_restart(struct dwc3 *dwc)
{
	int			ret = 0;
 	u32			reg;
	struct dwc3_ep		*dep;

	#ifdef	HNDEBUG
	//	printk("%s DWC3_DFCG = %x P1=%pH P2=%pH <HN>\n",__FUNCTION__,g,driver);
	#endif

	dwc->has_gadget = true;
	reg = dwc3_readl(dwc->regs, DWC3_DCFG);
	reg &= ~(DWC3_DCFG_SPEED_MASK);

	/**
	 * WORKAROUND: DWC3 revision < 2.20a have an issue
	 * which would cause metastability state on Run/Stop
	 * bit if we try to force the IP to USB2-only mode.
	 *
	 * Because of that, we cannot configure the IP to any
	 * speed other than the SuperSpeed
	 *
	 * Refers to:
	 *
	 * STAR#9000525659: Clock Domain Crossing on DCTL in
	 * USB 2.0 Mode
	 */
	if (dwc->revision < DWC3_REVISION_220A) {
		reg |= DWC3_DCFG_SUPERSPEED;
	} else {
		switch (dwc->maximum_speed) {
		case USB_SPEED_LOW:
			reg |= DWC3_DSTS_LOWSPEED;
			break;
		case USB_SPEED_FULL:
			reg |= DWC3_DSTS_FULLSPEED1;
			break;
		case USB_SPEED_HIGH:
			reg |= DWC3_DSTS_HIGHSPEED;
			break;
		case USB_SPEED_SUPER:	/* FALLTHROUGH */
		case USB_SPEED_UNKNOWN:	/* FALTHROUGH */
		default:
			reg |= DWC3_DSTS_SUPERSPEED;
		}
	}
//printk( KERN_ERR "%s DWC3_DFCG = %x\n",__FUNCTION__,reg);
	dwc3_writel(dwc->regs, DWC3_DCFG, reg);

	dwc->start_config_issued = false;

	/* Start with SuperSpeed Default */
	dwc3_gadget_ep0_desc.wMaxPacketSize = cpu_to_le16(512);

	dep = dwc->eps[0];
	ret = __dwc3_gadget_ep_enable(dep, &dwc3_gadget_ep0_desc, NULL, false,
			false);
	if (ret) {
		dev_err(dwc->dev, "failed to enable %s\n", dep->name);
		goto err0;
	}

	dep = dwc->eps[1];
	ret = __dwc3_gadget_ep_enable(dep, &dwc3_gadget_ep0_desc, NULL, false,
			false);
	if (ret) {
		dev_err(dwc->dev, "failed to enable %s\n", dep->name);
		goto err1;
	}

	/* begin to receive SETUP packets */
	dwc->ep0state = EP0_SETUP_PHASE;
	dwc3_ep0_out_start(dwc);

	dwc3_gadget_enable_irq(dwc);

	return 0;

err1:
	__dwc3_gadget_ep_disable(dwc->eps[0]);

err0:
	dwc->gadget_driver = NULL;

	return ret;
}


static int dwc3_gadget_start(struct usb_gadget *g,
		struct usb_gadget_driver *driver)
{
	struct dwc3		*dwc = gadget_to_dwc(g);
	unsigned long		flags;
	int			ret = 0;
	int			irq;

	irq = platform_get_irq(to_platform_device(dwc->dev), 0);
	ret = request_threaded_irq(irq, dwc3_interrupt, dwc3_thread_interrupt,
			IRQF_SHARED, "dwc3", dwc);
	if (ret) {
		dev_err(dwc->dev, "failed to request irq #%d --> %d\n",
				irq, ret);
		goto err0;
	}

	spin_lock_irqsave(&dwc->lock, flags);

	if (dwc->gadget_driver) {
		dev_err(dwc->dev, "%s is already bound to %s\n",
				dwc->gadget.name,
				dwc->gadget_driver->driver.name);
		ret = -EBUSY;
		goto err1;
	}

	dwc->gadget_driver	= driver;

	ret = dwc3_gadget_restart(dwc);
	if (ret)
		goto err1;

	spin_unlock_irqrestore(&dwc->lock, flags);

	return 0;
err1:
 	spin_unlock_irqrestore(&dwc->lock, flags);
	free_irq(irq, dwc);
err0:
 	return ret; 
}


int dwc3_gadget_stop_on_switch(struct dwc3 *dwc)
{
	dwc->has_gadget = false;
	__dwc3_gadget_ep_disable(dwc->eps[0]);
	__dwc3_gadget_ep_disable(dwc->eps[1]);

	return 0;
}


static int dwc3_gadget_stop(struct usb_gadget *g,
		struct usb_gadget_driver *driver)
{
	struct dwc3		*dwc = gadget_to_dwc(g);
	unsigned long		flags;
	int			irq;

#if	0	/*<HN>*/
	printk(KERN_ERR "%s <HN>\n",__FUNCTION__);
#endif
	spin_lock_irqsave(&dwc->lock, flags);

	dwc3_gadget_disable_irq(dwc);
	__dwc3_gadget_ep_disable(dwc->eps[0]);
	__dwc3_gadget_ep_disable(dwc->eps[1]);

	dwc->gadget_driver	= NULL;

	spin_unlock_irqrestore(&dwc->lock, flags);

	irq = platform_get_irq(to_platform_device(dwc->dev), 0);
	free_irq(irq, dwc);

	return 0;
}

static const struct usb_gadget_ops dwc3_gadget_ops = {
	.get_frame		= dwc3_gadget_get_frame,
	.wakeup			= dwc3_gadget_wakeup,
	.set_selfpowered	= dwc3_gadget_set_selfpowered,
	.pullup			= dwc3_gadget_pullup,
	.udc_start		= dwc3_gadget_start,
	.udc_stop		= dwc3_gadget_stop,
};

/* -------------------------------------------------------------------------- */

static int dwc3_gadget_init_hw_endpoints(struct dwc3 *dwc,
		u8 num, u32 direction)
{
	struct dwc3_ep			*dep;
	u8				i;

	for (i = 0; i < num; i++) {
		u8 epnum = (i << 1) | (!!direction);

		dep = kzalloc(sizeof(*dep), GFP_KERNEL);
		if (!dep) {
			dev_err(dwc->dev, "can't allocate endpoint %d\n",
					epnum);
			return -ENOMEM;
		}

		dep->dwc = dwc;
		dep->number = epnum;
		dep->direction = !!direction;
		dwc->eps[epnum] = dep;

		snprintf(dep->name, sizeof(dep->name), "ep%d%s", epnum >> 1,
				(epnum & 1) ? "in" : "out");

		dep->endpoint.name = dep->name;

		dev_vdbg(dwc->dev, "initializing %s\n", dep->name);

		if (epnum == 0 || epnum == 1) {
			usb_ep_set_maxpacket_limit(&dep->endpoint, 512);
			dep->endpoint.maxburst = 1;
			dep->endpoint.ops = &dwc3_gadget_ep0_ops;
			if (!epnum)
				dwc->gadget.ep0 = &dep->endpoint;
		} else {
			int		ret;

			usb_ep_set_maxpacket_limit(&dep->endpoint, 1024);
			dep->endpoint.max_streams = 15;
			dep->endpoint.ops = &dwc3_gadget_ep_ops;
			list_add_tail(&dep->endpoint.ep_list,
					&dwc->gadget.ep_list);

			ret = dwc3_alloc_trb_pool(dep);
			if (ret)
				return ret;
		}

		INIT_LIST_HEAD(&dep->request_list);
		INIT_LIST_HEAD(&dep->req_queued);
	}

	return 0;
}

static int dwc3_gadget_init_endpoints(struct dwc3 *dwc)
{
	int				ret;

	INIT_LIST_HEAD(&dwc->gadget.ep_list);

	ret = dwc3_gadget_init_hw_endpoints(dwc, dwc->num_out_eps, 0);
	if (ret < 0) {
		dev_vdbg(dwc->dev, "failed to allocate OUT endpoints\n");
		return ret;
	}

	ret = dwc3_gadget_init_hw_endpoints(dwc, dwc->num_in_eps, 1);
	if (ret < 0) {
		dev_vdbg(dwc->dev, "failed to allocate IN endpoints\n");
		return ret;
	}

	return 0;
}

static void dwc3_gadget_free_endpoints(struct dwc3 *dwc)
{
	struct dwc3_ep			*dep;
	u8				epnum;

	for (epnum = 0; epnum < DWC3_ENDPOINTS_NUM; epnum++) {
		dep = dwc->eps[epnum];
		if (!dep)
			continue;
		/*
		 * Physical endpoints 0 and 1 are special; they form the
		 * bi-directional USB endpoint 0.
		 *
		 * For those two physical endpoints, we don't allocate a TRB
		 * pool nor do we add them the endpoints list. Due to that, we
		 * shouldn't do these two operations otherwise we would end up
		 * with all sorts of bugs when removing dwc3.ko.
		 */
		if (epnum != 0 && epnum != 1) {
			dwc3_free_trb_pool(dep);
			list_del(&dep->endpoint.ep_list);
		}

		kfree(dep);
	}
}

/* -------------------------------------------------------------------------- */

static int __dwc3_cleanup_done_trbs(struct dwc3 *dwc, struct dwc3_ep *dep,
		struct dwc3_request *req, struct dwc3_trb *trb,
		const struct dwc3_event_depevt *event, int status)
{
	unsigned int		count;
	unsigned int		s_pkt = 0;
	unsigned int		trb_status;

#if	0	/*Too Much <HN> */
	printk(KERN_ERR "%s <HN>\n",__FUNCTION__);
#endif
	if ((trb->ctrl & DWC3_TRB_CTRL_HWO) && status != -ESHUTDOWN)
		/*
		 * We continue despite the error. There is not much we
		 * can do. If we don't clean it up we loop forever. If
		 * we skip the TRB then it gets overwritten after a
		 * while since we use them in a ring buffer. A BUG()
		 * would help. Lets hope that if this occurs, someone
		 * fixes the root cause instead of looking away :)
		 */
		dev_err(dwc->dev, "%s's TRB (%p) still owned by HW\n",
				dep->name, trb);
	count = trb->size & DWC3_TRB_SIZE_MASK;

	if (dep->direction) {
		if (count) {
			trb_status = DWC3_TRB_SIZE_TRBSTS(trb->size);
			if (trb_status == DWC3_TRBSTS_MISSED_ISOC) {
				dev_dbg(dwc->dev, "incomplete IN transfer %s\n",
						dep->name);
				/*
				 * If missed isoc occurred and there is
				 * no request queued then issue END
				 * TRANSFER, so that core generates
				 * next xfernotready and we will issue
				 * a fresh START TRANSFER.
				 * If there are still queued request
				 * then wait, do not issue either END
				 * or UPDATE TRANSFER, just attach next
				 * request in request_list during
				 * giveback.If any future queued request
				 * is successfully transferred then we
				 * will issue UPDATE TRANSFER for all
				 * request in the request_list.
				 */
				dep->flags |= DWC3_EP_MISSED_ISOC;
			} else {
				dev_err(dwc->dev, "incomplete IN transfer %s\n",
						dep->name);
				status = -ECONNRESET;
			}
		} else {
			dep->flags &= ~DWC3_EP_MISSED_ISOC;
		}
	} else {
		if (count && (event->status & DEPEVT_STATUS_SHORT))
			s_pkt = 1;
	}

	/*
	 * We assume here we will always receive the entire data block
	 * which we should receive. Meaning, if we program RX to
	 * receive 4K but we receive only 2K, we assume that's all we
	 * should receive and we simply bounce the request back to the
	 * gadget driver for further processing.
	 */
	req->request.actual += req->request.length - count;
	if (s_pkt)
		return 1;
	if ((event->status & DEPEVT_STATUS_LST) &&
			(trb->ctrl & (DWC3_TRB_CTRL_LST |
				DWC3_TRB_CTRL_HWO)))
		return 1;
	if ((event->status & DEPEVT_STATUS_IOC) &&
			(trb->ctrl & DWC3_TRB_CTRL_IOC))
		return 1;
	return 0;
}

static int dwc3_cleanup_done_reqs(struct dwc3 *dwc, struct dwc3_ep *dep,
		const struct dwc3_event_depevt *event, int status)
{
	struct dwc3_request	*req;
	struct dwc3_trb		*trb;
	unsigned int		slot;
	unsigned int		i;
	int			ret;

#if	0	/*Too Much <<HN>*/
	printk(KERN_ERR "%s <HN>\n",__FUNCTION__);
#endif
	do {
		req = next_request(&dep->req_queued);
		if (!req) {
			WARN_ON_ONCE(1);
			return 1;
		}
		i = 0;
		do {
			slot = req->start_slot + i;
			if ((slot == DWC3_TRB_NUM - 1) &&
				usb_endpoint_xfer_isoc(dep->endpoint.desc))
				slot++;
			slot %= DWC3_TRB_NUM;
			trb = &dep->trb_pool[slot];

			ret = __dwc3_cleanup_done_trbs(dwc, dep, req, trb,
					event, status);
			if (ret)
				break;
		}while (++i < req->request.num_mapped_sgs);

		dwc3_gadget_giveback(dep, req, status);

		if (ret)
			break;
	} while (1);

	if (usb_endpoint_xfer_isoc(dep->endpoint.desc) &&
			list_empty(&dep->req_queued)) {
		if (list_empty(&dep->request_list)) {
			/*
			 * If there is no entry in request list then do
			 * not issue END TRANSFER now. Just set PENDING
			 * flag, so that END TRANSFER is issued when an
			 * entry is added into request list.
			 */
			dep->flags = DWC3_EP_PENDING_REQUEST;
		} else {
			dwc3_stop_active_transfer(dwc, dep->number, true);
			dep->flags = DWC3_EP_ENABLED;
		}
		return 1;
	}

	return 1;
}

static void dwc3_endpoint_transfer_complete(struct dwc3 *dwc,
		struct dwc3_ep *dep, const struct dwc3_event_depevt *event,
		int start_new)
{
	unsigned		status = 0;
	int			clean_busy;

#if	0	/*Too Much <HN>*/
	printk(KERN_ERR "%s <HN>\n",__FUNCTION__);
#endif
	if (event->status & DEPEVT_STATUS_BUSERR)
		status = -ECONNRESET;

	clean_busy = dwc3_cleanup_done_reqs(dwc, dep, event, status);
	if (clean_busy)
		dep->flags &= ~DWC3_EP_BUSY;

	/*
	 * WORKAROUND: This is the 2nd half of U1/U2 -> U0 workaround.
	 * See dwc3_gadget_linksts_change_interrupt() for 1st half.
	 */
	if (dwc->revision < DWC3_REVISION_183A) {
		u32		reg;
		int		i;

		for (i = 0; i < DWC3_ENDPOINTS_NUM; i++) {
			dep = dwc->eps[i];

			if (!(dep->flags & DWC3_EP_ENABLED))
				continue;

			if (!list_empty(&dep->req_queued))
				return;
		}

		reg = dwc3_readl(dwc->regs, DWC3_DCTL);
		reg |= dwc->u1u2;
		dwc3_writel(dwc->regs, DWC3_DCTL, reg);

		dwc->u1u2 = 0;
	}
}

static void dwc3_endpoint_interrupt(struct dwc3 *dwc,
		const struct dwc3_event_depevt *event)
{
	struct dwc3_ep		*dep;
	u8			epnum = event->endpoint_number;

	dep = dwc->eps[epnum];

	if (!(dep->flags & DWC3_EP_ENABLED))
		return;

//printk(KERN_ERR "%s %s: %s\n",__FUNCTION__, dep->name,	/* yamano */
	dev_vdbg(dwc->dev, "%s: %s\n", dep->name,
			dwc3_ep_event_string(event->endpoint_event));

	if (epnum == 0 || epnum == 1) {
		dwc3_ep0_interrupt(dwc, event);
		return;
	}

	switch (event->endpoint_event) {
	case DWC3_DEPEVT_XFERCOMPLETE:
		dep->resource_index = 0;

		if (usb_endpoint_xfer_isoc(dep->endpoint.desc)) {
			dev_dbg(dwc->dev, "%s is an Isochronous endpoint\n",
					dep->name);
			return;
		}

		dwc3_endpoint_transfer_complete(dwc, dep, event, 1);
		break;
	case DWC3_DEPEVT_XFERINPROGRESS:
		if (!usb_endpoint_xfer_isoc(dep->endpoint.desc)) {
			dev_dbg(dwc->dev, "%s is not an Isochronous endpoint\n",
					dep->name);
			return;
		}

		dwc3_endpoint_transfer_complete(dwc, dep, event, 0);
		break;
	case DWC3_DEPEVT_XFERNOTREADY:
		if (usb_endpoint_xfer_isoc(dep->endpoint.desc)) {
			dwc3_gadget_start_isoc(dwc, dep, event);
		} else {
			int ret;

			dev_vdbg(dwc->dev, "%s: reason %s\n",
					dep->name, event->status &
					DEPEVT_STATUS_TRANSFER_ACTIVE
					? "Transfer Active"
					: "Transfer Not Active");
//printk(KERN_ERR "# GADGET # %s XFERNOTREADY\n",__func__);
			ret = __dwc3_gadget_kick_transfer(dep, 0, 1);
//printk(KERN_ERR "# GADGET # %s kick start XFERNOTREADY\n",__func__);
			if (!ret || ret == -EBUSY)
				return;

			dev_dbg(dwc->dev, "%s: failed to kick transfers\n",
					dep->name);
		}

		break;
	case DWC3_DEPEVT_STREAMEVT:
		if (!usb_endpoint_xfer_bulk(dep->endpoint.desc)) {
			dev_err(dwc->dev, "Stream event for non-Bulk %s\n",
					dep->name);
			return;
		}

		switch (event->status) {
		case DEPEVT_STREAMEVT_FOUND:
			dev_vdbg(dwc->dev, "Stream %d found and started\n",
					event->parameters);

			break;
		case DEPEVT_STREAMEVT_NOTFOUND:
			/* FALLTHROUGH */
		default:
			dev_dbg(dwc->dev, "Couldn't find suitable stream\n");
			break;/* <HN> */
		}
		break;
	case DWC3_DEPEVT_RXTXFIFOEVT:
//printk(KERN_ERR "%s FIFO Overrun\n", dep->name);
		dev_dbg(dwc->dev, "%s FIFO Overrun\n", dep->name);
		break;
	case DWC3_DEPEVT_EPCMDCMPLT:
		
//printk(KERN_ERR "%s Endpoint Command Complete\n",__func__);
		dev_dbg(dwc->dev, "Endpoint Command Complete\n");
		break;
	default:
		dev_dbg(dwc->dev, "Unknown Endpoint Command %xH <HN>\n",event->endpoint_event);
		break;
	}
}

static void dwc3_disconnect_gadget(struct dwc3 *dwc)
{
#if 0
printk(KERN_ERR "## %s entry \n",__func__);
#endif
	if (dwc->gadget_driver && dwc->gadget_driver->disconnect) {
		spin_unlock(&dwc->lock);
		dwc->gadget_driver->disconnect(&dwc->gadget);
		spin_lock(&dwc->lock);
	}
}

static void dwc3_suspend_gadget(struct dwc3 *dwc)
{
	if (dwc->gadget_driver && dwc->gadget_driver->suspend) {
		spin_unlock(&dwc->lock);
		dwc->gadget_driver->suspend(&dwc->gadget);
		spin_lock(&dwc->lock);
	}
}

static void dwc3_resume_gadget(struct dwc3 *dwc)
{
	if (dwc->gadget_driver && dwc->gadget_driver->resume) {
		spin_unlock(&dwc->lock);
		dwc->gadget_driver->resume(&dwc->gadget);
		spin_lock(&dwc->lock);
	}
}

static void dwc3_stop_active_transfer(struct dwc3 *dwc, u32 epnum, bool force)
{
	struct dwc3_ep *dep;
	struct dwc3_gadget_ep_cmd_params params;
	u32 cmd;
	int ret;
//printk(KERN_ERR "%s entry\n",__FUNCTION__);

	dep = dwc->eps[epnum];

	if (!dep->resource_index)
		return;

	/*
	 * NOTICE: We are violating what the Databook says about the
	 * EndTransfer command. Ideally we would _always_ wait for the
	 * EndTransfer Command Completion IRQ, but that's causing too
	 * much trouble synchronizing between us and gadget driver.
	 *
	 * We have discussed this with the IP Provider and it was
	 * suggested to giveback all requests here, but give HW some
	 * extra time to synchronize with the interconnect. We're using
	 * an arbitraty 100us delay for that.
	 *
	 * Note also that a similar handling was tested by Synopsys
	 * (thanks a lot Paul) and nothing bad has come out of it.
	 * In short, what we're doing is:
	 *
	 * - Issue EndTransfer WITH CMDIOC bit set
	 * - Wait 100us
	 */

	cmd = DWC3_DEPCMD_ENDTRANSFER;
	cmd |= force ? DWC3_DEPCMD_HIPRI_FORCERM : 0;
	cmd |= DWC3_DEPCMD_CMDIOC;
	cmd |= DWC3_DEPCMD_PARAM(dep->resource_index);
	memset(&params, 0, sizeof(params));
//printk(KERN_ERR "## %s Entry \n",__func__);
	ret = dwc3_send_gadget_ep_cmd(dwc, dep->number, cmd, &params);
	WARN_ON_ONCE(ret);
	dep->resource_index = 0;
	dep->flags &= ~DWC3_EP_BUSY;
#if	0	/*	<HN>	*/
	udelay(1000);
#else	/*Original*/
	udelay(100);
#endif
}

static void dwc3_stop_active_transfers(struct dwc3 *dwc)
{
	u32 epnum;

	for (epnum = 2; epnum < DWC3_ENDPOINTS_NUM; epnum++) {
		struct dwc3_ep *dep;

		dep = dwc->eps[epnum];
		if (!dep)
			continue;

		if (!(dep->flags & DWC3_EP_ENABLED))
			continue;

		dwc3_remove_requests(dwc, dep);
	}
}

static void dwc3_clear_stall_all_ep(struct dwc3 *dwc)
{
	u32 epnum;

	for (epnum = 1; epnum < DWC3_ENDPOINTS_NUM; epnum++) {
		struct dwc3_ep *dep;
		struct dwc3_gadget_ep_cmd_params params;
		int ret;

		dep = dwc->eps[epnum];
		if (!dep)
			continue;

		if (!(dep->flags & DWC3_EP_STALL))
			continue;

		dep->flags &= ~DWC3_EP_STALL;

		memset(&params, 0, sizeof(params));
//printk(KERN_ERR "## %s Entry \n",__func__);
		ret = dwc3_send_gadget_ep_cmd(dwc, dep->number,
				DWC3_DEPCMD_CLEARSTALL, &params);
		WARN_ON_ONCE(ret);
	}
}

static void dwc3_gadget_disconnect_interrupt(struct dwc3 *dwc)
{
	int			reg;

	dev_vdbg(dwc->dev, "%s\n", __func__);

	reg = dwc3_readl(dwc->regs, DWC3_DCTL);
	reg &= ~DWC3_DCTL_INITU1ENA;
	dwc3_writel(dwc->regs, DWC3_DCTL, reg);

	reg &= ~DWC3_DCTL_INITU2ENA;
	dwc3_writel(dwc->regs, DWC3_DCTL, reg);

	dwc3_disconnect_gadget(dwc);
	dwc->start_config_issued = false;

	dwc->gadget.speed = USB_SPEED_UNKNOWN;
	dwc->setup_packet_pending = false;

#if WORKAROUDN_55XXA0_DP_ISSUE
	/* 55XX A0 D+ workaround */
//printk(KERN_ERR "%s 55xx  phy_dp_pull_up 1,1 exit\n",__FUNCTION__);			
	phy_dp_pull_up(1, 1);
#endif
}

static void dwc3_gadget_reset_interrupt(struct dwc3 *dwc)
{
	u32			reg;

	dev_vdbg(dwc->dev, "%s\n", __func__);
//printk(KERN_ERR "## %s entry\n",__FUNCTION__);			

	/*
	 * WORKAROUND: DWC3 revisions <1.88a have an issue which
	 * would cause a missing Disconnect Event if there's a
	 * pending Setup Packet in the FIFO.
	 *
	 * There's no suggested workaround on the official Bug
	 * report, which states that "unless the driver/application
	 * is doing any special handling of a disconnect event,
	 * there is no functional issue".
	 *
	 * Unfortunately, it turns out that we _do_ some special
	 * handling of a disconnect event, namely complete all
	 * pending transfers, notify gadget driver of the
	 * disconnection, and so on.
	 *
	 * Our suggested workaround is to follow the Disconnect
	 * Event steps here, instead, based on a setup_packet_pending
	 * flag. Such flag gets set whenever we have a XferNotReady
	 * event on EP0 and gets cleared on XferComplete for the
	 * same endpoint.
	 *
	 * Refers to:
	 *
	 * STAR#9000466709: RTL: Device : Disconnect event not
	 * generated if setup packet pending in FIFO
	 */
//	if (dwc->revision < DWC3_REVISION_188A) {
//		if (dwc->setup_packet_pending)
//			dwc3_gadget_disconnect_interrupt(dwc);
//	}

#if WORKAROUDN_55XXA0_DP_ISSUE
	if (dwc->gadget.speed == USB_SPEED_UNKNOWN)
		/* 55XX A0 D+ workaround */
//printk(KERN_ERR "%s reset interrupt USB_SPEED_UNKOWN phy_dp_pull_up 0,0\n",__FUNCTION__);			
		phy_dp_pull_up(0, 0);
#endif

	/* after reset -> Default State */
	usb_gadget_set_state(&dwc->gadget, USB_STATE_DEFAULT);

	if (dwc->gadget.speed != USB_SPEED_UNKNOWN)
		dwc3_disconnect_gadget(dwc);

	reg = dwc3_readl(dwc->regs, DWC3_DCTL);
	reg &= ~DWC3_DCTL_TSTCTRL_MASK;
	dwc3_writel(dwc->regs, DWC3_DCTL, reg);
	dwc->test_mode = false;

	dwc3_stop_active_transfers(dwc);
	dwc3_clear_stall_all_ep(dwc);
	dwc->start_config_issued = false;

	/* Reset device address to zero */
	reg = dwc3_readl(dwc->regs, DWC3_DCFG);
	reg &= ~(DWC3_DCFG_DEVADDR_MASK);
	dwc3_writel(dwc->regs, DWC3_DCFG, reg);
}

static void dwc3_update_ram_clk_sel(struct dwc3 *dwc, u32 speed)
{
	u32 reg;
	u32 usb30_clock = DWC3_GCTL_CLK_BUS;
//printk(KERN_ERR "### %s entry\n",__FUNCTION__);
	/*
	 * We change the clock only at SS but I dunno why I would want to do
	 * this. Maybe it becomes part of the power saving plan.
	 */

	if (speed != DWC3_DSTS_SUPERSPEED)
		return;

	/*
	 * RAMClkSel is reset to 0 after USB reset, so it must be reprogrammed
	 * each time on Connect Done.
	 */
	if (!usb30_clock)
		return;

	reg = dwc3_readl(dwc->regs, DWC3_GCTL);
	reg |= DWC3_GCTL_RAMCLKSEL(usb30_clock);
	dwc3_writel(dwc->regs, DWC3_GCTL, reg);
//printk(KERN_ERR "### %s reg set exit\n",__FUNCTION__);
}

static void dwc3_gadget_conndone_interrupt(struct dwc3 *dwc)
{
	struct dwc3_ep		*dep;
	int			ret;
	u32			reg;
	u8			speed;

	dev_vdbg(dwc->dev, "%s\n", __func__);

	reg = dwc3_readl(dwc->regs, DWC3_DSTS);
	speed = reg & DWC3_DSTS_CONNECTSPD;
	dwc->speed = speed;

	dwc3_update_ram_clk_sel(dwc, speed);

//printk(KERN_ERR "%s entry\n",__FUNCTION__);
	switch (speed) {
	case DWC3_DCFG_SUPERSPEED:
		/*
		 * WORKAROUND: DWC3 revisions <1.90a have an issue which
		 * would cause a missing USB3 Reset event.
		 *
		 * In such situations, we should force a USB3 Reset
		 * event by calling our dwc3_gadget_reset_interrupt()
		 * routine.
		 *
		 * Refers to:
		 *
		 * STAR#9000483510: RTL: SS : USB3 reset event may
		 * not be generated always when the link enters poll
		 */
		if (dwc->revision < DWC3_REVISION_190A){
//printk(KERN_ERR "connection done reset\n");
			dwc3_gadget_reset_interrupt(dwc);	/* yamano */
		}
		dwc3_gadget_ep0_desc.wMaxPacketSize = cpu_to_le16(512);
		dwc->gadget.ep0->maxpacket = 512;
		dwc->gadget.speed = USB_SPEED_SUPER;
		break;
	case DWC3_DCFG_HIGHSPEED:
		dwc3_gadget_ep0_desc.wMaxPacketSize = cpu_to_le16(64);
		dwc->gadget.ep0->maxpacket = 64;
		dwc->gadget.speed = USB_SPEED_HIGH;
		break;
	case DWC3_DCFG_FULLSPEED2:
	case DWC3_DCFG_FULLSPEED1:
		dwc3_gadget_ep0_desc.wMaxPacketSize = cpu_to_le16(64);
		dwc->gadget.ep0->maxpacket = 64;
		dwc->gadget.speed = USB_SPEED_FULL;
		break;
	case DWC3_DCFG_LOWSPEED:
		dwc3_gadget_ep0_desc.wMaxPacketSize = cpu_to_le16(8);
		dwc->gadget.ep0->maxpacket = 8;
		dwc->gadget.speed = USB_SPEED_LOW;
		break;
	default:
		dev_vdbg(dwc->dev, "??? SPEED UNKNOW ??? <HN>\n");
		break;
	}

	/* Enable USB2 LPM Capability */

	if ((dwc->revision > DWC3_REVISION_194A)
			&& (speed != DWC3_DCFG_SUPERSPEED)) {
		reg = dwc3_readl(dwc->regs, DWC3_DCFG);
		reg |= DWC3_DCFG_LPM_CAP;
		dwc3_writel(dwc->regs, DWC3_DCFG, reg);

		reg = dwc3_readl(dwc->regs, DWC3_DCTL);
		reg &= ~(DWC3_DCTL_HIRD_THRES_MASK | DWC3_DCTL_L1_HIBER_EN);

		/*
		 * TODO: This should be configurable. For now using
		 * maximum allowed HIRD threshold value of 0b1100
		 */
		reg |= DWC3_DCTL_HIRD_THRES(12);

		dwc3_writel(dwc->regs, DWC3_DCTL, reg);
	} else {
		reg = dwc3_readl(dwc->regs, DWC3_DCTL);
		reg &= ~DWC3_DCTL_HIRD_THRES_MASK;
		dwc3_writel(dwc->regs, DWC3_DCTL, reg);
	}

	dep = dwc->eps[0];
	ret = __dwc3_gadget_ep_enable(dep, &dwc3_gadget_ep0_desc, NULL, true,
			false);
	if (ret) {
		dev_err(dwc->dev, "failed to enable %s\n", dep->name);
		return;
	}

	dep = dwc->eps[1];
	ret = __dwc3_gadget_ep_enable(dep, &dwc3_gadget_ep0_desc, NULL, true,
			false);
	if (ret) {
		dev_err(dwc->dev, "failed to enable %s\n", dep->name);
		return;
	}

	/*
	 * Configure PHY via GUSB3PIPECTLn if required.
	 *
	 * Update GTXFIFOSIZn
	 *
	 * In both cases reset values should be sufficient.
	 */
#ifdef	HNDEBUG		/* <HN> */
	dev_vdbg(dwc->dev, "Connn:=PSize=%d Speed=%d\n",
		dwc3_gadget_ep0_desc.wMaxPacketSize,dwc->gadget.speed = USB_SPEED_SUPER);
#endif
}

static void dwc3_gadget_wakeup_interrupt(struct dwc3 *dwc)
{
	dev_vdbg(dwc->dev, "%s\n", __func__);

	/*
	 * TODO take core out of low power mode when that's
	 * implemented.
	 */

	dwc->gadget_driver->resume(&dwc->gadget);
}

static void dwc3_gadget_linksts_change_interrupt(struct dwc3 *dwc,
		unsigned int evtinfo)
{
	enum dwc3_link_state	next = evtinfo & DWC3_LINK_STATE_MASK;
	unsigned int		pwropt;
#if	1	/* <HN> */
	unsigned int		act_flag=0;
#endif	/* <HN> */

//printk(KERN_ERR "%s entry next = %x\n",__FUNCTION__,next);
	/*
	 * WORKAROUND: DWC3 < 2.50a have an issue when configured without
	 * Hibernation mode enabled which would show up when device detects
	 * host-initiated U3 exit.
	 *
	 * In that case, device will generate a Link State Change Interrupt
	 * from U3 to RESUME which is only necessary if Hibernation is
	 * configured in.
	 *
	 * There are no functional changes due to such spurious event and we
	 * just need to ignore it.
	 *
	 * Refers to:
	 *
	 * STAR#9000570034 RTL: SS Resume event generated in non-Hibernation
	 * operational mode
	 */
	pwropt = DWC3_GHWPARAMS1_EN_PWROPT(dwc->hwparams.hwparams1);
	if ((dwc->revision < DWC3_REVISION_250A) &&
			(pwropt != DWC3_GHWPARAMS1_EN_PWROPT_HIB)) {
		if ((dwc->link_state == DWC3_LINK_STATE_U3) &&
				(next == DWC3_LINK_STATE_RESUME)) {
			dev_vdbg(dwc->dev, "ignoring transition U3 -> Resume\n");
//			printk(KERN_ERR "ignoring transition U3 -> Resume\n"); /* yamano */
			return;
		}
	}

	/*
	 * WORKAROUND: DWC3 Revisions <1.83a have an issue which, depending
	 * on the link partner, the USB session might do multiple entry/exit
	 * of low power states before a transfer takes place.
	 *
	 * Due to this problem, we might experience lower throughput. The
	 * suggested workaround is to disable DCTL[12:9] bits if we're
	 * transitioning from U1/U2 to U0 and enable those bits again
	 * after a transfer completes and there are no pending transfers
	 * on any of the enabled endpoints.
	 *
	 * This is the first half of that workaround.
	 *
	 * Refers to:
	 *
	 * STAR#9000446952: RTL: Device SS : if U1/U2 ->U0 takes >128us
	 * core send LGO_Ux entering U0
	 */
	if (dwc->revision < DWC3_REVISION_183A) {
		if (next == DWC3_LINK_STATE_U0) {
			u32	u1u2;
			u32	reg;

			switch (dwc->link_state) {
			case DWC3_LINK_STATE_U1:
			case DWC3_LINK_STATE_U2:
				reg = dwc3_readl(dwc->regs, DWC3_DCTL);
				u1u2 = reg & (DWC3_DCTL_INITU2ENA
						| DWC3_DCTL_ACCEPTU2ENA
						| DWC3_DCTL_INITU1ENA
						| DWC3_DCTL_ACCEPTU1ENA);

				if (!dwc->u1u2)
					dwc->u1u2 = reg & u1u2;

				reg &= ~u1u2;

				dwc3_writel(dwc->regs, DWC3_DCTL, reg);
				break;
			default:
				/* do nothing */
				break;
			}
		}
		dev_vdbg(dwc->dev, "ASSUMED OLD CHIP W.A. <HN>\n");
	}

#if WORKAROUDN_55XXA0_DP_ISSUE
	if ((dwc->gadget.speed != USB_SPEED_SUPER) &&
		(dwc->gadget.speed != USB_SPEED_UNKNOWN)) {
		if ((next == DWC3_LINK_STATE_U2) ||
			(next == DWC3_LINK_STATE_U3) ||
			(next == DWC3_LINK_STATE_RX_DET)) {
//printk(KERN_ERR "%s 55xx unkonwn phy_dp_pull_up 1,1\n",__FUNCTION__);			
			/* 55XX A0 D+ workaround */
			phy_dp_pull_up(1, 1);
			act_flag |= 0x0001; /*<HN>*/
		}
	}
	if (next == DWC3_LINK_STATE_SS_DIS) {
		/* 55XX A0 D+ workaround */
//printk(KERN_ERR "%s 55xx DWC3_LINK_STATE_SS_DIS phy_dp_pull_up 1,1\n",__FUNCTION__);			
		phy_dp_pull_up(1, 1);
		act_flag |= 0x0002; /*<HN>*/
	}
	if ((dwc->gadget.speed == USB_SPEED_FULL) &&
		(next == DWC3_LINK_STATE_U0)) {
//printk(KERN_ERR "%s 55xx DWC3_LINK_STATE_U0 phy_dp_pull_up 1,1\n",__FUNCTION__);			
		/* 55XX A0 D+ workaround */
		phy_dp_pull_up(1, 1);
		act_flag |= 0x0004; /*<HN>*/
	}
	if((dwc->gadget.speed == USB_SPEED_SUPER) && (next == DWC3_LINK_STATE_U0)){
		phy_dp_pull_up(1, 1);	/* yamano's back 0305 */
		act_flag |= 0x0008; /*<HN>*/
	}
#endif


	switch (next) {
	case DWC3_LINK_STATE_U1:
		if (dwc->speed == USB_SPEED_SUPER) {
			dwc3_suspend_gadget(dwc);
			act_flag |= 0x0010; /*<HN>*/
		}
		break;
	case DWC3_LINK_STATE_U2:
	case DWC3_LINK_STATE_U3:
		dwc3_suspend_gadget(dwc);
		act_flag |= 0x0020; /*<HN>*/
		if (dwc->gadget.speed != USB_SPEED_SUPER) {	/* 2015-03-05 <HN> */
			phy_dp_pull_up(1, 1);
			act_flag |= 0x0400; /*<HN>*/
		}
		break;
	case DWC3_LINK_STATE_RESUME:
		dwc3_resume_gadget(dwc);
		act_flag |= 0x0040; /*<HN>*/
		break;
	case DWC3_LINK_STATE_U0:
		if ( dwc->speed == USB_SPEED_UNKNOWN && dwc->gadget.speed == USB_SPEED_SUPER) {
			if (dwc->link_state == DWC3_LINK_STATE_POLL) {
				dwc3_suspend_gadget(dwc); /*If this issue, state-change will be stopped */
				act_flag |= 0x0080; /*<HN>*/
			}
		}
#if	1	/*<HN>*/
		break;
	case DWC3_LINK_STATE_POLL :
			dwc3_suspend_gadget(dwc); /*If this issue, state-change will be stopped */
/*			dwc3_resume_gadget(dwc);*/
			act_flag |= 0x0100; /*<HN>*/
#endif
		break;
	case DWC3_LINK_STATE_RX_DET:	/* 2015-03-04 <HN> */
		if(dwc->gadget.speed != USB_SPEED_SUPER){
			dwc3_suspend_gadget(dwc);
			phy_dp_pull_up(1, 1);
			dwc3_resume_gadget(dwc);
			act_flag |= 0x0200; 
		} else {
			dwc3_suspend_gadget(dwc);
			phy_dp_pull_up(1, 1);
			dwc3_resume_gadget(dwc);
			act_flag |= 0x1000; 
		}
		break;
	default:
		/* do nothing */
		dev_vdbg(dwc->dev, "Do Nothing <HN>\n");	/*	<HN>	*/
		break;
	}

//	printk(KERN_ERR "link change: %s [%d] -> %s [%d]\n",
#ifdef	HNDEBUG		/* <HN> */
	dev_vdbg(dwc->dev, "link change: %s [%d] -> %s [%d] Act=%xH\n",
			dwc3_gadget_link_string(dwc->link_state),
			dwc->link_state, dwc3_gadget_link_string(next), next,act_flag);
	
#else
	dev_vdbg(dwc->dev, "link change: %s [%d] -> %s [%d]\n",
			dwc3_gadget_link_string(dwc->link_state),
			dwc->link_state, dwc3_gadget_link_string(next), next);
#endif	/*<HN>*/

	dwc->link_state = next;
}

static void dwc3_gadget_hibernation_interrupt(struct dwc3 *dwc,
		unsigned int evtinfo)
{
	unsigned int is_ss = evtinfo & BIT(4);
	enum dwc3_link_state	next = evtinfo & DWC3_LINK_STATE_MASK;

//printk(KERN_ERR "%s next = %d is_ss = %d\n",__FUNCTION__,next,is_ss);
	/**
	 * WORKAROUND: DWC3 revison 2.20a with hibernation support
	 * have a known issue which can cause USB CV TD.9.23 to fail
	 * randomly.
	 *
	 * Because of this issue, core could generate bogus hibernation
	 * events which SW needs to ignore.
	 *
	 * Refers to:
	 *
	 * STAR#9000546576: Device Mode Hibernation: Issue in USB 2.0
	 * Device Fallback from SuperSpeed
	 */
	if (is_ss ^ (dwc->speed == USB_SPEED_SUPER))
		return;

	/* enter hibernation here */
}

static void dwc3_gadget_interrupt(struct dwc3 *dwc,
		const struct dwc3_event_devt *event)
{
	switch (event->type) {
	case DWC3_DEVICE_EVENT_DISCONNECT:
//printk(KERN_ERR "### %s # DISCONNECT type = %x\n",__FUNCTION__,event->type);
		dwc3_gadget_disconnect_interrupt(dwc);
		break;
	case DWC3_DEVICE_EVENT_RESET:
//printk(KERN_ERR "### %s # RESET type = %x\n",__FUNCTION__,event->type);
		dwc3_gadget_reset_interrupt(dwc);
		break;
	case DWC3_DEVICE_EVENT_CONNECT_DONE:
//printk(KERN_ERR "### %s # CONNECT_DONE type = %x\n",__FUNCTION__,event->type);
		dwc3_gadget_conndone_interrupt(dwc);
		break;
	case DWC3_DEVICE_EVENT_WAKEUP:
//printk(KERN_ERR "### %s # WAKEUP type = %x\n",__FUNCTION__,event->type);
		dwc3_gadget_wakeup_interrupt(dwc);
		break;
	case DWC3_DEVICE_EVENT_HIBER_REQ:
//printk(KERN_ERR "### %s # HIBER_REQ type = %x\n",__FUNCTION__,event->type);
		if (dev_WARN_ONCE(dwc->dev, !dwc->has_hibernation,
					"unexpected hibernation event\n"))
			break;

		dwc3_gadget_hibernation_interrupt(dwc, event->event_info);
		break;
	case DWC3_DEVICE_EVENT_LINK_STATUS_CHANGE:
//printk(KERN_ERR "### %s # STATUS type = %x\n",__FUNCTION__,event->type);
		dwc3_gadget_linksts_change_interrupt(dwc, event->event_info);
		break;
	case DWC3_DEVICE_EVENT_EOPF:
//printk(KERN_ERR "### %s # EOPF type = %x\n",__FUNCTION__,event->type);
		dev_vdbg(dwc->dev, "End of Periodic Frame\n");
		break;
	case DWC3_DEVICE_EVENT_SOF:
//printk(KERN_ERR "### %s # SOF type = %x\n",__FUNCTION__,event->type);
		dev_vdbg(dwc->dev, "Start of Periodic Frame\n");
		break;
	case DWC3_DEVICE_EVENT_ERRATIC_ERROR:
//printk(KERN_ERR "### %s # ERRATIC type = %x\n",__FUNCTION__,event->type);
		dev_vdbg(dwc->dev, "Erratic Error\n");
		break;
	case DWC3_DEVICE_EVENT_CMD_CMPL:
//printk(KERN_ERR "### %s # CMD COMPL type = %x\n",__FUNCTION__,event->type);
		dev_vdbg(dwc->dev, "Command Complete\n");
		break;
	case DWC3_DEVICE_EVENT_OVERFLOW:
//printk(KERN_ERR "### %s # OVERFLOW type = %x\n",__FUNCTION__,event->type);
		dev_vdbg(dwc->dev, "Overflow\n");
		break;
	default:
//printk(KERN_ERR "### %s # default type = %x\n",__FUNCTION__,event->type);
		dev_dbg(dwc->dev, "UNKNOWN IRQ %d\n", event->type);
	}
}

static void dwc3_process_event_entry(struct dwc3 *dwc,
		const union dwc3_event *event)
{
	/* Endpoint IRQ, handle it and return early */
	if (event->type.is_devspec == 0) {
		/* depevt */
		return dwc3_endpoint_interrupt(dwc, &event->depevt);
	}

	switch (event->type.type) {
	case DWC3_EVENT_TYPE_DEV:
		dwc3_gadget_interrupt(dwc, &event->devt);
		break;
	/* REVISIT what to do with Carkit and I2C events ? */
	default:
		dev_err(dwc->dev, "UNKNOWN IRQ type %d\n", event->raw);
#if 0
		printk(KERN_ERR "UNKNOWN IRQ type %d\n", event->raw);
#endif
	}
}

static irqreturn_t dwc3_process_event_buf(struct dwc3 *dwc, u32 buf)
{
	struct dwc3_event_buffer *evt;
	irqreturn_t ret = IRQ_NONE;
		int			left;
	u32 reg;

	evt = dwc->ev_buffs[buf];
	left = evt->count;

	if (!(evt->flags & DWC3_EVENT_PENDING))
		return IRQ_NONE;

	while (left > 0) {
		union dwc3_event event;

		event.raw = *(u32 *) (evt->buf + evt->lpos);

		dwc3_process_event_entry(dwc, &event);

		/*
		 * FIXME we wrap around correctly to the next entry as
		 * almost all entries are 4 bytes in size. There is one
		 * entry which has 12 bytes which is a regular entry
		 * followed by 8 bytes data. ATM I don't know how
		 * things are organized if we get next to the a
		 * boundary so I worry about that once we try to handle
		 * that.
		 */
		evt->lpos = (evt->lpos + 4) % DWC3_EVENT_BUFFERS_SIZE;
		left -= 4;

		dwc3_writel(dwc->regs, DWC3_GEVNTCOUNT(buf), 4);
	}

	evt->count = 0;
	evt->flags &= ~DWC3_EVENT_PENDING;
	ret = IRQ_HANDLED;

	/* Unmask interrupt */
	reg = dwc3_readl(dwc->regs, DWC3_GEVNTSIZ(buf));
	reg &= ~DWC3_GEVNTSIZ_INTMASK;
	dwc3_writel(dwc->regs, DWC3_GEVNTSIZ(buf), reg);

	return ret;
}

static irqreturn_t dwc3_thread_interrupt(int irq, void *_dwc)
{
	struct dwc3 *dwc = _dwc;
	unsigned long flags;
	irqreturn_t ret = IRQ_NONE;
	int i;

//	spin_lock_irqsave(&dwc->lock, flags);	/* yamano debug */
	spin_lock(&dwc->lock);

	for (i = 0; i < dwc->num_event_buffers; i++)
		ret |= dwc3_process_event_buf(dwc, i);

//	spin_unlock_irqrestore(&dwc->lock, flags);
	spin_unlock(&dwc->lock);

	return ret;
}

static irqreturn_t dwc3_check_event_buf(struct dwc3 *dwc, u32 buf)
{
	struct dwc3_event_buffer *evt;
	u32 count;
	u32 reg;

	evt = dwc->ev_buffs[buf];

	count = dwc3_readl(dwc->regs, DWC3_GEVNTCOUNT(buf));
	count &= DWC3_GEVNTCOUNT_MASK;
	if (!count)
		return IRQ_NONE;

	evt->count = count;
	evt->flags |= DWC3_EVENT_PENDING;

	/* Mask interrupt */
	reg = dwc3_readl(dwc->regs, DWC3_GEVNTSIZ(buf));
	reg |= DWC3_GEVNTSIZ_INTMASK;
	dwc3_writel(dwc->regs, DWC3_GEVNTSIZ(buf), reg);

	return IRQ_WAKE_THREAD;
}

static irqreturn_t dwc3_interrupt(int irq, void *_dwc)
{
	struct dwc3	*dwc = _dwc;
	int		i;
	irqreturn_t	ret = IRQ_NONE;
	unsigned long	flags;

#if 1
//	spin_lock_irqsave(&dwc->lock,flags);
#else
	spin_lock(&dwc->lock);
#endif
	for (i = 0; i < dwc->num_event_buffers; i++) {
		irqreturn_t status;

		status = dwc3_check_event_buf(dwc, i);
		if (status == IRQ_WAKE_THREAD)
			ret = status;
	}
#if 1
//	spin_unlock_irqrestore(&dwc->lock,flags);
#else
	spin_unlock(&dwc->lock);
#endif
	return ret;
}

/**
 *  * phy_reset - Reset USB device PHY
 *   */
static void phy_reset(void)
{
        volatile u8     __iomem *reg;

        reg = ioremap(USB3_PHY_PHY_RESET, 4);
        writel(readl(reg) | USB3_PHY_PHY_RESET__PHY_RESET__MASK, reg);
        udelay(5);
        writel(0, reg);
        iounmap(reg);
}

/**
 *  * phy_override - Override some USB device PHY registers
 *   */
static void phy_override(void)
{
        volatile u8     __iomem *reg;

        /* Initialize PHY settings */
        reg = ioremap(USB3_PHY_TXHSXVTUNE, 4);
        writel(0x01, reg);
        iounmap(reg);

        reg = ioremap(USB3_PHY_TXPREEMPAMPTUNE, 4);
        writel(0x03, reg);
        iounmap(reg);

        reg = ioremap(USB3_PHY_TXRISETUNE, 4);
        writel(0x02, reg);
        iounmap(reg);

        reg = ioremap(USB3_PHY_TXVREFTUNE, 4);
        writel(0x09, reg);
        iounmap(reg);

        reg = ioremap(USB3_PHY_pcs_tx_swing_full, 4);
        writel(0x79 /*0x7D*/, reg);
        iounmap(reg);

        /* Reset PHY to take all updated PHY settings */
        phy_reset();

        /* Use fixed equalization */
        phy_fixed_equalization(2);   // Must be done after phy_reset()
}

/**
 * dwc3_gadget_init - Initializes gadget related registers
 * @dwc: pointer to our controller context structure
 *
 * Returns 0 on success otherwise negative errno.
 */
int dwc3_gadget_init(struct dwc3 *dwc)
{
	int					ret;
	volatile u8	__iomem	*reg;

#if WORKAROUDN_55XXA0_DP_ISSUE
	/* 55XX A0 D+ workaround */
	cr_cap_addr = ioremap(USB3_PHY_CR_CAP_ADDR, 4);
	cr_cap_data = ioremap(USB3_PHY_CR_CAP_DATA, 4);
	cr_data_in = ioremap(USB3_PHY_CR_DATA_IN, 4);
        cr_read     = ioremap(USB3_PHY_CR_READ, 4);
	cr_write = ioremap(USB3_PHY_CR_WRITE, 4);
	cr_ack = ioremap(USB3_PHY_CR_ACK, 4);
        cr_data_out = ioremap(USB3_PHY_CR_DATA_OUT, 4);

        phy_override();
#endif
#if 0
	/* Initialize PHY settings */
	reg = ioremap(USB3_PHY_pcs_tx_swing_full, 4);
	writel(0x7D, reg);
	iounmap(reg);

	/* Reset PHY to take all updated PHY settings */
	reg = ioremap(USB3_PHY_PHY_RESET, 4);
	writel(readl(reg) | USB3_PHY_PHY_RESET__PHY_RESET__MASK, reg);
	udelay(5);
	writel(0, reg);
	iounmap(reg);
#endif	/* yamano */
	dwc->ctrl_req = dma_alloc_coherent(dwc->dev, sizeof(*dwc->ctrl_req),
			&dwc->ctrl_req_addr, GFP_KERNEL);
	if (!dwc->ctrl_req) {
		dev_err(dwc->dev, "failed to allocate ctrl request\n");
		ret = -ENOMEM;
		goto err0;
	}

	dwc->ep0_trb = dma_alloc_coherent(dwc->dev, sizeof(*dwc->ep0_trb),
			&dwc->ep0_trb_addr, GFP_KERNEL);
	if (!dwc->ep0_trb) {
		dev_err(dwc->dev, "failed to allocate ep0 trb\n");
		ret = -ENOMEM;
		goto err1;
	}

	dwc->setup_buf = kzalloc(DWC3_EP0_BOUNCE_SIZE, GFP_KERNEL);
	if (!dwc->setup_buf) {
		dev_err(dwc->dev, "failed to allocate setup buffer\n");
		ret = -ENOMEM;
		goto err2;
	}

	dwc->ep0_bounce = dma_alloc_coherent(dwc->dev,
			DWC3_EP0_BOUNCE_SIZE, &dwc->ep0_bounce_addr,
			GFP_KERNEL);
	if (!dwc->ep0_bounce) {
		dev_err(dwc->dev, "failed to allocate ep0 bounce buffer\n");
		ret = -ENOMEM;
		goto err3;
	}

	dwc->has_gadget			= true;
	dwc->gadget.ops			= &dwc3_gadget_ops;
	dwc->gadget.max_speed		= USB_SPEED_SUPER;
	dwc->gadget.speed		= USB_SPEED_UNKNOWN;
	dwc->gadget.sg_supported	= true;
	dwc->gadget.name		= "dwc3-gadget";

	/*
	 * Per databook, DWC3 needs buffer size to be aligned to MaxPacketSize
	 * on ep out.
	 */
	dwc->gadget.quirk_ep_out_aligned_size = true;

	/*
	 * REVISIT: Here we should clear all pending IRQs to be
	 * sure we're starting from a well known location.
	 */

	ret = dwc3_gadget_init_endpoints(dwc);
	if (ret)
		goto err4;

	ret = usb_add_gadget_udc(dwc->dev, &dwc->gadget);
	if (ret) {
		dev_err(dwc->dev, "failed to register udc\n");
		goto err4;
	}

	return 0;

err4:
	dwc3_gadget_free_endpoints(dwc);
	dma_free_coherent(dwc->dev, DWC3_EP0_BOUNCE_SIZE,
			dwc->ep0_bounce, dwc->ep0_bounce_addr);

err3:
	kfree(dwc->setup_buf);

err2:
	dma_free_coherent(dwc->dev, sizeof(*dwc->ep0_trb),
			dwc->ep0_trb, dwc->ep0_trb_addr);

err1:
	dma_free_coherent(dwc->dev, sizeof(*dwc->ctrl_req),
			dwc->ctrl_req, dwc->ctrl_req_addr);

err0:
	return ret;
}

/* -------------------------------------------------------------------------- */

void dwc3_gadget_exit(struct dwc3 *dwc)
{
	dwc->has_gadget = false;
	usb_del_gadget_udc(&dwc->gadget);

	dwc3_gadget_free_endpoints(dwc);

	dma_free_coherent(dwc->dev, DWC3_EP0_BOUNCE_SIZE,
			dwc->ep0_bounce, dwc->ep0_bounce_addr);

	kfree(dwc->setup_buf);

	dma_free_coherent(dwc->dev, sizeof(*dwc->ep0_trb),
			dwc->ep0_trb, dwc->ep0_trb_addr);

	dma_free_coherent(dwc->dev, sizeof(*dwc->ctrl_req),
			dwc->ctrl_req, dwc->ctrl_req_addr);

#if WORKAROUDN_55XXA0_DP_ISSUE
	/* 55XX A0 D+ workaround */
	iounmap(cr_cap_addr);
	iounmap(cr_cap_data);
	iounmap(cr_data_in);
	iounmap(cr_write);
	iounmap(cr_ack);
    iounmap(cr_data_out);	/* added for fail-safe  0307 <HN>*/
#endif
}

int dwc3_gadget_prepare(struct dwc3 *dwc)
{
	if (dwc->pullups_connected) {
		dwc3_gadget_disable_irq(dwc);
		dwc3_gadget_run_stop(dwc, true, true);
	}

	return 0;
}

void dwc3_gadget_complete(struct dwc3 *dwc)
{
	if (dwc->pullups_connected) {
		dwc3_gadget_enable_irq(dwc);
		dwc3_gadget_run_stop(dwc, true, false);
	}
}

int dwc3_gadget_suspend(struct dwc3 *dwc)
{
	__dwc3_gadget_ep_disable(dwc->eps[0]);
	__dwc3_gadget_ep_disable(dwc->eps[1]);

	dwc->dcfg = dwc3_readl(dwc->regs, DWC3_DCFG);

	return 0;
}

int dwc3_gadget_resume(struct dwc3 *dwc)
{
	struct dwc3_ep		*dep;
	int			ret;

	/* Start with SuperSpeed Default */
	dwc3_gadget_ep0_desc.wMaxPacketSize = cpu_to_le16(512);

	dep = dwc->eps[0];
	ret = __dwc3_gadget_ep_enable(dep, &dwc3_gadget_ep0_desc, NULL, false,
			false);
	if (ret)
		goto err0;

	dep = dwc->eps[1];
	ret = __dwc3_gadget_ep_enable(dep, &dwc3_gadget_ep0_desc, NULL, false,
			false);
	if (ret)
		goto err1;

	/* begin to receive SETUP packets */
	dwc->ep0state = EP0_SETUP_PHASE;
	dwc3_ep0_out_start(dwc);

	dwc3_writel(dwc->regs, DWC3_DCFG, dwc->dcfg);

	return 0;

err1:
	__dwc3_gadget_ep_disable(dwc->eps[0]);

err0:
	return ret;
}
