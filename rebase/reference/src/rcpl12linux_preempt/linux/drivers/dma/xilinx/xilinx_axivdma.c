/*
 * Xilinx Video DMA Engine support
 *
 * Copyright (C) 2010 Xilinx, Inc. All rights reserved.
 *
 * Based on the Freescale DMA driver.
 *
 * Description:
 *  . Axi VDMA engine, it does transfers between memory and video devices.
 *    It can be configured to have one channel or two channels. If configured
 *    as two channels, one is to transmit to the video device and another is
 *    to receive from the video device.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/dmapool.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/amba/xilinx_dma.h>


/* Hw specific definitions */
#define XILINX_VDMA_MAX_CHANS_PER_DEVICE	0x2
#define XILINX_VDMA_MAX_TRANS_LEN		0x7FFFFF

/* General register bits definitions */
#define XILINX_VDMA_CR_RESET_MASK	0x00000004
						/* Reset DMA engine */
#define XILINX_VDMA_CR_RUNSTOP_MASK	0x00000001
						/* Start/stop DMA engine */
#define XILINX_VDMA_CR_FSYNC_SRC_MASK	0x00000060
						/* FSYNC Source Mask */

#define XILINX_VDMA_SR_HALTED_MASK	0x00000001
						/* DMA channel halted */
#define XILINX_VDMA_SR_IDLE_MASK	0x00000002
						/* DMA channel idle */

#define XILINX_VDMA_SR_ERR_INTERNAL_MASK	0x00000010
						/* Datamover internal err */
#define XILINX_VDMA_SR_ERR_SLAVE_MASK		0x00000020
						/* Datamover slave err */
#define XILINX_VDMA_SR_ERR_DECODE_MASK		0x00000040
						/* Datamover decode err */
#define XILINX_VDMA_SR_ERR_SG_INT_MASK		0x00000100
						/* SG internal err */
#define XILINX_VDMA_SR_ERR_SG_SLV_MASK		0x00000200
						/* SG slave err */
#define XILINX_VDMA_SR_ERR_SG_DEC_MASK		0x00000400
						/* SG decode err */
#define XILINX_VDMA_SR_ERR_ALL_MASK		0x00000770
						/* All errors */

#define XILINX_VDMA_XR_IRQ_IOC_MASK	0x00001000
						/* Completion interrupt */
#define XILINX_VDMA_XR_IRQ_DELAY_MASK	0x00002000
						/* Delay interrupt */
#define XILINX_VDMA_XR_IRQ_ERROR_MASK	0x00004000
						/* Error interrupt */
#define XILINX_VDMA_XR_IRQ_ALL_MASK	0x00007000
						/* All interrupts */

#define XILINX_VDMA_XR_DELAY_MASK	0xFF000000
						/* Delay timeout counter */
#define XILINX_VDMA_XR_COALESCE_MASK	0x00FF0000
						/* Coalesce counter */

#define XILINX_VDMA_IRQ_SHIFT		12
#define XILINX_VDMA_DELAY_SHIFT		24
#define XILINX_VDMA_COALESCE_SHIFT	16

#define XILINX_VDMA_DELAY_MAX		0xFF
					/* Maximum delay counter value */
#define XILINX_VDMA_COALESCE_MAX	0xFF
					/* Maximum coalescing counter value */

#define XILINX_VDMA_RX_CHANNEL_OFFSET	0x30

#define XILINX_VDMA_CIRC_EN	0x00000002	/* Circular mode */
#define XILINX_VDMA_SYNC_EN	0x00000008	/* Sync enable mode */
#define XILINX_VDMA_FRMCNT_EN	0x00000010	/* Frm Cnt enable mode */
#define XILINX_VDMA_MSTR_MASK	0x00000F00	/* Master in control */

#define XILINX_VDMA_EXTFSYNC_SHIFT	5
#define XILINX_VDMA_MSTR_SHIFT		8
#define XILINX_VDMA_WR_REF_SHIFT	8

#define XILINX_VDMA_FRMDLY_SHIFT	24

#define XILINX_VDMA_DIRECT_REG_OFFSET		0x50
#define XILINX_VDMA_CHAN_DIRECT_REG_SIZE	0x50

#define XILINX_VDMA_PARK_REG_OFFSET		0x28

#define XILINX_VDMA_SR_ERR_FSIZE_LESS_MASK	0x00000080
						/* FSize Less Mismatch err */
#define XILINX_VDMA_SR_ERR_LSIZE_LESS_MASK	0x00000100
						/* LSize Less Mismatch err */
#define XILINX_VDMA_SR_ERR_FSIZE_MORE_MASK	0x00000800
						/* FSize more err */

/*
 * Recoverable errors are DMA Internal error, FSize Less, LSize Less
 * and FSize More mismatch errors.  These are only recoverable only
 * when C_FLUSH_ON_FSYNC is enabled in the hardware system.
 */
#define XILINX_VDMA_SR_ERR_RECOVER_MASK	0x00000990
						/* Recoverable errs */

/* Axi VDMA Flush on Fsync bits */
#define XILINX_VDMA_FLUSH_S2MM	3
#define XILINX_VDMA_FLUSH_MM2S	2
#define XILINX_VDMA_FLUSH_BOTH	1

/* Feature encodings */
#define XILINX_VDMA_FTR_HAS_SG		0x00000100
						/* Has SG */
#define XILINX_VDMA_FTR_HAS_SG_SHIFT	8
						/* Has SG shift */
#define XILINX_VDMA_FTR_FLUSH_MASK	0x00000600
						/* Flush-on-FSync Mask */
#define XILINX_VDMA_FTR_FLUSH_SHIFT	9
						/* Flush-on-FSync shift */

/* Delay loop counter to prevent hardware failure */
#define XILINX_VDMA_RESET_LOOP	1000000
#define XILINX_VDMA_HALT_LOOP	1000000

/* IO accessors */
#define VDMA_OUT(addr, val)	(iowrite32(val, addr))
#define VDMA_IN(addr)		(ioread32(addr))

/* Hardware descriptor */
struct xilinx_vdma_desc_hw {
	u32 next_desc;	/* 0x00 */
	u32 pad1;	/* 0x04 */
	u32 buf_addr;	/* 0x08 */
	u32 pad2;	/* 0x0C */
	u32 vsize;	/* 0x10 */
	u32 hsize;	/* 0x14 */
	u32 stride;	/* 0x18 */
} __aligned(64);

struct xilinx_vdma_desc_sw {
	struct xilinx_vdma_desc_hw hw;
	struct list_head node;
	struct list_head tx_list;
	struct dma_async_tx_descriptor async_tx;
} __aligned(64);

struct xvdma_regs {
	u32 cr;		/* 0x00 Control Register */
	u32 sr;		/* 0x04 Status Register */
	u32 cdr;	/* 0x08 Current Descriptor Register */
	u32 pad1;
	u32 tdr;	/* 0x10 Tail Descriptor Register */
	u32 pad2;
};

struct vdma_addr_regs {
	u32 vsize;		/* 0x0 Vertical size */
	u32 hsize;		/* 0x4 Horizontal size */
	u32 frmdly_stride;	/* 0x8 Frame delay and stride */
	u32 buf_addr[16];	/* 0xC - 0x48 Src addresses */
};

/* Per DMA specific operations should be embedded in the channel structure */
struct xilinx_vdma_chan {
	struct xvdma_regs __iomem *regs;	/* Control status registers */
	struct vdma_addr_regs *addr_regs;	/* Direct address registers */
	dma_cookie_t completed_cookie;		/* Maximum cookie completed */
	dma_cookie_t cookie;			/* The current cookie */
	spinlock_t lock;			/* Descriptor operation lock */
	bool sg_waiting;			/* SG transfer waiting */
	struct list_head active_list;		/* Active descriptors */
	struct list_head pending_list;		/* Descriptors waiting */
	struct dma_chan common;			/* DMA common channel */
	struct dma_pool *desc_pool;		/* Descriptors pool */
	struct device *dev;			/* The dma device */
	int irq;				/* Channel IRQ */
	int id;					/* Channel ID */
	enum dma_transfer_direction direction;	/* Transfer direction */
	int max_len;				/* Max data len per transfer */
	int is_lite;				/* Whether is light build */
	int num_frms;				/* Number of frames */
	int has_sg;				/* Support scatter transfers */
	int has_dre;				/* For unaligned transfers */
	int genlock;				/* Support genlock mode */
	int err;				/* Channel has errors */
	struct tasklet_struct tasklet;		/* Cleanup work after irq */
	u32 feature;				/* IP feature */
	u32 private;				/* Match info for
							channel request */
	void (*start_transfer)(struct xilinx_vdma_chan *chan);
	struct xilinx_vdma_config config;	/* Device configuration info */
	u32 flush_fsync;			/* Flush on Fsync */
};

struct xilinx_vdma_device {
	void __iomem *regs;
	struct device *dev;
	struct dma_device common;
	struct xilinx_vdma_chan *chan[XILINX_VDMA_MAX_CHANS_PER_DEVICE];
	u32 feature;
	int irq;
};

#define to_xilinx_chan(chan) \
			container_of(chan, struct xilinx_vdma_chan, common)

/* Required functions */

static int xilinx_vdma_alloc_chan_resources(struct dma_chan *dchan)
{
	struct xilinx_vdma_chan *chan = to_xilinx_chan(dchan);

	/* Has this channel already been allocated? */
	if (chan->desc_pool)
		return 1;

	/*
	 * We need the descriptor to be aligned to 64bytes
	 * for meeting Xilinx VDMA specification requirement.
	 */
	chan->desc_pool = dma_pool_create("xilinx_vdma_desc_pool",
				chan->dev,
				sizeof(struct xilinx_vdma_desc_sw),
				__alignof__(struct xilinx_vdma_desc_sw), 0);
	if (!chan->desc_pool) {
		dev_err(chan->dev,
			"unable to allocate channel %d descriptor pool\n",
			chan->id);
		return -ENOMEM;
	}

	chan->completed_cookie = 1;
	chan->cookie = 1;

	/* there is at least one descriptor free to be allocated */
	return 1;
}

static void xilinx_vdma_free_desc_list(struct xilinx_vdma_chan *chan,
					struct list_head *list)
{
	struct xilinx_vdma_desc_sw *desc, *_desc;

	list_for_each_entry_safe(desc, _desc, list, node) {
		list_del(&desc->node);
		dma_pool_free(chan->desc_pool, desc, desc->async_tx.phys);
	}
}

static void xilinx_vdma_free_desc_list_reverse(struct xilinx_vdma_chan *chan,
						struct list_head *list)
{
	struct xilinx_vdma_desc_sw *desc, *_desc;

	list_for_each_entry_safe_reverse(desc, _desc, list, node) {
		list_del(&desc->node);
		dma_pool_free(chan->desc_pool, desc, desc->async_tx.phys);
	}
}

static void xilinx_vdma_free_chan_resources(struct dma_chan *dchan)
{
	struct xilinx_vdma_chan *chan = to_xilinx_chan(dchan);
	unsigned long flags;

	dev_dbg(chan->dev, "Free all channel resources.\n");
	spin_lock_irqsave(&chan->lock, flags);
	xilinx_vdma_free_desc_list(chan, &chan->active_list);
	xilinx_vdma_free_desc_list(chan, &chan->pending_list);
	spin_unlock_irqrestore(&chan->lock, flags);

	dma_pool_destroy(chan->desc_pool);
	chan->desc_pool = NULL;
}

static enum dma_status xilinx_vdma_desc_status(struct xilinx_vdma_chan *chan,
					struct xilinx_vdma_desc_sw *desc)
{
	return dma_async_is_complete(desc->async_tx.cookie,
					chan->completed_cookie,
					chan->cookie);
}

static void xilinx_chan_desc_cleanup(struct xilinx_vdma_chan *chan)
{
	struct xilinx_vdma_desc_sw *desc, *_desc;
	unsigned long flags;

	spin_lock_irqsave(&chan->lock, flags);

	list_for_each_entry_safe(desc, _desc, &chan->active_list, node) {
		dma_async_tx_callback callback;
		void *callback_param;

		if (xilinx_vdma_desc_status(chan, desc) == DMA_IN_PROGRESS)
			break;

		/* Remove from the list of running transactions */
		list_del(&desc->node);

		/* Run the link descriptor callback function */
		callback = desc->async_tx.callback;
		callback_param = desc->async_tx.callback_param;
		if (callback) {
			spin_unlock_irqrestore(&chan->lock, flags);
			callback(callback_param);
			spin_lock_irqsave(&chan->lock, flags);
		}

		/* Run any dependencies, then free the descriptor */
		dma_run_dependencies(&desc->async_tx);
		dma_pool_free(chan->desc_pool, desc, desc->async_tx.phys);
	}

	spin_unlock_irqrestore(&chan->lock, flags);
}

static enum dma_status xilinx_tx_status(struct dma_chan *dchan,
					dma_cookie_t cookie,
					struct dma_tx_state *txstate)
{
	struct xilinx_vdma_chan *chan = to_xilinx_chan(dchan);
	dma_cookie_t last_used;
	dma_cookie_t last_complete;

	xilinx_chan_desc_cleanup(chan);

	last_used = dchan->cookie;
	last_complete = chan->completed_cookie;

	dma_set_tx_state(txstate, last_complete, last_used, 0);

	return dma_async_is_complete(cookie, last_complete, last_used);
}

static int dma_is_running(struct xilinx_vdma_chan *chan)
{
	return !(VDMA_IN(&chan->regs->sr) & XILINX_VDMA_SR_HALTED_MASK) &&
		(VDMA_IN(&chan->regs->cr) & XILINX_VDMA_CR_RUNSTOP_MASK);
}

static int dma_is_idle(struct xilinx_vdma_chan *chan)
{
	return VDMA_IN(&chan->regs->sr) & XILINX_VDMA_SR_IDLE_MASK;
}

#define XILINX_VDMA_DRIVER_DEBUG	0

#if (XILINX_VDMA_DRIVER_DEBUG == 1)
static void desc_dump(struct xilinx_vdma_desc_hw *hw)
{
	pr_info("hw desc %x:\n", (unsigned int)hw);
	pr_info("\tnext_desc %x\n", hw->next_desc);
	pr_info("\tbuf_addr %x\n", hw->buf_addr);
	pr_info("\tvsize %x\n", hw->vsize);
	pr_info("\thsize %x\n", hw->hsize);
	pr_info("\tstride %x\n", hw->stride);
	pr_info("\tstatus %x\n", hw->status);

}
#endif

/* Stop the hardware, the ongoing transfer will be finished */
static void vdma_halt(struct xilinx_vdma_chan *chan)
{
	int loop = XILINX_VDMA_HALT_LOOP;

	VDMA_OUT(&chan->regs->cr,
		VDMA_IN(&chan->regs->cr) & ~XILINX_VDMA_CR_RUNSTOP_MASK);

	/* Wait for the hardware to halt */
	while (loop) {
		if (!(VDMA_IN(&chan->regs->cr) & XILINX_VDMA_CR_RUNSTOP_MASK))
			break;

		loop -= 1;
	}

	if (!loop) {
		pr_debug("Cannot stop channel %x: %x\n",
			(unsigned int)chan,
			(unsigned int)VDMA_IN(&chan->regs->cr));
		chan->err = 1;
	}

	return;
}

/* Start the hardware. Transfers are not started yet */
static void vdma_start(struct xilinx_vdma_chan *chan)
{
	int loop = XILINX_VDMA_HALT_LOOP;

	VDMA_OUT(&chan->regs->cr,
		VDMA_IN(&chan->regs->cr) | XILINX_VDMA_CR_RUNSTOP_MASK);

	/* Wait for the hardware to start */
	while (loop) {
		if (VDMA_IN(&chan->regs->cr) & XILINX_VDMA_CR_RUNSTOP_MASK)
			break;

		loop -= 1;
	}

	if (!loop) {
		pr_debug("Cannot start channel %x: %x\n",
			(unsigned int)chan,
			(unsigned int)VDMA_IN(&chan->regs->cr));

		chan->err = 1;
	}

	return;
}

static void xilinx_vdma_start_transfer(struct xilinx_vdma_chan *chan)
{
	unsigned long flags;
	struct xilinx_vdma_desc_sw *desch, *desct = NULL;
	struct xilinx_vdma_config *config;
	u32 reg;
	u8 *chan_base;

	if (chan->err)
		return;

	spin_lock_irqsave(&chan->lock, flags);

	if (list_empty(&chan->pending_list))
		goto out_unlock;

	/* If it is SG mode and hardware is busy, cannot submit */
	if (chan->has_sg && dma_is_running(chan) && !dma_is_idle(chan)) {
		dev_dbg(chan->dev, "DMA controller still busy\n");
		goto out_unlock;
	}

	/*
	 * If hardware is idle, then all descriptors on the running lists are
	 * done, start new transfers
	 */
	if (chan->err)
		goto out_unlock;

	if (chan->has_sg) {
		desch = list_first_entry(&chan->pending_list,
				struct xilinx_vdma_desc_sw, node);

		desct = container_of(chan->pending_list.prev,
				struct xilinx_vdma_desc_sw, node);

		VDMA_OUT(&chan->regs->cdr, desch->async_tx.phys);
	}

	/* Configure the hardware using info in the config structure */
	config = &(chan->config);
	reg = VDMA_IN(&chan->regs->cr);

	if (config->frm_cnt_en)
		reg |= XILINX_VDMA_FRMCNT_EN;
	else
		reg &= ~XILINX_VDMA_FRMCNT_EN;

	/*
	 * With SG, start with circular mode, so that BDs can be fetched.
	 * In direct register mode, if not parking, enable circular mode
	 */
	if ((chan->has_sg) || (!config->park))
		reg |= XILINX_VDMA_CIRC_EN;

	if (config->park)
		reg &= ~XILINX_VDMA_CIRC_EN;

	VDMA_OUT(&chan->regs->cr, reg);

	if (config->park && (config->park_frm >= 0)
			&& (config->park_frm < chan->num_frms)) {
		if (config->direction == DMA_MEM_TO_DEV) {
			chan_base = (char *)chan->regs;
			VDMA_OUT((chan_base + XILINX_VDMA_PARK_REG_OFFSET),
					config->park_frm);
		} else {
			chan_base = ((char *)chan->regs -
					XILINX_VDMA_RX_CHANNEL_OFFSET);
			VDMA_OUT((chan_base + XILINX_VDMA_PARK_REG_OFFSET),
				config->park_frm << XILINX_VDMA_WR_REF_SHIFT);
		}
	}

	/* Start the hardware */
	vdma_start(chan);

	if (chan->err)
		goto out_unlock;
	list_splice_tail_init(&chan->pending_list, &chan->active_list);

	/*
	 * Enable interrupts
	 * park/genlock testing does not use interrupts
	 */
	if (!chan->config.disable_intr) {
		VDMA_OUT(&chan->regs->cr,
			VDMA_IN(&chan->regs->cr) |
				XILINX_VDMA_XR_IRQ_ALL_MASK);
	} else {
		VDMA_OUT(&chan->regs->cr,
			(VDMA_IN(&chan->regs->cr) |
				XILINX_VDMA_XR_IRQ_ALL_MASK) &
			~((chan->config.disable_intr <<
				XILINX_VDMA_IRQ_SHIFT)));
	}

	/* Start the transfer */
	if (chan->has_sg)
		VDMA_OUT(&chan->regs->tdr, desct->async_tx.phys);
	else
		VDMA_OUT(&chan->addr_regs->vsize, config->vsize);

out_unlock:
	spin_unlock_irqrestore(&chan->lock, flags);
}

static void xilinx_vdma_issue_pending(struct dma_chan *dchan)
{
	struct xilinx_vdma_chan *chan = to_xilinx_chan(dchan);

	xilinx_vdma_start_transfer(chan);
}

/**
 * xilinx_vdma_update_completed_cookie - Update the completed cookie.
 * @chan : xilinx DMA channel
 *
 * CONTEXT: hardirq
 */
static void xilinx_vdma_update_completed_cookie(struct xilinx_vdma_chan *chan)
{
	struct xilinx_vdma_desc_sw *desc = NULL;
	unsigned long flags;
	dma_cookie_t cookie = -EBUSY;
	int done = 0;

	spin_lock_irqsave(&chan->lock, flags);

	if (list_empty(&chan->active_list)) {
		dev_dbg(chan->dev, "no running descriptors\n");
		goto out_unlock;
	}

	/* Get the last completed descriptor, update the cookie to that */
	list_for_each_entry(desc, &chan->active_list, node) {
		/* In non-SG mode, all active entries are done */
		done = 1;
		cookie = desc->async_tx.cookie;
	}

	if (done)
		chan->completed_cookie = cookie;

out_unlock:
	spin_unlock_irqrestore(&chan->lock, flags);
}

/* Reset hardware */
static int vdma_init(struct xilinx_vdma_chan *chan)
{
	int loop = XILINX_VDMA_RESET_LOOP;
	u32 tmp;

	VDMA_OUT(&chan->regs->cr,
		VDMA_IN(&chan->regs->cr) | XILINX_VDMA_CR_RESET_MASK);

	tmp = VDMA_IN(&chan->regs->cr) & XILINX_VDMA_CR_RESET_MASK;

	/* Wait for the hardware to finish reset */
	while (loop && tmp) {
		tmp = VDMA_IN(&chan->regs->cr) & XILINX_VDMA_CR_RESET_MASK;
		loop -= 1;
	}

	if (!loop) {
		dev_err(chan->dev, "reset timeout, cr %x, sr %x\n",
			VDMA_IN(&chan->regs->cr), VDMA_IN(&chan->regs->sr));
		return 1;
	}

	return 0;
}


static irqreturn_t vdma_intr_handler(int irq, void *data)
{
	struct xilinx_vdma_chan *chan = data;
	int update_cookie = 0;
	int to_transfer = 0;
	u32 stat, reg;

	reg = VDMA_IN(&chan->regs->cr);

	/* Disable intr */
	VDMA_OUT(&chan->regs->cr,
		reg & ~XILINX_VDMA_XR_IRQ_ALL_MASK);

	stat = VDMA_IN(&chan->regs->sr);
	if (!(stat & XILINX_VDMA_XR_IRQ_ALL_MASK))
		return IRQ_NONE;

	/* Ack the interrupts */
	VDMA_OUT(&chan->regs->sr, XILINX_VDMA_XR_IRQ_ALL_MASK);

	/* Check for only the interrupts which are enabled */
	stat &= (reg & XILINX_VDMA_XR_IRQ_ALL_MASK);

	if (stat & XILINX_VDMA_XR_IRQ_ERROR_MASK) {
		if (chan->flush_fsync) {
			/*
			 * VDMA Recoverable Errors, only when
			 * C_FLUSH_ON_FSYNC is enabled
			 */
			u32 error = VDMA_IN(&chan->regs->sr) &
				XILINX_VDMA_SR_ERR_RECOVER_MASK;
			if (error)
				VDMA_OUT(&chan->regs->sr, error);
			else
				chan->err = 1;
		} else {
			dev_err(chan->dev,
				"Channel %x has errors %x, cdr %x tdr %x\n",
				(unsigned int)chan,
				(unsigned int)VDMA_IN(&chan->regs->sr),
				(unsigned int)VDMA_IN(&chan->regs->cdr),
				(unsigned int)VDMA_IN(&chan->regs->tdr));
			chan->err = 1;
		}
	}

	/*
	 * Device takes too long to do the transfer when user requires
	 * responsiveness
	 */
	if (stat & XILINX_VDMA_XR_IRQ_DELAY_MASK)
		dev_dbg(chan->dev, "Inter-packet latency too long\n");

	if (stat & XILINX_VDMA_XR_IRQ_IOC_MASK) {
		update_cookie = 1;
		to_transfer = 1;
	}

	if (update_cookie)
		xilinx_vdma_update_completed_cookie(chan);

	if (to_transfer)
		chan->start_transfer(chan);

	tasklet_schedule(&chan->tasklet);
	return IRQ_HANDLED;
}

static void dma_do_tasklet(unsigned long data)
{
	struct xilinx_vdma_chan *chan = (struct xilinx_vdma_chan *)data;

	xilinx_chan_desc_cleanup(chan);
}

/* Append the descriptor list to the pending list */
static void append_desc_queue(struct xilinx_vdma_chan *chan,
			struct xilinx_vdma_desc_sw *desc)
{
	struct xilinx_vdma_desc_sw *tail = container_of(chan->pending_list.prev,
					struct xilinx_vdma_desc_sw, node);
	struct xilinx_vdma_desc_hw *hw;

	if (list_empty(&chan->pending_list))
		goto out_splice;

	/*
	 * Add the hardware descriptor to the chain of hardware descriptors
	 * that already exists in memory.
	 */
	hw = &(tail->hw);
	hw->next_desc = (u32)desc->async_tx.phys;

	/*
	 * Add the software descriptor and all children to the list
	 * of pending transactions
	 */
out_splice:
	list_splice_tail_init(&desc->tx_list, &chan->pending_list);
}

/*
 * Assign cookie to each descriptor, and append the descriptors to the pending
 * list
 */
static dma_cookie_t xilinx_vdma_tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct xilinx_vdma_chan *chan = to_xilinx_chan(tx->chan);
	struct xilinx_vdma_desc_sw *desc = container_of(tx,
				struct xilinx_vdma_desc_sw, async_tx);
	struct xilinx_vdma_desc_sw *child;
	unsigned long flags;
	dma_cookie_t cookie = -EBUSY;

	if (chan->err) {
		/*
		 * If reset fails, need to hard reset the system.
		 * Channel is no longer functional
		 */
		if (!vdma_init(chan))
			chan->err = 0;
		else
			return cookie;
	}

	spin_lock_irqsave(&chan->lock, flags);

	/*
	 * assign cookies to all of the software descriptors
	 * that make up this transaction
	 */
	cookie = chan->cookie;
	list_for_each_entry(child, &desc->tx_list, node) {
		cookie++;
		if (cookie < 0)
			cookie = DMA_MIN_COOKIE;

		child->async_tx.cookie = cookie;
	}

	chan->cookie = cookie;

	/* Put this transaction onto the tail of the pending queue */
	append_desc_queue(chan, desc);

	spin_unlock_irqrestore(&chan->lock, flags);

	return cookie;
}

static struct xilinx_vdma_desc_sw *xilinx_vdma_alloc_descriptor(
					struct xilinx_vdma_chan *chan)
{
	struct xilinx_vdma_desc_sw *desc;
	dma_addr_t pdesc;

	desc = dma_pool_alloc(chan->desc_pool, GFP_ATOMIC, &pdesc);
	if (!desc) {
		dev_dbg(chan->dev, "out of memory for desc\n");
		return NULL;
	}

	memset(desc, 0, sizeof(*desc));
	INIT_LIST_HEAD(&desc->tx_list);
	dma_async_tx_descriptor_init(&desc->async_tx, &chan->common);
	desc->async_tx.tx_submit = xilinx_vdma_tx_submit;
	desc->async_tx.phys = pdesc;

	return desc;
}

/**
 * xilinx_vdma_prep_slave_sg - prepare descriptors for a DMA_SLAVE transaction
 * @chan: VDMA channel
 * @sgl: scatterlist to transfer to/from
 * @sg_len: number of entries in @scatterlist
 * @direction: DMA direction
 * @flags: transfer ack flags
 */
static struct dma_async_tx_descriptor *xilinx_vdma_prep_slave_sg(
	struct dma_chan *dchan, struct scatterlist *sgl, unsigned int sg_len,
	enum dma_transfer_direction direction, unsigned long flags,
	void *context)
{
	struct xilinx_vdma_chan *chan;
	struct xilinx_vdma_desc_sw *first = NULL, *prev = NULL, *new = NULL;
	struct xilinx_vdma_desc_hw *hw = NULL, *prev_hw = NULL;
	int i;
	struct scatterlist *sg;
	dma_addr_t dma_src;

	if (!dchan)
		return NULL;

	chan = to_xilinx_chan(dchan);

	if (chan->direction != direction)
		return NULL;

	/* Enforce one sg entry for one frame */
	if (sg_len != chan->num_frms) {
		dev_err(chan->dev,
		"number of entries %d not the same as num stores %d\n",
			sg_len, chan->num_frms);

		return NULL;
	}

	if (!chan->has_sg) {
		VDMA_OUT(&chan->addr_regs->hsize, chan->config.hsize);
		VDMA_OUT(&chan->addr_regs->frmdly_stride,
			chan->config.frm_dly << XILINX_VDMA_FRMDLY_SHIFT |
			chan->config.stride);
	}

	/* Build transactions using information in the scatter gather list */
	for_each_sg(sgl, sg, sg_len, i) {
		/* Allocate the link descriptor from DMA pool */
		new = xilinx_vdma_alloc_descriptor(chan);
		if (!new) {
			dev_err(chan->dev,
				"No free memory for link descriptor\n");
			goto fail;
		}

		/*
		 * Calculate the maximum number of bytes to transfer,
		 * making sure it is less than the hw limit
		 */
		hw = &(new->hw);

		dma_src = sg_dma_address(sg);
		if (chan->has_sg) {
			hw->buf_addr = dma_src;

			/* Fill in the descriptor */
			hw->vsize = chan->config.vsize;
			hw->hsize = chan->config.hsize;
			hw->stride = (chan->config.frm_dly <<
					XILINX_VDMA_FRMDLY_SHIFT) |
					chan->config.stride;
		} else {
			/* Update the registers */
			VDMA_OUT(&(chan->addr_regs->buf_addr[i]), dma_src);
		}

		/*
		 * If this is not the first descriptor, chain the
		 * current descriptor after the previous descriptor
		 */
		if (!first) {
			first = new;
		} else {
			prev_hw = &(prev->hw);
			prev_hw->next_desc = new->async_tx.phys;
		}

		new->async_tx.cookie = 0;
		async_tx_ack(&new->async_tx);

		prev = new;

		/* Insert the link descriptor into the list */
		list_add_tail(&new->node, &first->tx_list);
	}

	/* Link the last BD with the first BD */
	hw->next_desc = first->async_tx.phys;

	if (!first || !new)
		return NULL;

	new->async_tx.flags = flags;
	new->async_tx.cookie = -EBUSY;

	return &first->async_tx;

fail:
	/*
	 * If first was not set, then we failed to allocate the very first
	 * descriptor, and we're done */
	if (!first)
		return NULL;

	/*
	 * First is set, so all of the descriptors we allocated have been added
	 * to first->tx_list, INCLUDING "first" itself. Therefore we
	 * must traverse the list backwards freeing each descriptor in turn
	 */
	xilinx_vdma_free_desc_list_reverse(chan, &first->tx_list);
	return NULL;
}

/*
 * Run-time configuration for Axi VDMA, supports:
 * . halt the channel
 * . configure interrupt coalescing and inter-packet delay threshold
 * . start/stop parking
 * . enable genlock
 * . set transfer information using config struct
 */
static int xilinx_vdma_device_control(struct dma_chan *dchan,
				enum dma_ctrl_cmd cmd, unsigned long arg)
{
	struct xilinx_vdma_chan *chan;
	unsigned long flags;

	if (!dchan)
		return -EINVAL;

	chan = to_xilinx_chan(dchan);

	if (cmd == DMA_TERMINATE_ALL) {
		/* Halt the DMA engine */
		vdma_halt(chan);

		spin_lock_irqsave(&chan->lock, flags);

		/* Remove and free all of the descriptors in the lists */
		xilinx_vdma_free_desc_list(chan, &chan->pending_list);
		xilinx_vdma_free_desc_list(chan, &chan->active_list);

		spin_unlock_irqrestore(&chan->lock, flags);
		return 0;
	} else if (cmd == DMA_SLAVE_CONFIG) {
		struct xilinx_vdma_config *cfg =
				(struct xilinx_vdma_config *)arg;
		u32 reg;

		if (cfg->reset)
			vdma_init(chan);

		reg = VDMA_IN(&chan->regs->cr);

		/* If vsize is -1, it is park-related operations */
		if (cfg->vsize == -1) {
			if (cfg->park)
				reg &= ~XILINX_VDMA_CIRC_EN;
			else
				reg |= XILINX_VDMA_CIRC_EN;

			VDMA_OUT(&chan->regs->cr, reg);
			return 0;
		}

		/* If hsize is -1, it is interrupt threshold settings */
		if (cfg->hsize == -1) {
			if (cfg->coalesc <= XILINX_VDMA_COALESCE_MAX) {
				reg &= ~XILINX_VDMA_XR_COALESCE_MASK;
				reg |= cfg->coalesc <<
					XILINX_VDMA_COALESCE_SHIFT;
				chan->config.coalesc = cfg->coalesc;
			}

			if (cfg->delay <= XILINX_VDMA_DELAY_MAX) {
				reg &= ~XILINX_VDMA_XR_DELAY_MASK;
				reg |= cfg->delay << XILINX_VDMA_DELAY_SHIFT;
				chan->config.delay = cfg->delay;
			}

			VDMA_OUT(&chan->regs->cr, reg);
			return 0;
		}

		/* Transfer information */
		chan->config.vsize = cfg->vsize;
		chan->config.hsize = cfg->hsize;
		chan->config.stride = cfg->stride;
		chan->config.frm_dly = cfg->frm_dly;
		chan->config.park = cfg->park;
		chan->config.direction = cfg->direction;

		/* genlock settings */
		chan->config.gen_lock = cfg->gen_lock;
		chan->config.master = cfg->master;

		if (cfg->gen_lock) {
			if (chan->genlock) {
				reg |= XILINX_VDMA_SYNC_EN;
				reg |= cfg->master << XILINX_VDMA_MSTR_SHIFT;
			}
		}

		chan->config.frm_cnt_en = cfg->frm_cnt_en;
		if (cfg->park)
			chan->config.park_frm = cfg->park_frm;
		else
			chan->config.park_frm = -1;

		chan->config.coalesc = cfg->coalesc;
		chan->config.delay = cfg->delay;
		if (cfg->coalesc <= XILINX_VDMA_COALESCE_MAX) {
			reg |= cfg->coalesc << XILINX_VDMA_COALESCE_SHIFT;
			chan->config.coalesc = cfg->coalesc;
		}

		if (cfg->delay <= XILINX_VDMA_DELAY_MAX) {
			reg |= cfg->delay << XILINX_VDMA_DELAY_SHIFT;
			chan->config.delay = cfg->delay;
		}

		chan->config.disable_intr = cfg->disable_intr;

		/* FSync Source selection */
		reg &= ~XILINX_VDMA_CR_FSYNC_SRC_MASK;
		reg |= cfg->ext_fsync << XILINX_VDMA_EXTFSYNC_SHIFT;

		VDMA_OUT(&chan->regs->cr, reg);
		return 0;
	} else
		return -ENXIO;
}


/*
 * Logarithm function to compute alignment shift
 * Only deals with value less than 4096.
 */
static int my_log(int value)
{
	int i = 0;
	while ((1 << i) < value) {
		i++;

		if (i >= 12)
			return 0;
	}

	return i;
}

static void xilinx_vdma_chan_remove(struct xilinx_vdma_chan *chan)
{
	irq_dispose_mapping(chan->irq);
	list_del(&chan->common.device_node);
	kfree(chan);
}

/*
 * Probing channels
 *
 * . Get channel features from the device tree entry
 * . Initialize special channel handling routines
 */
static int xilinx_vdma_chan_probe(struct xilinx_vdma_device *xdev,
	struct device_node *node, u32 feature)
{
	struct xilinx_vdma_chan *chan;
	int err;
	const __be32 *value;
	u32 width = 0, device_id = 0, flush_fsync = 0;

	/* Alloc channel */
	chan = kzalloc(sizeof(*chan), GFP_KERNEL);
	if (!chan) {
		dev_err(xdev->dev, "no free memory for DMA channels!\n");
		err = -ENOMEM;
		goto out_return;
	}

	chan->feature = feature;
	chan->max_len = XILINX_VDMA_MAX_TRANS_LEN;

	value = of_get_property(node, "xlnx,include-dre", NULL);
	if (value)
		chan->has_dre = be32_to_cpup(value);

	value = (int *)of_get_property(node, "xlnx,genlock-mode", NULL);
	if (value)
		chan->genlock = be32_to_cpup(value);

	value = (int *)of_get_property(node, "xlnx,datawidth", NULL);
	if (value) {
		width = be32_to_cpup(value) >> 3; /* Convert bits to bytes */

		/* If data width is greater than 8 bytes, DRE is not in hw */
		if (width > 8)
			chan->has_dre = 0;

		chan->feature |= width - 1;
	}

	value = (int *)of_get_property(node, "xlnx,device-id", NULL);
	if (value)
		device_id = be32_to_cpup(value);

	flush_fsync = (xdev->feature & XILINX_VDMA_FTR_FLUSH_MASK) >>
			XILINX_VDMA_FTR_FLUSH_SHIFT;

	chan->start_transfer = xilinx_vdma_start_transfer;

	chan->has_sg = (xdev->feature & XILINX_VDMA_FTR_HAS_SG) >>
		XILINX_VDMA_FTR_HAS_SG_SHIFT;

	if (of_device_is_compatible(node,
			"xlnx,axi-vdma-mm2s-channel")) {
		chan->direction = DMA_MEM_TO_DEV;
		if (!chan->has_sg) {
			chan->addr_regs = (struct vdma_addr_regs *)
			    ((u32)xdev->regs +
				 XILINX_VDMA_DIRECT_REG_OFFSET);
		}

		if (flush_fsync == XILINX_VDMA_FLUSH_BOTH ||
			flush_fsync == XILINX_VDMA_FLUSH_MM2S)
			chan->flush_fsync = 1;
	}

	if (of_device_is_compatible(node, "xlnx,axi-vdma-s2mm-channel")) {
		chan->direction = DMA_DEV_TO_MEM;
		if (!chan->has_sg) {
			chan->addr_regs = (struct vdma_addr_regs *)
			    ((u32)xdev->regs +
				XILINX_VDMA_DIRECT_REG_OFFSET +
				XILINX_VDMA_CHAN_DIRECT_REG_SIZE);
		}

		if (flush_fsync == XILINX_VDMA_FLUSH_BOTH ||
			flush_fsync == XILINX_VDMA_FLUSH_S2MM)
			chan->flush_fsync = 1;
	}

	chan->regs = (struct xvdma_regs *)xdev->regs;

	if (chan->direction == DMA_DEV_TO_MEM) {
		chan->regs = (struct xvdma_regs *)((u32)xdev->regs +
					XILINX_VDMA_RX_CHANNEL_OFFSET);
		chan->id = 1;
	}

	/*
	 * Used by dmatest channel matching in slave transfers
	 * Can change it to be a structure to have more matching information
	 */
	chan->private = (chan->direction & 0xFF) |
		(chan->feature & XILINX_DMA_IP_MASK) |
		(device_id << XILINX_DMA_DEVICE_ID_SHIFT);
	chan->common.private = (void *)&(chan->private);

	if (!chan->has_dre)
		xdev->common.copy_align = my_log(width);

	chan->dev = xdev->dev;
	xdev->chan[chan->id] = chan;

	tasklet_init(&chan->tasklet, dma_do_tasklet, (unsigned long)chan);

	/* Initialize the channel */
	if (vdma_init(chan)) {
		dev_err(xdev->dev, "Reset channel failed\n");
		goto out_free_chan;
	}

	spin_lock_init(&chan->lock);
	INIT_LIST_HEAD(&chan->pending_list);
	INIT_LIST_HEAD(&chan->active_list);

	chan->common.device = &xdev->common;

	/* Find the IRQ line, if it exists in the device tree */
	chan->irq = irq_of_parse_and_map(node, 0);
	err = request_irq(chan->irq, vdma_intr_handler, IRQF_SHARED,
				"xilinx-vdma-controller", chan);
	if (err) {
		dev_err(xdev->dev, "unable to request IRQ\n");
		goto out_free_irq;
	}

	/* Add the channel to DMA device channel list */
	list_add_tail(&chan->common.device_node, &xdev->common.channels);
	xdev->common.chancnt++;

	return 0;

out_free_irq:
	irq_dispose_mapping(chan->irq);
out_free_chan:
	kfree(chan);
out_return:
	return err;
}

static int xilinx_vdma_of_probe(struct platform_device *op)
{
	struct xilinx_vdma_device *xdev;
	struct device_node *child, *node;
	int err, i;
	const __be32 *value;
	int num_frames = 0;

	dev_info(&op->dev, "Probing xilinx axi vdma engine\n");

	xdev = kzalloc(sizeof(struct xilinx_vdma_device), GFP_KERNEL);
	if (!xdev) {
		dev_err(&op->dev, "Not enough memory for device\n");
		err = -ENOMEM;
		goto out_return;
	}

	xdev->dev = &(op->dev);
	INIT_LIST_HEAD(&xdev->common.channels);

	node = op->dev.of_node;
	xdev->feature = 0;

	/* iomap registers */
	xdev->regs = of_iomap(node, 0);
	if (!xdev->regs) {
		dev_err(&op->dev, "unable to iomap registers\n");
		err = -ENOMEM;
		goto out_free_xdev;
	}

	/* Axi VDMA only do slave transfers */
	if (of_device_is_compatible(node, "xlnx,axi-vdma")) {
		xdev->feature |= XILINX_DMA_IP_VDMA;

		value = of_get_property(node, "xlnx,include-sg", NULL);
		if (value) {
			if (be32_to_cpup(value) == 1)
				xdev->feature |= XILINX_VDMA_FTR_HAS_SG;
		}

		value = of_get_property(node, "xlnx,num-fstores", NULL);
		if (value)
			num_frames = be32_to_cpup(value);

		value = of_get_property(node, "xlnx,flush-fsync", NULL);
		if (value)
			xdev->feature |= be32_to_cpup(value) <<
				XILINX_VDMA_FTR_FLUSH_SHIFT;

		dma_cap_set(DMA_SLAVE, xdev->common.cap_mask);
		dma_cap_set(DMA_PRIVATE, xdev->common.cap_mask);
		xdev->common.device_prep_slave_sg = xilinx_vdma_prep_slave_sg;
		xdev->common.device_control = xilinx_vdma_device_control;
		xdev->common.device_issue_pending = xilinx_vdma_issue_pending;
	}

	xdev->common.device_alloc_chan_resources =
				xilinx_vdma_alloc_chan_resources;
	xdev->common.device_free_chan_resources =
				xilinx_vdma_free_chan_resources;
	xdev->common.device_tx_status = xilinx_tx_status;
	xdev->common.dev = &op->dev;

	platform_set_drvdata(op, xdev);

	for_each_child_of_node(node, child) {
		xilinx_vdma_chan_probe(xdev, child, xdev->feature);
	}

	for (i = 0; i < XILINX_VDMA_MAX_CHANS_PER_DEVICE; i++) {
		if (xdev->chan[i])
			xdev->chan[i]->num_frms = num_frames;
	}

	dma_async_device_register(&xdev->common);

	return 0;

out_free_xdev:
	kfree(xdev);

out_return:
	return err;
}

static int xilinx_vdma_of_remove(struct platform_device *op)
{
	struct xilinx_vdma_device *xdev;
	int i;

	xdev = platform_get_drvdata(op);
	dma_async_device_unregister(&xdev->common);

	for (i = 0; i < XILINX_VDMA_MAX_CHANS_PER_DEVICE; i++) {
		if (xdev->chan[i])
			xilinx_vdma_chan_remove(xdev->chan[i]);
	}

	iounmap(xdev->regs);
	kfree(xdev);

	return 0;
}

static const struct of_device_id xilinx_vdma_of_ids[] = {
	{ .compatible = "xlnx,axi-vdma",},
	{}
};

static struct platform_driver xilinx_vdma_of_driver = {
	.driver = {
		.name = "xilinx-vdma",
		.owner = THIS_MODULE,
		.of_match_table = xilinx_vdma_of_ids,
	},
	.probe = xilinx_vdma_of_probe,
	.remove = xilinx_vdma_of_remove,
};

module_platform_driver(xilinx_vdma_of_driver);

MODULE_AUTHOR("Xilinx, Inc.");
MODULE_DESCRIPTION("Xilinx VDMA driver");
MODULE_LICENSE("GPL v2");
