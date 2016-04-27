/*
 * Copyright (C) 2012 Texas Instruments Incorporated
 * Authors: Cyril Chemparathy <cyril@ti.com
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

#ifndef __MACH_KEYSTONE_DMA_H__
#define __MACH_KEYSTONE_DMA_H__

#include <linux/dmaengine.h>

#define DMA_HAS_PSINFO		BIT(31)
#define DMA_HAS_EPIB		BIT(30)
#define DMA_HAS_FLOWTAG		BIT(29)
#define DMA_QNUM_SHIFT		24
#define DMA_QNUM_MASK		BITS(2)
#define DMA_FLOWTAG_SHIFT	18
#define DMA_FLOWTAG_MASK	BITS(6)
#define DMA_PORT(x)		x

typedef void (*dma_notify_fn)(struct dma_chan *chan, void *arg);

#define DMA_GET_RX_FLOW		1000
#define DMA_GET_RX_QUEUE	1001
#define DMA_POLL		1002
#define DMA_SET_NOTIFY		1003
#define DMA_KEYSTONE_CONFIG	1004
#define DMA_RXFREE_REFILL	1005
#define DMA_GET_TX_QUEUE	1006

struct dma_notify_info {
	dma_notify_fn		 fn;
	void			*fn_arg;
};

static inline int dma_get_rx_flow(struct dma_chan *chan)
{
	return dmaengine_device_control(chan, DMA_GET_RX_FLOW, 0);
}

static inline int dma_get_rx_queue(struct dma_chan *chan)
{
	return dmaengine_device_control(chan, DMA_GET_RX_QUEUE, 0);
}

static inline int dma_poll(struct dma_chan *chan, int budget)
{
	return dmaengine_device_control(chan, DMA_POLL, budget);
}

static inline int dma_set_notify(struct dma_chan *chan, dma_notify_fn fn,
				 void *fn_arg)
{
	struct dma_notify_info info;
	info.fn = fn;
	info.fn_arg = fn_arg;
	return dmaengine_device_control(chan, DMA_SET_NOTIFY,
					(unsigned long)&info);
}

static inline int dma_get_tx_queue(struct dma_chan *chan)
{
	return dmaengine_device_control(chan, DMA_GET_TX_QUEUE, 0);
}

/* Number of RX queues supported per channel */
#define	KEYSTONE_QUEUES_PER_CHAN	4

enum dma_keystone_thresholds {
	DMA_THRESH_NONE		= 0,
	DMA_THRESH_0		= 1,
	DMA_THRESH_0_1		= 3,
	DMA_THRESH_0_1_2	= 7
};

typedef struct dma_async_tx_descriptor *(*dma_rxpool_alloc_fn)(
		void *arg, unsigned q_num, unsigned bufsize);
typedef void (*dma_rxpool_free_fn)(void *arg, unsigned q_num, unsigned bufsize,
		struct dma_async_tx_descriptor *desc);

struct dma_keystone_info {
	enum dma_transfer_direction	 direction;
	unsigned int			 scatterlist_size;
	unsigned int			 tx_queue_depth;
	dma_rxpool_alloc_fn		 rxpool_allocator;
	dma_rxpool_free_fn		 rxpool_destructor;
	void				*rxpool_param;
	unsigned int			 rxpool_count;
	enum dma_keystone_thresholds	 rxpool_thresh_enable;
	struct {
		unsigned	pool_depth;
		unsigned	buffer_size;
	}				 rxpools[KEYSTONE_QUEUES_PER_CHAN];
};

static inline int dma_keystone_config(struct dma_chan *chan,
		struct dma_keystone_info *info)
{
	return dmaengine_device_control(chan, DMA_KEYSTONE_CONFIG,
					(unsigned long)info);
}

static inline int dma_rxfree_refill(struct dma_chan *chan)
{
	return dmaengine_device_control(chan, DMA_RXFREE_REFILL, 0);
}

#endif /* __MACH_KEYSTONE_DMA_H__ */

