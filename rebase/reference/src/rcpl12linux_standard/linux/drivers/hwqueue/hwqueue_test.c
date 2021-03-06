/*
 * Hardware queue test framework
 *
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "hwqueue-test: " fmt
#define irqpop_test1

#include <linux/kernel.h>
#include <linux/hwqueue.h>
#include <linux/module.h>
#include <linux/types.h>

struct hwqueue_test_ctx {
	struct hwqueue *qpool;
	struct hwqueue *qgenwr;
	struct hwqueue *qgenrd;
	struct hwqueue *qirqwr;
	struct hwqueue *qirqrd;
	unsigned desc_size;
	bool notified;
	bool notified_irq;
	bool notify_added;
	bool notify_irq_added;
	dma_addr_t desc;
};

static void hwqueue_test_notify(void *arg)
{
	struct hwqueue_test_ctx *ctx = arg;

	ctx->notified = true;
	pr_info("notification fired\n");
}

static void hwqueue_test_notify_irq(void *arg)
{
	struct hwqueue_test_ctx *ctx = arg;


	ctx->notified_irq = true;
	pr_info("notification for irq fired\n");

#ifdef irqpop_test1
	hwqueue_set_notifier(ctx->qirqrd, NULL, NULL);
	ctx->notify_irq_added = false;
#else
	unsigned desc_size;
	dma_addr_t desc;
	desc = hwqueue_pop(ctx->qirqrd, &desc_size, NULL, 0);
	if (!desc)
		pr_err("test3: failed to pop desc from read queue\n");

	pr_info("test3: popped desc %08x, size %d\n", desc, desc_size);

	if (desc != ctx->desc || desc_size != ctx->desc_size)
		pr_err("test3: unexpected descriptor popped on read\n");
#endif
}

static int __init hwqueue_test(void)
{
	unsigned long end_time, timeout = 1000;
	struct hwqueue_test_ctx ctx;
	struct hwqueue *qpool;
	struct hwqueue *qgenwr, *qgenrd;
	struct hwqueue *qirqwr, *qirqrd;
	unsigned desc_size;
	dma_addr_t desc;
	int ret;

	memset(&ctx, 0, sizeof(ctx));

	qpool = hwqueue_open("pool-net", HWQUEUE_BYNAME, O_RDWR);
	if (IS_ERR_OR_NULL(qpool)) {
		ret = PTR_ERR(qpool);
		pr_err("failed to open pool queue, errno=%d\n", ret);
		goto fail;
	}
	pr_info("opened qpool, id=%d\n", hwqueue_get_id(qpool));
	ctx.qpool = qpool;

	qgenwr = hwqueue_open("test", HWQUEUE_ANY, O_WRONLY | O_CREAT);
	if (IS_ERR_OR_NULL(qgenwr)) {
		ret = PTR_ERR(qgenwr);
		pr_err("failed to open write queue, errno=%d\n", ret);
		goto fail;
	}
	pr_info("opened qgenwr, id=%d qh = %p\n",
		hwqueue_get_id(qgenwr), qgenwr);
	ctx.qgenwr = qgenwr;

	qgenrd = hwqueue_open("test", hwqueue_get_id(qgenwr), O_RDONLY);
	if (IS_ERR_OR_NULL(qgenrd)) {
		ret = PTR_ERR(qgenrd);
		pr_err("failed to open read queue, errno=%d\n", ret);
		goto fail;
	}
	pr_info("opened qgenrd, id = %d, qh = %p\n",
		hwqueue_get_id(qgenrd), qgenrd);
	ctx.qgenrd = qgenrd;

	qirqwr = hwqueue_open("irq1", 655, O_LOWLATENCY | O_RDWR);
	if (IS_ERR_OR_NULL(qirqwr)) {
		ret = PTR_ERR(qirqwr);
		pr_err("failed to open write queue, errno=%d\n", ret);
		goto fail;
	}
	pr_info("opened qirqwr, id = %d qh = %p\n",
		hwqueue_get_id(qirqwr), qirqwr);
	ctx.qirqwr = qirqwr;

	qirqrd = hwqueue_open("irq1", hwqueue_get_id(qirqwr), O_RDONLY);
	if (IS_ERR_OR_NULL(qirqrd)) {
		ret = PTR_ERR(qirqrd);
		pr_err("failed to open read queue, errno=%d\n", ret);
		goto fail;
	}
	pr_info("opened qirqrd, id=%d qh = %p\n",
		hwqueue_get_id(qirqrd), qirqrd);
	ctx.qirqrd = qirqrd;

	ret = hwqueue_set_notifier(qirqrd, hwqueue_test_notify_irq, &ctx);
	if (ret < 0) {
		pr_err("failed to set notifier, errno=%d\n", ret);
		goto fail;
	}
	ctx.notify_irq_added = true;

	ret = hwqueue_set_notifier(qgenrd, hwqueue_test_notify, &ctx);
	if (ret < 0) {
		pr_err("failed to set notifier, errno=%d\n", ret);
		goto fail;
	}
	ctx.notify_added = true;

	desc = hwqueue_pop(qpool, &desc_size, NULL, 0);
	if (!desc) {
		ret = -ENOENT;
		pr_err("failed to pop pool desc\n");
		goto fail;
	}
	pr_info("popped desc from qpool %08x, size %d\n", desc, desc_size);
	ctx.desc = desc;
	ctx.desc_size = desc_size;

	/* Test 1: Simple push and pop */
	pr_err("test1: simple pop test:\n");
	ret = hwqueue_push(qgenwr, desc, desc_size, 0);
	if (ret < 0) {
		pr_err("test1: failed to push desc, errno=%d\n", ret);
		goto fail;
	}
	pr_info("test1: pushed desc %08x, size %d\n", desc, desc_size);

	desc = hwqueue_pop(qgenrd, &desc_size, NULL, 0);
	if (!desc) {
		ret = -ENOENT;
		pr_err("test1: failed to pop desc from read queue\n");
		goto fail;
	}
	pr_info("test1: popped desc %08x, size %d\n", desc, desc_size);

	if (desc != ctx.desc || desc_size != ctx.desc_size) {
		ret = -EINVAL;
		pr_err("unexpected descriptor popped on read\n");
	}

	/* Test 2: Notified push and pop */
	pr_err("test2: poll notified pop test:\n");
	ret = hwqueue_push(qgenwr, desc, desc_size, 0);
	if (ret < 0) {
		pr_err("failed to push desc, errno=%d\n", ret);
		goto fail;
	}
	pr_info("test2: pushed desc %08x, size %d\n", desc, desc_size);

	end_time = jiffies + msecs_to_jiffies(timeout);
	while (jiffies < end_time) {
		if (ctx.notified)
			break;
		cpu_relax();
	}
	if (!ctx.notified)
		pr_err("test2: timed out waiting for hwqueue notification\n");
	ctx.notified = false;

	desc = hwqueue_pop(qgenrd, &desc_size, NULL, 0);
	if (!desc) {
		ret = -ENOENT;
		pr_err("test2: failed to pop desc from read queue\n");
		goto fail;
	}
	pr_info("test2: popped desc %08x, size %d\n", desc, desc_size);

	if (desc != ctx.desc || desc_size != ctx.desc_size) {
		ret = -EINVAL;
		pr_err("test2: unexpected descriptor popped on read\n");
	}

	/* Test 3: IRQ push, notify and pop */
	pr_err("test3: irq notified pop test:\n");
	pr_info("test3: pushing desc %08x, size %d\n", desc, desc_size);
	ret = hwqueue_push(qirqwr, desc, desc_size, 0);
	if (ret < 0) {
		pr_err("failed to push desc, errno=%d\n", ret);
		goto fail;
	}

	end_time = jiffies + msecs_to_jiffies(timeout);
	while (jiffies < end_time) {
		if (ctx.notified_irq)
			break;
		cpu_relax();
	}
	if (!ctx.notified_irq)
		pr_err("test3: timed out waiting for hwqueue notification\n");
	ctx.notified_irq = false;

#ifdef irqpop_test1
	desc = hwqueue_pop(qirqrd, &desc_size, NULL, 0);
	if (!desc) {
		ret = -ENOENT;
		pr_err("test3: failed to pop desc from read queue\n");
		goto fail;
	}
	pr_info("test3: popped desc %08x, size %d\n", desc, desc_size);

	if (desc != ctx.desc || desc_size != ctx.desc_size) {
		ret = -EINVAL;
		pr_err("test3: unexpected descriptor popped on read\n");
	}
#endif

fail:
	if (ctx.desc)
		hwqueue_push(ctx.qpool, ctx.desc, ctx.desc_size, 0);
	if (ctx.notify_added)
		hwqueue_set_notifier(ctx.qgenrd, NULL, NULL);
	if (ctx.notify_irq_added)
		hwqueue_set_notifier(ctx.qirqrd, NULL, NULL);
	if (ctx.qgenrd)
		hwqueue_close(ctx.qgenrd);
	if (ctx.qgenwr)
		hwqueue_close(ctx.qgenwr);
	if (ctx.qirqwr)
		hwqueue_close(ctx.qirqwr);
	if (ctx.qirqrd)
		hwqueue_close(ctx.qirqrd);
	if (ctx.qpool)
		hwqueue_close(ctx.qpool);
	return ret;
}
module_init(hwqueue_test);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Hardware queue test driver");
