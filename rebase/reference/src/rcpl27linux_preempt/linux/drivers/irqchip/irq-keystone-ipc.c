/*
 * Texas Instruments Keystone IPC IRQ chip
 *
 * Copyright (C) 2012 Texas Instruments, Inc.
 * Author: Sajesh Kumar Saran <sajesh@ti.com>
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

#include <linux/io.h>
#include <linux/of.h>
#include <linux/irq.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/irqchip/keystone-ipc.h>

/* The source ID bits start from 4 to 31 (total 28 bits)*/
#define BIT_OFS			4
#define N_IPC_IRQ		(32 - BIT_OFS)

static bool debug_keystone_ipc;

struct keystone_ipc_device {
	void __iomem		*grh;
	void __iomem		*arh;
	struct irq_chip		 chip;
	u32			 mask;
	u32			 irq;
	struct irq_domain	*irqd;
};

#define from_irq_chip(ipc)		\
	container_of(ipc, struct keystone_ipc_device, chip)
#define to_irq_chip(ipc)		\
	(&(ipc)->chip)

static inline u32 keystone_ipc_readl(struct keystone_ipc_device *kipc)
{
	return __raw_readl(kipc->arh);
}

static inline void keystone_ipc_writel(struct keystone_ipc_device *kipc,
				       u32 value)
{
	__raw_writel(value, kipc->arh);
}

static void keystone_ipc_mask_irq(struct irq_data *d)
{
	struct keystone_ipc_device *kipc = from_irq_chip(d->chip);
	unsigned int irq = d->hwirq;

	kipc->mask |= BIT(irq);
	if (debug_keystone_ipc)
		pr_debug("ipc irq: mask %d [%x]\n", irq, kipc->mask);
}

void keystone_ipc_unmask_irq(struct irq_data *d)
{
	struct keystone_ipc_device *kipc = from_irq_chip(d->chip);
	unsigned int irq = d->hwirq;

	kipc->mask &= ~BIT(irq);
	if (debug_keystone_ipc)
		pr_debug("ipc irq: unmask %d [%x]\n",
			irq, kipc->mask);
}

void keystone_ipc_ack_irq(struct irq_data *d)
{
	struct keystone_ipc_device *kipc = from_irq_chip(d->chip);
	unsigned int irq = d->hwirq;

	/* nothing to do here */
	if (debug_keystone_ipc)
		pr_debug("ipc irq: ack %d [%x]\n", irq, kipc->mask);
}

void keystone_ipc_irq_handler(unsigned irq, struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct irq_data *irq_data = irq_desc_get_irq_data(desc);
	struct keystone_ipc_device *kipc = irq_desc_get_handler_data(desc);
	unsigned long pending;
	int src, virq;

	if (debug_keystone_ipc)
		pr_debug("ipc irq: start irq %d\n", irq);

	chip->irq_mask(irq_data);
	if(chip->irq_ack)
		chip->irq_ack(irq_data);
	if (chip->irq_eoi)
		chip->irq_eoi(&desc->irq_data);

	pending = keystone_ipc_readl(kipc);
	keystone_ipc_writel(kipc, pending);

	if (debug_keystone_ipc)
		pr_debug("ipc irq: pending 0x%lx, mask 0x%x\n",
			pending, kipc->mask);

	pending = (pending >> BIT_OFS) & ~kipc->mask;

	if (debug_keystone_ipc)
		pr_debug("ipc irq: pending after mask 0x%lx\n",
			pending);

	for (src = 0; src < N_IPC_IRQ; src++) {
		if (BIT(src) & pending) {
			virq = irq_linear_revmap(kipc->irqd, src);
			pr_debug("ipc irq: dispatch bit %d, virq %d\n",
				 src, virq);
			generic_handle_irq(virq);
		}
	}

	chip->irq_unmask(irq_data);

	if (debug_keystone_ipc)
		pr_debug("ipc irq: end irq %d\n", irq);
}

static int keystone_ipc_irq_map(struct irq_domain *h, unsigned int virq,
				irq_hw_number_t hw)
{
	struct keystone_ipc_device *kipc = h->host_data;

	irq_set_chip_data(virq, kipc);
	irq_set_chip_and_handler(virq, &kipc->chip, handle_level_irq);
	set_irq_flags(virq, IRQF_VALID | IRQF_PROBE);
	return 0;
}

static struct irq_domain_ops keystone_ipc_irq_ops = {
	.map	= keystone_ipc_irq_map,
	.xlate	= irq_domain_xlate_twocell,
};

int __init keystone_ipc_irq_of_init(struct device_node *node,
				    struct device_node *parent)
{
	struct keystone_ipc_device *kipc;
	int i;

	kipc = kzalloc(sizeof(struct keystone_ipc_device), GFP_KERNEL);
	if (!kipc)
		return -ENOMEM;

	kipc->mask = ~0x0;
	kipc->chip.name		= "keystone-ipc-irq";
	kipc->chip.irq_ack	= keystone_ipc_ack_irq;
	kipc->chip.irq_mask	= keystone_ipc_mask_irq;
	kipc->chip.irq_unmask	= keystone_ipc_unmask_irq;

	kipc->arh = of_iomap(node, 0);
	if (!kipc->arh) {
		pr_err("ipc irq: No ack register in dt binding\n");
		return -ENODEV;
	}

	kipc->grh = of_iomap(node, 1);
	if (!kipc->grh) {
		pr_err("ipc irq: No gen register in dt binding\n");
		return -ENODEV;
	}

	kipc->irq = irq_of_parse_and_map(node, 0);

	if (kipc->irq < 0) {
		pr_err("ipc irq: no irq resource\n");
		return -EINVAL;
	}

	kipc->irqd = irq_domain_add_linear(node, N_IPC_IRQ,
					   &keystone_ipc_irq_ops, kipc);

	if (!kipc->irqd) {
		pr_err("ipc irq: IRQ domain registration failed\n");
		return -ENODEV;
	}

	/* Map the ipc hw irqs */
	for (i = 0; i < N_IPC_IRQ; i++)
		irq_create_mapping(kipc->irqd, i);

	irq_set_chained_handler(kipc->irq, keystone_ipc_irq_handler);
	irq_set_handler_data(kipc->irq, kipc);

	/* clear all source bits */
	keystone_ipc_writel(kipc, ~0x0);

	pr_info("ipc irq: irqchip registered, range %d-%d\n",
		irq_linear_revmap(kipc->irqd, 0),
		irq_linear_revmap(kipc->irqd, N_IPC_IRQ - 1));

	return 0;
}
