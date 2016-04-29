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
 
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/version.h>

#include <mach/iproc_regs.h>


#include "gpio.h"


#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 37)
#define irq_get_chip_data get_irq_chip_data
#define irq_set_chip_data set_irq_chip_data
#define  irq_set_chip set_irq_chip
#define irq_set_handler set_irq_handler
#define status_use_accessors status
#endif


static struct iproc_gpio_chip *iproc_gpio_dev[MAX_NS_GPIO] = {};
static int dev = 0;

static unsigned int _iproc_gpio_readl(struct iproc_gpio_chip *chip, int reg)
{
    return readl(chip->ioaddr + reg);
}

static void _iproc_gpio_writel(struct iproc_gpio_chip *chip, unsigned int val, int reg)
{
	writel(val, chip->ioaddr + reg);
}


static int iproc_gpio_to_irq(struct iproc_gpio_chip *chip, unsigned int pin) {
    return (chip->irq_base + pin);
}

static int iproc_irq_to_gpio(struct iproc_gpio_chip *chip, unsigned int irq) {
    return (irq - chip->irq_base);
}
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 6, 5)
static void iproc_gpio_irq_ack(unsigned int irq)
{
#else
static void iproc_gpio_irq_ack(struct irq_data *d)
{
    unsigned int irq = d->irq;
#endif
    struct iproc_gpio_chip *ourchip = irq_get_chip_data(irq);
    int pin;

    pin = iproc_irq_to_gpio(ourchip, irq);

    if (ourchip->id == IPROC_GPIO_CCA_ID) {
        unsigned int  event_status, irq_type;

        event_status = 0;
        irq_type = irq_desc[irq].status_use_accessors & IRQ_TYPE_SENSE_MASK;
        if (irq_type & IRQ_TYPE_EDGE_BOTH) 
        {
            event_status |= (1 << pin);            
            _iproc_gpio_writel(ourchip, event_status,
                IPROC_GPIO_CCA_INT_EVENT);
        }

    }

    if (ourchip->id == IPROC_GPIO_CCB_ID) {
        unsigned int int_clear = 0;

        int_clear |= (1 << pin);
        _iproc_gpio_writel(ourchip, int_clear, IPROC_GPIO_CCB_INT_CLR);

    }
}
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 6, 5)
static void iproc_gpio_irq_unmask(unsigned int irq)
{
#else
static void iproc_gpio_irq_unmask(struct irq_data *d)
{
    unsigned int irq = d->irq;
#endif
    struct iproc_gpio_chip *ourchip = irq_get_chip_data(irq);
    int pin;
    unsigned int int_mask, irq_type;

    pin = iproc_irq_to_gpio(ourchip, irq);
    irq_type = irq_desc[irq].status_use_accessors & IRQ_TYPE_SENSE_MASK;

    if (ourchip->id == IPROC_GPIO_CCA_ID) {
        unsigned int  event_mask; 

        event_mask = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCA_INT_EVENT_MASK);
        int_mask = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCA_INT_LEVEL_MASK);

        if (irq_type & IRQ_TYPE_EDGE_BOTH) {
            event_mask |= 1 << pin;
            _iproc_gpio_writel(ourchip, event_mask, 
                IPROC_GPIO_CCA_INT_EVENT_MASK);
        } else {
            int_mask |= 1 << pin;
            _iproc_gpio_writel(ourchip, int_mask, 
                IPROC_GPIO_CCA_INT_LEVEL_MASK);
        }
    }

    if (ourchip->id == IPROC_GPIO_CCB_ID) {
        int_mask = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCB_INT_MASK);
        int_mask |= (1 << pin);
        _iproc_gpio_writel(ourchip, int_mask, IPROC_GPIO_CCB_INT_MASK);
    }

}
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 6, 5)
static void iproc_gpio_irq_mask(unsigned int irq)
{
#else
static void iproc_gpio_irq_mask(struct irq_data *d)
{
    unsigned int irq = d->irq;
#endif
    struct iproc_gpio_chip *ourchip = irq_get_chip_data(irq);
    int pin;
    unsigned int irq_type, int_mask;

    pin = iproc_irq_to_gpio(ourchip, irq);
    irq_type = irq_desc[irq].status_use_accessors & IRQ_TYPE_SENSE_MASK;

    if (ourchip->id == IPROC_GPIO_CCA_ID) {
        unsigned int  event_mask;
        
        event_mask = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCA_INT_EVENT_MASK);
        int_mask = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCA_INT_LEVEL_MASK);

        if (irq_type & IRQ_TYPE_EDGE_BOTH) {
            event_mask &= ~(1 << pin);
            _iproc_gpio_writel(ourchip, event_mask,
                IPROC_GPIO_CCA_INT_EVENT_MASK);
        } else {
            int_mask &= ~(1 << pin);
            _iproc_gpio_writel(ourchip, int_mask, 
                IPROC_GPIO_CCA_INT_LEVEL_MASK);
        }
    }

    if (ourchip->id == IPROC_GPIO_CCB_ID) {
        int_mask = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCB_INT_MASK);
        int_mask &= ~(1 << pin);
        _iproc_gpio_writel(ourchip, int_mask,IPROC_GPIO_CCB_INT_MASK);
    }


}
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 6, 5)
static int iproc_gpio_irq_set_type(unsigned int irq, unsigned int type)
{
#else
static int iproc_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
    unsigned int irq = d->irq;
#endif
    struct iproc_gpio_chip *ourchip = irq_get_chip_data(irq);
    int pin;    


    pin = iproc_irq_to_gpio(ourchip, irq);

    if (ourchip->id == IPROC_GPIO_CCA_ID) {
        unsigned int  event_pol, int_pol;

        switch (type & IRQ_TYPE_SENSE_MASK) {
        case IRQ_TYPE_EDGE_RISING:
            event_pol = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCA_INT_EDGE);
            event_pol &= ~(1 << pin);
            _iproc_gpio_writel(ourchip, event_pol, IPROC_GPIO_CCA_INT_EDGE);
            break;
        case IRQ_TYPE_EDGE_FALLING:
            event_pol = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCA_INT_EDGE);
            event_pol |= (1 << pin);
            _iproc_gpio_writel(ourchip, event_pol, IPROC_GPIO_CCA_INT_EDGE);
            break;
        case IRQ_TYPE_LEVEL_HIGH:
            int_pol = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCA_INT_LEVEL);
            int_pol &= ~(1 << pin);
            _iproc_gpio_writel(ourchip, int_pol, IPROC_GPIO_CCA_INT_LEVEL);
            break;
        case IRQ_TYPE_LEVEL_LOW:
            int_pol = _iproc_gpio_readl(ourchip,IPROC_GPIO_CCA_INT_LEVEL);
            int_pol |= (1 << pin);
            _iproc_gpio_writel(ourchip, int_pol, IPROC_GPIO_CCA_INT_LEVEL);
            break;
        default:
            printk(KERN_ERR "unsupport irq type !\n");
            return -EINVAL;
        }
    }

    if (ourchip->id == IPROC_GPIO_CCB_ID) {
        unsigned int  int_type, int_de, int_edge;
        int_type = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCB_INT_TYPE);
        int_edge = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCB_INT_EDGE);
        switch (type) {
            case IRQ_TYPE_EDGE_BOTH:
                int_type &= ~(1 << pin); 
                int_de = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCB_INT_DE);
                int_de |= (1 << pin); 
                _iproc_gpio_writel(ourchip, int_de, IPROC_GPIO_CCB_INT_DE);
                break;
            case IRQ_TYPE_EDGE_RISING:
                int_type &= ~(1 << pin); 
                int_edge |= (1 << pin); 
                
                int_de = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCB_INT_DE);
                int_de  &= ~(1 << pin); 
                _iproc_gpio_writel(ourchip, int_de, IPROC_GPIO_CCB_INT_DE);
                break;
            case IRQ_TYPE_EDGE_FALLING:
                int_type &= ~(1 << pin);
                int_edge &= ~(1 << pin);
                
                int_de = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCB_INT_DE);
                int_de  &= ~(1 << pin); 
                _iproc_gpio_writel(ourchip, int_de, IPROC_GPIO_CCB_INT_DE);
                break;
            case IRQ_TYPE_LEVEL_HIGH:
                int_type |= (1 << pin); 
                int_edge |= (1 << pin); 
                break;
            case IRQ_TYPE_LEVEL_LOW:
                int_type |= (1 << pin); 
                int_edge &= ~(1 << pin); 
                break;
            default:
                printk(KERN_ERR "unsupport irq type !\n");
                return -EINVAL;
        }
        _iproc_gpio_writel(ourchip, int_type, IPROC_GPIO_CCB_INT_TYPE);
        _iproc_gpio_writel(ourchip, int_edge, IPROC_GPIO_CCB_INT_EDGE);
    }

    return 0;
}


static irqreturn_t 
iproc_gpio_irq_handler_cca(int irq, void *dev)

{
    unsigned int  val, irq_type;
    unsigned int  int_mask, int_pol, in;
    unsigned int  event_mask, event, event_pol, tmp = 0;
    int iter, g_irq;
    struct iproc_gpio_chip *ourchip = dev;


    val = readl(ourchip->intr_ioaddr + IPROC_CCA_INT_STS);
    
    if (val & IPROC_CCA_INT_F_GPIOINT) {
        int_mask = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCA_INT_LEVEL_MASK);
        int_pol = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCA_INT_LEVEL);
        in = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCA_DIN);
        event_mask = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCA_INT_EVENT_MASK);
        event = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCA_INT_EVENT);
        event_pol = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCA_INT_EDGE);

        for (iter = 0; iter < ourchip->chip.ngpio; iter ++) {
            g_irq = iproc_gpio_to_irq(ourchip, iter);  
            irq_type = irq_desc[g_irq].status_use_accessors & IRQ_TYPE_SENSE_MASK;
            switch(irq_type) {
                case IRQ_TYPE_EDGE_RISING:
                    tmp = event_mask;
                    tmp &= event;
                    tmp &= ~event_pol;
                    if (tmp & (1 << iter)) {
                        generic_handle_irq(g_irq);
                    }
                    break;
                case IRQ_TYPE_EDGE_FALLING:
                    tmp = event_mask;
                    tmp &= event;
                    tmp &= event_pol;
                    if (tmp & (1 << iter)) {
                        generic_handle_irq(g_irq);
                    }                
                    break;
                case IRQ_TYPE_LEVEL_LOW:
                    tmp = in ^ int_pol;
                    tmp &= int_mask;
                    tmp &= int_pol;
                    if (tmp & (1 << iter)) {
                        generic_handle_irq(g_irq);
                    }                
                    break;
                case IRQ_TYPE_LEVEL_HIGH:
                    tmp = in ^ int_pol;
                    tmp &= int_mask;
                    tmp &= ~int_pol;
                    if (tmp & (1 << iter)) {
                        generic_handle_irq(g_irq);
                    }                
                    break;
                default:                        
                    break;
            }
        }
    }else {
        return IRQ_NONE;
    }

    return IRQ_HANDLED;
}


static irqreturn_t 
iproc_gpio_irq_handler_ccb(int irq, void *dev)
{
    struct iproc_gpio_chip *ourchip = dev;
    int iter;
    unsigned int  val;

    val = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCB_INT_MSTAT);
    if(!val){
        return IRQ_NONE;
    }

    for (iter = 0; iter < ourchip->chip.ngpio; iter ++) {
        if (val & (1 << iter)) {
            generic_handle_irq(iproc_gpio_to_irq(ourchip, iter));
        }
    }
    
    return IRQ_HANDLED;
}

static struct irq_chip iproc_gpio_irq_chip = {
    .name         = "IPROC-GPIO",
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 6, 5)
    .ack      = (void *) iproc_gpio_irq_ack,
    .mask     = (void *) iproc_gpio_irq_mask,
    .unmask   = (void *) iproc_gpio_irq_unmask,
    .set_type = (void *) iproc_gpio_irq_set_type,
#else
    .irq_ack      = (void *) iproc_gpio_irq_ack,
    .irq_mask     = (void *) iproc_gpio_irq_mask,
    .irq_unmask   = (void *) iproc_gpio_irq_unmask,
    .irq_set_type = (void *) iproc_gpio_irq_set_type,
#endif
};

struct iproc_gpio_chip *iproc_gpios[IPROC_GPIO_END];

static __init void iproc_gpiolib_track(struct iproc_gpio_chip *chip)
{
    unsigned int gpn;
    int i;

    gpn = chip->chip.base;
    for (i = 0; i < chip->chip.ngpio; i++, gpn++) {
        BUG_ON(gpn >= ARRAY_SIZE(iproc_gpios));
        iproc_gpios[gpn] = chip;
    }
}

static int iproc_gpiolib_input(struct gpio_chip *chip, unsigned gpio)
{
    struct iproc_gpio_chip *ourchip = to_iproc_gpio(chip);
    unsigned long flags;
    unsigned int  val;
    unsigned int  nBitMask = 1 << gpio;


    iproc_gpio_lock(ourchip, flags);

    val = _iproc_gpio_readl(ourchip, REGOFFSET_GPIO_EN);
    val &= ~nBitMask;
    _iproc_gpio_writel(ourchip, val, REGOFFSET_GPIO_EN);

    iproc_gpio_unlock(ourchip, flags);
    return 0;
}

static int iproc_gpiolib_output(struct gpio_chip *chip,
			      unsigned gpio, int value)
{
    struct iproc_gpio_chip *ourchip = to_iproc_gpio(chip);
    unsigned long flags;
    unsigned long val;
    unsigned int  nBitMask = 1 << gpio;

    iproc_gpio_lock(ourchip, flags);

    val = _iproc_gpio_readl(ourchip, REGOFFSET_GPIO_EN);
    val |= nBitMask;
    _iproc_gpio_writel(ourchip, val, REGOFFSET_GPIO_EN);

    iproc_gpio_unlock(ourchip, flags);
    return 0;
}

static void iproc_gpiolib_set(struct gpio_chip *chip,
			    unsigned gpio, int value)
{
    struct iproc_gpio_chip *ourchip = to_iproc_gpio(chip);
    unsigned long flags;
    unsigned long val;
    unsigned int  nBitMask = 1 << gpio;

    iproc_gpio_lock(ourchip, flags);


    /* determine the GPIO pin direction 
     */ 
    val = _iproc_gpio_readl(ourchip, REGOFFSET_GPIO_EN);
    val &= nBitMask;

    /* this function only applies to output pin
     */ 
    if (!val)
        return;

    val = _iproc_gpio_readl(ourchip, REGOFFSET_GPIO_DOUT);

    if ( value == 0 ){
        /* Set the pin to zero */
        val &= ~nBitMask;
    }else{
        /* Set the pin to 1 */
        val |= nBitMask;
    }    
    _iproc_gpio_writel(ourchip, val, REGOFFSET_GPIO_DOUT);

    iproc_gpio_unlock(ourchip, flags);

}


static int iproc_gpiolib_get(struct gpio_chip *chip, unsigned gpio)
{
    struct iproc_gpio_chip *ourchip = to_iproc_gpio(chip);
    unsigned long flags;
    unsigned int val, offset;
    unsigned int  nBitMask = 1 << gpio;    

    iproc_gpio_lock(ourchip, flags);
    /* determine the GPIO pin direction 
     */ 
    offset = _iproc_gpio_readl(ourchip, REGOFFSET_GPIO_EN);
    offset &= nBitMask;

    if (offset){
        val = _iproc_gpio_readl(ourchip, REGOFFSET_GPIO_DOUT);
    } else {
        val = _iproc_gpio_readl(ourchip, REGOFFSET_GPIO_DIN);    
    }
    val >>= gpio;
    val &= 1;
    iproc_gpio_unlock(ourchip, flags);

    return val;
}
static int iproc_gpiolib_to_irq(struct gpio_chip *chip,
                unsigned offset)
{
    struct iproc_gpio_chip *ourchip = to_iproc_gpio(chip);
    return iproc_gpio_to_irq(ourchip, offset);
}
void __init  iproc_gpiolib_add(struct iproc_gpio_chip *chip)
{
    struct resource *res;
    struct gpio_chip *gc = &chip->chip;
    int ret, i;    

    BUG_ON(!gc->label);
    BUG_ON(!gc->ngpio);
    
    spin_lock_init(&chip->lock);
    
    if (!gc->direction_input)
        gc->direction_input = iproc_gpiolib_input;
    if (!gc->direction_output)
        gc->direction_output = iproc_gpiolib_output;
    if (!gc->set)
        gc->set = iproc_gpiolib_set;
    if (!gc->get)
        gc->get = iproc_gpiolib_get;
    if (!gc->to_irq)
        gc->to_irq = iproc_gpiolib_to_irq;

    /* gpiochip_add() prints own failure message on error. */
    ret = gpiochip_add(gc);
    if (ret >= 0)
        iproc_gpiolib_track(chip);

    printk(KERN_INFO "iproc gpiochip add %s\n", gc->label);
    /* io remap */
    res = chip->resource;

    chip->ioaddr = ioremap_nocache(res->start, (res->end - res->start) + 1);
    printk(KERN_INFO "%s:ioaddr %p \n", gc->label, chip->ioaddr);
    chip->intr_ioaddr = NULL;
    chip->dmu_ioaddr = NULL;
    if(res->child){
        for (i=0; i< 2; i++){        
            if (!strcmp("intr", res->child[i].name)){
                chip->intr_ioaddr = 
                    ioremap_nocache(res->child[i].start, 
                    (res->child[i].end - res->child[i].start) + 1);
            }
            if (!strcmp("dmu", res->child[i].name)){
                chip->dmu_ioaddr = 
                    ioremap_nocache(res->child[i].start, 
                    (res->child[i].end - res->child[i].start) + 1);
            }
        }
        printk(KERN_INFO "%s:intr_ioaddr %p dmu_ioaddr %p\n",
            gc->label, chip->intr_ioaddr,chip->dmu_ioaddr);
    }


    if (chip->irq_base) {
        for (i = chip->irq_base; i < (chip->irq_base + gc->ngpio); i++) {
            irq_set_chip(i, &iproc_gpio_irq_chip);
            irq_set_chip_data(i,chip);
            irq_set_handler(i, handle_level_irq);
            set_irq_flags(i, IRQF_VALID);                            
    
        }        
        if (chip->id == IPROC_GPIO_CCA_ID ){
            unsigned int val;
            /* enable the GPIO in CCA interrupt mask */
            val = readl(chip->intr_ioaddr + IPROC_CCA_INT_MASK);
            val |= IPROC_CCA_INT_F_GPIOINT;
            writel(val, chip->intr_ioaddr + IPROC_CCA_INT_MASK);
            ret = request_irq(chip->irq, iproc_gpio_irq_handler_cca,
                IRQF_NO_SUSPEND|IRQF_SHARED, gc->label, chip);
            if (ret)
                printk(KERN_ERR "Unable to request IRQ%d: %d\n",
                    IPROC_GPIO_CCA_INT, ret);
        }
        if (chip->id == IPROC_GPIO_CCB_ID ){
            ret = request_irq(chip->irq, iproc_gpio_irq_handler_ccb,
                IRQF_NO_SUSPEND, gc->label, chip);
            if (ret)
                printk(KERN_ERR "Unable to request IRQ%d: %d\n",
                    IPROC_GPIO_CCB_INT, ret);
        }
    }
    iproc_gpio_dev[dev] = chip;
    dev++;
}

static int __init gpio_init(void)
{      
    iproc_gpiolib_init();
    
    return 0;
}
static void __exit gpio_exit(void)
{
    unsigned int val;
    int i=0;

    for (i = 0 ; i < MAX_NS_GPIO; i++) {        
        if(iproc_gpio_dev[i]){
            if(iproc_gpio_dev[i]->ioaddr){
                iounmap(iproc_gpio_dev[i]->ioaddr);
            }
            if(iproc_gpio_dev[i]->intr_ioaddr){
                if (iproc_gpio_dev[i]->id == IPROC_GPIO_CCA_ID ){
                  val = readl(iproc_gpio_dev[i]->intr_ioaddr + IPROC_CCA_INT_MASK);
                  val &= ~(IPROC_CCA_INT_F_GPIOINT);
                  writel(val, iproc_gpio_dev[i]->intr_ioaddr + IPROC_CCA_INT_MASK);
                }                      
                iounmap(iproc_gpio_dev[i]->intr_ioaddr);
            }
            if(iproc_gpio_dev[i]->dmu_ioaddr){
                iounmap(iproc_gpio_dev[i]->dmu_ioaddr);
            }                        
            if(iproc_gpio_dev[i]->irq_base) {
                free_irq(iproc_gpio_dev[i]->irq,iproc_gpio_dev[i]);
            }

            gpiochip_remove(&iproc_gpio_dev[i]->chip);
            iproc_gpio_dev[i] = NULL;
        }
    }
}

MODULE_DESCRIPTION("IPROC GPIO driver");
MODULE_LICENSE("GPL");

module_init(gpio_init);
module_exit(gpio_exit);
