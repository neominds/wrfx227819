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
#include <linux/spinlock.h>
#include <linux/ioport.h>

#include <mach/io_map.h>
#include <mach/reg_utils.h>
#include <mach/memory.h>

#include <asm/pgtable.h>

#include "gpio.h"
#include "gpio_cfg.h"



#if (defined(CONFIG_MACH_NS) || defined(CONFIG_MACH_NSP))
static struct iproc_gpio_cfg iproc_gpiocfg = {
    .get_pull = iproc_gpio_getpull_updown,
    .set_pull = iproc_gpio_setpull_updown,
    .get_config	= iproc_gpio_get_config,
    .set_config	= iproc_gpio_set_config,    
};

#endif /*CONFIG_MACH_NS || CONFIG_MACH_NSP */

static struct resource iproc_gpio_cca_config_resource[] = {
    [0] = {
		.start	= IPROC_CCA_BASE,
		.end	= IPROC_CCA_BASE + IPROC_GPIO_REG_SIZE - 1,
		.flags	= IORESOURCE_MEM,
		.name   = "intr",
    },
#if (defined(CONFIG_MACH_NS) || defined(CONFIG_MACH_NSP))
    [1] = {
		.start	= IPROC_DMU_BASE,
		.end	= IPROC_DMU_BASE + 0x200 - 1,
		.flags	= IORESOURCE_MEM,
		.name   = "dmu",
    },
#else
    [1] = {.name = "",},
#endif
};

static struct resource iproc_gpio_resources[] = {
	[0] = {
		.start	= IPROC_GPIO_CCA_BASE,
		.end	= IPROC_GPIO_CCA_BASE + IPROC_GPIO_REG_SIZE - 1,
		.flags	= IORESOURCE_MEM,
		.child = iproc_gpio_cca_config_resource,
	},
	[1] = {
		.start	= IPROC_GPIO_CCB_BASE,
		.end	= IPROC_GPIO_CCB_BASE + IPROC_GPIO_REG_SIZE -1,
		.flags	= IORESOURCE_MEM,
	}
};
#if defined(CONFIG_MACH_NS) || defined(CONFIG_MACH_NSP)

struct iproc_gpio_chip iproc_gpios_config[] = {
    [0] = {
        .id   = IPROC_GPIO_CCA_ID,
        .config = &iproc_gpiocfg,
        .chip   = {
            .base           = 0,
            .label          = "GPIOA",
            .ngpio          = 24,
        },
        .irq_base = IPROC_GPIO_CCA_IRQ_BASE,
        .resource = &iproc_gpio_resources[0],
        .irq = IPROC_GPIO_CCA_INT,
    },
};
/* CONFIG_MACH_NS */
#elif defined(CONFIG_MACH_HX4) || defined(CONFIG_MACH_KT2) 
/* 
 * Chip level GPIO 0-3 from CMICD, 
 * GPIO 4-11 from ChipcommonA 
 * Hence the base is 4 and the number is 8.
 */
struct iproc_gpio_chip iproc_gpios_config[] = {
    [0] = {
        .id   = IPROC_GPIO_CCA_ID,
        .chip   = {
            .base           = 4,
            .label          = "GPIOA",
            .ngpio          = 8,
        },
        .irq_base = IPROC_GPIO_CCA_IRQ_BASE,
        .resource = &iproc_gpio_resources[0],
        .irq = IPROC_GPIO_CCA_INT,
    },
};
#elif defined(CONFIG_MACH_HR2)  
/* 
 * Chip level GPIO 0-3 from CMICD, 
 * GPIO 4-15 are from ChipcommonA
 * where GPIO 8-15 are shared with MII or LED depends on strap pin
 * Hence the base is 4 and the number is 12.
 */
struct iproc_gpio_chip iproc_gpios_config[] = {
    [0] = {
        .id   = IPROC_GPIO_CCA_ID,
        .chip   = {
            .base           = 4,
            .label          = "GPIOA",
            .ngpio          = 12,
        },
        .irq_base = IPROC_GPIO_CCA_IRQ_BASE,
        .resource = &iproc_gpio_resources[0],
        .irq = IPROC_GPIO_CCA_INT,
    },
};
#else
struct iproc_gpio_chip iproc_gpios_config[] = {
    [0] = {
        .id   = IPROC_GPIO_CCA_ID,
        .chip   = {
            .base           = 0,
            .label          = "GPIOA",
            .ngpio          = 32,
        },
        .irq_base = IPROC_GPIO_CCA_IRQ_BASE,
        .resource = &iproc_gpio_resources[0],
        .irq = IPROC_GPIO_CCA_INT,
    },
    [1] = {
        .id   = IPROC_GPIO_CCB_ID,
        .chip   = {
            .base           = -EINVAL,
            .label          = "GPIOB",
            .ngpio          = 4,
        },
        .irq_base = IPROC_GPIO_CCB_IRQ_BASE,
        .resource = &iproc_gpio_resources[1],
        .irq = IPROC_GPIO_CCB_INT,
    },
};
#endif 

int iproc_gpiolib_init(void)
{
    struct iproc_gpio_chip *chip = iproc_gpios_config;
    int gpn;
    int temp_base;

#if defined(CONFIG_MACH_NS)
    /* bcm53012 support 24 gpios; bcm53010/53011 support 16 gpios */
    if ((__REG32(IPROC_IDM_REGISTER_VA + 0xd500) & 0xc) != 0x0) {
        iproc_gpios_config[0].chip.ngpio = 16;
    }
#endif
#if defined(CONFIG_MACH_NSP)
        /* bcm53025 support 32 gpios; bcm53022/53023 support 24 gpios */
        reg32_write((volatile uint32_t *)(IPROC_PCIE_AXIB0_REG_VA + PAXB_0_CONFIG_IND_ADDR_BASE), 0);

/*
	the mechanism to get the chip number does not work, always reads 22K.
	OTP must be programmed and then need to look at OTP
	for now assume 25K chip

        if ((__REG32(IPROC_PCIE_AXIB0_REG_VA + PAXB_0_CONFIG_IND_DATA_BASE) 
            & 0xffff0000) == 0x80250000) {
*/
        if (1) {
            iproc_gpios_config[0].chip.ngpio = 32;
        }
#endif


    temp_base = 0;
    for (gpn = 0; gpn < ARRAY_SIZE(iproc_gpios_config); gpn++, chip++) {
        if (gpn >= MAX_NS_GPIO){
            printk("Unavailabe to add gpiolib\n");
            return -EINVAL;
        }
            
        if (chip->chip.base == -EINVAL) {
            chip->chip.base = temp_base;            
        }

        iproc_gpiolib_add(chip);
        temp_base = chip->chip.base + chip->chip.ngpio;
	}

	return 0;
}
