/*
 * Low level suspend code for AM33XX SoCs
 *
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
 * Vaibhav Bedia <vaibhav.bedia@ti.com>
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

#include <linux/linkage.h>
#include <linux/ti_emif.h>
#include <asm/memory.h>
#include <asm/assembler.h>

#include "cm33xx.h"
#include "pm33xx.h"
#include "prm33xx.h"

#define EMIF_POWER_MGMT_WAIT_SELF_REFRESH_8192_CYCLES 	0x00a0
#define EMIF_POWER_MGMT_SR_TIMER_MASK					0x00f0

#define EMIF_POWER_MGMT_SELF_REFRESH_MODE 				0x0200
#define EMIF_POWER_MGMT_SELF_REFRESH_MODE_MASK  		0x0700
#define EMIF_POWER_MGMT_DELAY_PERIOD					0x1000

#define AM33XX_CM_CLKCTRL_MODULEMODE_DISABLE			0x0003
#define AM33XX_CM_CLKCTRL_MODULEMODE_ENABLE				0x0002

	.text
	.align 3

/*
 * This routine is executed from internal RAM and expects some
 * parameters to be passed in r0 _strictly_ in following order:
 * 1) emif_addr_virt - ioremapped EMIF address
 * 2) mem_type - 2 -> DDR2, 3-> DDR3
 * 3) dram_sync_word - uncached word in SDRAM
 *
 * The code loads these values taking r0 value as reference to
 * the array in registers starting from r0, i.e emif_addr_virt
 * goes to r1, mem_type goes to r2 and and so on. These are
 * then saved into memory locations before proceeding with the
 * sleep sequence and hence registers r0, r1 etc can still be
 * used in the rest of the sleep code.
 */

ENTRY(am33xx_do_wfi)
	stmfd	sp!, {r4 - r11, lr}	@ save registers on stack

	ldm	r0, {r1-r3}		@ gather values passed

	/* Save the values passed */
	str	r1, emif_addr_virt
	str	r2, mem_type
	str	r3, dram_sync_word

	/*
	 * Flush all data from the L1 and L2 data cache before disabling
	 * SCTLR.C bit.
	 */
	ldr	r1, kernel_flush
	blx	r1

	/*
	 * Clear the SCTLR.C bit to prevent further data cache
	 * allocation. Clearing SCTLR.C would make all the data accesses
	 * strongly ordered and would not hit the cache.
	 */
	mrc	p15, 0, r0, c1, c0, 0
	bic	r0, r0, #(1 << 2)	@ Disable the C bit
	mcr	p15, 0, r0, c1, c0, 0
	isb

	/*
	 * Invalidate L1 and L2 data cache.
	 */
	ldr	r1, kernel_flush
	blx	r1

	ldr	r0, emif_addr_virt
	/* Save EMIF configuration */
	ldr	r1, [r0, #EMIF_SDRAM_CONFIG]
	str	r1, emif_sdcfg_val
	ldr	r1, [r0, #EMIF_SDRAM_REFRESH_CONTROL]
	str	r1, emif_ref_ctrl_val
	ldr	r1, [r0, #EMIF_SDRAM_TIMING_1]
	str	r1, emif_timing1_val
	ldr	r1, [r0, #EMIF_SDRAM_TIMING_2]
	str	r1, emif_timing2_val
	ldr	r1, [r0, #EMIF_SDRAM_TIMING_3]
	str	r1, emif_timing3_val
	ldr	r1, [r0, #EMIF_POWER_MANAGEMENT_CONTROL]
	str	r1, emif_pmcr_val
	ldr	r1, [r0, #EMIF_POWER_MANAGEMENT_CTRL_SHDW]
	str	r1, emif_pmcr_shdw_val
	ldr	r1, [r0, #EMIF_SDRAM_OUTPUT_IMPEDANCE_CALIBRATION_CONFIG]
	str	r1, emif_zqcfg_val
	ldr	r1, [r0, #EMIF_DDR_PHY_CTRL_1]
	str	r1, emif_rd_lat_val

	/* Put SDRAM in self-refresh */
	ldr	r1, [r0, #EMIF_POWER_MANAGEMENT_CONTROL]
	bic	r1, r1, #EMIF_POWER_MGMT_SR_TIMER_MASK
	orr	r1, r1, #EMIF_POWER_MGMT_WAIT_SELF_REFRESH_8192_CYCLES
	str	r1, [r0, #EMIF_POWER_MANAGEMENT_CONTROL]
	str	r1, [r0, #EMIF_POWER_MANAGEMENT_CTRL_SHDW]

	ldr	r1, [r0, #EMIF_POWER_MANAGEMENT_CONTROL]
	bic	r1, r1, #EMIF_POWER_MGMT_SELF_REFRESH_MODE_MASK
	orr	r1, r1, #EMIF_POWER_MGMT_SELF_REFRESH_MODE
	str	r1, [r0, #EMIF_POWER_MANAGEMENT_CONTROL]

	ldr	r1, dram_sync_word	@ a dummy access to DDR as per spec
	ldr	r2, [r1, #0]


	mov	r1, #EMIF_POWER_MGMT_DELAY_PERIOD	@ Wait for system
wait_self_refresh:					@ to enter SR
	subs	r1, r1, #1
	bne	wait_self_refresh

	/* Disable EMIF */
	ldr	r1, virt_emif_clkctrl
	ldr	r2, [r1]
	bic	r2, r2, #AM33XX_CM_CLKCTRL_MODULEMODE_DISABLE
	str	r2, [r1]

	ldr	r1, virt_emif_clkctrl
wait_emif_disable:
	ldr	r2, [r1]
	ldr	r3, module_disabled_val
	cmp	r2, r3
	bne	wait_emif_disable

	/*
	 * For the MPU WFI to be registered as an interrupt
	 * to WKUP_M3, MPU_CLKCTRL.MODULEMODE needs to be set
	 * to DISABLED
	 */
	ldr	r1, virt_mpu_clkctrl
	ldr	r2, [r1]
	bic	r2, r2, #AM33XX_CM_CLKCTRL_MODULEMODE_DISABLE
	str	r2, [r1]

	/*
	 * Execute an ISB instruction to ensure that all of the
	 * CP15 register changes have been committed.
	 */
	isb

	/*
	 * Execute a barrier instruction to ensure that all cache,
	 * TLB and branch predictor maintenance operations issued
	 * have completed.
	 */
	dsb
	dmb

	/*
	 * Execute a WFI instruction and wait until the
	 * STANDBYWFI output is asserted to indicate that the
	 * CPU is in idle and low power state. CPU can specualatively
	 * prefetch the instructions so add NOPs after WFI. Thirteen
	 * NOPs as per Cortex-A8 pipeline.
	 */
	wfi

	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop

	/* We come here in case of an abort due to a late interrupt */

	/* Set MPU_CLKCTRL.MODULEMODE back to ENABLE */
	ldr	r1, virt_mpu_clkctrl
	mov	r2, #AM33XX_CM_CLKCTRL_MODULEMODE_ENABLE
	str	r2, [r1]

	/* Re-enable EMIF */
	ldr	r1, virt_emif_clkctrl
	mov	r2, #AM33XX_CM_CLKCTRL_MODULEMODE_ENABLE
	str	r2, [r1]
wait_emif_enable:
	ldr	r3, [r1]
	cmp	r2, r3
	bne	wait_emif_enable

	/* Disable EMIF self-refresh */
	ldr	r0, emif_addr_virt
	ldr	r1, [r0, #EMIF_POWER_MANAGEMENT_CONTROL]
	bic	r1, r1, #LP_MODE_MASK
	str	r1, [r0, #EMIF_POWER_MANAGEMENT_CONTROL]
	str	r1, [r0, #EMIF_POWER_MANAGEMENT_CTRL_SHDW]

	/*
	 * A write to SDRAM CONFIG register triggers
	 * an init sequence and hence it must be done
	 * at the end for DDR2
	 */
	ldr r0, emif_addr_virt
	add r0, r0, #EMIF_SDRAM_CONFIG
	ldr r4, emif_sdcfg_val
	str r4, [r0]

	/*
	 * Set SCTLR.C bit to allow data cache allocation
	 */
	mrc	p15, 0, r0, c1, c0, 0
	orr	r0, r0, #(1 << 2)	@ Enable the C bit
	mcr	p15, 0, r0, c1, c0, 0
	isb

	/* EMIF needs some time before read/write possible */
	mov r0, #EMIF_POWER_MGMT_DELAY_PERIOD
wait_abt:
	subs   r0, r0, #1
	bne wait_abt

	/* Let the suspend code know about the abort */
	mov	r0, #1
	ldmfd	sp!, {r4 - r11, pc}	@ restore regs and return
ENDPROC(am33xx_do_wfi)

	.align
ENTRY(am33xx_resume_offset)
	.word . - am33xx_do_wfi

ENTRY(am33xx_resume_from_deep_sleep)
	/* Re-enable EMIF */
	ldr	r0, phys_emif_clkctrl
	mov	r1, #AM33XX_CM_CLKCTRL_MODULEMODE_ENABLE
	str	r1, [r0]
wait_emif_enable1:
	ldr	r2, [r0]
	cmp	r1, r2
	bne	wait_emif_enable1

	/* Config EMIF Timings */
	ldr	r0, emif_phys_addr
	ldr	r1, emif_rd_lat_val
	str	r1, [r0, #EMIF_DDR_PHY_CTRL_1]
	str	r1, [r0, #EMIF_DDR_PHY_CTRL_1_SHDW]
	ldr	r1, emif_timing1_val
	str	r1, [r0, #EMIF_SDRAM_TIMING_1]
	str	r1, [r0, #EMIF_SDRAM_TIMING_1_SHDW]
	ldr	r1, emif_timing2_val
	str	r1, [r0, #EMIF_SDRAM_TIMING_2]
	str	r1, [r0, #EMIF_SDRAM_TIMING_2_SHDW]
	ldr	r1, emif_timing3_val
	str	r1, [r0, #EMIF_SDRAM_TIMING_3]
	str	r1, [r0, #EMIF_SDRAM_TIMING_3_SHDW]
	ldr	r1, emif_ref_ctrl_val
	str	r1, [r0, #EMIF_SDRAM_REFRESH_CONTROL]
	str	r1, [r0, #EMIF_SDRAM_REFRESH_CTRL_SHDW]

	ldr	r1, emif_pmcr_shdw_val
	str	r1, [r0, #EMIF_POWER_MANAGEMENT_CTRL_SHDW]
	/*
	 * Toggle EMIF to exit refresh mode:
	 * if EMIF lost context, PWR_MGT_CTRL is currently 0, writing disable
	 *   (0x0), wont do diddly squat! so do a toggle from SR(0x2) to disable
	 *   (0x0) here.
	 * *If* EMIF did not loose context, nothing broken as we write the same
	 *   value(0x2) to reg before we write a disable (0x0).
	 */
	ldr	r1, emif_pmcr_val
	bic	r1, r1, #EMIF_POWER_MGMT_SELF_REFRESH_MODE_MASK
	orr	r1, r1, #EMIF_POWER_MGMT_SELF_REFRESH_MODE
	str	r1, [r0, #EMIF_POWER_MANAGEMENT_CONTROL]
	bic	r1, r1, #EMIF_POWER_MGMT_SELF_REFRESH_MODE_MASK
	str	r1, [r0, #EMIF_POWER_MANAGEMENT_CONTROL]

	/*
	 * Output impedence calib needed only for DDR3
	 * but since the initial state of this will be
	 * disabled for DDR2 no harm in restoring the
	 * old configuration
	 */
	ldr	r1, emif_zqcfg_val
	str	r1, [r0, #EMIF_SDRAM_OUTPUT_IMPEDANCE_CALIBRATION_CONFIG]

	/* Write to SDRAM_CONFIG only for DDR2 */
	ldr	r2, mem_type
	cmp	r2, #MEM_TYPE_DDR2
	bne	resume_to_ddr

	/*
	 * A write to SDRAM CONFIG register triggers
	 * an init sequence and hence it must be done
	 * at the end for DDR2
	 */
	ldr	r1, emif_sdcfg_val
	str	r1, [r0, #EMIF_SDRAM_CONFIG]

resume_to_ddr:
	/* EMIF needs some time before read/write possible */
	mov	r1, #EMIF_POWER_MGMT_DELAY_PERIOD
wait_resume:
	subs	r1, r1, #1
	bne	wait_resume

	/* All done.. so restore back enter into suspend configuration */
	ldr	r1, emif_pmcr_val
	str	r1, [r0, #EMIF_POWER_MANAGEMENT_CONTROL]

	/* We are back. Branch to the common CPU resume routine */
	mov	r0, #0
	ldr	pc, resume_addr
ENDPROC(am33xx_resume_from_deep_sleep)


/*
 * Local variables
 */
	.align
resume_addr:
	.word	cpu_resume - PAGE_OFFSET + 0x80000000
kernel_flush:
	.word   v7_flush_dcache_all
ddr_start:
	.word	PAGE_OFFSET
emif_phys_addr:
	.word	AM33XX_EMIF_BASE
virt_mpu_clkctrl:
	.word	AM33XX_CM_MPU_MPU_CLKCTRL
virt_emif_clkctrl:
	.word	AM33XX_CM_PER_EMIF_CLKCTRL
phys_emif_clkctrl:
	.word	(AM33XX_CM_BASE + AM33XX_CM_PER_MOD + \
		AM33XX_CM_PER_EMIF_CLKCTRL_OFFSET)
module_disabled_val:
	.word	0x30000

/* DDR related defines */
dram_sync_word:
	.word	0xDEADBEEF
mem_type:
	.word	0xDEADBEEF
emif_addr_virt:
	.word	0xDEADBEEF
emif_rd_lat_val:
	.word	0xDEADBEEF
emif_timing1_val:
	.word	0xDEADBEEF
emif_timing2_val:
	.word	0xDEADBEEF
emif_timing3_val:
	.word	0xDEADBEEF
emif_sdcfg_val:
	.word	0xDEADBEEF
emif_ref_ctrl_val:
	.word	0xDEADBEEF
emif_zqcfg_val:
	.word	0xDEADBEEF
emif_pmcr_val:
	.word	0xDEADBEEF
emif_pmcr_shdw_val:
	.word	0xDEADBEEF

	.align 3
ENTRY(am33xx_do_wfi_sz)
	.word	. - am33xx_do_wfi
