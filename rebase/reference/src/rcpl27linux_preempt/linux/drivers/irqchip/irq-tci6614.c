/*
 * Texas Instruments TCI6614 IRQ Support
 *
 * Copyright (C) 2011 Texas Instruments
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
#include <linux/kernel.h>
#include <linux/of_irq.h>
#include <linux/irqdomain.h>
#include <linux/of_address.h>
#include <linux/irqchip/tci6614.h>

#include <asm/exception.h>

#define AINTC_IRQ_BASE		0
#define N_AINTC_IRQ		128
#define AINTC_IRQ(x)		(AINTC_IRQ_BASE + (x))

#define CP_INTD_IRQ_BASE	128
#define N_CP_INTD_IRQ		128
#define CP_INTD_IRQ(x)		(CP_INTD_IRQ_BASE + (x))

#define CP_INTC_IRQ_BASE	256
#define N_CP_INTC_IRQ		256
#define CP_INTC_IRQ(x)		(CP_INTC_IRQ_BASE + (x))

#define N_IRQ		(N_AINTC_IRQ + N_CP_INTD_IRQ + N_CP_INTC_IRQ)

/* TCI6614 specific interrupts */
#define IRQ_EMUINT		AINTC_IRQ(0)
#define IRQ_EVT1COMMTX		AINTC_IRQ(1)
#define IRQ_COMMRX		AINTC_IRQ(2)
#define IRQ_BENCH		AINTC_IRQ(3)
#define IRQ_INTERNAL		AINTC_IRQ(4)
#define IRQ_SSM_WFI_IRQ		AINTC_IRQ(5)
#define IRQ_SSM_IRQ		AINTC_IRQ(6)
#define IRQ_QM_INT_HIGH_1	CP_INTD_IRQ(3)
#define IRQ_IPC_H		CP_INTD_IRQ(1)
#define IRQ_QM_INT_HIGH_0	CP_INTD_IRQ(2)
#define IRQ_INTERCONNECT_ERROR	AINTC_IRQ(10)
#define IRQ_QM_INT_HIGH_2	CP_INTD_IRQ(4)
#define IRQ_QM_INT_HIGH_3	CP_INTD_IRQ(5)
#define IRQ_QM_INT_HIGH_4	CP_INTD_IRQ(6)
#define IRQ_QM_INT_HIGH_5	CP_INTD_IRQ(7)
#define IRQ_QM_INT_HIGH_6	CP_INTD_IRQ(8)
#define IRQ_QM_INT_HIGH_7	CP_INTD_IRQ(9)
#define IRQ_QM_INT_HIGH_8	CP_INTD_IRQ(10)
#define IRQ_QM_INT_HIGH_9	CP_INTD_IRQ(11)
#define IRQ_QM_INT_HIGH_10	CP_INTD_IRQ(12)
#define IRQ_QM_INT_HIGH_11	CP_INTD_IRQ(13)
#define IRQ_QM_INT_HIGH_12	CP_INTD_IRQ(14)
#define IRQ_QM_INT_HIGH_13	CP_INTD_IRQ(15)
#define IRQ_QM_INT_HIGH_14	CP_INTD_IRQ(16)
#define IRQ_QM_INT_HIGH_15	CP_INTD_IRQ(17)
#define IRQ_QM_INT_HIGH_16	CP_INTD_IRQ(18)
#define IRQ_QM_INT_HIGH_17	CP_INTD_IRQ(19)
#define IRQ_QM_INT_HIGH_18	CP_INTD_IRQ(20)
#define IRQ_QM_INT_HIGH_19	CP_INTD_IRQ(21)
#define IRQ_QM_INT_HIGH_20	CP_INTD_IRQ(22)
#define IRQ_QM_INT_HIGH_21	CP_INTD_IRQ(23)
#define IRQ_QM_INT_HIGH_22	CP_INTD_IRQ(24)
#define IRQ_QM_INT_HIGH_23	CP_INTD_IRQ(25)
#define IRQ_QM_INT_HIGH_24	CP_INTD_IRQ(26)
#define IRQ_QM_INT_HIGH_25	CP_INTD_IRQ(27)
#define IRQ_QM_INT_HIGH_26	CP_INTD_IRQ(28)
#define IRQ_QM_INT_HIGH_27	CP_INTD_IRQ(29)
#define IRQ_QM_INT_HIGH_28	CP_INTD_IRQ(30)
#define IRQ_QM_INT_HIGH_29	CP_INTD_IRQ(31)
#define IRQ_QM_INT_HIGH_30	CP_INTD_IRQ(32)
#define IRQ_QM_INT_HIGH_31	CP_INTD_IRQ(33)
#define IRQ_QM_INT_TXQ_PEND_650	AINTC_IRQ(41)
#define IRQ_QM_INT_TXQ_PEND_651	AINTC_IRQ(42)
#define IRQ_QM_INT_TXQ_PEND_652	AINTC_IRQ(43)
#define IRQ_QM_INT_TXQ_PEND_653	AINTC_IRQ(44)
#define IRQ_QM_INT_TXQ_PEND_654	AINTC_IRQ(45)
#define IRQ_QM_INT_TXQ_PEND_655	AINTC_IRQ(46)
#define IRQ_QM_INT_TXQ_PEND_656	AINTC_IRQ(47)
#define IRQ_QM_INT_TXQ_PEND_657	AINTC_IRQ(48)
#define IRQ_QM_INT_TXQ_PEND_670	AINTC_IRQ(49)
#define IRQ_QM_INT_TXQ_PEND_671	AINTC_IRQ(50)
#define IRQ_VUSR_INT		CP_INTD_IRQ(44)
#define IRQ_SEMERR7		CP_INTD_IRQ(45)
#define IRQ_SEMINT7		CP_INTD_IRQ(46)
#define IRQ_SRIO_INTDST20	CP_INTD_IRQ(47)
#define IRQ_SRIO_INTDST21	CP_INTD_IRQ(48)
#define IRQ_SRIO_INTDST22	CP_INTD_IRQ(49)
#define IRQ_SRIO_INTDST23	CP_INTD_IRQ(50)
#define IRQ_TINT4L		CP_INTD_IRQ(51)
#define IRQ_TINT4H		CP_INTD_IRQ(52)
#define IRQ_TINT5L		CP_INTD_IRQ(53)
#define IRQ_TINT5H		CP_INTD_IRQ(54)
#define IRQ_TINT6L		CP_INTD_IRQ(55)
#define IRQ_TINT6H		CP_INTD_IRQ(56)
#define IRQ_TINT7L		CP_INTD_IRQ(57)
#define IRQ_TINT7H		CP_INTD_IRQ(58)
#define IRQ_PCIE_ERR_INT	CP_INTD_IRQ(59)
#define IRQ_PCIE_PM_INT		CP_INTD_IRQ(60)
#define IRQ_PCIE_LEGACY_INTA	CP_INTD_IRQ(61)
#define IRQ_PCIE_LEGACY_INTB	CP_INTD_IRQ(62)
#define IRQ_PCIE_LEGACY_INTC	CP_INTD_IRQ(63)
#define IRQ_PCIE_LEGACY_INTD	CP_INTD_IRQ(64)
#define IRQ_PCIE_MSI_INT4	CP_INTD_IRQ(65)
#define IRQ_PCIE_MSI_INT5	CP_INTD_IRQ(66)
#define IRQ_PCIE_MSI_INT6	CP_INTD_IRQ(67)
#define IRQ_PCIE_MSI_INT7	CP_INTD_IRQ(68)
#define IRQ_TINT8L		CP_INTD_IRQ(69)
#define IRQ_TINT8H		CP_INTD_IRQ(70)
#define IRQ_TINT9L		CP_INTD_IRQ(71)
#define IRQ_TINT9H		CP_INTD_IRQ(72)
#define IRQ_TINT10L		CP_INTD_IRQ(73)
#define IRQ_TINT10H		CP_INTD_IRQ(74)
#define IRQ_TINT11L		CP_INTD_IRQ(75)
#define IRQ_TINT11H		CP_INTD_IRQ(76)
#define IRQ_TCP3D_A_REVT0	CP_INTD_IRQ(77)
#define IRQ_TCP3D_A_REVT1	CP_INTD_IRQ(78)
#define IRQ_TCP3D_B_REVT0	CP_INTD_IRQ(79)
#define IRQ_TCP3D_B_REVT1	CP_INTD_IRQ(80)
#define IRQ_CPU_3_1_TPCC_INT2	CP_INTD_IRQ(81)
#define IRQ_CPU_3_1_TPCC_INT6	CP_INTD_IRQ(82)
#define IRQ_CPU_3_2_TPCC_INT2	CP_INTD_IRQ(83)
#define IRQ_CPU_3_2_TPCC_INT6	CP_INTD_IRQ(84)
#define IRQ_CPU_2_TPCC_INT2	CP_INTD_IRQ(85)
#define IRQ_CPU_2_TPCC_INT6	CP_INTD_IRQ(86)
#define IRQ_WATCH_DOG_NMI	CP_INTD_IRQ(87)
#define IRQ_INTC3_OUT0		AINTC_IRQ(94)
#define IRQ_INTC3_OUT1		AINTC_IRQ(95)
#define IRQ_INTC3_OUT2		AINTC_IRQ(96)
#define IRQ_INTC3_OUT3		AINTC_IRQ(97)
#define IRQ_INTC3_OUT4		AINTC_IRQ(98)
#define IRQ_INTC3_OUT5		AINTC_IRQ(99)
#define IRQ_INTC3_OUT6		AINTC_IRQ(100)
#define IRQ_INTC3_OUT7		AINTC_IRQ(101)
#define IRQ_INTC3_OUT8		AINTC_IRQ(102)
#define IRQ_INTC3_OUT9		AINTC_IRQ(103)
#define IRQ_INTC3_OUT10		AINTC_IRQ(104)
#define IRQ_INTC3_OUT11		AINTC_IRQ(105)
#define IRQ_INTC3_OUT12		AINTC_IRQ(106)
#define IRQ_INTC3_OUT13		AINTC_IRQ(107)
#define IRQ_INTC3_OUT14		AINTC_IRQ(108)
#define IRQ_INTC3_OUT15		AINTC_IRQ(109)
#define IRQ_INTC3_OUT16		AINTC_IRQ(110)
#define IRQ_INTC3_OUT17		AINTC_IRQ(111)
#define IRQ_INTC3_OUT18		AINTC_IRQ(112)
#define IRQ_INTC3_OUT19		AINTC_IRQ(113)
#define IRQ_INTC3_OUT20		AINTC_IRQ(114)
#define IRQ_INTC3_OUT21		AINTC_IRQ(115)
#define IRQ_INTC3_OUT22		AINTC_IRQ(116)
#define IRQ_INTC3_OUT23		AINTC_IRQ(117)
#define IRQ_INTC3_OUT24		AINTC_IRQ(118)
#define IRQ_INTC3_OUT25		AINTC_IRQ(119)
#define IRQ_INTC3_OUT26		AINTC_IRQ(120)
#define IRQ_INTC3_OUT27		AINTC_IRQ(121)
#define IRQ_INTC3_OUT28		AINTC_IRQ(122)
#define IRQ_INTC3_OUT29		AINTC_IRQ(123)
#define IRQ_INTC3_OUT30		AINTC_IRQ(124)
#define IRQ_INTC3_OUT31		AINTC_IRQ(125)
#define IRQ_INTC3_OUT32		AINTC_IRQ(126)
#define IRQ_SEMERR4		CP_INTC_IRQ(0)
#define IRQ_SEMERR5		CP_INTC_IRQ(1)
#define IRQ_SEMERR6		CP_INTC_IRQ(2)
#define IRQ_SEMINT4		CP_INTC_IRQ(3)
#define IRQ_SEMINT5		CP_INTC_IRQ(4)
#define IRQ_SEMINT6		CP_INTC_IRQ(5)
#define IRQ_CPU_3_1_TPCC_ERR	CP_INTC_IRQ(8)
#define IRQ_CPU_3_1_TPCC_MPINT	CP_INTC_IRQ(9)
#define IRQ_CPU_3_1_TPTC_ERR0	CP_INTC_IRQ(10)
#define IRQ_CPU_3_1_TPTC_ERR1	CP_INTC_IRQ(11)
#define IRQ_CPU_3_1_TPTC_ERR2	CP_INTC_IRQ(12)
#define IRQ_CPU_3_1_TPTC_ERR3	CP_INTC_IRQ(13)
#define IRQ_CPU_3_1_TPCC_GINT	CP_INTC_IRQ(14)
#define IRQ_CPU_3_1_TPCCINT3	CP_INTC_IRQ(15)
#define IRQ_CPU_3_1_TPCCINT7	CP_INTC_IRQ(16)
#define IRQ_CPU_3_2_TPCC_ERR	CP_INTC_IRQ(17)
#define IRQ_CPU_3_2_TPCC_MPINT	CP_INTC_IRQ(18)
#define IRQ_CPU_3_2_TPTC_ERR0	CP_INTC_IRQ(19)
#define IRQ_CPU_3_2_TPTC_ERR1	CP_INTC_IRQ(20)
#define IRQ_CPU_3_2_TPTC_ERR2	CP_INTC_IRQ(21)
#define IRQ_CPU_3_2_TPTC_ERR3	CP_INTC_IRQ(22)
#define IRQ_CPU_3_2_TPCC_GINT	CP_INTC_IRQ(23)
#define IRQ_CPU_3_2_TPCCINT3	CP_INTC_IRQ(24)
#define IRQ_CPU_3_2_TPCCINT7	CP_INTC_IRQ(25)
#define IRQ_CPU_2_TPCC_ERR	CP_INTC_IRQ(26)
#define IRQ_CPU_2_TPCC_MPINT	CP_INTC_IRQ(27)
#define IRQ_CPU_2_TPTC_ERR0	CP_INTC_IRQ(28)
#define IRQ_CPU_2_TPTC_ERR1	CP_INTC_IRQ(29)
#define IRQ_CPU_2_TPCC_GINT	CP_INTC_IRQ(30)
#define IRQ_CPU_2_TPCCINT3	CP_INTC_IRQ(31)
#define IRQ_CPU_2_TPCCINT7	CP_INTC_IRQ(32)
#define IRQ_GPINT0		CP_INTC_IRQ(33)
#define IRQ_GPINT1		CP_INTC_IRQ(34)
#define IRQ_GPINT2		CP_INTC_IRQ(35)
#define IRQ_GPINT3		CP_INTC_IRQ(36)
#define IRQ_GPINT4		CP_INTC_IRQ(37)
#define IRQ_GPINT5		CP_INTC_IRQ(38)
#define IRQ_GPINT6		CP_INTC_IRQ(39)
#define IRQ_GPINT7		CP_INTC_IRQ(40)
#define IRQ_GPINT8		CP_INTC_IRQ(41)
#define IRQ_GPINT9		CP_INTC_IRQ(42)
#define IRQ_GPINT10		CP_INTC_IRQ(43)
#define IRQ_GPINT11		CP_INTC_IRQ(44)
#define IRQ_GPINT12		CP_INTC_IRQ(45)
#define IRQ_GPINT13		CP_INTC_IRQ(46)
#define IRQ_GPINT14		CP_INTC_IRQ(47)
#define IRQ_GPINT15		CP_INTC_IRQ(48)
#define IRQ_GPINT16		CP_INTC_IRQ(49)
#define IRQ_GPINT17		CP_INTC_IRQ(50)
#define IRQ_GPINT18		CP_INTC_IRQ(51)
#define IRQ_GPINT19		CP_INTC_IRQ(52)
#define IRQ_GPINT20		CP_INTC_IRQ(53)
#define IRQ_GPINT21		CP_INTC_IRQ(54)
#define IRQ_GPINT22		CP_INTC_IRQ(55)
#define IRQ_GPINT23		CP_INTC_IRQ(56)
#define IRQ_GPINT24		CP_INTC_IRQ(57)
#define IRQ_GPINT25		CP_INTC_IRQ(58)
#define IRQ_GPINT26		CP_INTC_IRQ(59)
#define IRQ_GPINT27		CP_INTC_IRQ(60)
#define IRQ_GPINT28		CP_INTC_IRQ(61)
#define IRQ_GPINT29		CP_INTC_IRQ(62)
#define IRQ_GPINT30		CP_INTC_IRQ(63)
#define IRQ_GPINT31		CP_INTC_IRQ(64)
#define IRQ_TETBHFULLINT2	CP_INTC_IRQ(65)
#define IRQ_TETBFULLINT2	CP_INTC_IRQ(66)
#define IRQ_TETBACQINT2		CP_INTC_IRQ(67)
#define IRQ_TETBOVFLINT2	CP_INTC_IRQ(68)
#define IRQ_TETBUNFLINT2	CP_INTC_IRQ(69)
#define IRQ_TETBHFULLINT3	CP_INTC_IRQ(70)
#define IRQ_TETBFULLINT3	CP_INTC_IRQ(71)
#define IRQ_TETBACQINT3		CP_INTC_IRQ(72)
#define IRQ_TETBOVFLINT3	CP_INTC_IRQ(73)
#define IRQ_TETBUNFLINT3	CP_INTC_IRQ(74)
#define IRQ_INTDST0		CP_INTC_IRQ(75)
#define IRQ_INTDST1		CP_INTC_IRQ(76)
#define IRQ_INTDST2		CP_INTC_IRQ(77)
#define IRQ_INTDST3		CP_INTC_IRQ(78)
#define IRQ_INTDST4		CP_INTC_IRQ(79)
#define IRQ_INTDST5		CP_INTC_IRQ(80)
#define IRQ_INTDST6		CP_INTC_IRQ(81)
#define IRQ_SPIINT0		CP_INTC_IRQ(82)
#define IRQ_SPIINT1		CP_INTC_IRQ(83)
#define IRQ_SPIXEVT		CP_INTC_IRQ(84)
#define IRQ_SPIREVT		CP_INTC_IRQ(85)
#define IRQ_I2CINT		CP_INTC_IRQ(86)
#define IRQ_I2CREVT		CP_INTC_IRQ(87)
#define IRQ_I2CXEVT		CP_INTC_IRQ(88)
#define IRQ_KEYMGR		CP_INTC_IRQ(89)
#define IRQ_SECCTL		CP_INTC_IRQ(90)
#define IRQ_TETBHFULLINT	CP_INTC_IRQ(91)
#define IRQ_TETBFULLINT		CP_INTC_IRQ(92)
#define IRQ_TETBACQINT		CP_INTC_IRQ(93)
#define IRQ_TETBOVFLINT		CP_INTC_IRQ(94)
#define IRQ_TETBUNFLINT		CP_INTC_IRQ(95)
#define IRQ_TETBHFULLINT0	CP_INTC_IRQ(96)
#define IRQ_TETBFULLINT0	CP_INTC_IRQ(97)
#define IRQ_TETBACQINT0		CP_INTC_IRQ(98)
#define IRQ_TETBOVFLINT0	CP_INTC_IRQ(99)
#define IRQ_TETBUNFLINT0	CP_INTC_IRQ(100)
#define IRQ_TETBHFULLINT1	CP_INTC_IRQ(101)
#define IRQ_TETBFULLINT1	CP_INTC_IRQ(102)
#define IRQ_TETBACQINT1		CP_INTC_IRQ(103)
#define IRQ_TETBOVFLINT1	CP_INTC_IRQ(104)
#define IRQ_TETBUNFLINT1	CP_INTC_IRQ(105)
#define IRQ_DFT_PBIST_CPU_INT	CP_INTC_IRQ(106)
#define IRQ_QM_INT_LOW_0	CP_INTC_IRQ(107)
#define IRQ_QM_INT_LOW_1	CP_INTC_IRQ(108)
#define IRQ_QM_INT_LOW_2	CP_INTC_IRQ(109)
#define IRQ_QM_INT_LOW_3	CP_INTC_IRQ(110)
#define IRQ_QM_INT_LOW_4	CP_INTC_IRQ(111)
#define IRQ_QM_INT_LOW_5	CP_INTC_IRQ(112)
#define IRQ_QM_INT_LOW_6	CP_INTC_IRQ(113)
#define IRQ_QM_INT_LOW_7	CP_INTC_IRQ(114)
#define IRQ_QM_INT_LOW_8	CP_INTC_IRQ(115)
#define IRQ_QM_INT_LOW_9	CP_INTC_IRQ(116)
#define IRQ_QM_INT_LOW_10	CP_INTC_IRQ(117)
#define IRQ_QM_INT_LOW_11	CP_INTC_IRQ(118)
#define IRQ_QM_INT_LOW_12	CP_INTC_IRQ(119)
#define IRQ_QM_INT_LOW_13	CP_INTC_IRQ(120)
#define IRQ_QM_INT_LOW_14	CP_INTC_IRQ(121)
#define IRQ_QM_INT_LOW_15	CP_INTC_IRQ(122)
#define IRQ_QM_INT_CDMA_0	CP_INTC_IRQ(123)
#define IRQ_QM_INT_CDMA_1	CP_INTC_IRQ(124)
#define IRQ_MDIO_LINK_INTR0	CP_INTC_IRQ(125)
#define IRQ_MDIO_LINK_INTR1	CP_INTC_IRQ(126)
#define IRQ_MDIO_USER_INTR0	CP_INTC_IRQ(127)
#define IRQ_MDIO_USER_INTR1	CP_INTC_IRQ(128)
#define IRQ_PASS_MISC		CP_INTC_IRQ(129)
#define IRQ_PASS_INT_CDMA_0	CP_INTC_IRQ(130)
#define IRQ_CP_TRACER_CORE_0	CP_INTC_IRQ(131)
#define IRQ_CP_TRACER_CORE_1	CP_INTC_IRQ(132)
#define IRQ_CP_TRACER_DDR	CP_INTC_IRQ(133)
#define IRQ_CP_TRACER_MSMC_0	CP_INTC_IRQ(134)
#define IRQ_CP_TRACER_MSMC_1	CP_INTC_IRQ(135)
#define IRQ_CP_TRACER_MSMC_2	CP_INTC_IRQ(136)
#define IRQ_CP_TRACER_MSMC_3	CP_INTC_IRQ(137)
#define IRQ_CP_TRACER_CFG	CP_INTC_IRQ(138)
#define IRQ_CP_TRACER_QM_SS_CFG	CP_INTC_IRQ(139)
#define IRQ_CP_TRACER_QM_SS_DMA	CP_INTC_IRQ(140)
#define IRQ_CP_TRACER_SEM	CP_INTC_IRQ(141)
#define IRQ_CP_TRACER_DDR_2_APP	CP_INTC_IRQ(142)
#define IRQ_CP_TRACER_RAC	CP_INTC_IRQ(143)
#define IRQ_CP_TRACER_RAC_FE	CP_INTC_IRQ(144)
#define IRQ_CP_TRACER_TAC	CP_INTC_IRQ(145)
#define IRQ_PSC_ALLINT		CP_INTC_IRQ(146)
#define IRQ_BOOTCFG_ERR		CP_INTC_IRQ(147)
#define IRQ_BOOTCFG_PROT	CP_INTC_IRQ(148)
#define IRQ_MPU7_ADDR_ERR_INT	CP_INTC_IRQ(149)
#define IRQ_MPU7_PROT_ERR_INT	CP_INTC_IRQ(150)
#define IRQ_MPU0_ADDR_ERR_INT	CP_INTC_IRQ(151)
#define IRQ_MPU0_PROT_ERR_INT	CP_INTC_IRQ(152)
#define IRQ_MPU1_ADDR_ERR_INT	CP_INTC_IRQ(153)
#define IRQ_MPU1_PROT_ERR_INT	CP_INTC_IRQ(154)
#define IRQ_MPU2_ADDR_ERR_INT	CP_INTC_IRQ(155)
#define IRQ_MPU2_PROT_ERR_INT	CP_INTC_IRQ(156)
#define IRQ_MPU3_ADDR_ERR_INT	CP_INTC_IRQ(157)
#define IRQ_MPU3_PROT_ERR_INT	CP_INTC_IRQ(158)
#define IRQ_MPU4_ADDR_ERR_INT	CP_INTC_IRQ(159)
#define IRQ_MPU4_PROT_ERR_INT	CP_INTC_IRQ(160)
#define IRQ_MPU5_ADDR_ERR_INT	CP_INTC_IRQ(161)
#define IRQ_MPU5_PROT_ERR_INT	CP_INTC_IRQ(162)
#define IRQ_MPU6_ADDR_ERR_INT	CP_INTC_IRQ(163)
#define IRQ_MPU6_PROT_ERR_INT	CP_INTC_IRQ(164)
#define IRQ_MSMC_DEDC_CERROR	CP_INTC_IRQ(165)
#define IRQ_MSMC_DEDC_NC_ERROR	CP_INTC_IRQ(166)
#define IRQ_MSMC_SCRUB_NC_ERROR	CP_INTC_IRQ(167)
#define IRQ_MSMC_SCRUB_CERROR	CP_INTC_IRQ(168)
#define IRQ_MSMC_MPF_ERROR0	CP_INTC_IRQ(169)
#define IRQ_MSMC_MPF_ERROR1	CP_INTC_IRQ(170)
#define IRQ_MSMC_MPF_ERROR2	CP_INTC_IRQ(171)
#define IRQ_MSMC_MPF_ERROR3	CP_INTC_IRQ(172)
#define IRQ_MSMC_MPF_ERROR4	CP_INTC_IRQ(173)
#define IRQ_MSMC_MPF_ERROR5	CP_INTC_IRQ(174)
#define IRQ_MSMC_MPF_ERROR6	CP_INTC_IRQ(175)
#define IRQ_MSMC_MPF_ERROR7	CP_INTC_IRQ(176)
#define IRQ_MSMC_MPF_ERROR8	CP_INTC_IRQ(177)
#define IRQ_MSMC_MPF_ERROR9	CP_INTC_IRQ(178)
#define IRQ_MSMC_MPF_ERROR10	CP_INTC_IRQ(179)
#define IRQ_MSMC_MPF_ERROR11	CP_INTC_IRQ(180)
#define IRQ_MSMC_MPF_ERROR12	CP_INTC_IRQ(181)
#define IRQ_MSMC_MPF_ERROR13	CP_INTC_IRQ(182)
#define IRQ_MSMC_MPF_ERROR14	CP_INTC_IRQ(183)
#define IRQ_MSMC_MPF_ERROR15	CP_INTC_IRQ(184)
#define IRQ_DDR3_ERR		CP_INTC_IRQ(185)
#define IRQ_EASYNCERR		CP_INTC_IRQ(186)
#define IRQ_TCP3D_A_INTD	CP_INTC_IRQ(189)
#define IRQ_CP_TRACER_CORE_2	CP_INTC_IRQ(191)
#define IRQ_UARTINT0		CP_INTC_IRQ(192)
#define IRQ_URXEVT0		CP_INTC_IRQ(193)
#define IRQ_UTXEVT0		CP_INTC_IRQ(194)
#define IRQ_UARTINT1		CP_INTC_IRQ(195)
#define IRQ_URXEVT1		CP_INTC_IRQ(196)
#define IRQ_UTXEVT1		CP_INTC_IRQ(197)
#define IRQ_SR_INTRREQ0		CP_INTC_IRQ(198)
#define IRQ_SR_INTRREQ		CP_INTC_IRQ(199)
#define IRQ_SR_INTRREQ2		CP_INTC_IRQ(200)
#define IRQ_SR_INTRREQ3		CP_INTC_IRQ(201)
#define IRQ_VPNOSMPSACK		CP_INTC_IRQ(202)
#define IRQ_VPEQVALUE		CP_INTC_IRQ(203)
#define IRQ_VPMAXVDD		CP_INTC_IRQ(204)
#define IRQ_VPMINVDD		CP_INTC_IRQ(205)
#define IRQ_VPINIDLE		CP_INTC_IRQ(206)
#define IRQ_VPOPPCHANGEDONE	CP_INTC_IRQ(207)
#define IRQ_PO_VCON_SMPSERR	CP_INTC_IRQ(208)
#define IRQ_PO_VP_SMPSACK	CP_INTC_IRQ(209)
#define IRQ_FFTC_A_INTD0	CP_INTC_IRQ(210)
#define IRQ_FFTC_A_INTD1	CP_INTC_IRQ(211)
#define IRQ_FFTC_A_INTD2	CP_INTC_IRQ(212)
#define IRQ_FFTC_A_INTD3	CP_INTC_IRQ(213)
#define IRQ_INTDST7		CP_INTC_IRQ(216)
#define IRQ_INTDST8		CP_INTC_IRQ(217)
#define IRQ_INTDST9		CP_INTC_IRQ(218)
#define IRQ_INTDST10		CP_INTC_IRQ(219)
#define IRQ_INTDST11		CP_INTC_IRQ(220)
#define IRQ_INTDST12		CP_INTC_IRQ(221)
#define IRQ_INTDST13		CP_INTC_IRQ(222)
#define IRQ_INTDST14		CP_INTC_IRQ(223)
#define IRQ_AIF_INTD		CP_INTC_IRQ(224)
#define IRQ_AIF_SEVT0		CP_INTC_IRQ(227)
#define IRQ_AIF_SEVT1		CP_INTC_IRQ(228)
#define IRQ_AIF_SEVT2		CP_INTC_IRQ(229)
#define IRQ_AIF_SEVT3		CP_INTC_IRQ(230)
#define IRQ_AIF_SEVT4		CP_INTC_IRQ(231)
#define IRQ_AIF_SEVT5		CP_INTC_IRQ(232)
#define IRQ_AIF_SEVT6		CP_INTC_IRQ(233)
#define IRQ_AIF_SEVT7		CP_INTC_IRQ(234)
#define IRQ_CP_TRACER_CORE_3	CP_INTC_IRQ(235)
#define IRQ_PONIRQ		CP_INTC_IRQ(236)
#define IRQ_INTDST15		CP_INTC_IRQ(237)
#define IRQ_ARM_ETBFULLINT	CP_INTC_IRQ(239)
#define IRQ_ARM_ETBACQINT	CP_INTC_IRQ(240)
#define IRQ_KEYMGRINT_B		CP_INTC_IRQ(241)
#define IRQ_BCP_ERROR0		CP_INTC_IRQ(242)
#define IRQ_BCP_ERROR1		CP_INTC_IRQ(243)
#define IRQ_BCP_ERROR2		CP_INTC_IRQ(244)
#define IRQ_BCP_ERROR3		CP_INTC_IRQ(245)
#define IRQ_RAPID_INT_CDMA_0	CP_INTC_IRQ(246)
#define IRQ_CP_TRACER_SCR_6P_A	CP_INTC_IRQ(247)
#define IRQ_FFTC_B_INTD0	CP_INTC_IRQ(248)
#define IRQ_FFTC_B_INTD1	CP_INTC_IRQ(249)
#define IRQ_FFTC_B_INTD2	CP_INTC_IRQ(250)
#define IRQ_FFTC_B_INTD3	CP_INTC_IRQ(251)
#define IRQ_POSDMARREQ_INTD	CP_INTC_IRQ(252)
#define IRQ_POSDMAWREQ_INTD	CP_INTC_IRQ(253)
#define IRQ_TCP3D_B_INTD	CP_INTC_IRQ(254)

static struct irq_domain *tci6614_domain;

/* FIQ are pri 0-1; otherwise 2-7, with 7 lowest priority */
static u8 aintc_irq_prios[N_AINTC_IRQ] = {
	/* fill in default priority 0 */
	[0 ... (N_AINTC_IRQ - 1)]	= 0,
	/* now override as needed, e.g. [xxx] = 5 */
};


static u32 aintc_to_intd_map[N_AINTC_IRQ] = {
	[7]   = IRQ_QM_INT_HIGH_1,
	[8]   = IRQ_IPC_H,
	[9]   = IRQ_QM_INT_HIGH_0,
	[11]  = IRQ_QM_INT_HIGH_2,
	[12]  = IRQ_QM_INT_HIGH_3,
	[13]  = IRQ_QM_INT_HIGH_4,
	[14]  = IRQ_QM_INT_HIGH_5,
	[15]  = IRQ_QM_INT_HIGH_6,
	[16]  = IRQ_QM_INT_HIGH_7,
	[17]  = IRQ_QM_INT_HIGH_8,
	[18]  = IRQ_QM_INT_HIGH_9,
	[19]  = IRQ_QM_INT_HIGH_10,
	[20]  = IRQ_QM_INT_HIGH_11,
	[21]  = IRQ_QM_INT_HIGH_12,
	[22]  = IRQ_QM_INT_HIGH_13,
	[23]  = IRQ_QM_INT_HIGH_14,
	[24]  = IRQ_QM_INT_HIGH_15,
	[25]  = IRQ_QM_INT_HIGH_16,
	[26]  = IRQ_QM_INT_HIGH_17,
	[27]  = IRQ_QM_INT_HIGH_18,
	[28]  = IRQ_QM_INT_HIGH_19,
	[29]  = IRQ_QM_INT_HIGH_20,
	[30]  = IRQ_QM_INT_HIGH_21,
	[31]  = IRQ_QM_INT_HIGH_22,
	[32]  = IRQ_QM_INT_HIGH_23,
	[33]  = IRQ_QM_INT_HIGH_24,
	[34]  = IRQ_QM_INT_HIGH_25,
	[35]  = IRQ_QM_INT_HIGH_26,
	[36]  = IRQ_QM_INT_HIGH_27,
	[37]  = IRQ_QM_INT_HIGH_28,
	[38]  = IRQ_QM_INT_HIGH_29,
	[39]  = IRQ_QM_INT_HIGH_30,
	[40]  = IRQ_QM_INT_HIGH_31,
	[51]  = IRQ_VUSR_INT,
	[52]  = IRQ_SEMERR7,
	[53]  = IRQ_SEMINT7,
	[54]  = IRQ_SRIO_INTDST20,
	[55]  = IRQ_SRIO_INTDST21,
	[56]  = IRQ_SRIO_INTDST22,
	[57]  = IRQ_SRIO_INTDST23,
	[58]  = IRQ_TINT4L,
	[59]  = IRQ_TINT4H,
	[60]  = IRQ_TINT5L,
	[61]  = IRQ_TINT5H,
	[62]  = IRQ_TINT6L,
	[63]  = IRQ_TINT6H,
	[64]  = IRQ_TINT7L,
	[65]  = IRQ_TINT7H,
	[66]  = IRQ_PCIE_ERR_INT,
	[67]  = IRQ_PCIE_PM_INT,
	[68]  = IRQ_PCIE_LEGACY_INTA,
	[69]  = IRQ_PCIE_LEGACY_INTB,
	[70]  = IRQ_PCIE_LEGACY_INTC,
	[71]  = IRQ_PCIE_LEGACY_INTD,
	[72]  = IRQ_PCIE_MSI_INT4,
	[73]  = IRQ_PCIE_MSI_INT5,
	[74]  = IRQ_PCIE_MSI_INT6,
	[75]  = IRQ_PCIE_MSI_INT7,
	[76]  = IRQ_TINT8L,
	[77]  = IRQ_TINT8H,
	[78]  = IRQ_TINT9L,
	[79]  = IRQ_TINT9H,
	[80]  = IRQ_TINT10L,
	[81]  = IRQ_TINT10H,
	[82]  = IRQ_TINT11L,
	[83]  = IRQ_TINT11H,
	[84]  = IRQ_TCP3D_A_REVT0,
	[85]  = IRQ_TCP3D_A_REVT1,
	[86]  = IRQ_TCP3D_B_REVT0,
	[87]  = IRQ_TCP3D_B_REVT1,
	[88]  = IRQ_CPU_3_1_TPCC_INT2,
	[89]  = IRQ_CPU_3_1_TPCC_INT6,
	[90]  = IRQ_CPU_3_2_TPCC_INT2,
	[91]  = IRQ_CPU_3_2_TPCC_INT6,
	[92]  = IRQ_CPU_2_TPCC_INT2,
	[93]  = IRQ_CPU_2_TPCC_INT6,
	[127] = IRQ_WATCH_DOG_NMI,
};

#define AINTC_REVISION		0x0000
#define AINTC_SYSCONFIG		0x0010
#define AINTC_SYSSTATUS		0x0014
#define AINTC_SIR		0x0040
#define AINTC_CONTROL		0x0048
#define AINTC_PROTECTION	0x004C
#define AINTC_IDLE		0x0050
#define AINTC_THRESHOLD		0x0068
#define AINTC_MIR0		0x0084
#define AINTC_MIR_CLEAR0	0x0088
#define AINTC_MIR_SET0		0x008c
#define AINTC_PENDING_IRQ0	0x0098
#define AINTC_PRIORITY_IRQ0	0x0100
#define AINTC_IRQ_MASK		0x7f

/* Number of IRQ state bits in each MIR register */
#define IRQ_BITS_PER_REG	32

static void __iomem *aintc_base;
static bool debug_aintc = false;
#define debug_aintc_irq(irq)	(false)

static void aintc_writel(u32 val, u16 reg)
{
	if (debug_aintc)
		pr_info("aintc: write reg = %x, value = %x\n",
			reg, val);
	__raw_writel(val, aintc_base + reg);
}

static u32 aintc_readl(u16 reg)
{
	u32 val = __raw_readl(aintc_base + reg);
	if (debug_aintc)
		pr_info("aintc: read reg = %x, value = %x\n",
			reg, val);
	return val;
}

static void aintc_ack_irq(struct irq_data *d)
{
	unsigned int irq = d->hwirq - AINTC_IRQ_BASE;

	if (debug_aintc || debug_aintc_irq(irq))
		pr_info("aintc: ack %d\n", irq);
	aintc_writel(0x1, AINTC_CONTROL);
}

static void aintc_mask_irq(struct irq_data *d)
{
	unsigned int irq = d->hwirq - AINTC_IRQ_BASE;
	int offset = irq & (~(IRQ_BITS_PER_REG - 1));

	if (debug_aintc || debug_aintc_irq(irq))
		pr_info("aintc: mask %d\n", irq);

	irq &= (IRQ_BITS_PER_REG - 1);

	aintc_writel(1 << irq, AINTC_MIR_SET0 + offset);
}

static void aintc_unmask_irq(struct irq_data *d)
{
	unsigned int irq = d->hwirq - AINTC_IRQ_BASE;
	int offset = irq & (~(IRQ_BITS_PER_REG - 1));

	if (debug_aintc || debug_aintc_irq(irq))
		pr_info("aintc: unmask %d\n", irq);

	irq &= (IRQ_BITS_PER_REG - 1);

	aintc_writel(1 << irq, AINTC_MIR_CLEAR0 + offset);
}

static struct irq_chip aintc_irq_chip = {
	.name		= "AINTC",
	.irq_ack	= aintc_ack_irq,
	.irq_mask	= aintc_mask_irq,
	.irq_unmask	= aintc_unmask_irq,
};

static void __init aintc_init(void)
{
	unsigned long nr_of_irqs = N_AINTC_IRQ;
	unsigned long tmp;
	int i, irq;

	tmp = aintc_readl(AINTC_REVISION) & 0xff;
	pr_info("IRQ: Found an omap-aintc at 0x%p (revision %ld.%ld)\n",
		aintc_base, tmp >> 4, tmp & 0xf);

	tmp = aintc_readl(AINTC_SYSCONFIG);
	tmp |= 1 << 1;	/* soft reset */
	aintc_writel(tmp, AINTC_SYSCONFIG);

	while (!(aintc_readl(AINTC_SYSSTATUS) & 0x1))
		/* Wait for reset to complete */;

	for (i = 0; i < nr_of_irqs; i++) {
		aintc_writel(aintc_irq_prios[i], AINTC_PRIORITY_IRQ0 + 4 * i);
		irq = irq_create_mapping(tci6614_domain, AINTC_IRQ(i));
		irq_set_chip_and_handler(irq, &aintc_irq_chip,
					 handle_level_irq);
		set_irq_flags(irq, IRQF_VALID);
		if (debug_aintc)
			pr_info("aintc[%d] setup, irq %d\n", i, irq);
	}
}

#define CP_INTD_REVISION	0x000
#define CP_INTD_ENABLE_SET	0x100
#define CP_INTD_ENABLE_CLEAR	0x180
#define CP_INTD_STATUS_SET	0x200
#define CP_INTD_STATUS_CLEAR	0x280

#define CP_INTD_REG(irq)	(BIT_WORD(irq) << 2)
#define CP_INTD_BIT(irq)	(BIT_MASK(irq))
#define CP_INTD_IRQ(x)		(CP_INTD_IRQ_BASE + (x))

static void __iomem *cp_intd_base;
static bool debug_cp_intd = false;
#define debug_cp_intd_irq(irq)	(false)

static inline u32 cp_intd_readl(int offset)
{
	u32 value = __raw_readl(cp_intd_base + offset);
	if (debug_cp_intd)
		pr_info("intd: read offset %x = %x\n", offset, value);
	return value;
}

static inline void cp_intd_writel(u32 value, int offset)
{
	if (debug_cp_intd)
		pr_info("intd: write offset %x = %x\n", offset, value);
	__raw_writel(value, cp_intd_base + offset);
}

static void cp_intd_mask_irq(struct irq_data *d)
{
	unsigned int irq = d->hwirq - CP_INTD_IRQ_BASE;

	if (debug_cp_intd || debug_cp_intd_irq(irq))
		pr_info("intd: mask %d\n", irq);
	cp_intd_writel(CP_INTD_BIT(irq), CP_INTD_ENABLE_CLEAR + CP_INTD_REG(irq));
	cp_intd_readl(CP_INTD_ENABLE_SET + CP_INTD_REG(irq));
}

static void cp_intd_unmask_irq(struct irq_data *d)
{
	unsigned int irq = d->hwirq - CP_INTD_IRQ_BASE;

	if (debug_cp_intd || debug_cp_intd_irq(irq))
		pr_info("intd: unmask %d\n", irq);
	cp_intd_writel(CP_INTD_BIT(irq), CP_INTD_ENABLE_SET + CP_INTD_REG(irq));
	cp_intd_readl(CP_INTD_ENABLE_SET + CP_INTD_REG(irq));
}

static void cp_intd_ack_irq(struct irq_data *d)
{
	unsigned int irq = d->hwirq - CP_INTD_IRQ_BASE;

	if (debug_cp_intd || debug_cp_intd_irq(irq))
		pr_info("intd: ack %d\n", irq);
	cp_intd_writel(CP_INTD_BIT(irq), CP_INTD_STATUS_CLEAR + CP_INTD_REG(irq));
	cp_intd_readl(CP_INTD_STATUS_SET + CP_INTD_REG(irq));
}

static bool cp_intd_irq_pending(unsigned int irq)
{
	bool enabled, pending;

	enabled = (cp_intd_readl(CP_INTD_ENABLE_SET + CP_INTD_REG(irq)) &
		   CP_INTD_BIT(irq)) ? true : false;
	pending = (cp_intd_readl(CP_INTD_STATUS_SET + CP_INTD_REG(irq)) &
		   CP_INTD_BIT(irq)) ? true : false;
	if (debug_cp_intd || debug_cp_intd_irq(irq)) {
		pr_info("intd: %d %s, %s\n", irq,
			pending ? "pending" : "not pending",
			enabled ? "enabled" : "not enabled");
	}
	return pending;
}

static struct irq_chip cp_intd_chip = {
	.name		= "CP_INTD",
	.irq_ack	= cp_intd_ack_irq,
	.irq_mask	= cp_intd_mask_irq,
	.irq_unmask	= cp_intd_unmask_irq,
};

static void cp_intd_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct irq_data *d = irq_desc_get_irq_data(desc);
	int aintc_irq = d->hwirq - AINTC_IRQ_BASE;
	int cp_intd_irq = aintc_to_intd_map[aintc_irq] - CP_INTD_IRQ_BASE;

	chip->irq_mask(d);
	chip->irq_ack(d);
	if (debug_cp_intd || debug_cp_intd_irq(cp_intd_irq))
		pr_info("intd: irq %d, aintc %d, cp_intd %d\n",
			irq, aintc_irq, cp_intd_irq);
	if (cp_intd_irq >= 0 && cp_intd_irq_pending(cp_intd_irq)) {
		generic_handle_irq(irq_linear_revmap(tci6614_domain,
						     CP_INTD_IRQ(cp_intd_irq)));
	}
	chip->irq_unmask(d);
}

static void __init cp_intd_init(void)
{
	unsigned long num_irq = N_CP_INTD_IRQ;
	unsigned long rev;
	int i, irq;

	rev = cp_intd_readl(CP_INTD_REVISION);
	pr_info("IRQ: intd version %ld.%ld at %p\n",
	       (rev >> 8) & 0x3, rev & 0x3f, cp_intd_base);

	for (i = 0; i < DIV_ROUND_UP(num_irq, 32); i++) {
		cp_intd_writel(0xffffffff, CP_INTD_STATUS_CLEAR + (4 * i));
		cp_intd_writel(0xffffffff, CP_INTD_ENABLE_CLEAR + (4 * i));
	}

	for (i = 0; i < num_irq; i++) {
		irq = irq_create_mapping(tci6614_domain, CP_INTD_IRQ(i));
		irq_set_chip_and_handler(irq, &cp_intd_chip, handle_edge_irq);
		set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);
		if (debug_cp_intd)
			pr_info("cp_intd[%d] setup, irq %d\n", i, irq);
	}

	for (i = 0; i < N_AINTC_IRQ; i++) {
		if (!aintc_to_intd_map[i])
			continue;
		irq = irq_linear_revmap(tci6614_domain, AINTC_IRQ(i));
		irq_set_chained_handler(irq, cp_intd_irq_handler);
		if (debug_aintc || debug_aintc_irq(i))
			pr_info("aintc[%d] attached to cp_intd\n", i);
	}
}

#define CP_INTC_REVISION		0x000
#define CP_INTC_GLOBAL_ENABLE		0x010
#define CP_INTC_STATUS_IDX_SET		0x020
#define CP_INTC_STATUS_IDX_CLEAR	0x024
#define CP_INTC_ENABLE_IDX_SET		0x028
#define CP_INTC_ENABLE_IDX_CLEAR	0x02c
#define CP_INTC_HOST_ENABLE_SET		0x034
#define CP_INTC_HOST_ENABLE_CLEAR	0x038
#define CP_INTC_GLOBAL_INDEX		0x080
#define CP_INTC_STATUS_SET		0x200
#define CP_INTC_STATUS_CLEAR		0x280
#define CP_INTC_ENABLE_SET		0x300
#define CP_INTC_ENABLE_CLEAR		0x380
#define CP_INTC_CHANNEL_MAP		0x400
#define CP_INTC_HOST_MAP		0x800

#define CP_INTC_CHANS			8
#define CP_INTC_REG(irq)		(BIT_WORD(irq) << 2)
#define CP_INTC_BIT(irq)		(BIT_MASK(irq))

static void __iomem *cp_intc_base;
static bool debug_cp_intc = false;
#define debug_cp_intc_irq(irq)	(false)

static inline u32 cp_intc_readl(int offset)
{
	u32 value = __raw_readl(cp_intc_base + offset);
	if (debug_cp_intc)
		pr_info("cp_intc: read offset %x = %x\n", offset, value);
	return value;
}

static inline void cp_intc_writel(u32 value, int offset)
{
	if (debug_cp_intc)
		pr_info("cp_intc: write offset %x = %x\n", offset, value);
	__raw_writel(value, cp_intc_base + offset);
}

static void cp_intc_mask_irq(struct irq_data *d)
{
	unsigned int irq = d->hwirq - CP_INTC_IRQ_BASE;

	if (debug_cp_intc || debug_cp_intc_irq(irq))
		pr_info("cp_intc: mask %d\n", irq);
	cp_intc_writel(irq, CP_INTC_ENABLE_IDX_CLEAR);
}

static void cp_intc_unmask_irq(struct irq_data *d)
{
	unsigned int irq = d->hwirq - CP_INTC_IRQ_BASE;

	if (debug_cp_intc || debug_cp_intc_irq(irq))
		pr_info("cp_intc: unmask %d\n", irq);
	cp_intc_writel(irq, CP_INTC_ENABLE_IDX_SET);
}

static void cp_intc_ack_irq(struct irq_data *d)
{
	unsigned int irq = d->hwirq - CP_INTC_IRQ_BASE;

	if (debug_cp_intc || debug_cp_intc_irq(irq))
		pr_info("cp_intc: ack %d\n", irq);
	cp_intc_writel(irq, CP_INTC_STATUS_IDX_CLEAR);
}

static bool cp_intc_irq_pending(unsigned int irq)
{
	bool enabled, pending;

	enabled = (cp_intc_readl(CP_INTC_ENABLE_SET + CP_INTC_REG(irq)) &
		   CP_INTC_BIT(irq)) ? true : false;
	pending = (cp_intc_readl(CP_INTC_STATUS_SET + CP_INTC_REG(irq)) &
		   CP_INTC_BIT(irq)) ? true : false;
	if (debug_cp_intc || debug_cp_intc_irq(irq)) {
		pr_info("cp_intc: %d %s, %s\n", irq,
			pending ? "pending" : "not pending",
			enabled ? "enabled" : "not enabled");
	}
	return pending;
}

static struct irq_chip cp_intc_chip = {
	.name		= "CP_INTC",
	.irq_ack	= cp_intc_ack_irq,
	.irq_mask	= cp_intc_mask_irq,
	.irq_unmask	= cp_intc_unmask_irq,
};

static void cp_intc_irq_handler(unsigned irq, struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct irq_data *d = irq_desc_get_irq_data(desc);
	unsigned cp_intc_irq;

	if (debug_cp_intc)
		pr_info("cp_intc: start irq %d\n", irq);

	chip->irq_mask(d);
	chip->irq_ack(d);

	for (;;) {
		cp_intc_irq = cp_intc_readl(CP_INTC_GLOBAL_INDEX);
		if (cp_intc_irq >= N_CP_INTC_IRQ ||
		    !cp_intc_irq_pending(cp_intc_irq))
			break;
		if (debug_cp_intc || debug_cp_intc_irq(cp_intc_irq))
			pr_info("intc: irq %d, cp_intc %d\n",
				irq, cp_intc_irq);
		generic_handle_irq(irq_linear_revmap(tci6614_domain,
						     CP_INTC_IRQ(cp_intc_irq)));
	}
	chip->irq_unmask(d);

	if (debug_cp_intc)
		pr_info("cp_intc: end irq %d\n", irq);
}

static void __init cp_intc_init(void)
{
	int i, irq;
	unsigned long rev;

	rev = cp_intc_readl(CP_INTC_REVISION);
	pr_info("IRQ: cp_intc version %ld.%ld at %p\n",
	       (rev >> 8) & 0x3, rev & 0x3f, cp_intc_base);

	/* map all irqs to channel 0 */
	for (i = 0; i < DIV_ROUND_UP(N_CP_INTC_IRQ, 4); i++)
		cp_intc_writel(0, CP_INTC_CHANNEL_MAP + (4 * i));

	/* map all channels to host 0 */
	for (i = 0; i < DIV_ROUND_UP(CP_INTC_CHANS, 4); i++)
		cp_intc_writel(0, CP_INTC_HOST_MAP + (4 * i));

	cp_intc_writel(0, CP_INTC_HOST_ENABLE_SET);

	for (i = 0; i < N_CP_INTC_IRQ; i++) {
		irq = irq_create_mapping(tci6614_domain, CP_INTC_IRQ(i));
		irq_set_chip_and_handler(irq, &cp_intc_chip, handle_edge_irq);
		set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);
		if (debug_cp_intc)
			pr_info("cp_intc[%d] setup, irq %d\n", i, irq);
	}

	for (i = IRQ_INTC3_OUT0; i <= IRQ_INTC3_OUT32; i++) {
		irq = irq_linear_revmap(tci6614_domain, i);
		irq_set_chained_handler(irq, cp_intc_irq_handler);
		if (debug_aintc || debug_aintc_irq(i))
			pr_info("aintc[%d] attached to cp_intc\n", i);
	}

	cp_intc_writel(1, CP_INTC_GLOBAL_ENABLE);
}

int __init tci6614_of_init_irq(struct device_node *np,
			       struct device_node *parent)
{
	aintc_base	= of_iomap(np, 0);
	cp_intd_base	= of_iomap(np, 1);
	cp_intc_base	= of_iomap(np, 2);

	if (!aintc_base || !cp_intd_base || !cp_intc_base)
		panic("failed to map irq controller registers\n");

	tci6614_domain = irq_domain_add_linear(np, N_IRQ,
					       &irq_domain_simple_ops, NULL);
	if (!tci6614_domain)
		panic("failed to add irq domain!\n");

	aintc_init();
	cp_intd_init();
	cp_intc_init();

	return 0;
}

asmlinkage void __exception_irq_entry tci6614_handle_irq(struct pt_regs *regs)
{
	u32 irqnr;
	int irq;

	do {
		irqnr = aintc_readl(0x98);
		if (!irqnr)
			irqnr = aintc_readl(0xb8);
		if (!irqnr)
			irqnr = aintc_readl(0xd8);
		if (!irqnr)
			irqnr = aintc_readl(0xf8);
		if (!irqnr)
			break;

		irqnr = aintc_readl(AINTC_SIR) & AINTC_IRQ_MASK;

		if (!irqnr)
			break;
		irq = irq_linear_revmap(tci6614_domain, AINTC_IRQ(irqnr));
		if (debug_aintc || debug_aintc_irq(irqnr))
			pr_info("aintc: dispatch irq %d [%d]\n", irqnr, irq);
		handle_IRQ(irq, regs);
	} while (irqnr);
}
