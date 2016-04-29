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
 *
 * These routines provide access to the serdes
 *
 */

/* ---- Include Files ---------------------------------------------------- */
#include <bcmutils.h>
#include <bcmenetphy.h>
#include "bcmiproc_serdes.h"
#include "bcmiproc_serdes_def.h"
#include "../../../mdio/iproc_mdio.h"

/* ---- External Variable Declarations ----------------------------------- */
/* ---- External Function Prototypes ------------------------------------- */
/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */
/* ---- Private Variables ------------------------------------------------ */

/* debug/trace */
//#define BCMDBG
//#define BCMDBG_ERR
#ifdef BCMDBG
#define	NET_ERROR(args) printf args
#define	NET_TRACE(args) printf args
#elif defined(BCMDBG_ERR)
#define	NET_ERROR(args) printf args
#define NET_TRACE(args)
#else
#define	NET_ERROR(args)
#define	NET_TRACE(args)
#endif /* BCMDBG */
#define	NET_REG_TRACE(args)


#ifndef ASSERT
#define ASSERT(exp)
#endif

/* ==== Public Functions ================================================= */

void
serdes_set_blk(uint eth_num, uint phyaddr, uint blk)
{
	uint16 blkaddr;
	uint16 destblk = (uint16)blk;

	NET_TRACE(("%s enter\n", __FUNCTION__));

	NET_REG_TRACE(("%s phyaddr(0x%x) blk(0x%x)\n",
		 __FUNCTION__, phyaddr, blk));

	/* check if need to update blk addr */
	ccb_mii_read(MII_DEV_LOCAL, phyaddr, PHY_REG_BLK_ADDR, &blkaddr);
	if (blkaddr!=destblk) {
		/* write block address */
		ccb_mii_write(MII_DEV_LOCAL, phyaddr, PHY_REG_BLK_ADDR, destblk);
	}
}


void
serdes_wr_reg(uint eth_num, uint phyaddr, uint reg, uint data)
{
	uint blk = reg&0x7ff0;
	uint off = reg&0x000f;
	uint16 tmpdata=(uint16)data;

	NET_TRACE(("%s enter\n", __FUNCTION__));

	if (reg&0x8000)
		off|=0x10;

	/* set block address */
	serdes_set_blk(eth_num, phyaddr, blk);

	NET_REG_TRACE(("%s wrt phyaddr(0x%x) reg(0x%x) data(0x%x)\n",
		 __FUNCTION__, phyaddr, reg, tmpdata));
	//printf("%s wrt phyaddr(0x%x) reg(0x%x) data(0x%x)\n",
	//	 __FUNCTION__, phyaddr, reg, tmpdata);
	/* write register */
	ccb_mii_write(MII_DEV_LOCAL, phyaddr, off, tmpdata);
}


uint16
serdes_rd_reg(uint eth_num, uint phyaddr, uint reg)
{
	uint blk = reg&0x7ff0;
	uint off = reg&0x000f;
	uint16	data;

	NET_TRACE(("%s enter\n", __FUNCTION__));

	if (reg&0x8000)
		off|=0x10;

	/* set block address */
	serdes_set_blk(eth_num, phyaddr, blk);

	/* read register */
	ccb_mii_read(MII_DEV_LOCAL, phyaddr, off, &data);
	NET_REG_TRACE(("%s rd phyaddr(0x%x) reg(0x%x) data(0x%x)\n",
		 __FUNCTION__, phyaddr, reg, data));
	//printf("%s rd phyaddr(0x%x) reg(0x%x) data(0x%x)\n",
	//	 __FUNCTION__, phyaddr, reg, data);

	return data;
}


uint16
serdes_get_id(uint eth_num, uint phyaddr, uint off)
{

	ASSERT(phyaddr < MAXEPHY);

	if (phyaddr == EPHY_NOREG)
		return 0;

	/* read the id high */
	return serdes_rd_reg(eth_num, phyaddr, XGXS16G_SERDESID_SERDESID0r+off);
}


void
serdes_reset(uint eth_num, uint phyaddr)
{
	uint ctrl;

	ASSERT(phyaddr < MAXEPHY);

	if (phyaddr == EPHY_NOREG)
		return;

	NET_TRACE(("et%d: %s: phyaddr %d\n", eth_num, __FUNCTION__, phyaddr));

	/* set reset flag */
	ctrl = serdes_rd_reg(eth_num, phyaddr, XGXS16G_IEEE0BLK_IEEECONTROL0r);
	ctrl |= IEEE0BLK_IEEECONTROL0_RST_HW_MASK;
	serdes_wr_reg(eth_num, phyaddr, XGXS16G_IEEE0BLK_IEEECONTROL0r, ctrl);
	udelay(100);
	/* check if out of reset */
	if (serdes_rd_reg(eth_num, phyaddr, XGXS16G_IEEE0BLK_IEEECONTROL0r) & IEEE0BLK_IEEECONTROL0_RST_HW_MASK) {
		NET_ERROR(("et%d: %s reset not complete\n", eth_num, __FUNCTION__));
	}
}


int
serdes_reset_core(uint eth_num, uint phyaddr)
{
	uint16		data16;
	uint16		serdes_id2;

	NET_TRACE(("et%d: %s: phyaddr %d\n", eth_num, __FUNCTION__, phyaddr));

	/* get serdes id */
	serdes_id2 = serdes_get_id(eth_num, phyaddr, 2);
	printf("et%d %s pbyaddr(0x%x) id2(0x%x)\n", eth_num, __FUNCTION__, phyaddr, serdes_id2);

	/* unlock lane */
	data16 = serdes_rd_reg(eth_num, phyaddr, 0x833c);
	data16 &= ~(0x0040);
	serdes_wr_reg(eth_num, phyaddr, 0x833c, data16);

	if ( phyaddr == 1 ) {
		/* Reset the core */
		/* Stop PLL Sequencer and configure the core into correct mode */
		data16 = (XGXSBLK0_XGXSCONTROL_MODE_10G_IndLane <<
					XGXSBLK0_XGXSCONTROL_MODE_10G_SHIFT) |
				XGXSBLK0_XGXSCONTROL_HSTL_MASK |
				XGXSBLK0_XGXSCONTROL_CDET_EN_MASK |
				XGXSBLK0_XGXSCONTROL_EDEN_MASK |
				XGXSBLK0_XGXSCONTROL_AFRST_EN_MASK |
				XGXSBLK0_XGXSCONTROL_TXCKO_DIV_MASK;
		serdes_wr_reg(eth_num, phyaddr, XGXS16G_XGXSBLK0_XGXSCONTROLr, data16);

		/* Disable IEEE block select auto-detect. 
		* The driver will select desired block as necessary.
		* By default, the driver keeps the XAUI block in
		* IEEE address space.
		*/
		data16 = serdes_rd_reg(eth_num, phyaddr, XGXS16G_XGXSBLK0_MISCCONTROL1r);
		if (XGXS16G_2p5G_ID(serdes_id2)) {
			data16 &= ~(	XGXSBLK0_MISCCONTROL1_IEEE_BLKSEL_AUTODET_MASK |
							XGXSBLK0_MISCCONTROL1_IEEE_BLKSEL_VAL_MASK);
		} else {
			data16 &= ~(	XGXSBLK0_MISCCONTROL1_IEEE_BLKSEL_AUTODET_MASK |
							XGXSBLK0_MISCCONTROL1_IEEE_BLKSEL_VAL_MASK);
#if !defined(CONFIG_MACH_KT2)
			data16 |= XGXSBLK0_MISCCONTROL1_IEEE_BLKSEL_VAL_MASK;
#endif /* (!defined(CONFIG_MACH_KT2)) */
		}
		serdes_wr_reg(eth_num, phyaddr, XGXS16G_XGXSBLK0_MISCCONTROL1r, data16);

		/* disable in-band MDIO. PHY-443 */
		data16 = serdes_rd_reg(eth_num, phyaddr, 0x8111);
		/* rx_inBandMdio_rst */
		data16 |= 1 << 3;
		serdes_wr_reg(eth_num, phyaddr, 0x8111, data16);
	}
	return 0;
}


int
serdes_start_pll(uint eth_num, uint phyaddr)
{
	uint16		data16;

	if ( phyaddr == 1 ) {
		uint32		count=250;
		 /* Start PLL Sequencer and wait for PLL to lock */
		data16 = serdes_rd_reg(eth_num, phyaddr, XGXS16G_XGXSBLK0_XGXSCONTROLr);
		data16 |= XGXSBLK0_XGXSCONTROL_START_SEQUENCER_MASK;
		serdes_wr_reg(eth_num, phyaddr, XGXS16G_XGXSBLK0_XGXSCONTROLr, data16);

		/* wait for PLL to lock */
		while (count!=0) {
			data16 = serdes_rd_reg(eth_num, phyaddr, XGXS16G_XGXSBLK0_XGXSSTATUSr);
			if ( data16 & XGXSBLK0_XGXSSTATUS_TXPLL_LOCK_MASK ) {
	            break;
			}
			/* wait 1 usec then dec counter */
			udelay(10);
			count--;
		}
		if (count == 0) {
			NET_ERROR(("%s TXPLL did not lock\n", __FUNCTION__));
		}
	}
	return 0;
}


/*
 * Function:
 *      serdes_init
 * Purpose:
 *      Initialize xgxs6 phys
 * Parameters:
 *      eth_num - ethernet data
 *      phyaddr - physical address
 * Returns:
 *      0
 */
int
serdes_init(uint eth_num, uint phyaddr)
{
	uint16		data16;
	uint16		serdes_id0, serdes_id1, serdes_id2;

	NET_TRACE(("et%d: %s: phyaddr %d\n", eth_num, __FUNCTION__, phyaddr));

	/* get serdes id */
	serdes_id0 = serdes_get_id(eth_num, phyaddr, 0);
	serdes_id1 = serdes_get_id(eth_num, phyaddr, 1);
	serdes_id2 = serdes_get_id(eth_num, phyaddr, 2);
	printf("%s pbyaddr(0x%x) id0(0x%x) id1(0x%x) id2(0x%x)\n", __FUNCTION__, phyaddr, serdes_id0, serdes_id1, serdes_id2);

	/* get more ids */
	serdes_id0 = serdes_rd_reg(eth_num, phyaddr, 2);
	serdes_id1 = serdes_rd_reg(eth_num, phyaddr, 3);
	//printf("%s pbyaddr(0x%x) SERDES PhyID_MS(0x%x) PhyID_LS(0x%x)\n", __FUNCTION__, phyaddr, serdes_id0, serdes_id1);

	/* unlock lane */
	data16 = serdes_rd_reg(eth_num, phyaddr, 0x833c);
	data16 &= ~(0x0040);
	serdes_wr_reg(eth_num, phyaddr, 0x833c, data16);

	/* disable CL73 BAM */
	data16 = serdes_rd_reg(eth_num, phyaddr, 0x8372);
	data16 &= ~(CL73_USERB0_CL73_BAMCTRL1_CL73_BAMEN_MASK);
	serdes_wr_reg(eth_num, phyaddr, 0x8372, data16);

	/* Set Local Advertising Configuration */
	data16 = MII_ANA_C37_FD | MII_ANA_C37_PAUSE | MII_ANA_C37_ASYM_PAUSE;
	serdes_wr_reg(eth_num, phyaddr, XGXS16G_COMBO_IEEE0_AUTONEGADVr, data16);

	/* Disable BAM in Independent Lane mode. Over1G AN not supported  */
	data16 = 0;
	serdes_wr_reg(eth_num, phyaddr, XGXS16G_BAM_NEXTPAGE_MP5_NEXTPAGECTRLr, data16);
	serdes_wr_reg(eth_num, phyaddr, XGXS16G_BAM_NEXTPAGE_UD_FIELDr, data16);

	data16 = SERDESDIGITAL_CONTROL1000X1_CRC_CHECKER_DISABLE_MASK |
				SERDESDIGITAL_CONTROL1000X1_DISABLE_PLL_PWRDWN_MASK;
	/*
	* Put the Serdes in SGMII mode
	* bit0 = 0; in SGMII mode
	*/
	serdes_wr_reg(eth_num, phyaddr, XGXS16G_SERDESDIGITAL_CONTROL1000X1r, data16);

	/* set autoneg */
	data16 = MII_CTRL_AE | MII_CTRL_RAN;
	serdes_wr_reg(eth_num, phyaddr, XGXS16G_COMBO_IEEE0_MIICNTLr, data16);

	/* Disable 10G parallel detect */
	data16 = 0;
	serdes_wr_reg(eth_num, phyaddr, XGXS16G_AN73_PDET_PARDET10GCONTROLr, data16);

	/* Disable BAM mode and Teton mode */
	serdes_wr_reg(eth_num, phyaddr, XGXS16G_BAM_NEXTPAGE_MP5_NEXTPAGECTRLr, data16);

	/* Enable lanes */
	data16 = serdes_rd_reg(eth_num, phyaddr, XGXS16G_XGXSBLK1_LANECTRL0r);
	data16 |= XGXSBLK1_LANECTRL0_CL36_PCS_EN_RX_MASK |
				XGXSBLK1_LANECTRL0_CL36_PCS_EN_TX_MASK;
	serdes_wr_reg(eth_num, phyaddr, XGXS16G_XGXSBLK1_LANECTRL0r, data16);

    /* set elasticity fifo size to 13.5k to support 12k jumbo pkt size*/
	data16 = serdes_rd_reg(eth_num, phyaddr, XGXS16G_SERDESDIGITAL_CONTROL1000X3r);
	data16 &= SERDESDIGITAL_CONTROL1000X3_FIFO_ELASICITY_TX_RX_MASK;
	data16 |= (1 << 2);
	serdes_wr_reg(eth_num, phyaddr, XGXS16G_SERDESDIGITAL_CONTROL1000X3r, data16);

    /* Enabble LPI passthru' for native mode EEE */
	data16 = serdes_rd_reg(eth_num, phyaddr, XGXS16G_REMOTEPHY_MISC5r);
	data16 |= 0xc000;
	serdes_wr_reg(eth_num, phyaddr, XGXS16G_REMOTEPHY_MISC5r, data16);
	data16 = serdes_rd_reg(eth_num, phyaddr, XGXS16G_XGXSBLK7_EEECONTROLr);
	data16 |= 0x0007;
	serdes_wr_reg(eth_num, phyaddr, XGXS16G_XGXSBLK7_EEECONTROLr, data16);

	return 0;
}
