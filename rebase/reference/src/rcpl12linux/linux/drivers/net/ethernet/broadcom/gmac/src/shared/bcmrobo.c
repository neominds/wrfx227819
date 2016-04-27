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
 * Broadcom 53xx RoboSwitch device driver.
 *
 * $Id: bcmrobo.c 327582 2012-04-14 05:02:37Z $
 */


#include <bcm_cfg.h>
#include <typedefs.h>
#include <osl.h>
#include <bcmutils.h>
#include <siutils.h>
#include <sbchipc.h>
#include <hndsoc.h>
#include <bcmutils.h>
#include <bcmendian.h>
#include <bcmparams.h>
#include <bcmnvram.h>
#include <bcmdevs.h>
#include <bcmrobo.h>
#include <bcmiproc_robo_serdes.h>
#include <proto/ethernet.h>

#include "bcm5301x_otp.h"

#ifdef	BCMDBG
#define	ET_ERROR(args)	printf args
#else	/* BCMDBG */
#define	ET_ERROR(args)
#endif	/* BCMDBG */
#define	ET_MSG(args)

#define VARG(var, len) (((len) == 1) ? *((uint8 *)(var)) : \
		        ((len) == 2) ? *((uint16 *)(var)) : \
		        *((uint32 *)(var)))

//#define BRCM_TAG true

/* Page numbers */
#define PAGE_CTRL	0x00	/* Control page */
#define PAGE_STATUS	0x01	/* Status page */
#define PAGE_MMR	0x02	/* 5397 Management/Mirroring page */
#define PAGE_VTBL	0x05	/* ARL/VLAN Table access page */
#define PAGE_VLAN	0x34	/* VLAN page */

/* Control page registers */
#define REG_CTRL_PORT0	0x00	/* Port 0 traffic control register */
#define REG_CTRL_PORT1	0x01	/* Port 1 traffic control register */
#define REG_CTRL_PORT2	0x02	/* Port 2 traffic control register */
#define REG_CTRL_PORT3	0x03	/* Port 3 traffic control register */
#define REG_CTRL_PORT4	0x04	/* Port 4 traffic control register */
#define REG_CTRL_PORT5	0x05	/* Port 5 traffic control register */
#define REG_CTRL_PORT6	0x06	/* Port 6 traffic control register */
#define REG_CTRL_PORT7	0x07	/* Port 7 traffic control register */
#define REG_CTRL_IMP	0x08	/* IMP port traffic control register */
#define REG_CTRL_MODE	0x0B	/* Switch Mode register */
#define REG_CTRL_MIIPO	0x0E	/* 5325: MII Port Override register */
#define REG_CTRL_PWRDOWN 0x0F   /* 5325: Power Down Mode register */
#define REG_CTRL_PHY_PWR	0x4a	/* phy power down register */
#define REG_CTRL_SRST	0x79	/* Software reset control register */

/* Status Page Registers */
#define REG_STATUS_LINK	0x00	/* Link Status Summary */
#define REG_STATUS_REV	0x50	/* Revision Register */

#define REG_MGMT_CFG	0x00	/* Global Management Configuration */
#define REG_BRCM_HDR	0x03	/* BRCM Header Control */
#define REG_DEVICE_ID	0x30	/* 539x Device id: */

/* VLAN page registers */
#define REG_VLAN_CTRL0	0x00	/* VLAN Control 0 register */
#define REG_VLAN_CTRL1	0x01	/* VLAN Control 1 register */
#define REG_VLAN_CTRL4	0x04	/* VLAN Control 4 register */
#define REG_VLAN_CTRL5	0x06	/* VLAN Control 5 register */
#define REG_VLAN_ACCESS	0x06	/* VLAN Table Access register */
#define REG_VLAN_WRITE	0x08	/* VLAN Write register */
#define REG_VLAN_READ	0x0C	/* VLAN Read register */
#define REG_VLAN_PTAG0	0x10	/* VLAN Default Port Tag register - port 0 */
#define REG_VLAN_PTAG1	0x12	/* VLAN Default Port Tag register - port 1 */
#define REG_VLAN_PTAG2	0x14	/* VLAN Default Port Tag register - port 2 */
#define REG_VLAN_PTAG3	0x16	/* VLAN Default Port Tag register - port 3 */
#define REG_VLAN_PTAG4	0x18	/* VLAN Default Port Tag register - port 4 */
#define REG_VLAN_PTAG5	0x1a	/* VLAN Default Port Tag register - port 5 */
#define REG_VLAN_PTAG6	0x1c	/* VLAN Default Port Tag register - port 6 */
#define REG_VLAN_PTAG7	0x1e	/* VLAN Default Port Tag register - port 7 */
#define REG_VLAN_PTAG8	0x20	/* 539x: VLAN Default Port Tag register - IMP port */
#define REG_VLAN_PMAP	0x20	/* 5325: VLAN Priority Re-map register */
#define REG_CTRL_PPORT	0x24	/* Protected port register */

#define VLAN_NUMVLANS	16	/* # of VLANs */


/* ARL/VLAN Table Access page registers */
#define REG_VTBL_CTRL		0x00	/* ARL Read/Write Control */
#define REG_VTBL_MINDX		0x02	/* MAC Address Index */
#define REG_VTBL_VINDX		0x08	/* VID Table Index */
#define REG_VTBL_ARL_E0		0x10	/* ARL Entry 0 */
#define REG_VTBL_ARL_E1		0x18	/* ARL Entry 1 */
#define REG_VTBL_DAT_E0		0x18	/* ARL Table Data Entry 0 */
#define REG_VTBL_SCTRL		0x20	/* ARL Search Control */
#define REG_VTBL_SADDR		0x22	/* ARL Search Address */
#define REG_VTBL_SRES		0x24	/* ARL Search Result */
#define REG_VTBL_SREXT		0x2c	/* ARL Search Result */
#define REG_VTBL_VID_E0		0x30	/* VID Entry 0 */
#define REG_VTBL_VID_E1		0x32	/* VID Entry 1 */
#define REG_VTBL_PREG		0xFF	/* Page Register */
#define REG_VTBL_ACCESS		0x60	/* VLAN table access register */
#define REG_VTBL_INDX		0x61	/* VLAN table address index register */
#define REG_VTBL_ENTRY		0x63	/* VLAN table entry register */
#define REG_VTBL_ACCESS_5395	0x80	/* VLAN table access register */
#define REG_VTBL_INDX_5395	0x81	/* VLAN table address index register */
#define REG_VTBL_ENTRY_5395	0x83	/* VLAN table entry register */

#define SRAB_MAX_RETRY		1000

void * (*bcm5301x_otp_init_fptr )(void) = NULL;
int  (*bcm5301x_otp_read_dword_fptr)(void *oh, uint wn, u32 *data) = NULL;
int (*bcm5301x_otp_exit_fptr)(void) = NULL;

static int
srab_request_grant(robo_info_t *robo)
{
	int i, ret = 0;
	uint32 val32;

	val32 = R_REG(si_osh(robo->sih), &robo->srabregs->ctrls);
	val32 |= CFG_F_rcareq_MASK;
	W_REG(si_osh(robo->sih), &robo->srabregs->ctrls, val32);

	/* Wait for command complete */
	for (i = SRAB_MAX_RETRY * 10; i > 0; i --) {
		val32 = R_REG(si_osh(robo->sih), &robo->srabregs->ctrls);
		if ((val32 & CFG_F_rcagnt_MASK))
			break;
	}

	/* timed out */
	if (!i) {
		ET_ERROR(("srab_request_grant: timeout"));
		ret = -1;
	}

	return ret;
}

static void
srab_release_grant(robo_info_t *robo)
{
	uint32 val32;

	val32 = R_REG(si_osh(robo->sih), &robo->srabregs->ctrls);
	val32 &= ~CFG_F_rcareq_MASK;
	W_REG(si_osh(robo->sih), &robo->srabregs->ctrls, val32);
}

static int
srab_interface_reset(robo_info_t *robo)
{
	int i, ret = 0;
	uint32 val32;

	/* Wait for switch initialization complete */
	for (i = SRAB_MAX_RETRY * 10; i > 0; i --) {
		val32 = R_REG(si_osh(robo->sih), &robo->srabregs->ctrls);
		if ((val32 & CFG_F_sw_init_done_MASK))
			break;
	}

	/* timed out */
	if (!i) {
		ET_ERROR(("srab_interface_reset: timeout sw_init_done"));
		ret = -1;
	}

	/* Set the SRAU reset bit */
	W_REG(si_osh(robo->sih), &robo->srabregs->cmdstat, CFG_F_sra_rst_MASK);

	/* Wait for it to auto-clear */
	for (i = SRAB_MAX_RETRY * 10; i > 0; i --) {
		val32 = R_REG(si_osh(robo->sih), &robo->srabregs->cmdstat);
		if ((val32 & CFG_F_sra_rst_MASK) == 0)
			break;
	}

	/* timed out */
	if (!i) {
		ET_ERROR(("srab_interface_reset: timeout sra_rst"));
		ret |= -2;
	}

	return ret;
}

static int
srab_wreg(robo_info_t *robo, uint8 page, uint8 reg, void *val, int len)
{
	uint16 val16;
	uint32 val32;
	uint32 val_h = 0, val_l = 0;
	int i, ret = 0;
	uint8 *ptr = (uint8 *)val;

	/* validate value length and buffer address */
	ASSERT(len == 1 || len == 6 || len == 8 ||
	       ((len == 2) && !((int)val & 1)) || ((len == 4) && !((int)val & 3)));

	ET_MSG(("%s: [0x%x-0x%x] := 0x%x (len %d)\n", __FUNCTION__, page, reg,
	       VARG(val, len), len));

	srab_request_grant(robo);

	/* Load the value to write */
	switch (len) {
	case 8:
		val16 = ptr[7];
		val16 = ((val16 << 8) | ptr[6]);
		val_h = val16 << 16;
		/* FALLTHRU */

	case 6:
		val16 = ptr[5];
		val16 = ((val16 << 8) | ptr[4]);
		val_h |= val16;

		val16 = ptr[3];
		val16 = ((val16 << 8) | ptr[2]);
		val_l = val16 << 16;
		val16 = ptr[1];
		val16 = ((val16 << 8) | ptr[0]);
		val_l |= val16;
		break;

	case 4:
		val_l = *(uint32 *)val;
		break;

	case 2:
		val_l = *(uint16 *)val;
		break;

	case 1:
		val_l = *(uint8 *)val;
		break;
	}
	W_REG(si_osh(robo->sih), &robo->srabregs->wd_h, val_h);
	W_REG(si_osh(robo->sih), &robo->srabregs->wd_l, val_l);

	/* We don't need this variable */
	if (robo->page != page)
		robo->page = page;

	/* Issue the write command */
	val32 = ((page << CFG_F_sra_page_R)
		| (reg << CFG_F_sra_offset_R)
		| CFG_F_sra_gordyn_MASK
		| CFG_F_sra_write_MASK);
	W_REG(si_osh(robo->sih), &robo->srabregs->cmdstat, val32);

	/* Wait for command complete */
	for (i = SRAB_MAX_RETRY; i > 0; i --) {
		val32 = R_REG(si_osh(robo->sih), &robo->srabregs->cmdstat);
		if ((val32 & CFG_F_sra_gordyn_MASK) == 0)
			break;
	}

	/* timed out */
	if (!i) {
		ET_ERROR(("srab_wreg: timeout"));
		srab_interface_reset(robo);
		ret = -1;
	}

	srab_release_grant(robo);

	return ret;
}

static int
srab_rreg(robo_info_t *robo, uint8 page, uint8 reg, void *val, int len)
{
	uint32 val32;
	uint32 val_h = 0, val_l = 0;
	int i, ret = 0;
	uint8 *ptr = (uint8 *)val;

	/* validate value length and buffer address */
	ASSERT(len == 1 || len == 6 || len == 8 ||
	       ((len == 2) && !((int)val & 1)) || ((len == 4) && !((int)val & 3)));

	srab_request_grant(robo);

	/* We don't need this variable */
	if (robo->page != page)
		robo->page = page;

	/* Assemble read command */
	srab_request_grant(robo);

	val32 = ((page << CFG_F_sra_page_R)
		| (reg << CFG_F_sra_offset_R)
		| CFG_F_sra_gordyn_MASK);
	W_REG(si_osh(robo->sih), &robo->srabregs->cmdstat, val32);

	/* is operation finished? */
	for (i = SRAB_MAX_RETRY; i > 0; i --) {
		val32 = R_REG(si_osh(robo->sih), &robo->srabregs->cmdstat);
		if ((val32 & CFG_F_sra_gordyn_MASK) == 0)
			break;
	}

	/* timed out */
	if (!i) {
		ET_ERROR(("srab_read: timeout"));
		srab_interface_reset(robo);
		ret = -1;
		goto err;
	}

	/* Didn't time out, read and return the value */
	val_h = R_REG(si_osh(robo->sih), &robo->srabregs->rd_h);
	val_l = R_REG(si_osh(robo->sih), &robo->srabregs->rd_l);

	switch (len) {
	case 8:
		ptr[7] = (val_h >> 24);
		ptr[6] = ((val_h >> 16) & 0xff);
		/* FALLTHRU */

	case 6:
		ptr[5] = ((val_h >> 8) & 0xff);
		ptr[4] = (val_h & 0xff);
		ptr[3] = (val_l >> 24);
		ptr[2] = ((val_l >> 16) & 0xff);
		ptr[1] = ((val_l >> 8) & 0xff);
		ptr[0] = (val_l & 0xff);
		break;

	case 4:
		*(uint32 *)val = val_l;
		break;

	case 2:
		*(uint16 *)val = (uint16)(val_l & 0xffff);
		break;

	case 1:
		*(uint8 *)val = (uint8)(val_l & 0xff);
		break;
	}

	ET_MSG(("%s: [0x%x-0x%x] => 0x%x (len %d)\n", __FUNCTION__, page, reg,
	       VARG(val, len), len));

err:
	srab_release_grant(robo);

	return ret;
}

/* SRAB interface functions */
static dev_ops_t srab = {
	NULL,
	NULL,
	srab_wreg,
	srab_rreg,
	"SRAB"
};

#if defined(CONFIG_MACH_NSP)
void
srab_sgmii_set_blk(robo_info_t *robo, uint page, uint blk)
{
	uint16 blkaddr;
	uint16 destblk = (uint16)blk;

	/* printf("%s page(0x%x) blk(0x%x)\n", __FUNCTION__, page, blk); */
	/* check if need to update blk addr */
	robo->ops->read_reg(robo, page, REG_SGMII_BLK_ADDR, &blkaddr, sizeof(blkaddr));
	if (blkaddr!=destblk) {
		/* write block address */
		robo->ops->write_reg(robo, page, REG_SGMII_BLK_ADDR, &destblk, sizeof(destblk));
	}
}

int
srab_sgmii_rreg(robo_info_t *robo, uint8 page, uint16 reg, uint16 *val)
{
	uint blk = reg&0xfff0;
	uint8 off = reg&0x000f;
	uint16 data;

	if (reg&0x8000)
		off|=0x10;

	/* spi offset is only even (multiple of 2) */
	off = off*2;

	/* check block addr */
	srab_sgmii_set_blk(robo, page, blk);

	/* read offset register */
	robo->ops->read_reg(robo, page, off, &data, sizeof(data));
	//printf("%s page(0x%x) blk(0x%x) offset(0x%x) value(0x%x)\n", __FUNCTION__, page, blk, off, data);
	*val = data;

	return 0;
}

int
srab_sgmii_wreg(robo_info_t *robo, uint8 page, uint16 reg, uint16 *val)
{
	uint blk = reg&0xfff0;
	uint8 off = reg&0x000f;
	uint16 data=*val;

	if (reg&0x8000)
		off|=0x10;

	/* spi offset is only even (multiple of 2) */
	off = off*2;

	/* check block addr */
	srab_sgmii_set_blk(robo, page, blk);

	/* write offset register */
	robo->ops->write_reg(robo, page, off, &data, sizeof(data));
	//printf("%s page(0x%x) blk(0x%x) offset(0x%x) value(0x%x)\n", __FUNCTION__, page, blk, off, data);

	return 0;
}
#endif /* defined(CONFIG_MACH_NSP) */

/* High level switch configuration functions. */

/* Get access to the RoboSwitch */
robo_info_t *
bcm_robo_attach(si_t *sih, void *h, char *vars, miird_f miird, miiwr_f miiwr)
{
	robo_info_t *robo;
	uint32 reset, idx;
#ifndef	_CFE_
//	const char *et1port, *et1phyaddr;
	int mdcport = 0, phyaddr = 0;
#endif /* _CFE_ */
	int lan_portenable = 0;

	/* Allocate and init private state */
	if (!(robo = MALLOC(si_osh(sih), sizeof(robo_info_t)))) {
		ET_ERROR(("robo_attach: out of memory, malloced %d bytes",
		          MALLOCED(si_osh(sih))));
		return NULL;
	}
	bzero(robo, sizeof(robo_info_t));

	robo->h = h;
	robo->sih = sih;
	robo->vars = vars;
	robo->miird = miird;
	robo->miiwr = miiwr;
	robo->page = -1;

	if (IS_BCM5301X_CHIP_ID(sih->chip)) {
		robo->miird = NULL;
		robo->miiwr = NULL;
		robo->srabregs = (srabregs_t *)REG_MAP(SI_NS_CHIPCB_SRAB, SI_CORE_SIZE);
	}

	/* Enable center tap voltage for LAN ports using gpio23. Usefull in case when
	 * romboot CFE loads linux over WAN port and Linux enables LAN ports later
	 */
	if ((lan_portenable = getgpiopin(robo->vars, "lanports_enable", GPIO_PIN_NOTDEFINED)) !=
	    GPIO_PIN_NOTDEFINED) {
		lan_portenable = 1 << lan_portenable;
		si_gpioouten(sih, lan_portenable, lan_portenable, GPIO_DRV_PRIORITY);
		si_gpioout(sih, lan_portenable, lan_portenable, GPIO_DRV_PRIORITY);
		bcm_mdelay(5);
	}

	/* Trigger external reset by nvram variable existance */
	if ((reset = getgpiopin(robo->vars, "robo_reset", GPIO_PIN_NOTDEFINED)) !=
	    GPIO_PIN_NOTDEFINED) {
		/*
		 * Reset sequence: RESET low(50ms)->high(20ms)
		 *
		 * We have to perform a full sequence for we don't know how long
		 * it has been from power on till now.
		 */
		ET_MSG(("%s: Using external reset in gpio pin %d\n", __FUNCTION__, reset));
		reset = 1 << reset;

		/* Keep RESET low for 50 ms */
		si_gpioout(sih, reset, 0, GPIO_DRV_PRIORITY);
		si_gpioouten(sih, reset, reset, GPIO_DRV_PRIORITY);
		bcm_mdelay(50);

		/* Keep RESET high for at least 20 ms */
		si_gpioout(sih, reset, reset, GPIO_DRV_PRIORITY);
		bcm_mdelay(20);
	} else {
		/* In case we need it */
		idx = si_coreidx(sih);

		if (si_setcore(sih, ROBO_CORE_ID, 0)) {
			/* If we have an internal robo core, reset it using si_core_reset */
			ET_MSG(("%s: Resetting internal robo core\n", __FUNCTION__));
			si_core_reset(sih, 0, 0);
			robo->corerev = si_corerev(sih);
		}
		else if (IS_BCM5301X_CHIP_ID(sih->chip)) {
			srab_interface_reset(robo);
			srab_rreg(robo, PAGE_MMR, REG_VERSION_ID, &robo->corerev, 1);
		}
		else {
			ET_ERROR(("%s: unknown switch\n", __FUNCTION__));
		}
		si_setcoreidx(sih, idx);
		ET_MSG(("%s: Internal robo rev %d\n", __FUNCTION__, robo->corerev));
	}

	if (IS_BCM5301X_CHIP_ID(sih->chip)) {
		int rc;

		rc = srab_rreg(robo, PAGE_MMR, REG_DEVICE_ID, &robo->devid32, sizeof(uint32));

		ET_MSG(("%s: devid read %ssuccesfully via srab: 0x%x\n",
			__FUNCTION__, rc ? "un" : "", robo->devid32));

		robo->ops = &srab;
		if ((rc != 0) || (robo->devid32 == 0)) {
			ET_ERROR(("%s: error reading devid\n", __FUNCTION__));
			MFREE(si_osh(robo->sih), robo, sizeof(robo_info_t));
			return NULL;
		}
		ET_MSG(("%s: devid32: 0x%x\n", __FUNCTION__, robo->devid32));
		printf("%s: devid32: 0x%x\n", __FUNCTION__, robo->devid32);
	}

#ifndef	_CFE_
	if (!robo->ops) {
		ET_ERROR(("%s: unknown switch", __FUNCTION__));
		goto error;
	}
#endif /* _CFE_ */

	/* sanity check */
	ASSERT(robo->ops);
	ASSERT(robo->ops->write_reg);
	ASSERT(robo->ops->read_reg);
	ASSERT(ROBO_IS_BCM5301X(robo->devid32));

#ifndef	_CFE_
	/* nvram variable switch_mode controls the power save mode on the switch
	 * set the default value in the beginning
	 */
	robo->pwrsave_mode_manual = getintvar(robo->vars, "switch_mode_manual");
	robo->pwrsave_mode_auto = getintvar(robo->vars, "switch_mode_auto");

	/* Determining what all phys need to be included in
	 * power save operation
	 */
	//et1port = getvar(vars, "et1mdcport");
	//if (et1port)
	//	mdcport = bcm_atoi(et1port);

	//et1phyaddr = getvar(vars, "et1phyaddr");
	//if (et1phyaddr)
	//	phyaddr = bcm_atoi(et1phyaddr);

	if ((mdcport == 0) && (phyaddr == 4))
		/* For 5325F switch we need to do only phys 0-3 */
		robo->pwrsave_phys = 0xf;
	else
		/* By default all 5 phys are put into power save if there is no link */
		robo->pwrsave_phys = 0x1f;
#endif /* _CFE_ */

	return robo;

#ifndef	_CFE_
error:
	bcm_robo_detach(robo);
	return NULL;
#endif /* _CFE_ */
}

/* Release access to the RoboSwitch */
void
bcm_robo_detach(robo_info_t *robo)
{
	if (robo->srabregs)
		REG_UNMAP(robo->srabregs);

	MFREE(si_osh(robo->sih), robo, sizeof(robo_info_t));
}

/* Enable the device and set it to a known good state */
int
bcm_robo_enable_device(robo_info_t *robo)
{
	int ret = 0;

	/* Enable management interface access */
	if (robo->ops->enable_mgmtif)
		robo->ops->enable_mgmtif(robo);

	/* Disable management interface access */
	if (robo->ops->disable_mgmtif)
		robo->ops->disable_mgmtif(robo);

	return ret;
}

/* Port flags */
#define FLAG_TAGGED	't'	/* output tagged (external ports only) */
#define FLAG_UNTAG	'u'	/* input & output untagged (CPU port only, for OS (linux, ...) */
#define FLAG_LAN	'*'	/* input & output untagged (CPU port only, for CFE */

/* port descriptor */
typedef	struct {
	uint32 untag;	/* untag enable bit (Page 0x05 Address 0x63-0x66 Bit[17:9]) */
	uint32 member;	/* vlan member bit (Page 0x05 Address 0x63-0x66 Bit[7:0]) */
	uint8 ptagr;	/* port tag register address (Page 0x34 Address 0x10-0x1F) */
	uint8 cpu;	/* is this cpu port? */
} pdesc_t;

pdesc_t pdesc97[] = {
	/* 5395/5397/5398/53115S is 0 ~ 7.  port 8 is IMP port. */
	/* port 0 */ {1 << 9, 1 << 0, REG_VLAN_PTAG0, 0},
	/* port 1 */ {1 << 10, 1 << 1, REG_VLAN_PTAG1, 0},
	/* port 2 */ {1 << 11, 1 << 2, REG_VLAN_PTAG2, 0},
	/* port 3 */ {1 << 12, 1 << 3, REG_VLAN_PTAG3, 0},
	/* port 4 */ {1 << 13, 1 << 4, REG_VLAN_PTAG4, 0},
#ifdef GMAC3
	/* port 5 */ {1 << 14, 1 << 5, REG_VLAN_PTAG5, 0},
	/* port 6 */ {1 << 15, 1 << 6, REG_VLAN_PTAG6, 0},
	/* port 7 */ {1 << 16, 1 << 7, REG_VLAN_PTAG7, 0},
#else /* !GMAC3 */
	/* port 5 */ {1 << 14, 1 << 5, REG_VLAN_PTAG5, 1},
	/* port 6 */ {1 << 15, 1 << 6, REG_VLAN_PTAG6, 1},
	/* port 7 */ {1 << 16, 1 << 7, REG_VLAN_PTAG7, 1},
#endif /* !GMAC3 */
	/* mii port */ {1 << 17, 1 << 8, REG_VLAN_PTAG8, 1},
};

/* Configure the VLANs */
int
bcm_robo_config_vlan(robo_info_t *robo, uint8 *mac_addr)
{
	uint8 val8;
	uint16 val16;
	uint32 val32;
	pdesc_t *pdesc;
	int pdescsz;
	uint16 vid;
	uint8 arl_entry[8] = { 0 };

	/* Enable management interface access */
	if (robo->ops->enable_mgmtif)
		robo->ops->enable_mgmtif(robo);

	/* setup global vlan configuration */
	/* VLAN Control 0 Register (Page 0x34, Address 0) */
	robo->ops->read_reg(robo, PAGE_VLAN, REG_VLAN_CTRL0, &val8, sizeof(val8));
	val8 |= ((1 << 7) |		/* enable 802.1Q VLAN */
	         (3 << 5));		/* individual VLAN learning mode */
	robo->ops->write_reg(robo, PAGE_VLAN, REG_VLAN_CTRL0, &val8, sizeof(val8));
	/* VLAN Control 1 Register (Page 0x34, Address 1) */
	robo->ops->read_reg(robo, PAGE_VLAN, REG_VLAN_CTRL1, &val8, sizeof(val8));
	val8 |= ((1 << 2) |		/* enable RSV multicast V Fwdmap */
		 (1 << 3));		/* enable RSV multicast V Untagmap */
	robo->ops->write_reg(robo, PAGE_VLAN, REG_VLAN_CTRL1, &val8, sizeof(val8));

	arl_entry[0] = mac_addr[5];
	arl_entry[1] = mac_addr[4];
	arl_entry[2] = mac_addr[3];
	arl_entry[3] = mac_addr[2];
	arl_entry[4] = mac_addr[1];
	arl_entry[5] = mac_addr[0];

	/* Initialize the MAC Addr Index Register */
	robo->ops->write_reg(robo, PAGE_VTBL, REG_VTBL_MINDX,
	                     arl_entry, ETHER_ADDR_LEN);

	pdesc = pdesc97;
	pdescsz = sizeof(pdesc97) / sizeof(pdesc_t);

	/* check if p5 is not CPU port */
	if (!robo_is_port5_cpu()) {
		pdesc[5].cpu = 0;
	}

	/* setup each vlan. max. 16 vlans. */
	/* force vlan id to be equal to vlan number */
	for (vid = 0; vid < VLAN_NUMVLANS; vid ++) {
		char vlanports[] = "vlanXXXXports";
		char port[] = "XXXX", *next;
		const char *ports, *cur;
		uint32 untag = 0;
		uint32 member = 0;
		int pid, len;
		int cpuport=0;

		/* no members if VLAN id is out of limitation */
		if (vid > VLAN_MAXVID)
			goto vlan_setup;

		/* get vlan member ports from nvram */
		sprintf(vlanports, "vlan%dports", vid);
		ports = getvar(robo->vars, vlanports);

		ET_MSG(("%s: getvar(%s) port %s \n",
					__FUNCTION__, vlanports, ports));
		/* vid == 0 is invalid?? */
		if (vid == 0) {
			if (ports)
				ET_ERROR(("VID 0 is set in nvram, Ignoring\n"));
			continue;
		}

		/* disable this vlan if not defined */
		if (!ports)
			goto vlan_setup;

		/*
		 * setup each port in the vlan. cpu port needs special handing
		 * (with or without output tagging) to support linux/pmon/cfe.
		 */
		for (cur = ports; cur; cur = next) {
			/* tokenize the port list */
			while (*cur == ' ')
				cur ++;
			next = bcmstrstr(cur, " ");
			len = next ? next - cur : strlen(cur);
			if (!len)
				break;
			if (len > sizeof(port) - 1)
				len = sizeof(port) - 1;
			strncpy(port, cur, len);
			port[len] = 0;

			/* make sure port # is within the range */
			pid = bcm_atoi(port);
			if (pid >= pdescsz) {
				ET_ERROR(("robo_config_vlan: port %d in vlan%dports is out "
				          "of range[0-%d]\n", pid, vid, pdescsz));
				continue;
			}
			if (pid == 6) {
				ET_ERROR(("robo_config_vlan: port %d in vlan%dports is not valid\n", pid, vid));
				continue;
			}

			/* build VLAN registers values */
#ifndef	_CFE_
			if ((!pdesc[pid].cpu && !strchr(port, FLAG_TAGGED)) ||
			    (pdesc[pid].cpu && strchr(port, FLAG_UNTAG)))
#endif
				untag |= pdesc[pid].untag;

			member |= pdesc[pid].member;

			/* set port tag - applies to untagged ingress frames */
			/* Default Port Tag Register (Page 0x34, Address 0x10-0x1D) */
#ifdef	_CFE_
#define	FL	FLAG_LAN
#else
#define	FL	FLAG_UNTAG
#endif /* _CFE_ */
			if (!pdesc[pid].cpu || strchr(port, FL)) {
				val16 = ((0 << 13) |		/* priority - always 0 */
				         vid);			/* vlan id */
				robo->ops->write_reg(robo, PAGE_VLAN, pdesc[pid].ptagr,
				                     &val16, sizeof(val16));
			}
			if (pdesc[pid].cpu)
				cpuport=pid;
		}

		/* Add static ARL entries */
		/* Set the VLAN Id in VLAN ID Index Register */
		val8 = vid;
		robo->ops->write_reg(robo, PAGE_VTBL, REG_VTBL_VINDX,
		                     &val8, sizeof(val8));

		/* Set the MAC addr and VLAN Id in ARL Table MAC/VID Entry 0
		 * Register.
		 */
		arl_entry[6] = vid;
		arl_entry[7] = 0x0;
		robo->ops->write_reg(robo, PAGE_VTBL, REG_VTBL_ARL_E0,
		                     arl_entry, sizeof(arl_entry));

		/* Set the Static bit , Valid bit and Port ID fields in
		 * ARL Table Data Entry 0 Register
		 */
		//val16 = 0xc100;	//0xc020;
		val32 = 0x08000 + (1<<cpuport);
		robo->ops->write_reg(robo, PAGE_VTBL, REG_VTBL_DAT_E0,
		                     &val32, sizeof(val32));

		/* Clear the ARL_R/W bit and set the START/DONE bit in
		 * the ARL Read/Write Control Register.
		 */
		val8 = 0x80;
		robo->ops->write_reg(robo, PAGE_VTBL, REG_VTBL_CTRL,
		                     &val8, sizeof(val8));
		/* Wait for write to complete */
		SPINWAIT((robo->ops->read_reg(robo, PAGE_VTBL, REG_VTBL_CTRL,
		         &val8, sizeof(val8)), ((val8 & 0x80) != 0)),
		         100 /* usec */);

vlan_setup:
		/* setup VLAN ID and VLAN memberships */

		val32 = (untag |			/* untag enable */
		         member);			/* vlan members */
		{
			uint8 vtble, vtbli, vtbla;

			vtble = REG_VTBL_ENTRY_5395;
			vtbli = REG_VTBL_INDX_5395;
			vtbla = REG_VTBL_ACCESS_5395;

			/* VLAN Table Entry Register (Page 0x05, Address 0x63-0x66/0x83-0x86) */
			robo->ops->write_reg(robo, PAGE_VTBL, vtble, &val32,
			                     sizeof(val32));
			/* VLAN Table Address Index Reg (Page 0x05, Address 0x61-0x62/0x81-0x82) */
			val16 = vid;        /* vlan id */
			robo->ops->write_reg(robo, PAGE_VTBL, vtbli, &val16,
			                     sizeof(val16));

			/* VLAN Table Access Register (Page 0x34, Address 0x60/0x80) */
			val8 = ((1 << 7) | 	/* start command */
			        0);	        /* write */
			robo->ops->write_reg(robo, PAGE_VTBL, vtbla, &val8,
			                     sizeof(val8));
		}
	}

	/* Disable management interface access */
	if (robo->ops->disable_mgmtif)
		robo->ops->disable_mgmtif(robo);

	return 0;
}

/* Enable switching/forwarding */
int
bcm_robo_enable_switch(robo_info_t *robo)
{
	int i, max_port_ind, ret = 0;
	uint8 val8;
	uint16 val16;
	bool bcm_tag_on=false;
#if (defined(CONFIG_IPROC_FA) || defined(CONFIG_IPROC_FA2))
	char *var;
#endif /* (defined(CONFIG_IPROC_FA) || defined(CONFIG_IPROC_FA2)) */
	uint32_t	val32;
	void		*oh;
	uint32_t	skuid=0;

#if (defined(CONFIG_IPROC_FA) || defined(CONFIG_IPROC_FA2))
	/* check if brcm tag is turned off */
	bcm_tag_on=true;
	var = getvar(NULL, "brcmtag");
	if (var) {
		int tag = bcm_strtoul(var, NULL, 0);
		if (tag==0) {
			ET_ERROR(("BRCM TAG disabled\n"));
			/* if brcm tag == 0 tag disabled */
			bcm_tag_on = false;
		}
	}
#elif defined(CONFIG_MACH_NSP)
	bcm_tag_on=true;
#endif /* (defined(CONFIG_IPROC_FA) || defined(CONFIG_IPROC_FA2)) */

	/* Enable management interface access */
	if (robo->ops->enable_mgmtif)
		robo->ops->enable_mgmtif(robo);

	/* Switch Mode register (Page 0, Address 0x0B) */
	robo->ops->read_reg(robo, PAGE_CTRL, REG_CTRL_MODE, &val8, sizeof(val8));

	/* Bit 1 enables switching/forwarding */
	if (!(val8 & (1 << 1))) {
		/* Set unmanaged mode */
		val8 &= (~(1 << 0));

		/* Enable forwarding */
		val8 |= (1 << 1);
		robo->ops->write_reg(robo, PAGE_CTRL, REG_CTRL_MODE, &val8, sizeof(val8));

		/* Read back */
		robo->ops->read_reg(robo, PAGE_CTRL, REG_CTRL_MODE, &val8, sizeof(val8));
		if (!(val8 & (1 << 1))) {
			ET_ERROR(("robo_enable_switch: enabling forwarding failed\n"));
			ret = -1;
		}

		/* No spanning tree for unmanaged mode */
		val8 = 0;
		if (ROBO_IS_BCM5301X(robo->devid32))
			max_port_ind = REG_CTRL_PORT7;
		else
			max_port_ind = REG_CTRL_PORT4;

		for (i = REG_CTRL_PORT0; i <= max_port_ind; i++) {
			if (ROBO_IS_BCM5301X(robo->devid32) && i == REG_CTRL_PORT6)
				continue;
			robo->ops->write_reg(robo, PAGE_CTRL, i, &val8, sizeof(val8));
		}

		/* No spanning tree on IMP port too */
		robo->ops->write_reg(robo, PAGE_CTRL, REG_CTRL_IMP, &val8, sizeof(val8));
	}
	else {
		/* Set managed mode */
		val8 |= 1;
		robo->ops->write_reg(robo, PAGE_CTRL, REG_CTRL_MODE, &val8, sizeof(val8));
	}

	if (ROBO_IS_BCM5301X(robo->devid32)) {
		/*
		 * Port N GMII Port States Override Register (Page 0x00 , address Offset: 0x0e , 0x58-0x5d and 0x5f )
		 * SPEED/ DUPLEX_MODE/ LINK_STS
		 */

		/* check if p5 is CPU port */
		if (robo_is_port5_cpu()) {
			/* Over ride GMAC0 Port5 status to make it link by default */
			val8 = 0;
			robo->ops->read_reg(robo, PAGE_CTRL, REG_CTRL_PORT5_GMIIPO, &val8, sizeof(val8));
			/* 2G_ENABLED: */
			val8 |= 0xf1;	/* Make Link pass and override it. */
			robo->ops->write_reg(robo, PAGE_CTRL, REG_CTRL_PORT5_GMIIPO, &val8, sizeof(val8));
		}

		/* Over ride GMAC1 Port7 status to make it link by default */
		val8 = 0;
		robo->ops->read_reg(robo, PAGE_CTRL, REG_CTRL_PORT7_GMIIPO, &val8, sizeof(val8));
		/* 2G_ENABLED: */
		val8 |= 0xf1;	/* Make Link pass and override it. */
		robo->ops->write_reg(robo, PAGE_CTRL, REG_CTRL_PORT7_GMIIPO, &val8, sizeof(val8));

		/* Over ride GMAC2 IMP(Port8) status to make it link by default */
		val8 = 0;
		robo->ops->read_reg(robo, PAGE_CTRL, REG_CTRL_MIIPO, &val8, sizeof(val8));
		/* 2G_ENABLED:
		* Page :0x00
		* ( Offset: 0xe ) IMP Port States Override Register
		* [6]: GMII SPEED UP 2G
		*/
		val8 |= 0xf1;	/* Make Link pass and override it. */
		robo->ops->write_reg(robo, PAGE_CTRL, REG_CTRL_MIIPO, &val8, sizeof(val8));

		/* GMAC2 IMP(Port8) config BRCM tag */
		val8 = 0;
		robo->ops->read_reg(robo, PAGE_MMR, REG_BRCM_HDR, &val8, sizeof(val8));
		if (bcm_tag_on) 
			val8 |= 0x01;
		else
			val8 &= 0xfe;
		robo->ops->write_reg(robo, PAGE_MMR, REG_BRCM_HDR, &val8, sizeof(val8));

		/* GMAC2 IMP(Port8) Enable receive all packets */
		val8 = 0;
		robo->ops->read_reg(robo, PAGE_CTRL, REG_CTRL_IMP, &val8, sizeof(val8));
		val8 |= 0x1c; 
		robo->ops->write_reg(robo, PAGE_CTRL, REG_CTRL_IMP, &val8, sizeof(val8));

		/* GMAC2 IMP(Port8) IMP port Enable */
		val8 = 0;
		robo->ops->read_reg(robo, PAGE_MMR, REG_MGMT_CFG, &val8, sizeof(val8));
		val8 |= 0x80; 
		robo->ops->write_reg(robo, PAGE_MMR, REG_MGMT_CFG, &val8, sizeof(val8));
	}

	if (bcm_tag_on) {
		/* GMAC2 IMP(Port8) enable ignore crc check */
		val8 = 0;
		robo->ops->read_reg(robo, PAGE_VLAN, REG_VLAN_CTRL5, &val8, sizeof(val8));
		val8 |= 0x01;
		robo->ops->write_reg(robo, PAGE_VLAN, REG_VLAN_CTRL5, &val8, sizeof(val8));
	}

	/* Disable management interface access */
	if (robo->ops->disable_mgmtif)
		robo->ops->disable_mgmtif(robo);

	/* make sure external ports are not in protected mode (Page 0, Address 0x24) */
	val16 = 0;
	robo->ops->write_reg(robo, PAGE_CTRL, REG_CTRL_PPORT, &val16, sizeof(val16));


	/* check if need to turn off unused ports */
	/* Check for Vega chip - get OTP skuid */
	bcm5301x_otp_init_fptr = symbol_get(bcm5301x_otp_init);
	bcm5301x_otp_read_dword_fptr = symbol_get(bcm5301x_otp_read_dword);
	bcm5301x_otp_exit_fptr = symbol_get(bcm5301x_otp_exit);


	if ( (bcm5301x_otp_init_fptr != NULL)  
		&& ( bcm5301x_otp_read_dword_fptr != NULL) 
			&& (bcm5301x_otp_exit_fptr != NULL))	 {

		oh = (*bcm5301x_otp_init_fptr)();
		(*bcm5301x_otp_read_dword_fptr)(oh, 0x0f, &skuid);
		(*bcm5301x_otp_exit_fptr)();
	}

	printf("%s OTP: skuid 0x%x\n", __FUNCTION__, skuid);
	if (IS_BCM5301X_CHIP_ID(sih->chip)) {
		printf("%s Northstar Family chip\n", __FUNCTION__);
		if (    (robo->devid32==DEVID53010 && skuid==OTP_SKU_ID_53014)
			 || (robo->devid32==DEVID53011 && skuid==OTP_SKU_ID_53015)
			 || (robo->devid32==DEVID53012 && skuid==OTP_SKU_ID_53016) ) {
			/* check for VEGA */
			printf("%s Vega chip\n", __FUNCTION__);
			/* only have ports 0-1, power down phy of ports 2-4 */
			robo->ops->read_reg(robo, PAGE_CTRL, REG_CTRL_PHY_PWR, &val32, sizeof(val32));
			val32 |= 0x1c;	/* power down ports 2-4. */
			robo->ops->write_reg(robo, PAGE_CTRL, REG_CTRL_PHY_PWR, &val32, sizeof(val32));
		}
		else if ( robo->devid32 == DEVID53025 ) {
			printf("%s Checking powered down port\n", __FUNCTION__);
			if (skuid == 0x1c) {
				printf("%s Powering down port 2-4\n", __FUNCTION__);
				/* only have ports 0-1, power down phy of ports 2-4 */
				robo->ops->read_reg(robo, PAGE_CTRL, REG_CTRL_PHY_PWR, &val32, sizeof(val32));
				val32 |= 0x1c;	/* power down ports 2-4. */
				robo->ops->write_reg(robo, PAGE_CTRL, REG_CTRL_PHY_PWR, &val32, sizeof(val32));
			} else if (skuid == 0x07) {
				printf("%s Powering down port 0-2\n", __FUNCTION__);
				/* only have ports 3-4, power down phy of ports 0-2 */
				robo->ops->read_reg(robo, PAGE_CTRL, REG_CTRL_PHY_PWR, &val32, sizeof(val32));
				val32 |= 0x7;	/* power down ports 0-2. */
				robo->ops->write_reg(robo, PAGE_CTRL, REG_CTRL_PHY_PWR, &val32, sizeof(val32));
			}
		}
	}

#if defined(CONFIG_MACH_NSP)
	if ( robo_is_port_cfg(PORTCFG_5, PORTCFG_SGMII)
		 	|| robo_is_port_cfg(PORTCFG_4, PORTCFG_SGMII) ) {
		/* either port5 or port4 SGMII enabled */
		/* enable serdes */
		robo_serdes_reset_core(robo, PAGE_P5_SGMII);
		if (robo_is_port_cfg(PORTCFG_5, PORTCFG_SGMII)) {
			/* enable port5 sgmii */
			robo_serdes_init(robo, PAGE_P5_SGMII);
		}
		if (robo_is_port_cfg(PORTCFG_4, PORTCFG_SGMII)) {
			/* enable port4 sgmii */
			robo_serdes_init(robo, PAGE_P4_SGMII);
		}
		/* start serdes pll */
		robo_serdes_start_pll(robo, PAGE_P5_SGMII);
	}
#endif /* defined(CONFIG_MACH_NSP) */

	return ret;
}

void
robo_reset_mib(robo_info_t *robo)
{
	uint8 val8;

	robo->ops->read_reg(robo, PAGE_MMR, REG_MGMT_CFG, &val8, sizeof(val8));
	/* set clear mib bit */
	val8 |= 0x01;
	robo->ops->write_reg(robo, PAGE_MMR, REG_MGMT_CFG, &val8, sizeof(val8));
	/* clear clear mib bit */
	val8 &= 0xfe;
	robo->ops->write_reg(robo, PAGE_MMR, REG_MGMT_CFG, &val8, sizeof(val8));
}

void
robo_dump_mib(robo_info_t *robo)
{
	uint32 tx32, rx32;
	int port;

	for (port=0x20; port<=0x28; port++) {
		if (port==0x26)
			continue;
		robo->ops->read_reg(robo, port, 0x00, &tx32, sizeof(tx32));
		robo->ops->read_reg(robo, port, 0x50, &rx32, sizeof(rx32));
		printf("port%d: TX Octets: 0x%x; RX Octets: 0x%x\n", port-0x20, tx32, rx32);
	}
}

void
robo_bprintf_mib(robo_info_t *robo, struct bcmstrbuf *b)
{
	uint32 tx32, txdrp32, txbcst32, txmcst32, txcol32, ucst32;
	uint32 rx32, rxusz32, rxosz32, rxale32, rxfcs32, rxdrp32, rxsachg32, rxfrag32, rxsym32, irec32, orec32, rxdis32;
	int port;

	for (port=0x20; port<=0x28; port++) {
		if (port==0x26)
			continue;
		robo->ops->read_reg(robo, port, 0x00, &tx32, sizeof(uint32));
		robo->ops->read_reg(robo, port, 0x08, &txdrp32, sizeof(uint32));
		robo->ops->read_reg(robo, port, 0x10, &txbcst32, sizeof(uint32));
		robo->ops->read_reg(robo, port, 0x14, &txmcst32, sizeof(uint32));
		robo->ops->read_reg(robo, port, 0x18, &ucst32, sizeof(uint32));
		robo->ops->read_reg(robo, port, 0x1c, &txcol32, sizeof(uint32));
		bcm_bprintf(b, "port%d TX: Octs(%x); Drp(%x) Bcst(%x) Mcst(%x) Ucst(%x) Col(%x)\n",
						port-0x20, tx32, txdrp32, txbcst32, txmcst32, ucst32, txcol32);
		robo->ops->read_reg(robo, port, 0x50, &rx32, sizeof(uint32));
		robo->ops->read_reg(robo, port, 0x58, &rxusz32, sizeof(uint32));
		robo->ops->read_reg(robo, port, 0x78, &rxosz32, sizeof(uint32));
		robo->ops->read_reg(robo, port, 0x80, &rxale32, sizeof(uint32));
		robo->ops->read_reg(robo, port, 0x84, &rxfcs32, sizeof(uint32));
		robo->ops->read_reg(robo, port, 0x90, &rxdrp32, sizeof(uint32));
		robo->ops->read_reg(robo, port, 0x94, &ucst32, sizeof(uint32));
		robo->ops->read_reg(robo, port, 0xa0, &rxsachg32, sizeof(uint32));
		robo->ops->read_reg(robo, port, 0xa4, &rxfrag32, sizeof(uint32));
		robo->ops->read_reg(robo, port, 0xac, &rxsym32, sizeof(uint32));
		robo->ops->read_reg(robo, port, 0xb0, &irec32, sizeof(uint32));
		robo->ops->read_reg(robo, port, 0xb4, &orec32, sizeof(uint32));
		robo->ops->read_reg(robo, port, 0xc0, &rxdis32, sizeof(uint32));
		bcm_bprintf(b, "port%d RX: Octs(%x); USz(%x) OSz(%x) AlgnEr(%x) FcsEr(%x) Drp(%x) Ucst(%x); SacCh(%x); Frag(%x) SymEr(%x) IRngEr(%x) ORngEr(%x) Dis(%x)\n",
						port-0x20, rx32, rxusz32, rxosz32, rxale32, rxfcs32, rxdrp32,
						ucst32, rxsachg32, rxfrag32, rxsym32, irec32, orec32, rxdis32);
	}
	robo_reset_mib(robo);
}


void
robo_dump_regs(robo_info_t *robo, struct bcmstrbuf *b)
{
	uint8 val8;
	uint16 val16;
	uint32 val32;
	pdesc_t *pdesc;
	int pdescsz;
	int i;

	bcm_bprintf(b, "%s:\n", robo->ops->desc);
	if (robo->miird == NULL && !strcmp(robo->ops->desc, "SPI (GPIO)"))
		bcm_bprintf(b, "SPI gpio pins: ss %d sck %d mosi %d miso %d\n",
		            robo->ss, robo->sck, robo->mosi, robo->miso);

	/* Enable management interface access */
	if (robo->ops->enable_mgmtif)
		robo->ops->enable_mgmtif(robo);

	/* Dump registers interested */
	robo->ops->read_reg(robo, PAGE_CTRL, REG_CTRL_MODE, &val8, sizeof(val8));
	bcm_bprintf(b, "(0x00,0x0B)Switch mode regsiter: 0x%02x\n", val8);

	pdesc = pdesc97;
	pdescsz = sizeof(pdesc97) / sizeof(pdesc_t);

	robo->ops->read_reg(robo, PAGE_VLAN, REG_VLAN_CTRL0, &val8, sizeof(val8));
	bcm_bprintf(b, "(0x34,0x00)VLAN control 0 register: 0x%02x\n", val8);
	robo->ops->read_reg(robo, PAGE_VLAN, REG_VLAN_CTRL1, &val8, sizeof(val8));
	bcm_bprintf(b, "(0x34,0x01)VLAN control 1 register: 0x%02x\n", val8);
	robo->ops->read_reg(robo, PAGE_VLAN, REG_VLAN_CTRL4, &val8, sizeof(val8));
	{
		uint8 vtble, vtbli, vtbla;

		vtble = REG_VTBL_ENTRY_5395;
		vtbli = REG_VTBL_INDX_5395;
		vtbla = REG_VTBL_ACCESS_5395;

		for (i = 0; i <= VLAN_MAXVID; i++) {
			/* VLAN Table Address Index Register (Page 0x05, Address 0x61-0x62/0x81-0x82) */
			val16 = i;		/* vlan id */
			robo->ops->write_reg(robo, PAGE_VTBL, vtbli, &val16,
			                     sizeof(val16));
			/* VLAN Table Access Register (Page 0x34, Address 0x60/0x80) */
			val8 = ((1 << 7) | 	/* start command */
			        1);		/* read */
			robo->ops->write_reg(robo, PAGE_VTBL, vtbla, &val8,
			                     sizeof(val8));
			/* VLAN Table Entry Register (Page 0x05, Address 0x63-0x66/0x83-0x86) */
			robo->ops->read_reg(robo, PAGE_VTBL, vtble, &val32,
			                    sizeof(val32));
			bcm_bprintf(b, "VLAN %d untag bits: 0x%02x member bits: 0x%02x\n",
			            i, (val32 & 0x3fe00) >> 9, (val32 & 0x1ff));
		}
	}
	for (i = 0; i < pdescsz; i++) {
		robo->ops->read_reg(robo, PAGE_VLAN, pdesc[i].ptagr, &val16, sizeof(val16));
		bcm_bprintf(b, "(0x34,0x%02x)Port %d Tag: 0x%04x\n", pdesc[i].ptagr, i, val16);
	}

	/* Disable management interface access */
	if (robo->ops->disable_mgmtif)
		robo->ops->disable_mgmtif(robo);
}

#ifndef	_CFE_
/*
 * Update the power save configuration for ports that changed link status.
 */
void
robo_power_save_mode_update(robo_info_t *robo)
{
	uint phy;

	for (phy = 0; phy < MAX_NO_PHYS; phy++) {
		if (robo->pwrsave_mode_auto & (1 << phy)) {
			ET_MSG(("%s: set port %d to auto mode\n",
				__FUNCTION__, phy));
			robo_power_save_mode(robo, ROBO_PWRSAVE_AUTO, phy);
		}
	}

	return;
}

static int32
robo_power_save_mode_clear_auto(robo_info_t *robo, int32 phy)
{
	return -1;
}

static int32
robo_power_save_mode_clear_manual(robo_info_t *robo, int32 phy)
{
	return -1;

}

/*
 * Function which periodically checks the power save mode on the switch
 */
int32
robo_power_save_toggle(robo_info_t *robo, int32 normal)
{
	int32 phy;
	uint16 link_status;


	/* read the link status of all ports */
	robo->ops->read_reg(robo, PAGE_STATUS, REG_STATUS_LINK,
		&link_status, sizeof(uint16));
	link_status &= 0x1f;

	/* Take the phys out of the manual mode first so that link status
	 * can be checked. Once out of that  mode check the link status
	 * and if any of the link is up do not put that phy into
	 * manual power save mode
	 */
	for (phy = 0; phy < MAX_NO_PHYS; phy++) {
		/* When auto+manual modes are enabled we toggle between
		 * manual and auto modes. When only manual mode is enabled
		 * we toggle between manual and normal modes. When only
		 * auto mode is enabled there is no need to do anything
		 * here since auto mode is one time config.
		 */
		if ((robo->pwrsave_phys & (1 << phy)) &&
		    (robo->pwrsave_mode_manual & (1 << phy))) {
			if (!normal) {
				/* Take the port out of the manual mode */
				robo_power_save_mode_clear_manual(robo, phy);
			} else {
				/* If the link is down put it back to manual else
				 * remain in the current state
				 */
				if (!(link_status & (1 << phy))) {
					ET_MSG(("%s: link down, set port %d to man mode\n",
						__FUNCTION__, phy));
					robo_power_save_mode(robo, ROBO_PWRSAVE_MANUAL, phy);
				}
			}
		}
	}

	return 0;
}

/*
 * Switch the ports to normal mode.
 */
static int32
robo_power_save_mode_normal(robo_info_t *robo, int32 phy)
{
	int32 error = 0;

	/* If the phy in the power save mode come out of it */
	switch (robo->pwrsave_mode_phys[phy]) {
		case ROBO_PWRSAVE_AUTO_MANUAL:
		case ROBO_PWRSAVE_AUTO:
			error = robo_power_save_mode_clear_auto(robo, phy);
			if ((error == -1) ||
			    (robo->pwrsave_mode_phys[phy] == ROBO_PWRSAVE_AUTO))
				break;

		case ROBO_PWRSAVE_MANUAL:
			error = robo_power_save_mode_clear_manual(robo, phy);
			break;

		default:
			break;
	}

	return error;
}

/*
 * Switch all the inactive ports to auto power down mode.
 */
static int32
robo_power_save_mode_auto(robo_info_t *robo, int32 phy)
{
	return -1;
}

/*
 * Switch all the inactive ports to manual power down mode.
 */
static int32
robo_power_save_mode_manual(robo_info_t *robo, int32 phy)
{
	uint16 val16;

	/* For both 5325 and 53115 the link status register is the same */
	robo->ops->read_reg(robo, PAGE_STATUS, REG_STATUS_LINK,
	                    &val16, sizeof(val16));
	if (val16 & (0x1 << phy))
		return 0;

	return -1;
}

/*
 * Set power save modes on the robo switch
 */
int32
robo_power_save_mode(robo_info_t *robo, int32 mode, int32 phy)
{
	int32 error = -1;

	if (phy > MAX_NO_PHYS) {
		ET_ERROR(("Passed parameter phy is out of range\n"));
		return -1;
	}

	/* Enable management interface access */
	if (robo->ops->enable_mgmtif)
		robo->ops->enable_mgmtif(robo);

	switch (mode) {
		case ROBO_PWRSAVE_NORMAL:
			/* If the phy in the power save mode come out of it */
			error = robo_power_save_mode_normal(robo, phy);
			break;

		case ROBO_PWRSAVE_AUTO_MANUAL:
			/* If the switch supports auto and manual power down
			 * enable both of them
			 */
		case ROBO_PWRSAVE_AUTO:
			error = robo_power_save_mode_auto(robo, phy);
			if ((error == -1) || (mode == ROBO_PWRSAVE_AUTO))
				break;

		case ROBO_PWRSAVE_MANUAL:
			error = robo_power_save_mode_manual(robo, phy);
			break;

		default:
			break;
	}

	/* Disable management interface access */
	if (robo->ops->disable_mgmtif)
		robo->ops->disable_mgmtif(robo);

	return error;
}

/*
 * Get the current power save mode of the switch ports.
 */
int32
robo_power_save_mode_get(robo_info_t *robo, int32 phy)
{
	ASSERT(robo);

	if (phy >= MAX_NO_PHYS)
		return -1;

	return robo->pwrsave_mode_phys[phy];
}

/*
 * Configure the power save mode for the switch ports.
 */
int32
robo_power_save_mode_set(robo_info_t *robo, int32 mode, int32 phy)
{
	int32 error;

	ASSERT(robo);

	if (phy >= MAX_NO_PHYS)
		return -1;

	error = robo_power_save_mode(robo, mode, phy);

	if (error)
		return error;

	if (mode == ROBO_PWRSAVE_NORMAL) {
		robo->pwrsave_mode_manual &= ~(1 << phy);
		robo->pwrsave_mode_auto &= ~(1 << phy);
	} else if (mode == ROBO_PWRSAVE_AUTO) {
		robo->pwrsave_mode_auto |= (1 << phy);
		robo->pwrsave_mode_manual &= ~(1 << phy);
		robo_power_save_mode_clear_manual(robo, phy);
	} else if (mode == ROBO_PWRSAVE_MANUAL) {
		robo->pwrsave_mode_manual |= (1 << phy);
		robo->pwrsave_mode_auto &= ~(1 << phy);
		robo_power_save_mode_clear_auto(robo, phy);
	} else {
		robo->pwrsave_mode_auto |= (1 << phy);
		robo->pwrsave_mode_manual |= (1 << phy);
	}

	return 0;
}
#endif /* _CFE_ */

void
robo_watchdog(robo_info_t *robo)
{
	return;
}

int
robo_write_reg(void *rih, unsigned char page, unsigned char reg, void *val, int len)
{
	robo_info_t *robo = (robo_info_t*)rih;
	return robo->ops->write_reg(robo, (uint8)page, (uint8)reg, val, len);
}


int
robo_read_reg(void *rih, unsigned char page, unsigned char reg, void *val, int len)
{
	robo_info_t *robo = (robo_info_t*)rih;
	return robo->ops->read_reg(robo, (uint8)page, (uint8)reg, val, len);
}


int
robo_is_port5_cpu(void)
{
	char name[16];
	char *var;

	/* get port5 config */
	sprintf(name, PORTCFG, PORTCFG_5);
	var = getvar(NULL, name);

	/* check if not CPU port */
	if (var == NULL) {
		/* if no port 5 config then CPU port */
		return 1;
	}
	/* now check if valid CONFIGURATION */
	if (strcmp(var, PORTCFG_RGMII)==0) {
		printf("%s port5 is configured as RGMII port\n", __FUNCTION__);
		return 0;
	}
	if (strcmp(var, PORTCFG_SGMII)==0) {
		printf("%s port5 is configured as SGMII port\n", __FUNCTION__);
		return 0;
	}
	if (strcmp(var, PORTCFG_GPHY)==0) {
		printf("%s port5 is configured as GPHY port\n", __FUNCTION__);
		return 0;
	}

	printf("%s port5 has UNKNOWN configuration: %s\n", __FUNCTION__, var);
	/* must be CPU port */
	return 1;
}


int
robo_is_port_cfg(int port, char *cfg)
{
	char name[16];
	char *var;

	/* get port5 config */
	sprintf(name, PORTCFG, port);
	var = getvar(NULL, name);
	if (var == NULL) {
		/* if no port config then normal port config */
		return 0;
	}

	if (strcmp(var, cfg)==0) {
		/* the port is the configuration we are looing for */
		return 1;
	}

	/* not config we are looking for */
	return 0;
}
