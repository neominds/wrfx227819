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
 * RoboSwitch setup functions
 *
 * $Id: bcmrobo.h 327582 2012-04-14 05:02:37Z $
 */

#ifndef _bcm_robo_h_
#define _bcm_robo_h_

/*
 * MODELID:
 * 0x53010: BCM53010, Select Low SKU device if SKU ID[1:0] = 01.
 * 0x53011: BCM53011, Select Middle SKU device if SKU ID[1:0] = 10.
 * 0x53012: BCM53012, Select High SKU device if SKU ID[1:0] = 00.
 * Note: The SKU ID[1:0] is loaded from OTP configuration data.
 */
#define DEVID53010	0x53010	/* 53010 */
#define DEVID53011	0x53011	/* 53011 */
#define DEVID53012	0x53012	/* 53012 */
#define DEVID53013	0x53013	/* 53013 */
#define DEVID53014	0x53014	/* 53014 */
#define DEVID53015	0x53015	/* 53015 */
#define DEVID53016	0x53016	/* 53016 */
#define DEVID53017	0x53017	/* 53017 */
#define DEVID53018	0x53018	/* 53018 */
#define DEVID53019	0x53019	/* 53019 */
#define DEVID53022	0x53022	/* 53022 */
#define DEVID53025	0x53025	/* 53025 */

#define ROBO_IS_BCM5301X(id) (1)
#define ROBO_IS_VEGA(id) ((id) >= DEVID53014 && (id) <= DEVID53017)

#define OTP_SKU_ID_53014		0x2
#define OTP_SKU_ID_53015		0x3
#define OTP_SKU_ID_53016		0x4

/* Power save duty cycle times */
#define MAX_NO_PHYS		5
#define PWRSAVE_SLEEP_TIME	12
#define PWRSAVE_WAKE_TIME	3

/* Power save modes for the switch */
#define ROBO_PWRSAVE_NORMAL 		0
#define ROBO_PWRSAVE_AUTO		1
#define ROBO_PWRSAVE_MANUAL		2
#define ROBO_PWRSAVE_AUTO_MANUAL 	3

#define ROBO_IS_PWRSAVE_MANUAL(r) ((r)->pwrsave_mode_manual)
#define ROBO_IS_PWRSAVE_AUTO(r) ((r)->pwrsave_mode_auto)

/* NorthStar SRAB interface */
/* Access switch registers through SRAB (Switch Register Access Bridge) */
#define REG_VERSION_ID		0x40
#define REG_CTRL_PORT0_GMIIPO	0x58	/* 53012: GMII Port0 Override register */
#define REG_CTRL_PORT1_GMIIPO	0x59	/* 53012: GMII Port1 Override register */
#define REG_CTRL_PORT2_GMIIPO	0x5a	/* 53012: GMII Port2 Override register */
#define REG_CTRL_PORT3_GMIIPO	0x5b	/* 53012: GMII Port3 Override register */
#define REG_CTRL_PORT4_GMIIPO	0x5c	/* 53012: GMII Port4 Override register */
#define REG_CTRL_PORT5_GMIIPO	0x5d	/* 53012: GMII Port5 Override register */
#define REG_CTRL_PORT7_GMIIPO	0x5f	/* 53012: GMII Port7 Override register */

/* Command and status register of the SRAB */
#define CFG_F_sra_rst_MASK		(1 << 2)
#define CFG_F_sra_write_MASK		(1 << 1)
#define CFG_F_sra_gordyn_MASK		(1 << 0)
#define CFG_F_sra_page_R		24
#define CFG_F_sra_offset_R		16

/* Switch interface controls */
#define CFG_F_sw_init_done_MASK		(1 << 6)
#define CFG_F_rcareq_MASK		(1 << 3)
#define CFG_F_rcagnt_MASK		(1 << 4)
 
#ifndef PAD
#define	_PADLINE(line)	pad ## line
#define	_XSTR(line)	_PADLINE(line)
#define	PAD		_XSTR(__LINE__)
#endif	/* PAD */

#define PAGE_P5_SGMII	0x16
#define PAGE_P4_SGMII	0x17

/* SGMII REGISTERS */
#define REG_SGMII_BLK_ADDR			0x3e
#define REG_IEEECTRL0				0x0000
#define REG_TX_ACTL0				0x8061
#define REG_TX_DRIVER				0x8065
#define REG_RX_CONTROL				0x80f1
#define REG_RX_ANLOGBIAS0L			0x80fc
#define REG_SERDES_CTL1000X1		0x8300
#define REG_SERDES_CTL1000X2		0x8301
#define REG_SERDES_CTL1000X3		0x8302
#define REG_SERDES_STAT1000X1		0x8304
#define REG_COMBO_IEEE0_MIICTL		0xffe0
#define REG_COMBO_IEEE0_ANADV		0xffe4
#define REG_COMBO_IEEE0_ANLP		0xffe5

#define PORTCFG_5		5
#define PORTCFG_4		4
#define PORTCFG			"port%dcfg"
#define PORTCFG_RGMII	"rgmii"
#define PORTCFG_SGMII	"sgmii"
#define PORTCFG_GPHY	"gphy"

typedef volatile struct {
	uint32	PAD[11];
	uint32	cmdstat;	/* 0x2c, command and status register of the SRAB */
	uint32	wd_h;		/* 0x30, high order word of write data to switch registe */
	uint32	wd_l;		/* 0x34, low order word of write data to switch registe */
	uint32	rd_h;		/* 0x38, high order word of read data from switch register */
	uint32	rd_l;		/* 0x3c, low order word of read data from switch register */
	uint32	ctrls;		/* 0x40, switch interface controls */
	uint32	intr;		/* 0x44, 	the register captures interrupt pulses from the switch */
} srabregs_t;

/* Forward declaration */
typedef struct robo_info_s robo_info_t;

/* Device access/config oprands */
typedef struct {
	/* low level routines */
	void (*enable_mgmtif)(robo_info_t *robo);	/* enable mgmt i/f, optional */
	void (*disable_mgmtif)(robo_info_t *robo);	/* disable mgmt i/f, optional */
	int (*write_reg)(robo_info_t *robo, uint8 page, uint8 reg, void *val, int len);
	int (*read_reg)(robo_info_t *robo, uint8 page, uint8 reg, void *val, int len);
	/* description */
	char *desc;
} dev_ops_t;


typedef	uint16 (*miird_f)(void *h, int add, int off);
typedef	void (*miiwr_f)(void *h, int add, int off, uint16 val);

/* Private state per RoboSwitch */
struct robo_info_s {
	si_t	*sih;			/* SiliconBackplane handle */
	char	*vars;			/* nvram variables handle */
	void	*h;			/* dev handle */
	uint16	devid;			/* Device id for the switch */
	uint32	devid32;		/* Device id for the switch (32bits) */
	uint32	corerev;		/* Core rev of internal switch */

	dev_ops_t *ops;			/* device ops */
	uint8	page;			/* current page */

	/* SPI */
	uint32	ss, sck, mosi, miso;	/* GPIO mapping */

	/* MII */
	miird_f	miird;
	miiwr_f	miiwr;

	/* SRAB */
	srabregs_t *srabregs;

	uint16	prev_status;		/* link status of switch ports */
	uint32	pwrsave_mode_manual; 	/* bitmap of ports in manual power save */
	uint32	pwrsave_mode_auto; 	/* bitmap of ports in auto power save mode */
	uint8	pwrsave_phys; 		/* Phys that can be put into power save mode */
	uint8	pwrsave_mode_phys[MAX_NO_PHYS];         /* Power save mode on the switch */
};

extern int srab_sgmii_rreg(robo_info_t *robo, uint8 page, uint16 reg, uint16 *val);
extern int srab_sgmii_wreg(robo_info_t *robo, uint8 page, uint16 reg, uint16 *val);

/* Power Save mode related functions */
extern int32 robo_power_save_mode_get(robo_info_t *robo, int32 phy);
extern int32 robo_power_save_mode_set(robo_info_t *robo, int32 mode, int32 phy);
extern void robo_power_save_mode_update(robo_info_t *robo);
extern int robo_power_save_mode(robo_info_t *robo, int mode, int phy);
extern int robo_power_save_toggle(robo_info_t *robo, int normal);

extern robo_info_t *bcm_robo_attach(si_t *sih, void *h, char *vars, miird_f miird, miiwr_f miiwr);
extern void bcm_robo_detach(robo_info_t *robo);
extern int bcm_robo_enable_device(robo_info_t *robo);
extern int bcm_robo_config_vlan(robo_info_t *robo, uint8 *mac_addr);
extern int bcm_robo_enable_switch(robo_info_t *robo);
extern int robo_is_port5_cpu(void);
extern int robo_is_port_cfg(int port, char *cfg);

extern void robo_dump_regs(robo_info_t *robo, struct bcmstrbuf *b);

extern void robo_watchdog(robo_info_t *robo);

#endif /* _bcm_robo_h_ */
