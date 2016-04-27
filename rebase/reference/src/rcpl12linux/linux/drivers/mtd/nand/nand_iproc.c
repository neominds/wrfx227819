
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
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/dma-mapping.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/bug.h>
#include <linux/kernel.h>
#include <linux/bitops.h>
#include <linux/mm.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <mach/nand_iproc.h>
#include <mach/iproc_regs.h>
#include "../mtdcore.h"

/*
 * Current version of NAND controller includes spare area for ECC calculation.
 * This is not what some file system (eg. JFFS2) expects because they could 
 * write OOB first and data later. Thus we need to do some special handling.
 */
#ifdef CONFIG_IPROC_MTD_NAND_USE_JFFS2
#define NAND_MTD_WRITE_OOB_SEPARATELY
#endif /* CONFIG_IPROC_MTD_NAND_USE_JFFS2 */
/*#define NAND_REPORT_ECC_UNCORR_ERRORS*/

/*
 * This flag controls if WP stays on between erase/write
 * commands to mitigate flash corruption due to power glitches. Values:
 * 0: NAND_WP is not used or not available
 * 1: NAND_WP is set by default, cleared for erase/write operations
 * 2: NAND_WP is always cleared
 */
static int wp_on = 1;
module_param(wp_on, int, 0444);

/***********************************************************************
 * Definitions
 ***********************************************************************/

#ifdef IPROC_NAND_DEBUG
#define DBG(args...)                    printk(args)
#else
#define DBG(args...)                    do { } while(0)
#endif

/*
 * Controller/driver specific
 */
#define DRV_NAME                        "nand_iproc"
#define MAX_CONTROLLER_OOB              64

/*
 * NAND flash commands
 */
#define CMD_PAGE_READ                   0x01
#define CMD_SPARE_AREA_READ             0x02
#define CMD_STATUS_READ                 0x03
#define CMD_PROGRAM_PAGE                0x04
#define CMD_PROGRAM_SPARE_AREA          0x05
#define CMD_COPY_BACK                   0x06
#define CMD_DEVICE_ID_READ              0x07
#define CMD_BLOCK_ERASE                 0x08
#define CMD_FLASH_RESET                 0x09
#define CMD_BLOCKS_LOCK                 0x0a
#define CMD_BLOCKS_LOCK_DOWN            0x0b
#define CMD_BLOCKS_UNLOCK               0x0c
#define CMD_READ_BLOCKS_LOCK_STATUS     0x0d
#define CMD_PARAMETER_READ              0x0e
#define CMD_PARAMETER_CHANGE_COL        0x0f
#define CMD_LOW_LEVEL_OP                0x10

/* 
 * NAND controller register offset 
 */
#define NCREG_REVISION                 0x000 /* Revision */
#define NCREG_CMD_START                0x004 /* Flash Command Start */
#define NCREG_CMD_EXT_ADDRESS          0x008 /* Flash Command Extended Address */
#define NCREG_CMD_ADDRESS              0x00c /* Flash Command Address */
#define NCREG_CMD_END_ADDRESS          0x010 /* Flash Command End Address */
#define NCREG_INTFC_STATUS             0x014 /* Flash Interface Status */
#define NCREG_CS_NAND_SELECT           0x018 /* Flash EBI CS Select */
#define NCREG_CS_NAND_XOR              0x01c /* Flash EBI CS Address XOR with 1FC0 Control */
#define NCREG_LL_OP                    0x020 /* Flash Low Level Operation */
#define NCREG_MPLANE_BASE_EXT_ADDRESS  0x024 /* Flash Multiplane base address */
#define NCREG_MPLANE_BASE_ADDRESS      0x028 /* Flash Multiplane base address */
#define NCREG_ACC_CONTROL_CS0          0x050 /* Flash Access Control */
#define NCREG_CONFIG_CS0               0x054 /* Flash Config */
#define NCREG_TIMING_1_CS0             0x058 /* Flash Timing Parameters 1 */
#define NCREG_TIMING_2_CS0             0x05c /* Flash Timing Parameters 2 */
#define NCREG_ACC_CONTROL_CS1          0x060 /* Flash Access Control */
#define NCREG_CONFIG_CS1               0x064 /* Flash Config */
#define NCREG_TIMING_1_CS1             0x068 /* Flash Timing Parameters 1 */
#define NCREG_TIMING_2_CS1             0x06c /* Flash Timing Parameters 2 */
#define NCREG_ACC_CONTROL_CS2          0x070 /* Flash Access Control */
#define NCREG_CONFIG_CS2               0x074 /* Flash Config */
#define NCREG_TIMING_1_CS2             0x078 /* Flash Timing Parameters 1 */
#define NCREG_TIMING_2_CS2             0x07c /* Flash Timing Parameters 2 */
#define NCREG_CORR_STAT_THRESHOLD      0x0c0 /* Correctable Error Reporting Threshold */
#define NCREG_BLK_WR_PROTECT           0x0c8 /* Block Write Protect Enable and Size for EBI_CS0b */
#define NCREG_MULTIPLANE_OPCODES_1     0x0cc /* Flash Multiplane Customerized Opcodes */
#define NCREG_MULTIPLANE_OPCODES_2     0x0d0 /* Flash Multiplane Customerized Opcodes */
#define NCREG_MULTIPLANE_CTRL          0x0d4 /* Flash Multiplane Control */
#define NCREG_UNCORR_ERROR_COUNT       0x0fc /* Read Uncorrectable Event Count */
#define NCREG_CORR_ERROR_COUNT         0x100 /* Read Error Count */
#define NCREG_READ_ERROR_COUNT         0x104 /* Read Error Count */
#define NCREG_BLOCK_LOCK_STATUS        0x108 /* Flash Block Lock Status */
#define NCREG_ECC_CORR_EXT_ADDR        0x10c /* ECC Correctable Error Extended Address */
#define NCREG_ECC_CORR_ADDR            0x110 /* ECC Correctable Error Address */
#define NCREG_ECC_UNC_EXT_ADDR         0x114 /* ECC Uncorrectable Error Extended Address */
#define NCREG_ECC_UNC_ADDR             0x118 /* ECC Uncorrectable Error Address */
#define NCREG_FLASH_READ_EXT_ADDR      0x11c /* Flash Read Data Extended Address */
#define NCREG_FLASH_READ_ADDR          0x120 /* Flash Read Data Address */
#define NCREG_PROGRAM_PAGE_EXT_ADDR    0x124 /* Page Program Extended Address */
#define NCREG_PROGRAM_PAGE_ADDR        0x128 /* Page Program Address */
#define NCREG_COPY_BACK_EXT_ADDR       0x12c /* Copy Back Extended Address */
#define NCREG_COPY_BACK_ADDR           0x130 /* Copy Back Address */
#define NCREG_BLOCK_ERASE_EXT_ADDR     0x134 /* Block Erase Extended Address */
#define NCREG_BLOCK_ERASE_ADDR         0x138 /* Block Erase Address */
#define NCREG_INV_READ_EXT_ADDR        0x13c /* Flash Invalid Data Extended Address */
#define NCREG_INV_READ_ADDR            0x140 /* Flash Invalid Data Address */
#define NCREG_INIT_STATUS              0x144 /* Initialization status */
#define NCREG_ONFI_STATUS              0x148 /* ONFI Status */
#define NCREG_ONFI_DEBUG_DATA          0x14c /* ONFI Debug Data */
#define NCREG_SEMAPHORE                0x150 /* Semaphore */
#define NCREG_FLASH_DEVICE_ID          0x194 /* Flash Device ID */
#define NCREG_FLASH_DEVICE_ID_EXT      0x198 /* Flash Extended Device ID */
#define NCREG_LL_RDDATA                0x19c /* Flash Low Level Read Data */
#define NCREG_SPARE_AREA_READ_OFS_0    0x200 /* Flash Spare Area Read Bytes */
#define NCREG_SPARE_AREA_WRITE_OFS_0   0x280 /* Flash Spare Area Write Bytes */
#define NCREG_FLASH_CACHE_BASE         0x400 /* Flash Cache Buffer Access */

/* 
 * Required NAND controller register fields
 */
#define NCFLD_CMD_START_OPCODE_SHIFT                            24
#define NCFLD_INTFC_STATUS_FLASH_STATUS_MASK                    0x000000FF
#define NCFLD_CS_NAND_SELECT_AUTO_DEVID_CONFIG                  0x40000000
#define NCFLD_CS_NAND_SELECT_WP                                 0x20000000
#define NCFLD_CS_NAND_SELECT_DIRECT_ACCESS_CS_MASK              0x000000FF
#define NCFLD_CS_NAND_XOR_CS_MASK                               0x000000FF
#define NCFLD_CONFIG_CS0_BLOCK_SIZE_MASK                        0x70000000
#define NCFLD_CONFIG_CS0_BLOCK_SIZE_SHIFT                       28
#define NCFLD_CONFIG_CS0_DEVICE_SIZE_MASK                       0x0f000000
#define NCFLD_CONFIG_CS0_DEVICE_SIZE_SHIFT                      24
#define NCFLD_CONFIG_CS0_DEVICE_WIDTH_MASK                      0x00800000
#define NCFLD_CONFIG_CS0_DEVICE_WIDTH_SHIFT                     23
#define NCFLD_CONFIG_CS0_PAGE_SIZE_MASK                         0x00300000
#define NCFLD_CONFIG_CS0_PAGE_SIZE_SHIFT                        20
#define NCFLD_CONFIG_CS0_FUL_ADR_BYTES_MASK                     0x00070000
#define NCFLD_CONFIG_CS0_FUL_ADR_BYTES_SHIFT                    16
#define NCFLD_CONFIG_CS0_COL_ADR_BYTES_MASK                     0x00007000
#define NCFLD_CONFIG_CS0_COL_ADR_BYTES_SHIFT                    12
#define NCFLD_CONFIG_CS0_BLK_ADR_BYTES_MASK                     0x00000700
#define NCFLD_CONFIG_CS0_BLK_ADR_BYTES_SHIFT                    8
#define NCFLD_ACC_CONTROL_CS0_RD_ECC_EN_MASK                    0x80000000
#define NCFLD_ACC_CONTROL_CS0_RD_ECC_EN_SHIFT                   31
#define NCFLD_ACC_CONTROL_CS0_WR_ECC_EN_MASK                    0x40000000
#define NCFLD_ACC_CONTROL_CS0_WR_ECC_EN_SHIFT                   30
#define NCFLD_ACC_CONTROL_CS0_FAST_PGM_RDIN_MASK                0x10000000
#define NCFLD_ACC_CONTROL_CS0_FAST_PGM_RDIN_SHIFT               28
#define NCFLD_ACC_CONTROL_CS0_RD_ERASED_ECC_EN_MASK             0x08000000
#define NCFLD_ACC_CONTROL_CS0_RD_ERASED_ECC_EN_SHIFT            27
#define NCFLD_ACC_CONTROL_CS0_PARTIAL_PAGE_EN_MASK              0x04000000
#define NCFLD_ACC_CONTROL_CS0_PARTIAL_PAGE_EN_SHIFT             26
#define NCFLD_ACC_CONTROL_CS0_PAGE_HIT_EN_MASK                  0x01000000
#define NCFLD_ACC_CONTROL_CS0_PAGE_HIT_EN_SHIFT                 24
#define NCFLD_ACC_CONTROL_CS0_ECC_LEVEL_MASK                    0x001f0000
#define NCFLD_ACC_CONTROL_CS0_ECC_LEVEL_SHIFT                   16
#define NCFLD_ACC_CONTROL_CS0_SECTOR_SIZE_1K_MASK               0x00000080
#define NCFLD_ACC_CONTROL_CS0_SECTOR_SIZE_1K_SHIFT              7
#define NCFLD_ACC_CONTROL_CS0_SPARE_AREA_SIZE_MASK              0x0000007f
#define NCFLD_ACC_CONTROL_CS0_SPARE_AREA_SIZE_SHIFT             0
#define NCFLD_CORR_STAT_THRESHOLD_CS0_MASK                      0x0000003f
#define NCFLD_CORR_STAT_THRESHOLD_CS0_SHIFT                     0
#define NCFLD_CORR_STAT_THRESHOLD_CS1_MASK                      0x00000fc0
#define NCFLD_CORR_STAT_THRESHOLD_CS1_SHIFT                     6

/* 
 * IDM register base (for interrupts)
 */
#define IDMREG_NAND_IO_CONTROL_DIRECT                           0x00000000

/* 
 * Required IDM NAND IO Control register fields
 */
#define IDMFLD_NAND_IO_CONTROL_DIRECT_AXI_BE_MODE              (1UL << 28)
#define IDMFLD_NAND_IO_CONTROL_DIRECT_APB_LE_MODE              (1UL << 24)
#define IDMFLD_NAND_IO_CONTROL_DIRECT_IRQ_SHIFT                2

/*
 * Interrupts
 */
#define NCINTR_NP_READ                                          0
#define NCINTR_BLKERA                                           1
#define NCINTR_CPYBK                                            2
#define NCINTR_PGMPG                                            3
#define NCINTR_CTLRDY                                           4
#define NCINTR_RBPIN                                            5
#define NCINTR_UNC                                              6
#define NCINTR_CORR                                             7

/* 512B flash cache in the NAND controller HW */
#define FC_SHIFT            9U
#define FC_BYTES            512U
#define FC_WORDS            (FC_BYTES >> 2)
#define FC(x)               (NCREG_FLASH_CACHE_BASE + ((x) << 2))

/*
 * Register access macros - generic
 */
#define REG_RD(x)           (*((volatile u32 *)(x)))
#define REG_WR(x, y)        \
    do { *((volatile u32 *)(x)) = (y); } while(0)

/*
 * Register access macros - NAND flash controller
 */
#define NAND_REG_RD(x)          REG_RD(ctrl.nand_regs + (x))
#define NAND_REG_WR(x, y)       \
    do { REG_WR(ctrl.nand_regs + (x), (y)); } while(0)
#define NAND_REG_UNSET(x, y)    \
    do { NAND_REG_WR((x), NAND_REG_RD(x) & ~(y)); } while(0)
#define NAND_REG_SET(x, y)      \
    do { NAND_REG_WR((x), NAND_REG_RD(x) | (y)); } while(0)
#define NAND_REG_WR_RB(x, y)    \
    do { NAND_REG_WR((x), (y)); NAND_REG_RD(x); } while(0)
#define NAND_REG_SET_RB(x, y)   \
    do { NAND_REG_SET((x), (y)); NAND_REG_RD(x); } while(0)
#define NAND_REG_UNSET_RB(x, y) \
    do { NAND_REG_UNSET((x), (y)); NAND_REG_RD(x); } while(0)

/*
 * IRQ operations
 */

#define NAND_ENABLE_IRQ(bit) do {                                       \
    REG_WR(ctrl.idm_nand_regs + IDMREG_NAND_IO_CONTROL_DIRECT,          \
        REG_RD(ctrl.idm_nand_regs + IDMREG_NAND_IO_CONTROL_DIRECT) |    \
        (1UL << ((bit) + IDMFLD_NAND_IO_CONTROL_DIRECT_IRQ_SHIFT))      \
        );                                                              \
    REG_RD(ctrl.idm_nand_regs + IDMREG_NAND_IO_CONTROL_DIRECT);         \
} while (0)

#define NAND_DISABLE_IRQ(bit) do {                                      \
    REG_WR(ctrl.idm_nand_regs + IDMREG_NAND_IO_CONTROL_DIRECT,          \
        REG_RD(ctrl.idm_nand_regs + IDMREG_NAND_IO_CONTROL_DIRECT) &    \
        ~(1UL << ((bit) + IDMFLD_NAND_IO_CONTROL_DIRECT_IRQ_SHIFT))     \
        );                                                              \
    REG_RD(ctrl.idm_nand_regs + IDMREG_NAND_IO_CONTROL_DIRECT);         \
} while (0)

#define NAND_ACK_IRQ(bit) do {                                          \
    REG_WR(((u32 *)ctrl.nand_intr_regs) + (bit), 1);                    \
    REG_RD(((u32 *)ctrl.nand_intr_regs) + (bit));                       \
} while(0)

#define NAND_TEST_IRQ(bit) (REG_RD(((u32 *)ctrl.nand_intr_regs) + (bit)) & 1)

/*
 * Data access macros for endianness
 */
#ifdef __LITTLE_ENDIAN
#define NAND_BEGIN_DATA_ACCESS() do {                                   \
    REG_WR(ctrl.idm_nand_regs + IDMREG_NAND_IO_CONTROL_DIRECT,          \
        REG_RD(ctrl.idm_nand_regs + IDMREG_NAND_IO_CONTROL_DIRECT) |    \
        IDMFLD_NAND_IO_CONTROL_DIRECT_APB_LE_MODE                       \
        );                                                              \
    REG_RD(ctrl.idm_nand_regs + IDMREG_NAND_IO_CONTROL_DIRECT);         \
} while (0)

#define NAND_END_DATA_ACCESS() do {                                     \
    REG_WR(ctrl.idm_nand_regs + IDMREG_NAND_IO_CONTROL_DIRECT,          \
        REG_RD(ctrl.idm_nand_regs + IDMREG_NAND_IO_CONTROL_DIRECT) &    \
        ~IDMFLD_NAND_IO_CONTROL_DIRECT_APB_LE_MODE                      \
        );                                                              \
    REG_RD(ctrl.idm_nand_regs + IDMREG_NAND_IO_CONTROL_DIRECT);         \
} while (0)
#else /* !__LITTLE_ENDIAN */
#define NAND_BEGIN_DATA_ACCESS()    do { } while (0)
#define NAND_END_DATA_ACCESS()      do { } while (0)
#endif /* !__LITTLE_ENDIAN */

/*
 * Misc NAND controller configuration/status macros
 */

#define NC_REG_CONFIG(cs) (NCREG_CONFIG_CS0 + ((cs) << 4))

#define WR_CONFIG(cs, field, val) do {                                  \
    u32 reg = NC_REG_CONFIG(cs), contents = NAND_REG_RD(reg);           \
    contents &= ~(NCFLD_CONFIG_CS0_##field##_MASK);                     \
    contents |= (val) << NCFLD_CONFIG_CS0_##field##_SHIFT;              \
    NAND_REG_WR(reg, contents);                                         \
} while(0)

#define RD_CONFIG(cs, field)                                            \
    ((NAND_REG_RD(NC_REG_CONFIG(cs)) & NCFLD_CONFIG_CS0_##field##_MASK) \
     >> NCFLD_CONFIG_CS0_##field##_SHIFT)

#define NC_REG_ACC_CONTROL(cs) (NCREG_ACC_CONTROL_CS0 + ((cs) << 4))

#define WR_ACC_CONTROL(cs, field, val) do {                             \
    u32 reg = NC_REG_ACC_CONTROL(cs), contents = NAND_REG_RD(reg);      \
    contents &= ~(NCFLD_ACC_CONTROL_CS0_##field##_MASK);                \
    contents |= (val) << NCFLD_ACC_CONTROL_CS0_##field##_SHIFT;         \
    NAND_REG_WR(reg, contents);                                         \
} while(0)

#define RD_ACC_CONTROL(cs, field)                                       \
    ((NAND_REG_RD(NC_REG_ACC_CONTROL(cs)) &                             \
    NCFLD_ACC_CONTROL_CS0_##field##_MASK)                               \
        >> NCFLD_ACC_CONTROL_CS0_##field##_SHIFT)

#define CORR_ERROR_COUNT (NAND_REG_RD(NCREG_CORR_ERROR_COUNT))
#define UNCORR_ERROR_COUNT (NAND_REG_RD(NCREG_UNCORR_ERROR_COUNT))

#define WR_CORR_THRESH(cs, val) do {                                    \
    u32 contents = NAND_REG_RD(NCREG_CORR_STAT_THRESHOLD);              \
    u32 shift = NCFLD_CORR_STAT_THRESHOLD_CS1_SHIFT * (cs);             \
    contents &= ~(NCFLD_CORR_STAT_THRESHOLD_CS0_MASK << shift);         \
    contents |= ((val) & NCFLD_CORR_STAT_THRESHOLD_CS0_MASK) << shift;  \
    NAND_REG_WR(NCREG_CORR_STAT_THRESHOLD, contents);                   \
} while(0)

/*
 * Internal structures
 */
struct iproc_nand_controller {
    struct nand_hw_control      controller;
    int                         irq;
    int                         cmd_pending;
    struct completion           done;
    int                         boot_inited;

    volatile void               *nand_regs;
    volatile void               *nand_intr_regs;
    volatile void               *idm_nand_regs;
};

struct iproc_nand_cfg {
    u64                         device_size;
    unsigned int                block_size;
    unsigned int                page_size;
    unsigned int                spare_area_size;
    unsigned int                device_width;
    unsigned int                col_adr_bytes;
    unsigned int                blk_adr_bytes;
    unsigned int                ful_adr_bytes;
    unsigned int                sector_size_1k;
};

struct iproc_nand_host {
    u32                         buf[FC_WORDS];
    struct nand_chip            chip;
    struct mtd_info             mtd;
    struct platform_device      *pdev;
    int                         cs;
    unsigned int                last_cmd;
    unsigned int                last_byte;
    u64                         last_addr;
    struct iproc_nand_cfg       hwcfg;
    
#ifdef NAND_MTD_WRITE_OOB_SEPARATELY
    u16                         eccpos;
    u16                         eccbytes;
#endif /* NAND_MTD_WRITE_OOB_SEPARATELY */
};

static struct nand_ecclayout iproc_nand_oob_layout;

struct iproc_nand_exception {
    const char                  *name;
    int                         id[7];
    int                         idlen; /* usable */
    unsigned int                chipsize; /* MB */
    unsigned int                writesize; /* B */
    unsigned int                erasesize; /* B */
    unsigned int                oobsize; /* B per page */
    int                         chipoptions;
    int                         badblockpos;
};

/*
 * Global variables
 */

static struct iproc_nand_controller ctrl;

static struct iproc_nand_exception iproc_exceptions_list[] = {
    {"Micron MT29F8G08ABACA",
        {0x2C, 0xD3, 0x90, 0xA6, 0x64, 0x00, 0x00},
        5, 0x00400, 4096, 0x040000, 224},
    {"Micron MT29F16G08ABABA",
        {0x2C, 0x48, 0x00, 0x26, 0x89, 0x00, 0x00},
        5, 0x00800, 4096, 0x080000, 224},
    {"Micron MT29F16G08CBABA",
        {0x2C, 0x48, 0x04, 0x46, 0x85, 0x00, 0x00},
        5, 0x00800, 4096, 0x100000, 224},
    {"Micron MT29F16G08CBACA",
        {0x2C, 0x48, 0x04, 0x4A, 0xA5, 0x00, 0x00},
        5, 0x00800, 4096, 0x100000, 224},
    {"Micron MT29F16G08MAA",
        {0x2C, 0xD5, 0x94, 0x3E, 0x74, 0x00, 0x00},
        5, 0x00800, 4096, 0x080000, 218},
    {"Micron MT29F32G08CBACA",
        {0x2C, 0x68, 0x04, 0x4A, 0xA9, 0x00, 0x00},
        5, 0x01000, 4096, 0x100000, 224},
    {"Micron MT29F64G08CBAAA",
        {0x2C, 0x88, 0x04, 0x4B, 0xA9, 0x00, 0x00},
        5, 0x02000, 8192, 0x200000, 448},
    {"Micron MT29F256G08CJAAA",
        {0x2C, 0xA8, 0x05, 0xCB, 0xA9, 0x00, 0x00},
        5, 0x08000, 8192, 0x200000, 448},
    {NULL,}
};

/* Used for running nand_scan_ident without the built-in heuristics */
static struct nand_flash_dev iproc_empty_flash_table[] = {
    {NULL,}
};

/* ECC bytes required per 512B */
static const uint8_t nand_iproc_ecc_levels[2] = { 18, 21 };
static const uint8_t nand_iproc_ecc_bytes[] = {
    0, 2, 4, 6, 7, 9, 11, 13, 14, 16, 18, 20, 21, 23, 25,
    27,     /* or 3 if SPARE_AREA_SIZE == 16 && SECTOR_SIZE_1K == 0*/
    28, 30, 32, 34, 35
};

/* Strap settings */
struct nand_strap_type_t {
    uint8_t     sector_1k;
    uint8_t     ecclevel;
    uint16_t    spare_size;
};
static const struct nand_strap_type_t nand_strap_types[] = {
    { 0,  0, 16 },
    { 0, 15, 16 },
    { 0,  4, 16 },
    { 0,  8, 16 },
    { 0,  8, 27 },
    { 0, 12, 27 },
    { 1, 12, 27 },
    { 1, 15, 27 },
    { 1, 20, 45 },
};
static const uint32_t nand_strap_page_sizes[] = { 2048, 2048, 4096, 8192 };

/***********************************************************************
 * Internal support functions
 ***********************************************************************/

static void 
iproc_nand_wp(struct mtd_info *mtd, int wp)
{
    if (wp_on == 1) {
        static int old_wp = -1;
        if (old_wp != wp) {
            DBG("%s: WP %s\n", __func__, wp ? "on" : "off");
            old_wp = wp;
        }
        if (wp) {
            NAND_REG_SET_RB(NCREG_CS_NAND_SELECT, NCFLD_CS_NAND_SELECT_WP);
        } else {
            NAND_REG_UNSET_RB(NCREG_CS_NAND_SELECT, NCFLD_CS_NAND_SELECT_WP);
        }
    }
}

/* Helper functions for reading and writing OOB registers */
static inline unsigned char 
oob_reg_read(int offs)
{
    if (offs >= MAX_CONTROLLER_OOB)
        return 0x77;

    return NAND_REG_RD(NCREG_SPARE_AREA_READ_OFS_0 + (offs & ~0x03))
        >> (24 - ((offs & 0x03) << 3));
}

static inline void 
oob_reg_write(int offs, unsigned long data)
{
    if (offs >= MAX_CONTROLLER_OOB)
        return;

    NAND_REG_WR(NCREG_SPARE_AREA_WRITE_OFS_0 + (offs & ~0x03),
            data);
}

/*
 * read_oob_from_regs - read data from OOB registers
 * @i: sub-page sector index
 * @oob: buffer to read to
 * @sas: spare area sector size (i.e., OOB size per FLASH_CACHE)
 * @sector_1k: 1 for 1KiB sectors, 0 for 512B, other values are illegal
 */
static int 
read_oob_from_regs(int i, u8 *oob, int sas, int sector_1k)
{
    int tbytes = sas << sector_1k;
    int j;

    /* Adjust OOB values for 1K sector size */
    if (sector_1k && (i & 0x01))
        tbytes = max(0, tbytes - MAX_CONTROLLER_OOB);
    tbytes = min(tbytes, MAX_CONTROLLER_OOB);

    for (j = 0; j < tbytes; j++)
        oob[j] = oob_reg_read(j);
    return tbytes;
}

/*
 * write_oob_to_regs - write data to OOB registers
 * @i: sub-page sector index
 * @oob: buffer to write from
 * @sas: spare area sector size (i.e., OOB size per FLASH_CACHE)
 * @sector_1k: 1 for 1KiB sectors, 0 for 512B, other values are illegal
 */
static int 
write_oob_to_regs(int i, const u8 *oob, int sas, int sector_1k)
{
    int tbytes = sas << sector_1k;
    int j;

    /* Adjust OOB values for 1K sector size */
    if (sector_1k && (i & 0x01))
        tbytes = max(0, tbytes - MAX_CONTROLLER_OOB);
    tbytes = min(tbytes, MAX_CONTROLLER_OOB);

    for (j = 0; j < tbytes; j += 4)
        oob_reg_write(j,
                (oob[j + 0] << 24) |
                (oob[j + 1] << 16) |
                (oob[j + 2] <<  8) |
                (oob[j + 3] <<  0));
    return tbytes;
}

static irqreturn_t 
iproc_nand_irq(int irq, void *data)
{
    if (NAND_TEST_IRQ(NCINTR_CTLRDY)) {
        NAND_ACK_IRQ(NCINTR_CTLRDY);
        if (ctrl.cmd_pending) {
            /* 
             * If the direct access region (eg. 0x1c000000 on NS) is accessed,
             * IRQ handler will also be called with NCINTR_CTLRDY asserted.
             * Thus we need to filter these events by ctrl.cmd_pending, or
             * ctrl.done will be mistakenly set and cause incorrect result  for 
             * the following command.
             * We actually should avoid direct access to the mapped region when
             * NAND driver is running.
             */
            complete(&ctrl.done);
        }
        return IRQ_HANDLED;
    }
    return IRQ_NONE;
}

static void 
iproc_nand_send_cmd(int cmd)
{
    DBG("%s: native cmd %d addr_lo 0x%lx\n", __func__, cmd,(unsigned long)NAND_REG_RD(NCREG_CMD_ADDRESS));
    BUG_ON(ctrl.cmd_pending != 0);
    ctrl.cmd_pending = cmd;
    mb();
    NAND_REG_WR(NCREG_CMD_START, cmd << NCFLD_CMD_START_OPCODE_SHIFT);
}

/***********************************************************************
 * NAND MTD API: read/program/erase
 ***********************************************************************/

static void 
iproc_nand_cmd_ctrl(struct mtd_info *mtd, int dat,
    unsigned int ctrl)
{
    /* intentionally left blank */
}

static int 
iproc_nand_waitfunc(struct mtd_info *mtd, struct nand_chip *this)
{
    struct nand_chip *chip = mtd->priv;
    struct iproc_nand_host *host = chip->priv;

    DBG("%s: native cmd %d\n", __func__, ctrl.cmd_pending);
    if (ctrl.cmd_pending &&
            wait_for_completion_timeout(&ctrl.done, HZ / 10) <= 0) {
        dev_err(&host->pdev->dev,
            "timeout waiting for command %u (%ld)\n",
            host->last_cmd, (unsigned long)NAND_REG_RD(NCREG_CMD_START) >> 24);
        dev_err(&host->pdev->dev,
            "irq status %08lx, intfc status %08lx\n",
            (unsigned long)NAND_TEST_IRQ(NCINTR_CTLRDY),
            (unsigned long)NAND_REG_RD(NCREG_INTFC_STATUS));
    }
    ctrl.cmd_pending = 0;
    iproc_nand_wp(mtd, 1);
    return NAND_REG_RD(NCREG_INTFC_STATUS) & 
            NCFLD_INTFC_STATUS_FLASH_STATUS_MASK;
}

static void 
iproc_nand_cmdfunc(struct mtd_info *mtd, unsigned command,
    int column, int page_addr)
{
    struct nand_chip *chip = mtd->priv;
    struct iproc_nand_host *host = chip->priv;
    u64 addr = (u64)page_addr << chip->page_shift;
    int native_cmd = 0;

    if (command == NAND_CMD_READID
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
        || command == NAND_CMD_PARAM
#endif
        )
        addr = (u64)column;

    DBG("%s: cmd 0x%x addr 0x%llx\n", __func__, command,(unsigned long long)addr);
    host->last_cmd = command;
    host->last_byte = 0;
    host->last_addr = addr;

    switch (command) {
    case NAND_CMD_RESET:
        native_cmd = CMD_FLASH_RESET;
        break;
    case NAND_CMD_STATUS:
        native_cmd = CMD_STATUS_READ;
        break;
    case NAND_CMD_READID:
        native_cmd = CMD_DEVICE_ID_READ;
        break;
    case NAND_CMD_READOOB:
        native_cmd = CMD_SPARE_AREA_READ;
        break;
    case NAND_CMD_ERASE1:
        native_cmd = CMD_BLOCK_ERASE;
        iproc_nand_wp(mtd, 0);
        break;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
    case NAND_CMD_PARAM:
        native_cmd = CMD_PARAMETER_READ;
        break;
#endif
    }

    if (!native_cmd)
        return;

    NAND_REG_WR_RB(NCREG_CMD_EXT_ADDRESS,
        (host->cs << 16) | ((addr >> 32) & 0xffff));
    NAND_REG_WR_RB(NCREG_CMD_ADDRESS, addr & 0xffffffff);

    iproc_nand_send_cmd(native_cmd);
    iproc_nand_waitfunc(mtd, chip);
}

static uint8_t 
iproc_nand_read_byte(struct mtd_info *mtd)
{
    struct nand_chip *chip = mtd->priv;
    struct iproc_nand_host *host = chip->priv;
    uint8_t ret = 0;

    switch (host->last_cmd) {
    case NAND_CMD_READID:
        if (host->last_byte < 4)
            ret = NAND_REG_RD(NCREG_FLASH_DEVICE_ID) >>
                (24 - (host->last_byte << 3));
        else if (host->last_byte < 8)
            ret = NAND_REG_RD(NCREG_FLASH_DEVICE_ID_EXT) >>
                (56 - (host->last_byte << 3));
        break;

    case NAND_CMD_READOOB:
        ret = oob_reg_read(host->last_byte);
        break;

    case NAND_CMD_STATUS:
        ret = NAND_REG_RD(NCREG_INTFC_STATUS) & 
            NCFLD_INTFC_STATUS_FLASH_STATUS_MASK;
        if (wp_on) {
            /* Hide WP status from MTD */
            ret |= NAND_STATUS_WP;
        }
        break;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
    case NAND_CMD_PARAM:
        if (host->last_byte < FC_BYTES)
            ret = NAND_REG_RD(FC(host->last_byte >> 2)) >>
                (24 - ((host->last_byte & 0x03) << 3));
        break;
#endif
    }

    DBG("%s: byte = 0x%02x\n", __func__, ret);
    host->last_byte++;

    return ret;
}

static void 
iproc_nand_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
    int i;

    for (i = 0; i < len; i++, buf++)
        *buf = iproc_nand_read_byte(mtd);
}

/* Copied from nand_base.c to support custom iproc_check_exceptions() */
static void 
iproc_nand_erase_cmd(struct mtd_info *mtd, int page)
{
    struct nand_chip *chip = mtd->priv;
    chip->cmdfunc(mtd, NAND_CMD_ERASE1, -1, page);
}

/*
 * Assumes proper CS is already set
 */
static void 
iproc_nand_read_by_pio(struct mtd_info *mtd,
    struct nand_chip *chip, u64 addr, unsigned int trans,
    u32 *buf, u8 *oob)
{
    struct iproc_nand_host *host = chip->priv;
    int i, j;

    for (i = 0; i < trans; i++, addr += FC_BYTES) {
        NAND_REG_WR_RB(NCREG_CMD_ADDRESS, addr & 0xffffffff);
        /* SPARE_AREA_READ does not use ECC, so just use PAGE_READ */
        iproc_nand_send_cmd(CMD_PAGE_READ);
        iproc_nand_waitfunc(mtd, chip);

        if (likely(buf)) {
            NAND_BEGIN_DATA_ACCESS();
            for (j = 0; j < FC_WORDS; j++, buf++)
                *buf = NAND_REG_RD(FC(j));
            NAND_END_DATA_ACCESS();
        }

        if (oob)
            oob += read_oob_from_regs(i, oob, 
                    mtd->oobsize / trans, host->hwcfg.sector_size_1k);
    }
}

static int 
iproc_nand_read(struct mtd_info *mtd,
    struct nand_chip *chip, u64 addr, unsigned int trans,
    u32 *buf, u8 *oob)
{
    struct iproc_nand_host *host = chip->priv;
    u64 err_addr;
    DBG("%s %llx -> %p\n", __func__, (unsigned long long)addr, buf);
    
#ifdef NAND_MTD_WRITE_OOB_SEPARATELY
    /* If reading OOB only, don't enable ECC correction */
    if (!buf) {
        WR_ACC_CONTROL(host->cs, RD_ECC_EN, 0);
    }
    
    /* We must read spare area to check false uncorrectable errors. */
    if (oob == NULL) {
        oob = (u8 *)host->buf;
    }
#endif /* NAND_MTD_WRITE_OOB_SEPARATELY */

    NAND_REG_WR_RB(NCREG_ECC_UNC_ADDR, 0);
    NAND_REG_WR_RB(NCREG_ECC_CORR_ADDR, 0);
    NAND_REG_WR_RB(NCREG_CMD_EXT_ADDRESS,
        (host->cs << 16) | ((addr >> 32) & 0xffff));

    iproc_nand_read_by_pio(mtd, chip, addr, trans, buf, oob);

#ifdef NAND_MTD_WRITE_OOB_SEPARATELY
    /* Rollback ECC correction */
    if (!buf) {
        WR_ACC_CONTROL(host->cs, RD_ECC_EN, 1);
        
        /* No ECC correction was performed */
        return 0;
    }
#endif /* NAND_MTD_WRITE_OOB_SEPARATELY */

    /* Check correctable errors */
    err_addr = NAND_REG_RD(NCREG_ECC_CORR_ADDR) |
        ((u64)(NAND_REG_RD(NCREG_ECC_CORR_EXT_ADDR) & 0xffff) << 32);
    if (err_addr) {
        /*printk(KERN_DEBUG "%s: corrected error at 0x%llx\n",
            DRV_NAME, (unsigned long long)err_addr);*/
        mtd->ecc_stats.corrected += CORR_ERROR_COUNT;
        /* NAND layer expects zero on ECC errors */
        return 0;
    }

    /* Check uncorrectable errors */
    err_addr = NAND_REG_RD(NCREG_ECC_UNC_ADDR) |
        ((u64)(NAND_REG_RD(NCREG_ECC_UNC_EXT_ADDR) & 0xffff) << 32);
    if (err_addr != 0) {
#ifdef NAND_MTD_WRITE_OOB_SEPARATELY
        int i;
        
        /* Check if ECC bytes are FFs. Only the first sector is required. */
        for(i=0; i<host->eccbytes; i++) {
            if (oob[host->eccpos + i] != 0xFF) {
                break;
            }
        }
        if (i == host->eccbytes) {
            /* False alarm (the page was written with OOB only and ECC off) */
            return 0;
        }
#endif /* NAND_MTD_WRITE_OOB_SEPARATELY */

#ifdef NAND_REPORT_ECC_UNCORR_ERRORS
        printk(KERN_WARNING "%s: uncorrectable error at 0x%llx\n",
            DRV_NAME, (unsigned long long)err_addr);
        mtd->ecc_stats.failed += UNCORR_ERROR_COUNT;
#endif /* NAND_REPORT_ECC_UNCORR_ERRORS */

        /* NAND layer expects zero on ECC errors */
        return 0;
    }

    return 0;
}

static int 
iproc_nand_read_page(struct mtd_info *mtd,
    struct nand_chip *chip, uint8_t *buf, int page)
{
    struct iproc_nand_host *host = chip->priv;

    return iproc_nand_read(mtd, chip, host->last_addr,
            mtd->writesize >> FC_SHIFT, (u32 *)buf,
            (u8 *)chip->oob_poi);
}

static int 
iproc_nand_read_page_raw(struct mtd_info *mtd,
    struct nand_chip *chip, uint8_t *buf, int page)
{
    struct iproc_nand_host *host = chip->priv;
    int ret;

    WR_ACC_CONTROL(host->cs, RD_ECC_EN, 0);
    ret = iproc_nand_read(mtd, chip, host->last_addr,
            mtd->writesize >> FC_SHIFT,
            (u32 *)buf, (u8 *)chip->oob_poi);
    WR_ACC_CONTROL(host->cs, RD_ECC_EN, 1);
    return ret;
}

static int 
iproc_nand_read_oob(struct mtd_info *mtd,
    struct nand_chip *chip, int page, int sndcmd)
{
    return iproc_nand_read(mtd, chip, (u64)page << chip->page_shift,
            mtd->writesize >> FC_SHIFT,
            NULL, (u8 *)chip->oob_poi);
}

#ifdef NAND_BBT_USE_FLASH
/* Patched MTD implementation */
static int 
iproc_nand_read_oob_raw(struct mtd_info *mtd,
    struct nand_chip *chip, int page, int sndcmd)
{
    struct iproc_nand_host *host = chip->priv;

    WR_ACC_CONTROL(host->cs, RD_ECC_EN, 0);
    iproc_nand_read(mtd, chip, (u64)page << chip->page_shift,
        mtd->writesize >> FC_SHIFT,
        NULL, (u8 *)chip->oob_poi);
    WR_ACC_CONTROL(host->cs, RD_ECC_EN, 1);
    return 0;
}
#endif 

static int 
iproc_nand_read_subpage(struct mtd_info *mtd,
    struct nand_chip *chip, uint32_t data_offs, uint32_t readlen,
    uint8_t *bufpoi)
{
    struct iproc_nand_host *host = chip->priv;

    return iproc_nand_read(mtd, chip, host->last_addr + data_offs,
            readlen >> FC_SHIFT, (u32 *)bufpoi, NULL);
}

static int 
iproc_nand_write(struct mtd_info *mtd,
    struct nand_chip *chip, u64 addr, const u32 *buf, u8 *oob)
{
    struct iproc_nand_host *host = chip->priv;
    unsigned int i = 0, j, trans = mtd->writesize >> FC_SHIFT;
    int status;

    DBG("%s %llx <- %p\n", __func__, (unsigned long long)addr, buf);

    if (unlikely((u32)buf & 0x03)) {
        dev_warn(&host->pdev->dev, "unaligned buffer: %p\n", buf);
        buf = (u32 *)((u32)buf & ~0x03);
    }
    
#ifdef NAND_MTD_WRITE_OOB_SEPARATELY
    /* Merge (AND) the new and old OOB data */
    if (oob) {
    
        u8 *oob0 = (u8 *)host->buf;
        int i;
        
        /* Read the spare area from flash */
        iproc_nand_read(mtd, chip, addr, 8, NULL, oob0);
        
        /* AND them with new OOB data */
        for(i=0; i<mtd->oobsize; i++) {
            oob[i] &= oob0[i];
        }
    }
#endif /* NAND_MTD_WRITE_OOB_SEPARATELY */

    NAND_REG_WR_RB(NCREG_CMD_EXT_ADDRESS,
        (host->cs << 16) | ((addr >> 32) & 0xffff));

    for (j = 0; j < MAX_CONTROLLER_OOB; j += 4)
        oob_reg_write(j, 0xffffffff);
        
#ifdef NAND_MTD_WRITE_OOB_SEPARATELY
    /* Disable ECC generation if writing OOB only */
    if (!buf) {
        WR_ACC_CONTROL(host->cs, WR_ECC_EN, 0);
    }
#endif /* NAND_MTD_WRITE_OOB_SEPARATELY */

    for (; i < trans; i++, addr += FC_BYTES) {
    
        /* full address MUST be set before populating FC */
        NAND_REG_WR_RB(NCREG_CMD_ADDRESS, addr & 0xffffffff);

        if (buf) {
            NAND_BEGIN_DATA_ACCESS();
            for (j = 0; j < FC_WORDS; j++, buf++)
                NAND_REG_WR(FC(j), *buf);
            NAND_END_DATA_ACCESS();
        } else if (oob) {
            for (j = 0; j < FC_WORDS; j++)
                NAND_REG_WR(FC(j), 0xffffffff);
        }

        if (oob) {
            oob += write_oob_to_regs(i, oob, mtd->oobsize / trans,
                    host->hwcfg.sector_size_1k);
        }
        
        iproc_nand_wp(mtd, 0);
        
        /* we cannot use SPARE_AREA_PROGRAM when PARTIAL_PAGE_EN=0 */
        iproc_nand_send_cmd(CMD_PROGRAM_PAGE);
        status = iproc_nand_waitfunc(mtd, chip);

        if (status & NAND_STATUS_FAIL) {
            dev_info(&host->pdev->dev, "program failed at %llx\n",
                (unsigned long long)addr);
            return -EIO;
        }
    }
    
#ifdef NAND_MTD_WRITE_OOB_SEPARATELY
    /* Rollback ECC generation */
    if (!buf) {
        WR_ACC_CONTROL(host->cs, WR_ECC_EN, 1);
    }
#endif /* NAND_MTD_WRITE_OOB_SEPARATELY */
    
    return 0;
}

static void 
iproc_nand_write_page(struct mtd_info *mtd,
    struct nand_chip *chip, const uint8_t *buf)
{
    struct iproc_nand_host *host = chip->priv;

    iproc_nand_write(mtd, chip, host->last_addr, (u32 *)buf, (u8 *)chip->oob_poi);
}

static void 
iproc_nand_write_page_raw(struct mtd_info *mtd,
    struct nand_chip *chip, const uint8_t *buf)
{
    struct iproc_nand_host *host = chip->priv;

    WR_ACC_CONTROL(host->cs, WR_ECC_EN, 0);
    iproc_nand_write(mtd, chip, host->last_addr, (u32 *)buf,
        (u8 *)chip->oob_poi);
    WR_ACC_CONTROL(host->cs, WR_ECC_EN, 1);
}

static int 
iproc_nand_write_oob(struct mtd_info *mtd,
    struct nand_chip *chip, int page)
{
    return iproc_nand_write(mtd, chip, (u64)page << chip->page_shift, NULL,
        (u8 *)chip->oob_poi);
}

#ifdef NAND_BBT_USE_FLASH
/* Patched MTD implementation */
static int 
iproc_nand_write_oob_raw(struct mtd_info *mtd,
    struct nand_chip *chip, int page)
{
    struct iproc_nand_host *host = chip->priv;
    int r;

    WR_ACC_CONTROL(host->cs, WR_ECC_EN, 0);
    r = iproc_nand_write(mtd, chip, (u64)page << chip->page_shift, NULL,
        (u8 *)chip->oob_poi);
    WR_ACC_CONTROL(host->cs, WR_ECC_EN, 1);
    return r;
}
#endif 


/***********************************************************************
 * Per-CS setup (1 NAND device)
 ***********************************************************************/

static const unsigned int block_sizes[] = { 8, 16, 128, 256, 512, 1024, 2048 };
static const unsigned int page_sizes[] = { 512, 2048, 4096, 8192 };

static void 
iproc_nand_set_cfg(struct iproc_nand_host *host,
    struct iproc_nand_cfg *cfg)
{
    int i, found;

    for (i = 0, found = 0; i < ARRAY_SIZE(block_sizes); i++)
        if ((block_sizes[i] << 10) == cfg->block_size) {
            WR_CONFIG(host->cs, BLOCK_SIZE, i);
            found = 1;
        }
    if (!found)
        dev_warn(&host->pdev->dev, "invalid block size %u\n",
            cfg->block_size);

    for (i = 0, found = 0; i < ARRAY_SIZE(page_sizes); i++)
        if (page_sizes[i] == cfg->page_size) {
            WR_CONFIG(host->cs, PAGE_SIZE, i);
            found = 1;
        }
    if (!found)
        dev_warn(&host->pdev->dev, "invalid page size %u\n",
            cfg->page_size);

    if (fls64(cfg->device_size) < 23)
        dev_warn(&host->pdev->dev, "invalid device size 0x%llx\n",
            (unsigned long long)cfg->device_size);

    WR_CONFIG(host->cs, DEVICE_SIZE, fls64(cfg->device_size) - 23);
    WR_CONFIG(host->cs, DEVICE_WIDTH, cfg->device_width == 16 ? 1 : 0);
    WR_CONFIG(host->cs, COL_ADR_BYTES, cfg->col_adr_bytes);
    WR_CONFIG(host->cs, BLK_ADR_BYTES, cfg->blk_adr_bytes);
    WR_CONFIG(host->cs, FUL_ADR_BYTES, cfg->ful_adr_bytes);

    WR_ACC_CONTROL(host->cs, SPARE_AREA_SIZE, cfg->spare_area_size);
    WR_ACC_CONTROL(host->cs, SECTOR_SIZE_1K, cfg->sector_size_1k);
}

static void 
iproc_nand_get_cfg(struct iproc_nand_host *host,
    struct iproc_nand_cfg *cfg)
{
    cfg->block_size = RD_CONFIG(host->cs, BLOCK_SIZE);
    cfg->device_size = (4ULL << 20) << RD_CONFIG(host->cs, DEVICE_SIZE);
    cfg->page_size = RD_CONFIG(host->cs, PAGE_SIZE);
    cfg->device_width = RD_CONFIG(host->cs, DEVICE_WIDTH) ? 16 : 8;
    cfg->col_adr_bytes = RD_CONFIG(host->cs, COL_ADR_BYTES);
    cfg->blk_adr_bytes = RD_CONFIG(host->cs, BLK_ADR_BYTES);
    cfg->ful_adr_bytes = RD_CONFIG(host->cs, FUL_ADR_BYTES);
    cfg->spare_area_size = RD_ACC_CONTROL(host->cs, SPARE_AREA_SIZE);
    cfg->sector_size_1k = RD_ACC_CONTROL(host->cs, SECTOR_SIZE_1K);

    if (cfg->block_size < ARRAY_SIZE(block_sizes))
        cfg->block_size = block_sizes[cfg->block_size] << 10;
    else
        cfg->block_size = 128 << 10;

    if (cfg->page_size < ARRAY_SIZE(page_sizes))
        cfg->page_size = page_sizes[cfg->page_size];
    else
        cfg->page_size = 2048;
}

static void 
iproc_nand_print_cfg(char *buf, struct iproc_nand_cfg *cfg)
{
    sprintf(buf,
        "%lluMiB total, %uKiB blocks, %u%s pages, %uB OOB, %u-bit",
        (unsigned long long)cfg->device_size >> 20,
        cfg->block_size >> 10,
        cfg->page_size >= 1024 ? cfg->page_size >> 10 : cfg->page_size,
        cfg->page_size >= 1024 ? "KiB" : "B",
        cfg->spare_area_size, cfg->device_width);
}

static int iproc_nand_setup_dev(
    struct iproc_nand_host *host, 
    struct brcmnand_platform_data *pd)
{
    struct mtd_info *mtd = &host->mtd;
    struct nand_chip *chip = &host->chip;
    struct iproc_nand_cfg orig_cfg, new_cfg;
    struct nand_oobfree *free = iproc_nand_oob_layout.oobfree;
    char msg[128];
    unsigned int ecclevel;

    iproc_nand_get_cfg(host, &orig_cfg);
    host->hwcfg = orig_cfg;

    memset(&new_cfg, 0, sizeof(new_cfg));
    new_cfg.device_size = mtd->size;
    new_cfg.block_size = mtd->erasesize;
    new_cfg.page_size = mtd->writesize;
    new_cfg.spare_area_size = mtd->oobsize / (mtd->writesize >> FC_SHIFT);
    new_cfg.device_width = (chip->options & NAND_BUSWIDTH_16) ? 16 : 8;
    new_cfg.col_adr_bytes = 2;

    if (mtd->writesize > 512)
        if (mtd->size >= (256 << 20))
            new_cfg.blk_adr_bytes = 3;
        else
            new_cfg.blk_adr_bytes = 2;
    else
        if (mtd->size >= (64 << 20))
            new_cfg.blk_adr_bytes = 3;
        else
            new_cfg.blk_adr_bytes = 2;
    new_cfg.ful_adr_bytes = new_cfg.blk_adr_bytes + new_cfg.col_adr_bytes;
    
    /* Original ECC level */
    ecclevel = RD_ACC_CONTROL(host->cs, ECC_LEVEL);
    
    /* Check settings inherited from bootloader */
    if(ctrl.boot_inited) {
    
        /* Check basic device attributes first */
        int sz1k = orig_cfg.sector_size_1k? 1 : 0;
        if (orig_cfg.device_size != new_cfg.device_size ||
            orig_cfg.block_size != new_cfg.block_size ||
            orig_cfg.page_size != new_cfg.page_size ||
            orig_cfg.device_width != new_cfg.device_width ||
            orig_cfg.col_adr_bytes != new_cfg.col_adr_bytes ||
            orig_cfg.blk_adr_bytes != new_cfg.blk_adr_bytes ||
            orig_cfg.ful_adr_bytes != new_cfg.ful_adr_bytes ||
            ecclevel == 0 || ecclevel >= nand_iproc_ecc_levels[sz1k] ||
            orig_cfg.spare_area_size > new_cfg.spare_area_size ||
            nand_iproc_ecc_bytes[ecclevel] > orig_cfg.spare_area_size) {
            
            ctrl.boot_inited = 0;
            printk(KERN_INFO "%s: invalid bootloader settings\n", DRV_NAME);
            
        } else {
            /* Bootloader has initialized the flash correctly. */
            new_cfg = orig_cfg;
            iproc_nand_print_cfg(msg, &orig_cfg);
            printk(KERN_INFO "%s: following bootloader settings\n", DRV_NAME);
            printk(KERN_INFO "%s: %s\n", DRV_NAME, msg);
        }
    }

    /* Decide ECC settings ourselves if it's not initialized before */
    if (!ctrl.boot_inited) {

        /* Check if strap settings are valid */
        if (pd->strap_type > 0 &&
            nand_strap_page_sizes[pd->strap_page_size] == new_cfg.page_size &&
            nand_strap_types[pd->strap_type].spare_size <= mtd->writesize ) {
            
            /* It's valid, follow the strap settings */
            new_cfg.spare_area_size = nand_strap_types[pd->strap_type].spare_size;
            new_cfg.sector_size_1k = nand_strap_types[pd->strap_type].sector_1k;
            ecclevel = nand_strap_types[pd->strap_type].ecclevel;
            if (pd->strap_page_size == 0) {
                new_cfg.blk_adr_bytes = 2;
                new_cfg.ful_adr_bytes = 4;
            } else {
                new_cfg.blk_adr_bytes = 3;
                new_cfg.ful_adr_bytes = 5;
            }

            iproc_nand_print_cfg(msg, &new_cfg);
            printk(KERN_INFO "%s: following strap settings\n", DRV_NAME);
            printk(KERN_INFO "%s: %s\n", DRV_NAME, msg);

        } else {

            /* 
             * Strap settings are not valid, decide the settings on our own 
             */
             
            /* Trying to fit with available strap settings */
            new_cfg.spare_area_size = new_cfg.spare_area_size >= 27 ? 27 : 16;
            new_cfg.sector_size_1k = 0;
            if (new_cfg.spare_area_size == 27) {
                ecclevel = 12;
                new_cfg.sector_size_1k = (new_cfg.page_size >= 2048) ? 1 : 0;
            } else if (chip->badblockpos == NAND_SMALL_BADBLOCK_POS) {
                ecclevel = 4;
            } else {
                ecclevel = 8;
            }
            
            iproc_nand_print_cfg(msg, &new_cfg);
            printk(KERN_ERR "*ERROR* Invalid board strap settings for NAND!");
            printk(KERN_INFO "%s: overriding invalid strap settings\n", 
                DRV_NAME);
            printk(KERN_INFO "%s: %s\n", DRV_NAME, msg);
        }
    
        iproc_nand_set_cfg(host, &new_cfg);
        host->hwcfg = new_cfg;

        WR_ACC_CONTROL(host->cs, ECC_LEVEL, ecclevel);
        /* threshold = ceil(BCH-level * 0.75) */
        WR_CORR_THRESH(host->cs, ((ecclevel << new_cfg.sector_size_1k)
                    * 3 + 2) / 4);

        /* Account for 24-bit per 1024-byte ECC settings */
        if (new_cfg.sector_size_1k)
            printk(KERN_INFO "%s: ECC set to BCH-%u (1KiB sector)\n", 
                DRV_NAME, ecclevel << 1);
        else
            printk(KERN_INFO "%s: ECC set to BCH-%u (512B sector)\n", 
                DRV_NAME, ecclevel);
    }

    WR_ACC_CONTROL(host->cs, RD_ECC_EN, 1);
    WR_ACC_CONTROL(host->cs, WR_ECC_EN, 1);
    WR_ACC_CONTROL(host->cs, FAST_PGM_RDIN, 0);
    WR_ACC_CONTROL(host->cs, RD_ERASED_ECC_EN, 0);
    WR_ACC_CONTROL(host->cs, PARTIAL_PAGE_EN, 0);
    WR_ACC_CONTROL(host->cs, PAGE_HIT_EN, 1);
    
    mb();
    
    /* Adjust MTD oobsize according to the configuration */
    mtd->oobsize = new_cfg.spare_area_size * (mtd->writesize >> FC_SHIFT);
    
    /* Adjust ECC layout for storing usb OOB data */
    free->length = 0;
    if (ecclevel < nand_iproc_ecc_levels[new_cfg.sector_size_1k]) {
    
        uint8_t steps = mtd->writesize >> FC_SHIFT;
        uint8_t eccbytes = nand_iproc_ecc_bytes[ecclevel];
        
        /* Special case: using Hamming code when ecclevel == 15 */
        if (ecclevel == 15) {
            if (new_cfg.spare_area_size == 16 && !new_cfg.sector_size_1k) {
                eccbytes = 3;
            }
        }
        
        /* These are not really used. We still prepare them for safety. */
        iproc_nand_oob_layout.eccbytes = eccbytes * steps;
        chip->ecc.bytes = eccbytes;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,4,0)
        chip->ecc.strength = ecclevel;
#endif
        
#ifdef NAND_MTD_WRITE_OOB_SEPARATELY
        host->eccpos = 
            (new_cfg.spare_area_size - eccbytes) << new_cfg.sector_size_1k;
        host->eccbytes = eccbytes << new_cfg.sector_size_1k;
#endif /* NAND_MTD_WRITE_OOB_SEPARATELY */

        /* Create oobfree for storing user OOB data */
        if (new_cfg.spare_area_size > eccbytes) {
        
            unsigned int spare_size;
            uint8_t i, cnt;

            spare_size = new_cfg.spare_area_size << new_cfg.sector_size_1k;
            eccbytes <<= new_cfg.sector_size_1k;
            steps >>= new_cfg.sector_size_1k;
            if (steps > MTD_MAX_OOBFREE_ENTRIES) {
                steps = MTD_MAX_OOBFREE_ENTRIES;
            }
            for(i=0, cnt=0; i<steps && cnt<MTD_MAX_OOBFREE_ENTRIES; i++) {
            
                if (eccbytes == 3) {
                    /* Hamming code: ECC bytes are 6~8; First part here. */
                    free->offset = i * spare_size;
                    free->length = 6;
                    
                } else {
                
                    /* BCH: ECC bytes at the bottom */
                    free->offset = i * spare_size;
                    free->length = spare_size - eccbytes;
                }
                
                /* Reserve the first two bytes of the page */
                if (i == 0) {
                    if (free->length <= 2) {
                        /* Don't claim this entry if less than 2 bytes */
                        continue;
                    }
                    free->offset += 2;
                    free->length -= 2;
                }
                
                if (eccbytes == 3) {
                    /* Hamming code: the 2nd free part */
                    free++;
                    cnt++;
                    if (cnt < MTD_MAX_OOBFREE_ENTRIES) {
                        free->offset = i * spare_size + 9;
                        free->length = 7;
                    } else {
                        /* The structure limits us. */
                        break;
                    }
                }
                
                free++;
                cnt++;
            }
            if (cnt < MTD_MAX_OOBFREE_ENTRIES) {
                /* Terminater */
                free->length = 0;
            }
            
            /* Print out oob space information */
            free = iproc_nand_oob_layout.oobfree;
            if (free->length) {
                spare_size = 0;
                while(free->length) {
                    spare_size += free->length;
                    free++;
                }
                printk(KERN_INFO "%s: user oob per page: %u bytes (%u steps)\n", 
                    DRV_NAME, spare_size, (int)steps);
            }
        }
    }
    
    if (iproc_nand_oob_layout.oobfree[0].length == 0) {
        printk(KERN_INFO "%s: no oob space available\n", DRV_NAME);
    }

    return 0;
}

static int 
iproc_check_exceptions(struct mtd_info *mtd)
{
    struct nand_chip *chip = mtd->priv;
    struct iproc_nand_exception *list = iproc_exceptions_list;
    int i;
    u8 id_data[8];

    /*
     * run default nand_base initialization w/o built-in ID table;
     * should return error, so we tell it to be "silent"
     */
    chip->options |= NAND_SCAN_SILENT_NODEV;
    nand_scan_ident(mtd, 1, iproc_empty_flash_table);
    chip->options &= ~NAND_SCAN_SILENT_NODEV;

    /* Send the command for reading device ID */
    chip->cmdfunc(mtd, NAND_CMD_READID, 0x00, -1);

    for (i = 0; i < 8; i++)
        id_data[i] = chip->read_byte(mtd);

    for (; list->name != NULL; list++) {
        for (i = 0; i < list->idlen; i++)
            if (id_data[i] != list->id[i])
                break;
        if (i == list->idlen)
            break;
    }

    if (!list->name)
        return -ENODEV;

    chip->chipsize = (uint64_t)list->chipsize << 20;
    mtd->size = chip->chipsize;

    mtd->erasesize = list->erasesize;
    mtd->writesize = list->writesize;
    mtd->oobsize = list->oobsize;

    chip->options |= list->chipoptions;
    chip->badblockpos = list->badblockpos;

    /* The 3rd id byte holds MLC / multichip data */
    chip->cellinfo = id_data[2];

    chip->numchips = 1;

    /* Calculate the address shift from the page size */
    chip->page_shift = ffs(mtd->writesize) - 1;
    /* Convert chipsize to number of pages per chip -1. */
    chip->pagemask = (chip->chipsize >> chip->page_shift) - 1;

    chip->bbt_erase_shift = chip->phys_erase_shift =
        ffs(mtd->erasesize) - 1;
    chip->chip_shift = fls64(chip->chipsize) - 1;

    chip->erase_cmd = iproc_nand_erase_cmd;

    printk(KERN_INFO "%s: heuristics exception detected, %s\n",
        DRV_NAME, list->name);
    return 0;
}

static int iproc_nand_probe(struct platform_device *pdev)
{
    struct brcmnand_platform_data *pd = pdev->dev.platform_data;
    struct iproc_nand_host *host;
    struct mtd_info *mtd;
    struct nand_chip *chip;
    int ret = 0;

#if defined(CONFIG_MTD_PARTITIONS) && (LINUX_VERSION_CODE < KERNEL_VERSION(3,2,0))
    int nr_parts;
    struct mtd_partition *parts;
    const char *part_probe_types[] = { "cmdlinepart", NULL };
#endif

    DBG("%s: id %d cs %d\n", __func__, pdev->id, pd->chip_select);

    host = kzalloc(sizeof(*host), GFP_KERNEL);
    if (!host) {
        dev_err(&pdev->dev, "can't allocate memory\n");
        return -ENOMEM;
    }

    host->cs = pd->chip_select;

    mtd = &host->mtd;
    chip = &host->chip;
    host->pdev = pdev;
    dev_set_drvdata(&pdev->dev, host);

    chip->priv = host;
    mtd->priv = chip;
    mtd->name = dev_name(&pdev->dev);
    mtd->owner = THIS_MODULE;
    mtd->dev.parent = &pdev->dev;

    chip->IO_ADDR_R = (void *)0xdeadbeef;
    chip->IO_ADDR_W = (void *)0xdeadbeef;

    chip->cmd_ctrl = iproc_nand_cmd_ctrl;
    chip->cmdfunc = iproc_nand_cmdfunc;
    chip->waitfunc = iproc_nand_waitfunc;
    chip->read_byte = iproc_nand_read_byte;
    chip->read_buf = iproc_nand_read_buf;

    chip->ecc.mode = NAND_ECC_HW;
    chip->ecc.size = 512;
    chip->ecc.layout = &iproc_nand_oob_layout;
    chip->ecc.read_page = (void *) iproc_nand_read_page;
    chip->ecc.read_subpage = iproc_nand_read_subpage;
    chip->ecc.write_page = (void *) iproc_nand_write_page;
    chip->ecc.read_page_raw = (void *) iproc_nand_read_page_raw;
    chip->ecc.write_page_raw = (void *) iproc_nand_write_page_raw;

#ifdef NAND_BBT_USE_FLASH
    /* Patched MTD implementation */
    chip->ecc.write_oob_raw = iproc_nand_write_oob_raw;
    chip->ecc.read_oob_raw = (void *) iproc_nand_read_oob_raw;
#endif 

    chip->ecc.read_oob = (void *) iproc_nand_read_oob;
    chip->ecc.write_oob = iproc_nand_write_oob;

    chip->controller = &ctrl.controller;

    if (iproc_check_exceptions(mtd) && nand_scan_ident(mtd, 1, NULL)) {
        ret = -ENXIO;
        goto err1;
    }
    
    chip->options |= NAND_NO_SUBPAGE_WRITE | NAND_SKIP_BBTSCAN;
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,5,0)
    chip->options |= NAND_NO_AUTOINCR;
#endif

#ifdef NAND_BBT_USE_FLASH
    /* patched MTD implementation */ 
    chip->bbt_options |= NAND_BBT_USE_FLASH | NAND_BBT_NO_OOB;
#else /* Standard MTD implementation */
    chip->options |= NAND_USE_FLASH_BBT;
#ifdef NAND_USE_FLASH_BBT_NO_OOB
    chip->options |= NAND_USE_FLASH_BBT_NO_OOB;
#endif /* NAND_USE_FLASH_BBT_NO_OOB */
#endif /* NAND_BBT_USE_FLASH */

    if (iproc_nand_setup_dev(host, pd) || nand_scan_tail(mtd) ||
            chip->scan_bbt(mtd)) {
        ret = -ENXIO;
        goto err1;
    }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,2,0)
    mtd_device_parse_register(mtd, NULL, NULL, pd->parts, pd->nr_parts);
#else
#ifdef CONFIG_MTD_PARTITIONS
    nr_parts = parse_mtd_partitions(mtd, part_probe_types, &parts, 0);
    if (nr_parts <= 0) {
        nr_parts = pd->nr_parts;
        parts = pd->parts;
    }

    if (nr_parts)
        add_mtd_partitions(mtd, parts, nr_parts);
    else
#endif
        add_mtd_device(mtd);
#endif
        
    return 0;

err1:
    kfree(host);
    return ret;
}

static int iproc_nand_remove(struct platform_device *pdev)
{
    struct iproc_nand_host *host = dev_get_drvdata(&pdev->dev);
    struct mtd_info *mtd = &host->mtd;

    nand_release(mtd);
    dev_set_drvdata(&pdev->dev, NULL);
    kfree(host);

    return 0;
}

/***********************************************************************
 * Platform driver setup (per controller)
 ***********************************************************************/
static struct platform_driver iproc_nand_driver = {
    .probe            = iproc_nand_probe,
    .remove            = iproc_nand_remove,
    .driver = {
        .name        = "nand_iproc",
        .owner        = THIS_MODULE,
    },
};

static int __init iproc_nand_init(void)
{
    int err = -ENODEV;

    init_completion(&ctrl.done);
    spin_lock_init(&ctrl.controller.lock);
    init_waitqueue_head(&ctrl.controller.wq);
    ctrl.cmd_pending = 0;
    ctrl.boot_inited = 1;
    
    /* Initialize registers and IRQ */
    ctrl.nand_regs = ctrl.nand_intr_regs = ctrl.idm_nand_regs = NULL;
    ctrl.nand_regs = (volatile void *)ioremap(NAND_NAND_FLASH_REV, 0x1000);
    if (!ctrl.nand_regs) {
        printk(KERN_ERR "%s: can't ioremap\n", DRV_NAME);
        err = -EIO;
        goto err;
    }
    ctrl.nand_intr_regs = ctrl.nand_regs + 
        (NAND_DIRECT_READ_RD_MISS - NAND_NAND_FLASH_REV);
    ctrl.idm_nand_regs = (volatile void *)ioremap(IPROC_IDM_NAND_REG_BASE, 4);
    if (!ctrl.idm_nand_regs) {
        printk(KERN_ERR "%s: can't ioremap\n", DRV_NAME);
        err = -EIO;
        goto err;
    }
    ctrl.irq = IPROC_NAND_IRQ_START;
    DBG("%s: nand_regs - %p\n", __func__, ctrl.nand_regs);
    DBG("%s: nand_intr_regs - %p\n", __func__, ctrl.nand_intr_regs);
    DBG("%s: idm_nand_regs - %p\n", __func__, ctrl.idm_nand_regs);
    DBG("%s: irq - %d\n", __func__, ctrl.irq);

    /* If bootloader has initialized it, auto-config should be cleared */
    if (NAND_REG_RD(NCREG_CS_NAND_SELECT) & 
        NCFLD_CS_NAND_SELECT_AUTO_DEVID_CONFIG) {
        ctrl.boot_inited = 0;
    }
    
    /* Perform basic controller initialization */
    NAND_REG_UNSET(NCREG_CS_NAND_SELECT, NCFLD_CS_NAND_SELECT_AUTO_DEVID_CONFIG);
    NAND_REG_UNSET(NCREG_CS_NAND_SELECT, NCFLD_CS_NAND_SELECT_DIRECT_ACCESS_CS_MASK);
    NAND_REG_UNSET(NCREG_CS_NAND_XOR, NCFLD_CS_NAND_XOR_CS_MASK);
    if (wp_on == 2)  {
        /* Permanently remove write-protection */
        NAND_REG_UNSET(NCREG_CS_NAND_SELECT, NCFLD_CS_NAND_SELECT_WP);
    }

    /* Attach IRQ handler */
    NAND_ACK_IRQ(NCINTR_CTLRDY);
    NAND_ENABLE_IRQ(NCINTR_CTLRDY);
    err = request_irq((unsigned int)ctrl.irq, iproc_nand_irq, 0,
        DRV_NAME, &ctrl);
    if (err < 0) {
        printk(KERN_ERR "%s: unable to allocate IRQ (error %d)\n", DRV_NAME, err);
        goto err;
    }
    
    err = platform_driver_register(&iproc_nand_driver);
    if (err < 0) {
        printk(KERN_ERR "%s: can't register platform driver "
            "(error %d)\n", DRV_NAME, err);
        free_irq(ctrl.irq, &ctrl);
        goto err;
    }

    printk(KERN_INFO DRV_NAME ": NAND controller driver is loaded\n");
    return 0;

err:
    NAND_DISABLE_IRQ(NCINTR_CTLRDY);
    if (ctrl.idm_nand_regs) {
        iounmap(ctrl.idm_nand_regs);
        ctrl.idm_nand_regs = NULL;
    }
    ctrl.nand_intr_regs = NULL;
    if (ctrl.nand_regs) {
        iounmap(ctrl.nand_regs);
        ctrl.nand_regs = NULL;
    }
    return err;
}

static void __exit iproc_nand_exit(void)
{
    platform_driver_unregister(&iproc_nand_driver);
    free_irq(ctrl.irq, &ctrl);
    NAND_DISABLE_IRQ(NCINTR_CTLRDY);
    if (ctrl.idm_nand_regs) {
        iounmap(ctrl.idm_nand_regs);
        ctrl.idm_nand_regs = NULL;
    }
    ctrl.nand_intr_regs = NULL;
    if (ctrl.nand_regs) {
        iounmap(ctrl.nand_regs);
        ctrl.nand_regs = NULL;
    }
}

module_init(iproc_nand_init);
module_exit(iproc_nand_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Broadcom Corporation");
MODULE_DESCRIPTION("NAND driver for iProc chips");
