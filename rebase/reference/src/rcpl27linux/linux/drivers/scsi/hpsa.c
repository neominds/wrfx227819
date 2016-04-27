/*
 *    Disk Array driver for HP Smart Array SAS controllers
 *    Copyright 2000, 2013 Hewlett-Packard Development Company, L.P.
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; version 2 of the License.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE, GOOD TITLE or
 *    NON INFRINGEMENT.  See the GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software
 *    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *    Questions/Comments/Bugfixes to iss_storagedev@hp.com
 *
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/compat.h>
#include <linux/blktrace_api.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/completion.h>
#include <linux/moduleparam.h>
#include <scsi/scsi.h>
#include <scsi/scsi_cmnd.h>
#include <scsi/scsi_device.h>
#include <scsi/scsi_host.h>
#include <scsi/scsi_tcq.h>
#include <linux/cciss_ioctl.h>
#include <linux/string.h>
#include <linux/bitmap.h>
#include <asm/atomic.h>
#include <linux/jiffies.h>
#include <linux/percpu.h>
#include <linux/workqueue.h>
#include <asm/div64.h>
#include "hpsa_cmd.h"
#include "hpsa.h"
#include "hpsa_kernel_compat.h"

/* HPSA_DRIVER_VERSION must be 3 byte values (0-255) separated by '.' */
#define HPSA_DRIVER_VERSION "3.4.6-170"
#define DRIVER_NAME "HP HPSA Driver (v " HPSA_DRIVER_VERSION ")"
#define HPSA "hpsa"

/* How long to wait (in milliseconds) for board to go into simple mode */
#define MAX_CONFIG_WAIT 30000
#define MAX_IOCTL_CONFIG_WAIT 1000

/*define how many times we will try a command because of bus resets */
#define MAX_CMD_RETRIES 3

/* Embedded module documentation macros - see modules.h */
MODULE_AUTHOR("Hewlett-Packard Company");
MODULE_DESCRIPTION("Driver for HP Smart Array Controller version " \
	HPSA_DRIVER_VERSION " (d1059/s1248)");
MODULE_SUPPORTED_DEVICE("HP Smart Array Controllers");
MODULE_VERSION(HPSA_DRIVER_VERSION);
MODULE_LICENSE("GPL");

static int hpsa_allow_any;
module_param(hpsa_allow_any, int, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(hpsa_allow_any,
		"Allow hpsa driver to access unknown HP Smart Array hardware");
static int hpsa_simple_mode;
module_param(hpsa_simple_mode, int, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(hpsa_simple_mode,
	"Use 'simple mode' rather than 'performant mode'");
static int reply_queues = 4;
module_param(reply_queues, int, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(reply_queues,
	"Specify desired number of reply queues. 1-16, default is 4, not to exceed number of online CPUs.");

#undef PCI_DEVICE_ID_HP_CISSF
#ifndef PCI_DEVICE_ID_HP_CISSF
#define PCI_DEVICE_ID_HP_CISSF 0x323B
#endif
#undef PCI_DEVICE_ID_HP_CISSG
#ifndef PCI_DEVICE_ID_HP_CISSG
#define PCI_DEVICE_ID_HP_CISSG 0x333F
#endif
#undef PCI_DEVICE_ID_HP_CISSH
#ifndef PCI_DEVICE_ID_HP_CISSH
#define PCI_DEVICE_ID_HP_CISSH 0x323C
#endif
#undef PCI_DEVICE_ID_HP_CISSI
#ifndef PCI_DEVICE_ID_HP_CISSI
#define PCI_DEVICE_ID_HP_CISSI 0x3239
#endif
#ifndef PCI_VENDOR_ID_3PAR
#define PCI_VENDOR_ID_3PAR 0x1590
#endif
#ifndef PCI_DEVICE_ID_3PAR
#define PCI_DEVICE_ID_3PAR 0x0075
#endif

/* define the PCI info for the cards we can control */
static const struct pci_device_id hpsa_pci_device_id[] = {
#if SA_CONTROLLERS_GEN6
	{PCI_VENDOR_ID_HP,     PCI_DEVICE_ID_HP_CISSE,     0x103C, 0x3241},
	{PCI_VENDOR_ID_HP,     PCI_DEVICE_ID_HP_CISSE,     0x103C, 0x3243},
	{PCI_VENDOR_ID_HP,     PCI_DEVICE_ID_HP_CISSE,     0x103C, 0x3245},
	{PCI_VENDOR_ID_HP,     PCI_DEVICE_ID_HP_CISSE,     0x103C, 0x3247},
	{PCI_VENDOR_ID_HP,     PCI_DEVICE_ID_HP_CISSE,     0x103C, 0x3249},
	{PCI_VENDOR_ID_HP,     PCI_DEVICE_ID_HP_CISSE,     0x103C, 0x324a},
	{PCI_VENDOR_ID_HP,     PCI_DEVICE_ID_HP_CISSE,     0x103C, 0x324b},
#endif
#if SA_CONTROLLERS_GEN8
	{PCI_VENDOR_ID_HP,     PCI_DEVICE_ID_HP_CISSF,     0x103C, 0x3350},
	{PCI_VENDOR_ID_HP,     PCI_DEVICE_ID_HP_CISSF,     0x103C, 0x3351},
	{PCI_VENDOR_ID_HP,     PCI_DEVICE_ID_HP_CISSF,     0x103C, 0x3352},
	{PCI_VENDOR_ID_HP,     PCI_DEVICE_ID_HP_CISSF,     0x103C, 0x3353},
	{PCI_VENDOR_ID_HP,     PCI_DEVICE_ID_HP_CISSF,     0x103C, 0x3354},
	{PCI_VENDOR_ID_HP,     PCI_DEVICE_ID_HP_CISSF,     0x103C, 0x3355},
	{PCI_VENDOR_ID_HP,     PCI_DEVICE_ID_HP_CISSF,     0x103C, 0x3356},
#endif
#if SA_CONTROLLERS_GEN8_2
   {PCI_VENDOR_ID_HP,     PCI_DEVICE_ID_HP_CISSF,     0x103C, 0x334D},
#endif
#if SA_CONTROLLERS_GEN8_5
	{PCI_VENDOR_ID_HP,    PCI_DEVICE_ID_HP_CISSH,      0x103c, 0x1920},
	{PCI_VENDOR_ID_HP,    PCI_DEVICE_ID_HP_CISSH,      0x103c, 0x1921},
	{PCI_VENDOR_ID_HP,    PCI_DEVICE_ID_HP_CISSH,      0x103c, 0x1922},
	{PCI_VENDOR_ID_HP,    PCI_DEVICE_ID_HP_CISSH,      0x103c, 0x1923},
	{PCI_VENDOR_ID_HP,    PCI_DEVICE_ID_HP_CISSH,      0x103c, 0x1924},
	{PCI_VENDOR_ID_HP,    PCI_DEVICE_ID_HP_CISSH,      0x103c, 0x1925},
	{PCI_VENDOR_ID_HP,    PCI_DEVICE_ID_HP_CISSH,      0x103c, 0x1926},
	{PCI_VENDOR_ID_HP,    PCI_DEVICE_ID_HP_CISSH,      0x103c, 0x1928},
	{PCI_VENDOR_ID_HP,    PCI_DEVICE_ID_HP_CISSH,      0x103c, 0x1929},
	{PCI_VENDOR_ID_HP,    PCI_DEVICE_ID_HP_CISSH,      0x103c, 0x192A},
#endif
#if SA_CONTROLLERS_GEN9
	{PCI_VENDOR_ID_HP,    PCI_DEVICE_ID_HP_CISSI,      0x103c, 0x21bd},
	{PCI_VENDOR_ID_HP,    PCI_DEVICE_ID_HP_CISSI,      0x103c, 0x21be},
	{PCI_VENDOR_ID_HP,    PCI_DEVICE_ID_HP_CISSI,      0x103c, 0x21bf},
	{PCI_VENDOR_ID_HP,    PCI_DEVICE_ID_HP_CISSI,      0x103c, 0x21c0},
	{PCI_VENDOR_ID_HP,    PCI_DEVICE_ID_HP_CISSI,      0x103c, 0x21c1},
	{PCI_VENDOR_ID_HP,    PCI_DEVICE_ID_HP_CISSI,      0x103c, 0x21c2},
	{PCI_VENDOR_ID_HP,    PCI_DEVICE_ID_HP_CISSI,      0x103c, 0x21c3},
	{PCI_VENDOR_ID_HP,    PCI_DEVICE_ID_HP_CISSI,      0x103c, 0x21c4},
	{PCI_VENDOR_ID_HP,    PCI_DEVICE_ID_HP_CISSI,      0x103c, 0x21c5},
	{PCI_VENDOR_ID_HP,    PCI_DEVICE_ID_HP_CISSI,      0x103c, 0x21c6},
	{PCI_VENDOR_ID_HP,    PCI_DEVICE_ID_HP_CISSI,      0x103c, 0x21c7},
	{PCI_VENDOR_ID_HP,    PCI_DEVICE_ID_HP_CISSI,      0x103c, 0x21c8},
	{PCI_VENDOR_ID_HP,    PCI_DEVICE_ID_HP_CISSI,      0x103c, 0x21c9},
	{PCI_VENDOR_ID_HP,    PCI_DEVICE_ID_HP_CISSI,      0x103c, 0x21ca},
	{PCI_VENDOR_ID_HP,    PCI_DEVICE_ID_HP_CISSI,      0x103c, 0x21cb},
	{PCI_VENDOR_ID_HP,    PCI_DEVICE_ID_HP_CISSI,      0x103c, 0x21cc},
	{PCI_VENDOR_ID_HP,    PCI_DEVICE_ID_HP_CISSI,      0x103c, 0x21cd},
	{PCI_VENDOR_ID_HP,    PCI_DEVICE_ID_HP_CISSI,      0x103c, 0x21ce},
#endif
	HPSA_STORAGEWORKS_1210m_PCI_IDS
        {PCI_VENDOR_ID_HP,     PCI_ANY_ID,       PCI_ANY_ID, PCI_ANY_ID,
                PCI_CLASS_STORAGE_RAID << 8, 0xffff << 8, 0},
        {PCI_VENDOR_ID_COMPAQ,     PCI_ANY_ID,   PCI_ANY_ID, PCI_ANY_ID,
                PCI_CLASS_STORAGE_RAID << 8, 0xffff << 8, 0},
	{0,}
};

MODULE_DEVICE_TABLE(pci, hpsa_pci_device_id);

/*  board_id = Subsystem Device ID & Vendor ID
 *  product = Marketing Name for the board
 *  access = Address of the struct of function pointers
 */
static struct board_type products[] = {
#if SA_CONTROLLERS_GEN6
	{0x3241103C, "Smart Array P212", &SA5_access},
	{0x3243103C, "Smart Array P410", &SA5_access},
	{0x3245103C, "Smart Array P410i", &SA5_access},
	{0x3247103C, "Smart Array P411", &SA5_access},
	{0x3249103C, "Smart Array P812", &SA5_access},
	{0x324A103C, "Smart Array P712m", &SA5_access},
	{0x324B103C, "Smart Array P711m", &SA5_access},
#endif
#if SA_CONTROLLERS_GEN8
	{0x3350103C, "Smart Array P222", &SA5_access},
	{0x3351103C, "Smart Array P420", &SA5_access},
	{0x3352103C, "Smart Array P421", &SA5_access},
	{0x3353103C, "Smart Array P822", &SA5_access},
	{0x3354103C, "Smart Array P420i", &SA5_access},
	{0x3355103C, "Smart Array P220i", &SA5_access},
	{0x3356103C, "Smart Array P721", &SA5_access},
#endif
#if SA_CONTROLLERS_GEN8_2
   {0x334D103C, "Smart Array", &SA5_access},
#endif
#if SA_CONTROLLERS_GEN8_5
	{0x1920103C, "Smart Array P430i", &SA5_access},
	{0x1921103C, "Smart Array P830i", &SA5_access},
	{0x1922103C, "Smart Array P430", &SA5_access},
	{0x1923103C, "Smart Array P431", &SA5_access},
	{0x1924103C, "Smart Array P830", &SA5_access},
	{0x1925103C, "Smart Array P831", &SA5_access},
	{0x1926103C, "Smart Array P731m", &SA5_access},
	{0x1928103C, "Smart Array P230i", &SA5_access},
	{0x1929103C, "Smart Array P530", &SA5_access},
	{0x192A103C, "Smart Array P531", &SA5_access},
#endif
#if SA_CONTROLLERS_GEN9
	{0x21bd103C, "Smart Array P244br", &SA5_access},
	{0x21be103C, "Smart Array P741m", &SA5_access},
	{0x21bf103C, "Smart HBA H240ar", &SA5_access},
	{0x21c0103C, "Smart Array P440ar", &SA5_access},
	{0x21c1103C, "Smart Array", &SA5_access},
	{0x21c2103C, "Smart Array P440", &SA5_access},
	{0x21c3103C, "Smart Array P441", &SA5_access},
	{0x21c4103C, "Smart Array", &SA5_access},
	{0x21c5103C, "Smart Array", &SA5_access},
	{0x21c6103C, "Smart HBA H244br", &SA5_access},
	{0x21c7103C, "Smart HBA H240", &SA5_access},
	{0x21c8103C, "Smart HBA H241", &SA5_access},
	{0x21c9103C, "Smart Array", &SA5_access},
	{0x21ca103C, "Smart Array", &SA5_access},
	{0x21cb103C, "Smart Array P840", &SA5_access},
	{0x21cc103C, "Smart Array", &SA5_access},
	{0x21cd103C, "Smart Array", &SA5_access},
	{0x21ce103C, "Smart HBA", &SA5_access},
#endif
	HPSA_STORAGEWORKS_1210m_PRODUCT_ENTRIES
	{0xFFFF103C, "Unknown Smart Array", &SA5_access},
};

static int number_of_controllers;

DECLARE_INTERRUPT_HANDLER(do_hpsa_intr_intx);
DECLARE_INTERRUPT_HANDLER(do_hpsa_intr_msi);
static int hpsa_ioctl(struct scsi_device *dev, int cmd, void *arg);

#ifdef CONFIG_COMPAT
static int hpsa_compat_ioctl(struct scsi_device *dev, int cmd, void *arg);
#endif

static void cmd_free(struct ctlr_info *h, struct CommandList *c);
static struct CommandList *cmd_alloc(struct ctlr_info *h);
static int fill_cmd(struct CommandList *c, u8 cmd, struct ctlr_info *h,
	void *buff, size_t size, u16 page_code, unsigned char *scsi3addr,
	int cmd_type);
#define VPD_PAGE (1 << 8)

DECLARE_QUEUECOMMAND(hpsa_scsi_queue_command);
static void hpsa_scan_start(struct Scsi_Host *);
static int hpsa_scan_finished(struct Scsi_Host *sh,
	unsigned long elapsed_time);
DECLARE_CHANGE_QUEUE_DEPTH(hpsa_change_queue_depth);

static int hpsa_eh_device_reset_handler(struct scsi_cmnd *scsicmd);
static int hpsa_eh_abort_handler(struct scsi_cmnd *scsicmd);
static int hpsa_slave_alloc(struct scsi_device *sdev);
static int hpsa_slave_configure(struct scsi_device *sdev);
static void hpsa_slave_destroy(struct scsi_device *sdev);

static void hpsa_update_scsi_devices(struct ctlr_info *h, int hostno);
static int check_for_unit_attention(struct ctlr_info *h,
	struct CommandList *c);
static void check_ioctl_unit_attention(struct ctlr_info *h,
	struct CommandList *c);
/* performant mode helper functions */
static void calc_bucket_map(int *bucket, int num_buckets,
	int nsgs, int min_blocks, int *bucket_map);
static __devinit void hpsa_put_ctlr_into_performant_mode(struct ctlr_info *h);
static inline u32 next_command(struct ctlr_info *h, u8 q);
static int hpsa_find_cfg_addrs(struct pci_dev *pdev,
	void __iomem *vaddr, u32 *cfg_base_addr, u64 *cfg_base_addr_index,
	u64 *cfg_offset);
static int __devinit hpsa_pci_find_memory_BAR(struct pci_dev *pdev,
	unsigned long *memory_bar);
static int __devinit hpsa_lookup_board_id(struct pci_dev *pdev, u32 *board_id);
static int hpsa_wait_for_board_state(struct pci_dev *pdev,
	void __iomem *vaddr, int wait_for_ready);
static inline u32 hpsa_tag_discard_error_bits(struct ctlr_info *h, u32 tag);
static inline void finish_cmd(struct CommandList *c);
static void hpsa_wait_for_clear_event_notify_ack(struct ctlr_info *h);
static void hpsa_wait_for_mode_change_ack(struct ctlr_info *h);
#define BOARD_NOT_READY 0
#define BOARD_READY 1
static void hpsa_drain_accel_commands(struct ctlr_info *h);
static void hpsa_flush_cache(struct ctlr_info *h);
static int hpsa_scsi_ioaccel_queue_command(struct ctlr_info *h,
        struct CommandList *c, u32 ioaccel_handle, u8 *cdb, int cdb_len,
        u8 *scsi3addr);
static void hpsa_command_resubmit_worker(struct work_struct *work);

static inline struct ctlr_info *sdev_to_hba(struct scsi_device *sdev)
{
	unsigned long *priv = shost_priv(sdev->host);
	return (struct ctlr_info *) *priv;
}

static inline struct ctlr_info *shost_to_hba(struct Scsi_Host *sh)
{
	unsigned long *priv = shost_priv(sh);
	return (struct ctlr_info *) *priv;
}

/* extract sense key, asc, and ascq from sense data.  -1 means invalid. */
static void decode_sense_data(const u8 *sense_data, int sense_data_len,
			int *sense_key, int *asc, int *ascq)
{
	if (sense_data_len < 1) {
		*sense_key = -1;
		*ascq = -1;
		*asc = -1;
		return;
	}

	switch (sense_data[0]) {
	case 0x70: /* old format sense data */
		*sense_key = (sense_data_len > 2) ? sense_data[2] & 0x0f : -1;
		*ascq = (sense_data_len > 13) ?  sense_data[13] : -1;
		*asc = (sense_data_len > 12) ?  sense_data[12] : -1;
		break;
	case 0x72: /* descriptor format sense data */
		*sense_key = (sense_data_len > 1) ? sense_data[1] & 0x0f : -1;
		*ascq = (sense_data_len > 2) ?  sense_data[2] : -1;
		*asc = (sense_data_len > 3) ?  sense_data[3] : -1;
		break;
	default:
		*sense_key = -1;
		*ascq = -1;
		*asc = -1;
		break;
	}
}

static int check_for_unit_attention(struct ctlr_info *h,
	struct CommandList *c)
{
	int sense_key, asc, ascq;
	int sense_len;

	if (c->err_info->SenseLen > sizeof(c->err_info->SenseInfo))
		sense_len = sizeof(c->err_info->SenseInfo);
	else
		sense_len = c->err_info->SenseLen;

	decode_sense_data(c->err_info->SenseInfo, sense_len,
				&sense_key, &asc, &ascq);
	if (sense_key != UNIT_ATTENTION || asc == -1)
		return 0;

	switch (asc) {
	case STATE_CHANGED:
		dev_warn(&h->pdev->dev, HPSA "%d: a state change "
			"detected, command retried\n", h->ctlr);
		break;
	case LUN_FAILED:
		dev_warn(&h->pdev->dev,
			HPSA "%d: LUN failure detected\n", h->ctlr);
		break;
	case REPORT_LUNS_CHANGED:
		dev_warn(&h->pdev->dev,
			HPSA "%d: report LUN data changed\n", h->ctlr);
	/*
	 * Note: this REPORT_LUNS_CHANGED condition only occurs on the external
	 * target (array) devices.
	 */
		break;
	case POWER_OR_RESET:
		dev_warn(&h->pdev->dev, HPSA "%d: a power on "
			"or device reset detected\n", h->ctlr);
		break;
	case UNIT_ATTENTION_CLEARED:
		dev_warn(&h->pdev->dev, HPSA "%d: unit attention "
		    "cleared by another initiator\n", h->ctlr);
		break;
	default:
		dev_warn(&h->pdev->dev, HPSA "%d: unknown "
			"unit attention detected\n", h->ctlr);
		break;
	}
	return 1;
}

static int check_for_busy(struct ctlr_info *h, struct CommandList *c)
{
	if (c->err_info->CommandStatus != CMD_TARGET_STATUS ||
		(c->err_info->ScsiStatus != SAM_STAT_BUSY && 
		 c->err_info->ScsiStatus != SAM_STAT_TASK_SET_FULL))
		return 0;
	dev_warn(&h->pdev->dev, HPSA "device busy");
	return 1;
}

DECLARE_DEVATTR_STORE_FUNC(host_store_hp_ssd_smart_path_status)
{
	int status, len;
	struct ctlr_info *h;
	struct Scsi_Host *shost = class_to_shost(dev);
	char tmpbuf[10];

	if (!capable(CAP_SYS_ADMIN) || !capable(CAP_SYS_RAWIO))
		return -EACCES;
	len = count > sizeof(tmpbuf) - 1 ? sizeof(tmpbuf) - 1 : count;
	strncpy(tmpbuf, buf, len);
	tmpbuf[len] = '\0';
	if (sscanf(tmpbuf, "%d", &status) != 1)
		return -EINVAL;
	h = shost_to_hba(shost);
	if (h->hba_mode_enabled) {
		dev_warn(&h->pdev->dev, "hpsa: HBA mode active, "
			"cannot enable HP SSD Smart Path.\n");
		return count;
	}
	h->acciopath_status = !!status;
	dev_warn(&h->pdev->dev, "hpsa: HP SSD Smart Path %s "
		"via sysfs update.\n",
		h->acciopath_status ? "enabled" : "disabled");
	return count;
}

DECLARE_DEVATTR_STORE_FUNC(host_store_raid_offload_debug)
{
	int debug_level, len;
	struct ctlr_info *h;
	struct Scsi_Host *shost = class_to_shost(dev);
	char tmpbuf[10];

	if (!capable(CAP_SYS_ADMIN) || !capable(CAP_SYS_RAWIO))
		return -EACCES;
	len = count > sizeof(tmpbuf) - 1 ? sizeof(tmpbuf) - 1 : count;
	strncpy(tmpbuf, buf, len);
	tmpbuf[len] = '\0';
	if (sscanf(tmpbuf, "%d", &debug_level) != 1)
		return -EINVAL;
	if (debug_level < 0 )
		debug_level = 0;
	h = shost_to_hba(shost);
	h->raid_offload_debug = debug_level;
	dev_warn(&h->pdev->dev, "hpsa: Set raid_offload_debug level = %d\n",
		h->raid_offload_debug);
	return count;
}

DECLARE_DEVATTR_STORE_FUNC(host_store_rescan)
{
	struct ctlr_info *h;
	struct Scsi_Host *shost = class_to_shost(dev);
	h = shost_to_hba(shost);
	hpsa_scan_start(h->scsi_host);
	return count;
}

DECLARE_DEVATTR_SHOW_FUNC(host_show_firmware_revision)
{
	struct ctlr_info *h;
	struct Scsi_Host *shost = class_to_shost(dev);
	unsigned char *fwrev;

	h = shost_to_hba(shost);
	if (!h->hba_inquiry_data)
		return 0;
	fwrev = &h->hba_inquiry_data[32];
	return snprintf(buf, 20, "%c%c%c%c\n",
		fwrev[0], fwrev[1], fwrev[2], fwrev[3]);
}

DECLARE_DEVATTR_SHOW_FUNC(host_show_commands_outstanding)
{
	struct Scsi_Host *shost = class_to_shost(dev);
	struct ctlr_info *h = shost_to_hba(shost);

	return snprintf(buf, 20, "%d\n",
			atomic_read(&h->commands_outstanding));
}

DECLARE_DEVATTR_SHOW_FUNC(host_show_transport_mode)
{
	struct ctlr_info *h;
	struct Scsi_Host *shost = class_to_shost(dev);

	h = shost_to_hba(shost);
	return snprintf(buf, 20, "%s\n",
		h->transMethod & CFGTBL_Trans_Performant ?
			"performant" : "simple");
}

DECLARE_DEVATTR_SHOW_FUNC(host_show_hp_ssd_smart_path_status)
{
	struct ctlr_info *h;
	struct Scsi_Host *shost = class_to_shost(dev);
	h = shost_to_hba(shost);
	return snprintf(buf, 30, "HP SSD Smart Path %s\n",
		(h->acciopath_status == 1) ?
		"enabled" : "disabled");
}


/* List of controllers which cannot be hard reset on kexec with reset_devices */
static u32 unresettable_controller[] = {
	0x324a103C, /* Smart Array P712m */
	0x324b103C, /* SmartArray P711m */
	0x3223103C, /* Smart Array P800 */
	0x3234103C, /* Smart Array P400 */
	0x3235103C, /* Smart Array P400i */
	0x3211103C, /* Smart Array E200i */
	0x3212103C, /* Smart Array E200 */
	0x3213103C, /* Smart Array E200i */
	0x3214103C, /* Smart Array E200i */
	0x3215103C, /* Smart Array E200i */
	0x3237103C, /* Smart Array E500 */
	0x323D103C, /* Smart Array P700m */
	0x409C0E11, /* Smart Array 6400 */
	0x409D0E11, /* Smart Array 6400 EM */
};

/* List of controllers which cannot even be soft reset */
static u32 soft_unresettable_controller[] = {
	/* Exclude 640x boards.  These are two pci devices in one slot
	 * which share a battery backed cache module.  One controls the
	 * cache, the other accesses the cache through the one that controls
	 * it.  If we reset the one controlling the cache, the other will
	 * likely not be happy.  Just forbid resetting this conjoined mess.
	 * The 640x isn't really supported by hpsa anyway.
	 */
	0x409C0E11, /* Smart Array 6400 */
	0x409D0E11, /* Smart Array 6400 EM */
};

static u32 needs_abort_tags_swizzled[] = {
	0x324a103C, /* Smart Array P712m */
	0x324b103C, /* SmartArray P711m */
};

static int board_id_in_array(u32 a[], int nelems, u32 board_id)
{
	int i;

	for (i = 0; i < nelems; i++)
		if (a[i] == board_id)
			return 1;
	return 0;
}

static int ctlr_is_hard_resettable(u32 board_id)
{
	return !board_id_in_array(unresettable_controller,
			ARRAY_SIZE(unresettable_controller), board_id);
}

static int ctlr_is_soft_resettable(u32 board_id)
{
	return !board_id_in_array(soft_unresettable_controller,
			ARRAY_SIZE(soft_unresettable_controller), board_id);
}

static int ctlr_is_resettable(u32 board_id)
{
	return ctlr_is_hard_resettable(board_id) ||
		ctlr_is_soft_resettable(board_id);
}

DECLARE_DEVATTR_SHOW_FUNC(host_show_resettable)
{
	struct ctlr_info *h;
	struct Scsi_Host *shost = class_to_shost(dev);

	h = shost_to_hba(shost);
	return snprintf(buf, 20, "%d\n", ctlr_is_resettable(h->board_id));
}

DECLARE_DEVATTR_SHOW_FUNC(host_show_heartbeat)
{
	struct ctlr_info *h;
	struct Scsi_Host *shost = class_to_shost(dev);
	u32 heartbeat;
	unsigned long flags;

	h = shost_to_hba(shost);
	spin_lock_irqsave(&h->lock, flags);
	heartbeat = readl(&h->cfgtable->HeartBeat);
	spin_unlock_irqrestore(&h->lock, flags);
	return snprintf(buf, 20, "0x%08x\n", heartbeat);
}

static inline int is_logical_dev_addr_mode(unsigned char scsi3addr[])
{
	return (scsi3addr[3] & 0xC0) == 0x40;
}

static const char *raid_label[] = { "0", "4", "1(1+0)", "5", "5+1", "ADG",
	"1(ADM)", "UNKNOWN"
};
#define HPSA_RAID_0	0
#define HPSA_RAID_4	1
#define HPSA_RAID_1	2 	/* also used for RAID 10 */
#define HPSA_RAID_5	3	/* also used for RAID 50 */
#define HPSA_RAID_51	4
#define HPSA_RAID_6	5	/* also used for RAID 60 */
#define HPSA_RAID_ADM	6	/* also used for RAID 1+0 ADM */
#define RAID_UNKNOWN (ARRAY_SIZE(raid_label) - 1)

static ssize_t raid_level_show(struct device *dev,
	     struct device_attribute *attr, char *buf)
{
	ssize_t l = 0;
	unsigned char rlevel;
	struct ctlr_info *h;
	struct scsi_device *sdev;
	struct hpsa_scsi_dev_t *hdev;
	unsigned long flags;

	sdev = to_scsi_device(dev);
	h = sdev_to_hba(sdev);
	spin_lock_irqsave(&h->lock, flags);
	hdev = sdev->hostdata;
	if (!hdev) {
		spin_unlock_irqrestore(&h->lock, flags);
		return -ENODEV;
	}

	/* Is this even a logical drive? */
	if (!is_logical_dev_addr_mode(hdev->scsi3addr)) {
		spin_unlock_irqrestore(&h->lock, flags);
		l = snprintf(buf, PAGE_SIZE, "N/A\n");
		return l;
	}

	rlevel = hdev->raid_level;
	spin_unlock_irqrestore(&h->lock, flags);
	if (rlevel > RAID_UNKNOWN)
		rlevel = RAID_UNKNOWN;
	l = snprintf(buf, PAGE_SIZE, "RAID %s\n", raid_label[rlevel]);
	return l;
}

static ssize_t host_store_lockup_detector(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct Scsi_Host *shost;
	struct ctlr_info *h;
	int len, enabled;
	char tmpbuf[10];

	if (!capable(CAP_SYS_ADMIN) || !capable(CAP_SYS_RAWIO))
		return -EACCES;
	len = count > sizeof(tmpbuf) - 1 ? sizeof(tmpbuf) - 1 : count;
	strncpy(tmpbuf, buf, len);
	tmpbuf[len] = '\0';
	if (sscanf(tmpbuf, "%d", &enabled) != 1)
		return -EINVAL;
	shost = class_to_shost(dev);
	h = shost_to_hba(shost);
	h->lockup_detector_enabled = !!enabled;
	return count;
}

static int ctlr_needs_abort_tags_swizzled(u32 board_id)
{
	return board_id_in_array(needs_abort_tags_swizzled,
			ARRAY_SIZE(needs_abort_tags_swizzled), board_id);
}

static ssize_t host_show_lockup_detector(struct device *dev,
	     struct device_attribute *attr, char *buf)
{
	struct ctlr_info *h;
	struct Scsi_Host *shost = class_to_shost(dev);

	h = shost_to_hba(shost);
	return snprintf(buf, 20, "%d\n", h->lockup_detector_enabled);
}

static ssize_t lunid_show(struct device *dev,
	     struct device_attribute *attr, char *buf)
{
	struct ctlr_info *h;
	struct scsi_device *sdev;
	struct hpsa_scsi_dev_t *hdev;
	unsigned long flags;
	unsigned char lunid[8];

	sdev = to_scsi_device(dev);
	h = sdev_to_hba(sdev);
	spin_lock_irqsave(&h->lock, flags);
	hdev = sdev->hostdata;
	if (!hdev) {
		spin_unlock_irqrestore(&h->lock, flags);
		return -ENODEV;
	}
	memcpy(lunid, hdev->scsi3addr, sizeof(lunid));
	spin_unlock_irqrestore(&h->lock, flags);
	return snprintf(buf, 20, "0x%02x%02x%02x%02x%02x%02x%02x%02x\n",
		lunid[0], lunid[1], lunid[2], lunid[3],
		lunid[4], lunid[5], lunid[6], lunid[7]);
}

static ssize_t unique_id_show(struct device *dev,
	     struct device_attribute *attr, char *buf)
{
	struct ctlr_info *h;
	struct scsi_device *sdev;
	struct hpsa_scsi_dev_t *hdev;
	unsigned long flags;
	unsigned char sn[16];

	sdev = to_scsi_device(dev);
	h = sdev_to_hba(sdev);
	spin_lock_irqsave(&h->lock, flags);
	hdev = sdev->hostdata;
	if (!hdev) {
		spin_unlock_irqrestore(&h->lock, flags);
		return -ENODEV;
	}
	memcpy(sn, hdev->device_id, sizeof(sn));
	spin_unlock_irqrestore(&h->lock, flags);
	return snprintf(buf, 16 * 2 + 2,
			"%02X%02X%02X%02X%02X%02X%02X%02X"
			"%02X%02X%02X%02X%02X%02X%02X%02X\n",
			sn[0], sn[1], sn[2], sn[3],
			sn[4], sn[5], sn[6], sn[7],
			sn[8], sn[9], sn[10], sn[11],
			sn[12], sn[13], sn[14], sn[15]);
}

static ssize_t hp_ssd_smart_path_enabled(struct device *dev,
	     struct device_attribute *attr, char *buf)
{
	struct ctlr_info *h;
	struct scsi_device *sdev;
	struct hpsa_scsi_dev_t *hdev;
	unsigned long flags;
	int offload_enabled;

	sdev = to_scsi_device(dev);
	h = sdev_to_hba(sdev);
	spin_lock_irqsave(&h->lock, flags);
	hdev = sdev->hostdata;
	if (!hdev) {
		spin_unlock_irqrestore(&h->lock, flags);
		return -ENODEV;
	}
	offload_enabled = hdev->offload_enabled;
	spin_unlock_irqrestore(&h->lock, flags);
	return snprintf(buf, 20, "%d\n", offload_enabled);
}

#define MAX_PATHS 8
#define PATH_STRING_LEN 30
static ssize_t path_info_show(struct device *dev,
	     struct device_attribute *attr, char *buf)
{
	struct ctlr_info *h;
	struct scsi_device *sdev;
	struct hpsa_scsi_dev_t *hdev;
	unsigned long flags;
	int i;
	int output_len = 0;
	u8 box;
	u8 bay;
	u8 path_map_index = 0;
	unsigned char active;
	unsigned char phys_connector[2];
	unsigned char path[MAX_PATHS][PATH_STRING_LEN];

	memset(path, 0, MAX_PATHS * PATH_STRING_LEN);
	sdev = to_scsi_device(dev);
	h = sdev_to_hba(sdev);
	spin_lock_irqsave(&h->devlock, flags);
	hdev = sdev->hostdata;
	if (!hdev) {
		spin_unlock_irqrestore(&h->devlock, flags);
		return -ENODEV;
	}
	bay = hdev->bay;
	for (i = 0; i < MAX_PATHS; i++) {
		box = hdev->box[i];
		memcpy(&phys_connector, &hdev->phys_connector[i],
			sizeof(phys_connector));
		path_map_index = 1<<i;
		if (i == hdev->active_path_index)
			active = 'A';
		else if (hdev->path_map & path_map_index)
			active = 'I';
		else
			continue;
		if (hdev->devtype == TYPE_DISK) {
			if (box == 0 || box == 0xFF)
				output_len += snprintf(path[i],
					PATH_STRING_LEN,
					"PORT: %.2s BAY: %hhu %c\n",
					phys_connector, bay, active);
			else
				output_len += snprintf(path[i],
					PATH_STRING_LEN,
					"PORT: %.2s BOX: %hhu BAY: %hhu %c\n",
					phys_connector, box, bay, active);
		} else if (box != 0 && box != 0xFF) {
			output_len += snprintf(path[i],
				PATH_STRING_LEN, "PORT: %.2s BOX: %hhu %c\n",
				phys_connector, box, active);
		} else {
			output_len += snprintf(path[i], PATH_STRING_LEN,
				"PORT: %.2s %c\n", phys_connector, active);
		}
	}

	spin_unlock_irqrestore(&h->devlock, flags);
	return snprintf(buf, output_len+1, "%s%s%s%s%s%s%s%s",
		path[0], path[1], path[2], path[3],
		path[4], path[5], path[6], path[7]);
}

static DEVICE_ATTR(raid_level, S_IRUGO, raid_level_show, NULL);
static DEVICE_ATTR(lunid, S_IRUGO, lunid_show, NULL);
static DEVICE_ATTR(unique_id, S_IRUGO, unique_id_show, NULL);
static DEVICE_ATTR(hp_ssd_smart_path_enabled, S_IRUGO,
			hp_ssd_smart_path_enabled, NULL);
static DEVICE_ATTR(path_info, S_IRUGO, path_info_show, NULL);
DECLARE_HOST_DEVICE_ATTR(hp_ssd_smart_path_status, S_IWUSR|S_IRUGO|S_IROTH,
	host_show_hp_ssd_smart_path_status, host_store_hp_ssd_smart_path_status);
DECLARE_HOST_DEVICE_ATTR(raid_offload_debug, S_IWUSR, NULL, host_store_raid_offload_debug);
DECLARE_HOST_DEVICE_ATTR(rescan, S_IWUSR, NULL, host_store_rescan);
DECLARE_HOST_DEVICE_ATTR(firmware_revision, S_IRUGO,
	host_show_firmware_revision, NULL);
DECLARE_HOST_DEVICE_ATTR(commands_outstanding, S_IRUGO,
	host_show_commands_outstanding, NULL);
DECLARE_HOST_DEVICE_ATTR(transport_mode, S_IRUGO,
	host_show_transport_mode, NULL);
DECLARE_HOST_DEVICE_ATTR(resettable, S_IRUGO,
	host_show_resettable, NULL);
DECLARE_HOST_DEVICE_ATTR(heartbeat, S_IRUGO,
	host_show_heartbeat, NULL);
DECLARE_HOST_DEVICE_ATTR(lockup_detector, S_IWUSR|S_IRUGO,
	host_show_lockup_detector, host_store_lockup_detector);

static struct device_attribute *hpsa_sdev_attrs[] = {
	&dev_attr_raid_level,
	&dev_attr_lunid,
	&dev_attr_unique_id,
	&dev_attr_hp_ssd_smart_path_enabled,
	&dev_attr_path_info,
	NULL,
};

DECLARE_HOST_ATTR_LIST(hpsa_shost_attrs) = {
	&dev_attr_rescan,
	&dev_attr_firmware_revision,
	&dev_attr_commands_outstanding,
	&dev_attr_transport_mode,
	&dev_attr_resettable,
	&dev_attr_heartbeat,
	&dev_attr_raid_offload_debug,
	&dev_attr_hp_ssd_smart_path_status,
	&dev_attr_lockup_detector,
	NULL,
};

static struct scsi_host_template hpsa_driver_template = {
	.module			= THIS_MODULE,
	.name			= HPSA,
	.proc_name		= HPSA,
	.queuecommand		= hpsa_scsi_queue_command,
	INITIALIZE_SCAN_START(hpsa_scan_start)
	INITIALIZE_SCAN_FINISHED(hpsa_scan_finished)
	HPSA_SKIP_HOST_LOCK
	.change_queue_depth	= hpsa_change_queue_depth,
	.this_id		= -1,
	.use_clustering		= ENABLE_CLUSTERING,
	.eh_abort_handler	= hpsa_eh_abort_handler,
	.eh_device_reset_handler = hpsa_eh_device_reset_handler,
	.ioctl			= hpsa_ioctl,
	.slave_alloc		= hpsa_slave_alloc,
	.slave_configure	= hpsa_slave_configure,
	.slave_destroy		= hpsa_slave_destroy,
#ifdef CONFIG_COMPAT
	.compat_ioctl		= hpsa_compat_ioctl,
#endif
	.sdev_attrs = hpsa_sdev_attrs,
	.shost_attrs = hpsa_shost_attrs,
	.max_sectors = 8192,
};

static inline u32 next_command(struct ctlr_info *h, u8 q)
{
	u32 a;
	struct reply_queue_buffer *rq = &h->reply_queue[q];

	if (h->transMethod & CFGTBL_Trans_io_accel1)
		return h->access.command_completed(h, q);

	if (unlikely(!(h->transMethod & CFGTBL_Trans_Performant)))
		return h->access.command_completed(h, q);

	if ((rq->head[rq->current_entry] & 1) == rq->wraparound) {
		a = rq->head[rq->current_entry];
		rq->current_entry++;
		atomic_dec(&h->commands_outstanding);
	} else {
		a = FIFO_EMPTY;
	}
	/* Check for wraparound */
	if (rq->current_entry == h->max_commands) {
		rq->current_entry = 0;
		rq->wraparound ^= 1;
	}
	return a;
}

/*
 * There are some special bits in the bus address of the
 * command that we have to set for the controller to know
 * how to process the command:
 *
 * Normal performant mode:
 * bit 0: 1 means performant mode, 0 means simple mode.
 * bits 1-3 = block fetch table entry
 * bits 4-6 = command type (== 0)
 *
 * ioaccel1 mode:
 * bit 0 = "performant mode" bit.
 * bits 1-3 = block fetch table entry
 * bits 4-6 = command type (== 110)
 * (command type is needed because ioaccel1 mode 
 * commands are submitted through the same register as normal
 * mode commands, so this is how the controller knows whether
 * the command is normal mode or ioaccel1 mode.)
 *
 * ioaccel2 mode:
 * bit 0 = "performant mode" bit.
 * bits 1-4 = block fetch table entry (note extra bit)
 * bits 4-6 = not needed, because ioaccel2 mode has
 * a separate special register for submitting commands.
 */

/* set_performant_mode: Modify the tag for cciss performant
 * set bit 0 for pull model, bits 3-1 for block fetch
 * register number
 */
#define DEFAULT_REPLY_QUEUE (-1)
static void set_performant_mode(struct ctlr_info *h, struct CommandList *c,
					int reply_queue)
{
	if (likely(h->transMethod & CFGTBL_Trans_Performant)) {
		c->busaddr |= 1 | (h->blockFetchTable[c->Header.SGList] << 1);
		if (unlikely(!h->msix_vector))
			return;
		if (likely(reply_queue == DEFAULT_REPLY_QUEUE))
			c->Header.ReplyQueue =
				raw_smp_processor_id() % h->nreply_queues;
		else
			c->Header.ReplyQueue = reply_queue % h->nreply_queues;
	}
}

static void set_ioaccel1_performant_mode(struct ctlr_info *h,
						struct CommandList *c,
						int reply_queue)
{
	struct io_accel1_cmd *cp = &h->ioaccel_cmd_pool[c->cmdindex];

	/* Tell the controller to post the reply to the queue for this
	 * processor.  This seems to give the best I/O throughput.
	 */
	if (likely(reply_queue == DEFAULT_REPLY_QUEUE))
		cp->ReplyQueue = smp_processor_id() % h->nreply_queues;
	else
		cp->ReplyQueue = reply_queue % h->nreply_queues;
	/* Set the bits in the address sent down to include:
	 *  - performant mode bit (bit 0)
	 *  - pull count (bits 1-3)
	 *  - command type (bits 4-6)
	 */
	c->busaddr |= 1 | (h->ioaccel1_blockFetchTable[c->Header.SGList] << 1) |
					IOACCEL1_BUSADDR_CMDTYPE;
}

static void set_ioaccel2_performant_mode(struct ctlr_info *h,
						struct CommandList *c,
						int reply_queue)
{
	struct io_accel2_cmd *cp = &h->ioaccel2_cmd_pool[c->cmdindex];

	/* Tell the controller to post the reply to the queue for this
	 * processor.  This seems to give the best I/O throughput.
	 */
	if (likely(reply_queue == DEFAULT_REPLY_QUEUE))
		cp->reply_queue = smp_processor_id() % h->nreply_queues;
	else
		cp->reply_queue = reply_queue % h->nreply_queues;
	/* Set the bits in the address sent down to include:
	 *  - performant mode bit not used in ioaccel mode 2
	 *  - pull count (bits 0-3)
	 *  - command type isn't needed for ioaccel2
	 */
	c->busaddr |= (h->ioaccel2_blockFetchTable[cp->sg_count]);
}

static int is_firmware_flash_cmd(u8 *cdb)
{
	return cdb[0] == BMIC_WRITE && cdb[6] == BMIC_FLASH_FIRMWARE;
}

/*
 * During firmware flash, the heartbeat register may not update as frequently
 * as it should.  So we dial down lockup detection during firmware flash. and
 * dial it back up when firmware flash completes.
 */
#define HEARTBEAT_SAMPLE_INTERVAL_DURING_FLASH (240 * HZ)
#define HEARTBEAT_SAMPLE_INTERVAL (30 * HZ)
static void dial_down_lockup_detection_during_fw_flash(struct ctlr_info *h,
		struct CommandList *c)
{
	if (!is_firmware_flash_cmd(c->Request.CDB))
		return;
	atomic_inc(&h->firmware_flash_in_progress);
	h->heartbeat_sample_interval = HEARTBEAT_SAMPLE_INTERVAL_DURING_FLASH;
}

static void dial_up_lockup_detection_on_fw_flash_complete(struct ctlr_info *h,
		struct CommandList *c)
{
	if (is_firmware_flash_cmd(c->Request.CDB) &&
		atomic_dec_and_test(&h->firmware_flash_in_progress))
		h->heartbeat_sample_interval = HEARTBEAT_SAMPLE_INTERVAL;
}

static void __enqueue_cmd_and_start_io(struct ctlr_info *h,
	struct CommandList *c, int reply_queue)
{
	int count;

	dial_down_lockup_detection_during_fw_flash(h, c);
	atomic_inc(&h->commands_outstanding);
	count = atomic_read(&h->commands_outstanding);
	switch (c->cmd_type) {
	case CMD_IOACCEL1:
		set_ioaccel1_performant_mode(h, c, reply_queue);
		writel(c->busaddr, h->vaddr + SA5_REQUEST_PORT_OFFSET);
		break;
	case CMD_IOACCEL2:
		set_ioaccel2_performant_mode(h, c, reply_queue);
		writel(c->busaddr, h->vaddr + IOACCEL2_INBOUND_POSTQ_32);
		break;
	default:
		set_performant_mode(h, c, reply_queue);
		h->access.submit_command(h, c);
	}
}

static void enqueue_cmd_and_start_io(struct ctlr_info *h,
					struct CommandList *c)
{
	__enqueue_cmd_and_start_io(h, c, DEFAULT_REPLY_QUEUE);
}

static inline int is_hba_lunid(unsigned char scsi3addr[])
{
	return memcmp(scsi3addr, RAID_CTLR_LUNID, 8) == 0;
}

static inline int is_scsi_rev_5(struct ctlr_info *h)
{
	if (!h->hba_inquiry_data)
		return 0;
	if ((h->hba_inquiry_data[2] & 0x07) == 5)
		return 1;
	return 0;
}

static int hpsa_find_target_lun(struct ctlr_info *h,
	unsigned char scsi3addr[], int bus, int *target, int *lun)
{
	/* finds an unused bus, target, lun for a new physical device
	 * assumes h->devlock is held
	 */
	int i, found = 0;
	DECLARE_BITMAP(lun_taken, HPSA_MAX_DEVICES);

	bitmap_zero(lun_taken, HPSA_MAX_DEVICES);

	for (i = 0; i < h->ndevices; i++) {
		if (h->dev[i]->bus == bus && h->dev[i]->target != -1)
			__set_bit(h->dev[i]->target, lun_taken);
	}

	i = find_first_zero_bit(lun_taken, HPSA_MAX_DEVICES);
	if (i < HPSA_MAX_DEVICES) {
		/* *bus = 1; */
		*target = i;
		*lun = 0;
		found = 1;
	}
	return !found;
}

/* Add an entry into h->dev[] array. */
static int hpsa_scsi_add_entry(struct ctlr_info *h, int hostno,
		struct hpsa_scsi_dev_t *device,
		struct hpsa_scsi_dev_t *added[], int *nadded)
{
	/* assumes h->devlock is held */
	int n = h->ndevices;
	int i;
	unsigned char addr1[8], addr2[8];
	struct hpsa_scsi_dev_t *sd;

	if (n >= HPSA_MAX_DEVICES) {
		dev_err(&h->pdev->dev, "too many devices, some will be "
			"inaccessible.\n");
		return -1;
	}

	/* physical devices do not have lun or target assigned until now. */
	if (device->lun != -1)
		/* Logical device, lun is already assigned. */
		goto lun_assigned;

	/* If this device a non-zero lun of a multi-lun device
	 * byte 4 of the 8-byte LUN addr will contain the logical
	 * unit no, zero otherise.
	 */
	if (device->scsi3addr[4] == 0) {
		/* This is not a non-zero lun of a multi-lun device */
		if (hpsa_find_target_lun(h, device->scsi3addr,
			device->bus, &device->target, &device->lun) != 0)
			return -1;
		goto lun_assigned;
	}

	/* This is a non-zero lun of a multi-lun device.
	 * Search through our list and find the device which
	 * has the same 8 byte LUN address, excepting byte 4.
	 * Assign the same bus and target for this new LUN.
	 * Use the logical unit number from the firmware.
	 */
	memcpy(addr1, device->scsi3addr, 8);
	addr1[4] = 0;
	for (i = 0; i < n; i++) {
		sd = h->dev[i];
		memcpy(addr2, sd->scsi3addr, 8);
		addr2[4] = 0;
		/* differ only in byte 4? */
		if (memcmp(addr1, addr2, 8) == 0) {
			device->bus = sd->bus;
			device->target = sd->target;
			device->lun = device->scsi3addr[4];
			break;
		}
	}
	if (device->lun == -1) {
		dev_warn(&h->pdev->dev, "physical device with no LUN=0,"
			" suspect firmware bug or unsupported hardware "
			"configuration.\n");
			return -1;
	}

lun_assigned:

	h->dev[n] = device;
	h->ndevices++;
	added[*nadded] = device;
	(*nadded)++;

	/* initially, (before registering with scsi layer) we don't
	 * know our hostno and we don't want to print anything first
	 * time anyway (the scsi layer's inquiries will show that info)
	 */
	/* if (hostno != -1) */
		dev_info(&h->pdev->dev, "%s device c%db%dt%dl%d added.\n",
			scsi_device_type(device->devtype), hostno,
			device->bus, device->target, device->lun);
	return 0;
}

/* Update an entry in h->dev[] array. */
static void hpsa_scsi_update_entry(struct ctlr_info *h, int hostno,
	int entry, struct hpsa_scsi_dev_t *new_entry)
{
	/* assumes h->devlock is held */
	BUG_ON(entry < 0 || entry >= HPSA_MAX_DEVICES);

	/* Raid level changed. */
	h->dev[entry]->raid_level = new_entry->raid_level;

	/* Raid offload parameters changed.  Careful about the ordering. */
	if (new_entry->offload_config && new_entry->offload_enabled) {
		/* if drive is newly offload_enabled, we want to copy the
		 * raid map data first.  If previously offload_enabled and
		 * offload_config were set, raid map data had better be
		 * the same as it was before.  if raid map data is changed
		 * then it had better be the case that
		 * h->dev[entry]->offload_enabled is currently 0.
		 */
		h->dev[entry]->raid_map = new_entry->raid_map;
		h->dev[entry]->ioaccel_handle = new_entry->ioaccel_handle;
		wmb(); /* ensure raid map updated prior to ->offload_enabled */
	}
	if (new_entry->hba_ioaccel_enabled) {
		h->dev[entry]->ioaccel_handle = new_entry->ioaccel_handle;
		wmb(); /* set ioaccel_handle *before* hba_ioaccel_eanbled */
	}
	h->dev[entry]->hba_ioaccel_enabled = new_entry->hba_ioaccel_enabled;
	h->dev[entry]->offload_config = new_entry->offload_config;
	h->dev[entry]->offload_to_mirror = new_entry->offload_to_mirror;
	h->dev[entry]->offload_enabled = new_entry->offload_enabled;

	dev_info(&h->pdev->dev, "%s device c%db%dt%dl%d updated.\n",
		scsi_device_type(new_entry->devtype), hostno, new_entry->bus,
		new_entry->target, new_entry->lun);
}

/* Replace an entry from h->dev[] array. */
static void hpsa_scsi_replace_entry(struct ctlr_info *h, int hostno,
	int entry, struct hpsa_scsi_dev_t *new_entry,
	struct hpsa_scsi_dev_t *added[], int *nadded,
	struct hpsa_scsi_dev_t *removed[], int *nremoved)
{
	/* assumes h->devlock is held */
	BUG_ON(entry < 0 || entry >= HPSA_MAX_DEVICES);
	removed[*nremoved] = h->dev[entry];
	(*nremoved)++;

	/*
	 * New physical devices won't have target/lun assigned yet
	 * so we need to preserve the values in the slot we are replacing.
	 */
	if (new_entry->target == -1) {
		new_entry->target = h->dev[entry]->target;
		new_entry->lun = h->dev[entry]->lun;
	}

	h->dev[entry] = new_entry;
	added[*nadded] = new_entry;
	(*nadded)++;
	dev_info(&h->pdev->dev, "%s device c%db%dt%dl%d changed.\n",
		scsi_device_type(new_entry->devtype), hostno, new_entry->bus,
			new_entry->target, new_entry->lun);
}

/* Remove an entry from h->dev[] array. */
static void hpsa_scsi_remove_entry(struct ctlr_info *h, int hostno, int entry,
	struct hpsa_scsi_dev_t *removed[], int *nremoved)
{
	/* assumes h->devlock is held */
	int i;
	struct hpsa_scsi_dev_t *sd;

	BUG_ON(entry < 0 || entry >= HPSA_MAX_DEVICES);

	sd = h->dev[entry];
	removed[*nremoved] = h->dev[entry];
	(*nremoved)++;

	for (i = entry; i < h->ndevices-1; i++)
		h->dev[i] = h->dev[i+1];
	h->ndevices--;
	dev_info(&h->pdev->dev, "%s device c%db%dt%dl%d removed.\n",
		scsi_device_type(sd->devtype), hostno, sd->bus, sd->target,
		sd->lun);
}

#define SCSI3ADDR_EQ(a, b) ( \
	(a)[7] == (b)[7] && \
	(a)[6] == (b)[6] && \
	(a)[5] == (b)[5] && \
	(a)[4] == (b)[4] && \
	(a)[3] == (b)[3] && \
	(a)[2] == (b)[2] && \
	(a)[1] == (b)[1] && \
	(a)[0] == (b)[0])

static void fixup_botched_add(struct ctlr_info *h,
	struct hpsa_scsi_dev_t *added)
{
	/* called when scsi_add_device fails in order to re-adjust
	 * h->dev[] to match the mid layer's view.
	 */
	unsigned long flags;
	int i, j;

	spin_lock_irqsave(&h->lock, flags);
	for (i = 0; i < h->ndevices; i++) {
		if (h->dev[i] == added) {
			for (j = i; j < h->ndevices-1; j++)
				h->dev[j] = h->dev[j+1];
			h->ndevices--;
			break;
		}
	}
	spin_unlock_irqrestore(&h->lock, flags);
	kfree(added);
}

static inline int device_is_the_same(struct hpsa_scsi_dev_t *dev1,
	struct hpsa_scsi_dev_t *dev2)
{
	/* we compare everything except lun and target as these
	 * are not yet assigned.  Compare parts likely
	 * to differ first
	 */
	if (memcmp(dev1->scsi3addr, dev2->scsi3addr,
		sizeof(dev1->scsi3addr)) != 0)
		return 0;
	if (memcmp(dev1->device_id, dev2->device_id,
		sizeof(dev1->device_id)) != 0)
		return 0;
	if (memcmp(dev1->model, dev2->model, sizeof(dev1->model)) != 0)
		return 0;
	if (memcmp(dev1->vendor, dev2->vendor, sizeof(dev1->vendor)) != 0)
		return 0;
	if (dev1->devtype != dev2->devtype)
		return 0;
	if (dev1->bus != dev2->bus)
		return 0;
	return 1;
}

static inline int device_updated(struct hpsa_scsi_dev_t *dev1,
	struct hpsa_scsi_dev_t *dev2)
{
	/* Device attributes that can change, but don't mean
	 * that the device is a different device, nor that the OS
	 * needs to be told anything about the change.
	 */
	if (dev1->raid_level != dev2->raid_level)
		return 1;
	if (dev1->offload_config != dev2->offload_config)
		return 1;
	if (dev1->offload_enabled != dev2->offload_enabled)
		return 1;
	return 0;
}

/* Find needle in haystack.  If exact match found, return DEVICE_SAME,
 * and return needle location in *index.  If scsi3addr matches, but not
 * vendor, model, serial num, etc. return DEVICE_CHANGED, and return needle
 * location in *index.
 * In the case of a minor device attribute change, such as RAID level, just
 * return DEVICE_UPDATED, along with the updated device's location in index.
 * If needle not found, return DEVICE_NOT_FOUND.
 */
static int hpsa_scsi_find_entry(struct hpsa_scsi_dev_t *needle,
	struct hpsa_scsi_dev_t *haystack[], int haystack_size,
	int *index)
{
	int i;
#define DEVICE_NOT_FOUND 0
#define DEVICE_CHANGED 1
#define DEVICE_SAME 2
#define DEVICE_UPDATED 3
	for (i = 0; i < haystack_size; i++) {
		if (haystack[i] == NULL) /* previously removed. */
			continue;
		if (SCSI3ADDR_EQ(needle->scsi3addr, haystack[i]->scsi3addr)) {
			*index = i;
			if (device_is_the_same(needle, haystack[i])) {
				if (device_updated(needle, haystack[i]))
					return DEVICE_UPDATED;
				return DEVICE_SAME;
			} else {
				/* Keep offline devices offline */
				if (needle->volume_offline) 
					return DEVICE_NOT_FOUND;
				return DEVICE_CHANGED;
			}
		}
	}
	*index = -1;
	return DEVICE_NOT_FOUND;
}

static void hpsa_monitor_offline_device(struct ctlr_info *h,
					unsigned char scsi3addr[])
{
	struct offline_device_entry *device;
	unsigned long flags;

	/* Check to see if device is already on the list */
	spin_lock_irqsave(&h->offline_device_lock, flags);
	list_for_each_entry(device, &h->offline_device_list, offline_list) {
		if (memcmp(device->scsi3addr, scsi3addr,
				sizeof(device->scsi3addr)) == 0) {
			spin_unlock_irqrestore(&h->offline_device_lock, flags);
			return;
		}
	}
	spin_unlock_irqrestore(&h->offline_device_lock, flags);

	/* Device is not on the list, add it. */
	device = kmalloc(sizeof(*device), GFP_KERNEL);
	if (!device) {
		dev_warn(&h->pdev->dev, "out of memory in %s\n", __func__);
		return;
	}
	memcpy(device->scsi3addr, scsi3addr, sizeof(device->scsi3addr));
	spin_lock_irqsave(&h->offline_device_lock, flags);
	list_add_tail(&device->offline_list, &h->offline_device_list);
	spin_unlock_irqrestore(&h->offline_device_lock, flags);
}

/* Print a message explaining various offline volume states */
static void hpsa_show_volume_status(struct ctlr_info *h,
	struct hpsa_scsi_dev_t *sd)
{
	if (sd->volume_offline == HPSA_VPD_LV_STATUS_UNSUPPORTED)
		dev_info(&h->pdev->dev,
			"C%d:B%d:T%d:L%d Volume status is not "
			"available through vital product data pages.\n",
			h->scsi_host->host_no,
			sd->bus, sd->target, sd->lun);
	switch (sd->volume_offline) {
		case HPSA_LV_OK:
			break;
		case HPSA_LV_UNDERGOING_ERASE:
			dev_info(&h->pdev->dev,
				"C%d:B%d:T%d:L%d Volume is undergoing "
				"background erase process.\n",
				h->scsi_host->host_no,
				sd->bus, sd->target, sd->lun);
			break;
		case HPSA_LV_UNDERGOING_RPI:
			dev_info(&h->pdev->dev,
				"C%d:B%d:T%d:L%d Volume is undergoing "
				"rapid parity initialization process.\n",
				h->scsi_host->host_no,
				sd->bus, sd->target, sd->lun);
			break;
		case HPSA_LV_PENDING_RPI:
			dev_info(&h->pdev->dev,
				"C%d:B%d:T%d:L%d Volume is queued for "
				"rapid parity initialization process.\n",
				h->scsi_host->host_no,
				sd->bus, sd->target, sd->lun);
			break;
		case HPSA_LV_ENCRYPTED_NO_KEY:
			dev_info(&h->pdev->dev,
				"C%d:B%d:T%d:L%d "
				"Volume is encrypted and cannot be accessed because "
				"key is not present.\n",
				h->scsi_host->host_no,
				sd->bus, sd->target, sd->lun);
			break;
		case HPSA_LV_PLAINTEXT_IN_ENCRYPT_ONLY_CONTROLLER:
			dev_info(&h->pdev->dev,
				"C%d:B%d:T%d:L%d Volume is not encrypted "
				"and cannot be accessed because "
				"controller is in encryption-only mode.\n",
				h->scsi_host->host_no,
				sd->bus, sd->target, sd->lun);
			break;
		case HPSA_LV_UNDERGOING_ENCRYPTION:
			dev_info(&h->pdev->dev,
				"C%d:B%d:T%d:L%d Volume is undergoing "
				"encryption process.\n",
				h->scsi_host->host_no,
				sd->bus, sd->target, sd->lun);
			break;
		case HPSA_LV_UNDERGOING_ENCRYPTION_REKEYING:
			dev_info(&h->pdev->dev,
				"C%d:B%d:T%d:L%d Volume is undergoing "
				"encryption re-keying process.\n",
				h->scsi_host->host_no,
				sd->bus, sd->target, sd->lun);
			break;
		case HPSA_LV_ENCRYPTED_IN_NON_ENCRYPTED_CONTROLLER:
			dev_info(&h->pdev->dev,
				"C%d:B%d:T%d:L%d Volume is encrypted "
				"and cannot be accessed because "
				"controller does not have encryption enabled.\n",
				h->scsi_host->host_no,
				sd->bus, sd->target, sd->lun);
			break;
		case HPSA_LV_PENDING_ENCRYPTION:
			dev_info(&h->pdev->dev,
				"C%d:B%d:T%d:L%d Volume is pending migration "
				"to encrypted state, but process has not "
				"started.\n",
				h->scsi_host->host_no,
				sd->bus, sd->target, sd->lun);
			break;
		case HPSA_LV_PENDING_ENCRYPTION_REKEYING:
			dev_info(&h->pdev->dev,
				"C%d:B%d:T%d:L%d Volume is encrypted "
				"and is pending encryption rekeying.\n",
				h->scsi_host->host_no,
				sd->bus, sd->target, sd->lun);
			break;
	}
}

static void adjust_hpsa_scsi_table(struct ctlr_info *h, int hostno,
	struct hpsa_scsi_dev_t *sd[], int nsds)
{
	/* sd contains scsi3 addresses and devtypes, and inquiry
	 * data.  This function takes what's in sd to be the current
	 * reality and updates h->dev[] to reflect that reality.
	 */
	int i, entry, device_change, changes = 0;
	struct hpsa_scsi_dev_t *csd;
	unsigned long flags;
	struct hpsa_scsi_dev_t **added, **removed;
	int nadded, nremoved;
	struct Scsi_Host *sh = NULL;

	added = kzalloc(sizeof(*added) * HPSA_MAX_DEVICES, GFP_KERNEL);
	removed = kzalloc(sizeof(*removed) * HPSA_MAX_DEVICES, GFP_KERNEL);

	if (!added || !removed) {
		dev_warn(&h->pdev->dev, "out of memory in "
			"adjust_hpsa_scsi_table\n");
		goto free_and_out;
	}

	spin_lock_irqsave(&h->devlock, flags);

	/* find any devices in h->dev[] that are not in
	 * sd[] and remove them from h->dev[], and for any
	 * devices which have changed, remove the old device
	 * info and add the new device info.
	 * If minor device attributes change, just update
	 * the existing device structure.
	 */
	i = 0;
	nremoved = 0;
	nadded = 0;
	while (i < h->ndevices) {
		csd = h->dev[i];
		device_change = hpsa_scsi_find_entry(csd, sd, nsds, &entry);
		if (device_change == DEVICE_NOT_FOUND) {
			changes++;
			hpsa_scsi_remove_entry(h, hostno, i,
				removed, &nremoved);
			continue; /* remove ^^^, hence i not incremented */
		} else if (device_change == DEVICE_CHANGED) {
			changes++;
			hpsa_scsi_replace_entry(h, hostno, i, sd[entry],
				added, &nadded, removed, &nremoved);
			/* Set it to NULL to prevent it from being freed
			 * at the bottom of hpsa_update_scsi_devices()
			 */
			sd[entry] = NULL;
		} else if (device_change == DEVICE_UPDATED) {
			hpsa_scsi_update_entry(h, hostno, i, sd[entry]);
		}
		i++;
	}

	/* Now, make sure every device listed in sd[] is also
	 * listed in h->dev[], adding them if they aren't found
	 */

	for (i = 0; i < nsds; i++) {
		if (!sd[i]) /* if already added above. */
			continue;

		/* Don't add devices which are NOT READY, FORMAT IN PROGRESS
		 * as the SCSI mid-layer does not handle such devices well.
		 * It relentlessly loops sending TUR at 3Hz, then READ(10)
		 * at 160Hz, and prevents the system from coming up.
		 */
		if (sd[i]->volume_offline) {
			hpsa_show_volume_status(h, sd[i]);
			dev_info(&h->pdev->dev, "c%db%dt%dl%d: temporarily offline\n",
				h->scsi_host->host_no,
				sd[i]->bus, sd[i]->target, sd[i]->lun);
			continue;
		}

		device_change = hpsa_scsi_find_entry(sd[i], h->dev,
					h->ndevices, &entry);
		if (device_change == DEVICE_NOT_FOUND) {
			changes++;
			if (hpsa_scsi_add_entry(h, hostno, sd[i],
				added, &nadded) != 0)
				break;
			sd[i] = NULL; /* prevent from being freed later. */
		} else if (device_change == DEVICE_CHANGED) {
			/* should never happen... */
			changes++;
			dev_warn(&h->pdev->dev,
				"device unexpectedly changed.\n");
			/* but if it does happen, we just ignore that device */
		}
	}
	spin_unlock_irqrestore(&h->devlock, flags);

	/* Monitor devices which are in one of several NOT READY states to be
	 * brought online later. This must be done without holding h->devlock,
	 * so don't touch h->dev[]
	 */
	for (i = 0; i < nsds; i++) {
		if (!sd[i]) /* if already added above. */
			continue;
		if (sd[i]->volume_offline)
			hpsa_monitor_offline_device(h, sd[i]->scsi3addr);
	}

	/* Don't notify scsi mid layer of any changes the first time through
	 * (or if there are no changes) scsi_scan_host will do it later the
	 * first time through.
	 */
	if (hostno == -1 || !changes)
		goto free_and_out;

	sh = h->scsi_host;
	/* Notify scsi mid layer of any removed devices */
	for (i = 0; i < nremoved; i++) {
		if (removed[i]->expose_state & HPSA_SCSI_ADD) {
			struct scsi_device *sdev =
				scsi_device_lookup(sh, removed[i]->bus,
					removed[i]->target, removed[i]->lun);
			if (sdev != NULL) {
				scsi_remove_device(sdev);
				scsi_device_put(sdev);
			} else {
				/* We don't expect to get here.
				 * future cmds to this device will get selection
				 * timeout as if the device was gone.
				 */
				dev_warn(&h->pdev->dev,
					"didn't find c%db%dt%dl%d for removal.",
					hostno, removed[i]->bus,
					removed[i]->target, removed[i]->lun);
			}
		}
		kfree(removed[i]);
		removed[i] = NULL;
	}

	/* Notify scsi mid layer of any added devices */
	for (i = 0; i < nadded; i++) {
		if (!(added[i]->expose_state & HPSA_SCSI_ADD))
			continue;
		if (scsi_add_device(sh, added[i]->bus,
			added[i]->target, added[i]->lun) == 0)
			continue;
		dev_warn(&h->pdev->dev, "scsi_add_device c%db%dt%dl%d failed, "
			"device not added.\n", hostno, added[i]->bus,
			added[i]->target, added[i]->lun);
		/* now we have to remove it from h->dev,
		 * since it didn't get added to scsi mid layer
		 */
		fixup_botched_add(h, added[i]);
	}

free_and_out:
	kfree(added);
	kfree(removed);
}

/*
 * Lookup bus/target/lun and retrun corresponding struct hpsa_scsi_dev_t *
 * Assume's h->devlock is held.
 */
static struct hpsa_scsi_dev_t *lookup_hpsa_scsi_dev(struct ctlr_info *h,
	int bus, int target, int lun)
{
	int i;
	struct hpsa_scsi_dev_t *sd;

	for (i = 0; i < h->ndevices; i++) {
		sd = h->dev[i];
		if (sd->bus == bus && sd->target == target && sd->lun == lun)
			return sd;
	}
	return NULL;
}

/* link sdev->hostdata to our per-device structure. */
static int hpsa_slave_alloc(struct scsi_device *sdev)
{
	struct hpsa_scsi_dev_t *sd;
	unsigned long flags;
	struct ctlr_info *h;

	h = sdev_to_hba(sdev);
	spin_lock_irqsave(&h->devlock, flags);
	sd = lookup_hpsa_scsi_dev(h, sdev_channel(sdev),
		sdev_id(sdev), sdev->lun);
	if (sd && (sd->expose_state & HPSA_SCSI_ADD)) {
		sdev->hostdata = sd;
		if (sd->queue_depth)
			scsi_adjust_queue_depth(sdev, scsi_get_tag_type(sdev),
				sd->queue_depth);
		atomic_set(&sd->ioaccel_cmds_out, 0);
	} else if (sd) {
		sdev->hostdata = NULL;
	}

	spin_unlock_irqrestore(&h->devlock, flags);
	return 0;
}

/* configure scsi device based on internal per-device structure */
static int hpsa_slave_configure(struct scsi_device *sdev)
{
	struct hpsa_scsi_dev_t *sd;
	unsigned long flags;
	struct ctlr_info *h;

	h = sdev_to_hba(sdev);
	spin_lock_irqsave(&h->devlock, flags);
	sd = sdev->hostdata;
	if (sd && sd->expose_state & HPSA_NO_ULD_ATTACH)
		sdev->no_uld_attach = 1;

	spin_unlock_irqrestore(&h->devlock, flags);
	return 0;
}

static void hpsa_slave_destroy(struct scsi_device *sdev)
{
	/* nothing to do. */
}

static void hpsa_free_ioaccel2_sg_chain_blocks(struct ctlr_info *h)
{
	int i;

	if (!h->ioaccel2_cmd_sg_list)
		return;
	for (i = 0; i < h->nr_cmds; i++) {
		kfree(h->ioaccel2_cmd_sg_list[i]);
		h->ioaccel2_cmd_sg_list[i] = NULL;
	}
	kfree(h->ioaccel2_cmd_sg_list);
	h->ioaccel2_cmd_sg_list = NULL;
}

static int hpsa_allocate_ioaccel2_sg_chain_blocks(struct ctlr_info *h)
{
	int i;

	if (h->chainsize <= 0)
		return 0;

	h->ioaccel2_cmd_sg_list = 
		kzalloc(sizeof(*h->ioaccel2_cmd_sg_list) * h->nr_cmds,
		GFP_KERNEL);
	if (!h->ioaccel2_cmd_sg_list)
		return -ENOMEM;
	for (i = 0; i < h->nr_cmds; i++) {
		h->ioaccel2_cmd_sg_list[i] = 
			kmalloc(sizeof(*h->ioaccel2_cmd_sg_list[i]) *
			h->maxsgentries, GFP_KERNEL);
		if (!h->ioaccel2_cmd_sg_list[i])
			goto clean;
	}
	return 0;

clean:
	hpsa_free_ioaccel2_sg_chain_blocks(h);
	return -ENOMEM;
}

static void hpsa_free_sg_chain_blocks(struct ctlr_info *h)
{
	int i;

	if (!h->cmd_sg_list)
		return;
	for (i = 0; i < h->nr_cmds; i++) {
		kfree(h->cmd_sg_list[i]);
		h->cmd_sg_list[i] = NULL;
	}
	kfree(h->cmd_sg_list);
	h->cmd_sg_list = NULL;
}

static int hpsa_allocate_sg_chain_blocks(struct ctlr_info *h)
{
	int i;

	if (h->chainsize <= 0)
		return 0;

	h->cmd_sg_list = kzalloc(sizeof(*h->cmd_sg_list) * h->nr_cmds,
				GFP_KERNEL);
	if (!h->cmd_sg_list)
		return -ENOMEM;
	for (i = 0; i < h->nr_cmds; i++) {
		h->cmd_sg_list[i] = kmalloc(sizeof(*h->cmd_sg_list[i]) *
						h->chainsize, GFP_KERNEL);
		if (!h->cmd_sg_list[i])
			goto clean;
	}
	return 0;

clean:
	hpsa_free_sg_chain_blocks(h);
	return -ENOMEM;
}

static int hpsa_map_ioaccel2_sg_chain_block(struct ctlr_info *h,
	struct io_accel2_cmd *cp, struct CommandList *c)
{
	struct ioaccel2_sg_element *chain_block;
	u64 temp64;
	u32 chain_size;

	chain_block = h->ioaccel2_cmd_sg_list[c->cmdindex];      
	chain_size = le32_to_cpu(cp->data_len);
	temp64 = pci_map_single(h->pdev, chain_block, chain_size,
				PCI_DMA_TODEVICE);
	if (hpsa_dma_mapping_error(&h->pdev->dev, temp64)) {
		/* prevent subsequent unmapping */
		cp->sg->address = 0;
		return -1;
	}
	cp->sg->address = (u64) cpu_to_le64(temp64);
	return 0;
}

static void hpsa_unmap_ioaccel2_sg_chain_block(struct ctlr_info *h,
	struct io_accel2_cmd *cp)
{
	struct ioaccel2_sg_element *chain_sg;
	u64 temp64;
	u32 chain_size;

	chain_sg = cp->sg;
	temp64 = le64_to_cpu(chain_sg->address);
	chain_size = le32_to_cpu(cp->data_len);
	pci_unmap_single(h->pdev, temp64, chain_size, PCI_DMA_TODEVICE);
}

static int hpsa_map_sg_chain_block(struct ctlr_info *h,
	struct CommandList *c)
{
	struct SGDescriptor *chain_sg, *chain_block;
	u64 temp64;
	u32 chain_len;

	chain_sg = &c->SG[h->max_cmd_sg_entries - 1];
	chain_block = h->cmd_sg_list[c->cmdindex];
	chain_sg->Ext = cpu_to_le32(HPSA_SG_CHAIN);
	chain_len = sizeof(*chain_sg) *
		(c->Header.SGTotal - h->max_cmd_sg_entries);
	chain_sg->Len = cpu_to_le32(chain_len);
	temp64 = pci_map_single(h->pdev, chain_block, chain_sg->Len,
				PCI_DMA_TODEVICE);
	if (hpsa_dma_mapping_error(&h->pdev->dev, temp64)) {
		/* prevent subsequent unmapping */
		chain_sg->Addr = 0;
		return -1;
	}
	chain_sg->Addr = cpu_to_le64(temp64);
	return 0;
}

static void hpsa_unmap_sg_chain_block(struct ctlr_info *h,
	struct CommandList *c)
{
	struct SGDescriptor *chain_sg;

	if (le16_to_cpu(c->Header.SGTotal) <= h->max_cmd_sg_entries)
		return;

	chain_sg = &c->SG[h->max_cmd_sg_entries - 1];
	pci_unmap_single(h->pdev, le64_to_cpu(chain_sg->Addr),
			le32_to_cpu(chain_sg->Len), PCI_DMA_TODEVICE);
}


/* Decode the various types of errors on ioaccel2 path. 
 * Return 1 for any error that should generate a RAID path retry.
 * Return 0 for errors that don't require a RAID path retry.
 */
static int handle_ioaccel_mode2_error(struct ctlr_info *h,
					struct CommandList *c,
					struct scsi_cmnd *cmd,
					struct io_accel2_cmd *c2)
{
	int data_len;
	int retry = 0;
	u32 ioaccel2_resid = 0;

	switch (c2->error_data.serv_response) {
	case IOACCEL2_SERV_RESPONSE_COMPLETE:
		switch (c2->error_data.status) {
		case IOACCEL2_STATUS_SR_TASK_COMP_GOOD:
			break;
		case IOACCEL2_STATUS_SR_TASK_COMP_CHK_COND:
			cmd->result |= SAM_STAT_CHECK_CONDITION;
			if (c2->error_data.data_present !=
					IOACCEL2_SENSE_DATA_PRESENT) {
				memset(cmd->sense_buffer, 0,
					SCSI_SENSE_BUFFERSIZE);
				break;
			}
			/* copy the sense data */
			data_len = c2->error_data.sense_data_len;
			if (data_len > SCSI_SENSE_BUFFERSIZE)
				data_len = SCSI_SENSE_BUFFERSIZE;
			if (data_len > sizeof(c2->error_data.sense_data_buff))
				data_len = sizeof(c2->error_data.sense_data_buff);
			memcpy(cmd->sense_buffer,
				c2->error_data.sense_data_buff, data_len);
			retry = 1;
			break;
		case IOACCEL2_STATUS_SR_TASK_COMP_BUSY:
			dev_warn(&h->pdev->dev, "%s: task complete with "
				"BUSY status.\n", "HP SSD Smart Path");
			retry = 1;
			break;
		case IOACCEL2_STATUS_SR_TASK_COMP_RES_CON:
			dev_warn(&h->pdev->dev, "%s: task complete with "
				"reservation conflict.\n", "HP SSD Smart Path");
			retry = 1;
			break;
		case IOACCEL2_STATUS_SR_TASK_COMP_SET_FULL:
			retry = 1;
			break;
		case IOACCEL2_STATUS_SR_TASK_COMP_ABORTED:
			dev_warn(&h->pdev->dev, "%s: task complete with "
				"aborted status.\n", "HP SSD Smart Path");
			retry = 1;
			break;
		default:
			dev_warn(&h->pdev->dev, "%s: task complete with "
				"unrecognized status: 0x%02x\n", 
				"HP SSD Smart Path", c2->error_data.status);
			retry = 1;
			break;
		}
		break;
	case IOACCEL2_SERV_RESPONSE_FAILURE:
		switch (c2->error_data.status) {
		case IOACCEL2_STATUS_SR_IO_ERROR:
		case IOACCEL2_STATUS_SR_IO_ABORTED:
		case IOACCEL2_STATUS_SR_OVERRUN:
			retry = 1;
			break;
		case IOACCEL2_STATUS_SR_UNDERRUN:
			cmd->result = (DID_OK << 16);
			cmd->result |= (COMMAND_COMPLETE << 8);
			ioaccel2_resid = c2->error_data.resid_cnt[3] << 24;
			ioaccel2_resid |= c2->error_data.resid_cnt[2] << 16;
			ioaccel2_resid |= c2->error_data.resid_cnt[1] << 8;
			ioaccel2_resid |= c2->error_data.resid_cnt[0];
			scsi_set_resid(cmd, ioaccel2_resid);
			break;
		case IOACCEL2_STATUS_SR_NO_PATH_TO_DEVICE:
		case IOACCEL2_STATUS_SR_INVALID_DEVICE:
		case IOACCEL2_STATUS_SR_IOACCEL_DISABLED:
			/* We will get an event from ctlr to trigger rescan */
			retry = 1;
			break;
		default:
			retry = 1;
			dev_warn(&h->pdev->dev,
				"unexpected delivery or target failure, "
				"status = 0x%02x\n", c2->error_data.status);
		}
		break;
	case IOACCEL2_SERV_RESPONSE_TMF_COMPLETE:
		break;
	case IOACCEL2_SERV_RESPONSE_TMF_SUCCESS:
		break;
	case IOACCEL2_SERV_RESPONSE_TMF_REJECTED:
		/* TODO: Figure out what to do here. */
		dev_warn(&h->pdev->dev, "task management function rejected.\n");
		break;
	case IOACCEL2_SERV_RESPONSE_TMF_WRONG_LUN:
		/* TODO: Figure out what to do here. */
		dev_warn(&h->pdev->dev, "task management function invalid LUN\n");
		break;
	default:
		dev_warn(&h->pdev->dev, "%s: Unrecognized TMF response: "
			"0x%02x\n", "HP SSD Smart Path", 
			c2->error_data.serv_response);
		retry = 1;
		break;
	}

	return retry;	/* retry on raid path? */
}

static void process_ioaccel2_completion(struct ctlr_info *h,
		struct CommandList *c, struct scsi_cmnd *cmd,
		struct hpsa_scsi_dev_t *dev)
{
	struct io_accel2_cmd *c2 = &h->ioaccel2_cmd_pool[c->cmdindex];

	if (c2->sg[0].chain_indicator == IOACCEL2_CHAIN)
		hpsa_unmap_ioaccel2_sg_chain_block(h, c2);	

	atomic_dec(&dev->ioaccel_cmds_out);

	/* check for good status */
	if (likely(c2->error_data.serv_response == 0 &&
			c2->error_data.status == 0)) {
		cmd_free(h, c);
		cmd->scsi_done(cmd);
		return;
	}

	/* Any RAID offload error results in retry which will use
	 * the normal I/O path so the controller can handle whatever's
	 * wrong.
	 */
	if (is_logical_dev_addr_mode(dev->scsi3addr) &&
		c2->error_data.serv_response ==
			IOACCEL2_SERV_RESPONSE_FAILURE) {
		if (c2->error_data.status ==
			IOACCEL2_STATUS_SR_IOACCEL_DISABLED)
			dev->offload_enabled = 0;
		goto retry_cmd;
	}
	if (handle_ioaccel_mode2_error(h, c, cmd, c2))
		goto retry_cmd;

	cmd_free(h, c);
	cmd->scsi_done(cmd);
	return;

retry_cmd:
	INIT_WORK(&c->work, hpsa_command_resubmit_worker);
	queue_work_on(raw_smp_processor_id(), h->resubmit_wq, &c->work);
}

/* Returns 0 on success, < 0 otherwise. */
static int hpsa_evaluate_tmf_status(struct ctlr_info *h,
					struct CommandList *cp)
{
	u8 tmf_status = cp->err_info->ScsiStatus;

	switch (tmf_status) {
	case CISS_TMF_COMPLETE:
		/* CISS_TMF_COMPLETE never happens, instead,
		 * ei->CommandStatus == 0 for this case.
		 */
	case CISS_TMF_SUCCESS:
		return 0;
	case CISS_TMF_INVALID_FRAME:
	case CISS_TMF_NOT_SUPPORTED:
	case CISS_TMF_FAILED:
	case CISS_TMF_WRONG_LUN:
	case CISS_TMF_OVERLAPPED_TAG:
		break;
	default:
		dev_warn(&h->pdev->dev, "Unknown TMF status: %02x\n",
				tmf_status);
		break;
	}
	return -tmf_status;
}

static void complete_scsi_command(struct CommandList *cp)
{
	struct scsi_cmnd *cmd;
	struct ctlr_info *h;
	struct ErrorInfo *ei;
	struct hpsa_scsi_dev_t *dev;

	int sense_key;
	int asc;      /* additional sense code */
	int ascq;     /* additional sense code qualifier */
	unsigned long sense_data_size;

	ei = cp->err_info;
	cmd = cp->scsi_cmd;
	h = cp->h;
	dev = cmd->device->hostdata;

	scsi_dma_unmap(cmd); /* undo the DMA mappings */
	if ((cp->cmd_type == CMD_SCSI) &&
		(cp->Header.SGTotal > h->max_cmd_sg_entries))
		hpsa_unmap_sg_chain_block(h, cp);

	cmd->result = (DID_OK << 16); 		/* host byte */
	cmd->result |= (COMMAND_COMPLETE << 8);	/* msg byte */

	if (cp->cmd_type == CMD_IOACCEL2)
		return process_ioaccel2_completion(h, cp, cmd, dev);

	scsi_set_resid(cmd, ei->ResidualCnt);
	if (ei->CommandStatus == 0) {
		cmd_free(h, cp);
		cmd->scsi_done(cmd);
		return;
	}

	/* For I/O accelerator commands, copy over some fields to the normal
	 * CISS header used below for error handling.
	 */
	if (cp->cmd_type == CMD_IOACCEL1) {
		struct io_accel1_cmd *c = &h->ioaccel_cmd_pool[cp->cmdindex];
		atomic_dec(&dev->ioaccel_cmds_out);
		cp->Header.SGList = cp->Header.SGTotal = scsi_sg_count(cmd);
		cp->Request.CDBLen = c->io_flags & IOACCEL1_IOFLAGS_CDBLEN_MASK;
		cp->Header.tag = c->tag;
		memcpy(cp->Header.LUN.LunAddrBytes, c->CISS_LUN, 8);
		memcpy(cp->Request.CDB, c->CDB, cp->Request.CDBLen);

		/* Any RAID offload error results in retry which will use
		 * the normal I/O path so the controller can handle whatever's
		 * wrong.
		 */
		if (is_logical_dev_addr_mode(dev->scsi3addr)) {
			if (ei->CommandStatus == CMD_IOACCEL_DISABLED)
				dev->offload_enabled = 0;
			INIT_WORK(&cp->work, hpsa_command_resubmit_worker);
			queue_work_on(raw_smp_processor_id(),
					h->resubmit_wq, &cp->work);
			return;
		}
	}

	/* an error has occurred */
	switch (ei->CommandStatus) {

	case CMD_TARGET_STATUS:
		cmd->result |= ei->ScsiStatus;
		/* copy the sense data */
		if (SCSI_SENSE_BUFFERSIZE < sizeof(ei->SenseInfo))
			sense_data_size = SCSI_SENSE_BUFFERSIZE;
		else
			sense_data_size = sizeof(ei->SenseInfo);
		if (ei->SenseLen < sense_data_size)
			sense_data_size = ei->SenseLen;
		memcpy(cmd->sense_buffer, ei->SenseInfo, sense_data_size);
		if (ei->ScsiStatus)
			decode_sense_data(ei->SenseInfo, sense_data_size,
				&sense_key, &asc, &ascq);
		if (ei->ScsiStatus == SAM_STAT_CHECK_CONDITION) {
			if (sense_key == ABORTED_COMMAND) {
				cmd->result |= DID_SOFT_ERROR << 16;
				break;
			}
			break;
		}
		/* Problem was not a check condition
		 * Pass it up to the upper layers...
		 */
		if (ei->ScsiStatus) {
			dev_warn(&h->pdev->dev, "cp %p has status 0x%x "
				"Sense: 0x%x, ASC: 0x%x, ASCQ: 0x%x, "
				"Returning result: 0x%x\n",
				cp, ei->ScsiStatus,
				sense_key, asc, ascq,
				cmd->result);
		} else {  /* scsi status is zero??? How??? */
			dev_warn(&h->pdev->dev, "cp %p SCSI status was 0. "
				"Returning no connection.\n", cp),

			/* Ordinarily, this case should never happen,
			 * but there is a bug in some released firmware
			 * revisions that allows it to happen if, for
			 * example, a 4100 backplane loses power and
			 * the tape drive is in it.  We assume that
			 * it's a fatal error of some kind because we
			 * can't show that it wasn't. We will make it
			 * look like selection timeout since that is
			 * the most common reason for this to occur,
			 * and it's severe enough.
			 */

			cmd->result = DID_NO_CONNECT << 16;
		}
		break;

	case CMD_DATA_UNDERRUN: /* let mid layer handle it. */
		break;
	case CMD_DATA_OVERRUN:
		dev_warn(&h->pdev->dev,
			"CDB " phN16 " data overrun\n", phNbytes16(cp->Request.CDB));
		break;
	case CMD_INVALID: {
		/* print_bytes(cp, sizeof(*cp), 1, 0);
		print_cmd(cp); */
		/* We get CMD_INVALID if you address a non-existent device
		 * instead of a selection timeout (no response).  You will
		 * see this if you yank out a drive, then try to access it.
		 * This is kind of a shame because it means that any other
		 * CMD_INVALID (e.g. driver bug) will get interpreted as a
		 * missing target. */
		cmd->result = DID_NO_CONNECT << 16;
	}
		break;
	case CMD_PROTOCOL_ERR:
		cmd->result = DID_ERROR << 16;
		dev_warn(&h->pdev->dev, "CDB " phN16 " : protocol error\n",
				phNbytes16(cp->Request.CDB));
		break;
	case CMD_HARDWARE_ERR:
		cmd->result = DID_ERROR << 16;
		dev_warn(&h->pdev->dev, "CDB " phN16 " : hardware error\n",
			phNbytes16(cp->Request.CDB));
		break;
	case CMD_CONNECTION_LOST:
		cmd->result = DID_ERROR << 16;
		dev_warn(&h->pdev->dev, "CDB " phN16 " : connection lost\n",
			phNbytes16(cp->Request.CDB));
		break;
	case CMD_ABORTED:
		cmd->result = DID_ABORT << 16;
		dev_warn(&h->pdev->dev, "CDB " phN16 " was aborted with status 0x%x\n",
				phNbytes16(cp->Request.CDB), ei->ScsiStatus);
		break;
	case CMD_ABORT_FAILED:
		cmd->result = DID_ERROR << 16;
		dev_warn(&h->pdev->dev, "CDB " phN16 " : abort failed\n",
			phNbytes16(cp->Request.CDB));
		break;
	case CMD_UNSOLICITED_ABORT:
		cmd->result = DID_SOFT_ERROR << 16; /* retry the command */
		dev_warn(&h->pdev->dev, "CDB " phN16 " : unsolicited abort\n",
			phNbytes16(cp->Request.CDB));
		break;
	case CMD_TIMEOUT:
		cmd->result = DID_TIME_OUT << 16;
		dev_warn(&h->pdev->dev, "CDB " phN16 " timedout\n",
			phNbytes16(cp->Request.CDB));
		break;
	case CMD_UNABORTABLE:
		cmd->result = DID_ERROR << 16;
		dev_warn(&h->pdev->dev, "Command unabortable\n");
		break;
	case CMD_TMF_STATUS:
		if (hpsa_evaluate_tmf_status(h, cp)) /* TMF failed? */
			cmd->result = DID_ERROR << 16;
		break;
	case CMD_IOACCEL_DISABLED:
		/* This only handles the direct pass-through case since RAID
		 * offload is handled above.  Just attempt a retry.
		 */
		cmd->result = DID_SOFT_ERROR << 16;
		dev_warn(&h->pdev->dev,
				"cp %p had HP SSD Smart Path error\n", cp);
		break;
	default:
		cmd->result = DID_ERROR << 16;
		dev_warn(&h->pdev->dev, "cp %p returned unknown status %x\n",
				cp, ei->CommandStatus);
	}

	/* Prevent the following race in the abort handler:
	 *
	 * 1. LLD is requested to abort a scsi command
	 * 2. scsi command completes
	 * 3. The struct CommandList associated with 2 is made available.
	 * 4. new io request to LLD to another LUN re-uses struct CommandList
	 * 5. abort handler follows scsi_cmnd->host_scribble and
	 *    finds struct CommandList and tries to aborts it.
	 * Now we have aborted the wrong command.
	 * Clear cp->scsi_cmd here so that if this get re-used, the abort
	 * handler will know it's a different scsi_cmnd.
	 */
	cp->scsi_cmd = NULL;

	cmd_free(h, cp);
	cmd->scsi_done(cmd);
}

static void hpsa_pci_unmap(struct pci_dev *pdev,
	struct CommandList *c, int sg_used, int data_direction)
{
	int i;

	for (i = 0; i < sg_used; i++)
		pci_unmap_single(pdev, (dma_addr_t) le64_to_cpu(c->SG[i].Addr),
				le32_to_cpu(c->SG[i].Len),
				data_direction);
}

static int hpsa_map_one(struct pci_dev *pdev,
		struct CommandList *cp,
		unsigned char *buf,
		size_t buflen,
		int data_direction)
{
	u64 addr64;

	if (buflen == 0 || data_direction == PCI_DMA_NONE) {
		cp->Header.SGList = 0;
		cp->Header.SGTotal = cpu_to_le16(0);
		return 0;
	}

	addr64 = (u64) pci_map_single(pdev, buf, buflen, data_direction);
	if (hpsa_dma_mapping_error(&pdev->dev, addr64)) {
		/* Prevent subsequent unmap of something never mapped */
		cp->Header.SGList = 0;
		cp->Header.SGTotal = cpu_to_le16(0);
		return -1;
	}
	cp->SG[0].Addr = cpu_to_le64(addr64);
	cp->SG[0].Len = cpu_to_le32(buflen);
	cp->SG[0].Ext = cpu_to_le32(HPSA_SG_LAST); /* we are not chaining */
	cp->Header.SGList = (u8) 1;   /* no. SGs contig in this cmd */
	cp->Header.SGTotal = (u16) cpu_to_le16(1); /* total sgs in cmd list */
	return 0;
}

#define NO_TIMEOUT ((unsigned long) -1)
#define DEFAULT_TIMEOUT (30000) /* milliseconds */
static int __hpsa_scsi_do_simple_cmd_core(struct ctlr_info *h,
	struct CommandList *c, int reply_queue, unsigned long timeout_msecs)
{
	DECLARE_COMPLETION_ONSTACK(wait);

	c->waiting = &wait;
	__enqueue_cmd_and_start_io(h, c, reply_queue);
	if (timeout_msecs == NO_TIMEOUT) {
		/* TODO: get rid of this no-timeout thing */
		wait_for_completion_io(&wait);
		return 0;
	}
	if (!wait_for_completion_io_timeout(&wait,
					msecs_to_jiffies(timeout_msecs))) {
		dev_warn(&h->pdev->dev, "Command timed out.\n");
		return -ETIMEDOUT;
	}
	return 0;
}

static int hpsa_scsi_do_simple_cmd_core(struct ctlr_info *h,
	struct CommandList *c, unsigned long timeout_msecs)
{
	return __hpsa_scsi_do_simple_cmd_core(h, c,
					DEFAULT_REPLY_QUEUE, timeout_msecs);
}

static u32 lockup_detected(struct ctlr_info *h)
{
	int cpu;
	u32 rc, *lockup_detected;

	cpu = get_cpu();
	lockup_detected = per_cpu_ptr(h->lockup_detected, cpu);
	rc = *lockup_detected;
	put_cpu();
	return rc;
}

static int hpsa_scsi_do_simple_cmd_core_if_no_lockup(struct ctlr_info *h,
	struct CommandList *c, unsigned long timeout_msecs)
{
	/* If controller lockup detected, fake a hardware error. */
	if (unlikely(lockup_detected(h))) {
		c->err_info->CommandStatus = CMD_HARDWARE_ERR;
		return 0;
	}
	return hpsa_scsi_do_simple_cmd_core(h, c, timeout_msecs);
}

#define MAX_DRIVER_CMD_RETRIES 25
static int hpsa_scsi_do_simple_cmd_with_retry(struct ctlr_info *h,
	struct CommandList *c, int data_direction, unsigned long timeout_msecs)
{
	int backoff_time = 10, retry_count = 0;
	int rc;

	do {
		memset(c->err_info, 0, sizeof(*c->err_info));
		rc = hpsa_scsi_do_simple_cmd_core(h, c, timeout_msecs);
		if (rc)
			break;
		retry_count++;
		if (retry_count > 3) {
			msleep(backoff_time);
			if (backoff_time < 1000)
				backoff_time *= 2;
		}
	} while ((check_for_unit_attention(h, c) ||
			check_for_busy(h, c)) &&
			retry_count <= MAX_DRIVER_CMD_RETRIES);
	hpsa_pci_unmap(h->pdev, c, 1, data_direction);
	if (retry_count > MAX_DRIVER_CMD_RETRIES)
		rc = -1; /* FIXME do something better? */
	return rc;
}

static void hpsa_scsi_interpret_error(struct CommandList *cp)
{
	const struct ErrorInfo *ei = cp->err_info;
	struct device *d = &cp->h->pdev->dev;
	int sense_key, asc, ascq, sense_len;

	switch (ei->CommandStatus) {
	case CMD_TARGET_STATUS:
		if (ei->SenseLen > sizeof(ei->SenseInfo))
			sense_len = sizeof(ei->SenseInfo);
		else
			sense_len = ei->SenseLen;
		decode_sense_data(ei->SenseInfo, sense_len,
					&sense_key, &asc, &ascq);
		if (ei->ScsiStatus == SAM_STAT_CHECK_CONDITION)
			dev_warn(d, "SCSI Status = 0x02, Sense key = 0x%02x, ASC = 0x%02x, ASCQ = 0x%02x\n",
				sense_key, asc, ascq);
		else
			dev_warn(d, "SCSI Status = 0x%02x\n", ei->ScsiStatus);
		if (ei->ScsiStatus == 0)
			dev_warn(d, "SCSI status is abnormally zero.  "
			"(probably indicates selection timeout "
			"reported incorrectly due to a known "
			"firmware bug, circa July, 2001.)\n");
		break;
	case CMD_DATA_UNDERRUN: /* let mid layer handle it. */
			dev_info(d, "UNDERRUN\n");
		break;
	case CMD_DATA_OVERRUN:
		dev_warn(d, "cp %p has completed with data overrun\n", cp);
		break;
	case CMD_INVALID: {
		/* controller unfortunately reports SCSI passthru's
		 * to non-existent targets as invalid commands.
		 */
		dev_warn(d, "cp %p is reported invalid (probably means "
			"target device no longer present)\n", cp);
		/* print_bytes((unsigned char *) cp, sizeof(*cp), 1, 0);
		print_cmd(cp);  */
		}
		break;
	case CMD_PROTOCOL_ERR:
		dev_warn(d, "cp %p has protocol error \n", cp);
		break;
	case CMD_HARDWARE_ERR:
		/* cmd->result = DID_ERROR << 16; */
		dev_warn(d, "cp %p had hardware error\n", cp);
		break;
	case CMD_CONNECTION_LOST:
		dev_warn(d, "cp %p had connection lost\n", cp);
		break;
	case CMD_ABORTED:
		dev_warn(d, "cp %p was aborted\n", cp);
		break;
	case CMD_ABORT_FAILED:
		dev_warn(d, "cp %p reports abort failed\n", cp);
		break;
	case CMD_UNSOLICITED_ABORT:
		dev_warn(d, "cp %p aborted due to an unsolicited abort\n", cp);
		break;
	case CMD_TIMEOUT:
		dev_warn(d, "cp %p timed out\n", cp);
		break;
	case CMD_UNABORTABLE:
		dev_warn(d, "Command unabortable\n");
		break;
	default:
		dev_warn(d, "cp %p returned unknown status %x\n", cp,
				ei->CommandStatus);
	}
}

static int hpsa_scsi_do_inquiry(struct ctlr_info *h, unsigned char *scsi3addr,
			u16 page, unsigned char *buf,
			unsigned char bufsize)
{
	int rc = IO_OK;
	struct CommandList *c;
	struct ErrorInfo *ei;

	c = cmd_alloc(h);
	if (!c) {			/* trouble... */
		dev_warn(&h->pdev->dev, "cmd_alloc returned NULL!\n");
		return -ENOMEM;
	}

	if (fill_cmd(c, HPSA_INQUIRY, h, buf, bufsize,
			page, scsi3addr, TYPE_CMD)) {
		rc = -1;
		goto out;
	}
	rc = hpsa_scsi_do_simple_cmd_with_retry(h, c,
		PCI_DMA_FROMDEVICE, NO_TIMEOUT);
	if (rc)
		goto out;
	ei = c->err_info;
	if (ei->CommandStatus != 0 && ei->CommandStatus != CMD_DATA_UNDERRUN) {
		hpsa_scsi_interpret_error(c);
		rc = -1;
	}
out:
	cmd_free(h, c);
	return rc;
}


static int hpsa_bmic_ctrl_mode_sense(struct ctlr_info *h,
			unsigned char *scsi3addr, unsigned char page,
			struct bmic_controller_parameters *buf, size_t bufsize)
{
	int rc = IO_OK;
	struct CommandList *c;
	struct ErrorInfo *ei;

	c = cmd_alloc(h);

	if (c == NULL) {			/* trouble... */
		dev_warn(&h->pdev->dev, "cmd_alloc returned NULL!\n");
		return -ENOMEM;
	}

	if (fill_cmd(c, BMIC_SENSE_CONTROLLER_PARAMETERS, h, buf, bufsize,
			page, scsi3addr, TYPE_CMD)) {
		rc = -1;
		goto out;
	}
	rc = hpsa_scsi_do_simple_cmd_with_retry(h, c,
					PCI_DMA_FROMDEVICE, NO_TIMEOUT);
	if (rc)
		goto out;
	ei = c->err_info;
	if (ei->CommandStatus != 0 && ei->CommandStatus != CMD_DATA_UNDERRUN) {
		hpsa_scsi_interpret_error(c);
		rc = -1;
	}
out:
	cmd_free(h, c);
	return rc;
}

static int hpsa_send_reset(struct ctlr_info *h, unsigned char *scsi3addr, 
	int reset_type)
{
	int rc = IO_OK;
	struct CommandList *c;
	struct ErrorInfo *ei;

	c = cmd_alloc(h);
	if (c == NULL) {			/* trouble... */
		dev_warn(&h->pdev->dev, "cmd_alloc returned NULL!\n");
		return -ENOMEM;
	}

	/* fill_cmd can't fail here, no data buffer to map. */
	(void) fill_cmd(c, reset_type, h, NULL, 0, 0, scsi3addr, TYPE_MSG);
	rc = hpsa_scsi_do_simple_cmd_core(h, c, NO_TIMEOUT);
	if (rc) {
		dev_warn(&h->pdev->dev, "Failed to send reset command\n");
		goto out;
	}
	/* no unmap needed here because no data xfer. */

	ei = c->err_info;
	if (ei->CommandStatus != 0) {
		hpsa_scsi_interpret_error(c);
		rc = -1;
	}
out:
	cmd_free(h, c);
	return rc;
}

static void hpsa_get_raid_level(struct ctlr_info *h,
	unsigned char *scsi3addr, unsigned char *raid_level)
{
	int rc;
	unsigned char *buf;

	*raid_level = RAID_UNKNOWN;
	buf = kzalloc(64, GFP_KERNEL);
	if (!buf)
		return;
	rc = hpsa_scsi_do_inquiry(h, scsi3addr, VPD_PAGE | 0xC1, buf, 64);
	if (rc == 0)
		*raid_level = buf[8];
	if (*raid_level > RAID_UNKNOWN)
		*raid_level = RAID_UNKNOWN;
	kfree(buf);
	return;
}

#define HPSA_MAP_DEBUG
#ifdef HPSA_MAP_DEBUG
static void hpsa_debug_map_buff(struct ctlr_info *h, int rc,
				struct raid_map_data *map_buff)
{
	struct raid_map_disk_data *dd = &map_buff->data[0];
	int map, row, col;
	u16 map_cnt, row_cnt, disks_per_row;

	if (rc != 0)
		return;

	/* Show details only if debugging has been activated. */
	if (h->raid_offload_debug < 2)
		return;

	dev_info(&h->pdev->dev, "structure_size = %u\n",
				le32_to_cpu(map_buff->structure_size));
	dev_info(&h->pdev->dev, "volume_blk_size = %u\n",
			le32_to_cpu(map_buff->volume_blk_size));
	dev_info(&h->pdev->dev, "volume_blk_cnt = 0x%llx\n",
			le64_to_cpu(map_buff->volume_blk_cnt));
	dev_info(&h->pdev->dev, "physicalBlockShift = %u\n",
			map_buff->phys_blk_shift);
	dev_info(&h->pdev->dev, "parity_rotation_shift = %u\n",
			map_buff->parity_rotation_shift);
	dev_info(&h->pdev->dev, "strip_size = %u\n",
			le16_to_cpu(map_buff->strip_size));
	dev_info(&h->pdev->dev, "disk_starting_blk = 0x%llx\n",
			le64_to_cpu(map_buff->disk_starting_blk));
	dev_info(&h->pdev->dev, "disk_blk_cnt = 0x%llx\n",
			le64_to_cpu(map_buff->disk_blk_cnt));
	dev_info(&h->pdev->dev, "data_disks_per_row = %u\n",
			le16_to_cpu(map_buff->data_disks_per_row));
	dev_info(&h->pdev->dev, "metadata_disks_per_row = %u\n",
			le16_to_cpu(map_buff->metadata_disks_per_row));
	dev_info(&h->pdev->dev, "row_cnt = %u\n",
			le16_to_cpu(map_buff->row_cnt));
	dev_info(&h->pdev->dev, "layout_map_count = %u\n",
			le16_to_cpu(map_buff->layout_map_count));
	dev_info(&h->pdev->dev, "flags = %u\n",
			le16_to_cpu(map_buff->flags));
	if (map_buff->flags & RAID_MAP_FLAG_ENCRYPT_ON ) 
		dev_info(&h->pdev->dev, "encrypytion = ON\n");
	else 	
		dev_info(&h->pdev->dev, "encrypytion = OFF\n");
	dev_info(&h->pdev->dev, "dekindex = %u\n",
			le16_to_cpu(map_buff->dekindex));

	map_cnt = le16_to_cpu(map_buff->layout_map_count);
	for (map = 0; map < map_cnt; map++) {
		dev_info(&h->pdev->dev, "Map%u:\n", map);
		row_cnt = le16_to_cpu(map_buff->row_cnt);
		for (row = 0; row < row_cnt; row++) {
			dev_info(&h->pdev->dev, "  Row%u:\n", row);
			disks_per_row = le16_to_cpu(map_buff->data_disks_per_row);
			for (col = 0; col < disks_per_row; col++, dd++)
				dev_info(&h->pdev->dev,
					"    D%02u: h=0x%04x xor=%u,%u\n",
					col, dd->ioaccel_handle,
					dd->xor_mult[0], dd->xor_mult[1]);
			disks_per_row =
				le16_to_cpu(map_buff->metadata_disks_per_row);
			for (col = 0; col < disks_per_row; col++, dd++)
				dev_info(&h->pdev->dev,
					"    M%02u: h=0x%04x xor=%u,%u\n",
					col, dd->ioaccel_handle,
					dd->xor_mult[0], dd->xor_mult[1]);
		}
	}
}
#else
static void hpsa_debug_map_buff(__attribute__((unused)) struct ctlr_info *h,
			__attribute__((unused)) int rc,
			__attribute__((unused)) struct raid_map_data *map_buff)
{
}
#endif

static int hpsa_get_raid_map(struct ctlr_info *h,
	unsigned char *scsi3addr, struct hpsa_scsi_dev_t *this_device)
{
	int rc = 0;
	struct CommandList *c;
	struct ErrorInfo *ei;

	c = cmd_alloc(h);
	if (c == NULL) {
		dev_warn(&h->pdev->dev, "cmd_alloc returned NULL!\n");
		return -ENOMEM;
	}
	if (fill_cmd(c, HPSA_GET_RAID_MAP, h, &this_device->raid_map,
			sizeof(this_device->raid_map), 0, scsi3addr, TYPE_CMD)) {
		cmd_free(h, c);
		return -ENOMEM;
	}
	rc = hpsa_scsi_do_simple_cmd_with_retry(h, c,
			PCI_DMA_FROMDEVICE, NO_TIMEOUT);
	if (rc)
		goto out;
	ei = c->err_info;
	if (ei->CommandStatus != 0 && ei->CommandStatus != CMD_DATA_UNDERRUN) {
		hpsa_scsi_interpret_error(c);
		cmd_free(h, c);
		return -1;
	}
out:
	cmd_free(h, c);

	/* @todo in the future, dynamically allocate RAID map memory */
	if (le32_to_cpu(this_device->raid_map.structure_size) >
				sizeof(this_device->raid_map)) {
		dev_warn(&h->pdev->dev, "RAID map size is too large!\n");
		rc = -1;
	}
	hpsa_debug_map_buff(h, rc, &this_device->raid_map);
	return rc;
}

static int hpsa_bmic_id_physical_device(struct ctlr_info *h,
		unsigned char scsi3addr[], u16 bmic_device_index,
		struct bmic_identify_physical_device *buf, size_t bufsize)
{
	int rc = IO_OK;
	struct CommandList *c;
	struct ErrorInfo *ei;

	c = cmd_alloc(h);

	if (c == NULL) {			/* trouble... */
		dev_warn(&h->pdev->dev, "cmd_alloc returned NULL!\n");
		return -ENOMEM;
	}

	rc = fill_cmd(c, BMIC_IDENTIFY_PHYSICAL_DEVICE, h, buf, bufsize,
		0, RAID_CTLR_LUNID, TYPE_CMD);
	if (rc)
		goto out;

	c->Request.CDB[2] = bmic_device_index & 0xff;
	c->Request.CDB[9] = (bmic_device_index >> 8) & 0xff;

	rc = hpsa_scsi_do_simple_cmd_with_retry(h, c, PCI_DMA_FROMDEVICE, NO_TIMEOUT);
	if (rc)
		goto out;
	ei = c->err_info;
	if (ei->CommandStatus != 0 && ei->CommandStatus != CMD_DATA_UNDERRUN) {
		hpsa_scsi_interpret_error(c);
		rc = -1;
	}
out:
	cmd_free(h, c);
	return rc;
}

static int hpsa_vpd_page_supported(struct ctlr_info *h,
	unsigned char scsi3addr[], u8 page)
{
        int rc;
	int i;
	int pages;
        unsigned char *buf, bufsize;

        buf = kzalloc(256, GFP_KERNEL);
        if (!buf)
		return 0;

	/* Get the size of the page list first */
        rc = hpsa_scsi_do_inquiry(h, scsi3addr,
				VPD_PAGE | HPSA_VPD_SUPPORTED_PAGES,
				buf, HPSA_VPD_HEADER_SZ);
        if (rc != 0)
		goto exit_unsupported;
	pages = buf[3];
	if ((pages + HPSA_VPD_HEADER_SZ) <= 255)
		bufsize = pages + HPSA_VPD_HEADER_SZ;
	else
		bufsize = 255;

	/* Get the whole VPD page list */
        rc = hpsa_scsi_do_inquiry(h, scsi3addr,
				VPD_PAGE | HPSA_VPD_SUPPORTED_PAGES,
				buf, bufsize);
        if (rc != 0)
		goto exit_unsupported;

	pages = buf[3];
	for (i = 1; i <= pages; i++)
		if (buf[3 + i] == page)
			goto exit_supported;
exit_unsupported:
	kfree(buf);
	return 0;
exit_supported:
	kfree(buf);
	return 1;
}

static void hpsa_get_ioaccel_status(struct ctlr_info *h,
	unsigned char *scsi3addr, struct hpsa_scsi_dev_t *this_device)
{
	int rc;
	unsigned char *buf;
	u8 ioaccel_status;

	this_device->offload_config = 0;
	this_device->offload_enabled = 0;

	buf = kzalloc(64, GFP_KERNEL);
	if (!buf)
		return;
	if (!hpsa_vpd_page_supported(h, scsi3addr, HPSA_VPD_LV_IOACCEL_STATUS))
		goto out;
	rc = hpsa_scsi_do_inquiry(h, scsi3addr,
			VPD_PAGE | HPSA_VPD_LV_IOACCEL_STATUS, buf, 64);
	if (rc != 0)
		goto out;

#define IOACCEL_STATUS_BYTE 4
#define OFFLOAD_CONFIGURED_BIT 0x01
#define OFFLOAD_ENABLED_BIT 0x02
	ioaccel_status = buf[IOACCEL_STATUS_BYTE];
	this_device->offload_config =
		!!(ioaccel_status & OFFLOAD_CONFIGURED_BIT);
	if (this_device->offload_config) {
		this_device->offload_enabled =
			!!(ioaccel_status & OFFLOAD_ENABLED_BIT);
		if (hpsa_get_raid_map(h, scsi3addr, this_device))
			this_device->offload_enabled = 0;
	}
out:
	kfree(buf);
	return;
}

/* Get the device id from inquiry page 0x83 */
static int hpsa_get_device_id(struct ctlr_info *h, unsigned char *scsi3addr,
	unsigned char *device_id, int buflen)
{
	int rc;
	unsigned char *buf;

	if (buflen > 16)
		buflen = 16;
	buf = kzalloc(64, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	rc = hpsa_scsi_do_inquiry(h, scsi3addr, VPD_PAGE | 0x83, buf, 64);
	if (rc == 0)
		memcpy(device_id, &buf[8], buflen);
	kfree(buf);
	return rc != 0;
}

static int hpsa_scsi_do_report_luns(struct ctlr_info *h, int logical,
		void *buf, int bufsize,
		int extended_response)
{
	int rc = IO_OK;
	struct CommandList *c;
	unsigned char scsi3addr[8];
	struct ErrorInfo *ei;

	c = cmd_alloc(h);
	if (c == NULL) {			/* trouble... */
		dev_err(&h->pdev->dev, "cmd_alloc returned NULL!\n");
		return -1;
	}
	/* address the controller */
	memset(scsi3addr, 0, sizeof(scsi3addr));
	if (fill_cmd(c, logical ? HPSA_REPORT_LOG : HPSA_REPORT_PHYS, h,
		buf, bufsize, 0, scsi3addr, TYPE_CMD)) {
		rc = -1;
		goto out;
	}
	if (extended_response)
		c->Request.CDB[1] = extended_response;
	rc = hpsa_scsi_do_simple_cmd_with_retry(h, c,
					PCI_DMA_FROMDEVICE, NO_TIMEOUT);
	if (rc)
		goto out;
	ei = c->err_info;
	if (ei->CommandStatus != 0 &&
	    ei->CommandStatus != CMD_DATA_UNDERRUN) {
		hpsa_scsi_interpret_error(c);
		rc = -1;
	} else {
		struct ReportLUNdata *rld = buf;
		if (rld->extended_response_flag != extended_response) {
			dev_err(&h->pdev->dev,
				"report luns requested format %u, got %u\n",
				extended_response,
				rld->extended_response_flag);
			rc = -1;
		}
	}
out:
	cmd_free(h, c);
	return rc;
}

static inline int hpsa_scsi_do_report_phys_luns(struct ctlr_info *h,
		struct ReportExtendedLUNdata *buf, int bufsize)
{
	return hpsa_scsi_do_report_luns(h, 0, buf, bufsize,
						HPSA_REPORT_PHYS_EXTENDED);
}

static inline int hpsa_scsi_do_report_log_luns(struct ctlr_info *h,
		struct ReportLUNdata *buf, int bufsize)
{
	return hpsa_scsi_do_report_luns(h, 1, buf, bufsize, 0);
}

static inline void hpsa_set_bus_target_lun(struct hpsa_scsi_dev_t *device,
	int bus, int target, int lun)
{
	device->bus = bus;
	device->target = target;
	device->lun = lun;
}

/* Use VPD inquiry to get details of volume status */
static int hpsa_get_volume_status(struct ctlr_info *h,
	unsigned char scsi3addr[])
{
	int rc;
        int status;
	int size;
        unsigned char *buf;

        buf = kzalloc(64, GFP_KERNEL);
        if (!buf)
		return HPSA_VPD_LV_STATUS_UNSUPPORTED;

	/* Does controller have VPD for logical volume status? */
	if (!hpsa_vpd_page_supported(h, scsi3addr, HPSA_VPD_LV_STATUS))
		goto exit_failed;

	/* Get the size of the VPD return buffer */
        rc = hpsa_scsi_do_inquiry(h, scsi3addr, VPD_PAGE | HPSA_VPD_LV_STATUS,
					buf, HPSA_VPD_HEADER_SZ);
	if (rc != 0)
		goto exit_failed;
	size = buf[3];

	/* Now get the whole VPD buffer */
        rc = hpsa_scsi_do_inquiry(h, scsi3addr, VPD_PAGE | HPSA_VPD_LV_STATUS,
					buf, size + HPSA_VPD_HEADER_SZ);
	if (rc != 0)
		goto exit_failed;
	status = buf[4]; /* status byte */

	kfree(buf);
	return status;
exit_failed:
        kfree(buf);
	return HPSA_VPD_LV_STATUS_UNSUPPORTED;
}

/* Determine offline status of a volume.
 * Return either:
 *  0 (not offline)
 *  0xff (offline for unknown reasons)
 *  # (integer code indicating one of several NOT READY states
 *     describing why a volume is to be kept offline)
 */
static int hpsa_volume_offline(struct ctlr_info *h,
		unsigned char scsi3addr[])
{
	struct CommandList *c;
	unsigned char *sense;
	int sense_key, asc, ascq, sense_len;
	int rc, ldstat = 0;
	u16 cmd_status;
	u8 scsi_status;
#define ASC_LUN_NOT_READY 0x04
#define ASCQ_LUN_NOT_READY_FORMAT_IN_PROGRESS 0x04
#define ASCQ_LUN_NOT_READY_INITIALIZING_CMD_REQ 0x02

	c = cmd_alloc(h);
	if (!c)
		return 0;
	fill_cmd(c, TEST_UNIT_READY, h, NULL, 0, 0, scsi3addr, TYPE_CMD);
	rc = hpsa_scsi_do_simple_cmd_core(h, c, NO_TIMEOUT);
	if (rc) {
		cmd_free(h, c);
		return 0;
	}
	sense = c->err_info->SenseInfo;
	if (c->err_info->SenseLen > sizeof(c->err_info->SenseInfo))
		sense_len = sizeof(c->err_info->SenseInfo);
	else
		sense_len = c->err_info->SenseLen;
	decode_sense_data(sense, sense_len, &sense_key, &asc, &ascq);
	cmd_status = c->err_info->CommandStatus;
	scsi_status = c->err_info->ScsiStatus;
	cmd_free(h, c);

	/* Is the volume 'not ready'? */
	if (cmd_status != CMD_TARGET_STATUS || 
		scsi_status != SAM_STAT_CHECK_CONDITION || 
		sense_key != NOT_READY || 
		asc != ASC_LUN_NOT_READY)  {
		return 0;
	}
		
	/* Determine the reason for not ready state */
	ldstat = hpsa_get_volume_status(h, scsi3addr);

	/* Keep volume offline in certain cases: */
	switch (ldstat) {
		case HPSA_LV_UNDERGOING_ERASE:
		case HPSA_LV_UNDERGOING_RPI:
		case HPSA_LV_PENDING_RPI:
		case HPSA_LV_ENCRYPTED_NO_KEY:
		case HPSA_LV_PLAINTEXT_IN_ENCRYPT_ONLY_CONTROLLER:
		case HPSA_LV_UNDERGOING_ENCRYPTION:
		case HPSA_LV_UNDERGOING_ENCRYPTION_REKEYING:
		case HPSA_LV_ENCRYPTED_IN_NON_ENCRYPTED_CONTROLLER:
			return ldstat;
		case HPSA_VPD_LV_STATUS_UNSUPPORTED:
			/* If VPD status page isn't available,
			 * use ASC/ASCQ to determine state
			 */
			if ((ascq == ASCQ_LUN_NOT_READY_FORMAT_IN_PROGRESS) ||
				(ascq == ASCQ_LUN_NOT_READY_INITIALIZING_CMD_REQ))
				return ldstat;
			break;
		default:
			break;
	}
	return 0;
}

/* Find out if a logical device supports aborts by simply trying one.
 * Smart Array may claim not to support aborts on logical drives, but
 * if a MSA2000 * is connected, the drives on that will be presented
 * by the Smart Array as logical drives, and aborts may be sent to
 * those devices successfully.  So the simplest way to find out is
 * to simply try an abort and see how the device responds.
 */
static int hpsa_device_supports_aborts(struct ctlr_info *h,
					unsigned char *scsi3addr)
{
	struct CommandList *c;
	struct ErrorInfo *ei;
	int rc = 0;

	u64 tag = (u64) -1; /* bogus tag */

	/* Assume that physical devices support aborts */
	if (!is_logical_dev_addr_mode(scsi3addr))
		return 1;

	c = cmd_alloc(h);
	if (!c)
		return -ENOMEM;
	(void) fill_cmd(c, HPSA_ABORT_MSG, h, &tag, 0, 0, scsi3addr, TYPE_MSG);
	(void) __hpsa_scsi_do_simple_cmd_core(h, c, 0, NO_TIMEOUT);
	/* no unmap needed here because no data xfer. */
	ei = c->err_info;
	switch (ei->CommandStatus) {
	case CMD_INVALID:
		rc = 0;
		break;
	case CMD_UNABORTABLE:
	case CMD_ABORT_FAILED:
		rc = 1;
		break;
	case CMD_TMF_STATUS:
		rc = hpsa_evaluate_tmf_status(h, c);
		break;
	default:
		rc = 0;
		break;
	}
	cmd_free(h, c);
	return rc;
}

static int hpsa_update_device_info(struct ctlr_info *h,
	unsigned char scsi3addr[], struct hpsa_scsi_dev_t *this_device,
	unsigned char *is_OBDR_device)
{

#define OBDR_SIG_OFFSET 43
#define OBDR_TAPE_SIG "$DR-10"
#define OBDR_SIG_LEN (sizeof(OBDR_TAPE_SIG) - 1)
#define OBDR_TAPE_INQ_SIZE (OBDR_SIG_OFFSET + OBDR_SIG_LEN)

	unsigned char *inq_buff;
	unsigned char *obdr_sig;
	unsigned long flags;
	int entry;
	int rc = 0;

	inq_buff = kzalloc(OBDR_TAPE_INQ_SIZE, GFP_KERNEL);
	if (!inq_buff) {
		rc = -ENOMEM;
		goto bail_out;
	}

	/* Do an inquiry to the device to see what it is. */
	if (hpsa_scsi_do_inquiry(h, scsi3addr, 0, inq_buff,
		(unsigned char) OBDR_TAPE_INQ_SIZE) != 0) {
		/* Inquiry failed (msg printed already) */
		dev_err(&h->pdev->dev,
			"hpsa_update_device_info: inquiry failed\n");
		rc = 1;
		goto bail_out;
	}

	this_device->devtype = (inq_buff[0] & 0x1f);
	memcpy(this_device->scsi3addr, scsi3addr, 8);
	memcpy(this_device->vendor, &inq_buff[8],
		sizeof(this_device->vendor));
	memcpy(this_device->model, &inq_buff[16],
		sizeof(this_device->model));
	memset(this_device->device_id, 0,
		sizeof(this_device->device_id));
	hpsa_get_device_id(h, scsi3addr, this_device->device_id,
		sizeof(this_device->device_id));

	if (this_device->devtype == TYPE_DISK &&
		is_logical_dev_addr_mode(scsi3addr)) {
		int volume_offline;

		hpsa_get_raid_level(h, scsi3addr, &this_device->raid_level);
		if (h->fw_support & MISC_FW_RAID_OFFLOAD_BASIC)
			hpsa_get_ioaccel_status(h, scsi3addr, this_device);
		volume_offline = hpsa_volume_offline(h, scsi3addr);
		if (volume_offline < 0 || volume_offline > 0xff)
			volume_offline = HPSA_VPD_LV_STATUS_UNSUPPORTED;
		this_device->volume_offline = volume_offline & 0xff;
	} else {
		this_device->raid_level = RAID_UNKNOWN;
		this_device->offload_config = 0;
		this_device->offload_enabled = 0;
		this_device->hba_ioaccel_enabled = 0;
		this_device->volume_offline = 0;
	}

	if (is_OBDR_device) {
		/* See if this is a One-Button-Disaster-Recovery device
		 * by looking for "$DR-10" at offset 43 in inquiry data.
		 */
		obdr_sig = &inq_buff[OBDR_SIG_OFFSET];
		*is_OBDR_device = (this_device->devtype == TYPE_ROM &&
					strncmp(obdr_sig, OBDR_TAPE_SIG,
						OBDR_SIG_LEN) == 0);
	}
	kfree(inq_buff);

	/*
	 * See if this device supports aborts.  If we already know
	 * the device, we already know if it supports aborts, otherwise
	 * we have to find out if it supports aborts by trying one.
	 */
	spin_lock_irqsave(&h->devlock, flags);
	rc = hpsa_scsi_find_entry(this_device, h->dev, h->ndevices, &entry);
	if ((rc == DEVICE_SAME || rc == DEVICE_UPDATED) &&
		entry >= 0 && entry < h->ndevices) {
		this_device->supports_aborts = h->dev[entry]->supports_aborts;
		spin_unlock_irqrestore(&h->devlock, flags);
	} else {
		spin_unlock_irqrestore(&h->devlock, flags);
		this_device->supports_aborts =
				hpsa_device_supports_aborts(h, scsi3addr);
		if (this_device->supports_aborts < 0)
			this_device->supports_aborts = 0;
	}
	return 0;

bail_out:
	kfree(inq_buff);
	return rc;
}

static unsigned char *ext_target_model[] = {
	"MSA2012",
	"MSA2024",
	"MSA2312",
	"MSA2324",
	"P2000 G3 SAS",
	"MSA 2040 SAS",
	NULL,
};

static int is_ext_target(struct ctlr_info *h, struct hpsa_scsi_dev_t *device)
{
	int i;

	for (i = 0; ext_target_model[i]; i++)
		if (strncmp(device->model, ext_target_model[i],
			strlen(ext_target_model[i])) == 0)
			return 1;
	return 0;
}

/* Helper function to assign bus, target, lun mapping of devices.
 * Puts non-external target logical volumes on bus 0, external target logical
 * volumes on bus 1, physical devices on bus 2. and the hba on bus 3.
 * Logical drive target and lun are assigned at this time, but
 * physical device lun and target assignment are deferred (assigned
 * in hpsa_find_target_lun, called by hpsa_scsi_add_entry.)
 */
static void figure_bus_target_lun(struct ctlr_info *h,
	u8 *lunaddrbytes, struct hpsa_scsi_dev_t *device)
{
	u32 lunid = le32_to_cpu(*((__le32 *) lunaddrbytes));

	if (!is_logical_dev_addr_mode(lunaddrbytes)) {
		/* physical device, target and lun filled in later */
		if (is_hba_lunid(lunaddrbytes))
			hpsa_set_bus_target_lun(device, 3, 0, lunid & 0x3fff);
		else
			/* defer target, lun assignment for physical devices */
			hpsa_set_bus_target_lun(device, 2, -1, -1);
		return;
	}
	/* It's a logical device */
	if (is_ext_target(h, device)) {
		/* external target way, put logicals on bus 1
		 * and match target/lun numbers box
		 * reports, other smart array, bus 0, target 0, match lunid
		 */
		hpsa_set_bus_target_lun(device,
			1, (lunid >> 16) & 0x3fff, lunid & 0x00ff);
		return;
	}
	hpsa_set_bus_target_lun(device, 0, 0, lunid & 0x3fff);
}

/*
 * If there is no lun 0 on a target, linux won't find any devices.
 * For the external targets (arrays), we have to manually detect the enclosure
 * which is at lun zero, as CCISS_REPORT_PHYSICAL_LUNS doesn't report
 * it for some reason.  *tmpdevice is the target we're adding,
 * this_device is a pointer into the current element of currentsd[]
 * that we're building up in update_scsi_devices(), below.
 * lunzerobits is a bitmap that tracks which targets already have a
 * lun 0 assigned.
 * Returns 1 if an enclosure was added, 0 if not.
 */
static int add_ext_target_dev(struct ctlr_info *h,
	struct hpsa_scsi_dev_t *tmpdevice,
	struct hpsa_scsi_dev_t *this_device, u8 *lunaddrbytes,
	unsigned long lunzerobits[], int *n_ext_target_devs)
{
	unsigned char scsi3addr[8];

	if (test_bit(tmpdevice->target, lunzerobits))
		return 0; /* There is already a lun 0 on this target. */

	if (!is_logical_dev_addr_mode(lunaddrbytes))
		return 0; /* It's the logical targets that may lack lun 0. */

	if (!is_ext_target(h, tmpdevice))
		return 0; /* Only external target devices have this problem. */

	if (tmpdevice->lun == 0) /* if lun is 0, then we have a lun 0. */
		return 0;

	memset(scsi3addr, 0, 8);
	scsi3addr[3] = tmpdevice->target;
	if (is_hba_lunid(scsi3addr))
		return 0; /* Don't add the RAID controller here. */

	if (is_scsi_rev_5(h))
		return 0; /* p1210m doesn't need to do this. */

	if (*n_ext_target_devs >= MAX_EXT_TARGETS) {
		dev_warn(&h->pdev->dev, "Maximum number of external "
			"target devices exceeded.  Check your hardware "
			"configuration.");
		return 0;
	}

	if (hpsa_update_device_info(h, scsi3addr, this_device, NULL))
		return 0;
	(*n_ext_target_devs)++;
	hpsa_set_bus_target_lun(this_device,
				tmpdevice->bus, tmpdevice->target, 0);
	set_bit(tmpdevice->target, lunzerobits);
	return 1;
}

/*
 * Get address of physical disk used for an ioaccel2 mode command:
 * 	1. Extract ioaccel2 handle from the command.
	2. Find a matching ioaccel2 handle from list of physical disks.
 *	3. Return: 	
 * 		1 and set scsi3addr to address of matching physical
 * 		0 if no matching physical disk was found.
 */
static int hpsa_get_pdisk_of_ioaccel2(struct ctlr_info *h,
	struct CommandList *ioaccel2_cmd_to_abort, unsigned char *scsi3addr)
{
	struct io_accel2_cmd *c2 =
			&h->ioaccel2_cmd_pool[ioaccel2_cmd_to_abort->cmdindex];
	unsigned long flags;
	int i;

	spin_lock_irqsave(&h->devlock, flags);
	for (i = 0; i < h->ndevices; i++)
		if (h->dev[i]->ioaccel_handle == c2->scsi_nexus) {
			memcpy(scsi3addr, h->dev[i]->scsi3addr,
				sizeof(h->dev[i]->scsi3addr));
			spin_unlock_irqrestore(&h->devlock, flags);
			return 1;
		}
	spin_unlock_irqrestore(&h->devlock, flags);
	return 0;
}

/*
 * Do CISS_REPORT_PHYS and CISS_REPORT_LOG.  Data is returned in physdev,
 * logdev.  The number of luns in physdev and logdev are returned in
 * *nphysicals and *nlogicals, respectively.
 * Returns 0 on success, -1 otherwise.
 */
static int hpsa_gather_lun_info(struct ctlr_info *h,
	struct ReportExtendedLUNdata *physdev, u32 *nphysicals,
	struct ReportLUNdata *logdev, u32 *nlogicals)
{
	if (hpsa_scsi_do_report_phys_luns(h, physdev, sizeof(*physdev))) {
		dev_err(&h->pdev->dev, "report physical LUNs failed.\n");
		return -1;
	}
	*nphysicals = be32_to_cpu(*((__be32 *)physdev->LUNListLength)) / 24;
	if (*nphysicals > HPSA_MAX_PHYS_LUN) {
		dev_warn(&h->pdev->dev, "maximum physical LUNs (%d) exceeded."
			"  %d LUNs ignored.\n", HPSA_MAX_PHYS_LUN,
			*nphysicals - HPSA_MAX_PHYS_LUN);
		*nphysicals = HPSA_MAX_PHYS_LUN;
	}
	if (hpsa_scsi_do_report_log_luns(h, logdev, sizeof(*logdev))) {
		dev_err(&h->pdev->dev, "report logical LUNs failed.\n");
		return -1;
	}
	*nlogicals = be32_to_cpu(*((__be32 *) logdev->LUNListLength)) / 8;
	/* Reject Logicals in excess of our max capability. */
	if (*nlogicals > HPSA_MAX_LUN) {
		dev_warn(&h->pdev->dev,
			"maximum logical LUNs (%d) exceeded.  "
			"%d LUNs ignored.\n", HPSA_MAX_LUN,
			*nlogicals - HPSA_MAX_LUN);
			*nlogicals = HPSA_MAX_LUN;
	}
	if (*nlogicals + *nphysicals > HPSA_MAX_PHYS_LUN) {
		dev_warn(&h->pdev->dev,
			"maximum logical + physical LUNs (%d) exceeded. "
			"%d LUNs ignored.\n", HPSA_MAX_PHYS_LUN,
			*nphysicals + *nlogicals - HPSA_MAX_PHYS_LUN);
		*nlogicals = HPSA_MAX_PHYS_LUN - *nphysicals;
	}
	return 0;
}

u8 *figure_lunaddrbytes(struct ctlr_info *h, int raid_ctlr_position, int i,
	int nphysicals, int nlogicals,
	struct ReportExtendedLUNdata *physdev_list,
	struct ReportLUNdata *logdev_list)
{
	/* Helper function, figure out where the LUN ID info is coming from
	 * given index i, lists of physical and logical devices, where in
	 * the list the raid controller is supposed to appear (first or last)
	 */

	int logicals_start = nphysicals + (raid_ctlr_position == 0);
	int last_device = nphysicals + nlogicals + (raid_ctlr_position == 0);

	if (i == raid_ctlr_position)
		return RAID_CTLR_LUNID;

	if (i < logicals_start) {
		const int index = i - (raid_ctlr_position == 0);
		return &physdev_list->LUN[index].lunid[0];
	}
	if (i < last_device)
		return &logdev_list->LUN[i - nphysicals -
			(raid_ctlr_position == 0)][0];
	BUG();
	return NULL;
}

static int hpsa_hba_mode_enabled(struct ctlr_info *h)
{
	int rc;
	int hba_mode_enabled;
	struct bmic_controller_parameters *ctlr_params;
	ctlr_params = kzalloc(sizeof(struct bmic_controller_parameters),
				GFP_KERNEL);
	
	if (!ctlr_params)
		return -ENOMEM;
	rc = hpsa_bmic_ctrl_mode_sense(h, RAID_CTLR_LUNID, 0, ctlr_params,
				sizeof(struct bmic_controller_parameters));
	if (rc != 0) {
		kfree(ctlr_params);
		return rc;
	}
	hba_mode_enabled =
		((ctlr_params->nvram_flags & HBA_MODE_ENABLED_FLAG) != 0);
	kfree(ctlr_params);
	return hba_mode_enabled;
}

/* get physical drive ioaccel handle and queue depth */
void hpsa_get_ioaccel_drive_info(struct ctlr_info *h,
		struct hpsa_scsi_dev_t *dev,
		u8 *lunaddrbytes,
		struct bmic_identify_physical_device *id_phys)
{
	int rc;
	struct ext_report_lun_entry *rle =
		(struct ext_report_lun_entry *) lunaddrbytes;

	dev->ioaccel_handle = rle->ioaccel_handle;
	memset(id_phys, 0, sizeof(*id_phys));
	rc = hpsa_bmic_id_physical_device(h, lunaddrbytes,
			GET_BMIC_DRIVE_NUMBER(lunaddrbytes), id_phys,
			sizeof(*id_phys));
	if (!rc)
		/* Reserve space for FW operations */
#define DRIVE_CMDS_RESERVED_FOR_FW 2
		dev->queue_depth =
			le16_to_cpu(id_phys->current_queue_depth_limit) -
				DRIVE_CMDS_RESERVED_FOR_FW;
	else
		dev->queue_depth = 7; /* conservative */
}

static void hpsa_update_scsi_devices(struct ctlr_info *h, int hostno)
{
	/* the idea here is we could get notified
	 * that some devices have changed, so we do a report
	 * physical luns and report logical luns cmd, and adjust
	 * our list of devices accordingly.
	 *
	 * The scsi3addr's of devices won't change so long as the
	 * adapter is not reset.  That means we can rescan and
	 * tell which devices we already know about, vs. new
	 * devices, vs.  disappearing devices.
	 */
	struct ReportExtendedLUNdata *physdev_list = NULL;
	struct ReportLUNdata *logdev_list = NULL;
	struct bmic_identify_physical_device *id_phys = NULL;
	u32 nphysicals = 0;
	u32 nlogicals = 0;
	u32 ndev_allocated = 0;
	struct hpsa_scsi_dev_t **currentsd, *this_device, *tmpdevice;
	int ncurrent = 0;
	int i, n_ext_target_devs, ndevs_to_allocate;
	int rc = 0;
	int raid_ctlr_position;
	int rescan_hba_mode;
	DECLARE_BITMAP(lunzerobits, MAX_EXT_TARGETS);

	currentsd = kzalloc(sizeof(*currentsd) * HPSA_MAX_DEVICES, GFP_KERNEL);
	physdev_list = kzalloc(sizeof(*physdev_list), GFP_KERNEL);
	logdev_list = kzalloc(sizeof(*logdev_list), GFP_KERNEL);
	tmpdevice = kzalloc(sizeof(*tmpdevice), GFP_KERNEL);
	id_phys = kzalloc(sizeof(*id_phys), GFP_KERNEL);

	if (!currentsd || !physdev_list || !logdev_list ||
		!tmpdevice || !id_phys) {
		dev_err(&h->pdev->dev, "out of memory\n");
		goto out;
	}
	memset(lunzerobits, 0, sizeof(lunzerobits));

	rescan_hba_mode = hpsa_hba_mode_enabled(h);

	if (rescan_hba_mode < 0) {
		h->drv_req_rescan = 1;
		goto out;
	}

	if (!h->hba_mode_enabled && rescan_hba_mode)
		dev_info(&h->pdev->dev, "HBA mode enabled\n");
	else if (h->hba_mode_enabled && !rescan_hba_mode)
		dev_info(&h->pdev->dev, "HBA mode disabled\n");

	h->hba_mode_enabled = rescan_hba_mode;

	if (hpsa_gather_lun_info(h, physdev_list, &nphysicals,
			logdev_list, &nlogicals)) {
		h->drv_req_rescan = 1;
		goto out;
	}

	/* We might see up to the maximum number of logical and physical disks
	 * plus external target devices, and a device for the local RAID
	 * controller.
	 */
	ndevs_to_allocate = nphysicals + nlogicals + MAX_EXT_TARGETS + 1;

	/* Allocate the per device structures */
	for (i = 0; i < ndevs_to_allocate; i++) {
		if (i >= HPSA_MAX_DEVICES) {
			dev_warn(&h->pdev->dev, "maximum devices (%d) exceeded."
				"  %d devices ignored.\n", HPSA_MAX_DEVICES,
				ndevs_to_allocate - HPSA_MAX_DEVICES);
			break;
		}

		currentsd[i] = kzalloc(sizeof(*currentsd[i]), GFP_KERNEL);
		if (!currentsd[i]) {
			dev_warn(&h->pdev->dev, "out of memory at %s:%d\n",
				__FILE__, __LINE__);
			h->drv_req_rescan = 1;
			goto out;
		}
		ndev_allocated++;
	}

	if (is_scsi_rev_5(h))
		raid_ctlr_position = 0;
	else
		raid_ctlr_position = nphysicals + nlogicals;

	/* adjust our table of devices */
	n_ext_target_devs = 0;
	for (i = 0; i < nphysicals + nlogicals + 1; i++) {
		u8 *lunaddrbytes, is_OBDR = 0;

		/* Figure out where the LUN ID info is coming from */
		lunaddrbytes = figure_lunaddrbytes(h, raid_ctlr_position,
			i, nphysicals, nlogicals, physdev_list, logdev_list);
		/* skip masked non-disk devices */
		if (MASKED_DEVICE(lunaddrbytes))
			if (i < nphysicals + (raid_ctlr_position == 0) &&
				NON_DISK_PHYS_DEV(lunaddrbytes))
				continue;

		/* Get device type, vendor, model, device id */
		rc = hpsa_update_device_info(h, lunaddrbytes, tmpdevice,
							&is_OBDR);
		if (rc) {
			dev_warn(&h->pdev->dev, "%s, rescan stopped.\n",
				(rc == -ENOMEM) ? "Out of memory" :
				"Inquiry failed");
			h->drv_req_rescan = 1;
			goto out;
		}

		figure_bus_target_lun(h, lunaddrbytes, tmpdevice);
		this_device = currentsd[ncurrent];

		/*
		 * For external target devices, we have to insert a LUN 0 which
		 * doesn't show up in CCISS_REPORT_PHYSICAL data, but there
		 * is nonetheless an enclosure device there.  We have to
		 * present that otherwise linux won't find anything if
		 * there is no lun 0.
		 */
		if (add_ext_target_dev(h, tmpdevice, this_device,
				lunaddrbytes, lunzerobits,
				&n_ext_target_devs)) {
			ncurrent++;
			this_device = currentsd[ncurrent];
		}

		*this_device = *tmpdevice;

		this_device->bay = 0;
		this_device->active_path_index = 0;
		this_device->path_map = 0;
		memset(this_device->box, 0, sizeof(this_device->box));
		memset(this_device->phys_connector, 0,
			sizeof(this_device->phys_connector));

		/* do not expose masked devices */
		if (MASKED_DEVICE(lunaddrbytes) &&
			i < nphysicals + (raid_ctlr_position == 0)) {
			if (h->hba_mode_enabled)
				dev_warn(&h->pdev->dev,
					"Masked physical device detected\n");
			this_device->expose_state = HPSA_DO_NOT_EXPOSE;
		} else {
			this_device->expose_state = HPSA_EXPOSE;
		}

		switch (this_device->devtype) {
		case TYPE_ROM:
			/* We don't *really* support actual CD-ROM devices,
			 * on Smart Arrays, just "One Button Disaster
			 * Recovery" tape drive which temporarily pretends to
			 * be a CD-ROM drive.  So we check that the device is
			 * really an OBDR tape device by checking for "$DR-10"
			 * in bytes 43-48 of the inquiry data.
			 */
			if (is_OBDR)
				ncurrent++;
			break;
		case TYPE_DISK:
			/* HBA mode supported*/
			if (h->hba_mode_enabled) {
				/* never use raid mapper in HBA mode */
				this_device->offload_enabled = 0;
				hpsa_get_ioaccel_drive_info(h, this_device,
						lunaddrbytes, id_phys);

				if (PHYS_IOACCEL(lunaddrbytes)
					&& this_device->ioaccel_handle)
					this_device->hba_ioaccel_enabled = 1;

				memcpy(&this_device->active_path_index,
					&id_phys->active_path_number,
					sizeof(this_device->active_path_index));
				memcpy(&this_device->path_map,
					&id_phys->redundant_path_present_map,
					sizeof(this_device->path_map));
				memcpy(&this_device->box,
					&id_phys->alternate_paths_phys_box_on_port,
					sizeof(this_device->box));
				memcpy(&this_device->phys_connector,
					&id_phys->alternate_paths_phys_connector,
					sizeof(this_device->phys_connector));
				memcpy(&this_device->bay,
					&id_phys->phys_bay_in_box,
					sizeof(this_device->bay));

				ncurrent++;
				memset(id_phys, 0, sizeof(*id_phys));
				break;
			}
			/* HP SSD Smart Path Mode supported */
			else if (h->acciopath_status) {
				if (i >= nphysicals) {
					ncurrent++;
					break;
				}
				if (h->transMethod & CFGTBL_Trans_io_accel1 ||
					h->transMethod & CFGTBL_Trans_io_accel2) {
					hpsa_get_ioaccel_drive_info(h, this_device,
							lunaddrbytes, id_phys);
					ncurrent++;
				}
				break;
			}
			else {  /* normal mode: no ioaccel or HBA mode */
				if (i < nphysicals)
					break;
				ncurrent++;
				break;
                        }
		case TYPE_TAPE:
		case TYPE_MEDIUM_CHANGER:
			ncurrent++;
			break;
		case TYPE_ENCLOSURE:
			if (h->hba_mode_enabled)
				ncurrent++;
			break;
		case TYPE_RAID:
			/* Only present the Smartarray HBA as a RAID controller.
			 * If it's a RAID controller other than the HBA itself
			 * (an external RAID controller, MSA500 or similar)
			 * don't present it.
			 */
			if (!is_hba_lunid(lunaddrbytes))
				break;
			ncurrent++;
			break;
		default:
			break;
		}
		if (ncurrent >= HPSA_MAX_DEVICES)
			break;
	}
	adjust_hpsa_scsi_table(h, hostno, currentsd, ncurrent);
out:
	kfree(tmpdevice);
	if (currentsd) {
		for (i = 0; i < ndev_allocated; i++)
			kfree(currentsd[i]);
		kfree(currentsd);
	}
	kfree(physdev_list);
	kfree(logdev_list);
	kfree(id_phys);
}

#if KFEATURE_HAS_SCSI_DMA_FUNCTIONS
/* hpsa_scatter_gather takes a struct scsi_cmnd, (cmd), and does the pci
 * dma mapping  and fills in the scatter gather entries of the
 * hpsa command, cp.
 */
static int hpsa_scatter_gather(struct ctlr_info *h,
		struct CommandList *cp,
		struct scsi_cmnd *cmd)
{
	unsigned int len;
	struct scatterlist *sg;
	u64 addr64;
	int use_sg, i, sg_index, chained, last_sg;
	struct SGDescriptor *curr_sg;

	BUG_ON(scsi_sg_count(cmd) > h->maxsgentries);

	use_sg = scsi_dma_map(cmd);
	if (use_sg < 0)
		return use_sg;

	if (!use_sg)
		goto sglist_finished;

	curr_sg = cp->SG;
	chained = 0;
	sg_index = 0;
	last_sg = scsi_sg_count(cmd) - 1;
	scsi_for_each_sg(cmd, sg, use_sg, i) {
		if (i == h->max_cmd_sg_entries - 1 &&
			use_sg > h->max_cmd_sg_entries) {
			chained = 1;
			curr_sg = h->cmd_sg_list[cp->cmdindex];
			sg_index = 0;
		}
		addr64 = (u64) sg_dma_address(sg);
		len  = sg_dma_len(sg);
		curr_sg->Addr = cpu_to_le64(addr64);
		curr_sg->Len = cpu_to_le32(len);
		curr_sg->Ext = cpu_to_le32((i == last_sg) * HPSA_SG_LAST);
		curr_sg++;
	}

	if (use_sg + chained > h->maxSG)
		h->maxSG = use_sg + chained;

	if (chained) {
		cp->Header.SGList = h->max_cmd_sg_entries;
		cp->Header.SGTotal = (u16) (use_sg + 1);
		if (hpsa_map_sg_chain_block(h, cp)) {
			scsi_dma_unmap(cmd);
			return -1;
		}
		return 0;
	}

sglist_finished:

	cp->Header.SGList = (u8) use_sg;   /* no. SGs contig in this cmd */
	cp->Header.SGTotal = (u16) use_sg; /* total sgs in this cmd list */
	return 0;
}
#endif /* if KFEATURE_HAS_SCSI_DMA_FUNCTIONS
	* otherwise hpsa_scatter_gather() is defined in hpsa_kernel_compat.h
	*/

#define IO_ACCEL_INELIGIBLE (1)
static int fixup_ioaccel_cdb(u8 *cdb, int *cdb_len)
{
	int is_write = 0;
	u32 block;
	u32 block_cnt;

	/* Perform some CDB fixups if needed using 10 byte reads/writes only */
	switch (cdb[0]) {
	case WRITE_6:
	case WRITE_12:
		is_write = 1;
	case READ_6:
	case READ_12:
		if (*cdb_len == 6) {
			block = (((u32) cdb[2]) << 8) | cdb[3];
			block_cnt = cdb[4];
		} else {
			BUG_ON(*cdb_len != 12);
			block = (((u32) cdb[2]) << 24) |
				(((u32) cdb[3]) << 16) |
				(((u32) cdb[4]) << 8) |
				cdb[5];
			block_cnt =
				(((u32) cdb[6]) << 24) |
				(((u32) cdb[7]) << 16) |
				(((u32) cdb[8]) << 8) |
				cdb[9];
		}
		if (block_cnt > 0xffff)
			return IO_ACCEL_INELIGIBLE;

		cdb[0] = is_write ? WRITE_10 : READ_10;
		cdb[1] = 0;
		cdb[2] = (u8) (block >> 24);
		cdb[3] = (u8) (block >> 16);
		cdb[4] = (u8) (block >> 8);
		cdb[5] = (u8) (block);
		cdb[6] = 0;
		cdb[7] = (u8) (block_cnt >> 8);
		cdb[8] = (u8) (block_cnt);
		cdb[9] = 0;
		*cdb_len = 10;
		break;
	}
	return 0;
}

static int hpsa_scsi_ioaccel1_queue_command(struct ctlr_info *h,
	struct CommandList *c, u32 ioaccel_handle, u8 *cdb, int cdb_len,
	u8 *scsi3addr)
{
	struct scsi_cmnd *cmd = c->scsi_cmd;
	struct io_accel1_cmd *cp = &h->ioaccel_cmd_pool[c->cmdindex];
	unsigned int len;
	unsigned int total_len = 0;
	struct scatterlist *sg;
	u64 addr64;
	int use_sg, i;
	struct SGDescriptor *curr_sg;
	u32 control = IOACCEL1_CONTROL_SIMPLEQUEUE;

	/* TODO: implement chaining support */
	if (scsi_sg_count(cmd) > h->ioaccel_maxsg)
		return IO_ACCEL_INELIGIBLE;

	BUG_ON(cmd->cmd_len > IOACCEL1_IOFLAGS_CDBLEN_MAX);

	if (fixup_ioaccel_cdb(cdb, &cdb_len))
		return IO_ACCEL_INELIGIBLE;

	c->cmd_type = CMD_IOACCEL1;

	/* Adjust the DMA address to point to the accelerated command buffer */
	c->busaddr = (u32) h->ioaccel_cmd_pool_dhandle +
				(c->cmdindex * sizeof(*cp));
	BUG_ON(c->busaddr & 0x0000007F);

	use_sg = scsi_dma_map(cmd);
	if (use_sg < 0)
		return use_sg;

	if (use_sg) {
		curr_sg = cp->SG;
		scsi_for_each_sg(cmd, sg, use_sg, i) {
			addr64 = (u64) sg_dma_address(sg);
			len  = sg_dma_len(sg);
			total_len += len;
			curr_sg->Addr = cpu_to_le64(addr64);
			curr_sg->Len = cpu_to_le32(len);

			if (i == (scsi_sg_count(cmd) - 1))
				curr_sg->Ext = cpu_to_le32(HPSA_SG_LAST);
			else
				curr_sg->Ext = cpu_to_le32(0); /* not chain */
			curr_sg++;
		}

		switch (cmd->sc_data_direction) {
		case DMA_TO_DEVICE:
			control |= IOACCEL1_CONTROL_DATA_OUT;
			break;
		case DMA_FROM_DEVICE:
			control |= IOACCEL1_CONTROL_DATA_IN;
			break;
		case DMA_NONE:
			control |= IOACCEL1_CONTROL_NODATAXFER;
			break;
		default:
			dev_err(&h->pdev->dev, "unknown data direction: %d\n",
			cmd->sc_data_direction);
			BUG();
			break;
		}
	} else {
		control |= IOACCEL1_CONTROL_NODATAXFER;
	}

	c->Header.SGList = use_sg;
	/* Fill out the command structure to submit */
	cp->dev_handle = ioaccel_handle & 0xFFFF;
	cp->transfer_len = total_len;
	cp->io_flags = IOACCEL1_IOFLAGS_IO_REQ |
			(cdb_len & IOACCEL1_IOFLAGS_CDBLEN_MASK);
	cp->control = control;
	memcpy(cp->CDB, cdb, cdb_len);
	memcpy(cp->CISS_LUN, scsi3addr, 8);
	/* Tag was already set at init time. */
	enqueue_cmd_and_start_io(h, c);
	return 0;
}
 
/*
 * Queue a command directly to a device behind the controller using the
 * I/O accelerator path.
 */
static int hpsa_scsi_ioaccel_direct_map(struct ctlr_info *h,
	struct CommandList *c)
{
	struct scsi_cmnd *cmd = c->scsi_cmd;
	struct hpsa_scsi_dev_t *dev = cmd->device->hostdata;

	return hpsa_scsi_ioaccel_queue_command(h, c, dev->ioaccel_handle,
		cmd->cmnd, cmd->cmd_len, dev->scsi3addr);
}

static void __attribute__((unused)) verify_offsets(void)
{
#define VERIFY_OFFSET(member, offset) \
	BUILD_BUG_ON(offsetof(struct raid_map_data, member) != offset)

	VERIFY_OFFSET(structure_size, 0);
	VERIFY_OFFSET(volume_blk_size, 4);
	VERIFY_OFFSET(volume_blk_cnt, 8);
	VERIFY_OFFSET(phys_blk_shift, 16);
	VERIFY_OFFSET(parity_rotation_shift, 17);
	VERIFY_OFFSET(strip_size, 18);
	VERIFY_OFFSET(disk_starting_blk, 20);
	VERIFY_OFFSET(disk_blk_cnt, 28);
	VERIFY_OFFSET(data_disks_per_row, 36);
	VERIFY_OFFSET(metadata_disks_per_row, 38);
	VERIFY_OFFSET(row_cnt, 40);
	VERIFY_OFFSET(layout_map_count, 42);
	VERIFY_OFFSET(flags, 44);
	VERIFY_OFFSET(dekindex, 46);
	/* VERIFY_OFFSET(reserved, 48 */
	VERIFY_OFFSET(data, 64);

#undef VERIFY_OFFSET

#define VERIFY_OFFSET(member, offset) \
	BUILD_BUG_ON(offsetof(struct io_accel2_cmd, member) != offset)

	VERIFY_OFFSET(IU_type, 0);
	VERIFY_OFFSET(direction, 1);
	VERIFY_OFFSET(reply_queue, 2);
	/* VERIFY_OFFSET(reserved1, 3);  */
	VERIFY_OFFSET(scsi_nexus, 4);
	VERIFY_OFFSET(Tag, 8);
	VERIFY_OFFSET(tweak_lower, 12);
	VERIFY_OFFSET(cdb, 16);
	VERIFY_OFFSET(cciss_lun, 32);
	VERIFY_OFFSET(data_len, 40);
	VERIFY_OFFSET(cmd_priority_task_attr, 44);
	VERIFY_OFFSET(sg_count, 45); 
	VERIFY_OFFSET(dekindex, 46);
	VERIFY_OFFSET(err_ptr, 48);
	VERIFY_OFFSET(err_len, 56);
	VERIFY_OFFSET(tweak_upper, 60);
	VERIFY_OFFSET(sg, 64);

#undef VERIFY_OFFSET

#define VERIFY_OFFSET(member, offset) \
	BUILD_BUG_ON(offsetof(struct io_accel1_cmd, member) != offset)
	
	VERIFY_OFFSET(dev_handle, 0x00);
	VERIFY_OFFSET(reserved1, 0x02);
	VERIFY_OFFSET(function, 0x03);
	VERIFY_OFFSET(reserved2, 0x04);
	VERIFY_OFFSET(err_info, 0x0C);
	VERIFY_OFFSET(reserved3, 0x10);
	VERIFY_OFFSET(err_info_len, 0x12);
	VERIFY_OFFSET(reserved4, 0x13);
	VERIFY_OFFSET(sgl_offset, 0x14);
	VERIFY_OFFSET(reserved5, 0x15);
	VERIFY_OFFSET(transfer_len, 0x1C);
	VERIFY_OFFSET(reserved6, 0x20);
	VERIFY_OFFSET(io_flags, 0x24);
	VERIFY_OFFSET(reserved7, 0x26);
	VERIFY_OFFSET(LUN, 0x34);
	VERIFY_OFFSET(control, 0x3C);
	VERIFY_OFFSET(CDB, 0x40);
	VERIFY_OFFSET(reserved8, 0x50);
	VERIFY_OFFSET(host_context_flags, 0x60);
	VERIFY_OFFSET(timeout_sec, 0x62);
	VERIFY_OFFSET(ReplyQueue, 0x64);
	VERIFY_OFFSET(reserved9, 0x65);
	VERIFY_OFFSET(tag, 0x68);
	VERIFY_OFFSET(host_addr, 0x70);
	VERIFY_OFFSET(CISS_LUN, 0x78);
	VERIFY_OFFSET(SG, 0x78 + 8);
#undef VERIFY_OFFSET
}

/*
 * Set encryption parameters for the ioaccel2 request 
 */
static void set_encrypt_ioaccel2(struct ctlr_info *h,
	struct CommandList *c, struct io_accel2_cmd *cp)
{
	struct scsi_cmnd *cmd = c->scsi_cmd;
	struct hpsa_scsi_dev_t *dev = cmd->device->hostdata;
	struct raid_map_data *map = &dev->raid_map;
	u64 first_block;

	/* Are we doing encryption on this device */
	if (! (map->flags & RAID_MAP_FLAG_ENCRYPT_ON) )
		return;
	/* Set the data encryption key index. */
	cp->dekindex = map->dekindex;

	/* Set the encryption enable flag, encoded into direction field. */
	cp->direction |= IOACCEL2_DIRECTION_ENCRYPT_MASK;

	/* Set encryption tweak values based on logical block address
	 * If block size is 512, tweak value is LBA.
	 * For other block sizes, tweak is (LBA * block size)/ 512)
	 */
	switch (cmd->cmnd[0]) {
	/* Required? 6-byte cdbs eliminated by fixup_ioaccel_cdb */
	case WRITE_6:
	case READ_6:
		if (map->volume_blk_size == 512 ) {
			cp->tweak_lower = 
				(((u32) cmd->cmnd[2]) << 8) |
					cmd->cmnd[3];
			cp->tweak_upper = 0; 
		}
		else {
			first_block = 
				(((u64) cmd->cmnd[2]) << 8) |
					cmd->cmnd[3];
			first_block = (first_block * map->volume_blk_size)/512;
			cp->tweak_lower = (u32)first_block;
			cp->tweak_upper = (u32)(first_block >> 32);
		}
		break;
	case WRITE_10:
	case READ_10:
		if (map->volume_blk_size == 512 ) {
			cp->tweak_lower = 
				(((u32) cmd->cmnd[2]) << 24) |
				(((u32) cmd->cmnd[3]) << 16) |
				(((u32) cmd->cmnd[4]) << 8) |
					cmd->cmnd[5];
			cp->tweak_upper = 0; 
		}
		else {
			first_block =
				(((u64) cmd->cmnd[2]) << 24) |
				(((u64) cmd->cmnd[3]) << 16) |
				(((u64) cmd->cmnd[4]) << 8) |
					cmd->cmnd[5];
			first_block = (first_block * map->volume_blk_size)/512;
			cp->tweak_lower = (u32)first_block;
			cp->tweak_upper = (u32)(first_block >> 32);
		}
		break;
	/* Required? 12-byte cdbs eliminated by fixup_ioaccel_cdb */
	case WRITE_12:
	case READ_12:
		if (map->volume_blk_size == 512 ) {
			cp->tweak_lower = 
				(((u32) cmd->cmnd[2]) << 24) |
				(((u32) cmd->cmnd[3]) << 16) |
				(((u32) cmd->cmnd[4]) << 8) |
					cmd->cmnd[5];
			cp->tweak_upper = 0; 
		}
		else {
			first_block =
				(((u64) cmd->cmnd[2]) << 24) |
				(((u64) cmd->cmnd[3]) << 16) |
				(((u64) cmd->cmnd[4]) << 8) |
					cmd->cmnd[5];
			first_block = (first_block * map->volume_blk_size)/512;
			cp->tweak_lower = (u32)first_block;
			cp->tweak_upper = (u32)(first_block >> 32);
		}
		break;
	case WRITE_16:
	case READ_16:
		if (map->volume_blk_size == 512 ) {
			cp->tweak_lower = 
				(((u32) cmd->cmnd[6]) << 24) |
				(((u32) cmd->cmnd[7]) << 16) |
				(((u32) cmd->cmnd[8]) << 8) |
					cmd->cmnd[9];
			cp->tweak_upper = 
				(((u32) cmd->cmnd[2]) << 24) |
				(((u32) cmd->cmnd[3]) << 16) |
				(((u32) cmd->cmnd[4]) << 8) |
					cmd->cmnd[5];
		}
		else {
			first_block =
				(((u64) cmd->cmnd[2]) << 56) |
				(((u64) cmd->cmnd[3]) << 48) |
				(((u64) cmd->cmnd[4]) << 40) |
				(((u64) cmd->cmnd[5]) << 32) |
				(((u64) cmd->cmnd[6]) << 24) |
				(((u64) cmd->cmnd[7]) << 16) |
				(((u64) cmd->cmnd[8]) << 8) |
					cmd->cmnd[9];
			first_block = (first_block * map->volume_blk_size)/512;
			cp->tweak_lower = (u32)first_block;
			cp->tweak_upper = (u32)(first_block >> 32);
		}
		break;
	default:
		dev_err(&h->pdev->dev, "ERROR: %s: IOACCEL request "
			"CDB size not supported for encryption\n", __func__);
		BUG();	
		break;
	}
}


static int hpsa_scsi_ioaccel2_queue_command(struct ctlr_info *h,
	struct CommandList *c, u32 ioaccel_handle, u8 *cdb, int cdb_len,
	u8 *scsi3addr)
{
	struct scsi_cmnd *cmd = c->scsi_cmd;
	struct io_accel2_cmd *cp = &h->ioaccel2_cmd_pool[c->cmdindex];
	struct ioaccel2_sg_element *curr_sg;
	int use_sg, i;
	struct scatterlist *sg;
	u64 addr64;
	u32 len;
	u32 total_len = 0;

	BUG_ON(scsi_sg_count(cmd) > h->maxsgentries);

	if (fixup_ioaccel_cdb(cdb, &cdb_len))
		return IO_ACCEL_INELIGIBLE;
	c->cmd_type = CMD_IOACCEL2;
	/* Adjust the DMA address to point to the accelerated command buffer */
	c->busaddr = (u32) h->ioaccel2_cmd_pool_dhandle +
				(c->cmdindex * sizeof(*cp));
	BUG_ON(c->busaddr & 0x0000007F);

	memset(cp, 0, sizeof(*cp));
	cp->IU_type = IOACCEL2_IU_TYPE;

	use_sg = scsi_dma_map(cmd);
	if (use_sg < 0)
		return use_sg;

	if (use_sg) {
		curr_sg = cp->sg;
		if (use_sg > h->ioaccel_maxsg) {
			addr64 = h->ioaccel2_cmd_sg_list[c->cmdindex]->address;
			curr_sg->address = cpu_to_le64(addr64);
			curr_sg->length = 0;
			curr_sg->reserved[0] = 0;
			curr_sg->reserved[1] = 0;
			curr_sg->reserved[2] = 0;
			curr_sg->chain_indicator = 0x80;

			curr_sg = h->ioaccel2_cmd_sg_list[c->cmdindex];
		}
			
		scsi_for_each_sg(cmd, sg, use_sg, i) {
			addr64 = (u64) sg_dma_address(sg);
			len  = sg_dma_len(sg);
			total_len += len;
			curr_sg->address = cpu_to_le64(addr64);
			curr_sg->length = cpu_to_le32(len);
			curr_sg->reserved[0] = 0;
			curr_sg->reserved[1] = 0;
			curr_sg->reserved[2] = 0;
			curr_sg->chain_indicator = 0;
			curr_sg++;
		}

		switch (cmd->sc_data_direction) {
		case DMA_TO_DEVICE:
			cp->direction &= ~IOACCEL2_DIRECTION_MASK; 
			cp->direction |= IOACCEL2_DIR_DATA_OUT;
			break;
		case DMA_FROM_DEVICE:
			cp->direction &= ~IOACCEL2_DIRECTION_MASK; 
			cp->direction |= IOACCEL2_DIR_DATA_IN;
			break;
		case DMA_NONE:
			cp->direction &= ~IOACCEL2_DIRECTION_MASK; 
			cp->direction |= IOACCEL2_DIR_NO_DATA;
			break;
		default:
			dev_err(&h->pdev->dev, "unknown data direction: %d\n",
				cmd->sc_data_direction);
			BUG();
			break;
		}
	} else {
		cp->direction &= ~IOACCEL2_DIRECTION_MASK; 
		cp->direction |= IOACCEL2_DIR_NO_DATA;
	}

	/* Set encryption parameters, if necessary */
	set_encrypt_ioaccel2(h, c, cp);
		
	cp->scsi_nexus = ioaccel_handle; /* FIXME: is this correct? */
	cp->Tag = c->cmdindex << DIRECT_LOOKUP_SHIFT;
	memcpy(cp->cdb, cdb, sizeof(cp->cdb));
	memset(cp->cciss_lun, 0, sizeof(cp->cciss_lun)); /* FIXME: cciss_lun doesn't make sense here */
	cp->cmd_priority_task_attr = 0;


	cp->data_len = cpu_to_le32(total_len);
	cp->err_ptr = cpu_to_le64(c->busaddr + offsetof(struct io_accel2_cmd, error_data));
	cp->err_len = cpu_to_le32((u32) sizeof(cp->error_data));

	/* fill in sg elements */
	if (use_sg > h->ioaccel_maxsg) {
		cp->sg_count = 1;
		if (hpsa_map_ioaccel2_sg_chain_block(h, cp, c)) {
			scsi_dma_unmap(cmd);
			return -1;
		}
	} else {
		cp->sg_count = (u8) use_sg;
	}
	if (unlikely(lockup_detected(h))) {
		cmd->result = DID_NO_CONNECT << 16;
		cmd_free(h, c);
		cmd->scsi_done(cmd);
		return 0;
	}

	enqueue_cmd_and_start_io(h, c);
	return 0;
}

/*
 * Queue a command to the correct I/O accelerator path.
 */
static int hpsa_scsi_ioaccel_queue_command(struct ctlr_info *h,
	struct CommandList *c, u32 ioaccel_handle, u8 *cdb, int cdb_len,
	u8 *scsi3addr)
{
	if (h->transMethod & CFGTBL_Trans_io_accel1)
		return hpsa_scsi_ioaccel1_queue_command(h, c, ioaccel_handle,
						cdb, cdb_len, scsi3addr);
	else
		return hpsa_scsi_ioaccel2_queue_command(h, c, ioaccel_handle,
						cdb, cdb_len, scsi3addr);
}

/*
 * Attempt to perform offload RAID mapping for a logical volume I/O.
 */
static int hpsa_scsi_ioaccel_raid_map(struct ctlr_info *h,
	struct CommandList *c)
{
	struct scsi_cmnd *cmd = c->scsi_cmd;
	struct hpsa_scsi_dev_t *dev = cmd->device->hostdata;
	struct raid_map_data *map = &dev->raid_map;
	struct raid_map_disk_data *dd = &map->data[0];
	int is_write = 0;
	u32 map_index;
	u64 first_block, last_block;
	u32 block_cnt;
	u32 blocks_per_row;
	u64 first_row, last_row;
	u32 first_row_offset, last_row_offset;
	u32 first_column, last_column;
	u64 r0_first_row, r0_last_row;
	u32 r5or6_blocks_per_row;
	u64 r5or6_first_row, r5or6_last_row;
	u32 r5or6_first_row_offset, r5or6_last_row_offset;
	u32 r5or6_first_column, r5or6_last_column;
	u32 total_disks_per_row;
	u32 stripesize;
	u32 first_group, last_group, current_group;
	u32 map_row;
	u32 disk_handle;
	u64 disk_block;
	u32 disk_block_cnt;
	u8 cdb[16];
	u8 cdb_len;
#if BITS_PER_LONG == 32
	u64 tmpdiv;
#endif
	int offload_to_mirror;

	BUG_ON(!(dev->offload_config && dev->offload_enabled));

	/* check for valid opcode, get LBA and block count */
	switch (cmd->cmnd[0]) {
	case WRITE_6:
		is_write = 1;
	case READ_6:
		first_block =
			(((u64) cmd->cmnd[2]) << 8) |
			cmd->cmnd[3];
		block_cnt = cmd->cmnd[4];
		break;
	case WRITE_10:
		is_write = 1;
	case READ_10:
		first_block =
			(((u64) cmd->cmnd[2]) << 24) |
			(((u64) cmd->cmnd[3]) << 16) |
			(((u64) cmd->cmnd[4]) << 8) |
			cmd->cmnd[5];
		block_cnt =
			(((u32) cmd->cmnd[7]) << 8) |
			cmd->cmnd[8];
		break;
	case WRITE_12:
		is_write = 1;
	case READ_12:
		first_block =
			(((u64) cmd->cmnd[2]) << 24) |
			(((u64) cmd->cmnd[3]) << 16) |
			(((u64) cmd->cmnd[4]) << 8) |
			cmd->cmnd[5];
		block_cnt =
			(((u32) cmd->cmnd[6]) << 24) |
			(((u32) cmd->cmnd[7]) << 16) |
			(((u32) cmd->cmnd[8]) << 8) |
		cmd->cmnd[9];
		break;
	case WRITE_16:
		is_write = 1;
	case READ_16:
		first_block =
			(((u64) cmd->cmnd[2]) << 56) |
			(((u64) cmd->cmnd[3]) << 48) |
			(((u64) cmd->cmnd[4]) << 40) |
			(((u64) cmd->cmnd[5]) << 32) |
			(((u64) cmd->cmnd[6]) << 24) |
			(((u64) cmd->cmnd[7]) << 16) |
			(((u64) cmd->cmnd[8]) << 8) |
			cmd->cmnd[9];
		block_cnt =
			(((u32) cmd->cmnd[10]) << 24) |
			(((u32) cmd->cmnd[11]) << 16) |
			(((u32) cmd->cmnd[12]) << 8) |
			cmd->cmnd[13];
		break;
	default:
		return IO_ACCEL_INELIGIBLE; /* process via normal I/O path */
	}
	BUG_ON(block_cnt == 0);
	last_block = first_block + block_cnt - 1;

	/* check for write to non-RAID-0 */
	if (is_write && dev->raid_level != 0)
		return IO_ACCEL_INELIGIBLE;

	/* check for invalid block or wraparound */
	if (last_block >= map->volume_blk_cnt || last_block < first_block)
		return IO_ACCEL_INELIGIBLE;

	/* calculate stripe information for the request */
	blocks_per_row = map->data_disks_per_row * map->strip_size;
#if BITS_PER_LONG == 32
	tmpdiv = first_block;
	(void) do_div(tmpdiv, blocks_per_row);
	first_row = tmpdiv;
	tmpdiv = last_block;
	(void) do_div(tmpdiv, blocks_per_row);
	last_row = tmpdiv;
	first_row_offset = (u32) (first_block - (first_row * blocks_per_row));
	last_row_offset = (u32) (last_block - (last_row * blocks_per_row));
	tmpdiv = first_row_offset;
	(void) do_div(tmpdiv,  map->strip_size);
	first_column = tmpdiv;
	tmpdiv = last_row_offset;
	(void) do_div(tmpdiv, map->strip_size);
	last_column = tmpdiv;
#else
	first_row = first_block / blocks_per_row;
	last_row = last_block / blocks_per_row;
	first_row_offset = (u32) (first_block - (first_row * blocks_per_row));
	last_row_offset = (u32) (last_block - (last_row * blocks_per_row));
	first_column = first_row_offset / map->strip_size;
	last_column = last_row_offset / map->strip_size;
#endif

	/* if this isn't a single row/column then give to the controller */
	if ((first_row != last_row) || (first_column != last_column))
		return IO_ACCEL_INELIGIBLE;

	/* proceeding with driver mapping */
	total_disks_per_row = map->data_disks_per_row + map->metadata_disks_per_row;
	map_row = ((u32)(first_row >> map->parity_rotation_shift)) % map->row_cnt;
	map_index = (map_row * total_disks_per_row ) + first_column;


	/* RAID 1 */ 
	if (dev->raid_level == HPSA_RAID_1) {
		/* Handles load balance across RAID 1 members.
		 * (2-drive R1 and R10 with even # of drives.)
		 * Appropriate for SSDs, not optimal for HDDs 
		 */
		BUG_ON(map->layout_map_count != 2 );
		if (dev->offload_to_mirror)
			map_index += map->data_disks_per_row;
		dev->offload_to_mirror = !dev->offload_to_mirror;
	}
	/* RAID ADM */
        else if (dev->raid_level == HPSA_RAID_ADM) {
		/* Handles N-way mirrors  (R1-ADM)
		 * and R10 with # of drives divisible by 3.)
		 */
		BUG_ON(map->layout_map_count != 3 );
		
		offload_to_mirror = dev->offload_to_mirror;
		if (offload_to_mirror == 0)  {
			/* use physical disk in the first mirrored group. */
			map_index %= map->data_disks_per_row;
		} else {
			
			do {
				/* determine mirror group that map_index indicates */
				current_group = map_index / map->data_disks_per_row;
	
				if (offload_to_mirror != current_group ) {
	
					if (current_group < (map->layout_map_count - 1)) {
						/* select map index from next group */
						map_index += map->data_disks_per_row;
						current_group++;	
					}
					else {
						/* select map index from first group */
						map_index %= map->data_disks_per_row;
						current_group = 0;
					}
					
				}
			} while (offload_to_mirror != current_group);

		}

                /* set mirror group to use next time */
                offload_to_mirror = (offload_to_mirror >= map->layout_map_count-1) 
                        ? 0 : offload_to_mirror+1;
                BUG_ON(offload_to_mirror >= map->layout_map_count); /* FIXME: remove after debug/dev */
		//dev_warn(&h->pdev->dev, "DEBUG: Using physical disk map index %d "
		//	"from mirror group %d\n", map_index, offload_to_mirror);
                dev->offload_to_mirror = offload_to_mirror;
                /* Avoid direct use of dev->offload_to_mirror within this function since multiple threads
                 * might simultaneously increment it beyond the range of dev->layout_map_count -1.
                 */
        }

	/* RAID 50/60 */
        else if ((dev->raid_level == HPSA_RAID_5 || 
		dev->raid_level == HPSA_RAID_6) &&
		map->layout_map_count > 1) {

		/* Verify first and last block are in same RAID group */
		r5or6_blocks_per_row = map->strip_size * map->data_disks_per_row;
		BUG_ON(r5or6_blocks_per_row == 0);
		stripesize = r5or6_blocks_per_row * map->layout_map_count;
#if BITS_PER_LONG == 32
		tmpdiv = first_block;
		first_group = do_div(tmpdiv, stripesize);
		tmpdiv = first_group;
		(void) do_div(tmpdiv, r5or6_blocks_per_row);					
		first_group = tmpdiv;
		tmpdiv = last_block;
		last_group = do_div(tmpdiv, stripesize);
		tmpdiv = last_group;
		(void) do_div(tmpdiv, r5or6_blocks_per_row);					
		last_group = tmpdiv;
#else
		first_group = (first_block % stripesize) / r5or6_blocks_per_row;
		last_group = (last_block % stripesize) / r5or6_blocks_per_row;
#endif
		if (first_group != last_group )  
			return IO_ACCEL_INELIGIBLE;

		/* Verify request is in a single row of RAID 5/6 */
#if BITS_PER_LONG == 32
		tmpdiv = first_block;
		(void) do_div(tmpdiv, stripesize);
		first_row = r5or6_first_row = r0_first_row = tmpdiv;
		tmpdiv = last_block;
		(void) do_div(tmpdiv, stripesize);
		r5or6_last_row = r0_last_row = tmpdiv;
#else 
		first_row = r5or6_first_row = r0_first_row = first_block / stripesize;
		r5or6_last_row = r0_last_row = last_block / stripesize;
#endif
		if (r5or6_first_row != r5or6_last_row )
			return IO_ACCEL_INELIGIBLE;
		
	
		/* Verify request is in a single column */
#if BITS_PER_LONG == 32
		tmpdiv = first_block;
		first_row_offset = do_div(tmpdiv, stripesize); 
		tmpdiv = first_row_offset;
		first_row_offset = (u32) do_div(tmpdiv, r5or6_blocks_per_row); 
		r5or6_first_row_offset = first_row_offset;
		tmpdiv = last_block;
		r5or6_last_row_offset = do_div(tmpdiv, stripesize); 
		tmpdiv = r5or6_last_row_offset;
		r5or6_last_row_offset = do_div(tmpdiv, r5or6_blocks_per_row);
		tmpdiv = r5or6_first_row_offset;
		(void) do_div(tmpdiv, map->strip_size);
		first_column = r5or6_first_column = tmpdiv; 
		tmpdiv = r5or6_last_row_offset;
		(void) do_div(tmpdiv, map->strip_size);
		r5or6_last_column = tmpdiv; 
#else
		first_row_offset = r5or6_first_row_offset = 
			(u32)((first_block % stripesize ) % r5or6_blocks_per_row);	

		r5or6_last_row_offset = 
			(u32)((last_block % stripesize ) % r5or6_blocks_per_row);	

		first_column = r5or6_first_column = 
			r5or6_first_row_offset / map->strip_size; 
		r5or6_last_column = 
			r5or6_last_row_offset / map->strip_size;
#endif
		if (r5or6_first_column != r5or6_last_column ) 
			return IO_ACCEL_INELIGIBLE;

		/* Request is eligible */
		map_row = ((u32)(first_row >> map->parity_rotation_shift)) %
			map->row_cnt;
			
		map_index = (first_group * 
			(map->row_cnt * total_disks_per_row)) + 
			(map_row * total_disks_per_row) + first_column; 
		
	} /* end RAID 50/60 */

	disk_handle = dd[map_index].ioaccel_handle;
	disk_block = map->disk_starting_blk + (first_row * map->strip_size) +
			(first_row_offset - (first_column * map->strip_size));
	disk_block_cnt = block_cnt;

	/* handle differing logical/physical block sizes */
	if (map->phys_blk_shift) {
		disk_block <<= map->phys_blk_shift;
		disk_block_cnt <<= map->phys_blk_shift;
	}
	BUG_ON(disk_block_cnt > 0xffff);

	/* build the new CDB for the physical disk I/O */
	if (disk_block > 0xffffffff) {
		cdb[0] = is_write ? WRITE_16 : READ_16;
		cdb[1] = 0;
		cdb[2] = (u8) (disk_block >> 56);
		cdb[3] = (u8) (disk_block >> 48);
		cdb[4] = (u8) (disk_block >> 40);
		cdb[5] = (u8) (disk_block >> 32);
		cdb[6] = (u8) (disk_block >> 24);
		cdb[7] = (u8) (disk_block >> 16);
		cdb[8] = (u8) (disk_block >> 8);
		cdb[9] = (u8) (disk_block);
		cdb[10] = (u8) (disk_block_cnt >> 24);
		cdb[11] = (u8) (disk_block_cnt >> 16);
		cdb[12] = (u8) (disk_block_cnt >> 8);
		cdb[13] = (u8) (disk_block_cnt);
		cdb[14] = 0;
		cdb[15] = 0;
		cdb_len = 16;
	} else {
		cdb[0] = is_write ? WRITE_10 : READ_10;
		cdb[1] = 0;
		cdb[2] = (u8) (disk_block >> 24);
		cdb[3] = (u8) (disk_block >> 16);
		cdb[4] = (u8) (disk_block >> 8);
		cdb[5] = (u8) (disk_block);
		cdb[6] = 0;
		cdb[7] = (u8) (disk_block_cnt >> 8);
		cdb[8] = (u8) (disk_block_cnt);
		cdb[9] = 0;
		cdb_len = 10;
	}
	return hpsa_scsi_ioaccel_queue_command(h, c, disk_handle, cdb, cdb_len,
						dev->scsi3addr);
}

/* Submit commands down the "normal" RAID stack path */
static int hpsa_ciss_submit(struct ctlr_info *h,
	struct CommandList *c, struct scsi_cmnd *cmd,
	unsigned char scsi3addr[])
{
	cmd->host_scribble = (unsigned char *) c;
	c->cmd_type = CMD_SCSI;
	c->scsi_cmd = cmd;
	c->Header.ReplyQueue = 0;  /* unused in simple mode */
	memcpy(&c->Header.LUN.LunAddrBytes[0], &scsi3addr[0], 8);
	c->Header.tag = cpu_to_le64((u64) c->cmdindex << DIRECT_LOOKUP_SHIFT);

	/* Fill in the request block... */

	c->Request.Timeout = 0;
	memset(c->Request.CDB, 0, sizeof(c->Request.CDB));
	BUG_ON(cmd->cmd_len > sizeof(c->Request.CDB));
	c->Request.CDBLen = cmd->cmd_len;
	memcpy(c->Request.CDB, cmd->cmnd, cmd->cmd_len);
	switch (cmd->sc_data_direction) {
	case DMA_TO_DEVICE:
		c->Request.type_attr_dir =
			TYPE_ATTR_DIR(TYPE_CMD, ATTR_SIMPLE, XFER_WRITE);
		break;
	case DMA_FROM_DEVICE:
		c->Request.type_attr_dir =
			TYPE_ATTR_DIR(TYPE_CMD, ATTR_SIMPLE, XFER_READ);
		break;
	case DMA_NONE:
		c->Request.type_attr_dir =
			TYPE_ATTR_DIR(TYPE_CMD, ATTR_SIMPLE, XFER_NONE);
		break;
	case DMA_BIDIRECTIONAL:
		/* This can happen if a buggy application does a scsi passthru
		 * and sets both inlen and outlen to non-zero. ( see
		 * ../scsi/scsi_ioctl.c:scsi_ioctl_send_command() )
		 */

		c->Request.type_attr_dir =
			TYPE_ATTR_DIR(TYPE_CMD, ATTR_SIMPLE, XFER_RSVD);
		/* This is technically wrong, and hpsa controllers should
		 * reject it with CMD_INVALID, which is the most correct
		 * response, but non-fibre backends appear to let it
		 * slide by, and give the same results as if this field
		 * were set correctly.  Either way is acceptable for
		 * our purposes here.
		 */

		break;

	default:
		dev_err(&h->pdev->dev, "unknown data direction: %d\n",
			cmd->sc_data_direction);
		BUG();
		break;
	}

	if (hpsa_scatter_gather(h, c, cmd) < 0) { /* Fill SG list */
		cmd_free(h, c);
		return SCSI_MLQUEUE_HOST_BUSY;
	}
	enqueue_cmd_and_start_io(h, c);
	/* the cmd'll come back via intr handler in complete_scsi_command()  */
	return 0;
}

static inline void hpsa_cmd_init(struct ctlr_info *h, int index,
				struct CommandList *c)
{
	dma_addr_t cmd_dma_handle, err_dma_handle;
	union u64bit temp64;

	/* Zero out all of commandlist except the last field, refcount */
	memset(c, 0, offsetof(struct CommandList, refcount));
	c->Header.tag = cpu_to_le64((u64) (index << DIRECT_LOOKUP_SHIFT));
	cmd_dma_handle = h->cmd_pool_dhandle + index * sizeof(*c);
	c->err_info = h->errinfo_pool + index;
	memset(c->err_info, 0, sizeof(*c->err_info));
	err_dma_handle = h->errinfo_pool_dhandle
	    + index * sizeof(*c->err_info);
	c->cmdindex = index;
	c->busaddr = (u32) cmd_dma_handle;
	temp64.val = (u64) err_dma_handle;
	c->ErrDesc.Addr = cpu_to_le64((u64) err_dma_handle);
	c->ErrDesc.Len = cpu_to_le32((u32) sizeof(*c->err_info));
	c->h = h;
}

static int hpsa_ioaccel_submit(struct ctlr_info *h,
		struct CommandList *c, struct scsi_cmnd *cmd,
		unsigned char *scsi3addr)
{
	struct hpsa_scsi_dev_t *dev = cmd->device->hostdata;
	int rc = IO_ACCEL_INELIGIBLE;

	cmd->host_scribble = (unsigned char *) c;
	c->scsi_cmd = cmd;
	c->cmd_type = CMD_SCSI;

	/* Try to honor the device's queue depth */
	atomic_inc(&dev->ioaccel_cmds_out);
	if (atomic_read(&dev->ioaccel_cmds_out) > dev->queue_depth) {
		atomic_dec(&dev->ioaccel_cmds_out);
		return IO_ACCEL_INELIGIBLE;
	}

	if (dev->offload_enabled &&
		cmd->request->cmd_type == REQ_TYPE_FS) {
		rc = hpsa_scsi_ioaccel_raid_map(h, c);
		if (rc == 0)
			return 0; /* Sent on ioaccel path */
		if (rc < 0) {   /* scsi_dma_map failed. */
			cmd_free(h, c);
			atomic_dec(&dev->ioaccel_cmds_out);
			return SCSI_MLQUEUE_HOST_BUSY;
		}
	} else if (dev->hba_ioaccel_enabled) {
		rc = hpsa_scsi_ioaccel_direct_map(h, c);
		if (rc == 0)
			return 0; /* Sent on direct map path */
		if (rc < 0) {   /* scsi_dma_map failed. */
			cmd_free(h, c);
			atomic_dec(&dev->ioaccel_cmds_out);
			return SCSI_MLQUEUE_HOST_BUSY;
		}
	}
	atomic_dec(&dev->ioaccel_cmds_out);
	return rc;
}

static void hpsa_command_resubmit_worker(struct work_struct *work)
{
	struct scsi_cmnd *cmd;
	struct hpsa_scsi_dev_t *dev;
	struct CommandList *c =
			container_of(work, struct CommandList, work);

	cmd = c->scsi_cmd;
	dev = cmd->device->hostdata;
	if (!dev) {
		cmd->result = DID_NO_CONNECT << 16;
		cmd->scsi_done(cmd);
		return;
	}
	if (c->cmd_type == CMD_IOACCEL2) {
		struct ctlr_info *h = c->h;
		struct io_accel2_cmd *c2 = &h->ioaccel2_cmd_pool[c->cmdindex];
		int rc;

		if (c2->error_data.serv_response ==
				IOACCEL2_STATUS_SR_TASK_COMP_SET_FULL) {
			rc = hpsa_ioaccel_submit(h, c, cmd, dev->scsi3addr);
			if (rc == 0)
				return;
			if (rc < 0) {
				/*
				 * If we get here, it means dma mapping failed.
				 * Try again via scsi mid layer, which will
				 * then get SCSI_MLQUEUE_HOST_BUSY.
				 */
				cmd->result = DID_IMM_RETRY << 16;
				cmd->scsi_done(cmd);
			}
			/* if rc > 0, fall thru and resubmit down CISS path */
		}
	}
	hpsa_cmd_init(c->h, c->cmdindex, c);
	if (hpsa_ciss_submit(c->h, c, cmd, dev->scsi3addr)) {
		/*
		 * If we get here, it means dma mapping failed. Try
		 * again via scsi mid layer, which will then get
		 * SCSI_MLQUEUE_HOST_BUSY.
		 */
		cmd->result = DID_IMM_RETRY << 16;
		cmd->scsi_done(cmd);
	}
}

DECLARE_QUEUECOMMAND(hpsa_scsi_queue_command)
{
	struct ctlr_info *h;
	struct hpsa_scsi_dev_t *dev;
	unsigned char scsi3addr[8];
	struct CommandList *c;
	int rc = 0;

	/* Get the ptr to our adapter structure out of cmd->host. */
	h = sdev_to_hba(cmd->device);
	dev = cmd->device->hostdata;
	if (!dev) {
		cmd->result = DID_NO_CONNECT << 16;
		hpsa_scsi_done(cmd, done);
		return 0;
	}

	if (bail_on_report_luns_if_no_scan_start(cmd, done))
		return 0;

	memcpy(scsi3addr, dev->scsi3addr, sizeof(scsi3addr));

	if (unlikely(lockup_detected(h))) {
		cmd->result = DID_NO_CONNECT << 16;
		hpsa_scsi_done(cmd, done);
		return 0;
	}
	c = cmd_alloc(h);
	if (c == NULL) {			/* trouble... */
		dev_err(&h->pdev->dev, "cmd_alloc returned NULL!\n");
		return SCSI_MLQUEUE_HOST_BUSY;
	}

	/* Fill in the command list header */
	hpsa_save_scsi_done(cmd, done);

	/* Call alternate submit routine for I/O accelerated commands.
 	 * Retries always go down the normal I/O path.
	 */
	if (likely(h->acciopath_status) && cmd->retries == 0) {
		rc = hpsa_ioaccel_submit(h, c, cmd, scsi3addr);
		if (rc == 0)
			return 0;
		if (rc < 0)
			return SCSI_MLQUEUE_HOST_BUSY;
	}
	return hpsa_ciss_submit(h, c, cmd, scsi3addr);
}

static int do_not_scan_if_controller_locked_up(struct ctlr_info *h)
{
	unsigned long flags;

	/*
	 * Don't let rescans be initiated on a controller known
	 * to be locked up.  If the controller locks up *during*
	 * a rescan, that thread is probably hosed, but at least
	 * we can prevent new rescan threads from piling up on a
	 * locked up controller.
	 */
	if (unlikely(lockup_detected(h))) {
		spin_lock_irqsave(&h->scan_lock, flags);
		h->scan_finished = 1;
		wake_up_all(&h->scan_wait_queue);
		spin_unlock_irqrestore(&h->scan_lock, flags);
		return 1;
	}
	return 0;
}

static void hpsa_scan_start(struct Scsi_Host *sh)
{
	struct ctlr_info *h = shost_to_hba(sh);
	unsigned long flags;

	if (do_not_scan_if_controller_locked_up(h))
		return;

	/* wait until any scan already in progress is finished. */
	while (1) {
		spin_lock_irqsave(&h->scan_lock, flags);
		if (h->scan_finished)
			break;
		spin_unlock_irqrestore(&h->scan_lock, flags);
		wait_event(h->scan_wait_queue, h->scan_finished);
		/* Note: We don't need to worry about a race between this
		 * thread and driver unload because the midlayer will
		 * have incremented the reference count, so unload won't
		 * happen if we're in here.
		 */
	}
	h->scan_finished = 0; /* mark scan as in progress */
	spin_unlock_irqrestore(&h->scan_lock, flags);

	if (do_not_scan_if_controller_locked_up(h))
		return;

	hpsa_update_scsi_devices(h, h->scsi_host->host_no);

	spin_lock_irqsave(&h->scan_lock, flags);
	h->scan_finished = 1; /* mark scan as finished. */
	wake_up_all(&h->scan_wait_queue);
	spin_unlock_irqrestore(&h->scan_lock, flags);
}

static int hpsa_scan_finished(struct Scsi_Host *sh,
	unsigned long elapsed_time)
{
	struct ctlr_info *h = shost_to_hba(sh);
	unsigned long flags;
	int finished;

	spin_lock_irqsave(&h->scan_lock, flags);
	finished = h->scan_finished;
	spin_unlock_irqrestore(&h->scan_lock, flags);
	return finished;
}

DECLARE_CHANGE_QUEUE_DEPTH(hpsa_change_queue_depth)
{
	struct ctlr_info *h = sdev_to_hba(sdev);

	BAIL_ON_BAD_REASON;

	if (qdepth < 1)
		qdepth = 1;
	else
		if (qdepth > h->nr_cmds)
			qdepth = h->nr_cmds;
	scsi_adjust_queue_depth(sdev, scsi_get_tag_type(sdev), qdepth);
	return sdev->queue_depth;
}

static void hpsa_unregister_scsi(struct ctlr_info *h)
{
	/* we are being forcibly unloaded, and may not refuse. */
	scsi_remove_host(h->scsi_host);
	scsi_host_put(h->scsi_host);
	h->scsi_host = NULL;
}

static int hpsa_register_scsi(struct ctlr_info *h)
{
	struct Scsi_Host *sh;
	int error;

	/* This is a no-op on kernels with working ->scan_start() */
	hpsa_initial_update_scsi_devices(h);

	sh = scsi_host_alloc(&hpsa_driver_template, sizeof(h));
	if (sh == NULL)
		goto fail;

	sh->io_port = 0;
	sh->n_io_port = 0;
	sh->this_id = -1;
	sh->max_channel = 3;
	sh->max_cmd_len = MAX_COMMAND_SIZE;
	sh->max_lun = HPSA_MAX_LUN;
	sh->max_id = HPSA_MAX_LUN;
	sh->can_queue = h->nr_cmds -
			HPSA_CMDS_RESERVED_FOR_ABORTS -
			HPSA_CMDS_RESERVED_FOR_DRIVER -
			HPSA_MAX_CONCURRENT_PASSTHRUS;
	sh->cmd_per_lun = sh->can_queue;
	sh->sg_tablesize = h->maxsgentries;
	h->scsi_host = sh;
	sh->hostdata[0] = (unsigned long) h;
	sh->irq = h->intr[h->intr_mode];
	sh->unique_id = sh->irq;
	error = scsi_add_host(sh, &h->pdev->dev);
	if (error)
		goto fail_host_put;
	scsi_scan_host(sh);
	return 0;

 fail_host_put:
	dev_err(&h->pdev->dev, "%s: scsi_add_host"
		" failed for controller %d\n", __func__, h->ctlr);
	scsi_host_put(sh);
	return error;
 fail:
	dev_err(&h->pdev->dev, "%s: scsi_host_alloc"
		" failed for controller %d\n", __func__, h->ctlr);
	return -ENOMEM;
}

static int wait_for_device_to_become_ready(struct ctlr_info *h,
	unsigned char lunaddr[])
{
	int rc;
	int count = 0;
	int waittime = 1; /* seconds */
	struct CommandList *c;

	c = cmd_alloc(h);
	if (!c) {
		dev_warn(&h->pdev->dev, "out of memory in "
			"wait_for_device_to_become_ready.\n");
		return IO_ERROR;
	}

	/* Send test unit ready until device ready, or give up. */
	while (count < HPSA_TUR_RETRY_LIMIT) {

		/* Wait for a bit.  do this first, because if we send
		 * the TUR right away, the reset will just abort it.
		 */
		msleep(1000 * waittime);
		count++;
		rc = 0; /* device ready. */

		/* Increase wait time with each try, up to a point. */
		if (waittime < HPSA_MAX_WAIT_INTERVAL_SECS)
			waittime = waittime * 2;

		/* Send the Test Unit Ready, fill_cmd can't fail, no mapping */
		(void) fill_cmd(c, TEST_UNIT_READY, h,
				NULL, 0, 0, lunaddr, TYPE_CMD);
		rc = hpsa_scsi_do_simple_cmd_core(h, c, NO_TIMEOUT);
		if (rc)
			goto do_it_again;
		/* no unmap needed here because no data xfer. */

		if (c->err_info->CommandStatus == CMD_SUCCESS)
			break;

		if (c->err_info->CommandStatus == CMD_TARGET_STATUS &&
			c->err_info->ScsiStatus == SAM_STAT_CHECK_CONDITION &&
			(c->err_info->SenseInfo[2] == NO_SENSE ||
			c->err_info->SenseInfo[2] == UNIT_ATTENTION))
			break;
do_it_again:
		dev_warn(&h->pdev->dev, "waiting %d secs "
			"for device to become ready.\n", waittime);
		rc = 1; /* device not ready. */
	}

	if (rc)
		dev_warn(&h->pdev->dev, "giving up on device.\n");
	else
		dev_warn(&h->pdev->dev, "device is ready.\n");

	cmd_free(h, c);
	return rc;
}

static int hpsa_eh_device_reset_handler(struct scsi_cmnd *scsicmd)
{
	int rc;
	struct ctlr_info *h;
	struct hpsa_scsi_dev_t *dev;

	/* find the controller to which the command to be aborted was sent */
	h = sdev_to_hba(scsicmd->device);
	if (h == NULL) /* paranoia */
		return FAILED;

	if (lockup_detected(h))
		return FAILED;

	dev = scsicmd->device->hostdata;
	if (!dev) {
		dev_err(&h->pdev->dev, "hpsa_eh_device_reset_handler: "
			"device lookup failed.\n");
		return FAILED;
	}
	dev_warn(&h->pdev->dev, "resetting device %d:%d:%d:%d\n",
		h->scsi_host->host_no, dev->bus, dev->target, dev->lun);
	/* send a reset to the SCSI LUN which the command was sent to */
	rc = hpsa_send_reset(h, dev->scsi3addr, HPSA_DEVICE_RESET_MSG);
	if (rc == 0 && wait_for_device_to_become_ready(h, dev->scsi3addr) == 0)
		return SUCCESS;

	dev_warn(&h->pdev->dev, "resetting device failed.\n");
	return FAILED;
}

static void swizzle_abort_tag(u8 *tag)
{
	u8 original_tag[8];

	memcpy(original_tag, tag, 8);
	tag[0] = original_tag[3];
	tag[1] = original_tag[2];
	tag[2] = original_tag[1];
	tag[3] = original_tag[0];
	tag[4] = original_tag[7];
	tag[5] = original_tag[6];
	tag[6] = original_tag[5];
	tag[7] = original_tag[4];
}

static void hpsa_get_tag(struct ctlr_info *h,
	struct CommandList *c, u32 *taglower, u32 *tagupper)
{

	if (c->cmd_type == CMD_IOACCEL1) {
		struct io_accel1_cmd *cm1 = (struct io_accel1_cmd *)
			&h->ioaccel_cmd_pool[c->cmdindex];
		*tagupper = (u32) (cm1->tag >> 32);
		*taglower = (u32) (cm1->tag & 0x0ffffffffULL);
		return;
	}
	if (c->cmd_type == CMD_IOACCEL2) {
		struct io_accel2_cmd *cm2 = (struct io_accel2_cmd *)
			&h->ioaccel2_cmd_pool[c->cmdindex];
		memset(tagupper, 0, sizeof(*tagupper)); /* upper tag isn't used in ioaccel2 mode */	
		*taglower = cm2->Tag;
		return;
	}
	*tagupper = (u32) (c->Header.tag >> 32);
	*taglower = (u32) (c->Header.tag & 0x0ffffffffULL);
}

static int hpsa_send_abort(struct ctlr_info *h, unsigned char *scsi3addr,
	struct CommandList *abort, int reply_queue)
{
	int rc = IO_OK;
	struct CommandList *c;
	struct ErrorInfo *ei;
	u32 tagupper, taglower;

	c = cmd_alloc(h);
	if (c == NULL) {	/* trouble... */
		dev_warn(&h->pdev->dev, "cmd_alloc returned NULL!\n");
		return -ENOMEM;
	}

	/* fill_cmd can't fail here, no buffer to map */
	(void) fill_cmd(c, HPSA_ABORT_MSG, h, &abort->Header.tag,
		0, 0, scsi3addr, TYPE_MSG);
	if (h->needs_abort_tags_swizzled)
		swizzle_abort_tag(&c->Request.CDB[4]);
	(void) __hpsa_scsi_do_simple_cmd_core(h, c, reply_queue, NO_TIMEOUT);
	hpsa_get_tag(h, abort, &taglower, &tagupper);
	dev_dbg(&h->pdev->dev, "%s: Tag:0x%08x:%08x: "
		"do_simple_cmd_core completed.\n",
		__func__, tagupper, taglower);
	/* no unmap needed here because no data xfer. */

	ei = c->err_info;
	switch (ei->CommandStatus) {
	case CMD_SUCCESS:
		break;
	case CMD_TMF_STATUS:
		rc = hpsa_evaluate_tmf_status(h, c);
		break;
	case CMD_UNABORTABLE: /* Very common, don't make noise. */
		rc = -1;
		break;
	default:
		dev_dbg(&h->pdev->dev, "%s: Tag:0x%08x:%08x: "
			"interpreting error.\n",
			__func__, tagupper, taglower);
		hpsa_scsi_interpret_error(c);
		rc = -1;
		break;
	}
	cmd_free(h, c);
	dev_dbg(&h->pdev->dev, "%s: Tag:0x%08x:%08x: "
		"Finished.\n", __func__, tagupper, taglower);
	return rc;
}

#if 0
static void setup_ioaccel2_abort_cmd(struct CommandList *c, struct ctlr_info *h,
	unsigned char *scsi3addr, struct CommandList *command_to_abort)
{
	struct io_accel2_cmd *c2 = &h->ioaccel2_cmd_pool[c->cmdindex];
	struct hpsa_tmf_struct *ac = (struct hpsa_tmf_struct *) c2;
	struct io_accel2_cmd *c2a =
		&h->ioaccel2_cmd_pool[command_to_abort->cmdindex];
	struct scsi_cmnd *scmd =
		(struct scsi_cmnd *) command_to_abort->scsi_cmd;
	struct hpsa_scsi_dev_t *dev = scmd->device->hostdata;

	/*
	 * We're overlaying struct hpsa_tmf_struct on top of something which
	 * was allocated as a struct io_accel2_cmd, so we better be sure it
	 * actually fits, and doesn't overrun the error info space.
	 */
	BUILD_BUG_ON(sizeof(struct hpsa_tmf_struct) >
			sizeof(struct io_accel2_cmd)); 
	BUG_ON(offsetof(struct io_accel2_cmd, error_data) <
			offsetof(struct hpsa_tmf_struct, error_len) +
				sizeof(ac->error_len));

	c->cmd_type = CMD_IOACCEL2;
	/* Adjust the DMA address to point to the accelerated command buffer */
	c->busaddr = (u32) h->ioaccel2_cmd_pool_dhandle +
				(c->cmdindex * sizeof(struct io_accel2_cmd));
	BUG_ON(c->busaddr & 0x0000007F);

	memset(ac, 0, sizeof(*c2)); /* yes this is correct */
	ac->iu_type = IU_TYPE_TMF;
	ac->reply_queue = smp_processor_id() % h->nreply_queues;
	ac->tmf = ABORT_TASK; 
	ac->it_nexus = cpu_to_le32((u32) dev->ioaccel_handle);
	memcpy(ac->lun_id, c2a->cciss_lun, sizeof(ac->lun_id));
	ac->Tag.lower = cpu_to_le32((c->cmdindex << DIRECT_LOOKUP_SHIFT) |
					DIRECT_LOOKUP_BIT);
	ac->abort_tag = c2a->Tag;
	ac->error_ptr = cpu_to_le64((u64) c->busaddr +
			offsetof(struct io_accel2_cmd, error_data));
	ac->error_len = cpu_to_le32((u32) sizeof(c2->error_data));
	/* Set the bits in the address sent down to include:
	 *  - performant mode bit not used in ioaccel mode 2
	 *  - pull count (bits 0-3)
	 */
}
#endif

/* ioaccel2 path firmware cannot handle abort task requests.
 * Change abort requests to physical target reset, and send to the 
 * address of the physical disk used for the ioaccel 2 command.
 * Return 0 on success (IO_OK)
 *	 -1 on failure
 */

static int hpsa_send_reset_as_abort_ioaccel2(struct ctlr_info *h,
	unsigned char *scsi3addr, struct CommandList *abort,
	__attribute__((unused)) int reply_queue)
{
	int rc = IO_OK;
        struct scsi_cmnd *scmd; /* scsi command within request being aborted */
	struct hpsa_scsi_dev_t *dev; /* device to which scsi cmd was sent */
	unsigned char phys_scsi3addr[8]; /* address of physical disk with volume */
	unsigned char *psa = &phys_scsi3addr[0];

        /* Get a pointer to the hpsa logical device. */
	scmd = abort->scsi_cmd;
	dev = (struct hpsa_scsi_dev_t *)(scmd->device->hostdata);
	if ( dev == NULL ) {
		printk(KERN_WARNING 
			"Cannot abort: no device pointer for command.\n");
			return -1; /* not abortable */
	}
	
	if (h->raid_offload_debug > 0 )
		dev_info(&h->pdev->dev,
			"Reset as abort: Abort requested on C%d:B%d:T%d:L%d "
			"scsi3addr 0x%02x%02x%02x%02x%02x%02x%02x%02x\n",
               		 h->scsi_host->host_no, dev->bus, dev->target, dev->lun,
			scsi3addr[0], scsi3addr[1], scsi3addr[2], scsi3addr[3],
			scsi3addr[4], scsi3addr[5], scsi3addr[6], scsi3addr[7]);

	if (!dev->offload_enabled) {
		dev_warn(&h->pdev->dev, "Can't abort: device is not operating "
			"in HP SSD Smart Path mode.\n"); 
		return -1; /* not abortable */
	}

	/* Incoming scsi3addr is logical addr. We need physical disk addr. */
	if (! hpsa_get_pdisk_of_ioaccel2(h, abort, psa)) {
		dev_warn(&h->pdev->dev, "Can't abort: Failed lookup of physical address.\n"); 
		return -1; /* not abortable */
	}

	/* send the reset */
	if (h->raid_offload_debug > 0 )
		dev_info(&h->pdev->dev,
			"Reset as abort: Resetting physical device at "
			"scsi3addr 0x%02x%02x%02x%02x%02x%02x%02x%02x\n",
			psa[0], psa[1], psa[2], psa[3], 
			psa[4], psa[5], psa[6], psa[7]);
	rc = hpsa_send_reset(h, psa, HPSA_PHYS_TARGET_RESET);
	if (rc != 0) {
		dev_warn(&h->pdev->dev,
			"Reset as abort: Failed on physical device at "
			"scsi3addr 0x%02x%02x%02x%02x%02x%02x%02x%02x\n",
			psa[0], psa[1], psa[2], psa[3], 
			psa[4], psa[5], psa[6], psa[7]);
		return rc; /* failed to reset */
	}

	/* wait for device to recover */
	if (wait_for_device_to_become_ready(h, psa) != 0) {
		dev_warn(&h->pdev->dev,
			"Reset as abort: Failed: Device never recovered "
			"from reset: 0x%02x%02x%02x%02x%02x%02x%02x%02x\n",
			psa[0], psa[1], psa[2], psa[3], 
			psa[4], psa[5], psa[6], psa[7]);
		return -1;  /* failed to recover */
	}

	/* device recovered */
	dev_info(&h->pdev->dev,
		"Reset as abort: Device recovered from reset: "
		"scsi3addr 0x%02x%02x%02x%02x%02x%02x%02x%02x\n",
		psa[0], psa[1], psa[2], psa[3], 
		psa[4], psa[5], psa[6], psa[7]);

	return rc; /* success */
}

#if 0
static int hpsa_send_abort_ioaccel2(struct ctlr_info *h,
	unsigned char *scsi3addr, struct CommandList *abort)
{
	int rc = IO_OK;
	struct CommandList *c;
	u32 taglower, tagupper;
	struct io_accel2_scsi_response *ei;
#if 0
	struct hpsa_scsi_dev_t *dev;

	/*
	 * FIXME:
	 * If RAID offload is not enabled on this device, don't try the
	 * abort instead do ??????
	 */

	dev = container_of(scsi3addr, struct hpsa_scsi_dev_t, scsi3addr);
	if (!dev->offload_enabled)
		return 0;
#endif

	c = cmd_alloc(h);
	if (c == NULL) {
		dev_warn(&h->pdev->dev, "cmd_alloc returned NULL!\n");
		return -ENOMEM;
	}

	setup_ioaccel2_abort_cmd(c, h, scsi3addr, abort);
	hpsa_scsi_do_simple_cmd_core(h, c);
	hpsa_get_tag(h, abort, &taglower, &tagupper);
	dev_dbg(&h->pdev->dev, "%s: Tag:0x%08x:%08x: do_simple_cmd_core completed.\n",
		__func__, tagupper, taglower);
	/* no unmap needed here because no data xfer. */

	ei = (struct io_accel2_scsi_response *) c->err_info;
	dev_dbg(&h->pdev->dev, "%s: Tag:0x%08x:%08x: abort service response = 0x%02x.\n",
		__func__, tagupper, taglower, ei->serv_response);
	switch (ei->serv_response) {
	case IOACCEL2_SERV_RESPONSE_TMF_COMPLETE:
	case IOACCEL2_SERV_RESPONSE_TMF_SUCCESS:
	case IOACCEL2_SERV_RESPONSE_TMF_REJECTED:
		rc = 0;
		break;
	case IOACCEL2_SERV_RESPONSE_FAILURE:
	case IOACCEL2_SERV_RESPONSE_TMF_WRONG_LUN:
		rc = -1;
		break;
	default:
		dev_warn(&h->pdev->dev, "%s: Tag:0x%08x:%08x: unknown abort service response x0%02x\n",
			__func__, tagupper, taglower, ei->serv_response);
		rc = -1;
	}
	cmd_free(h, c);
	dev_dbg(&h->pdev->dev, "%s: Tag:0x%08x:%08x: Finished.\n", __func__,
		tagupper, taglower);
	return rc;
	
}
#endif 

static int hpsa_send_abort_both_ways(struct ctlr_info *h,
	unsigned char *scsi3addr, struct CommandList *abort, int reply_queue)
{
	/* ioccelerator mode 2 commands should be aborted via the 
	 * accelerated path, since RAID path is unaware of these commands, 
	 * but underlying firmware can't handle abort TMF.
	 * Change abort to physical device reset.
	 */
	if (abort->cmd_type == CMD_IOACCEL2) 
		return hpsa_send_reset_as_abort_ioaccel2(h, scsi3addr,
							abort, reply_queue);
	return hpsa_send_abort(h, scsi3addr, abort, reply_queue);
}

/* Find out which reply queue a command was meant to return on */
static int hpsa_extract_reply_queue(struct ctlr_info *h,
					struct CommandList *c)
{
	if (c->cmd_type == CMD_IOACCEL2)
		return h->ioaccel2_cmd_pool[c->cmdindex].reply_queue;
	return c->Header.ReplyQueue;
}

/*
 * Limit concurrency of abort commands to prevent
 * over-subscription of commands
 */
static inline int wait_for_available_abort_cmd(struct ctlr_info *h)
{
#define ABORT_CMD_WAIT_MSECS 5000
	return !wait_event_timeout(h->abort_cmd_wait_queue,
			atomic_dec_if_positive(&h->abort_cmds_available) >= 0,
			msecs_to_jiffies(ABORT_CMD_WAIT_MSECS));
}

/* Send an abort for the specified command.
 *	If the device and controller support it,
 *		send a task abort request.
 */
static int hpsa_eh_abort_handler(struct scsi_cmnd *sc)
{

	int rc;
	struct ctlr_info *h;
	struct hpsa_scsi_dev_t *dev;
	struct CommandList *abort; /* pointer to command to be aborted */
	struct scsi_cmnd *as;	/* ptr to scsi cmd inside aborted command. */
	char msg[256];		/* For debug messaging. */
	int ml = 0;
	u32 tagupper, taglower;
	int refcount, reply_queue;

	/* Find the controller of the command to be aborted */
	h = sdev_to_hba(sc->device);
	if (!h) {
		printk(KERN_WARNING "hpsa: ABORT REQUEST FAILED, "
			"controller lookup failed\n");
		return FAILED;
	}

	if (lockup_detected(h))
		return FAILED;

	/* Check that controller supports some kind of task abort */
	if (!(HPSATMF_PHYS_TASK_ABORT & h->TMFSupportFlags) &&
		!(HPSATMF_LOG_TASK_ABORT & h->TMFSupportFlags))
		return FAILED;

	memset(msg, 0, sizeof(msg));
	ml += sprintf(msg+ml, "ABORT REQUEST on C%d:B%d:T%d:L%d ",
		h->scsi_host->host_no, sc->device->channel,
		sc->device->id, sc->device->lun);

	/* Find the device of the command to be aborted */
	dev = sc->device->hostdata;
	if (!dev) {
		dev_err(&h->pdev->dev, "%s FAILED, "
			"Device lookup failed.\n", msg);
		return FAILED;
	}

	/* Get SCSI command to be aborted */
	abort = (struct CommandList *) sc->host_scribble;
	if (abort == NULL) {
		/* This can happen if the command already completed. */
		return SUCCESS;
	}
	refcount = atomic_inc_return(&abort->refcount);
	if (refcount == 1) { /* Command is done already. */
		cmd_free(h, abort);
		return SUCCESS;
	}

	/* Don't bother trying the abort if we know it won't work. */
	if (abort->cmd_type != CMD_IOACCEL2 &&
		abort->cmd_type != CMD_IOACCEL1 && !dev->supports_aborts) {
		cmd_free(h, abort);
		return FAILED;
	}

	/* Check that we're aborting the right command.
	 * It's possible the CommandList already completed and got re-used.
	 */
	if (abort->scsi_cmd != sc) {
		cmd_free(h, abort);
		return SUCCESS;
	}

	hpsa_get_tag(h, abort, &taglower, &tagupper);
	reply_queue = hpsa_extract_reply_queue(h, abort);
	ml += sprintf(msg+ml, "Tag:0x%08x:%08x ", tagupper, taglower);
	as  = abort->scsi_cmd;
	if (as != NULL)
		ml += sprintf(msg+ml, "Command:0x%x SN:0x%lx ",
			as->cmnd[0], as->serial_number);
	dev_dbg(&h->pdev->dev, "%s\n", msg);
	dev_warn(&h->pdev->dev, "Abort request on C%d:B%d:T%d:L%d\n",
		h->scsi_host->host_no, dev->bus, dev->target, dev->lun);

	/*
	 * Command is in flight, or possibly already completed
	 * by the firmware (but not to the scsi mid layer) but we can't
	 * distinguish which.  Send the abort down.
	 */
	if (wait_for_available_abort_cmd(h)) {
		dev_warn(&h->pdev->dev,
			"Timed out waiting for an abort command to become available.\n");
		cmd_free(h, abort);
		return FAILED;
	}
	rc = hpsa_send_abort_both_ways(h, dev->scsi3addr, abort, reply_queue);
	atomic_inc(&h->abort_cmds_available);
	wake_up_all(&h->abort_cmd_wait_queue);
	if (rc != 0) {
		dev_dbg(&h->pdev->dev, "%s Request FAILED.\n", msg);
		dev_warn(&h->pdev->dev, "FAILED abort on device "
			"C%d:B%d:T%d:L%d\n",
			h->scsi_host->host_no,
			dev->bus, dev->target, dev->lun);
		cmd_free(h, abort);
		return FAILED;
	}
	dev_info(&h->pdev->dev, "%s REQUEST SUCCEEDED.\n", msg);
	cmd_free(h, abort);
	return SUCCESS;
}

/*
 * For operations that cannot sleep, a command block is allocated at init,
 * and managed by cmd_alloc() and cmd_free() using a simple bitmap to track
 * which ones are free or in use.  Lock must be held when calling this.
 * cmd_free() is the complement.
 */
static struct CommandList *cmd_alloc(struct ctlr_info *h)
{
	struct CommandList *c;
	int refcount, i;
	unsigned long offset = 0;

	/* There is some *extremely* small but non-zero chance that that
	 * multiple threads could get in here, and one thread could
	 * be scanning through the list of bits looking for a free
	 * one, but the free ones are always behind him, and other
	 * threads sneak in behind him and eat them before he can
	 * get to them, so that while there is always a free one, a
	 * very unlucky thread might be starved anyway, never able to
	 * beat the other threads.  In reality, this happens so
	 * infrequently as to be indistinguishable from never.
	 */

	offset = h->last_allocation; /* benighly racy */
	for (;;) {
		i = find_next_zero_bit(h->cmd_pool_bits, h->nr_cmds, offset);
		if (unlikely(i == h->nr_cmds)) {
			offset = 0;
			continue;
		}
		c = h->cmd_pool + i;
		refcount = atomic_inc_return(&c->refcount);
		if (unlikely(refcount > 1)) {
			cmd_free(h, c); /* already in use */
			offset = (i + 1) % h->nr_cmds;
			continue;
		}
		set_bit(i & (BITS_PER_LONG - 1),
			h->cmd_pool_bits + (i / BITS_PER_LONG));
		break; /* it's ours now. */
	}
	h->last_allocation = i; /* benignly racy */
	hpsa_cmd_init(h, i, c);
	return c;
}

static void cmd_free(struct ctlr_info *h, struct CommandList *c)
{
	if (atomic_dec_and_test(&c->refcount)) {
		int i;

		i = c - h->cmd_pool;
		clear_bit(i & (BITS_PER_LONG - 1),
			  h->cmd_pool_bits + (i / BITS_PER_LONG));
	}
}

#ifdef CONFIG_COMPAT

static int hpsa_ioctl32_passthru(struct scsi_device *dev, int cmd, void *arg)
{
	IOCTL32_Command_struct __user *arg32 =
	    (IOCTL32_Command_struct __user *) arg;
	IOCTL_Command_struct arg64;
	IOCTL_Command_struct __user *p = compat_alloc_user_space(sizeof(arg64));
	int err;
	u32 cp;

	memset(&arg64, 0, sizeof(arg64));
	err = 0;
	err |= copy_from_user(&arg64.LUN_info, &arg32->LUN_info,
			   sizeof(arg64.LUN_info));
	err |= copy_from_user(&arg64.Request, &arg32->Request,
			   sizeof(arg64.Request));
	err |= copy_from_user(&arg64.error_info, &arg32->error_info,
			   sizeof(arg64.error_info));
	err |= get_user(arg64.buf_size, &arg32->buf_size);
	err |= get_user(cp, &arg32->buf);
	arg64.buf = compat_ptr(cp);
	err |= copy_to_user(p, &arg64, sizeof(arg64));

	if (err)
		return -EFAULT;

	err = hpsa_ioctl(dev, CCISS_PASSTHRU, (void *)p);
	if (err)
		return err;
	err |= copy_in_user(&arg32->error_info, &p->error_info,
			 sizeof(arg32->error_info));
	if (err)
		return -EFAULT;
	return err;
}

static int hpsa_ioctl32_big_passthru(struct scsi_device *dev,
	int cmd, void *arg)
{
	BIG_IOCTL32_Command_struct __user *arg32 =
	    (BIG_IOCTL32_Command_struct __user *) arg;
	BIG_IOCTL_Command_struct arg64;
	BIG_IOCTL_Command_struct __user *p =
	    compat_alloc_user_space(sizeof(arg64));
	int err;
	u32 cp;

	memset(&arg64, 0, sizeof(arg64));
	err = 0;
	err |= copy_from_user(&arg64.LUN_info, &arg32->LUN_info,
			   sizeof(arg64.LUN_info));
	err |= copy_from_user(&arg64.Request, &arg32->Request,
			   sizeof(arg64.Request));
	err |= copy_from_user(&arg64.error_info, &arg32->error_info,
			   sizeof(arg64.error_info));
	err |= get_user(arg64.buf_size, &arg32->buf_size);
	err |= get_user(arg64.malloc_size, &arg32->malloc_size);
	err |= get_user(cp, &arg32->buf);
	arg64.buf = compat_ptr(cp);
	err |= copy_to_user(p, &arg64, sizeof(arg64));

	if (err)
		return -EFAULT;

	err = hpsa_ioctl(dev, CCISS_BIG_PASSTHRU, (void *)p);
	if (err)
		return err;
	err |= copy_in_user(&arg32->error_info, &p->error_info,
			 sizeof(arg32->error_info));
	if (err)
		return -EFAULT;
	return err;
}

static int hpsa_compat_ioctl(struct scsi_device *dev, int cmd, void *arg)
{
	switch (cmd) {
	case CCISS_GETPCIINFO:
	case CCISS_GETINTINFO:
	case CCISS_SETINTINFO:
	case CCISS_GETNODENAME:
	case CCISS_SETNODENAME:
	case CCISS_GETBUSTYPES:
	case CCISS_GETFIRMVER:
	case CCISS_GETDRIVVER:
	case CCISS_REVALIDVOLS:
	case CCISS_DEREGDISK:
	case CCISS_REGNEWDISK:
	case CCISS_REGNEWD:
	case CCISS_RESCANDISK:
	case CCISS_GETLUNINFO:
		return hpsa_ioctl(dev, cmd, arg);

	case CCISS_PASSTHRU32:
		return hpsa_ioctl32_passthru(dev, cmd, arg);
	case CCISS_BIG_PASSTHRU32:
		return hpsa_ioctl32_big_passthru(dev, cmd, arg);

	default:
		return -ENOIOCTLCMD;
	}
}
#endif

static int hpsa_getpciinfo_ioctl(struct ctlr_info *h, void __user *argp)
{
	struct hpsa_pci_info pciinfo;

	if (!argp)
		return -EINVAL;
	pciinfo.domain = pci_domain_nr(h->pdev->bus);
	pciinfo.bus = h->pdev->bus->number;
	pciinfo.dev_fn = h->pdev->devfn;
	pciinfo.board_id = h->board_id;
	if (copy_to_user(argp, &pciinfo, sizeof(pciinfo)))
		return -EFAULT;
	return 0;
}

static int hpsa_getdrivver_ioctl(struct ctlr_info *h, void __user *argp)
{
	DriverVer_type DriverVer;
	unsigned char vmaj, vmin, vsubmin;
	int rc;

	rc = sscanf(HPSA_DRIVER_VERSION, "%hhu.%hhu.%hhu",
		&vmaj, &vmin, &vsubmin);
	if (rc != 3) {
		dev_info(&h->pdev->dev, "driver version string '%s' "
			"unrecognized.", HPSA_DRIVER_VERSION);
		vmaj = 0;
		vmin = 0;
		vsubmin = 0;
	}
	DriverVer = (vmaj << 16) | (vmin << 8) | vsubmin;
	if (!argp)
		return -EINVAL;
	if (copy_to_user(argp, &DriverVer, sizeof(DriverVer_type)))
		return -EFAULT;
	return 0;
}

static int hpsa_passthru_ioctl(struct ctlr_info *h, void __user *argp)
{
	IOCTL_Command_struct iocommand;
	struct CommandList *c;
	char *buff = NULL;
	u64 temp64;
	int rc = 0;

	if (!argp)
		return -EINVAL;
	if (!capable(CAP_SYS_RAWIO))
		return -EPERM;
	if (copy_from_user(&iocommand, argp, sizeof(iocommand)))
		return -EFAULT;
	if ((iocommand.buf_size < 1) &&
	    (iocommand.Request.Type.Direction != XFER_NONE)) {
		return -EINVAL;
	}
	if (iocommand.buf_size > 0) {
		buff = kmalloc(iocommand.buf_size, GFP_KERNEL);
		if (buff == NULL)
			return -EFAULT;
		if (iocommand.Request.Type.Direction & XFER_WRITE) {
			/* Copy the data into the buffer we created */
			if (copy_from_user(buff, iocommand.buf,
				iocommand.buf_size)) {
				rc = -EFAULT;
				goto out_kfree;
			}
		} else {
			memset(buff, 0, iocommand.buf_size);
		}
	}
	c = cmd_alloc(h);
	if (c == NULL) {
		rc = -ENOMEM;
		goto out_kfree;
	}
	/* Fill in the command type */
	c->cmd_type = CMD_IOCTL_PEND;
	/* Fill in Command Header */
	c->Header.ReplyQueue = 0; /* unused in simple mode */
	if (iocommand.buf_size > 0) {	/* buffer to fill */
		c->Header.SGList = (u8) 1;
		c->Header.SGTotal = cpu_to_le16(1);
	} else	{ /* no buffers to fill */
		c->Header.SGList = 0;
		c->Header.SGTotal = cpu_to_le16(0);
	}
	memcpy(&c->Header.LUN, &iocommand.LUN_info, sizeof(c->Header.LUN));

	/* Fill in Request block */
	memcpy(&c->Request, &iocommand.Request,
		sizeof(c->Request));

	/* Fill in the scatter gather information */
	if (iocommand.buf_size > 0) {
		temp64 = (u64) pci_map_single(h->pdev, buff,
			iocommand.buf_size, PCI_DMA_BIDIRECTIONAL);
		if (hpsa_dma_mapping_error(&h->pdev->dev,
						(dma_addr_t) temp64)) {
			c->SG[0].Addr = cpu_to_le64(0);
			c->SG[0].Len = cpu_to_le32(0);
			rc = -ENOMEM;
			goto out;
		}
		c->SG[0].Addr = cpu_to_le64(temp64);
		c->SG[0].Len = cpu_to_le32(iocommand.buf_size);
		c->SG[0].Ext = cpu_to_le32(HPSA_SG_LAST); /* not chaining */
	}
	rc = hpsa_scsi_do_simple_cmd_core_if_no_lockup(h, c, NO_TIMEOUT);
	if (rc)
		rc = -EIO;
	if (iocommand.buf_size > 0)
		hpsa_pci_unmap(h->pdev, c, 1, PCI_DMA_BIDIRECTIONAL);
	check_ioctl_unit_attention(h, c);

	/* Copy the error information out */
	memcpy(&iocommand.error_info, c->err_info,
		sizeof(iocommand.error_info));
	if (copy_to_user(argp, &iocommand, sizeof(iocommand))) {
		rc = -EFAULT;
		goto out;
	}
	if ((iocommand.Request.Type.Direction & XFER_READ) &&
		iocommand.buf_size > 0) {
		/* Copy the data out of the buffer we created */
		if (copy_to_user(iocommand.buf, buff, iocommand.buf_size)) {
			rc = -EFAULT;
			goto out;
		}
	}
out:
	cmd_free(h, c);
out_kfree:
	kfree(buff);
	return rc;
}

static int hpsa_big_passthru_ioctl(struct ctlr_info *h, void __user *argp)
{
	BIG_IOCTL_Command_struct *ioc;
	struct CommandList *c;
	unsigned char **buff = NULL;
	int *buff_size = NULL;
	u64 temp64;
	BYTE sg_used = 0;
	int status = 0;
	int i;
	u32 left;
	u32 sz;
	BYTE __user *data_ptr;

	if (!argp)
		return -EINVAL;
	if (!capable(CAP_SYS_RAWIO))
		return -EPERM;
	ioc = (BIG_IOCTL_Command_struct *)
	    kmalloc(sizeof(*ioc), GFP_KERNEL);
	if (!ioc) {
		status = -ENOMEM;
		goto cleanup1;
	}
	if (copy_from_user(ioc, argp, sizeof(*ioc))) {
		status = -EFAULT;
		goto cleanup1;
	}
	if ((ioc->buf_size < 1) &&
	    (ioc->Request.Type.Direction != XFER_NONE)) {
		status = -EINVAL;
		goto cleanup1;
	}
	/* Check kmalloc limits  using all SGs */
	if (ioc->malloc_size > MAX_KMALLOC_SIZE) {
		status = -EINVAL;
		goto cleanup1;
	}
	if (ioc->buf_size > ioc->malloc_size * SG_ENTRIES_IN_CMD) {
		status = -EINVAL;
		goto cleanup1;
	}
	buff = kzalloc(SG_ENTRIES_IN_CMD * sizeof(char *), GFP_KERNEL);
	if (!buff) {
		status = -ENOMEM;
		goto cleanup1;
	}
	buff_size = kmalloc(SG_ENTRIES_IN_CMD * sizeof(int), GFP_KERNEL);
	if (!buff_size) {
		status = -ENOMEM;
		goto cleanup1;
	}
	left = ioc->buf_size;
	data_ptr = ioc->buf;
	while (left) {
		sz = (left > ioc->malloc_size) ? ioc->malloc_size : left;
		buff_size[sg_used] = sz;
		buff[sg_used] = kmalloc(sz, GFP_KERNEL);
		if (buff[sg_used] == NULL) {
			status = -ENOMEM;
			goto cleanup1;
		}
		if (ioc->Request.Type.Direction & XFER_WRITE) {
			if (copy_from_user(buff[sg_used], data_ptr, sz)) {
				status = -EFAULT;
				goto cleanup1;
			}
		} else
			memset(buff[sg_used], 0, sz);
		left -= sz;
		data_ptr += sz;
		sg_used++;
	}
	c = cmd_alloc(h);
	if (c == NULL) {
		status = -ENOMEM;
		goto cleanup1;
	}
	c->cmd_type = CMD_IOCTL_PEND;
	c->Header.ReplyQueue = 0;
	c->Header.SGList = (u8) sg_used;
	c->Header.SGTotal = cpu_to_le16(sg_used);
	memcpy(&c->Header.LUN, &ioc->LUN_info, sizeof(c->Header.LUN));
	memcpy(&c->Request, &ioc->Request, sizeof(c->Request));
	if (ioc->buf_size > 0) {
		int i;
		for (i = 0; i < sg_used; i++) {
			temp64 = (u64) pci_map_single(h->pdev, buff[i],
				    buff_size[i], PCI_DMA_BIDIRECTIONAL);
			if (hpsa_dma_mapping_error(&h->pdev->dev,
							(dma_addr_t) temp64)) {
				c->SG[i].Addr = 0;
				c->SG[i].Len = 0;
				hpsa_pci_unmap(h->pdev, c, i,
					PCI_DMA_BIDIRECTIONAL);
				status = -ENOMEM;
				goto cleanup0;
			}
			c->SG[i].Addr = cpu_to_le64(temp64);
			c->SG[i].Len = cpu_to_le32(buff_size[i]);
			/* we are not chaining */
			c->SG[i].Ext =
				cpu_to_le32((i == sg_used) * HPSA_SG_LAST);
		}
	}
	status = hpsa_scsi_do_simple_cmd_core_if_no_lockup(h, c, NO_TIMEOUT);
	if (status) {
		status = -EIO;
		goto cleanup0;
	}
	if (sg_used)
		hpsa_pci_unmap(h->pdev, c, sg_used, PCI_DMA_BIDIRECTIONAL);
	check_ioctl_unit_attention(h, c);
	/* Copy the error information out */
	memcpy(&ioc->error_info, c->err_info, sizeof(ioc->error_info));
	if (copy_to_user(argp, ioc, sizeof(*ioc))) {
		status = -EFAULT;
		goto cleanup0;
	}
	if ((ioc->Request.Type.Direction & XFER_READ) && ioc->buf_size > 0) {
		/* Copy the data out of the buffer we created */
		BYTE __user *ptr = ioc->buf;
		for (i = 0; i < sg_used; i++) {
			if (copy_to_user(ptr, buff[i], buff_size[i])) {
				status = -EFAULT;
				goto cleanup0;
			}
			ptr += buff_size[i];
		}
	}
	status = 0;
cleanup0:
	cmd_free(h, c);
cleanup1:
	if (buff) {
		for (i = 0; i < sg_used; i++)
			kfree(buff[i]);
		kfree(buff);
	}
	kfree(buff_size);
	kfree(ioc);
	return status;
}

static void check_ioctl_unit_attention(struct ctlr_info *h,
	struct CommandList *c)
{
	if (c->err_info->CommandStatus == CMD_TARGET_STATUS &&
			c->err_info->ScsiStatus != SAM_STAT_CHECK_CONDITION)
		(void) check_for_unit_attention(h, c);
}

/*
 * ioctl
 */
static int hpsa_ioctl(struct scsi_device *dev, int cmd, void *arg)
{
	struct ctlr_info *h;
	void __user *argp = (void __user *)arg;
	int rc;

	h = sdev_to_hba(dev);

	switch (cmd) {
	case CCISS_DEREGDISK:
	case CCISS_REGNEWDISK:
	case CCISS_REGNEWD:
		hpsa_scan_start(h->scsi_host);
		return 0;
	case CCISS_GETPCIINFO:
		return hpsa_getpciinfo_ioctl(h, argp);
	case CCISS_GETDRIVVER:
		return hpsa_getdrivver_ioctl(h, argp);
	case CCISS_PASSTHRU:
		if (atomic_dec_if_positive(&h->passthru_cmds_avail) < 0)
			return -EAGAIN;
		rc = hpsa_passthru_ioctl(h, argp);
		atomic_inc(&h->passthru_cmds_avail);
		return rc;
	case CCISS_BIG_PASSTHRU:
		if (atomic_dec_if_positive(&h->passthru_cmds_avail) < 0)
			return -EAGAIN;
		rc = hpsa_big_passthru_ioctl(h, argp);
		atomic_inc(&h->passthru_cmds_avail);
		return rc;
	default:
		return -ENOTTY;
	}
}

static int __devinit hpsa_send_host_reset(struct ctlr_info *h, unsigned char *scsi3addr,
	u8 reset_type)
{
	struct CommandList *c;

	c = cmd_alloc(h);
	if (!c)
		return -ENOMEM;
	/* fill_cmd can't fail here, no data buffer to map */
	(void) fill_cmd(c, HPSA_DEVICE_RESET_MSG, h, NULL, 0, 0,
		RAID_CTLR_LUNID, TYPE_MSG);
	c->Request.CDB[1] = reset_type; /* fill_cmd defaults to target reset */
	c->waiting = NULL;
	enqueue_cmd_and_start_io(h, c);
	/* Don't wait for completion, the reset won't complete.  Don't free
	 * the command either.  This is the last command we will send before
	 * re-initializing everything, so it doesn't matter and won't leak.
	 */
	return 0;
}

static int fill_cmd(struct CommandList *c, u8 cmd, struct ctlr_info *h,
	void *buff, size_t size, u16 page_code, unsigned char *scsi3addr,
	int cmd_type)
{
	int pci_dir = XFER_NONE;
	u64 tag; /* for commands to be aborted */
	u32 tupper, tlower;

	c->cmd_type = CMD_IOCTL_PEND;
	c->Header.ReplyQueue = 0;
	if (buff != NULL && size > 0) {
		c->Header.SGList = (u8) 1;
		c->Header.SGTotal = cpu_to_le32(1);
	} else {
		c->Header.SGList = 0;
		c->Header.SGTotal = 0;
	}
	memcpy(c->Header.LUN.LunAddrBytes, scsi3addr, 8);

	if (cmd_type == TYPE_CMD) {
		switch (cmd) {
		case HPSA_INQUIRY:
			/* are we trying to read a vital product page */
			if (page_code & VPD_PAGE) {
				c->Request.CDB[1] = 0x01;
				c->Request.CDB[2] = (page_code & 0xff);
			}
			c->Request.CDBLen = 6;
			c->Request.type_attr_dir =
				TYPE_ATTR_DIR(cmd_type, ATTR_SIMPLE, XFER_READ);
			c->Request.Timeout = 0;
			c->Request.CDB[0] = HPSA_INQUIRY;
			c->Request.CDB[4] = size & 0xFF;
			break;
		case HPSA_REPORT_LOG:
		case HPSA_REPORT_PHYS:
			/* Talking to controller so It's a physical command
			   mode = 00 target = 0.  Nothing to write.
			 */
			c->Request.CDBLen = 12;
			c->Request.type_attr_dir =
				TYPE_ATTR_DIR(cmd_type, ATTR_SIMPLE, XFER_READ);
			c->Request.Timeout = 0;
			c->Request.CDB[0] = cmd;
			c->Request.CDB[6] = (size >> 24) & 0xFF; /* MSB */
			c->Request.CDB[7] = (size >> 16) & 0xFF;
			c->Request.CDB[8] = (size >> 8) & 0xFF;
			c->Request.CDB[9] = size & 0xFF;
			break;
		case HPSA_CACHE_FLUSH:
			c->Request.CDBLen = 12;
			c->Request.type_attr_dir =
					TYPE_ATTR_DIR(cmd_type,
						ATTR_SIMPLE, XFER_WRITE);
			c->Request.Timeout = 0;
			c->Request.CDB[0] = BMIC_WRITE;
			c->Request.CDB[6] = BMIC_CACHE_FLUSH;
			c->Request.CDB[7] = (size >> 8) & 0xFF;
			c->Request.CDB[8] = size & 0xFF;
			break;
		case TEST_UNIT_READY:
			c->Request.CDBLen = 6;
			c->Request.type_attr_dir =
				TYPE_ATTR_DIR(cmd_type, ATTR_SIMPLE, XFER_NONE);
			c->Request.Timeout = 0;
			break;
		case HPSA_GET_RAID_MAP:
			c->Request.CDBLen = 12;
			c->Request.type_attr_dir =
				TYPE_ATTR_DIR(cmd_type, ATTR_SIMPLE, XFER_READ);
			c->Request.Timeout = 0;
			c->Request.CDB[0] = HPSA_CISS_READ;
			c->Request.CDB[1] = cmd;
			c->Request.CDB[6] = (size >> 24) & 0xFF; /* MSB */
			c->Request.CDB[7] = (size >> 16) & 0xFF;
			c->Request.CDB[8] = (size >> 8) & 0xFF;
			c->Request.CDB[9] = size & 0xFF;
			break;
		case BMIC_SENSE_CONTROLLER_PARAMETERS:
			c->Request.CDBLen = 10;
			c->Request.type_attr_dir =
				TYPE_ATTR_DIR(cmd_type, ATTR_SIMPLE, XFER_READ);
			c->Request.Timeout = 0;
			c->Request.CDB[0] = BMIC_READ;
			c->Request.CDB[6] = BMIC_SENSE_CONTROLLER_PARAMETERS;
			c->Request.CDB[7] = (size >> 16) & 0xFF;
			c->Request.CDB[8] = (size >> 8) & 0xFF;
			break;
		case BMIC_IDENTIFY_PHYSICAL_DEVICE:
			c->Request.CDBLen = 10;
			c->Request.type_attr_dir =
				TYPE_ATTR_DIR(cmd_type, ATTR_SIMPLE, XFER_READ);
			c->Request.Timeout = 0;
			c->Request.CDB[0] = BMIC_READ;
			c->Request.CDB[6] = BMIC_IDENTIFY_PHYSICAL_DEVICE;
			c->Request.CDB[7] = (size >> 16) & 0xFF;
			c->Request.CDB[8] = (size >> 8) & 0XFF;
			break;
		default:
			dev_warn(&h->pdev->dev, "unknown command 0x%c\n", cmd);
			BUG();
			return -1;
		}
	} else if (cmd_type == TYPE_MSG) {
		switch (cmd) {

		case  HPSA_PHYS_TARGET_RESET:
			c->Request.CDBLen = 16;
			c->Request.type_attr_dir =
				TYPE_ATTR_DIR(cmd_type, ATTR_SIMPLE, XFER_NONE);
			c->Request.Timeout = 0; /* Don't time out */
			memset(&c->Request.CDB[0], 0, sizeof(c->Request.CDB));
			c->Request.CDB[0] = HPSA_RESET;
			c->Request.CDB[1] = HPSA_TARGET_RESET_TYPE;
			/* Physical target reset needs no control bytes 4-7*/
			c->Request.CDB[4] = 0x00;
			c->Request.CDB[5] = 0x00;
			c->Request.CDB[6] = 0x00;
			c->Request.CDB[7] = 0x00;
			break;
		case  HPSA_DEVICE_RESET_MSG:
			c->Request.CDBLen = 16;
			c->Request.type_attr_dir =
				TYPE_ATTR_DIR(cmd_type, ATTR_SIMPLE, XFER_NONE);
			c->Request.Timeout = 0; /* Don't time out */
			memset(&c->Request.CDB[0], 0, sizeof(c->Request.CDB));
			c->Request.CDB[0] =  cmd;
			c->Request.CDB[1] = HPSA_LUN_RESET_TYPE;
			/* If bytes 4-7 are zero, it means reset the */
			/* LunID device */
			c->Request.CDB[4] = 0x00;
			c->Request.CDB[5] = 0x00;
			c->Request.CDB[6] = 0x00;
			c->Request.CDB[7] = 0x00;
			break;
		case  HPSA_ABORT_MSG:
			memcpy(&tag, buff, sizeof(tag));
			if (buff == NULL) {
				dev_warn(&h->pdev->dev, "Null Abort request.\n");
				BUG();
			}
			dev_dbg(&h->pdev->dev,
				"Abort Tag:0x%016llx using request Tag:0x%016llx",
				tag, c->Header.tag);
			tlower = (u32) (tag >> 32);
			tupper = (u32) (tag & 0x0ffffffffULL);
			c->Request.CDBLen = 16;
			c->Request.type_attr_dir =
					TYPE_ATTR_DIR(cmd_type,
						ATTR_SIMPLE, XFER_WRITE);
			c->Request.Timeout = 0; /* Don't time out */
			c->Request.CDB[0] = HPSA_TASK_MANAGEMENT;
			c->Request.CDB[1] = HPSA_TMF_ABORT_TASK;
			c->Request.CDB[2] = 0x00; /* reserved */
			c->Request.CDB[3] = 0x00; /* reserved */
			/* Tag to abort goes in CDB[4]-CDB[11] */

			/* FIXME: Double check the byte swapping here. */
			c->Request.CDB[4] = tlower & 0xFF;
			c->Request.CDB[5] = (tlower >> 8) & 0xFF;
			c->Request.CDB[6] = (tlower >> 16) & 0xFF;
			c->Request.CDB[7] = (tlower >> 24) & 0xFF;
			c->Request.CDB[8] = tupper & 0xFF;
			c->Request.CDB[9] = (tupper >> 8) & 0xFF;
			c->Request.CDB[10] = (tupper >> 16) & 0xFF;
			c->Request.CDB[11] = tupper & 0xFF;

			c->Request.CDB[12] = 0x00; /* reserved */
			c->Request.CDB[13] = 0x00; /* reserved */
			c->Request.CDB[14] = 0x00; /* reserved */
			c->Request.CDB[15] = 0x00; /* reserved */
		break;
		default:
			dev_warn(&h->pdev->dev, "unknown message type %d\n",
				cmd);
			BUG();
		}
	} else {
		dev_warn(&h->pdev->dev, "unknown command type %d\n", cmd_type);
		BUG();
	}

	switch (GET_DIR(c->Request.type_attr_dir)) {
	case XFER_READ:
		pci_dir = PCI_DMA_FROMDEVICE;
		break;
	case XFER_WRITE:
		pci_dir = PCI_DMA_TODEVICE;
		break;
	case XFER_NONE:
		pci_dir = PCI_DMA_NONE;
		break;
	default:
		pci_dir = PCI_DMA_BIDIRECTIONAL;
	}
	if (hpsa_map_one(h->pdev, c, buff, size, pci_dir))
		return -1;
	return 0;
}

/*
 * Map (physical) PCI mem into (virtual) kernel space
 */
static void __iomem *remap_pci_mem(ulong base, ulong size)
{
	ulong page_base = ((ulong) base) & PAGE_MASK;
	ulong page_offs = ((ulong) base) - page_base;
	void __iomem *page_remapped = ioremap_nocache(page_base, page_offs + size);

	return page_remapped ? (page_remapped + page_offs) : NULL;
}

static inline unsigned long get_next_completion(struct ctlr_info *h, u8 q)
{
	return h->access.command_completed(h, q);
}

static inline int interrupt_pending(struct ctlr_info *h)
{
	return h->access.intr_pending(h);
}

static inline long interrupt_not_for_us(struct ctlr_info *h)
{
	return (h->access.intr_pending(h) == 0) ||
		(h->interrupts_enabled == 0);
}

static inline int bad_tag(struct ctlr_info *h, u32 tag_index,
	u32 raw_tag)
{
	if (unlikely(tag_index >= h->nr_cmds)) {
		dev_warn(&h->pdev->dev, "bad tag 0x%08x ignored.\n", raw_tag);
		return 1;
	}
	return 0;
}

static inline void finish_cmd(struct CommandList *c)
{
	dial_up_lockup_detection_on_fw_flash_complete(c->h, c);
	if (likely(c->cmd_type == CMD_IOACCEL1 || c->cmd_type == CMD_SCSI
			|| c->cmd_type == CMD_IOACCEL2))
		complete_scsi_command(c);
	else if (c->cmd_type == CMD_IOCTL_PEND)
		complete(c->waiting);
}

static inline u32 hpsa_tag_discard_error_bits(struct ctlr_info *h, u32 tag)
{
#define HPSA_PERF_ERROR_BITS ((1 << DIRECT_LOOKUP_SHIFT) - 1)
#define HPSA_SIMPLE_ERROR_BITS 0x03
	if (unlikely(!(h->transMethod & CFGTBL_Trans_Performant)))
		return tag & ~HPSA_SIMPLE_ERROR_BITS;
	return tag & ~HPSA_PERF_ERROR_BITS;
}

/* process completion of an indexed ("direct lookup") command */
static inline void process_indexed_cmd(struct ctlr_info *h,
	u32 raw_tag)
{
	u32 tag_index;
	struct CommandList *c;

	tag_index = raw_tag >> DIRECT_LOOKUP_SHIFT;
	if (!bad_tag(h, tag_index, raw_tag)) {
		c = h->cmd_pool + tag_index;
		finish_cmd(c);
	}
}

/* Some controllers, like p400, will give us one interrupt
 * after a soft reset, even if we turned interrupts off.
 * Only need to check for this in the hpsa_xxx_discard_completions
 * functions.
 */
static int ignore_bogus_interrupt(struct ctlr_info *h)
{
	if (likely(!reset_devices))
		return 0;

	if (likely(h->interrupts_enabled))
		return 0;

	dev_info(&h->pdev->dev, "Received interrupt while interrupts disabled "
		"(known firmware bug.)  Ignoring.\n");

	return 1;
}

/*
 * Convert &h->q[x] (passed to interrupt handlers) back to h.
 * Relies on (h-q[x] == x) being true for x such that
 * 0 <= x < h->nreply_queues.
 */
static struct ctlr_info *queue_to_hba(u8 *queue)
{
	return container_of((queue - *queue), struct ctlr_info, q[0]);
}

DECLARE_INTERRUPT_HANDLER(hpsa_intx_discard_completions)
{
	struct ctlr_info *h = queue_to_hba(queue);
	u8 q = *(u8 *) queue;
	u32 raw_tag;

	if (ignore_bogus_interrupt(h))
		return IRQ_NONE;

	if (interrupt_not_for_us(h))
		return IRQ_NONE;
	h->last_intr_timestamp = get_jiffies_64();
	while (interrupt_pending(h)) {
		raw_tag = get_next_completion(h, q);
		while (raw_tag != FIFO_EMPTY)
			raw_tag = next_command(h, q);
	}
	return IRQ_HANDLED;
}

DECLARE_INTERRUPT_HANDLER(hpsa_msix_discard_completions)
{
	struct ctlr_info *h = queue_to_hba(queue);
	u32 raw_tag;
	u8 q = *(u8 *) queue;

	if (ignore_bogus_interrupt(h))
		return IRQ_NONE;

	h->last_intr_timestamp = get_jiffies_64();
	raw_tag = get_next_completion(h, q);
	while (raw_tag != FIFO_EMPTY)
		raw_tag = next_command(h, q);
	return IRQ_HANDLED;
}

DECLARE_INTERRUPT_HANDLER(do_hpsa_intr_intx)
{
	struct ctlr_info *h = queue_to_hba(queue);
	u32 raw_tag;
	u8 q = *(u8 *) queue;

	if (interrupt_not_for_us(h))
		return IRQ_NONE;
	h->last_intr_timestamp = get_jiffies_64();
	while (interrupt_pending(h)) {
		raw_tag = get_next_completion(h, q);
		while (raw_tag != FIFO_EMPTY) {
			process_indexed_cmd(h, raw_tag);
			raw_tag = next_command(h, q);
		}
	}
	return IRQ_HANDLED;
}

DECLARE_INTERRUPT_HANDLER(do_hpsa_intr_msi)
{
	struct ctlr_info *h = queue_to_hba(queue);
	u32 raw_tag;
	u8 q = *(u8 *) queue;

	h->last_intr_timestamp = get_jiffies_64();
	raw_tag = get_next_completion(h, q);
	while (raw_tag != FIFO_EMPTY) {
		process_indexed_cmd(h, raw_tag);
		raw_tag = next_command(h, q);
	}
	return IRQ_HANDLED;
}

/* Send a message CDB to the firmware.  Careful, this only works
 * in simple mode, not performant mode due to the tag lookup.
 * We only ever use this immediately after a controller reset.
 */
static __devinit int hpsa_message(struct pci_dev *pdev, unsigned char opcode,
						unsigned char type)
{
	struct Command {
		struct CommandListHeader CommandHeader;
		struct RequestBlock Request;
		struct ErrDescriptor ErrorDescriptor;
	};
	struct Command *cmd;
	static const size_t cmd_sz = sizeof(*cmd) +
					sizeof(cmd->ErrorDescriptor);
	dma_addr_t paddr64;
	uint32_t paddr32, tag;
	void __iomem *vaddr;
	int i, err;

	/* kernel.org uses pci_remap_bar() here, but 2.6.27 doesn't have it.*/
	vaddr = ioremap_nocache(pci_resource_start(pdev, 0),
					pci_resource_len(pdev, 0));
	if (vaddr == NULL)
		return -ENOMEM;

	/* The Inbound Post Queue only accepts 32-bit physical addresses for the
	 * CCISS commands, so they must be allocated from the lower 4GiB of
	 * memory.
	 */
	err = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(32));
	if (err) {
		iounmap(vaddr);
		return -ENOMEM;
	}

	cmd = pci_alloc_consistent(pdev, cmd_sz, &paddr64);
	if (cmd == NULL) {
		iounmap(vaddr);
		return -ENOMEM;
	}

	/* This must fit, because of the 32-bit consistent DMA mask.  Also,
	 * although there's no guarantee, we assume that the address is at
	 * least 4-byte aligned (most likely, it's page-aligned).
	 */
	paddr32 = paddr64;

	cmd->CommandHeader.ReplyQueue = 0;
	cmd->CommandHeader.SGList = 0;
	cmd->CommandHeader.SGTotal = 0;
	cmd->CommandHeader.tag = (u64) paddr32;
	memset(&cmd->CommandHeader.LUN.LunAddrBytes, 0, 8);

	cmd->Request.CDBLen = 16;
	cmd->Request.type_attr_dir =
			TYPE_ATTR_DIR(TYPE_MSG, ATTR_HEADOFQUEUE, XFER_NONE);
	cmd->Request.Timeout = 0; /* Don't time out */
	cmd->Request.CDB[0] = opcode;
	cmd->Request.CDB[1] = type;
	memset(&cmd->Request.CDB[2], 0, 14); /* rest of the CDB is reserved */
	cmd->ErrorDescriptor.Addr =
			cpu_to_le64((u64) (paddr32 + sizeof(*cmd)));
	cmd->ErrorDescriptor.Len = cpu_to_le32(sizeof(struct ErrorInfo));

	writel(paddr32, vaddr + SA5_REQUEST_PORT_OFFSET);

	for (i = 0; i < HPSA_MSG_SEND_RETRY_LIMIT; i++) {
		tag = readl(vaddr + SA5_REPLY_PORT_OFFSET);
		if ((tag & ~HPSA_SIMPLE_ERROR_BITS) == paddr32)
			break;
		msleep(HPSA_MSG_SEND_RETRY_INTERVAL_MSECS);
	}

	iounmap(vaddr);

	/* we leak the DMA buffer here ... no choice since the controller could
	 *  still complete the command.
	 */
	if (i == HPSA_MSG_SEND_RETRY_LIMIT) {
		dev_err(&pdev->dev, "controller message %02x:%02x timed out\n",
			opcode, type);
		return -ETIMEDOUT;
	}

	pci_free_consistent(pdev, cmd_sz, cmd, paddr64);

	if (tag & HPSA_ERROR_BIT) {
		dev_err(&pdev->dev, "controller message %02x:%02x failed\n",
			opcode, type);
		return -EIO;
	}

	dev_info(&pdev->dev, "controller message %02x:%02x succeeded\n",
		opcode, type);
	return 0;
}

#define hpsa_noop(p) hpsa_message(p, 3, 0)

static int hpsa_controller_hard_reset(struct pci_dev *pdev,
	void * __iomem vaddr, u32 use_doorbell)
{
	u16 pmcsr;
	int pos;

	if (use_doorbell) {
		/* For everything after the P600, the PCI power state method
		 * of resetting the controller doesn't work, so we have this
		 * other way using the doorbell register.
		 */
		dev_info(&pdev->dev, "using doorbell to reset controller\n");
		writel(use_doorbell, vaddr + SA5_DOORBELL);

		/* PMC hardware guys tell us we need a 10 second delay after
		 * doorbell reset and before any attempt to talk to the board
		 * at all to ensure that this actually works and doesn't fall
		 * over in some weird corner cases.
		 */
		msleep(10000);
	} else { /* Try to do it the PCI power state way */

		/* Quoting from the Open CISS Specification: "The Power
		 * Management Control/Status Register (CSR) controls the power
		 * state of the device.  The normal operating state is D0,
		 * CSR=00h.  The software off state is D3, CSR=03h.  To reset
		 * the controller, place the interface device in D3 then to D0,
		 * this causes a secondary PCI reset which will reset the
		 * controller." */

		pos = pci_find_capability(pdev, PCI_CAP_ID_PM);
		if (pos == 0) {
			dev_err(&pdev->dev,
				"hpsa_reset_controller: "
				"PCI PM not supported\n");
			return -ENODEV;
		}
		dev_info(&pdev->dev, "using PCI PM to reset controller\n");
		/* enter the D3hot power management state */
		pci_read_config_word(pdev, pos + PCI_PM_CTRL, &pmcsr);
		pmcsr &= ~PCI_PM_CTRL_STATE_MASK;
		pmcsr |= PCI_D3hot;
		pci_write_config_word(pdev, pos + PCI_PM_CTRL, pmcsr);

		msleep(500);

		/* enter the D0 power management state */
		pmcsr &= ~PCI_PM_CTRL_STATE_MASK;
		pmcsr |= PCI_D0;
		pci_write_config_word(pdev, pos + PCI_PM_CTRL, pmcsr);
	}
	return 0;
}

static int hpsa_wait_for_board_state(struct pci_dev *pdev,
	void __iomem *vaddr, int wait_for_ready)
{
	int i, iterations;
	u32 scratchpad;
	if (wait_for_ready)
		iterations = HPSA_BOARD_READY_ITERATIONS;
	else
		iterations = HPSA_BOARD_NOT_READY_ITERATIONS;

	for (i = 0; i < iterations; i++) {
		scratchpad = readl(vaddr + SA5_SCRATCHPAD_OFFSET);
		if (wait_for_ready) {
			if (scratchpad == HPSA_FIRMWARE_READY)
				return 0;
		} else {
			if (scratchpad != HPSA_FIRMWARE_READY)
				return 0;
		}
		msleep(HPSA_BOARD_READY_POLL_INTERVAL_MSECS);
	}
	dev_warn(&pdev->dev, "board not ready, timed out.\n");
	return -ENODEV;
}

static __devinit void init_driver_version(char *driver_version, int len)
{
	memset(driver_version, 0, len);
	strncpy(driver_version, HPSA " " HPSA_DRIVER_VERSION, len - 1);
}

static __devinit int write_driver_ver_to_cfgtable(
	struct CfgTable __iomem *cfgtable)
{
	char *driver_version;
	int i, size = sizeof(cfgtable->driver_version);

	driver_version = kmalloc(size, GFP_KERNEL);
	if (!driver_version)
		return -ENOMEM;

	init_driver_version(driver_version, size);
	for (i = 0; i < size; i++)
		writeb(driver_version[i], &cfgtable->driver_version[i]);
	kfree(driver_version);
	return 0;
}

static __devinit void read_driver_ver_from_cfgtable(
	struct CfgTable __iomem *cfgtable, unsigned char *driver_ver)
{
	int i;

	for (i = 0; i < sizeof(cfgtable->driver_version); i++)
		driver_ver[i] = readb(&cfgtable->driver_version[i]);
}

static __devinit int controller_reset_failed(
	struct CfgTable __iomem *cfgtable)
{
	char *driver_ver, *old_driver_ver;
	int rc, size = sizeof(cfgtable->driver_version);

	old_driver_ver = kmalloc(2 * size, GFP_KERNEL);
	if (!old_driver_ver ) 
		return 1; 
	driver_ver = old_driver_ver + size;

	/* After a reset, the 32 bytes of "driver version" in the cfgtable
	 * should have been changed, otherwise we know the reset failed.
	 */
	init_driver_version(old_driver_ver, size);
	read_driver_ver_from_cfgtable(cfgtable, driver_ver);
	rc = !memcmp(driver_ver, old_driver_ver, size);
	kfree(old_driver_ver);
	return rc;
}

/* This does a hard reset of the controller using PCI power management
 * states or the using the doorbell register.
 */
static __devinit int hpsa_kdump_hard_reset_controller(struct pci_dev *pdev)
{
	u64 cfg_offset;
	u32 cfg_base_addr;
	u64 cfg_base_addr_index;
	void __iomem *vaddr;
	unsigned long paddr;
	u32 misc_fw_support;
	int rc;
	struct CfgTable __iomem *cfgtable;
	u32 use_doorbell;
	u32 board_id;
	u16 command_register;

	/* For controllers as old as the P600, this is very nearly
	 * the same thing as
	 *
	 * pci_save_state(pci_dev);
	 * pci_set_power_state(pci_dev, PCI_D3hot);
	 * pci_set_power_state(pci_dev, PCI_D0);
	 * pci_restore_state(pci_dev);
	 *
	 * For controllers newer than the P600, the pci power state
	 * method of resetting doesn't work so we have another way
	 * using the doorbell register.
	 */

	rc = hpsa_lookup_board_id(pdev, &board_id);
	if (rc < 0 || !ctlr_is_resettable(board_id)) {
		dev_warn(&pdev->dev, "Not resetting device.\n");
		return -ENODEV;
	}

	/* if controller is soft- but not hard resettable... */
	if (!ctlr_is_hard_resettable(board_id))
		return -ENOTSUPP; /* try soft reset later. */

	/* Save the PCI command register */
	pci_read_config_word(pdev, 4, &command_register);
	/* Turn the board off.  This is so that later pci_restore_state()
	 * won't turn the board on before the rest of config space is ready.
	 */
	pci_disable_device(pdev);
	pci_save_state(pdev);

	/* find the first memory BAR, so we can find the cfg table */
	rc = hpsa_pci_find_memory_BAR(pdev, &paddr);
	if (rc)
		return rc;
	vaddr = remap_pci_mem(paddr, 0x250);
	if (!vaddr)
		return -ENOMEM;

	/* find cfgtable in order to check if reset via doorbell is supported */
	rc = hpsa_find_cfg_addrs(pdev, vaddr, &cfg_base_addr,
					&cfg_base_addr_index, &cfg_offset);
	if (rc)
		goto unmap_vaddr;
	cfgtable = remap_pci_mem(pci_resource_start(pdev,
		       cfg_base_addr_index) + cfg_offset, sizeof(*cfgtable));
	if (!cfgtable) {
		rc = -ENOMEM;
		goto unmap_vaddr;
	}
	rc = write_driver_ver_to_cfgtable(cfgtable);
	if (rc)
		return rc;

	/* If reset via doorbell register is supported, use that.
	 * There are two such methods.  Favor the newest method.
	 */
	misc_fw_support = readl(&cfgtable->misc_fw_support);
	use_doorbell = misc_fw_support & MISC_FW_DOORBELL_RESET2;
	if (use_doorbell) {
		use_doorbell = DOORBELL_CTLR_RESET2;
	} else {
		use_doorbell = misc_fw_support & MISC_FW_DOORBELL_RESET;
		if (use_doorbell) {
			dev_warn(&pdev->dev, "Controller claims that "
				"'Bit 2 doorbell reset' is "
				"supported, but not 'bit 5 doorbell reset'.  "
				"Firmware update is recommended.\n");
			rc = -ENOTSUPP; /* try soft reset */
			goto unmap_cfgtable;
		}
	}

	rc = hpsa_controller_hard_reset(pdev, vaddr, use_doorbell);
	if (rc)
		goto unmap_cfgtable;

	pci_restore_state(pdev);
	rc = pci_enable_device(pdev);
	if (rc) {
		dev_warn(&pdev->dev, "failed to enable device.\n");
		goto unmap_cfgtable;
	}
	pci_write_config_word(pdev, 4, command_register);

	/* Some devices (notably the HP Smart Array 5i Controller)
	 * need a little pause here.  Not all controllers though,
	 * and for some we will miss the transition to NOT READY
	 * if we wait here.   As a heuristic, if either doorbell
	 * reset method is supported, we'll skip this sleep.
	 */
	if (!use_doorbell)
		msleep(HPSA_POST_RESET_PAUSE_MSECS);

	if (!use_doorbell) {
		/* Wait for board to become not ready, then ready.
		 * (if we used the doorbell, then we already waited 5 secs
		 * so the "not ready" state is already gone by so we
		 * won't catch it.)
		 */
		dev_info(&pdev->dev, "Waiting for board to reset.\n");
		rc = hpsa_wait_for_board_state(pdev, vaddr, BOARD_NOT_READY);
		if (rc) {
			dev_warn(&pdev->dev,
				"failed waiting for board to reset."
				" Will try soft reset.\n");
			/* Not expected, but try soft reset later */
			rc = -ENOTSUPP;
			goto unmap_cfgtable;
		}
	}
	rc = hpsa_wait_for_board_state(pdev, vaddr, BOARD_READY);
	if (rc) {
		dev_warn(&pdev->dev,
			"failed waiting for board to become ready "
			"after hard reset\n");
		goto unmap_cfgtable;
	}

	rc = controller_reset_failed(vaddr);
	if (rc < 0)
		goto unmap_cfgtable;
	if (rc) {
		dev_warn(&pdev->dev, "Unable to successfully reset "
			"controller. Will try soft reset.\n");
		rc = -ENOTSUPP;
	} else {
		dev_info(&pdev->dev, "board ready after hard reset.\n");
	}

unmap_cfgtable:
	iounmap(cfgtable);

unmap_vaddr:
	iounmap(vaddr);
	return rc;
}

/*
 *  We cannot read the structure directly, for portability we must use
 *   the io functions.
 *   This is for debug only.
 */
static void print_cfg_table(struct device *dev, struct CfgTable *tb)
{
#ifdef HPSA_DEBUG
	int i;
	char temp_name[17];

	dev_info(dev, "Controller Configuration information\n");
	dev_info(dev, "------------------------------------\n");
	for (i = 0; i < 4; i++)
		temp_name[i] = readb(&(tb->Signature[i]));
	temp_name[4] = '\0';
	dev_info(dev, "   Signature = %s\n", temp_name);
	dev_info(dev, "   Spec Number = %d\n", readl(&(tb->SpecValence)));
	dev_info(dev, "   Transport methods supported = 0x%x\n",
	       readl(&(tb->TransportSupport)));
	dev_info(dev, "   Transport methods active = 0x%x\n",
	       readl(&(tb->TransportActive)));
	dev_info(dev, "   Requested transport Method = 0x%x\n",
	       readl(&(tb->HostWrite.TransportRequest)));
	dev_info(dev, "   Coalesce Interrupt Delay = 0x%x\n",
	       readl(&(tb->HostWrite.CoalIntDelay)));
	dev_info(dev, "   Coalesce Interrupt Count = 0x%x\n",
	       readl(&(tb->HostWrite.CoalIntCount)));
	dev_info(dev, "   Max outstanding commands = 0x%d\n",
	       readl(&(tb->CmdsOutMax)));
	dev_info(dev, "   Bus Types = 0x%x\n", readl(&(tb->BusTypes)));
	for (i = 0; i < 16; i++)
		temp_name[i] = readb(&(tb->ServerName[i]));
	temp_name[16] = '\0';
	dev_info(dev, "   Server Name = %s\n", temp_name);
	dev_info(dev, "   Heartbeat Counter = 0x%x\n\n\n",
		readl(&(tb->HeartBeat)));
#endif				/* HPSA_DEBUG */
}

static int find_PCI_BAR_index(struct pci_dev *pdev, unsigned long pci_bar_addr)
{
	int i, offset, mem_type, bar_type;

	if (pci_bar_addr == PCI_BASE_ADDRESS_0)	/* looking for BAR zero? */
		return 0;
	offset = 0;
	for (i = 0; i < DEVICE_COUNT_RESOURCE; i++) {
		bar_type = pci_resource_flags(pdev, i) & PCI_BASE_ADDRESS_SPACE;
		if (bar_type == PCI_BASE_ADDRESS_SPACE_IO)
			offset += 4;
		else {
			mem_type = pci_resource_flags(pdev, i) &
			    PCI_BASE_ADDRESS_MEM_TYPE_MASK;
			switch (mem_type) {
			case PCI_BASE_ADDRESS_MEM_TYPE_32:
			case PCI_BASE_ADDRESS_MEM_TYPE_1M:
				offset += 4;	/* 32 bit */
				break;
			case PCI_BASE_ADDRESS_MEM_TYPE_64:
				offset += 8;
				break;
			default:	/* reserved in PCI 2.2 */
				dev_warn(&pdev->dev,
				       "base address is invalid\n");
				return -1;
				break;
			}
		}
		if (offset == pci_bar_addr - PCI_BASE_ADDRESS_0)
			return i + 1;
	}
	return -1;
}

static int calculate_nreply_queues(void)
{
	/* Make sure reply_queues module param is in bounds */
	if (reply_queues < 1)
		return 1;
	if (reply_queues > MAX_REPLY_QUEUES)
		return MAX_REPLY_QUEUES;
	/* Cap reply queues to number of CPUs, more is a waste */
	if (num_online_cpus() < reply_queues)
		return num_online_cpus();
	return reply_queues;
}

/* If MSI/MSI-X is supported by the kernel we will try to enable it on
 * controllers that are capable. If not, we use IO-APIC mode.
 */

static void __devinit hpsa_interrupt_mode(struct ctlr_info *h)
{
#ifdef CONFIG_PCI_MSI
	int err, i;
	struct msix_entry hpsa_msix_entries[MAX_REPLY_QUEUES];

	h->nreply_queues = calculate_nreply_queues();

	for (i = 0; i < MAX_REPLY_QUEUES; i++) {
		hpsa_msix_entries[i].vector = 0;
		hpsa_msix_entries[i].entry = i;
	}

	/* Some boards advertise MSI but don't really support it */
	if ((h->board_id == 0x40700E11) || (h->board_id == 0x40800E11) ||
	    (h->board_id == 0x40820E11) || (h->board_id == 0x40830E11))
		goto default_int_mode;
	if (pci_find_capability(h->pdev, PCI_CAP_ID_MSIX)) {
		dev_info(&h->pdev->dev, "MSIX\n");

		h->msix_vector = MAX_REPLY_QUEUES;
		if (h->msix_vector > num_online_cpus())
			h->msix_vector = num_online_cpus();

		err = pci_enable_msix(h->pdev, hpsa_msix_entries,
					h->nreply_queues);
		if (!err) {
			for (i = 0; i < h->nreply_queues; i++)
				h->intr[i] = hpsa_msix_entries[i].vector;
			h->msix_vector = 1;
			dev_warn(&h->pdev->dev, "Using %d reply queue(s)\n",
					h->nreply_queues);
			return;
		}
		if (err > 0) {
			dev_warn(&h->pdev->dev, "only %d MSI-X vectors "
			       "available\n", err);
			goto default_int_mode;
		} else {
			dev_warn(&h->pdev->dev, "MSI-X init failed %d\n",
			       err);
			goto default_int_mode;
		}
	}
	if (pci_find_capability(h->pdev, PCI_CAP_ID_MSI)) {
		dev_info(&h->pdev->dev, "MSI\n");
		if (!pci_enable_msi(h->pdev))
			h->msi_vector = 1;
		else
			dev_warn(&h->pdev->dev, "MSI init failed\n");
	}
default_int_mode:
	h->nreply_queues = 1;
#endif				/* CONFIG_PCI_MSI */
	/* if we get here we're going to use the default interrupt mode */
	dev_info(&h->pdev->dev, "Using %d reply queue(s)\n", h->nreply_queues);
	h->intr[h->intr_mode] = h->pdev->irq;
}

static int __devinit hpsa_lookup_board_id(struct pci_dev *pdev, u32 *board_id)
{
	int i;
	u32 subsystem_vendor_id, subsystem_device_id;

	subsystem_vendor_id = pdev->subsystem_vendor;
	subsystem_device_id = pdev->subsystem_device;
	*board_id = ((subsystem_device_id << 16) & 0xffff0000) |
		    subsystem_vendor_id;

	for (i = 0; i < ARRAY_SIZE(products); i++)
		if (*board_id == products[i].board_id)
			return i;

	if ((subsystem_vendor_id != PCI_VENDOR_ID_HP &&
		subsystem_vendor_id != PCI_VENDOR_ID_COMPAQ) ||
		!hpsa_allow_any) {
		dev_warn(&pdev->dev, "unrecognized board ID: "
			"0x%08x, ignoring.\n", *board_id);
		return -ENODEV;
	}
	return ARRAY_SIZE(products) - 1; /* generic unknown smart array */
}

static int __devinit hpsa_pci_find_memory_BAR(struct pci_dev *pdev,
	unsigned long *memory_bar)
{
	int i;

	for (i = 0; i < DEVICE_COUNT_RESOURCE; i++)
		if (pci_resource_flags(pdev, i) & IORESOURCE_MEM) {
			/* addressing mode bits already removed */
			*memory_bar = pci_resource_start(pdev, i);
			dev_dbg(&pdev->dev, "memory BAR = %lx\n",
				*memory_bar);
			return 0;
		}
	dev_warn(&pdev->dev, "no memory BAR found\n");
	return -ENODEV;
}

static int hpsa_find_cfg_addrs(struct pci_dev *pdev,
	void __iomem *vaddr, u32 *cfg_base_addr, u64 *cfg_base_addr_index,
	u64 *cfg_offset)
{
	*cfg_base_addr = readl(vaddr + SA5_CTCFG_OFFSET);
	*cfg_offset = readl(vaddr + SA5_CTMEM_OFFSET);
	*cfg_base_addr &= (u32) 0x0000ffff;
	*cfg_base_addr_index = find_PCI_BAR_index(pdev, *cfg_base_addr);
	if (*cfg_base_addr_index == -1) {
		dev_warn(&pdev->dev, "cannot find cfg_base_addr_index\n");
		return -ENODEV;
	}
	return 0;
}

static int __devinit hpsa_find_cfgtables(struct ctlr_info *h)
{
	u64 cfg_offset;
	u32 cfg_base_addr;
	u64 cfg_base_addr_index;
	u32 trans_offset;
	int rc;

	rc = hpsa_find_cfg_addrs(h->pdev, h->vaddr, &cfg_base_addr,
		&cfg_base_addr_index, &cfg_offset);
	if (rc)
		return rc;
	h->cfgtable = remap_pci_mem(pci_resource_start(h->pdev,
		       cfg_base_addr_index) + cfg_offset, sizeof(*h->cfgtable));
	if (!h->cfgtable)
		return -ENOMEM;
	rc = write_driver_ver_to_cfgtable(h->cfgtable);
	if (rc)
		return -ENOMEM;
	/* Find performant mode table. */
	trans_offset = readl(&h->cfgtable->TransMethodOffset);
	h->transtable = remap_pci_mem(pci_resource_start(h->pdev,
				cfg_base_addr_index)+cfg_offset+trans_offset,
				sizeof(*h->transtable));
	if (!h->transtable)
		return -ENOMEM;
	return 0;
}

static void __devinit hpsa_get_max_perf_mode_cmds(struct ctlr_info *h)
{
	h->max_commands = readl(&(h->cfgtable->MaxPerformantModeCommands));

	/* Limit commands in memory limited kdump scenario. */
	if (reset_devices && h->max_commands > 32)
		h->max_commands = 32;

	if (h->max_commands < 16) {
		dev_warn(&h->pdev->dev, "Controller reports "
			"max supported commands of %d, an obvious lie. "
			"Using 16.  Ensure that firmware is up to date.\n",
			h->max_commands);
		h->max_commands = 16;
	}
}

/* Interrogate the hardware for some limits:
 * max commands, max SG elements without chaining, and with chaining,
 * SG chain block size, etc.
 */
static void __devinit hpsa_find_board_params(struct ctlr_info *h)
{
	hpsa_get_max_perf_mode_cmds(h);
	h->nr_cmds = h->max_commands;
	h->maxsgentries = readl(&(h->cfgtable->MaxScatterGatherElements));
	h->fw_support = readl(&(h->cfgtable->misc_fw_support));
	/*
	 * Limit in-command s/g elements to 32 save dma'able memory.
	 * Howvever spec says if 0, use 31
	 */
	h->max_cmd_sg_entries = 31;
	if (h->maxsgentries > 512) {
		h->max_cmd_sg_entries = 32;
		h->chainsize = h->maxsgentries - h->max_cmd_sg_entries + 1;
		h->maxsgentries--; /* save one for chain pointer */
	} else {
		h->maxsgentries = 31; /* default to traditional values */
		h->chainsize = 0;
	}

	/* Find out what task management functions are supported and cache */
	h->TMFSupportFlags = readl(&(h->cfgtable->TMFSupportFlags));
	printk(KERN_WARNING "Physical aborts supported: %s\n",
		(HPSATMF_PHYS_TASK_ABORT & h->TMFSupportFlags) ? "yes" : "no");
	printk(KERN_WARNING "Logical aborts supported: %s\n",
		(HPSATMF_LOG_TASK_ABORT & h->TMFSupportFlags) ? "yes" : "no");
}

static inline int hpsa_CISS_signature_present(struct ctlr_info *h)
{
	if ((readb(&h->cfgtable->Signature[0]) != 'C') ||
	    (readb(&h->cfgtable->Signature[1]) != 'I') ||
	    (readb(&h->cfgtable->Signature[2]) != 'S') ||
	    (readb(&h->cfgtable->Signature[3]) != 'S')) {
		dev_warn(&h->pdev->dev, "not a valid CISS config table\n");
		return 0;
	}
	return 1;
}

/* Need to enable prefetch in the SCSI core for 6400 in x86 */
static inline void hpsa_enable_scsi_prefetch(struct ctlr_info *h)
{
#ifdef CONFIG_X86
	u32 prefetch;

	prefetch = readl(&(h->cfgtable->driver_support));
	prefetch |= ENABLE_SCSI_PREFETCH;
	writel(prefetch, &(h->cfgtable->driver_support));
#endif
}

/* Disable DMA prefetch for the P600.  Otherwise an ASIC bug may result
 * in a prefetch beyond physical memory.
 */
static inline void hpsa_p600_dma_prefetch_quirk(struct ctlr_info *h)
{
	u32 dma_prefetch;

	if (h->board_id != 0x3225103C)
		return;
	dma_prefetch = readl(h->vaddr + I2O_DMA1_CFG);
	dma_prefetch |= 0x8000;
	writel(dma_prefetch, h->vaddr + I2O_DMA1_CFG);
}

static void hpsa_wait_for_clear_event_notify_ack(struct ctlr_info *h)
{
	int i;
	u32 doorbell_value;
	unsigned long flags;
	/* wait until the clear_event_notify bit 6 is cleared by controller. */
	for (i = 0; i < MAX_CONFIG_WAIT; i++) {
		spin_lock_irqsave(&h->lock, flags);
		doorbell_value = readl(h->vaddr + SA5_DOORBELL);
		spin_unlock_irqrestore(&h->lock, flags);
		if (!(doorbell_value & DOORBELL_CLEAR_EVENTS))
			break;
		/* delay and try again */
		msleep(10);
	}
}

static void hpsa_wait_for_mode_change_ack(struct ctlr_info *h)
{
	int i;
	u32 doorbell_value;
	unsigned long flags;

	/* under certain very rare conditions, this can take awhile.
	 * (e.g.: hot replace a failed 144GB drive in a RAID 5 set right
	 * as we enter this code.)
	 */
	for (i = 0; i < MAX_CONFIG_WAIT; i++) {
		spin_lock_irqsave(&h->lock, flags);
		doorbell_value = readl(h->vaddr + SA5_DOORBELL);
		spin_unlock_irqrestore(&h->lock, flags);
		if (!(doorbell_value & CFGTBL_ChangeReq))
			break;
		/* delay and try again */
		msleep(10);
	}
}

static int __devinit hpsa_enter_simple_mode(struct ctlr_info *h)
{
	u32 trans_support;

	trans_support = readl(&(h->cfgtable->TransportSupport));
	if (!(trans_support & SIMPLE_MODE))
		return -ENOTSUPP;

	h->max_commands = readl(&(h->cfgtable->CmdsOutMax));

	/* Update the field, and then ring the doorbell */
	writel(CFGTBL_Trans_Simple, &(h->cfgtable->HostWrite.TransportRequest));
	writel(0, &h->cfgtable->HostWrite.command_pool_addr_hi);
	writel(CFGTBL_ChangeReq, h->vaddr + SA5_DOORBELL);
	hpsa_wait_for_mode_change_ack(h);
	print_cfg_table(&h->pdev->dev, h->cfgtable);
	if (!(readl(&(h->cfgtable->TransportActive)) & CFGTBL_Trans_Simple))
		goto error;
	h->transMethod = CFGTBL_Trans_Simple;
	return 0;
error:
	dev_warn(&h->pdev->dev, "unable to get board into simple mode\n");
	return -ENODEV;
}

static void __devexit hpsa_remove_one(struct pci_dev *pdev);

static int __devinit hpsa_pci_init(struct ctlr_info *h)
{
	int prod_index, err;

	prod_index = hpsa_lookup_board_id(h->pdev, &h->board_id);
	if (prod_index < 0)
		return -ENODEV;

	h->product_name = products[prod_index].product_name;
	h->access = *(products[prod_index].access);

	h->needs_abort_tags_swizzled =
		ctlr_needs_abort_tags_swizzled(h->board_id);

	err = pci_enable_device(h->pdev);
	if (err) {
		dev_warn(&h->pdev->dev, "unable to enable PCI device\n");
		return err;
	}

	/* Enable bus mastering (pci_disable_device may disable this) */
	pci_set_master(h->pdev);

	err = pci_request_regions(h->pdev, HPSA);
	if (err) {
		dev_err(&h->pdev->dev,
			"cannot obtain PCI resources, aborting\n");
		return err;
	}

	hpsa_interrupt_mode(h);

	err = hpsa_pci_find_memory_BAR(h->pdev, &h->paddr);
	if (err)
		goto err_out_free_res;
	h->vaddr = remap_pci_mem(h->paddr, 0x250);
	if (!h->vaddr) {
		err = -ENOMEM;
		goto err_out_free_res;
	}
	err = hpsa_wait_for_board_state(h->pdev, h->vaddr, BOARD_READY);
	if (err)
		goto err_out_free_res;
	err = hpsa_find_cfgtables(h);
	if (err)
		goto err_out_free_res;
	hpsa_find_board_params(h);

	if (!hpsa_CISS_signature_present(h)) {
		err = -ENODEV;
		goto err_out_free_res;
	}
	hpsa_enable_scsi_prefetch(h);
	hpsa_p600_dma_prefetch_quirk(h);
	err = hpsa_enter_simple_mode(h);
	if (err)
		goto err_out_free_res;
	return 0;

err_out_free_res:
	if (h->transtable)
		iounmap(h->transtable);
	if (h->cfgtable)
		iounmap(h->cfgtable);
	if (h->vaddr)
		iounmap(h->vaddr);
	pci_disable_device(h->pdev);
	pci_release_regions(h->pdev);
	return err;
}

static void hpsa_hba_inquiry(struct ctlr_info *h)
{
	int rc;

#define HBA_INQUIRY_BYTE_COUNT 64
	h->hba_inquiry_data = kmalloc(HBA_INQUIRY_BYTE_COUNT, GFP_KERNEL);
	if (!h->hba_inquiry_data)
		return;
	rc = hpsa_scsi_do_inquiry(h, RAID_CTLR_LUNID, 0,
		h->hba_inquiry_data, HBA_INQUIRY_BYTE_COUNT);
	if (rc != 0) {
		kfree(h->hba_inquiry_data);
		h->hba_inquiry_data = NULL;
	}
}

static __devinit int hpsa_init_reset_devices(struct pci_dev *pdev)
{
	int rc, i;

	if (!reset_devices)
		return 0;

	/* Reset the controller with a PCI power-cycle or via doorbell */
	rc = hpsa_kdump_hard_reset_controller(pdev);

	/* -ENOTSUPP here means we cannot reset the controller
	 * but it's already (and still) up and running in
	 * "performant mode".  Or, it might be 640x, which can't reset
	 * due to concerns about shared bbwc between 6402/6404 pair.
	 */
	if (rc == -ENOTSUPP)
		return rc; /* just try to do the kdump anyhow. */
	if (rc)
		return -ENODEV;

	/* Now try to get the controller to respond to a no-op */
	dev_warn(&pdev->dev, "Waiting for controller to respond to no-op\n");
	for (i = 0; i < HPSA_POST_RESET_NOOP_RETRIES; i++) {
		if (hpsa_noop(pdev) == 0)
			break;
		else
			dev_warn(&pdev->dev, "no-op failed%s\n",
					(i < 11 ? "; re-trying" : ""));
	}
	return 0;
}

static __devinit int hpsa_allocate_cmd_pool(struct ctlr_info *h)
{
	h->cmd_pool_bits = kzalloc(
		DIV_ROUND_UP(h->nr_cmds, BITS_PER_LONG) *
		sizeof(unsigned long), GFP_KERNEL);
	h->cmd_pool = pci_alloc_consistent(h->pdev,
		    h->nr_cmds * sizeof(*h->cmd_pool),
		    &(h->cmd_pool_dhandle));
	h->errinfo_pool = pci_alloc_consistent(h->pdev,
		    h->nr_cmds * sizeof(*h->errinfo_pool),
		    &(h->errinfo_pool_dhandle));
	if ((h->cmd_pool_bits == NULL)
	    || (h->cmd_pool == NULL)
	    || (h->errinfo_pool == NULL)) {
		dev_err(&h->pdev->dev, "out of memory in %s", __func__);
		return -ENOMEM;
	}
	return 0;
}

static void hpsa_free_cmd_pool(struct ctlr_info *h)
{
	kfree(h->cmd_pool_bits);
	if (h->cmd_pool)
		pci_free_consistent(h->pdev,
			    h->nr_cmds * sizeof(struct CommandList),
			    h->cmd_pool, h->cmd_pool_dhandle);
	if (h->ioaccel2_cmd_pool)
		pci_free_consistent(h->pdev,
			h->nr_cmds * sizeof(*h->ioaccel2_cmd_pool),
			h->ioaccel2_cmd_pool, h->ioaccel2_cmd_pool_dhandle);
	if (h->errinfo_pool)
		pci_free_consistent(h->pdev,
			    h->nr_cmds * sizeof(struct ErrorInfo),
			    h->errinfo_pool,
			    h->errinfo_pool_dhandle);
	if (h->ioaccel_cmd_pool)
		pci_free_consistent(h->pdev,
			h->nr_cmds * sizeof(struct io_accel1_cmd),
			h->ioaccel_cmd_pool, h->ioaccel_cmd_pool_dhandle);
}

static void hpsa_irq_affinity_hints(struct ctlr_info *h)
{
	int i, cpu, rc;

	cpu = cpumask_first(cpu_online_mask);
	for (i = 0; i < h->msix_vector; i++) {
		rc = irq_set_affinity_hint(h->intr[i], get_cpu_mask(cpu));
		cpu = cpumask_next(cpu, cpu_online_mask);
	}
}

static int hpsa_request_irq(struct ctlr_info *h,
	INTERRUPT_HANDLER_TYPE(msixhandler),
	INTERRUPT_HANDLER_TYPE(intxhandler))
{
	int rc = -1, i;

	/*
	 * initialize h->q[x] = x so that interrupt handlers know which
	 * queue to process.
	 */
	for (i = 0; i < MAX_REPLY_QUEUES; i++)
		h->q[i] = (u8) i;

	if (h->intr_mode == PERF_MODE_INT && h->msix_vector) {
		/* If performant mode and MSI-X, use multiple reply queues */
		for (i = 0; i < h->nreply_queues; i++)
			rc = request_irq(h->intr[i], msixhandler,
					0, h->devname,
					&h->q[i]);
		hpsa_irq_affinity_hints(h);
	} else {
		/* Use single reply pool */
		if (h->msix_vector || h->msi_vector) {
			rc = request_irq(h->intr[h->intr_mode],
				msixhandler, 0, h->devname,
				&h->q[h->intr_mode]);
		} else {
			rc = request_irq(h->intr[h->intr_mode],
				intxhandler, IRQF_SHARED, h->devname,
				&h->q[h->intr_mode]);
		}
	}
	if (rc) {
		dev_err(&h->pdev->dev, "unable to get irq %d for %s\n",
		       h->intr[h->intr_mode], h->devname);
		return -ENODEV;
	}
	return 0;
}

static int hpsa_kdump_soft_reset(struct ctlr_info *h)
{
	if (hpsa_send_host_reset(h, RAID_CTLR_LUNID,
		HPSA_RESET_TYPE_CONTROLLER)) {
		dev_warn(&h->pdev->dev, "Resetting array controller failed.\n");
		return -EIO;
	}

	dev_info(&h->pdev->dev, "Waiting for board to soft reset.\n");
	if (hpsa_wait_for_board_state(h->pdev, h->vaddr, BOARD_NOT_READY)) {
		dev_warn(&h->pdev->dev, "Soft reset had no effect.\n");
		return -1;
	}

	dev_info(&h->pdev->dev, "Board reset, awaiting READY status.\n");
	if (hpsa_wait_for_board_state(h->pdev, h->vaddr, BOARD_READY)) {
		dev_warn(&h->pdev->dev, "Board failed to become ready "
			"after soft reset.\n");
		return -1;
	}

	return 0;
}

static void free_irqs(struct ctlr_info *h)
{
	int i;

	if (!h->msix_vector || h->intr_mode != PERF_MODE_INT) {
		/* Single reply queue, only one irq to free */
		i = h->intr_mode;
		irq_set_affinity_hint(h->intr[i], NULL);
		free_irq(h->intr[i], &h->q[i]);
		return;
	}

	for (i = 0; i < h->nreply_queues; i++) {
		irq_set_affinity_hint(h->intr[i], NULL);
		free_irq(h->intr[i], &h->q[i]);
	}
}

static void hpsa_free_irqs_and_disable_msix(struct ctlr_info *h)
{
	free_irqs(h);
#ifdef CONFIG_PCI_MSI
	if (h->msix_vector) {
		if (h->pdev->msix_enabled) {
			pci_disable_msix(h->pdev);
		}
	} else if (h->msi_vector) {
		if (h->pdev->msi_enabled) {
			pci_disable_msi(h->pdev);
		}
	}
#endif				/* CONFIG_PCI_MSI */
}

static void hpsa_free_reply_queues(struct ctlr_info *h)
{
	int i;

	for (i = 0; i < h->nreply_queues; i++) {
		if (!h->reply_queue[i].head)
			continue;
		pci_free_consistent(h->pdev, h->reply_queue_size,
			h->reply_queue[i].head, h->reply_queue[i].busaddr);
		h->reply_queue[i].head = NULL;
		h->reply_queue[i].busaddr = 0;
	}
}

static void hpsa_undo_allocations_after_kdump_soft_reset(struct ctlr_info *h)
{
	hpsa_free_irqs_and_disable_msix(h);
	hpsa_free_sg_chain_blocks(h);
	hpsa_free_ioaccel2_sg_chain_blocks(h);
	hpsa_free_cmd_pool(h);
	kfree(h->ioaccel1_blockFetchTable);
	kfree(h->blockFetchTable);
	hpsa_free_reply_queues(h);
	if (h->vaddr)
		iounmap(h->vaddr);
	if (h->transtable)
		iounmap(h->transtable);
	if (h->cfgtable)
		iounmap(h->cfgtable);
	pci_release_regions(h->pdev);
	kfree(h);
}

/* Called when controller lockup detected. */
static void fail_all_outstanding_cmds(struct ctlr_info *h)
{
	int i, refcount;
	struct CommandList *c;

	flush_workqueue(h->resubmit_wq); /* ensure all cmds are fully built */
	for (i = 0; i < h->nr_cmds; i++) {
		c = h->cmd_pool + i;
		refcount = atomic_inc_return(&c->refcount);
		if (refcount > 1) {
			c->err_info->CommandStatus = CMD_HARDWARE_ERR;
			finish_cmd(c);
			atomic_dec(&h->commands_outstanding);
		}
		cmd_free(h, c);
	}
}

static void set_lockup_detected_for_all_cpus(struct ctlr_info *h, u32 value)
{
	int i, cpu;

	cpu = cpumask_first(cpu_online_mask);
	for (i = 0; i < num_online_cpus(); i++) {
		u32 *lockup_detected;
		lockup_detected = per_cpu_ptr(h->lockup_detected, cpu);
		*lockup_detected = value;
		cpu = cpumask_next(cpu, cpu_online_mask);
	}
	wmb(); /* be sure the per-cpu variables are out to memory */
}

static void controller_lockup_detected(struct ctlr_info *h)
{
	unsigned long flags;
	u32 lockup_detected;

	h->access.set_intr_mask(h, HPSA_INTR_OFF);
	spin_lock_irqsave(&h->lock, flags);
	lockup_detected = readl(h->vaddr + SA5_SCRATCHPAD_OFFSET);
	if (!lockup_detected) {
		/* no heartbeat, but controller gave us a zero. */
		dev_warn(&h->pdev->dev,
			"lockup detected but scratchpad register is zero\n");
		lockup_detected = 0xffffffff;
	}
	set_lockup_detected_for_all_cpus(h, lockup_detected);
	spin_unlock_irqrestore(&h->lock, flags);
	dev_warn(&h->pdev->dev, "Controller lockup detected: 0x%08x\n",
			lockup_detected);
	pci_disable_device(h->pdev);
	fail_all_outstanding_cmds(h);
}

static void detect_controller_lockup(struct ctlr_info *h)
{
	u64 now;
	u32 heartbeat;
	unsigned long flags;

	if (!h->lockup_detector_enabled)
		return;

	now = get_jiffies_64();
	/* If we've received an interrupt recently, we're ok. */
	if (time_after64(h->last_intr_timestamp +
				(h->heartbeat_sample_interval), now))
		return;

	/*
	 * If we've already checked the heartbeat recently, we're ok.
	 * This could happen if someone sends us a signal. We
	 * otherwise don't care about signals in this thread.
	 */
	if (time_after64(h->last_heartbeat_timestamp +
				(h->heartbeat_sample_interval), now))
		return;

	/* If heartbeat has not changed since we last looked, we're not ok. */
	spin_lock_irqsave(&h->lock, flags);
	heartbeat = readl(&h->cfgtable->HeartBeat);
	spin_unlock_irqrestore(&h->lock, flags);
	if (h->last_heartbeat == heartbeat) {
		controller_lockup_detected(h);
		return;
	}

	/* We're ok. */
	h->last_heartbeat = heartbeat;
	h->last_heartbeat_timestamp = now;
}

static void hpsa_ack_ctlr_events(struct ctlr_info *h)
{
	int i;
	char *event_type;

	/* Ask the controller to clear the events we're handling. */
	if ((h->transMethod & (CFGTBL_Trans_io_accel1
			| CFGTBL_Trans_io_accel2)) &&
		(h->events & HPSA_EVENT_NOTIFY_ACCEL_IO_PATH_STATE_CHANGE ||
		 h->events & HPSA_EVENT_NOTIFY_ACCEL_IO_PATH_CONFIG_CHANGE)) {

		if (h->events & HPSA_EVENT_NOTIFY_ACCEL_IO_PATH_STATE_CHANGE )
			event_type = "state change";
		if (h->events & HPSA_EVENT_NOTIFY_ACCEL_IO_PATH_CONFIG_CHANGE )
			event_type = "configuration change";
		/* Stop sending new RAID offload reqs via the IO accelerator */
		scsi_block_requests(h->scsi_host);
		for (i = 0; i < h->ndevices; i++)
			h->dev[i]->offload_enabled = 0; 
		hpsa_drain_accel_commands(h);
		/* Set 'accelerator path config change' bit in clear_event_notify field */
		dev_warn(&h->pdev->dev,
			"Acknowledging event: 0x%08x (HP SSD Smart Path %s)\n",
			h->events, event_type);
		writel(h->events, &(h->cfgtable->clear_event_notify));
		/* Set the "clear event notify field update" bit 6 */
		writel(DOORBELL_CLEAR_EVENTS, h->vaddr + SA5_DOORBELL);
		/* Wait until ctlr clears 'clear event notify field', bit 6 */
		hpsa_wait_for_clear_event_notify_ack(h);
		scsi_unblock_requests(h->scsi_host);
	} else {
		/* Acknowledge controller notification events. */
		writel(h->events, &(h->cfgtable->clear_event_notify));
		writel(DOORBELL_CLEAR_EVENTS, h->vaddr + SA5_DOORBELL);
		hpsa_wait_for_clear_event_notify_ack(h);
#if 0
		writel(CFGTBL_ChangeReq, h->vaddr + SA5_DOORBELL);
		hpsa_wait_for_mode_change_ack(h);
#endif
	}
	return;
}

/* Check a register on the controller to see if there are configuration
 * changes (added/changed/removed logical drives, etc.) which mean that
 * we should rescan the controller for devices.  
 * Also check flag for driver-initiated rescan. 
 */
static int hpsa_ctlr_needs_rescan(struct ctlr_info *h)
{
	if (h->drv_req_rescan) {
		h->drv_req_rescan = 0;
		return 1;
	}

	if (!(h->fw_support & MISC_FW_EVENT_NOTIFY))
		return 0;

	h->events = readl(&(h->cfgtable->event_notify));
	return h->events & RESCAN_REQUIRED_EVENT_BITS;
}

/*
 * Check if any of the offline devices have become ready
 */
static int hpsa_offline_devices_ready(struct ctlr_info *h)
{
	unsigned long flags;
	struct offline_device_entry *d;
	struct list_head *this, *tmp;

	spin_lock_irqsave(&h->offline_device_lock, flags);
	list_for_each_safe(this, tmp, &h->offline_device_list) {
		d = list_entry(this, struct offline_device_entry,
				offline_list);
		spin_unlock_irqrestore(&h->offline_device_lock, flags);
		if (!hpsa_volume_offline(h, d->scsi3addr)) {
			spin_lock_irqsave(&h->offline_device_lock, flags);
			list_del(&d->offline_list);
			spin_unlock_irqrestore(&h->offline_device_lock, flags);
			return 1;
		}
		spin_lock_irqsave(&h->offline_device_lock, flags);
	}
	spin_unlock_irqrestore(&h->offline_device_lock, flags);
	return 0;
}

static void hpsa_rescan_ctlr_worker(struct work_struct *work)
{
	unsigned long flags;
	struct ctlr_info *h = container_of(to_delayed_work(work),
					struct ctlr_info, rescan_ctlr_work);

	if (hpsa_ctlr_needs_rescan(h) || hpsa_offline_devices_ready(h)) {
		scsi_host_get(h->scsi_host);
		hpsa_ack_ctlr_events(h);
		hpsa_scan_start(h->scsi_host);
		scsi_host_put(h->scsi_host);
	}
	spin_lock_irqsave(&h->lock, flags);
	if (h->remove_in_progress)
		cancel_delayed_work(&h->rescan_ctlr_work);
	else
		schedule_delayed_work(&h->rescan_ctlr_work,
				h->heartbeat_sample_interval);
	spin_unlock_irqrestore(&h->lock, flags);
}

static void hpsa_monitor_ctlr_worker(struct work_struct *work)
{
	unsigned long flags;
	struct ctlr_info *h = container_of(to_delayed_work(work),
					struct ctlr_info, monitor_ctlr_work);
	detect_controller_lockup(h);
	if (lockup_detected(h))
		return;

	spin_lock_irqsave(&h->lock, flags);
	if (h->remove_in_progress) {
		spin_unlock_irqrestore(&h->lock, flags);
		return;
	}
	schedule_delayed_work(&h->monitor_ctlr_work,
				h->heartbeat_sample_interval);
	spin_unlock_irqrestore(&h->lock, flags);
}

static int __devinit hpsa_init_one(struct pci_dev *pdev,
				    const struct pci_device_id *ent)
{
	int dac, rc;
	struct ctlr_info *h;
	int try_soft_reset = 0;
	unsigned long flags;

	if (number_of_controllers == 0)
		printk(KERN_INFO DRIVER_NAME "\n");

	rc = hpsa_init_reset_devices(pdev);
	if (rc) {
		if (rc != -ENOTSUPP)
			return rc;
		/* If the reset fails in a particular way (it has no way to do
		 * a proper hard reset, so returns -ENOTSUPP) we can try to do
		 * a soft reset once we get the controller configured up to the
		 * point that it can accept a command.
		 */
		try_soft_reset = 1;
		rc = 0;
	}

reinit_after_soft_reset:

	/* Command structures must be aligned on a 32-byte boundary because
	 * the 5 lower bits of the address are used by the hardware. and by
	 * the driver.  See comments in hpsa.h for more info.
	 */
	BUILD_BUG_ON(sizeof(struct CommandList) % COMMANDLIST_ALIGNMENT);
	h = kzalloc(sizeof(*h), GFP_KERNEL);
	if (!h)
		return -ENOMEM;

	h->pdev = pdev;
	h->intr_mode = hpsa_simple_mode ? SIMPLE_MODE_INT : PERF_MODE_INT;
	INIT_LIST_HEAD(&h->offline_device_list);
	spin_lock_init(&h->lock);
	spin_lock_init(&h->scan_lock);
	spin_lock_init(&h->offline_device_lock);
	atomic_set(&h->passthru_cmds_avail, HPSA_MAX_CONCURRENT_PASSTHRUS);
	atomic_set(&h->abort_cmds_available, HPSA_CMDS_RESERVED_FOR_ABORTS);

	h->resubmit_wq = alloc_workqueue("hpsa", WQ_MEM_RECLAIM, 0);
	if (!h->resubmit_wq) {
		dev_warn(&h->pdev->dev, "Failed to allocate work queue\n");
		goto clean1;
	}
	/* Allocate and clear per-cpu variable lockup_detected */
	h->lockup_detected = alloc_percpu(u32);
	if (!h->lockup_detected) {
		rc = -ENOMEM;
		goto clean1;
	}
	set_lockup_detected_for_all_cpus(h, 0);

	rc = hpsa_pci_init(h);
	if (rc != 0)
		goto clean1;

	sprintf(h->devname, HPSA "%d", number_of_controllers);
	h->ctlr = number_of_controllers;
	number_of_controllers++;

	/* configure PCI DMA stuff */
	rc = pci_set_dma_mask(pdev, DMA_BIT_MASK(64));
	if (rc == 0) {
		dac = 1;
	} else {
		rc = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
		if (rc == 0) {
			dac = 0;
		} else {
			dev_err(&pdev->dev, "no suitable DMA available\n");
			goto clean1;
		}
	}

	/* make sure the board interrupts are off */
	h->access.set_intr_mask(h, HPSA_INTR_OFF);

	if (hpsa_request_irq(h, do_hpsa_intr_msi, do_hpsa_intr_intx))
		goto clean2;
	dev_info(&pdev->dev, "%s: <0x%x> at IRQ %d%s using DAC\n",
	       h->devname, pdev->device,
	       h->intr[h->intr_mode], dac ? "" : " not");
	if (hpsa_allocate_cmd_pool(h))
		goto clean4;
	if (hpsa_allocate_sg_chain_blocks(h))
		goto clean4;
	init_waitqueue_head(&h->scan_wait_queue);
	init_waitqueue_head(&h->abort_cmd_wait_queue);
	h->scan_finished = 1; /* no scan currently in progress */

	pci_set_drvdata(pdev, h);
	h->ndevices = 0;
	h->hba_mode_enabled = 0;
	h->scsi_host = NULL;
	spin_lock_init(&h->devlock);
	hpsa_put_ctlr_into_performant_mode(h);

	/* At this point, the controller is ready to take commands.
	 * Now, if reset_devices and the hard reset didn't work, try
	 * the soft reset and see if that works.
	 */
	if (try_soft_reset) {

		/* This is kind of gross.  We may or may not get a completion
		 * from the soft reset command, and if we do, then the value
		 * from the fifo may or may not be valid.  So, we wait 10 secs
		 * after the reset throwing away any completions we get during
		 * that time.  Unregister the interrupt handler and register
		 * fake ones to scoop up any residual completions.
		 */
		spin_lock_irqsave(&h->lock, flags);
		h->access.set_intr_mask(h, HPSA_INTR_OFF);
		cancel_delayed_work(&h->rescan_ctlr_work);
		spin_unlock_irqrestore(&h->lock, flags);
		free_irqs(h);
		rc = hpsa_request_irq(h, hpsa_msix_discard_completions,
					hpsa_intx_discard_completions);
		if (rc) {
			dev_warn(&h->pdev->dev, "Failed to request_irq after "
				"soft reset.\n");
			goto clean4;
		}

		rc = hpsa_kdump_soft_reset(h);
		if (rc)
			/* Neither hard nor soft reset worked, we're hosed. */
			goto clean4;

		dev_info(&h->pdev->dev, "Board READY.\n");
		dev_info(&h->pdev->dev,
			"Waiting for stale completions to drain.\n");
		h->access.set_intr_mask(h, HPSA_INTR_ON);
		msleep(10000);
		h->access.set_intr_mask(h, HPSA_INTR_OFF);

		rc = controller_reset_failed(h->cfgtable);
		if (rc)
			dev_info(&h->pdev->dev,
				"Soft reset appears to have failed.\n");

		/* since the controller's reset, we have to go back and re-init
		 * everything.  Easiest to just forget what we've done and do it
		 * all over again.
		 */
		hpsa_undo_allocations_after_kdump_soft_reset(h);
		try_soft_reset = 0;
		if (rc)
			/* don't go to clean4, we already unallocated */
			return -ENODEV;

		goto reinit_after_soft_reset;
	}

		/* Enable Accelerated IO path at driver layer */
		h->acciopath_status = 1;

	h->lockup_detector_enabled = 1;

	/* Turn the interrupts on so we can service requests */
	h->access.set_intr_mask(h, HPSA_INTR_ON);

	hpsa_hba_inquiry(h);
	if (hpsa_register_scsi(h)) /* hook ourselves into SCSI subsystem */
		goto clean4;

	/* Monitor the controller for firmware lockups */
	h->heartbeat_sample_interval = HEARTBEAT_SAMPLE_INTERVAL;
	INIT_DELAYED_WORK(&h->monitor_ctlr_work, hpsa_monitor_ctlr_worker);
	schedule_delayed_work(&h->monitor_ctlr_work,
			h->heartbeat_sample_interval);
	INIT_DELAYED_WORK(&h->rescan_ctlr_work, hpsa_rescan_ctlr_worker);
	schedule_delayed_work(&h->rescan_ctlr_work,
			h->heartbeat_sample_interval);
	return 0;

clean4:
	hpsa_free_sg_chain_blocks(h);
	hpsa_free_cmd_pool(h);
	free_irqs(h);
clean2:
clean1:
	if (h->resubmit_wq)
		destroy_workqueue(h->resubmit_wq);
	if (h->lockup_detected)
		free_percpu(h->lockup_detected);
	kfree(h);
	return rc;
}
EXPORT_SYMBOL(hpsa_init_one);

static void hpsa_flush_cache(struct ctlr_info *h)
{
	char *flush_buf;
	struct CommandList *c;
	int rc;

	/* Don't bother trying to flush the cache if locked up */
	if (unlikely(lockup_detected(h)))
		return;
	flush_buf = kzalloc(4, GFP_KERNEL);
	if (!flush_buf)
		return;

	c = cmd_alloc(h);
	if (!c) {
		dev_warn(&h->pdev->dev, "cmd_alloc returned NULL!\n");
		goto out_of_memory;
	}
	if (fill_cmd(c, HPSA_CACHE_FLUSH, h, flush_buf, 4, 0,
		RAID_CTLR_LUNID, TYPE_CMD)) {
		goto out;
	}
	rc = hpsa_scsi_do_simple_cmd_with_retry(h, c,
					PCI_DMA_TODEVICE, NO_TIMEOUT);
	if (rc)
		goto out;
	if (c->err_info->CommandStatus != 0)
out:
		dev_warn(&h->pdev->dev,
			"error flushing cache on controller\n");
	cmd_free(h, c);
out_of_memory:
	kfree(flush_buf);
}

static void hpsa_shutdown(struct pci_dev *pdev)
{
	struct ctlr_info *h;

	h = pci_get_drvdata(pdev);
	/* Turn board interrupts off  and send the flush cache command
	 * sendcmd will turn off interrupt, and send the flush...
	 * To write all data in the battery backed cache to disks
	 */
	hpsa_flush_cache(h);
	h->access.set_intr_mask(h, HPSA_INTR_OFF);
	hpsa_free_irqs_and_disable_msix(h);
}

static void __devexit hpsa_free_device_info(struct ctlr_info *h)
{
	int i;

	for (i = 0; i < h->ndevices; i++)
		kfree(h->dev[i]);
}

static void __devexit hpsa_remove_one(struct pci_dev *pdev)
{
	struct ctlr_info *h;
	unsigned long flags;

	if (pci_get_drvdata(pdev) == NULL) {
		dev_err(&pdev->dev, "unable to remove device\n");
		return;
	}
	h = pci_get_drvdata(pdev);

	/* Get rid of any controller monitoring work items */
	spin_lock_irqsave(&h->lock, flags);
	h->remove_in_progress = 1;
	cancel_delayed_work(&h->monitor_ctlr_work);
	cancel_delayed_work(&h->rescan_ctlr_work);
	spin_unlock_irqrestore(&h->lock, flags);
	hpsa_unregister_scsi(h);	/* unhook from SCSI subsystem */
	hpsa_shutdown(pdev);
	destroy_workqueue(h->resubmit_wq);
	iounmap(h->vaddr);
	iounmap(h->transtable);
	iounmap(h->cfgtable);
	hpsa_free_device_info(h);
	hpsa_free_sg_chain_blocks(h);
	hpsa_free_ioaccel2_sg_chain_blocks(h);
	hpsa_free_cmd_pool(h);
	if (h->intr_mode != SIMPLE_MODE_INT)
	hpsa_free_reply_queues(h);
	kfree(h->blockFetchTable);
	kfree(h->ioaccel1_blockFetchTable);
	kfree(h->ioaccel2_blockFetchTable);
	kfree(h->hba_inquiry_data);
	pci_release_regions(pdev);
	pci_disable_device(h->pdev);
	pci_set_drvdata(pdev, NULL);
	free_percpu(h->lockup_detected);
	kfree(h);
}

static int hpsa_suspend(__attribute__((unused)) struct pci_dev *pdev,
	__attribute__((unused)) pm_message_t state)
{
	return -ENOSYS;
}

static int hpsa_resume(__attribute__((unused)) struct pci_dev *pdev)
{
	return -ENOSYS;
}

static struct pci_driver hpsa_pci_driver = {
	.name = HPSA,
	.probe = hpsa_init_one,
	.remove = __devexit_p(hpsa_remove_one),
	.id_table = hpsa_pci_device_id,	/* id_table */
	.shutdown = hpsa_shutdown,
	.suspend = hpsa_suspend,
	.resume = hpsa_resume,
};

/* Fill in bucket_map[], given nsgs (the max number of
 * scatter gather elements supported) and bucket[],
 * which is an array of 8 integers.  The bucket[] array
 * contains 8 different DMA transfer sizes (in 16
 * byte increments) which the controller uses to fetch
 * commands.  This function fills in bucket_map[], which
 * maps a given number of scatter gather elements to one of
 * the 8 DMA transfer sizes.  The point of it is to allow the
 * controller to only do as much DMA as needed to fetch the
 * command, with the DMA transfer size encoded in the lower
 * bits of the command address.
 */
static void  calc_bucket_map(int bucket[], int num_buckets,
	int nsgs, int min_blocks, int *bucket_map)
{
	int i, j, b, size;

	/* Note, bucket_map must have nsgs+1 entries. */
	for (i = 0; i <= nsgs; i++) {
		/* Compute size of a command with i SG entries */
		size = i + min_blocks;
		b = num_buckets; /* Assume the biggest bucket */
		/* Find the bucket that is just big enough */
		for (j = 0; j < num_buckets; j++) {
			if (bucket[j] >= size) {
				b = j;
				break;
			}
		}
		/* for a command with i SG entries, use bucket b. */
		bucket_map[i] = b;
	}
}

static void hpsa_enter_performant_mode(struct ctlr_info *h,
	u32 trans_support)
{
	int i;
	unsigned long register_value;
	unsigned long transMethod = CFGTBL_Trans_Performant |
			(trans_support & CFGTBL_Trans_use_short_tags) |
				CFGTBL_Trans_enable_directed_msix |
			(trans_support & (CFGTBL_Trans_io_accel1 |
				CFGTBL_Trans_io_accel2));
	struct access_method access = SA5_performant_access;

	/* This is a bit complicated.  There are 8 registers on
	 * the controller which we write to to tell it 8 different
	 * sizes of commands which there may be.  It's a way of
	 * reducing the DMA done to fetch each command.  Encoded into
	 * each command's tag are 3 bits which communicate to the controller
	 * which of the eight sizes that command fits within.  The size of
	 * each command depends on how many scatter gather entries there are.
	 * Each SG entry requires 16 bytes.  The eight registers are programmed
	 * with the number of 16-byte blocks a command of that size requires.
	 * The smallest command possible requires 5 such 16 byte blocks.
	 * the largest command possible requires SG_ENTRIES_IN_CMD + 4 16-byte
	 * blocks.  Note, this only extends to the SG entries contained
	 * within the command block, and does not extend to chained blocks
	 * of SG elements.   bft[] contains the eight values we write to
	 * the registers.  They are not evenly distributed, but have more
	 * sizes for small commands, and fewer sizes for larger commands.
	 */
	int bft[8] = {5, 6, 8, 10, 12, 20, 28, SG_ENTRIES_IN_CMD + 4};
#define MIN_IOACCEL2_BFT_ENTRY 5
#define HPSA_IOACCEL2_HEADER_SZ 4
	int bft2[16] = {MIN_IOACCEL2_BFT_ENTRY, 6, 7, 8, 9, 10, 11, 12, 
			13, 14, 15, 16, 17, 18, 19, 
			HPSA_IOACCEL2_HEADER_SZ + IOACCEL2_MAXSGENTRIES};
	BUILD_BUG_ON(ARRAY_SIZE(bft2) != 16);
	BUILD_BUG_ON(ARRAY_SIZE(bft) != 8);
	BUILD_BUG_ON(offsetof(struct io_accel2_cmd, sg) >
				 16 * MIN_IOACCEL2_BFT_ENTRY);
	BUILD_BUG_ON(sizeof(struct ioaccel2_sg_element) != 16);
	BUILD_BUG_ON(28 > SG_ENTRIES_IN_CMD + 4);
	/*  5 = 1 s/g entry or 4k
	 *  6 = 2 s/g entry or 8k
	 *  8 = 4 s/g entry or 16k
	 * 10 = 6 s/g entry or 24k
	 */

	/* If the controller supports either ioaccel method then
	 * we can also use the RAID stack submit path that does not
	 * the superfluous readl() after each command submission.
	 */
	if (trans_support & (CFGTBL_Trans_io_accel1 | CFGTBL_Trans_io_accel2))
		access = SA5_performant_access_no_read;

	/* Controller spec: zero out this buffer. */
	for (i = 0; i < h->nreply_queues; i++)
		memset(h->reply_queue[i].head, 0, h->reply_queue_size);

	bft[7] = SG_ENTRIES_IN_CMD + 4;
	calc_bucket_map(bft, ARRAY_SIZE(bft),
				SG_ENTRIES_IN_CMD, 4, h->blockFetchTable);
	for (i = 0; i < 8; i++)
		writel(bft[i], &h->transtable->BlockFetch[i]);

	/* size of controller ring buffer */
	writel(h->max_commands, &h->transtable->RepQSize);
	writel(h->nreply_queues, &h->transtable->RepQCount);
	writel(0, &h->transtable->RepQCtrAddrLow32);
	writel(0, &h->transtable->RepQCtrAddrHigh32);

	for (i = 0; i < h->nreply_queues; i++) {
		writel(0, &h->transtable->RepQAddr[i].upper);
		writel(h->reply_queue[i].busaddr,
			&h->transtable->RepQAddr[i].lower);
	}

	writel(0, &h->cfgtable->HostWrite.command_pool_addr_hi);
	writel(transMethod, &(h->cfgtable->HostWrite.TransportRequest));
	/*
	 * enable outbound interrupt coalescing in accelerator mode;
	 */
	if (trans_support & CFGTBL_Trans_io_accel1) {
		access = SA5_ioaccel_mode1_access;
		writel(10, &h->cfgtable->HostWrite.CoalIntDelay);
		writel(4, &h->cfgtable->HostWrite.CoalIntCount);
	} else {
		if (trans_support & CFGTBL_Trans_io_accel2) {
			access = SA5_ioaccel_mode2_access;
			writel(10, &h->cfgtable->HostWrite.CoalIntDelay);
			writel(4, &h->cfgtable->HostWrite.CoalIntCount);
		}
	}
	writel(CFGTBL_ChangeReq, h->vaddr + SA5_DOORBELL);
	hpsa_wait_for_mode_change_ack(h);
	register_value = readl(&(h->cfgtable->TransportActive));
	if (!(register_value & CFGTBL_Trans_Performant)) {
		dev_warn(&h->pdev->dev, "unable to get board into"
					" performant mode\n");
		return;
	}
	/* Change the access methods to the performant access methods */
	h->access = access;
	h->transMethod = transMethod;

	if (!((trans_support & CFGTBL_Trans_io_accel1) ||
		(trans_support & CFGTBL_Trans_io_accel2)))
		return;

	if (trans_support & CFGTBL_Trans_io_accel1) {
		/* Set up I/O accelerator mode */
		for (i = 0; i < h->nreply_queues; i++) {
			writel(i, h->vaddr + IOACCEL_MODE1_REPLY_QUEUE_INDEX);
			h->reply_queue[i].current_entry =
				readl(h->vaddr + IOACCEL_MODE1_PRODUCER_INDEX);
		}
		bft[7] = h->ioaccel_maxsg + 8;
		calc_bucket_map(bft, ARRAY_SIZE(bft), h->ioaccel_maxsg, 8,
				h->ioaccel1_blockFetchTable);

		/* initialize all reply queue entries to unused */
		for (i = 0; i < h->nreply_queues; i++)
			memset(h->reply_queue[i].head,
				(u8) IOACCEL_MODE1_REPLY_UNUSED,
				h->reply_queue_size);

		/* set all the constant fields in the accelerator command
		 * frames once at init time to save CPU cycles later.
		 */
		for (i = 0; i < h->nr_cmds; i++) {
			struct io_accel1_cmd *cp = &h->ioaccel_cmd_pool[i];

			cp->function = IOACCEL1_FUNCTION_SCSIIO;
			cp->err_info = (u32) (h->errinfo_pool_dhandle +
					(i * sizeof(struct ErrorInfo)));
			cp->err_info_len = sizeof(struct ErrorInfo);
			cp->sgl_offset = IOACCEL1_SGLOFFSET;
			cp->host_context_flags = IOACCEL1_HCFLAGS_CISS_FORMAT;
			cp->timeout_sec = 0;
			cp->ReplyQueue = 0;
			cp->tag =
				cpu_to_le64((u64) (i << DIRECT_LOOKUP_SHIFT));
			cp->host_addr =
				cpu_to_le64((u64) (h->ioaccel_cmd_pool_dhandle +
					(i * sizeof(struct io_accel1_cmd))));
		}
	} else if (trans_support & CFGTBL_Trans_io_accel2) {
		u64 cfg_offset, cfg_base_addr_index;
		u32 bft2_offset, cfg_base_addr;
		int rc;

		rc = hpsa_find_cfg_addrs(h->pdev, h->vaddr, &cfg_base_addr,
			&cfg_base_addr_index, &cfg_offset);
		BUILD_BUG_ON(offsetof(struct io_accel2_cmd, sg) != 64);
		bft2[15] = h->ioaccel_maxsg + HPSA_IOACCEL2_HEADER_SZ;
		calc_bucket_map(bft2, ARRAY_SIZE(bft2), h->ioaccel_maxsg,
				4, h->ioaccel2_blockFetchTable);
		bft2_offset = readl(&h->cfgtable->io_accel_request_size_offset);
		BUILD_BUG_ON(offsetof(struct CfgTable, io_accel_request_size_offset)
			!= 0xb8); 
		printk(KERN_WARNING "bft2_offset = 0x%x\n", bft2_offset);
		h->ioaccel2_bft2_regs =
			remap_pci_mem(pci_resource_start(h->pdev, cfg_base_addr_index) + 
					cfg_offset +
					bft2_offset, ARRAY_SIZE(bft2) * sizeof(*h->ioaccel2_bft2_regs));
		for (i = 0; i < ARRAY_SIZE(bft2); i++)
			writel(bft2[i], &h->ioaccel2_bft2_regs[i]);
	}
	writel(CFGTBL_ChangeReq, h->vaddr + SA5_DOORBELL);
	hpsa_wait_for_mode_change_ack(h);
}

static int hpsa_alloc_ioaccel_cmd_and_bft(struct ctlr_info *h)
{
	h->ioaccel_maxsg =
		readl(&(h->cfgtable->io_accel_max_embedded_sg_count));
	if (h->ioaccel_maxsg > IOACCEL1_MAXSGENTRIES)
		h->ioaccel_maxsg = IOACCEL1_MAXSGENTRIES;

	/* Command structures must be aligned on a 128-byte boundary
	 * because the 7 lower bits of the address are used by the
	 * hardware.
	 */
	BUILD_BUG_ON(sizeof(struct io_accel1_cmd) %
			IOACCEL1_COMMANDLIST_ALIGNMENT);

	h->ioaccel_cmd_pool =
		pci_alloc_consistent(h->pdev,
			h->nr_cmds * sizeof(*h->ioaccel_cmd_pool),
			&(h->ioaccel_cmd_pool_dhandle));

	h->ioaccel1_blockFetchTable =
		kmalloc(((h->ioaccel_maxsg + 1) *
				sizeof(u32)), GFP_KERNEL);

	if ((h->ioaccel_cmd_pool == NULL) ||
		(h->ioaccel1_blockFetchTable == NULL))
		goto clean_up;

	memset(h->ioaccel_cmd_pool, 0,
		h->nr_cmds * sizeof(*h->ioaccel_cmd_pool));
	return 0;

clean_up:
	if (h->ioaccel_cmd_pool)
		pci_free_consistent(h->pdev,
			h->nr_cmds * sizeof(*h->ioaccel_cmd_pool),
			h->ioaccel_cmd_pool, h->ioaccel_cmd_pool_dhandle);
	kfree(h->ioaccel1_blockFetchTable);
	return 1;
}

static int ioaccel2_alloc_cmds_and_bft(struct ctlr_info *h)
{
	/* Allocate ioaccel2 mode command blocks and block fetch table */

	h->ioaccel_maxsg =
		readl(&(h->cfgtable->io_accel_max_embedded_sg_count));
	if (h->ioaccel_maxsg > IOACCEL2_MAXSGENTRIES)
		h->ioaccel_maxsg = IOACCEL2_MAXSGENTRIES;

	BUILD_BUG_ON(sizeof(struct io_accel2_cmd) %
			IOACCEL2_COMMANDLIST_ALIGNMENT);
	h->ioaccel2_cmd_pool =
		pci_alloc_consistent(h->pdev,
			h->nr_cmds * sizeof(*h->ioaccel2_cmd_pool),
			&(h->ioaccel2_cmd_pool_dhandle));

	h->ioaccel2_blockFetchTable =
		kmalloc(((h->ioaccel_maxsg + 1) *
				sizeof(u32)), GFP_KERNEL);

	if ((h->ioaccel2_cmd_pool == NULL) ||
		(h->ioaccel2_blockFetchTable == NULL))
		goto clean_up;

	if (hpsa_allocate_ioaccel2_sg_chain_blocks(h))
		goto clean_up;

	memset(h->ioaccel2_cmd_pool, 0,
		h->nr_cmds * sizeof(*h->ioaccel2_cmd_pool));
	return 0;

clean_up:
	if (h->ioaccel2_cmd_pool)
		pci_free_consistent(h->pdev,
			h->nr_cmds * sizeof(*h->ioaccel2_cmd_pool),
			h->ioaccel2_cmd_pool, h->ioaccel2_cmd_pool_dhandle);
	kfree(h->ioaccel2_blockFetchTable);
	return 1;
}

static __devinit void hpsa_put_ctlr_into_performant_mode(struct ctlr_info *h)
{
	u32 trans_support;
	unsigned long transMethod = CFGTBL_Trans_Performant |
					CFGTBL_Trans_use_short_tags;
	int i;

	if (h->intr_mode == SIMPLE_MODE_INT)
		return;

	trans_support = readl(&(h->cfgtable->TransportSupport));
	if (!(trans_support & PERFORMANT_MODE))
		return;

	/* Check for I/O accelerator mode support */
	if (trans_support & CFGTBL_Trans_io_accel1) {
		transMethod |= CFGTBL_Trans_io_accel1 |
				CFGTBL_Trans_enable_directed_msix;
		if (hpsa_alloc_ioaccel_cmd_and_bft(h))
			goto clean_up;
	} else {
		if (trans_support & CFGTBL_Trans_io_accel2) {
				transMethod |= CFGTBL_Trans_io_accel2 |
				CFGTBL_Trans_enable_directed_msix;
		if (ioaccel2_alloc_cmds_and_bft(h))
			goto clean_up;
		}
	}

	hpsa_get_max_perf_mode_cmds(h);
	/* Performant mode ring buffer and supporting data structures */
	h->reply_queue_size = h->max_commands * sizeof(u64);

	for (i = 0; i < h->nreply_queues; i++) {
		h->reply_queue[i].head = pci_alloc_consistent(h->pdev,
						h->reply_queue_size,
						&(h->reply_queue[i].busaddr));
		if (!h->reply_queue[i].head)
			goto clean_up;
		h->reply_queue[i].size = h->max_commands;
		h->reply_queue[i].wraparound = 1;  /* spec: init to 1 */
		h->reply_queue[i].current_entry = 0;
	}

	/* Need a block fetch table for performant mode */
	h->blockFetchTable = kmalloc(((SG_ENTRIES_IN_CMD + 1) *
				sizeof(u32)), GFP_KERNEL);
	if (!h->blockFetchTable)
		goto clean_up;

	hpsa_enter_performant_mode(h, trans_support);
	return;

clean_up:
	hpsa_free_reply_queues(h);
	kfree(h->blockFetchTable);
}

static int is_accelerated_cmd(struct CommandList *c)
{
	return (c->cmd_type == CMD_IOACCEL1 ||
		c->cmd_type == CMD_IOACCEL2);
}

static void hpsa_drain_accel_commands(struct ctlr_info *h)
{
	struct CommandList *c = NULL;
	int i, accel_cmds_out;
	int refcount;

	do { /* wait for all outstanding ioaccel commands to drain out */
		accel_cmds_out = 0;
		for (i = 0; i < h->nr_cmds; i++) {
			c = h->cmd_pool + i;
			refcount = atomic_inc_return(&c->refcount);
			if (refcount > 1) /* Command is allocated */
				accel_cmds_out += is_accelerated_cmd(c);
			cmd_free(h, c);
		}
		if (accel_cmds_out <= 0)
			break;
		msleep(100);
	} while (1);
}

/*
 *  This is it.  Register the PCI driver information for the cards we control
 *  the OS will call our registered routines when it finds one of our cards.
 */
static int __init hpsa_init(void)
{
	return pci_register_driver(&hpsa_pci_driver);
}

static void __exit hpsa_cleanup(void)
{
	pci_unregister_driver(&hpsa_pci_driver);
}

module_init(hpsa_init);
module_exit(hpsa_cleanup);
