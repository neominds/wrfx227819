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
 * Linux device driver for
 * Broadcom BCM47XX 10/100/1000 Mbps Ethernet Controller
 *
 * $Id: et_linux.c 327582 2012-04-14 05:02:37Z $
 */

#include <et_cfg.h>
#define __UNDEF_NO_VERSION__

#include <typedefs.h>

#include <linux/module.h>
#include <linuxver.h>
#include <bcmdefs.h>
#include <osl.h>

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/sockios.h>
#ifdef SIOCETHTOOL
#include <linux/ethtool.h>
#endif /* SIOCETHTOOL */
#include <linux/ip.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/syscalls.h>

#include <mach/iproc_regs.h>

#include <asm/system.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/pgtable.h>
#include <asm/uaccess.h>

#include <epivers.h>
#include <bcmendian.h>
#include <bcmdefs.h>
#include <proto/ethernet.h>
#include <proto/vlan.h>
#include <proto/bcmip.h>
#include <bcmdevs.h>
#include <bcmenetmib.h>
#include <bcmgmacmib.h>
#include <bcmenetrxh.h>
#include <bcmenetphy.h>
#include <etioctl.h>
#include <bcmutils.h>
#include <pcicfg.h>
#include <et_dbg.h>
#include <hndsoc.h>
#include <bcmgmacrxh.h>
#include <etc.h>
#ifdef HNDCTF
#include <ctf/hndctf.h>
#endif /* HNDCTF */
#ifdef GMAC3
#include <hndfwd.h>	/* GMAC3 */
#endif /* GMAC3 */

/* to be cleaned and fixed */
/* to be cleaned Makefile */
#include <bcmnvram.h>
#include <siutils.h>
#include <hndcpu.h>
#include <sbchipc.h>
#include <hndchipc.h>
#include <trxhdr.h>
#include "plat/shm.h"

#if defined(CONFIG_IPROC_FA2)
#include "../../../fa2/fa2_defs.h"
#include "../../../fa2/fa2_if.h"
#endif /* CONFIG_IPROC_FA2 */

#ifdef CONFIG_BCM_IPROC_GMAC_PREFETCH
#include <linux/prefetch.h>

#define	SKB_PREFETCH_LEN (128)

/* 30 rxhdr + 34 mac & ip */
#define	SKB_DATA_PREFETCH_LEN (96)

#endif

#define NS_MAX_GMAC_CORES	4

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 36)
#define HAVE_NET_DEVICE_OPS	1
#define HAVE_NETDEV_PRIV	1
#endif

int gmac_pdev_loaded[NS_MAX_GMAC_CORES];

/* Global SB handle */
si_t *bcm947xx_sih = NULL;
spinlock_t bcm947xx_sih_lock;
EXPORT_SYMBOL(bcm947xx_sih);
EXPORT_SYMBOL(bcm947xx_sih_lock);

/* Convenience */
#define sih bcm947xx_sih
#define sih_lock bcm947xx_sih_lock

#ifdef ET_ALL_PASSIVE_ON
/* When ET_ALL_PASSIVE_ON, ET_ALL_PASSIVE must be true */
#define ET_ALL_PASSIVE_ENAB(et)	1
#else
#ifdef ET_ALL_PASSIVE
#define ET_ALL_PASSIVE_ENAB(et)	(!(et)->all_dispatch_mode)
#else /* ET_ALL_PASSIVE */
#define ET_ALL_PASSIVE_ENAB(et)	0
#endif /* ET_ALL_PASSIVE */
#endif	/* ET_ALL_PASSIVE_ON */

#ifdef ET_ALL_PASSIVE
#define ET_LIMIT_TXQ
#endif

#ifdef PKTC
#ifndef HNDCTF
#error "Packet chaining feature can't work w/o CTF"
#endif
#define PKTC_ENAB(et)	((et)->etc->pktc)

#ifdef GMAC3
#define PKT_CHAINABLE(et, p, evh, prio, h_sa, h_da, h_prio) \
	(!eacmp((h_da), ((struct ethervlan_header *)(evh))->ether_dhost) && \
	 !eacmp((h_sa), ((struct ethervlan_header *)(evh))->ether_shost) && \
	 ((h_prio) == (prio)) && !RXH_FLAGS((et)->etc, PKTDATA((et)->osh, (p))) && \
	 ((((struct ether_header *)(evh))->ether_type == HTON16(ETHER_TYPE_IP)) || \
	  (((struct ether_header *)(evh))->ether_type == HTON16(ETHER_TYPE_IPV6))))
#else	/* !GMAC3 */
#define PKT_CHAINABLE(et, p, evh, h_sa, h_da, h_prio) \
	((et)->brc_hot && !RXH_FLAGS((et)->etc, PKTDATA((et)->osh, (p))) && \
	 (((struct ethervlan_header *)(evh))->vlan_type == HTON16(ETHER_TYPE_8021Q)) && \
	 (((struct ethervlan_header *)(evh))->ether_type == HTON16(ETHER_TYPE_IP)) && \
	 ((h_prio) == (IPV4_TOS((evh) + ETHERVLAN_HDR_LEN) >> IPV4_TOS_PREC_SHIFT)) && \
	 !eacmp((h_da), ((struct ethervlan_header *)(evh))->ether_dhost) && \
	 !eacmp((h_sa), ((struct ethervlan_header *)(evh))->ether_shost) && \
	 CTF_HOTBRC_CMP((et)->brc_hot, (evh)))
#endif	/* !GMAC3 */

#define PKTCMC	2
struct pktc_data {
	void	*chead;		/* chain head */
	void	*ctail;		/* chain tail */
	uint8	*h_da;		/* pointer to da of chain head */
	uint8	*h_sa;		/* pointer to sa of chain head */
	uint8	h_prio;		/* prio of chain head */
};
typedef struct pktc_data pktc_data_t;
#else /* PKTC */
#define PKTC_ENAB(et)	0
#define PKT_CHAINABLE(et, p, evh, h_sa, h_da, h_prio) 	0
#endif /* PKTC */

static char bcm5301x_gmac0_string[] = "bcmiproc-gmac0";
static char bcm5301x_gmac1_string[] = "bcmiproc-gmac1";
static char bcm5301x_gmac2_string[] = "bcmiproc-gmac2";
static char bcm5301x_gmac3_string[] = "bcmiproc-gmac3";

#ifdef GMAC_RATE_LIMITING
static int et_rx_rate_limit = 0;
extern void etc_check_rate_limiting(etc_info_t *etc, void *pch);
#endif /* GMAC_RATE_LIMITING */
extern int nvram_env_gmac_name(int gmac, char *name);

#if defined(CONFIG_IPROC_FA)
extern int fc_receive(struct sk_buff *skb_p);
extern int fc_transmit(struct sk_buff *skb_p);
#else
#define fc_receive(arg)		{}
#define fc_transmit(arg)	{}
#endif /* defined(CONFIG_IPROC_FA) */

#if defined(CONFIG_IPROC_FA2)
extern int fa2_receive(struct sk_buff *skb_p);
extern int fa2_transmit(struct sk_buff *skb_p, struct fa2_pkt_info *pkt_info);
extern int fa2_get_packet_info(struct sk_buff *skb, struct fa2_pkt_info *info);

#else
#define fa2_receive(arg)		{}
#define fa2_transmit(arg)	{}
#endif /* defined(CONFIG_IPROC_FA2) */

/* In 2.6.20 kernels work functions get passed a pointer to the
 * struct work, so things will continue to work as long as the work
 * structure is the first component of the task structure.
 */
typedef struct et_task {
	struct work_struct work;
	void *context;
} et_task_t;

typedef struct et_info {
	etc_info_t	*etc;		/* pointer to common os-independent data */
	struct net_device *dev;		/* backpoint to device */
	struct pci_dev *pdev;		/* backpoint to pci_dev */
	void		*osh;		/* pointer to os handle */
#ifdef GMAC3
	struct fwder *fwdh;		/* pointer to my upstream forwarder handle */
#endif	/* GMAC3 */
#ifdef HNDCTF
	ctf_t		*cih;		/* ctf instance handle */
	ctf_brc_hot_t	*brc_hot;	/* hot bridge cache entry */
#endif
	struct semaphore sem;		/* use semaphore to allow sleep */
	spinlock_t	lock;		/* per-device perimeter lock */
	spinlock_t	txq_lock;	/* lock for txq protection */
	spinlock_t	isr_lock;	/* lock for irq reentrancy protection */
	struct sk_buff_head txq[NUMTXQ];	/* send queue */
	void *regsva;			/* opaque chip registers virtual address */
	struct timer_list timer;	/* one second watchdog timer */
	bool set;                     /* indicate the timer is set or not */
	struct net_device_stats stats;	/* stat counter reporting structure */
	int events;			/* bit channel between isr and dpc */
	struct et_info *next;		/* pointer to next et_info_t in chain */
#ifdef	NAPI2_POLL
	struct napi_struct napi_poll;
#endif	/* NAPI2_POLL */
#ifndef NAPI_POLL
	struct tasklet_struct tasklet;	/* dpc tasklet */
#endif /* NAPI_POLL */
#ifdef ET_ALL_PASSIVE
	et_task_t	dpc_task;	/* work queue for rx dpc */
	et_task_t	txq_task;	/* work queue for tx frames */
	bool		all_dispatch_mode;	/* dispatch mode: tasklets or passive */
#endif /* ET_ALL_PASSIVE */
	bool resched;			/* dpc was rescheduled */
} et_info_t;

static int et_found = 0;
static et_info_t *et_list = NULL;

/* defines */
#define	DATAHIWAT	1000		/* data msg txq hiwat mark */

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 37)
#define init_MUTEX(x) sema_init(x,1)
#endif


#ifndef	HAVE_NETDEV_PRIV
#define	HAVE_NETDEV_PRIV
#define	netdev_priv(dev)	((dev)->priv)
#define	ET_INFO(dev)	(et_info_t *)((dev)->priv)
#else
#define	ET_INFO(dev)	netdev_priv(dev)
#endif	/* HAVE_NETDEV_PRIV */


#define ET_LOCK(et) \
do { \
	if (ET_ALL_PASSIVE_ENAB(et)) \
		down(&(et)->sem); \
	else \
		spin_lock_bh(&(et)->lock); \
} while (0)

#define ET_UNLOCK(et) \
do { \
	if (ET_ALL_PASSIVE_ENAB(et)) \
		up(&(et)->sem); \
	else \
		spin_unlock_bh(&(et)->lock); \
} while (0)

#define ET_TXQ_LOCK(et)		spin_lock_bh(&(et)->txq_lock)
#define ET_TXQ_UNLOCK(et)	spin_unlock_bh(&(et)->txq_lock)

#define INT_LOCK(et, flags)	spin_lock_irqsave(&(et)->isr_lock, flags)
#define INT_UNLOCK(et, flags)	spin_unlock_irqrestore(&(et)->isr_lock, flags)

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2, 4, 5)
#error Linux version must be newer than 2.4.5
#endif	/* LINUX_VERSION_CODE <= KERNEL_VERSION(2, 4, 5) */

/* linux 2.4 doesn't have in_atomic */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)
#define in_atomic() 0
#endif

/* prototypes called by etc.c */
void et_init(et_info_t *et, uint options);
void et_reset(et_info_t *et);
void et_link_up(et_info_t *et);
void et_link_down(et_info_t *et);
void et_up(et_info_t *et);
void et_down(et_info_t *et, int reset);
void et_dump(et_info_t *et, struct bcmstrbuf *b);
#ifdef HNDCTF
void et_dump_ctf(et_info_t *et, struct bcmstrbuf *b);
#endif

/* local prototypes */
static void et_free(et_info_t *et);
static int et_open(struct net_device *dev);
static int et_close(struct net_device *dev);
static int et_start(struct sk_buff *skb, struct net_device *dev);
static void et_sendnext(et_info_t *et);
static struct net_device_stats *et_get_stats(struct net_device *dev);
static int et_set_mac_address(struct net_device *dev, void *addr);
static void et_set_multicast_list(struct net_device *dev);
static void _et_watchdog(struct net_device *data);
static void et_watchdog(ulong data);
#ifdef ET_ALL_PASSIVE
static void et_watchdog_task(et_task_t *task);
#endif /* ET_ALL_PASSIVE */
static int et_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 20)
static irqreturn_t et_isr(int irq, void *dev_id);
#else
static irqreturn_t et_isr(int irq, void *dev_id, struct pt_regs *ptregs);
#endif
#ifdef	NAPI2_POLL
static int et_poll(struct napi_struct *napi, int budget);
#elif defined(NAPI_POLL)
static int et_poll(struct net_device *dev, int *budget);
#else /* ! NAPI_POLL */
static void et_dpc(ulong data);
#endif /* NAPI_POLL */
#ifdef ET_ALL_PASSIVE
static void et_dpc_work(struct et_task *task);
static void et_txq_work(struct et_task *task);
static int et_schedule_task(et_info_t *et, void (*fn)(struct et_task *task), void *context);
#endif /* ET_ALL_PASSIVE */
static void et_sendup(et_info_t *et, struct sk_buff *skb);
static void et_dumpet(et_info_t *et, struct bcmstrbuf *b);

static int __init bcm5301x_gmac_probe(struct platform_device*);
static int __exit bcm5301x_gmac_remove(struct platform_device*);
static int __init bcm5301x_gmac_init_module(void);
static void __exit bcm5301x_gmac_cleanup_module(void);
#ifdef CONFIG_PM
static int bcm5301x_gmac_drv_suspend(struct platform_device *pdev, pm_message_t state);
static int bcm5301x_gmac_drv_resume(struct platform_device *pdev);
#endif
static void bcm5301x_gmac_release (struct device *dev);
#if 0 //dgb  /* Functions related to PROC file system support */
static int get_debug_level(char *page, char **start, off_t off, int count, int *eof, void *data);
static int set_debug_level(struct file *file, const char *buffer, unsigned long count, void *data);
#define MIN_DEBUG_LEVEL 0
#define MAX_DEBUG_LEVEL 3
static int cur_dbg_lvl = MIN_DEBUG_LEVEL;

#endif 
static int eth_mac_proc_create(struct net_device *dev );
static void eth_mac_proc_remove(void);
#if (defined(CONFIG_IPROC_FA2) && defined(CONFIG_IPROC_FA2_CS_OFFLOAD))
static et_info_t *et_get_eth3_info(void);
#endif

#define DISABLE_FA_BYPASS 0
#define ENABLE_FA_BYPASS 1
static unsigned int gBypass = DISABLE_FA_BYPASS;

#ifdef HAVE_NET_DEVICE_OPS
static const struct net_device_ops et_netdev_ops = {
	.ndo_open = et_open,
	.ndo_stop = et_close,
	.ndo_start_xmit = et_start,
	.ndo_get_stats = et_get_stats,
	.ndo_set_mac_address = et_set_mac_address,
	.ndo_set_rx_mode = et_set_multicast_list,
	.ndo_do_ioctl = et_ioctl,
};
#endif /*HAVE_NET_DEVICE_OPS*/

static struct resource bcm5301x_gmac0_resources[] = {
	[0] = {
		.flags  = IORESOURCE_IRQ,
		.start  = IPROC_GMAC0_INT,
	},
	[1] = {
		.flags  = IORESOURCE_MEM,
		.start  = IPROC_GMAC0_REG_BASE,
		.end    = IPROC_GMAC0_REG_BASE+0xbff,
	},
};
static struct resource bcm5301x_gmac1_resources[] = {
	[0] = {
		.flags  = IORESOURCE_IRQ,
		.start  = IPROC_GMAC1_INT,
	},
	[1] = {
		.flags  = IORESOURCE_MEM,
		.start  = IPROC_GMAC1_REG_BASE,
		.end    = IPROC_GMAC1_REG_BASE+0xbff,
	},
};
static struct resource bcm5301x_gmac2_resources[] = {
	[0] = {
		.flags  = IORESOURCE_IRQ,
		.start  = IPROC_GMAC2_INT,
	},
	[1] = {
		.flags  = IORESOURCE_MEM,
		.start  = IPROC_GMAC2_REG_BASE,
		.end    = IPROC_GMAC2_REG_BASE+0xbff,
	},
};
static struct resource bcm5301x_gmac3_resources[] = {
	[0] = {
		.flags  = IORESOURCE_IRQ,
		.start  = IPROC_GMAC3_INT,
	},
	[1] = {
		.flags  = IORESOURCE_MEM,
		.start  = IPROC_GMAC3_REG_BASE,
		.end    = IPROC_GMAC3_REG_BASE+0xbff,
	},
};

#if defined(BCMDBG)
static uint32 msglevel = 0xdeadbeef;
module_param(msglevel, uint, 0644);
#endif /* defined(BCMDBG) */

#if (defined(CONFIG_IPROC_FA) || defined(CONFIG_IPROC_FA2))
static bool brcm_tag=true;
#endif /* (defined(CONFIG_IPROC_FA) || defined(CONFIG_IPROC_FA2)) */

#ifdef ET_ALL_PASSIVE
/* passive mode: 1: enable, 0: disable */
static int passivemode = 0;
module_param(passivemode, int, 0);
#endif /* ET_ALL_PASSIVE */
#ifdef ET_LIMIT_TXQ
#define ET_TXQ_THRESH	0
static int et_txq_thresh = ET_TXQ_THRESH;
module_param(et_txq_thresh, int, 0);
#endif /* ET_LIMIT_TXQ */

static bool
et_ctf_active(et_info_t *et)
{
	bool retval=false;
#if defined(CONFIG_IPROC_FA)
	if (brcm_tag == true) {
		if (et->etc->unit == 2) {
			retval = true;
		}
	}
#elif defined(CONFIG_MACH_NSP)
	if (et->etc->unit == 2 || et->etc->unit == 3) {
		retval = true;
	}
#endif /* defined(CONFIG_IPROC_FA) */

	return retval;
}

static bool
et_ctf_pipeline_loopback(et_info_t *et)
{
	if (et->etc->unit == 3) {
		return true;
	} else {
		return false;
	}
}

void *et_get_hndl(uint unit)
{
	et_info_t *listptr;
    void *roboptr;

    roboptr = NULL;

	for (listptr = et_list; listptr; listptr = listptr->next) {
		if (listptr->etc->unit == unit) {
            roboptr = listptr->etc->robo;
			break;
		}
    }

    return roboptr;
}

static void
et_free(et_info_t *et)
{
	et_info_t **prev;
	osl_t *osh;

	if (et == NULL)
		return;

	ET_TRACE(("et: et_free\n"));

	if (et->dev && et->dev->irq)
		free_irq(et->dev->irq, et);

#ifdef	NAPI2_POLL
	napi_disable(&et->napi_poll);
	netif_napi_del(&et->napi_poll);
#endif	/* NAPI2_POLL */

#ifdef HNDCTF
	if (et->cih)
		ctf_dev_unregister(et->cih, et->dev);
#endif /* HNDCTF */

	if (et->dev) {
		unregister_netdev(et->dev);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 0)
		free_netdev(et->dev);
#else
		MFREE(et->osh, et->dev, sizeof(struct net_device));
#endif
		et->dev = NULL;
	}

#ifdef CTFPOOL
	/* free the buffers in fast pool */
	osl_ctfpool_cleanup(et->osh);
#endif /* CTFPOOL */

#ifdef HNDCTF
	/* free ctf resources */
	if (et->cih)
		ctf_detach(et->cih);
#endif /* HNDCTF */

	/* free common resources */
	if (et->etc) {
		etc_detach(et->etc);
		et->etc = NULL;
	}

	/*
	 * unregister_netdev() calls get_stats() which may read chip registers
	 * so we cannot unmap the chip registers until after calling unregister_netdev() .
	 */
	if (et->regsva) {
		iounmap((void *)et->regsva);
		et->regsva = NULL;
	}

	/* remove us from the global linked list */
	for (prev = &et_list; *prev; prev = &(*prev)->next)
		if (*prev == et) {
			*prev = et->next;
			break;
		}

	osh = et->osh;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36) 
	free_netdev(et->dev); 
	et->dev = NULL; 
#else 
	MFREE(et->osh, et, sizeof(et_info_t));
#endif

	if (MALLOCED(osh)) {
		ET_ERROR(("Memory leak of bytes %d\n", MALLOCED(osh)));
	}
	ASSERT(MALLOCED(osh) == 0);

	osl_detach(osh);
}

static int
et_open(struct net_device *dev)
{
	et_info_t *et;

	et = ET_INFO(dev);

	ET_TRACE(("et%d: et_open\n", et->etc->unit));

	et->etc->promisc = (dev->flags & IFF_PROMISC)? TRUE: FALSE;
	et->etc->allmulti = (dev->flags & IFF_ALLMULTI)? TRUE: et->etc->promisc;
#ifdef GMAC_RATE_LIMITING
	et->etc->rl_enabled = et_rx_rate_limit;
#endif /* GMAC_RATE_LIMITING */

	ET_LOCK(et);
	et_up(et);
	ET_UNLOCK(et);

	OLD_MOD_INC_USE_COUNT;

	return (0);
}

static int
et_close(struct net_device *dev)
{
	et_info_t *et;

	et = ET_INFO(dev);

	ET_TRACE(("et%d: et_close\n", et->etc->unit));

	et->etc->promisc = FALSE;
	et->etc->allmulti = FALSE;

	ET_LOCK(et);
	et_down(et, 1);
	ET_UNLOCK(et);

	OLD_MOD_DEC_USE_COUNT;

	return (0);
}

#ifdef ET_ALL_PASSIVE
/* Schedule a completion handler to run at safe time */
static int
et_schedule_task(et_info_t *et, void (*fn)(struct et_task *task), void *context)
{
	et_task_t *task;

	ET_TRACE(("et%d: et_schedule_task\n", et->etc->unit));

	if (!(task = MALLOC(et->osh, sizeof(et_task_t)))) {
		ET_ERROR(("et%d: et_schedule_task: out of memory, malloced %d bytes\n",
		          et->etc->unit, MALLOCED(et->osh)));
		return -ENOMEM;
	}

	MY_INIT_WORK(&task->work, (work_func_t)fn);
	task->context = context;

	if (!schedule_work(&task->work)) {
		ET_ERROR(("et%d: schedule_work() failed\n", et->etc->unit));
		MFREE(et->osh, task, sizeof(et_task_t));
		return -ENOMEM;
	}

	return 0;
}

static void BCMFASTPATH
et_txq_work(struct et_task *task)
{
	et_info_t *et = (et_info_t *)task->context;

#ifndef CONFIG_BCM_IPROC_GMAC_LOCK_OPT
	ET_LOCK(et);
#endif /* !CONFIG_BCM_IPROC_GMAC_LOCK_OPT */ 
	et_sendnext(et);
#ifndef CONFIG_BCM_IPROC_GMAC_LOCK_OPT
	ET_UNLOCK(et);
#endif /* !CONFIG_BCM_IPROC_GMAC_LOCK_OPT */ 
	return;
}
#endif /* ET_ALL_PASSIVE */

#ifdef GMAC3
/* et_start counterpart (test performance of using a queue) */
static int BCMFASTPATH
et_forward(struct sk_buff *skb, struct net_device *dev, int cnt)
{
	et_info_t *et;
	etc_info_t *etc;
	void *p, *n;
#ifdef PRINT_PKT
	int i;
#endif /* PRINT_PKT */

	et = ET_INFO(dev);
	etc = et->etc;

	/* BUZZZ_DPL1(ET_FORWARD, 3, etc->unit, (uint32)skb, cnt); */
	ET_TRACE(("et%d: et_forward\n", etc->unit));
	ET_LOG("et%d: et_forward", etc->unit, 0);

	ET_PRHDR("tx", (struct ether_header *)skb->data, skb->len, etc->unit);
	ET_PRPKT("txpkt", skb->data, skb->len, etc->unit);

	//p = PKTFRMFORWARD(etc->osh, skb, cnt);
	p = PKTFRMNATIVE(etc->osh, skb);
	ASSERT(p != NULL);

	ET_TRACE(("%s: sdu %p chained %d chain sz %d next %p\n",
                  __FUNCTION__, p, PKTISCHAINED(p), PKTCCNT(p), PKTCLINK(p)));
#ifdef PRINT_PKT
	printk("et%d: %s len(0x%x) fwdpkt:", etc->unit, __FUNCTION__, skb->len);
	for (i=0; i<skb->len; i++) {
		if ( (i % 16) == 0 )
			printk("\n");
		printk("%02x ", skb->data[i]);
	}
	printk("\n");
#endif /* PRINT_PKT */

	/* ---------------------------------------- */
	/* ---------------------------------------- */

	FOREACH_CHAINED_PKT(p, n) {

		PKTCLRCHAINED(et->osh, p);
		if (n == NULL)
			PKTCSETFLAG(p, 1);
		(*etc->chops->tx)(etc->ch, p);

		etc->txframe++;
		etc->txbyte += PKTLEN(et->osh, p);
	}
	/* BUZZZ_DPL2(ET_FORWARD_RTN, 0); */

	return FWDER_SUCCESS;
}
#endif /* GMAC3 */

/*
 * Yeah, queueing the packets on a tx queue instead of throwing them
 * directly into the descriptor ring in the case of dma is kinda lame,
 * but this results in a unified transmit path for both dma and pio
 * and localizes/simplifies the netif_*_queue semantics, too.
 */
static int BCMFASTPATH
et_start(struct sk_buff *skb, struct net_device *dev)
{
	et_info_t *et;
	uint32 q = 0;
#ifdef ET_LIMIT_TXQ
	int qlen;
#endif /* ET_LIMIT_TXQ */

	et = ET_INFO(dev);

#if defined(CONFIG_MACH_NS)
	if (ET_GMAC(et->etc) && (et->etc->qos))
		q = etc_up2tc(PKTPRIO(skb));
#endif /* defined(CONFIG_MACH_NS) */

	ET_TRACE(("et%d: et_start: len %d\n", et->etc->unit, skb->len));
	ET_LOG("et%d: et_start: len %d", et->etc->unit, skb->len);

#ifdef ET_LIMIT_TXQ
#ifndef CONFIG_BCM_IPROC_GMAC_LOCK_OPT
	ET_TXQ_LOCK(et);
#endif /* CONFIG_BCM_IPROC_GMAC_LOCK_OPT */
	qlen = skb_queue_len(&et->txq[q]);
#ifndef CONFIG_BCM_IPROC_GMAC_LOCK_OPT
	ET_TXQ_UNLOCK(et);
#endif /* CONFIG_BCM_IPROC_GMAC_LOCK_OPT */
	et->etc->txfrm++;
	if (qlen > et->etc->txqlen)
		et->etc->txqlen = qlen;
	if (et_txq_thresh && (qlen >= et_txq_thresh)) {
		//PKTCFREE(et->osh, skb, TRUE);
		//return 0;
		et->etc->txfrmdropped++;
		/* schedule work */
#ifdef ET_ALL_PASSIVE
		if (ET_ALL_PASSIVE_ENAB(et)) {
#ifdef CONFIG_BCM_IPROC_GMAC_TXONCPU1
			schedule_work_on(1, &et->txq_task.work);
#else
			schedule_work(&et->txq_task.work);
#endif
		}
#endif /* ET_ALL_PASSIVE */
		return NETDEV_TX_BUSY;
	}
#endif /* ET_LIMIT_TXQ */

#if (defined(CONFIG_IPROC_FA) || defined(CONFIG_IPROC_FA2) || defined(CONFIG_MACH_NSP))

    if (et_ctf_pipeline_loopback(et)) {
        int bcm_hdr_size = 8;  /* type 3 */

		/* add brcm tag; tag is locate at offset 0-3 */
		ET_TRACE(("et%d %s: headroom(0x%x)\n", et->etc->unit, __FUNCTION__, skb_headroom(skb)));
		ET_TRACE(("et%d: NOT enough headroom for BRCM tag.\n", et->etc->unit));
		if (skb_headroom(skb) < bcm_hdr_size) {
			struct sk_buff *sk_tmp = skb;
			skb = skb_realloc_headroom(sk_tmp, bcm_hdr_size);
			PKTCFREE(et->osh, sk_tmp, TRUE);
			if (!skb) {
				ET_ERROR(("et%d: Failed to realloc headroom for BRCM tag; NOT transmitting frame.\n", et->etc->unit));
				return 0;
			}
		}

		ET_TRACE(("Adding BRCM TAG\n"));
		__skb_push(skb, bcm_hdr_size);

			/* insert egress hdr type 3*/
			skb->data[0] = 0x60; /* opcode b011 */
			skb->data[1] = 0x00;
			skb->data[2] = 0x00;
			skb->data[3] = 0x00;
			skb->data[4] = 0x00;
			skb->data[5] = 0x00;
			skb->data[6] = 0x00;
			skb->data[7] = 0x28; /* fwd to AXI1, proc by SPU */

        if (skb->len < 68) {
            ET_TRACE(("forcing skb->len (0x%x) to 68\n", skb->len));
            skb->len = 68;
        }
        __skb_trim(skb, skb->len);

    } else if (et_ctf_active(et)) {
		int bcm_hdr_size = 4;

#if defined(CONFIG_IPROC_FA2_CS_OFFLOAD)
		bcm_hdr_size = 8;
#endif 
		/* add brcm tag; tag is located at offset 0-3 */
		ET_TRACE(("et%d %s: headroom(0x%x)\n", et->etc->unit, __FUNCTION__, skb_headroom(skb)));
		if (skb_headroom(skb) < bcm_hdr_size) {
			struct sk_buff *sk_tmp = skb;

			ET_TRACE(("et%d: NOT enough headroom for BRCM tag.\n", et->etc->unit));
			skb = skb_realloc_headroom(sk_tmp, bcm_hdr_size);
			PKTCFREE(et->osh, sk_tmp, TRUE);
			if (!skb) {
				ET_ERROR(("et%d: Failed to realloc headroom for BRCM tag; NOT transmitting frame.\n", et->etc->unit));
				return 0;
			}
		}

		ET_TRACE(("Adding BRCM TAG\n"));
		__skb_push(skb, 4);

			/* insert ingress hdr type 0*/
			skb->data[0] = 0x00;
			skb->data[1] = 0x00;
			skb->data[2] = 0x00;
			skb->data[3] = 0x00;

#if (defined(CONFIG_IPROC_FA2) && defined(CONFIG_IPROC_FA2_CS_OFFLOAD) && \
	defined(CONFIG_IPROC_FA2_CS_OFFLOAD_SMALL_PKT_WA))
		if (skb->len <= 68) {

				if (skb->ip_summed == CHECKSUM_PARTIAL) {
					int ret;

					ret = skb_checksum_help(skb);

					if (ret) {
						printk(KERN_DEBUG "\n%s: csum_help returned error %d\n",
								__func__, ret);
					 }

			}
		}
#endif
		if (skb->len < 68) {
			ET_TRACE(("forcing skb->len (0x%x) to 68\n", skb->len));
			skb->len = 68;
		}
		__skb_trim(skb, skb->len);

#if defined(CONFIG_IPROC_FA)
        if(!gBypass) {
		    fc_transmit(skb);
        }
#endif /* defined(CONFIG_IPROC_FA) */
#if defined(CONFIG_IPROC_FA2)
        if(!gBypass) {
#if defined(CONFIG_IPROC_FA2_CS_OFFLOAD)
			struct fa2_pkt_info pkt_info;
			extern spinlock_t fa2_lock;
			uint8_t p_op;

			//memset((void *)&pkt_info, 0x0, sizeof(pkt_info));
            /* Initialize pkt_info */
			pkt_info.mac = pkt_info.ipv4_or_ipv6 = pkt_info.tcp_or_udp = NULL;

			pkt_info.vlan_tag = pkt_info.vlan_tag_next = pkt_info.et_type = 
			pkt_info.eth_snapllc = pkt_info.need_hdr_bytes = 
			pkt_info.hdr_words[0] = pkt_info.hdr_words[1] = 0;

			pkt_info.proto = FA2_PROTO_NOT_SUPPORTED;
			pkt_info.pkt_type = FA2_INVALID_PKT;

			spin_lock(&fa2_lock);
			fa2_get_packet_info(skb, &pkt_info);
			spin_unlock(&fa2_lock);

            /* Check if this packet can be processed by FA+ pipeline.
			 * If not, let the eth driver handle it.
			 * If yes, do fa+ processing
			 */
			if (pkt_info.proto != FA2_PROTO_NOT_SUPPORTED) {
				if (pkt_info.pkt_type != FA2_FWD_PKT) {

					p_op = pkt_info.hdr_words[1] & FA2_BCMHDR_OP_3_PROC_OP;

					/* Check for proc_ops 0, 1, 2 and 5 */
					if ((p_op <= 0x2) || (p_op == 0x5)) {

		    			fa2_transmit(skb, &pkt_info);
					}
				} else {
		    		fa2_transmit(skb, &pkt_info);
				}
#if defined(CONFIG_IPROC_FA2_CS_OFFLOAD_SMALL_PKT_WA)
				if (pkt_info.pkt_type == FA2_LOCAL_SMALL_TX_PKT) {
		    		fa2_transmit(skb, &pkt_info);
				}
#endif
			}

            /* If the packet is a L4 packet, and it is _not_ a forwarded packet,
             * then add bcm hdr 0x3 bytes
             */
            if ((pkt_info.proto != FA2_PROTO_NOT_SUPPORTED) &&
                (pkt_info.pkt_type == FA2_LOCAL_TX_PKT)) {

                /* Add Broadcom header bytes (8 bytes). Note 4 bytes were 
                 * 'push'ed earlier
                 */
				__skb_push(skb, 4);
        		*((uint32_t *)skb->data) = htonl(pkt_info.hdr_words[0]);
		        *((uint32_t *)skb->data + 1) = htonl(pkt_info.hdr_words[1]);
				__skb_trim(skb, skb->len);

				/* Send pkt to AXI1 */
				et = et_get_eth3_info();
            }


#else
		    fa2_transmit(skb, NULL);
#endif
        }
#endif /* defined(CONFIG_IPROC_FA2) */
	}
#endif /* (defined(CONFIG_IPROC_FA) || defined(CONFIG_IPROC_FA2) || defined(CONFIG_MACH_NSP)) */

	/* put it on the tx queue and call sendnext */
	ET_TXQ_LOCK(et);
	__skb_queue_tail(&et->txq[q], skb);
	et->etc->txq_state |= (1 << q);
	ET_TXQ_UNLOCK(et);

	if (!ET_ALL_PASSIVE_ENAB(et)) {
		ET_LOCK(et);
		et_sendnext(et);
		ET_UNLOCK(et);
	}
#ifdef ET_ALL_PASSIVE
	else
#ifdef CONFIG_BCM_IPROC_GMAC_TXONCPU1
		schedule_work_on(1, &et->txq_task.work);
#else
		schedule_work(&et->txq_task.work);
#endif

#endif /* ET_ALL_PASSIVE */

	ET_LOG("et%d: et_start ret\n", et->etc->unit, 0);

	return (0);
}

static void BCMFASTPATH
et_sendnext(et_info_t *et)
{
	etc_info_t *etc;
	struct sk_buff *skb;
	void *p, *n;
	uint32 priq = TX_Q0;
#ifdef DMA
	uint32 txavail;
#endif
#ifdef PRINT_PKT_SUM
	int	tagoff=12;
#endif /* PRINT_PKT_SUM */

#ifdef PRINT_PKT
	int i;
#endif /* PRINT_PKT */

	etc = et->etc;

	ET_TRACE(("et%d: et_sendnext\n", etc->unit));
	ET_LOG("et%d: et_sendnext", etc->unit, 0);

	/* dequeue packets from highest priority queue and send */
	while (1) {
		ET_TXQ_LOCK(et);

		if (etc->txq_state == 0)
			break;

		priq = etc_priq(etc->txq_state);

		ET_TRACE(("et%d: txq_state %x priq %d txavail %d\n",
		          etc->unit, etc->txq_state, priq,
		          *(uint *)etc->txavail[priq]));

		if ((skb = skb_peek(&et->txq[priq])) == NULL) {
			etc->txq_state &= ~(1 << priq);
			ET_TXQ_UNLOCK(et);
			continue;
		}

#ifdef DMA
		/* current highest priority dma queue is full */
		txavail = *(uint *)(etc->txavail[priq]);
		if ((PKTISCHAINED(skb) && (txavail < PKTCCNT(skb))) || (txavail == 0))
#else /* DMA */
		if (etc->pioactive != NULL)
#endif /* DMA */
			break;


		skb = __skb_dequeue(&et->txq[priq]);

		ET_TXQ_UNLOCK(et);
		ET_PRHDR("tx", (struct ether_header *)skb->data, skb->len, etc->unit);
		ET_PRPKT("txpkt", skb->data, skb->len, etc->unit);

#ifdef PRINT_PKT_SUM
		tagoff = 16;
		printf("et%d: txpkt len(0x%x) tag:0x%02x%02x%02x%02x\n", etc->unit, skb->len,
				skb->data[tagoff], skb->data[tagoff+1], skb->data[tagoff+2], skb->data[tagoff+3]);
#endif /* PRINT_PKT_SUM */
#ifdef PRINT_PKT
		printk("et%d: %s len(0x%x) txpkt:", etc->unit, __FUNCTION__, skb->len);
		for (i=0; i<skb->len; i++) {
			if ( (i % 16) == 0 )
				printk("\n");
			printk("%02x ", skb->data[i]);
		}
		printk("\n");
#endif /* PRINT_PKT */
		/* convert the packet. */
		p = PKTFRMNATIVE(etc->osh, skb);
		ASSERT(p != NULL);

		ET_TRACE(("%s: sdu %p chained %d chain sz %d next %p\n",
		          __FUNCTION__, p, PKTISCHAINED(p), PKTCCNT(p), PKTCLINK(p)));

		FOREACH_CHAINED_PKT(p, n) {
			/* replicate vlan header contents from curr frame */
			if (n != NULL) {
				uint8 *n_evh;
				n_evh = PKTPUSH(et->osh, n, VLAN_TAG_LEN);
				*(struct ethervlan_header *)n_evh =
				*(struct ethervlan_header *)PKTDATA(et->osh, p);
			}
			(*etc->chops->tx)(etc->ch, p);
#ifdef CONFIG_BCM_IPROC_GMAC_LOCK_OPT
            ET_LOCK(et);
#endif /* CONFIG_BCM_IPROC_GMAC_LOCK_OPT */
			etc->txframe++;
			etc->txbyte += PKTLEN(et->osh, p);
#ifdef CONFIG_BCM_IPROC_GMAC_LOCK_OPT
            ET_UNLOCK(et);
#endif /* CONFIG_BCM_IPROC_GMAC_LOCK_OPT */
		}
	}

	/* no flow control when qos is enabled */
	if (!et->etc->qos) {
		/* stop the queue whenever txq fills */
		if ((skb_queue_len(&et->txq[TX_Q0]) > DATAHIWAT) && !netif_queue_stopped(et->dev))
			netif_stop_queue(et->dev);
		else if (netif_queue_stopped(et->dev) &&
		         (skb_queue_len(&et->txq[TX_Q0]) < (DATAHIWAT/2)))
			netif_wake_queue(et->dev);
	} else {
		/* drop the frame if corresponding prec txq len exceeds hiwat
		 * when qos is enabled.
		 */
		if ((priq != TC_NONE) && (skb_queue_len(&et->txq[priq]) > DATAHIWAT)) {
			skb = __skb_dequeue(&et->txq[priq]);
			PKTCFREE(et->osh, skb, TRUE);
			ET_ERROR(("et%d: %s: txqlen %d\n", et->etc->unit,
			          __FUNCTION__, skb_queue_len(&et->txq[priq])));
		}
	}

	ET_TXQ_UNLOCK(et);
}

void
et_init(et_info_t *et, uint options)
{
	ET_TRACE(("et%d: et_init\n", et->etc->unit));
	ET_LOG("et%d: et_init", et->etc->unit, 0);

	etc_init(et->etc, options);

#if defined(CONFIG_MACH_HX4) || defined(CONFIG_MACH_HR2) || defined(CONFIG_MACH_KT2)
	netif_carrier_off(et->dev);
#endif /* defined(CONFIG_MACH_HX4) || defined(CONFIG_MACH_HR2) || defined(CONFIG_MACH_KT2) */
}


void
et_reset(et_info_t *et)
{
	ET_TRACE(("et%d: et_reset\n", et->etc->unit));

	etc_reset(et->etc);

	/* zap any pending dpc interrupt bits */
	et->events = 0;

	/* dpc will not be rescheduled */
	et->resched = 0;
}

void
et_up(et_info_t *et)
{
	etc_info_t *etc;

	etc = et->etc;

	if (etc->up)
		return;

	ET_TRACE(("et%d: et_up\n", etc->unit));

	etc_up(etc);

	/* schedule one second watchdog timer */
	et->timer.expires = jiffies + HZ;
	add_timer(&et->timer);
	et->set=TRUE;

	netif_start_queue(et->dev);

#ifdef GMAC3
	if (DEV_FWDER(et->etc)) {
		et->etc->pktc = TRUE;

		/*
		 * Attach my transmit handler to UPSTREAM fwder instance on core=unit
		 *		wl# MAC -> wl_sendup -> et_forward -> et::GMAC#
		 * and get the DNSTREAM direction transmit handler for use in sendup.
		 *		et_sendup/chain -> et->fwdh->start_xmit=wl_start -> wl# MAC
		 */
		et->fwdh = fwder_attach(et_forward, et->dev, et->etc->unit, FWD_UPSTREAM);
		/* fwder_dump_all(); */
	}
#endif	/* GMAC3 */
}

void
et_down(et_info_t *et, int reset)
{
	etc_info_t *etc;
	struct sk_buff *skb;
	int32 i;

	etc = et->etc;

	ET_TRACE(("et%d: et_down\n", etc->unit));

#ifdef GMAC3
	if (DEV_FWDER(et->etc)) {
		et->fwdh = fwder_dettach(et->fwdh);
		/* fwder_dump_all(); */
	}
#endif	/* GMAC3 */

	netif_down(et->dev);
	netif_stop_queue(et->dev);

	/* stop watchdog timer */
	del_timer(&et->timer);
	et->set = FALSE;

#ifdef GMAC_RATE_LIMITING
	/* stop ratelimiting timer */
	del_timer(&et->etc->rl_timer);
	et->etc->rl_set = FALSE;
#endif /* GMAC_RATE_LIMITING */

	etc_down(etc, reset);

	/* flush the txq(s) */
	for (i = 0; i < NUMTXQ; i++)
		while ((skb = skb_dequeue(&et->txq[i])))
			PKTFREE(etc->osh, skb, TRUE);

#if !defined(NAPI_POLL) && !defined(NAPI2_POLL)
	/* kill dpc */
	ET_UNLOCK(et);
	tasklet_kill(&et->tasklet);
	ET_LOCK(et);
#endif /* NAPI_POLL */
}

/*
 * These are interrupt on/off entry points. Disable interrupts
 * during interrupt state transition.
 */
void
et_intrson(et_info_t *et)
{
	unsigned long flags;
	INT_LOCK(et, flags);
	(*et->etc->chops->intrson)(et->etc->ch);
	INT_UNLOCK(et, flags);
}

static void
_et_watchdog(struct net_device *dev)
{
	et_info_t *et;

	et = ET_INFO(dev);

	ET_LOCK(et);

	etc_watchdog(et->etc);

	if (et->set) {
		/* reschedule one second watchdog timer */
		et->timer.expires = jiffies + HZ;
		add_timer(&et->timer);
	}

#ifdef CTFPOOL
	/* allocate and add a new skb to the pkt pool */

#ifndef CONFIG_BCM_IPROC_GMAC_SKB_RECYCLING
	if (CTF_ENAB(et->cih))
#endif /* !CONFIG_BCM_IPROC_GMAC_SKB_RECYCLING */

		osl_ctfpool_replenish(et->osh, CTFPOOL_REFILL_THRESH);
#endif /* CTFPOOL */
	ET_UNLOCK(et);
}

#ifdef ET_ALL_PASSIVE
static void
et_watchdog_task(et_task_t *task)
{
	et_info_t *et = ET_INFO((struct net_device *)task->context);

	_et_watchdog((struct net_device *)task->context);
	MFREE(et->osh, task, sizeof(et_task_t));
}
#endif /* ET_ALL_PASSIVE */

static void
et_watchdog(ulong data)
{
	struct net_device *dev = (struct net_device *)data;
#ifdef ET_ALL_PASSIVE
	et_info_t *et = ET_INFO(dev);
#endif /* ET_ALL_PASSIVE */

	if (!ET_ALL_PASSIVE_ENAB(et))
		_et_watchdog(dev);
#ifdef ET_ALL_PASSIVE
	else
		et_schedule_task(et, et_watchdog_task, dev);
#endif /* ET_ALL_PASSIVE */
}

/* Rate limiting */
#ifdef GMAC_RATE_LIMITING
static void et_release_congestion(ulong data) 
{
	struct net_device *dev = (struct net_device *)data;
	et_info_t *et = ET_INFO(dev);

	if (!et) {
		return;
	}
	if (et->etc->rl_stopping_broadcasts) {
		//printf("et%d: %s: releasing broadcast packet congestion; dropped: 0x%x\n", et->etc->unit, __FUNCTION__, et->etc->rl_dropped_bc_packets);
		et->etc->rl_stopping_broadcasts = 0;
		/* clear the number of dropped broadcast packets */
		et->etc->rl_dropped_bc_packets = 0;
	}
	if (et->etc->rl_stopping_all_packets) {
		//printf("et%d: %s: releasing all packet congestion; dropped: 0x%x\n", et->etc->unit, __FUNCTION__, et->etc->rl_dropped_all_packets);
		et->etc->rl_stopping_all_packets = 0;
		et->etc->rl_dropped_all_packets = 0;
	}
}
#endif /* GMAC_RATE_LIMITING */



#ifdef SIOCETHTOOL
static int
et_ethtool(et_info_t *et, struct ethtool_cmd *ecmd)
{
	int ret = 0;
	int speed;
	struct ethtool_drvinfo *info;

	ET_LOCK(et);

	switch (ecmd->cmd) {
	case ETHTOOL_GSET:
		ecmd->supported = (SUPPORTED_10baseT_Half | SUPPORTED_10baseT_Full |
		                   SUPPORTED_100baseT_Half | SUPPORTED_100baseT_Full |
		                   SUPPORTED_Autoneg);
		ecmd->advertising = ADVERTISED_TP;
		ecmd->advertising |= (et->etc->advertise & ADV_10HALF) ?
		        ADVERTISED_10baseT_Half : 0;
		ecmd->advertising |= (et->etc->advertise & ADV_10FULL) ?
		        ADVERTISED_10baseT_Full : 0;
		ecmd->advertising |= (et->etc->advertise & ADV_100HALF) ?
		        ADVERTISED_100baseT_Half : 0;
		ecmd->advertising |= (et->etc->advertise & ADV_100FULL) ?
		        ADVERTISED_100baseT_Full : 0;
		ecmd->advertising |= (et->etc->advertise2 & ADV_1000FULL) ?
		        ADVERTISED_1000baseT_Full : 0;
		ecmd->advertising |= (et->etc->advertise2 & ADV_1000HALF) ?
		        ADVERTISED_1000baseT_Half : 0;
		ecmd->advertising |= (et->etc->forcespeed == ET_AUTO) ?
		        ADVERTISED_Autoneg : 0;
		if (et->etc->linkstate) {
			ecmd->speed = (et->etc->speed == 1000) ? SPEED_1000 :
			               ((et->etc->speed == 100) ? SPEED_100 : SPEED_10);
			ecmd->duplex = (et->etc->duplex == 1) ? DUPLEX_FULL : DUPLEX_HALF;
		} else {
			ecmd->speed = 0;
			ecmd->duplex = 0;
		}
		ecmd->port = PORT_TP;
		ecmd->phy_address = 0;
		ecmd->transceiver = XCVR_INTERNAL;
		ecmd->autoneg = (et->etc->forcespeed == ET_AUTO) ? AUTONEG_ENABLE : AUTONEG_DISABLE;
		ecmd->maxtxpkt = 0;
		ecmd->maxrxpkt = 0;
		break;
	case ETHTOOL_SSET:
		if (!capable(CAP_NET_ADMIN)) {
			ret = -EPERM;
			break;
		}
		else if (ecmd->speed == SPEED_10 && ecmd->duplex == DUPLEX_HALF)
			speed = ET_10HALF;
		else if (ecmd->speed == SPEED_10 && ecmd->duplex == DUPLEX_FULL)
			speed = ET_10FULL;
		else if (ecmd->speed == SPEED_100 && ecmd->duplex == DUPLEX_HALF)
			speed = ET_100HALF;
		else if (ecmd->speed == SPEED_100 && ecmd->duplex == DUPLEX_FULL)
			speed = ET_100FULL;
		else if (ecmd->speed == SPEED_1000 && ecmd->duplex == DUPLEX_FULL)
			speed = ET_1000FULL;
		else if (ecmd->autoneg == AUTONEG_ENABLE)
			speed = ET_AUTO;
		else {
			ret = -EINVAL;
			break;
		}
		ret = etc_ioctl(et->etc, ETCSPEED, &speed);
		break;
	case ETHTOOL_GDRVINFO:
		info = (struct ethtool_drvinfo *)ecmd;
		bzero(info, sizeof(struct ethtool_drvinfo));
		info->cmd = ETHTOOL_GDRVINFO;
		sprintf(info->driver, "et%d", et->etc->unit);
		strcpy(info->version, EPI_VERSION_STR);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	ET_UNLOCK(et);

	return (ret);
}
#endif /* SIOCETHTOOL */

static int
et_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	et_info_t *et;
	int error;
	char *buf;
	int size, ethtoolcmd;
	bool get = 0, set;
	et_var_t *var = NULL;
	void *buffer = NULL;

	et = ET_INFO(dev);

	ET_TRACE(("et%d: et_ioctl: cmd 0x%x\n", et->etc->unit, cmd));

	switch (cmd) {
#ifdef SIOCETHTOOL
	case SIOCETHTOOL:
		if (copy_from_user(&ethtoolcmd, ifr->ifr_data, sizeof(uint32)))
			return (-EFAULT);

		if (ethtoolcmd == ETHTOOL_GDRVINFO)
			size = sizeof(struct ethtool_drvinfo);
		else
			size = sizeof(struct ethtool_cmd);
		get = TRUE; set = TRUE;
		break;
#endif /* SIOCETHTOOL */
	case SIOCGETCDUMP:
		size = IOCBUFSZ;
		get = TRUE; set = FALSE;
		break;
	case SIOCGETCPHYRD:
	case SIOCGETCPHYRD2:
	case SIOCGETCROBORD:
		size = sizeof(int) * 2;
		get = TRUE; set = TRUE;
		break;
	case SIOCSETCPHYWR:
	case SIOCSETCPHYWR2:
	case SIOCSETCROBOWR:
		size = sizeof(int) * 2;
		get = FALSE; set = TRUE;
		break;
	case SIOCSETGETVAR:
		size = sizeof(et_var_t);
		set = TRUE;
		break;
	default:
		size = sizeof(int);
		get = FALSE; set = TRUE;
		break;
	}

	if ((buf = MALLOC(et->osh, size)) == NULL) {
		ET_ERROR(("et: et_ioctl: out of memory, malloced %d bytes\n", MALLOCED(et->osh)));
		return (-ENOMEM);
	}

	if (set && copy_from_user(buf, ifr->ifr_data, size)) {
		MFREE(et->osh, buf, size);
		return (-EFAULT);
	}

	if (cmd == SIOCSETGETVAR) {
		var = (et_var_t *)buf;
		if (var->buf) {
			if (!var->set)
				get = TRUE;

			if (!(buffer = (void *) MALLOC(et->osh, var->len))) {
				ET_ERROR(("et: et_ioctl: out of memory, malloced %d bytes\n",
					MALLOCED(et->osh)));
				MFREE(et->osh, buf, size);
				return (-ENOMEM);
			}

			if (copy_from_user(buffer, var->buf, var->len)) {
				MFREE(et->osh, buffer, var->len);
				MFREE(et->osh, buf, size);
				return (-EFAULT);
			}
		}
	}

	switch (cmd) {
#ifdef SIOCETHTOOL
	case SIOCETHTOOL:
		error = et_ethtool(et, (struct ethtool_cmd *)buf);
		break;
#endif /* SIOCETHTOOL */
	case SIOCSETGETVAR:
		ET_LOCK(et);
		error = etc_iovar(et->etc, var->cmd, var->set, buffer);
		ET_UNLOCK(et);
		if (!error && get)
			error = copy_to_user(var->buf, buffer, var->len);

		if (buffer)
			MFREE(et->osh, buffer, var->len);
		break;
	default:
		ET_LOCK(et);
		error = etc_ioctl(et->etc, cmd - SIOCSETCUP, buf) ? -EINVAL : 0;
		ET_UNLOCK(et);
		break;
	}

	if (!error && get)
		error = copy_to_user(ifr->ifr_data, buf, size);

	MFREE(et->osh, buf, size);

	return (error);
}

static struct net_device_stats *
et_get_stats(struct net_device *dev)
{
	et_info_t *et;
	etc_info_t *etc;
	struct net_device_stats *stats;
	int locked = 0;

	et = ET_INFO(dev);

	ET_TRACE(("et%d: et_get_stats\n", et->etc->unit));

	if (!in_atomic()) {
		locked = 1;
		ET_LOCK(et);
	}

	etc = et->etc;
	stats = &et->stats;
	bzero(stats, sizeof(struct net_device_stats));

	/* refresh stats */
	if (et->etc->up)
		(*etc->chops->statsupd)(etc->ch);

	/* SWAG */
	stats->rx_packets = etc->rxframe;
	stats->tx_packets = etc->txframe;
	stats->rx_bytes = etc->rxbyte;
	stats->tx_bytes = etc->txbyte;
	stats->rx_errors = etc->rxerror;
	stats->tx_errors = etc->txerror;

	if (ET_GMAC(etc)) {
		gmacmib_t *mib;

		mib = etc->mib;
		stats->collisions = mib->tx_total_cols;
		stats->rx_length_errors = (mib->rx_oversize_pkts + mib->rx_undersize);
		stats->rx_crc_errors = mib->rx_crc_errs;
		stats->rx_frame_errors = mib->rx_align_errs;
		stats->rx_missed_errors = mib->rx_missed_pkts;
	} else {
		bcmenetmib_t *mib;

		mib = etc->mib;
		stats->collisions = mib->tx_total_cols;
		stats->rx_length_errors = (mib->rx_oversize_pkts + mib->rx_undersize);
		stats->rx_crc_errors = mib->rx_crc_errs;
		stats->rx_frame_errors = mib->rx_align_errs;
		stats->rx_missed_errors = mib->rx_missed_pkts;

	}

	stats->rx_fifo_errors = etc->rxoflo;
	stats->rx_over_errors = etc->rxoflo;
	stats->tx_fifo_errors = etc->txuflo;

	//etc_robomib(etc);

	if (locked)
		ET_UNLOCK(et);

	return (stats);
}

static int
et_set_mac_address(struct net_device *dev, void *addr)
{
	et_info_t *et;
	struct sockaddr *sa = (struct sockaddr *) addr;

	et = ET_INFO(dev);
	ET_TRACE(("et%d: et_set_mac_address\n", et->etc->unit));

	if (et->etc->up)
		return -EBUSY;

	bcopy(sa->sa_data, dev->dev_addr, ETHER_ADDR_LEN);
	bcopy(dev->dev_addr, &et->etc->cur_etheraddr, ETHER_ADDR_LEN);

	return 0;
}

static void
et_set_multicast_list(struct net_device *dev)
{
	et_info_t *et;
	etc_info_t *etc;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 35)
	struct dev_mc_list *mclist;
#else
	struct netdev_hw_addr *ha ;
#endif
	int i;
	int locked = 0;

	et = ET_INFO(dev);
	etc = et->etc;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 35)
	mclist = NULL ;		/* fend off warnings */
#else
	ha = NULL ;
#endif

	ET_TRACE(("et%d: et_set_multicast_list\n", etc->unit));

	if (!in_atomic()) {
		locked = 1;
		ET_LOCK(et);
	}

	if (etc->up) {
		etc->promisc = (dev->flags & IFF_PROMISC)? TRUE: FALSE;
		etc->allmulti = (dev->flags & IFF_ALLMULTI)? TRUE: etc->promisc;

		/* copy the list of multicasts into our private table */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 35)
		for (i = 0, mclist = dev->mc_list; mclist && (i < dev->mc_count);
			i++, mclist = mclist->next) {
			if (i >= MAXMULTILIST) {
				etc->allmulti = TRUE;
				i = 0;
				break;
			}
			etc->multicast[i] = *((struct ether_addr *)mclist->dmi_addr);
		}
#else	/* >= 2.6.36 */
		i = 0;
		netdev_for_each_mc_addr(ha, dev) {
			i ++;
			if (i >= MAXMULTILIST) {
				etc->allmulti = TRUE;
				i = 0;
				break;
			}
			etc->multicast[i] = *((struct ether_addr *)ha->addr);
		} /* for each ha */
#endif /* LINUX_VERSION_CODE */
		etc->nmulticast = i;

		/* LR: partial re-init, DMA is already initialized */
		et_init(et, ET_INIT_INTRON);
	}

	if (locked)
		ET_UNLOCK(et);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 20)
static irqreturn_t BCMFASTPATH
et_isr(int irq, void *dev_id)
#else
static irqreturn_t BCMFASTPATH
et_isr(int irq, void *dev_id, struct pt_regs *ptregs)
#endif
{
	et_info_t *et;
	struct chops *chops;
	void *ch;
	uint events = 0;

	et = (et_info_t *)dev_id;
	chops = et->etc->chops;
	ch = et->etc->ch;

	/* guard against shared interrupts */
	if (!et->etc->up) {
		ET_TRACE(("et%d: et_isr: not up\n", et->etc->unit));
		goto done;
	}

	/* get interrupt condition bits */
	events = (*chops->getintrevents)(ch, TRUE);

	/* not for us */
	if (!(events & INTR_NEW))
		goto done;

	ET_TRACE(("et%d: et_isr: events 0x%x\n", et->etc->unit, events));
	ET_LOG("et%d: et_isr: events 0x%x", et->etc->unit, events);

	/* disable interrupts */
	(*chops->intrsoff)(ch);

	/* save intstatus bits */
	ASSERT(et->events == 0);
	et->events = events;

	ASSERT(et->resched == FALSE);

#ifdef NAPI2_POLL

	napi_schedule(&et->napi_poll);

#elif defined(NAPI_POLL)
	/* allow the device to be added to the cpu polling list if we are up */
	if (netif_rx_schedule_prep(et->dev)) {
		/* tell the network core that we have packets to send up */
		__netif_rx_schedule(et->dev);
	} else {
		ET_ERROR(("et%d: et_isr: intr while in poll!\n",
		          et->etc->unit));
		(*chops->intrson)(ch);
	}
#else /* ! NAPI_POLL && ! NAPI2_POLL */
	/* schedule dpc */
#ifdef ET_ALL_PASSIVE
	if (ET_ALL_PASSIVE_ENAB(et)) {
		schedule_work(&et->dpc_task.work);
	} else
#endif /* ET_ALL_PASSIVE */
	tasklet_schedule(&et->tasklet);
#endif /* NAPI_POLL */

done:
	ET_LOG("et%d: et_isr ret", et->etc->unit, 0);

	return IRQ_RETVAL(events & INTR_NEW);
}

#ifdef GMAC3
static struct net_device * BCMFASTPATH
et_find_dev(struct sk_buff *skb) {

	/* ----------------------------------------------- */
	/* ----------------------------------------------- */

	return NULL;
}
#endif	/* GMAC3 */

#ifdef PKTC
static void BCMFASTPATH
et_sendup_chain(et_info_t *et, void *h)
{
	struct sk_buff *skb;
	uint sz = PKTCCNT(h);

	ASSERT(h != NULL);
	ASSERT((sz > 0) && (sz <= PKTCBND));
	ET_TRACE(("et%d: %s: sending up packet chain of sz %d\n",
	          et->etc->unit, __FUNCTION__, sz));
	et->etc->chained += sz;
#ifdef BCMDBG
	et->etc->chainsz[sz - 1] += sz;
#endif
	et->etc->currchainsz = sz;
	et->etc->maxchainsz = MAX(et->etc->maxchainsz, sz);

#ifdef GMAC3
	/* Forward chain directly to wl transmit */
	if (DEV_FWDER(et->etc)) {
		struct net_device * dev;

		//skb = PKTTOFORWARD(et->etc->osh, h, sz);
		skb = PKTTONATIVE(et->etc->osh, h);
		skb->dev = et->dev;

		dev = et_find_dev(skb);

		if (fwder_transmit(skb, dev, et->fwdh, sz) == FWDER_FAILURE) {
			PKTCFREE(et->etc->osh, skb, FALSE);
		}
	}
	else
		ASSERT(DEV_NTKIF(et->etc));
#else	/* !GMAC3 */

	skb = PKTTONATIVE(et->etc->osh, h);
	skb->dev = et->dev;

	/* send up the packet chain */
	ctf_forward(et->cih, h, et->dev);
#endif	/* !GMAC3 */

}
#endif /* PKTC */

static inline int
et_rxevent(osl_t *osh, et_info_t *et, struct chops *chops, void *ch, int quota)
{
	uint processed = 0;
	void *p, *h = NULL, *t = NULL;
	struct sk_buff *skb;
#ifdef PKTC
	pktc_data_t cd[PKTCMC] = {{0}};
	uint8 *evh, prio;
	int32 i = 0, cidx = 0;
#ifdef GMAC3
	bool chaining = DEV_FWDER(et->etc);
#else	/* !GMAC3 */
	bool chaining = PKTC_ENAB(et);
#endif	/* !GMAC3 */
#endif

#ifdef GMAC_RATE_LIMITING
	/* rate limiting */
	if ( et->etc->rl_enabled )
		etc_check_rate_limiting(et->etc, ch);
#endif /* GMAC_RATE_LIMITING */

	/* read the buffers first */
	while ((p = (*chops->rx)(ch))) {
#ifdef PKTC
		ASSERT(PKTCLINK(p) == NULL);
		evh = PKTDATA(et->osh, p) + HWRXOFF;
#ifdef GMAC3
		if (DEV_FWDER(et->etc))
			prio = IP_TOS46(evh + ETHER_HDR_LEN) >> IPV4_TOS_PREC_SHIFT;
		else
#endif	/* GMAC3 */
		prio = IP_TOS46(evh + ETHERVLAN_HDR_LEN) >> IPV4_TOS_PREC_SHIFT;
		if (cd[0].h_da == NULL) {
			cd[0].h_da = evh; cd[0].h_sa = evh + ETHER_ADDR_LEN;
			cd[0].h_prio = prio;
		}

		/* if current frame doesn't match cached src/dest/prio or has err flags
		 * set then stop chaining.
		 */
		if (chaining) {
			for (i = 0; i <= cidx; i++) {
				if (PKT_CHAINABLE(et, p, evh, prio, cd[i].h_sa,
				                  cd[i].h_da, cd[i].h_prio))
					break;
				else if ((i + 1 < PKTCMC) && (cd[i + 1].h_da == NULL)) {
					cidx++;
					cd[cidx].h_da = evh;
					cd[cidx].h_sa = evh + ETHER_ADDR_LEN;
					cd[cidx].h_prio = prio;
				}
			}
			chaining = (i < PKTCMC);
		}

		if (chaining) {
			PKTCENQTAIL(cd[i].chead, cd[i].ctail, p);
			/* strip off rxhdr */
			PKTPULL(et->osh, p, HWRXOFF);

			et->etc->rxframe++;
			et->etc->rxbyte += PKTLEN(et->osh, p);

			/* strip off crc32 */
			PKTSETLEN(et->osh, p, PKTLEN(et->osh, p) - ETHER_CRC_LEN);

#ifndef GMAC3
			/* update header for non-first frames */
			if (cd[i].chead != p)
				CTF_HOTBRC_L2HDR_PREP(et->osh, et->brc_hot, prio,
				                      PKTDATA(et->osh, p), p);
#endif /* !GMAC3 */

			PKTCINCRCNT(cd[i].chead);
			PKTSETCHAINED(et->osh, p);
			PKTCADDLEN(cd[i].chead, PKTLEN(et->osh, p));
		} else
			PKTCENQTAIL(h, t, p);
#else /* PKTC */
		PKTSETLINK(p, NULL);
		if (t == NULL)
			h = t = p;
		else {
			PKTSETLINK(t, p);
			t = p;
		}
#endif /* PKTC */

		/* we reached quota already */
		if (++processed >= quota) {
			/* reschedule et_dpc()/et_poll() */
			et->resched = TRUE;
			break;
		}
	}

	/* prefetch the headers */
	if (h != NULL)
#ifdef CONFIG_BCM_IPROC_GMAC_PREFETCH
    prefetch_range(PKTDATA(osh, h), SKB_DATA_PREFETCH_LEN);
#else
		ETPREFHDRS(PKTDATA(osh, h), PREFSZ);
#endif
            
	/* post more rx bufs */
	(*chops->rxfill)(ch);

#ifdef PKTC
	/* send up the chain(s) at one fell swoop */
	ASSERT(cidx < PKTCMC);
	for (i = 0; i <= cidx; i++) {
		if (cd[i].chead != NULL) {
#ifdef GMAC3
			PKTSETPRIO(cd[i].chead, cd[i].h_prio);
#endif
			et_sendup_chain(et, cd[i].chead);
		}
	}
#endif

	while ((p = h) != NULL) {
#ifdef PKTC
		h = PKTCLINK(h);
		PKTSETCLINK(p, NULL);
#else
		h = PKTLINK(h);
		PKTSETLINK(p, NULL);
#endif
		/* prefetch the headers */
		if (h != NULL)
#ifdef CONFIG_BCM_IPROC_GMAC_PREFETCH
    prefetch_range(PKTDATA(osh, h), SKB_DATA_PREFETCH_LEN);
#else
    ETPREFHDRS(PKTDATA(osh, h), PREFSZ);
#endif
                    
#ifdef GMAC3
		if (DEV_FWDER(et->etc)) {
			uint8 *evh1;
			//skb = PKTTOFORWARD(osh, p, 1);
			skb = PKTTONATIVE(osh, p);
			evh1 = skb->data + HWRXOFF;
			skb->priority = IPV4_TOS(evh1 + ETHER_HDR_LEN) >> IPV4_TOS_PREC_SHIFT;
		} else
#endif
		skb = PKTTONATIVE(osh, p);
		et->etc->unchained++;
		et_sendup(et, skb);
	}

	return (processed);
}

#if defined(NAPI2_POLL)
static int BCMFASTPATH
et_poll(struct napi_struct *napi, int budget)
{
	int quota = budget;
	struct net_device *dev = napi->dev;
	et_info_t *et = ET_INFO(dev);

#elif defined(NAPI_POLL)
static int BCMFASTPATH
et_poll(struct net_device *dev, int *budget)
{
	int quota = min(RXBND, *budget);
	et_info_t *et = ET_INFO(dev);
#else /* NAPI_POLL */
static void BCMFASTPATH
et_dpc(ulong data)
{
	et_info_t *et = (et_info_t *)data;
#ifndef GMAC3
	int quota = PKTC_ENAB(et) ? et->etc->pktcbnd : RXBND;
#else /* GMAC3 */
	int quota = PKTC_ENAB(et) ? et->etc->pktcbnd : RXBND;
#endif /* GMAC3 */
#endif /* NAPI_POLL */
	struct chops *chops;
	void *ch;
	osl_t *osh;
	uint nrx = 0;

	chops = et->etc->chops;
	ch = et->etc->ch;
	osh = et->etc->osh;

	ET_TRACE(("et%d: et_dpc: events 0x%x\n", et->etc->unit, et->events));
	ET_LOG("et%d: et_dpc: events 0x%x", et->etc->unit, et->events);

#if !defined(NAPI_POLL) && !defined(NAPI2_POLL)
	ET_LOCK(et);
#endif /* ! NAPIx_POLL */

	if (!et->etc->up)
		goto done;

	/* get interrupt condition bits again when dpc was rescheduled */
	if (et->resched) {
		et->events = (*chops->getintrevents)(ch, FALSE);
		et->resched = FALSE;
	}

	if (et->events & INTR_RX)
		nrx = et_rxevent(osh, et, chops, ch, quota);

	if (et->events & INTR_TX) {
		(*chops->txreclaim)(ch, FALSE);
	}

	(*chops->rxfill)(ch);

	/* handle error conditions, if reset required leave interrupts off! */
	if (et->events & INTR_ERROR) {
		if ((*chops->errors)(ch))
			et_init(et, ET_INIT_INTROFF);
		else
			if (nrx < quota)
				nrx += et_rxevent(osh, et, chops, ch, quota);
	}

	/* run the tx queue */
	if (et->etc->txq_state != 0) {
		if (!ET_ALL_PASSIVE_ENAB(et)) {
			et_sendnext(et);
			}
#ifdef ET_ALL_PASSIVE
		else
#ifdef CONFIG_BCM_IPROC_GMAC_TXONCPU1
		schedule_work_on(1, &et->txq_task.work);
#else
		schedule_work(&et->txq_task.work);
#endif

#endif /* ET_ALL_PASSIVE */
		}

	/* clear this before re-enabling interrupts */
	et->events = 0;

	/* something may bring the driver down */
	if (!et->etc->up) {
		et->resched = FALSE;
		goto done;
	}

#if !defined(NAPI_POLL) && !defined(NAPI2_POLL)
#ifdef ET_ALL_PASSIVE
	if (et->resched) {
		if (!ET_ALL_PASSIVE_ENAB(et))
			tasklet_schedule(&et->tasklet);
		else
			schedule_work(&et->dpc_task.work);
	}
	else
		(*chops->intrson)(ch);
#else /* ET_ALL_PASSIVE */
	/* there may be frames left, reschedule et_dpc() */
	if (et->resched)
		tasklet_schedule(&et->tasklet);
	/* re-enable interrupts */
	else
		(*chops->intrson)(ch);
#endif /* ET_ALL_PASSIVE */
#endif /* ! NAPIx_POLL */

done:
	ET_LOG("et%d: et_dpc ret", et->etc->unit, 0);

#if defined(NAPI_POLL) || defined(NAPI2_POLL)
#ifdef	NAPI_POLL
	/* update number of frames processed */
	*budget -= nrx;
	dev->quota -= nrx;

	ET_TRACE(("et%d: et_poll: quota %d budget %d\n",
	          et->etc->unit, dev->quota, *budget));
#else
	ET_TRACE(("et%d: et_poll: budget %d\n",
	          et->etc->unit, budget));
#endif

	/* we got packets but no quota */
	if (et->resched)
		/* indicate that we are not done, don't enable
		 * interrupts yet. linux network core will call
		 * us again.
		 */
		return (1);

#ifdef	NAPI2_POLL
	napi_complete(napi);
#else	/* NAPI_POLL */
	netif_rx_complete(dev);
#endif

	/* enable interrupts now */
	(*chops->intrson)(ch);

	/* indicate that we are done */
	return (0);
#else /* NAPI_POLL */
	ET_UNLOCK(et);
	return;
#endif /* NAPI_POLL */
}

#ifdef ET_ALL_PASSIVE
static void BCMFASTPATH
et_dpc_work(struct et_task *task)
{
#if !defined(NAPI_POLL) && !defined(NAPI2_POLL)
	et_info_t *et = (et_info_t *)task->context;
	et_dpc((unsigned long)et);
#else
	BUG_ON(1);
#endif
	return;
}
#endif /* ET_ALL_PASSIVE */

static void
et_error(et_info_t *et, struct sk_buff *skb, void *rxh)
{
	uchar eabuf[32];
	struct ether_header *eh;

	eh = (struct ether_header *)skb->data;
	bcm_ether_ntoa((struct ether_addr *)eh->ether_shost, eabuf);

	if (RXH_OVERSIZE(et->etc, rxh)) {
		ET_ERROR(("et%d: rx: over size packet from %s\n", et->etc->unit, eabuf));
	}
	if (RXH_CRC(et->etc, rxh)) {
		ET_ERROR(("et%d: rx: crc error from %s\n", et->etc->unit, eabuf));
	}
	if (RXH_OVF(et->etc, rxh)) {
		ET_ERROR(("et%d: rx: fifo overflow\n", et->etc->unit));
	}
	if (RXH_NO(et->etc, rxh)) {
		ET_ERROR(("et%d: rx: crc error (odd nibbles) from %s\n",
		          et->etc->unit, eabuf));
	}
	if (RXH_RXER(et->etc, rxh)) {
		ET_ERROR(("et%d: rx: symbol error from %s\n", et->etc->unit, eabuf));
	}
}

static inline int32
et_ctf_forward(et_info_t *et, struct sk_buff *skb)
{
#ifdef HNDCTF
	/* use slow path if ctf is disabled */
	if (!CTF_ENAB(et->cih))
		return (BCME_ERROR);

	/* try cut thru first */
	if (ctf_forward(et->cih, skb, skb->dev) != BCME_ERROR)
		return (BCME_OK);

	/* clear skipct flag before sending up */
	PKTCLRSKIPCT(et->osh, skb);
#endif /* HNDCTF */

#ifdef CTFPOOL
	/* allocate and add a new skb to the pkt pool */
	if (PKTISFAST(et->osh, skb))
		osl_ctfpool_add(et->osh);

	/* clear fast buf flag before sending up */
	PKTCLRFAST(et->osh, skb);

	/* re-init the hijacked field */
	CTFPOOLPTR(et->osh, skb) = NULL;
#endif /* CTFPOOL */

	/* map the unmapped buffer memory before sending up */
	PKTCTFMAP(et->osh, skb);

	return (BCME_ERROR);
}

void BCMFASTPATH
et_sendup(et_info_t *et, struct sk_buff *skb)
{
	etc_info_t *etc;
	void *rxh;
	uint16 flags;
#ifdef PRINT_PKT
	int i;
#endif /* PRINT_PKT */
#if defined(CONFIG_IPROC_FA2)
	uint32 rcv_sts_word;
#endif

	etc = et->etc;

	/* packet buffer starts with rxhdr */
	rxh = skb->data;

#if defined(CONFIG_IPROC_FA2)
	rcv_sts_word = LTOH32(*((uint32 *)skb->data));
#endif
	/* strip off rxhdr */
	__skb_pull(skb, HWRXOFF);

	ET_TRACE(("et%d: et_sendup: %d bytes\n", et->etc->unit, skb->len));
	ET_LOG("et%d: et_sendup: len %d", et->etc->unit, skb->len);

	etc->rxframe++;
	etc->rxbyte += skb->len;

#if (defined(CONFIG_IPROC_FA) || defined(CONFIG_IPROC_FA2) || defined(CONFIG_MACH_NSP))
	if (et_ctf_active(et)) {
#if defined(CONFIG_IPROC_FA)
        if(!gBypass) {
		    if (fc_receive(skb) == -1) 
			    goto err;
        }
#endif /* defined(CONFIG_IPROC_FA) */

#if defined(CONFIG_IPROC_FA2)
        if(!gBypass) {

			/* If pipeline did not indicate error, proceed with rx processing */
			if (!(RXH_CTFERROR(etc, rxh))) {
		    	if (fa2_receive(skb) == FA2_PKT_DONE) {
                	goto drop_pkt;
            	}
			} else {
				/*
    			printk(KERN_DEBUG "\n=== rxstsword is 0x%08X\n", 
						rcv_sts_word);
				*/
			}
        }
#endif /* defined(CONFIG_IPROC_FA2) */

		/* remove brcm tag */
		ET_TRACE(("Removing BRCM TAG\n"));
		/* size depends on egress tag opcode */
		switch ((skb->data[0] & 0xe0) >> 5) {
		case 0:			   
		case 1:
		case 2:
			skb_pull(skb, 4);
			break;
		case 3:
			skb_pull(skb, 8);
			break;
		}
	}
#endif /* (defined(CONFIG_IPROC_FA) || defined(CONFIG_IPROC_FA2) || defined(CONFIG_MACH_NSP)) */

	/* eh should now be aligned 2-mod-4 */
	ASSERT(((ulong)skb->data & 3) == 2);

	/* strip off crc32 */
	__skb_trim(skb, skb->len - ETHER_CRC_LEN);

	ET_PRHDR("rx", (struct ether_header *)skb->data, skb->len, etc->unit);
	ET_PRPKT("rxpkt", skb->data, skb->len, etc->unit);
#ifdef PRINT_PKT_SUM
	printk("et%d: rxpkt len(0x%x) tag:0x%02x%02x%02x%02x\n", etc->unit, skb->len,
			skb->data[12], skb->data[13], skb->data[14], skb->data[15]);
#endif /* PRINT_PKT_SUM */
#ifdef PRINT_PKT
	printk("et%d: %s len(0x%x) rxpkt:", etc->unit, __FUNCTION__, skb->len);
	for (i=0; i<skb->len; i++) {
		if ( (i % 16) == 0 )
			printk("\n");
		printk("%02x ", skb->data[i]);
	}
	printk("\n");
#endif /* PRINT_PKT */

	/* get the error flags */
	flags = RXH_FLAGS(etc, rxh);

	/* check for reported frame errors */
	if (flags)
		goto err;

	skb->dev = et->dev;

#ifdef GMAC3
	if (DEV_FWDER(et->etc)) {
		struct net_device * dev = et_find_dev(skb);

		if (fwder_transmit(skb, dev, et->fwdh, 1) == FWDER_FAILURE) {
			PKTFRMNATIVE(etc->osh, skb);
			PKTFREE(etc->osh, skb, FALSE);
		}
		return;
	}
#endif	/* !GMAC3 */

#ifdef HNDCTF
	/* try cut thru' before sending up */
	if (et_ctf_forward(et, skb) != BCME_ERROR)
		return;
#endif /* HNDCTF */

	ASSERT(!PKTISCHAINED(skb));

	/* extract priority from payload and store it out-of-band
	 * in skb->priority
	 */
	if (et->etc->qos)
		pktsetprio(skb, TRUE);

	skb->protocol = eth_type_trans(skb, et->dev);

#ifdef CONFIG_BCM_IPROC_GMAC_PREFETCH
#ifndef CONFIG_BCM_IPROC_GMAC_SKB_RECYCLING
    {
        struct sk_buff *next = skb->next;
        while (1) {
            if (next != NULL) {

                prefetch_range(next, SKB_PREFETCH_LEN);
                next = next->next;
            } else {
                break;
            }
        } 
    }
#endif /* CONFIG_BCM_IPROC_GMAC_SKB_RECYCLING */
#endif

	/* send it up */
#if defined(NAPI_POLL) || defined(NAPI2_POLL)
	netif_receive_skb(skb);
#else /* NAPI_POLL */
	netif_rx(skb);
#endif /* NAPI_POLL */

	ET_LOG("et%d: et_sendup ret", et->etc->unit, 0);

	return;

err:
	et_error(et, skb, rxh);

#if defined(CONFIG_IPROC_FA2)
drop_pkt:
#endif /* defined(CONFIG_IPROC_FA2) */
	PKTFRMNATIVE(etc->osh, skb);
	PKTFREE(etc->osh, skb, FALSE);

	return;
}

#ifdef HNDCTF
void
et_dump_ctf(et_info_t *et, struct bcmstrbuf *b)
{
	ctf_dump(et->cih, b);
}
#endif

void
et_dump(et_info_t *et, struct bcmstrbuf *b)
{
	bcm_bprintf(b, "et%d: %s %s version %s\n", et->etc->unit,
		__DATE__, __TIME__, EPI_VERSION_STR);

#ifdef HNDCTF
#if defined(BCMDBG)
	ctf_dump(et->cih, b);
#endif 
#endif /* HNDCTF */

	et_dumpet(et, b);
	etc_dump(et->etc, b);

	bcm_bprintf(b, "txdfrm(%d); txdfrmropped(%d); txqlen(%d)\n",
			et->etc->txfrm, et->etc->txfrmdropped, et->etc->txqlen);

#ifdef GMAC_RATE_LIMITING
	bcm_bprintf(b, "rxd_dropped_packets(%d)\n",
			et->etc->rl_dropped_packets);
#endif /* GMAC_RATE_LIMITING */

}

static void
et_dumpet(et_info_t *et, struct bcmstrbuf *b)
{
	bcm_bprintf(b, "et %p dev %p name %s tbusy %d txq[0].qlen %d malloced %d\n",
		et, et->dev, et->dev->name, (uint)netif_queue_stopped(et->dev), et->txq[0].qlen,
		MALLOCED(et->osh));
}

void
et_link_up(et_info_t *et)
{
	ET_ERROR(("et%d: link up (%d%s)\n",
		et->etc->unit, et->etc->speed, (et->etc->duplex? "FD" : "HD")));
	/* printf("et%d Link Up: %d%s\n", et->etc->unit, et->etc->speed, et->etc->duplex?"FD":"HD"); */
	netif_carrier_on(et->dev);
}

void
et_link_down(et_info_t *et)
{
	ET_ERROR(("et%d: link down\n", et->etc->unit));
	/* printf("et%d Link Down\n", et->etc->unit); */
	netif_carrier_off(et->dev);
}

int
et_enable_device( uint idx )
{
	ulong flags;
	uint coreidx, coreid;
	int rc = -1;

	spin_lock_irqsave(&sih_lock, flags);

	si_setcore(sih, GMAC_CORE_ID, idx);
	coreidx = si_coreidx(sih);
	coreid = si_coreid(sih);

	//printk(KERN_DEBUG "%s coreidx(0x%x) coreid(0x%x)\n", __FUNCTION__, coreidx, coreid);
	/* 2G_ENABLED: Enable IDM 250MHz for 2G mode */
/* #if 1 */
#if (defined(CONFIG_MACH_NS) || defined(CONFIG_MACH_NSP))
	si_core_reset(sih, 0x44, 0);
#else
	si_core_reset(sih, 0, 0);
#endif

	/* Initialize USBHC core OK */
	rc = 0;

	si_setcoreidx(sih, coreidx);
	spin_unlock_irqrestore(&sih_lock, flags);

	return rc;
}


/**********************************************************************
 *  bcm5301x_gmac_probe(device)
 *
 *  The Platform Driver Probe function.
 *
 *  Input parameters:
 *         device: The Device Context
 *
 *  Return value:
 *		    0: Driver Probe is Succesful
 *		not 0: ERROR
 **********************************************************************/
static int __init bcm5301x_gmac_probe(struct platform_device* pldev)
{
	struct net_device *dev		= NULL;
	void __iomem *macbase		= NULL;
	struct resource *memres		= NULL;
	struct resource *irqres		= NULL;
	osl_t *osh					= NULL;
	et_info_t *et				= NULL;
	int unit					= et_found;
	int err						= 0;
	char name[128];
	int i;
#if (defined(CONFIG_IPROC_FA) || defined(CONFIG_IPROC_FA2))
	char *var;
#endif /* (defined(CONFIG_IPROC_FA) || defined(CONFIG_IPROC_FA2)) */

	printk(KERN_DEBUG "%s enter name:%s; id:0x%x; unit:%d\n", __FUNCTION__, pldev->name, pldev->id, unit);

	/*Validation of platform device structure*/
	if (!pldev) {
		ET_ERROR(("WRONG INPUT\nplatfrom_device ppointer should not be NULL.\n"));
		return -EINVAL;
	}

	et_found++;

	/* pre-qualify et unit, that can save the effort to do et_detach */ 

	nvram_env_gmac_name(unit, name);
	if (getvar(NULL, name) == NULL) { 
		printk(KERN_DEBUG "et%d: %s not found, ignore it\n", unit, name);
		return -ENODEV; 
	}

	osh = osl_attach(pldev, PCI_BUS, FALSE);
	ASSERT(osh);

	/* Get global SB handle */
	sih = si_kattach(SI_OSH);

	/* reset core */
	et_enable_device(unit);

	ET_TRACE(("%s call alloc_etherdev\n", __FUNCTION__));
	if ((dev = alloc_etherdev(sizeof( et_info_t ))) == NULL) {
		ET_ERROR(("%s: alloc_etherdev() failed\n", __FUNCTION__));
		err = -ENOMEM;
		goto Exit;
	}

	et = ET_INFO(dev);
	bzero(et, sizeof(et_info_t));	/* Is this needed in 2.6.36 ? -LR */
	et->dev = dev;
	et->osh = osh;

	ET_TRACE(("%s get resources\n", __FUNCTION__));
	memres = iproc_platform_get_resource(pldev, IORESOURCE_MEM, 0);
	if (NULL == memres) {
		ET_ERROR(("ERROR: Could not get Platform Resource MAC Register Memory Resurce\n"));
		err = -ENOMEM;
		goto Exit;
	}

	if (!request_mem_region(memres->start, (memres->end - memres->start + 1), pldev->name)) {
		ET_ERROR(("ERROR: Could not request mem region. In file %s, LN:%d\n",
		       __FILE__, __LINE__));
		err = -ENOMEM;
		goto Exit;
	}
	irqres = iproc_platform_get_resource(pldev, IORESOURCE_IRQ, 0);
	if (NULL == irqres) {
		ET_ERROR(("ERROR: Could not get Platform Resource GMAC Register IRQ Resource\n"));
		err = -ENOMEM;
		goto Exit;
	}

	dev->base_addr = memres->start;
	dev->irq = irqres->start;

	printk(KERN_DEBUG "et%d: base_addr (0x%x) irq (%d)\n", unit, (uint32)dev->base_addr, dev->irq); 

//	if ((et->regsva = ioremap_nocache(dev->base_addr, PCI_BAR0_WINSZ)) == NULL) {
	if ((et->regsva = ioremap_nocache(dev->base_addr, 0xc00)) == NULL) {
		ET_ERROR(("et%d: ioremap() failed\n", unit));
		err = -ENOMEM;
		goto Exit;
	}
	ET_TRACE(("%s base_addr: 0x%x; regsva:0x%x\n", __FUNCTION__, (uint32)dev->base_addr, (uint32)et->regsva));

	pldev->id = dev->base_addr;
	dev_set_drvdata(&(pldev->dev), dev);
	SET_NETDEV_DEV(dev, (&pldev->dev));

	init_MUTEX(&et->sem);
	spin_lock_init(&et->lock);
	spin_lock_init(&et->txq_lock);
	spin_lock_init(&et->isr_lock);

	for (i = 0; i < NUMTXQ; i++)
		skb_queue_head_init(&et->txq[i]);

	/* common load-time initialization */
	et->etc = etc_attach((void *)et, VENDOR_BROADCOM, BCMIPROC_CHIP_ID, unit, osh, et->regsva);
	if (et->etc == NULL) {
		ET_ERROR(("et%d: etc_attach() failed\n", unit));
		err = -ENOMEM;
		goto Exit;
	}

#ifdef GMAC3
	et->fwdh = (fwder_t *)NULL;	/* attached/dettached on et up/dn */
	/* The ethernet network interface uses "eth0". Use fwd0, fwd1 instead */
	if (DEV_FWDER(et->etc))
		strncpy(dev->name, DEV_FWDER_NAME, 3);

#endif	/* GMAC3 */

#ifdef HNDCTF
	et->cih = ctf_attach(osh, dev->name, &et_msg_level, et_ctf_detach, et);

	if (ctf_dev_register(et->cih, dev, FALSE) != BCME_OK) {
		ET_ERROR(("et%d: ctf_dev_register() failed\n", unit));
		goto fail;
	}
#endif /* HNDCTF */

#ifdef CTFPOOL
	/* create ctf packet pool with specified number of buffers */
#ifdef CONFIG_BCM_IPROC_GMAC_SKB_RECYCLING
if ((osl_ctfpool_init(unit, osh, CTFPOOLSZ, RXBUFSZ+BCMEXTRAHDROOM) < 0)) {
#else
	if (CTF_ENAB(et->cih) && (num_physpages >= 8192) &&
	    (osl_ctfpool_init(osh, CTFPOOLSZ, RXBUFSZ+BCMEXTRAHDROOM) < 0)) {
#endif /* CONFIG_BCM_IPROC_GMAC_SKB_RECYCLING */
		ET_ERROR(("et%d: chipattach: ctfpool alloc/init failed\n", unit));
#ifdef CONFIG_BCM_IPROC_GMAC_SKB_RECYCLING
		goto Exit;
#else
		goto fail;
#endif /* CONFIG_BCM_IPROC_GMAC_SKB_RECYCLING */
    } else {
                printk(KERN_DEBUG "\net%d: chipattach: ctfpool alloc/init successful\n", unit);
}
#endif /* CTFPOOL */

	bcopy(&et->etc->cur_etheraddr, dev->dev_addr, ETHER_ADDR_LEN);

	/* init 1 second watchdog timer */
	init_timer(&et->timer);
	et->timer.data = (ulong)dev;
	et->timer.function = et_watchdog;

#ifdef GMAC_RATE_LIMITING
	/* init 1 second watchdog timer */
	init_timer(&et->etc->rl_timer);
	et->etc->rl_timer.data = (ulong)dev;
	et->etc->rl_timer.function = et_release_congestion;
#endif /* GMAC_RATE_LIMITING */

#ifdef	NAPI2_POLL
	netif_napi_add(dev, & et->napi_poll, et_poll, 64);
	napi_enable(&et->napi_poll);
#endif	/* NAPI2_POLL */

#if !defined(NAPI_POLL) && !defined(NAPI2_POLL)
	/* setup the bottom half handler */
	tasklet_init(&et->tasklet, et_dpc, (ulong)et);
#endif /* NAPI_POLL */

#ifdef ET_ALL_PASSIVE
	if (ET_ALL_PASSIVE_ENAB(et)) {
		MY_INIT_WORK(&et->dpc_task.work, (work_func_t)et_dpc_work);
		et->dpc_task.context = et;
		MY_INIT_WORK(&et->txq_task.work, (work_func_t)et_txq_work);
		et->txq_task.context = et;
	}
    if (et_ctf_pipeline_loopback(et)) {
        et->all_dispatch_mode = FALSE;
    } else {
		et->all_dispatch_mode = (passivemode == 0) ? TRUE : FALSE;
	}
#endif  /* ET_ALL_PASSIVE */

	ET_TRACE(("%s request irq\n", __FUNCTION__));
	/* register our interrupt handler */
	if (request_irq(dev->irq, et_isr, IRQF_SHARED, dev->name, et)) {
		ET_ERROR(("%s: request_irq(%d) failed\n", __FUNCTION__, dev->irq));
		err = -ENOMEM;
		goto Exit;
	}

	/* add us to the global linked list */
	et->next = et_list;
	et_list = et;

#ifndef HAVE_NET_DEVICE_OPS
	/* lastly, enable our entry points */
	dev->open = et_open;
	dev->stop = et_close;
	dev->hard_start_xmit = et_start;
	dev->get_stats = et_get_stats;
	dev->set_mac_address = et_set_mac_address;
	dev->set_multicast_list = et_set_multicast_list;
	dev->do_ioctl = et_ioctl;
#ifdef NAPI_POLL
	dev->poll = et_poll;
	dev->weight = (ET_GMAC(et->etc) ? 64 : 32);
#endif /* NAPI_POLL */
#else /* HAVE_NET_DEVICE_OPS */
	/* Linux 2.6.36 and up. - LR */
	dev->netdev_ops = &et_netdev_ops ;
#ifdef NAPI_POLL
	dev->poll = et_poll;
	dev->weight = (ET_GMAC(et->etc) ? 64 : 32);
#endif /* NAPI_POLL */

#endif /* !HAVE_NET_DEVICE_OPS */

#if (defined(CONFIG_IPROC_FA2) && defined(CONFIG_IPROC_FA2_CS_OFFLOAD))
	if (et->etc->unit == 2) {
		dev->features = (NETIF_F_IP_CSUM);
		//dev->features = (NETIF_F_IP_CSUM | NETIF_F_SG);
		//dev->hw_features = dev->features;
		dev->vlan_features = (NETIF_F_IP_CSUM);
		printk(KERN_DEBUG "\n          Enabling checksum offload ...\n");
	}
#endif /* CONFIG_IPROC_FA2 && CONFIG_IPROC_FA2_CS_OFFLOAD */

	ET_TRACE(("%s register netdev\n", __FUNCTION__));
	if (register_netdev(dev)) {
		ET_ERROR(("%s register_netdev() failed\n", __FUNCTION__));
		err = -ENOMEM;
		goto Exit;
	}

	/* print hello string */
	(*et->etc->chops->longname)(et->etc->ch, name, sizeof(name));
	printk(KERN_DEBUG "%s: %s %s\n", dev->name, name, EPI_VERSION_STR);

	eth_mac_proc_create(dev);	

#ifdef HNDCTF
	if (ctf_enable(et->cih, dev, TRUE, &et->brc_hot) != BCME_OK) {
		ET_ERROR(("et%d: ctf_enable() failed\n", unit));
		goto fail;
	}
#endif

#if (defined(CONFIG_IPROC_FA) || defined(CONFIG_IPROC_FA2))
	/* check if brcm tag is turned off */
	var = getvar(NULL, "brcmtag");
	if (var) {
		int tag = bcm_strtoul(var, NULL, 0);
		if (tag==0) {
			ET_ERROR(("BRCM TAG disabled\n"));
			brcm_tag = false;
		}
	}
	printk(KERN_DEBUG "BRCM TAG %sabled\n", brcm_tag?"en":"dis");
#endif /* (defined(CONFIG_IPROC_FA) || defined(CONFIG_IPROC_FA2)) */
	printk(KERN_DEBUG "et_ctf_active %sabled\n", et_ctf_active(et)?"en":"dis");

	ET_TRACE(("%s: exit\n", __FUNCTION__));

	return 0;

Exit:
	if (macbase) {
		iounmap(macbase);
		macbase=NULL;
	}
	if (memres) {
		release_mem_region(memres->start, (memres->end - memres->start + 1));
		memres=NULL;
	}
	if (dev) {
		free_netdev(dev);
		dev = NULL;
	}
	if (osh) {
		osl_detach(osh);
		osh=NULL;
	}
	if (et) {
		etc_detach(et->etc);
		et->dev = NULL;
		et->osh = NULL;
		et_free(et);
		et=NULL;
	}
	return err;
}


/**********************************************************************
 *  bcm5301x_gmac_remove(device)
 *
 *  The Removal of Platform Device, and un-initialize the previously
 *  added MAC, and it's MEM Regions and Resources.
 *
 *  Input parameters:
 *         device: The Device Context
 *
 *  Return value:
 *		    0: Driver Entry is Succesfull
 **********************************************************************/
static int __exit bcm5301x_gmac_remove(struct platform_device *pldev)
{
	struct net_device *dev = platform_get_drvdata(pldev);
	int retVal = 0;
	et_info_t *et				= NULL;
	struct resource *memres		= NULL;

	ET_TRACE(("%s: enter\n", __FUNCTION__));
	printk(KERN_DEBUG "%s: enter\n", __FUNCTION__);

#ifdef CONFIG_PM
	bcm5301x_gmac_drv_suspend(pldev, PMSG_SUSPEND);
#endif

	et = ET_INFO(dev);

	iounmap(et->regsva);
	unregister_netdev(dev);

	memres = iproc_platform_get_resource(pldev, IORESOURCE_MEM, 0);
	if (memres) {
		release_mem_region(memres->start, (memres->end - memres->start + 1));
	} else {
		ET_ERROR(("ERROR: Could not get Platform Resource GMAC Register Memory Resource\n"));
		retVal = -ENOMEM;
	}

	free_netdev(dev);

	et->dev = NULL;
	et_free(et);

	ET_TRACE(("%s: exit\n", __FUNCTION__));

	return retVal;
}

#ifdef CONFIG_PM
static int bcm5301x_gmac_drv_suspend(struct platform_device *pdev, pm_message_t state)
{
	int ret;
	char *filename = "/usr/sbin/ifdown";
	char *argv[] = {filename, "eth0", NULL};
	char *envp[] = {"HOME=/",
					 "TERM=linux",
					 "PATH=/sbin:/usr/sbin:/bin:/usr/bin",
					 NULL};

	ET_TRACE(("%s: enter\n", __FUNCTION__));
	printk(KERN_DEBUG "%s: enter\n", __FUNCTION__);
	ret = do_execve(filename, (const char * const*) argv, (const char * const*) envp);
	ET_TRACE(("%s: exit\n", __FUNCTION__));

	return 0;
}

static int bcm5301x_gmac_drv_resume(struct platform_device *pdev)
{
	int ret;
	char *filename = "/usr/sbin/ifup";
	char *argv[] = {filename, "eth0", NULL};
	char *envp[] = {"HOME=/",
					 "TERM=linux",
					 "PATH=/sbin:/usr/sbin:/bin:/usr/bin",
					 NULL};

	ET_TRACE(("%s: enter\n", __FUNCTION__));
	printk(KERN_DEBUG "%s: enter\n", __FUNCTION__);
	ret = do_execve(filename, (const char * const*) argv, (const char * const*) envp);
	ET_TRACE(("%s: exit\n", __FUNCTION__));

	return 0;
}
#else
#define bcm5301x_gmac_drv_suspend NULL
#define bcm5301x_gmac_drv_resume NULL
#endif

/**********************************************************************
 *  GMAC0 driver:
 * This structure defines the methods to be called by a bus driver 
 * during the lifecycle of a device on that bus.
**********************************************************************/
static struct platform_driver bcm5301x_gmac0_driver = 
{
	.probe = bcm5301x_gmac_probe,
	.remove = __exit_p(bcm5301x_gmac_remove),
	.suspend = bcm5301x_gmac_drv_suspend,
	.resume = bcm5301x_gmac_drv_resume,
	.driver =
	{
		.name = bcm5301x_gmac0_string,
	},
};

/**********************************************************************
 *  GMAC0 device:
 *  This structure defines the methods to be called by a platform device
 *  during the lifecycle of a device
**********************************************************************/
static u64 gmac0_dmamask = DMA_BIT_MASK(32);
static struct platform_device bcm5301x_gmac0_pdev = {
	.name = bcm5301x_gmac0_string,
	.id = 0,
	.dev =  {
		.release = bcm5301x_gmac_release,
		.init_name = bcm5301x_gmac0_string,
		.dma_mask = &gmac0_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.resource = bcm5301x_gmac0_resources,
	.num_resources = ARRAY_SIZE(bcm5301x_gmac0_resources),
};

/**********************************************************************
 *  GMAC1 driver:
 * This structure defines the methods to be called by a bus driver 
 * during the lifecycle of a device on that bus.
**********************************************************************/
static struct platform_driver bcm5301x_gmac1_driver = 
{
	.probe = bcm5301x_gmac_probe,
	.remove = __exit_p(bcm5301x_gmac_remove),
	.suspend = bcm5301x_gmac_drv_suspend,
	.resume = bcm5301x_gmac_drv_resume,
	.driver =
	{
		.name = bcm5301x_gmac1_string,
	},
};

/**********************************************************************
 *  GMAC1 device:
 *  This structure defines the methods to be called by a platform device
 *  during the lifecycle of a device
**********************************************************************/
static u64 gmac1_dmamask = DMA_BIT_MASK(32);
static struct platform_device bcm5301x_gmac1_pdev = {
	.name = bcm5301x_gmac1_string,
	.id = 0,
	.dev =  {
		.release = bcm5301x_gmac_release,
		.init_name = bcm5301x_gmac1_string,
		.dma_mask = &gmac1_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.resource = bcm5301x_gmac1_resources,
	.num_resources = ARRAY_SIZE(bcm5301x_gmac1_resources),
};

/**********************************************************************
 *  GMAC2 driver:
 * This structure defines the methods to be called by a bus driver 
 * during the lifecycle of a device on that bus.
**********************************************************************/
static struct platform_driver bcm5301x_gmac2_driver = 
{
	.probe = bcm5301x_gmac_probe,
	.remove = __exit_p(bcm5301x_gmac_remove),
	.suspend = bcm5301x_gmac_drv_suspend,
	.resume = bcm5301x_gmac_drv_resume,
	.driver =
	{
		.name = bcm5301x_gmac2_string,
	},
};

/**********************************************************************
 *  GMAC2 device:
 *  This structure defines the methods to be called by a platform device
 *  during the lifecycle of a device
**********************************************************************/
static u64 gmac2_dmamask = DMA_BIT_MASK(32);
static struct platform_device bcm5301x_gmac2_pdev = {
	.name = bcm5301x_gmac2_string,
	.id = 0,
	.dev =  {
		.release = bcm5301x_gmac_release,
		.init_name = bcm5301x_gmac2_string,
		.dma_mask = &gmac2_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.resource = bcm5301x_gmac2_resources,
	.num_resources = ARRAY_SIZE(bcm5301x_gmac2_resources),
};

/**********************************************************************
 *  GMAC3 driver:
 * This structure defines the methods to be called by a bus driver 
 * during the lifecycle of a device on that bus.
**********************************************************************/
static struct platform_driver bcm5301x_gmac3_driver = 
{
	.probe = bcm5301x_gmac_probe,
	.remove = __exit_p(bcm5301x_gmac_remove),
	.suspend = bcm5301x_gmac_drv_suspend,
	.resume = bcm5301x_gmac_drv_resume,
	.driver =
	{
		.name = bcm5301x_gmac3_string,
	},
};

/**********************************************************************
 *  GMAC3 device:
 *  This structure defines the methods to be called by a platform device
 *  during the lifecycle of a device
**********************************************************************/
static u64 gmac3_dmamask = DMA_BIT_MASK(32);
static struct platform_device bcm5301x_gmac3_pdev = {
	.name = bcm5301x_gmac3_string,
	.id = 0,
	.dev =  {
		.release = bcm5301x_gmac_release,
		.init_name = bcm5301x_gmac3_string,
		.dma_mask = &gmac3_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.resource = bcm5301x_gmac3_resources,
	.num_resources = ARRAY_SIZE(bcm5301x_gmac3_resources),
};

/**********************************************************************
 *   This function calls the device structure. 
 *   Input Parameter:	
 *		dev - pointer to the struct device
 **********************************************************************/

static void bcm5301x_gmac_release (struct device *dev) {}

/**********************************************************************
 *  bcm5301x_gmac_init_module(VOID)
 *
 *  The Driver Entry Function
 *
 *  Input parameters:
 *         None
 *
 *  Return value:
 *		    0: Driver Entry is Succesful
 *		not 0: ERROR
 **********************************************************************/
static int __init
bcm5301x_gmac_init_module(void)
{
	int err = -1;
	int deverr = -1;
	int idx;

	ET_TRACE(("%s: enter\n", __FUNCTION__));

    spin_lock_init(&sih_lock);

#if defined(BCMDBG)
	if (msglevel != 0xdeadbeef)
		et_msg_level = msglevel;
	else {
		char *var = getvar(NULL, "et_msglevel");
		if (var)
			et_msg_level = bcm_strtoul(var, NULL, 0);
	}

	printk(KERN_DEBUG "%s: msglevel set to 0x%x\n", __FUNCTION__, et_msg_level);
#endif /* defined(BCMDBG) */

#ifdef ET_ALL_PASSIVE
	{
		char *var = getvar(NULL, "et_dispatch_mode");
		if (var)
			passivemode = bcm_strtoul(var, NULL, 0);
		printk(KERN_DEBUG "%s: passivemode set to 0x%x\n", __FUNCTION__, passivemode);
	}
#endif /* ET_ALL_PASSIVE */
#ifdef	NAPI_POLL
	printk(KERN_DEBUG "%s: NAPI_POLL mode\n", __FUNCTION__);
#endif	/* NAPI_POLL */
#ifdef	NAPI2_POLL
	printk(KERN_DEBUG "%s: NAPI2_POLL mode\n", __FUNCTION__);
#endif	/* NAPI2_POLL */

#ifdef ET_LIMIT_TXQ
	{
		char *var = getvar(NULL, "et_txq_thresh");
		if (var)
			et_txq_thresh = bcm_strtoul(var, NULL, 0);
		printk(KERN_DEBUG "%s: et_txq_thresh set to 0x%x\n", __FUNCTION__, et_txq_thresh);
	}
#endif /* ET_LIMIT_TXQ */
#ifdef GMAC_RATE_LIMITING
	{
		char *var = getvar(NULL, "et_rx_rate_limit");
		if (var)
			et_rx_rate_limit = bcm_strtoul(var, NULL, 0);
		printk(KERN_DEBUG "%s: et_rx_rate_limit set to 0x%x\n", __FUNCTION__, et_rx_rate_limit);
	}
#endif /* GMAC_RATE_LIMITING */

#ifdef GMAC3
	fwder_init();
	/* fwder_dump_all(); */
#endif	/*  GMAC3 */

	/* keep track of which ones loaded */
	for (idx=0; idx<NS_MAX_GMAC_CORES; idx++)
		gmac_pdev_loaded[idx] = false;

	/* load GMAC0 driver */
	err = iproc_platform_driver_register(&bcm5301x_gmac0_driver);
	if (!err) {
		/* load GMAC0 device */
		err = iproc_platform_device_register(&bcm5301x_gmac0_pdev);
		if (err) {
			iproc_platform_driver_unregister(&bcm5301x_gmac0_driver);
			ET_ERROR(("ERROR module_init, could not iproc_platform_driver_register\n"));
			ET_ERROR(("Error Code = 0x%08x\n", err));
		} else {
			gmac_pdev_loaded[0] = true;
			deverr = 0;
			
		}
	} else {
		ET_ERROR(("ERROR module_init, could not iproc_platform_driver_register\n"));
		ET_ERROR(("Error Code = 0x%08x\n", err));
	}

	if (IPROC_NUM_GMACS>1) {
		/* load GMAC1 driver */
		err = iproc_platform_driver_register(&bcm5301x_gmac1_driver);
		if (!err) {
			/* load GMAC1 device */
			err = iproc_platform_device_register(&bcm5301x_gmac1_pdev);
			if (err) {
				iproc_platform_driver_unregister(&bcm5301x_gmac1_driver);
				ET_ERROR(("ERROR module_init, could not iproc_platform_driver_register\n"));
				ET_ERROR(("Error Code = 0x%08x\n", err));
			} else {
				gmac_pdev_loaded[1] = true;
				deverr = 0;
			}
		} else {
			ET_ERROR(("ERROR module_init, could not iproc_platform_driver_register\n"));
			ET_ERROR(("Error Code = 0x%08x\n", err));
		}
	}

	if (IPROC_NUM_GMACS>2) {
		/* load GMAC2 driver */
		err = iproc_platform_driver_register(&bcm5301x_gmac2_driver);
		if (!err) {
			/* load GMAC2 device */
			err = iproc_platform_device_register(&bcm5301x_gmac2_pdev);
			if (err) {
				iproc_platform_driver_unregister(&bcm5301x_gmac2_driver);
				ET_ERROR(("ERROR module_init, could not iproc_platform_driver_register\n"));
				ET_ERROR(("Error Code = 0x%08x\n", err));
			} else {
				gmac_pdev_loaded[2] = true;
				deverr = 0;
			}
		} else {
			ET_ERROR(("ERROR module_init, could not iproc_platform_driver_register\n"));
			ET_ERROR(("Error Code = 0x%08x\n", err));
		}
	}

	if (IPROC_NUM_GMACS>3) {
		/* load GMAC3 driver */
		err = iproc_platform_driver_register(&bcm5301x_gmac3_driver);
		if (!err) {
			/* load GMAC3 device */
			err = iproc_platform_device_register(&bcm5301x_gmac3_pdev);
			if (err) {
				iproc_platform_driver_unregister(&bcm5301x_gmac3_driver);
				ET_ERROR(("ERROR module_init, could not iproc_platform_driver_register\n"));
				ET_ERROR(("Error Code = 0x%08x\n", err));
			} else {
				gmac_pdev_loaded[3] = true;
				deverr = 0;
			}
		} else {
			ET_ERROR(("ERROR module_init, could not iproc_platform_driver_register\n"));
			ET_ERROR(("Error Code = 0x%08x\n", err));
		}
	}

	ET_TRACE(("%s: exit\n", __FUNCTION__));
	return deverr;
}

/**********************************************************************
 *  bcm5301x_gmac_cleanup_module(VOID)
 *
 *  The Driver Exit Function
 *
 *  Input parameters:
 *         None
 *
 *  Return value:
 *		    Nothing
 **********************************************************************/
static void __exit
bcm5301x_gmac_cleanup_module(void)
{
	ET_TRACE(("%s: enter\n", __FUNCTION__));
	printk(KERN_DEBUG "%s: enter\n", __FUNCTION__);

	if ( gmac_pdev_loaded[0] ) {
		/* unregister device */
		iproc_platform_device_unregister(&bcm5301x_gmac0_pdev);
		/* Unregister the driver*/
		iproc_platform_driver_unregister(&bcm5301x_gmac0_driver);
	}

	if (IPROC_NUM_GMACS>1) {
		if ( gmac_pdev_loaded[1] ) {
			/* unregister device */
			iproc_platform_device_unregister(&bcm5301x_gmac1_pdev);
			/* Unregister the driver*/
			iproc_platform_driver_unregister(&bcm5301x_gmac1_driver);
		}
	}

	if (IPROC_NUM_GMACS>2) {
		if ( gmac_pdev_loaded[2] ) {
			/* unregister device */
			iproc_platform_device_unregister(&bcm5301x_gmac2_pdev);
			/* Unregister the driver*/
			iproc_platform_driver_unregister(&bcm5301x_gmac2_driver);
		}
	}

	if (IPROC_NUM_GMACS>3) {
		if ( gmac_pdev_loaded[3] ) {
			/* unregister device */
			iproc_platform_device_unregister(&bcm5301x_gmac3_pdev);
			/* Unregister the driver*/
			iproc_platform_driver_unregister(&bcm5301x_gmac3_driver);
		}
	}

	//clean up the proc directory
    eth_mac_proc_remove();

	ET_TRACE(("%s: exit\n", __FUNCTION__));
	return;
}

static ssize_t get_fa_bypass (struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
	static const char message[60];
	sprintf(message, "\n\n## Current FA Bypass setting = 0x%x, %s ##\n\n",gBypass, gBypass?"enabled":"disabled");
	return simple_read_from_buffer(buf, size, ppos, message, sizeof(message));
}

static ssize_t set_fa_bypass(struct file *file, const char *buffer, unsigned long count, loff_t *ppos)
{
	unsigned int 	len=1;
	unsigned char 	debug_buffer[2];
	int		bypass =0;

	//printk(KERN_DEBUG "count %x ## \n\n",(unsigned int) count);
	if (count != 2)
	{
		ET_ERROR(("Please pass (one:1) digit FA bypass value only, 0=disable FA bypass, 1 = enable FA bypass\n"));
		return -EINVAL;
	}

	// Last buffer byte will be LF or CR only
	if(copy_from_user(&debug_buffer[0], buffer, len))
	{
		ET_ERROR(("Problem in copying invalid user buffer\n"));
		return -EFAULT;
	}

	debug_buffer[len]='\0'; // Only one byte value is available now
	if ( sscanf(debug_buffer,"%d",&bypass) != 1)
	{
		ET_ERROR(("\n##Invalid value :%s: is passed ##\n",debug_buffer));
		return -EINVAL;
	}
	if (!((bypass >=DISABLE_FA_BYPASS) && (bypass <= ENABLE_FA_BYPASS)))
	{
		ET_ERROR(("\n##Passed value :%d: is not in valid range %d-%d \n",bypass,DISABLE_FA_BYPASS,ENABLE_FA_BYPASS));
		return -EINVAL;
	}
	ET_TRACE(("\n##set_fa_bypass(): Previous:  0x%x %s ##\n", gBypass, gBypass?"enabled":"disabled"));
	gBypass = bypass;
	ET_TRACE(("\n##set_fa_bypass(): New:  0x%x %s ##\n", gBypass, gBypass?"enabled":"disabled"));
	return count;
}


static char* bcm5301x_eth_proc_root="bcm5301x_eth";
static struct proc_dir_entry *bcm5301x_eth_root_dir ; // BCM5892  eth proc root directory

static const struct file_operations iproc_fops = {
   .read = get_fa_bypass,
   .write = set_fa_bypass,
   .llseek = default_llseek,
};

static int eth_mac_proc_create(struct net_device *dev )
{
	struct proc_dir_entry *dent, *ent;
	et_info_t *et;
	etc_info_t *etc;
    char fname[32];

	et = ET_INFO(dev);
    if (et != NULL) {
	    etc = et->etc;
    }

    if ((et == NULL) || (etc == NULL)) {
	   printk(KERN_DEBUG "%s: error: Unit probably not initialized by probe function."
              " et=0x%pm etc=0x%p\n", __FUNCTION__, et, etc);
        return -1;
   }

	ET_TRACE(("%s: enter\n", __FUNCTION__));
	printk(KERN_DEBUG "%s: enter\n", __FUNCTION__);

    snprintf(fname, 32, "%s%u", bcm5301x_eth_proc_root, etc->unit);

	dent =	proc_mkdir(fname,bcm5301x_eth_root_dir);
	if (dent) {
        /* unit 2 has FA connectivity, create bypass path only for unit 2 */
        if (etc->unit == 2) {
	        printk(KERN_DEBUG "\nCreating fa bypass proc entry\n");

		    ent = proc_create("fa_bypass", S_IFREG|S_IRUGO, dent, &iproc_fops);
		    if (!ent) {
	            	printk(KERN_DEBUG "Error creating proc_entry, returning\n");
                	return -1;
		    }
        }
	}
	ET_TRACE(("%s: exit\n", __FUNCTION__));
	return 0; 
}

static void eth_mac_proc_remove(void)
{
	ET_TRACE(("%s: enter\n", __FUNCTION__));
	printk(KERN_DEBUG "%s: enter\n", __FUNCTION__);
	remove_proc_entry(bcm5301x_eth_proc_root,NULL);
	ET_TRACE(("%s: exit\n", __FUNCTION__));
} 


#if defined(CONFIG_IPROC_FA2)
int et_fa2_spu_tx(struct sk_buff *skb)
{
	struct net_device *dev = platform_get_drvdata(&bcm5301x_gmac3_pdev);
	return et_start(skb, dev);
}

#if defined(CONFIG_IPROC_FA2_CS_OFFLOAD)
static et_info_t *et_get_eth3_info()
{
	et_info_t *et;
	struct net_device *d = platform_get_drvdata(&bcm5301x_gmac3_pdev);

	et = ET_INFO(d);

	if (et == NULL) {
		printk(KERN_INFO "\net for dev3 is NULL, using dev2\n");
		 // NOTEet = ET_INFO(dev);
        
    }

	return(et);
}
#endif /* CONFIG_IPROC_FA2_CS_OFFLOAD */
#endif /* CONFIG_IPROC_FA2 */


module_init(bcm5301x_gmac_init_module);
module_exit(bcm5301x_gmac_cleanup_module);

MODULE_DESCRIPTION("Broadcom Northstar Ethernet Driver");
MODULE_LICENSE("GPL");
