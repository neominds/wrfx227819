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
 * HND GMAC Forwarder
 * When multiple ports from a switch to a dual core host processor are available
 * each Wl MAC instance (wl0 and wl1) may be tied to each core. A forwarding
 * driver would recieve/transmit packets from/to switch on behalf of the Wl
 * driver per core, essentially translating each core of the host processor to
 * be a WL MAC that is becomes just another "W"-LAN port on the switch.
 *
 * hndfwd.h provides the API for the Fwder network device driver and wl driver
 * to register their respective transmit handlers as a per cpu object.
 *
 * HND Fwder in SMP assumes "dual" core:
 *		Core1: wl0=11ac
 *		Core0: wl1=11n + eth0
 * HND Fwder in Uniprocessor mode would use 2 units.
 *
 * $Id$
 */

#ifndef _hndfwd_h_
#define _hndfwd_h_

#if defined(GMAC3)

/*
 * Multiple GMAC to Host SMP Model: The ethernet driver operates as a
 *  FWD: Forwarder to MAC Interface such as wl0, wl1
 *  NTK: Ethernet Network Interface binding to network stack (via CTF)
 */
#define DEV_FWDER_NAME			"fwd"
#define DEV_NTKIF(etc)			((etc)->gmac_fwd == FALSE)
#define DEV_FWDER(etc)			((etc)->gmac_fwd == TRUE)

#if defined(CONFIG_SMP)
#define _FWDER_LOCK(fwder)		spin_lock_bh(&(fwder)->lock)
#define _FWDER_UNLOCK(fwder)	spin_unlock_bh(&(fwder)->lock)
#else	/* !CONFIG_SMP */
#define _FWDER_LOCK(fwder)		local_bh_disable()
#define _FWDER_UNLOCK(fwder)	local_bh_enable()
#endif	/* !CONFIG_SMP */

#define FWDER_FAILURE		1
#define FWDER_SUCCESS		0

#define FWDER_DEBUG
#define FWDER_MAX_UNITS		2

/* hard start xmit handler of a network device driver */
typedef int (* fwder_start_t)(struct sk_buff * buff, struct net_device * dev,
                              int cnt);

typedef enum fwder_dir {
    FWD_UPSTREAM,			/* WL##(RX) -> GMAC(TX) Start Xmit Handler */
    FWD_DNSTREAM,			/* GMAC(RX) -> Wl##(TX) Start Xmit Handler */
    FWD_MAX_DIRS
} fwder_dir_t;

typedef struct fwder {
    struct net_device * dev;
    fwder_start_t forward;
	int unit;
	unsigned int error;
#if defined(CONFIG_SMP)
	spinlock_t lock;
#endif	/*  CONFIG_SMP */
} fwder_t;

#if defined(CONFIG_SMP)
DECLARE_PER_CPU(struct fwder, fwd_upstream);    /* Per Core GMAC Transmit */
DECLARE_PER_CPU(struct fwder, fwd_dnstream);    /* Per Core Wl## Transmit */
#else	/* !CONFIG_SMP */
extern struct fwder fwd_upstream[FWDER_MAX_UNITS];
extern struct fwder fwd_dnstream[FWDER_MAX_UNITS];
#endif	/* !CONFIG_SMP */

extern int fwder_init(void); /* Invoked in eth0 module_init */

/* Register a transmit handler and return the reverse dir handler */
extern struct fwder * fwder_attach(fwder_start_t forward,
	const struct net_device * dev, const int unit, const enum fwder_dir dir);

/* Deregister a transmit handler */
extern struct fwder * fwder_dettach(struct fwder * fwder_p);

extern void fwder_dump(const struct fwder * fwder_p);
extern void fwder_dump_all(void);

static inline int
fwder_transmit(struct sk_buff * skb, struct net_device * dev,
	           struct fwder * fwder_p, int cnt)
{
	int ret;

	ASSERT(fwder_p != (struct fwder*)NULL);

	_FWDER_LOCK(fwder_p);								/* ++LOCK */

	if (dev == (struct net_device *)NULL) {
		skb->dev = (struct net_device *)fwder_p->dev;
	}

	ret = fwder_p->forward(skb, skb->dev, cnt);

#if defined(FWDER_DEBUG)
	if (ret == FWDER_FAILURE)
		fwder_p->error++;
#endif

	_FWDER_UNLOCK(fwder_p);							/* --LOCK */

	return ret;
}

#else	/* !GMAC3 */

#define DEV_FWDER_NAME       "eth"
#define DEV_NTKIF(etc)       1
#define DEV_FWDER(etc)       0

#endif	/* !GMAC3 */

#endif  /* _hndfwd_h_ */
