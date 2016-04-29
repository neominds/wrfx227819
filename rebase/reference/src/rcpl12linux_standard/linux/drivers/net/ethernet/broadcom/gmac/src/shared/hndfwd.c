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
 *
 * $Id$
 */

#if defined(GMAC3)

#include <linux/skbuff.h>
#include <osl.h>
#include <hndfwd.h>

#if defined(CONFIG_SMP)
#define _FWDER_GET(fwder_instance, unit)	&per_cpu(fwder_instance, unit)
#else	/* !CONFIG_SMP */
#define _FWDER_GET(fwder_instance, unit)	&fwder_instance[unit]
#endif	/* !CONFIG_SMP */


static int /* default dummy xmit handler when tx device is down */
_fwder_default(struct sk_buff * skb, struct net_device * dev, int cnt)
{
	return FWDER_FAILURE;
}

#if defined(CONFIG_SMP)
DEFINE_PER_CPU(struct fwder, fwd_upstream) = {
	.lock = __SPIN_LOCK_UNLOCKED(.lock),	/* static init */
	.dev = (struct net_device *)NULL,
	.forward = (fwder_start_t)NULL,
	.error = 0U
};

DEFINE_PER_CPU(struct fwder, fwd_dnstream) = {
	.lock = __SPIN_LOCK_UNLOCKED(.lock),
	.dev = (struct net_device *)NULL,
	.forward = (fwder_start_t)NULL,
	.error = 0U
};
#else	/* !CONFIG_SMP */
struct fwder fwd_upstream[FWDER_MAX_UNITS] = {
	{
		.dev = (struct net_device *)NULL,
		.forward = (fwder_start_t)NULL,
		.unit = 0,
		.error = 0U
	},
	{
		.dev = (struct net_device *)NULL,
		.forward = (fwder_start_t)NULL,
		.unit = 1,
		.error = 0U
	}
};
struct fwder fwd_dnstream[FWDER_MAX_UNITS] = {
	{
		.dev = (struct net_device *)NULL,
		.forward = (fwder_start_t)NULL,
		.unit = 0,
		.error = 0U
	},
	{
		.dev = (struct net_device *)NULL,
		.forward = (fwder_start_t)NULL,
		.unit = 1,
		.error = 0U
	}
};
#endif	/* !CONFIG_SMP */

static inline
struct fwder * _get_other(struct fwder * fwder_p)
{
	struct fwder * other_p;

	other_p = _FWDER_GET(fwd_upstream, fwder_p->unit);
	if (other_p == fwder_p)
		return _FWDER_GET(fwd_dnstream, fwder_p->unit);
	else
		return other_p;
}

int	/* Initialization of fwder in et_module_init */
fwder_init(void)
{
	int dir;
	fwder_t * fwder_p;
	int unit;

#if defined(CONFIG_SMP)
	for_each_online_cpu(unit)
#else	/* !CONFIG_SMP */
	for (unit = 0; unit < FWDER_MAX_UNITS; unit++)
#endif	/* !CONFIG_SMP */
	{
		for (dir = (int)FWD_UPSTREAM; dir < (int)FWD_MAX_DIRS; dir++) {
			if (dir == (int)FWD_UPSTREAM)
				fwder_p = _FWDER_GET(fwd_upstream, unit);
			else
				fwder_p = _FWDER_GET(fwd_dnstream, unit);
			fwder_p->dev = (struct net_device *)fwder_p;
			fwder_p->forward = (fwder_start_t)_fwder_default;
			fwder_p->unit = unit;
			fwder_p->error = 0U;
		}

	}	/* for_each_online_cpu / for unit = 0 .. FWDER_MAX_UNITS */

printk(KERN_DEBUG "===FWD: fwder_init\n");	// DELETE ME
	return 0;
}

struct fwder * /* Driver registers its xx_start_xmit() handler on netdev open */
fwder_attach(fwder_start_t forward, const struct net_device * dev,
	         const int unit, const enum fwder_dir dir)
{
	fwder_t * fwder_p;
	fwder_t * other_p;	/* reverse direction forwarder */

	printk(KERN_DEBUG "fwder_attach forward<%p> dev<%p> unit<%d> dir<%d>\n",
		forward, dev, unit, dir);

	ASSERT((dir < (int)FWD_MAX_DIRS) && (dev != (struct net_device *)NULL)
		   && (forward != (fwder_start_t)NULL));

#if defined(CONFIG_SMP)
	ASSERT(unit < NR_CPUS);
#else	/* !CONFIG_SMP */
	ASSERT(unit < FWDER_MAX_UNITS);
#endif	/* !CONFIG_SMP */

	if (dir == (int)FWD_UPSTREAM) {
		fwder_p = _FWDER_GET(fwd_upstream, unit);
		other_p = _FWDER_GET(fwd_dnstream, unit);
	} else {
		fwder_p = _FWDER_GET(fwd_dnstream, unit);
		other_p = _FWDER_GET(fwd_upstream, unit);
	}

	_FWDER_LOCK(fwder_p);									/* ++LOCK */

	fwder_p->dev = (struct net_device *)dev;
	fwder_p->forward = forward;

	_FWDER_UNLOCK(fwder_p);									/* ++LOCK */

printk(KERN_DEBUG "===FWD: ATTACH<%d,%d>: <%p> unit<%d> dir<%d> dev<%p> xmit<%p>\n", unit, (int)dir, fwder_p, fwder_p->unit, dir, fwder_p->dev, fwder_p->forward);	// DELETE ME
printk(KERN_DEBUG "===FWD: RETURN<%d,%d>: <%p> unit<%d> dir<%d> dev<%p> xmit<%p>\n", unit, (int)dir, other_p, other_p->unit, dir^1, other_p->dev, other_p->forward); // DELETE ME

	return other_p;
}

struct fwder * /* A driver deregisters itself on netdev close */
fwder_dettach(struct fwder * fwder_p)
{

	if (fwder_p == (fwder_t *)NULL)
		return (fwder_t *)NULL;

	printk(KERN_DEBUG "fwder_dettach <%p>\n", fwder_p);

	fwder_p = _get_other(fwder_p);

	_FWDER_LOCK(fwder_p);									/* ++LOCK */

	fwder_p->dev = (struct net_device *)NULL;
	fwder_p->forward = (fwder_start_t)_fwder_default;

	_FWDER_UNLOCK(fwder_p);									/* ++LOCK */

	return (fwder_t *)NULL;
}

void
fwder_dump(const struct fwder * fwder_p)
{
	if (fwder_p == (fwder_t *)NULL)
		return;

	printk(KERN_DEBUG "FWD<%p>: dev<%p> forward<%p> unit<%d> error<%u>\n",
		fwder_p, fwder_p->dev, fwder_p->forward,
		fwder_p->unit, fwder_p->error);
}

void
fwder_dump_all(void)
{
	int unit;
	struct fwder * fwder_p;

	printk(KERN_DEBUG "FWDER DUMP ALL default<%p>\n", _fwder_default);
#if defined(CONFIG_SMP)
	for_each_online_cpu(unit)
#else	/* !CONFIG_SMP */
	for (unit = 0; unit < FWDER_MAX_UNITS; unit++)
#endif	/* !CONFIG_SMP */
	{
		fwder_p = _FWDER_GET(fwd_upstream, unit);
		printk(KERN_DEBUG "FWD[UP]<%p> CPU<%d>: dev<%p> forward<%p> unit<%d> error<%u>\n",
			fwder_p, unit, fwder_p->dev, fwder_p->forward,
			fwder_p->unit, fwder_p->error);

		fwder_p = _FWDER_GET(fwd_dnstream, unit);
		printk(KERN_DEBUG "FWD[DN]<%p> CPU<%d>: dev<%p> forward<%p> unit<%d> error<%u>\n",
			fwder_p, unit, fwder_p->dev, fwder_p->forward,
			fwder_p->unit, fwder_p->error);

	}	/* for_each_online_cpu / for unit = 0 .. FWDER_MAX_UNITS */
}

#endif  /*  GMAC3 */
