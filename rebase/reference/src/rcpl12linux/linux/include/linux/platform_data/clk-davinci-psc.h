/*
 *  DaVinci Power & Sleep Controller (PSC) clk driver platform data
 *
 *  Copyright (C) 2006-2012 Texas Instruments.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */
#ifndef __CLK_DAVINCI_PSC_H
#define __CLK_DAVINCI_PSC_H

/* PSC flags */
/* Disable state is SwRstDisable */
#define CLK_DAVINCI_PSC_SWRSTDISABLE	BIT(0)
/* Force module state transtition */
#define CLK_DAVINCI_PSC_FORCE		BIT(1)
/* PSC has external power control available (for DM6446 SoC) */
#define CLK_DAVINCI_PSC_HAS_EXT_POWER_CNTL	BIT(2)
struct clk_davinci_psc_data {
	/* base address of the PSC */
	void __iomem *base;
	/* framework flags */
	u32	flags;
	/* psc specific flags */
	u32	psc_flags;
	u8	lpsc;
	u8	gpsc;
	u8	domain;
};

extern struct clk *clk_register_davinci_psc(struct device *dev,
			const char *name, const char *parent_name,
			struct clk_davinci_psc_data *psc_data,
			spinlock_t *lock);

#ifdef CONFIG_OF
extern void __init of_davinci_psc_clk_init(struct device_node *node,
			 spinlock_t *lock);
#endif
#endif /* __CLK_DAVINCI_PSC_H */
