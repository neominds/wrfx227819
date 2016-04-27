#
# Makefile for Broadcom Home Networking Division (HND) shared driver code
#
# Copyright (C) 2013, Broadcom Corporation. All Rights Reserved.
# 
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
# 
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
# SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
# OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
# CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
# $Id: Makefile,v 1.5 2008-05-02 22:49:54 $
#

SHARED   = ../src/shared


EXTRA_CFLAGS 	+= -DBCM5354 -DLINUX_HYBRID_POSTPROCESSING_REMOVAL -DBCMDBG_ERR -DBCMDRIVER

include drivers/net/ethernet/broadcom/gmac/src/shared/wl_config

### for bcm5301x_otp.h
EXTRA_CFLAGS += -Idrivers/net/ethernet/broadcom/gmac/otp/

ifneq ($(CONFIG_BCM947XX),y)
hnd-objs += $(SHARED)/nvramstubs.o
endif

ifeq ($(CONFIG_MACH_HX4),y)
hnd-objs += $(SHARED)/bcmiproc_serdes.o
hnd-objs += $(SHARED)/bcmiproc_phy5461s.o
endif
ifeq ($(CONFIG_MACH_KT2),y)
hnd-objs += $(SHARED)/bcmiproc_serdes.o
hnd-objs += $(SHARED)/bcmiproc_phy5461s.o
endif

ifeq ($(CONFIG_MACH_HR2),y)
hnd-objs += $(SHARED)/bcmiproc_phy5221.o
endif

ifeq ($(CONFIG_MACH_NSP),y)
hnd-objs += $(SHARED)/bcmiproc_robo_serdes.o
endif

ifeq ($(CONFIG_ET_47XX),y)
hnd-objs += $(SHARED)/bcmrobo.o
endif

#ifdef HNDDMA
ifeq ($(HNDDMA),1)
hnd-objs += $(SHARED)/hnddma.o
endif
#endif

#ifdef BCMUTILS
ifeq ($(BCMUTILS),1)
hnd-objs += $(SHARED)/bcmutils.o
endif
#endif

#ifdef SIUTILS
ifeq ($(SIUTILS),1)
hnd-objs += $(SHARED)/siutils.o
hnd-objs += $(SHARED)/aiutils.o
ifeq ($(CONFIG_MACH_HX4),y)
hnd-objs += $(SHARED)/hx4_erom.o
endif
ifeq ($(CONFIG_MACH_HR2),y)
hnd-objs += $(SHARED)/hr2_erom.o
endif
ifeq ($(CONFIG_MACH_NSP),y)
hnd-objs += $(SHARED)/nsp_erom.o
endif
ifeq ($(CONFIG_MACH_KT2),y)
hnd-objs += $(SHARED)/kt2_erom.o
endif
endif
#endif /* SIUTILS */

#ifdef BCMSROM
ifeq ($(BCMSROM),1)
hnd-objs += $(SHARED)/bcmsrom.o
hnd-objs += $(SHARED)/bcmotp.o
endif
#endif

#ifdef BCMOTP
ifeq ($(BCMOTP),1)
ifneq ($(BCMSROM),1)
hnd-objs += $(SHARED)/bcmotp.o
endif
EXTRA_CFLAGS += -DBCMNVRAMR
endif
#endif

#ifdef OSLLX
# linux osl
ifeq ($(OSLLX),1)
hnd-objs += $(SHARED)/linux_osl.o
endif
#endif

# BCMDMA32
ifeq ($(BCMDMA32),1)
EXTRA_CFLAGS += -DBCMDMA32
endif

obj-$(CONFIG_HND) := hnd.o

ifeq ($(wildcard $(SHARED)/bcmutils.c),)
$(SHARED)/bcmutils.o: $(SHARED)/linux/bcmutils.o
	cp $< $@
endif

ifeq ($(wildcard $(SHARED)/bcmrobo.c),)
$(SHARED)/bcmrobo.o: $(SHARED)/linux/bcmrobo.o
	cp $< $@
endif

ifeq ($(wildcard $(SHARED)/hnddma.c),)
$(SHARED)/hnddma.o: $(SHARED)/linux/hnddma.o
	cp $< $@
endif

ifeq ($(wildcard $(SHARED)/linux_osl.c),)
$(SHARED)/linux_osl.o: $(SHARED)/linux/linux_osl.o
	cp $< $@
endif

ifeq ($(wildcard $(SHARED)/nvramstubs.c),)
$(SHARED)/nvramstubs.o: $(SHARED)/linux/nvramstubs.o
	cp $< $@
endif

ifeq ($(wildcard $(SHARED)/bcmsrom.c),)
$(SHARED)/bcmsrom.o: $(SHARED)/linux/bcmsrom.o
	cp $< $@
endif

ifeq ($(wildcard $(SHARED)/bcmotp.c),)
$(SHARED)/bcmotp.o: $(SHARED)/linux/bcmotp.o
	cp $< $@
endif

ifeq ($(wildcard $(SHARED)/bcm5301x_otp.c),)
$(SHARED)/bcmotp.o: $(SHARED)/linux/bcm5301x_otp.o
	cp $< $@
endif

EXTRA_CFLAGS    += -Idrivers/net/ethernet/broadcom/gmac/src/et/sys
EXTRA_CFLAGS    += -Idrivers/net/ethernet/broadcom/gmac/src/include
