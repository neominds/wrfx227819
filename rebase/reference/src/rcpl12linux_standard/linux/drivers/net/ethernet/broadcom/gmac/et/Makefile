#
# Makefile for the Broadcom et driver
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
# $Id: Makefile,v 1.5 2010-12-07 04:47:36 $
#


ETSRCDIR = ../src/et

et-objs = $(addprefix $(ETSRCDIR)/sys/, et_linux.o etc.o)

EXTRA_CFLAGS += -DGMAC_RATE_LIMITING -DBCMDRIVER

ifeq ($(CONFIG_ET_47XX),y)
et-objs		+= $(ETSRCDIR)/sys/etcgmac.o
EXTRA_CFLAGS	+= -DCFG_GMAC -DBCMDMA64 -DBCMDMA32  -DBCMDBG_ERR
ifeq ($(CONFIG_ROOT_NFS),y)
else
ifeq ($(CONFIG_MACH_NS),y)
EXTRA_CFLAGS	+= -DETROBO
endif
endif
ifeq ($(CONFIG_MACH_NSP),y)
EXTRA_CFLAGS	+= -DETROBO
endif
endif

obj-$(CONFIG_ET) := et.o

EXTRA_CFLAGS	+= -DDMA -Wno-error
ifeq ($(CONFIG_ET_ALL_PASSIVE_ON),y)
EXTRA_CFLAGS	+= -DET_ALL_PASSIVE
else
ifeq ($(CONFIG_ET_ALL_PASSIVE_RUNTIME),y)
EXTRA_CFLAGS	+= -DET_ALL_PASSIVE
else
ifeq ($(CONFIG_ET_NAPI_POLL),y)
EXTRA_CFLAGS	+= -DNAPI_POLL
else
ifeq ($(CONFIG_ET_NAPI2_POLL),y)
EXTRA_CFLAGS	+= -DNAPI2_POLL
endif
endif
endif
endif

EXTRA_CFLAGS	+= -Idrivers/net/ethernet/broadcom/gmac/src/et/sys
EXTRA_CFLAGS    += -Idrivers/net/ethernet/broadcom/gmac/src/include
