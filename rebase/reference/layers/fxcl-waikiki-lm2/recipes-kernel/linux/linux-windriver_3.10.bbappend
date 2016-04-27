#
# Copyright (C) 2014 Wind River Systems, Inc.
#

include linux-windriver-fxcl-waikiki-lm2.inc

#KMACHINE_arm-coretile-express-a15 = "fxcl-waikiki-lm2"

FILESEXTRAPATHS_prepend := "${THISDIR}/files:"
SRC_URI_append += "file://fxcl-waikiki-lm2-standard.scc"
