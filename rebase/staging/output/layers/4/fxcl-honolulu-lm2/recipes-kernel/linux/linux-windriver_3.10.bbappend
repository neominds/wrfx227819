#
# Copyright (C) 2014 Wind River Systems, Inc.
#

include linux-windriver-fxcl-honolulu-lm2.inc

#KMACHINE_arm-coretile-express-a15 = "fxcl-honolulu-lm2"

FILESEXTRAPATHS_prepend := "${THISDIR}/files:"
SRC_URI_append += "file://fxcl-honolulu-lm2-standard.scc"
