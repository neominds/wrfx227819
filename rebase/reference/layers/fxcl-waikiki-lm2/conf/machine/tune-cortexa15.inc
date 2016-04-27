#
# Copyright (C) 2013 Wind River Systems, Inc.
#
# Refer to tune-cortexa15.inc of yocto's meta-ti and tune-cortexa9.inc of windriver
DEFAULTTUNE ?= "cortexa15t-neon"

# WR-specific:  This file radically changes the CCARGS values.
require conf/machine/include/tune-armv7at.inc

TUNEVALID[cortexa15] = "Enable Cortex-A15 specific processor optimizations"

TUNEABI_CCARGS_tune_cortexa15t-nvfp = "${TUNEABI_CCARGS_tune_armv7at-nvfp}"
TUNEABI_CCARGS_tune_cortexa15t = "${TUNEABI_CCARGS_tune_armv7at}"
TUNEABI_CCARGS_tune_cortexa15t-neon = "${TUNEABI_CCARGS_tune_armv7at-neon}"

TUNE_CCARGS += "${@bb.utils.contains("TUNE_FEATURES", "cortexa15", "-mtune=cortex-a15", "", d)}"

# Little Endian base configs
AVAILTUNES += "cortexa15t-nvfp cortexa15 cortexa15t cortexa15t-neon"
TUNE_FEATURES_tune-cortexa15t-nvfp = "${TUNE_FEATURES_tune-armv7at-nvfp} cortexa15"
TUNE_FEATURES_tune-cortexa15 = "${TUNE_FEATURES_tune-armv7a} cortexa15"
TUNE_FEATURES_tune-cortexa15t ?= "${TUNE_FEATURES_tune-armv7at} cortexa15"
TUNE_FEATURES_tune-cortexa15t-neon ?= "${TUNE_FEATURES_tune-armv7at-neon} cortexa15"

TUNEABI_tune-cortexa15t-nvfp = "armv7at-nvfp"
TUNEABI_tune-cortexa15t ?= "armv7at"
TUNEABI_tune-cortexa15 ?= "armv7a"
TUNEABI_tune-cortexa15t-neon ?= "armv7at-neon"

PACKAGE_EXTRA_ARCHS_tune-cortexa15t-nvfp = "${PACKAGE_EXTRA_ARCHS_tune-armv7at-nvfp}"
PACKAGE_EXTRA_ARCHS_tune-cortexa15 = "${PACKAGE_EXTRA_ARCHS_tune-armv7a}"
PACKAGE_EXTRA_ARCHS_tune-cortexa15t = "${PACKAGE_EXTRA_ARCHS_tune-armv7at}"
PACKAGE_EXTRA_ARCHS_tune-cortexa15t-neon = "${PACKAGE_EXTRA_ARCHS_tune-armv7at-neon}"

# VFP Tunes
AVAILTUNES += "cortexa15hf cortexa15thf cortexa15hf-neon"
TUNE_FEATURES_tune-cortexa15hf ?= "${TUNE_FEATURES_tune-armv7ahf} cortexa15"
TUNE_FEATURES_tune-cortexa15thf ?= "${TUNE_FEATURES_tune-armv7athf} cortexa15"
TUNE_FEATURES_tune-cortexa15hf-neon ?= "${TUNE_FEATURES_tune-armv7ahf-neon} cortexa15"

TUNEABI_tune-cortexa15hf = "armv7ahf"
TUNEABI_tune-cortexa15thf ?= "armv7athf"
TUNEABI_tune-cortexa15hf-neon ?= "armv7ahf-neon"

PACKAGE_EXTRA_ARCHS_tune-cortexa15hf = "${PACKAGE_EXTRA_ARCHS_tune-armv7ahf}"
PACKAGE_EXTRA_ARCHS_tune-cortexa15thf = "${PACKAGE_EXTRA_ARCHS_tune-armv7athf}"
PACKAGE_EXTRA_ARCHS_tune-cortexa15hf-neon = "${PACKAGE_EXTRA_ARCHS_tune-armv7ahf-neon}"
