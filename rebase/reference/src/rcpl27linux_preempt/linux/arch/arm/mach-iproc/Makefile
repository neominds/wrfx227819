ccflags-$(CONFIG_MACH_IPROC) := -I$(srctree)/drivers/net/ethernet/broadcom/gmac/src/include/
KBUILD_CFLAGS  += -DBCMDRIVER

obj-y := io_map.o northstar.o common.o northstar_dmu.o board_bu.o idm.o

obj-$(CONFIG_PM) += pm.o
obj-$(CONFIG_MTD) += flash.o
obj-$(CONFIG_IPROC_PMU) += iproc-pmu.o
