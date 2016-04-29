# kbuild trick to avoid linker error. Can be omitted if a module is built.
obj- := dummy.o

hostprogs-$(CONFIG_SECCOMP_FILTER) := bpf-fancy dropper bpf-direct

HOSTCFLAGS_bpf-fancy.o += -I$(srctree)/usr/include
HOSTCFLAGS_bpf-fancy.o += -idirafter $(srctree)/include
HOSTCFLAGS_bpf-helper.o += -I$(srctree)/usr/include
HOSTCFLAGS_bpf-helper.o += -idirafter $(srctree)/include
bpf-fancy-objs := bpf-fancy.o bpf-helper.o

HOSTCFLAGS_dropper.o += -I$(srctree)/usr/include
HOSTCFLAGS_dropper.o += -idirafter $(srctree)/include
dropper-objs := dropper.o

HOSTCFLAGS_bpf-direct.o += -I$(srctree)/usr/include
HOSTCFLAGS_bpf-direct.o += -idirafter $(srctree)/include
bpf-direct-objs := bpf-direct.o

# Try to match the kernel target.
ifndef CONFIG_64BIT
ifndef CROSS_COMPILE

# s390 has -m31 flag to build 31 bit binaries
ifndef CONFIG_S390
MFLAG = -m32
else
MFLAG = -m31
endif

HOSTCFLAGS_bpf-direct.o += $(MFLAG)
HOSTCFLAGS_dropper.o += $(MFLAG)
HOSTCFLAGS_bpf-helper.o += $(MFLAG)
HOSTCFLAGS_bpf-fancy.o += $(MFLAG)
HOSTLOADLIBES_bpf-direct += $(MFLAG)
HOSTLOADLIBES_bpf-fancy += $(MFLAG)
HOSTLOADLIBES_dropper += $(MFLAG)
endif
endif

ifeq ($(CROSS_COMPILE),)
# Tell kbuild to always build the programs
always := $(hostprogs-y)
endif
