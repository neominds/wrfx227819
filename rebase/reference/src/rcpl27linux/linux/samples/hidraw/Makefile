# kbuild trick to avoid linker error. Can be omitted if a module is built.
obj- := dummy.o

# List of programs to build
hostprogs-y := hid-example

ifeq ($(CROSS_COMPILE),)
# Tell kbuild to always build the programs
always := $(hostprogs-y)
endif

HOSTCFLAGS_hid-example.o += -I$(srctree)/usr/include
