#
# Makefile for MIPS-specific library files..
#

lib-y	+= csum_partial_copy.o memcpy.o memcpy-inatomic.o promlib.o \
	   strlen_user.o strncpy_user.o strnlen_user.o uncached.o

obj-y			+= iomap.o
obj-$(CONFIG_PCI)	+= iomap-pci.o

# libgcc-style stuff needed in the kernel
# PR43073: changed from lib-y
obj-y += ashldi3.o ashrdi3.o lshrdi3.o

EXTRA_AFLAGS := $(CFLAGS)
