ccflags-y += -DPLATFORM_LINUX
obj-y   += fusb30x_whole.o
fusb30x_whole-objs :=	fusb30x_driver.o \
			fusb30x_global.o \
			platform.o \
			platform_helpers.o \
			hostcomm.o \
			core/AlternateModes.o \
			core/core.o \
			core/fusb30X.o \
			core/Log.o \
			core/PDPolicy.o \
			core/PDProtocol.o \
			core/TypeC.o \
			core/vdm/bitfield_translators.o \
			core/vdm/vdm.o \
			core/vdm/vdm_callbacks.o \
			core/vdm/vdm_config.o \
			core/vdm/DisplayPort/configure.o \
			core/vdm/DisplayPort/dp.o \
			core/vdm/DisplayPort/dp_system_stubs.o

KERNELVER               ?= $(shell uname -r)
KERNELDIR               ?= /lib/modules/$(KERNELVER)/build
INSTALL_MOD_PATH        ?= /..
PWD                     := $(shell pwd)

all: module

module:
	$(MAKE) -C $(KERNELDIR) M=$(PWD)

clean:
	rm -f *.o *~ .depend .*.cmd *.ko *.mod.c
	rm -f Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers

