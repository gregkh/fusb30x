ccflags-y += -DPLATFORM_LINUX
obj-y   += fusb30x_whole.o
fusb30x_whole-objs :=	fusb30x_driver.o \
			fusb30x_global.o \
			platform.o \
			platform_helpers.o \
			hostcomm.o \
			AlternateModes.o \
			core.o \
			fusb30X.o \
			Log.o \
			PDPolicy.o \
			PDProtocol.o \
			TypeC.o \
			vdm/bitfield_translators.o \
			vdm/vdm.o \
			vdm/vdm_callbacks.o \
			vdm/vdm_config.o \
			vdm/DisplayPort/configure.o \
			vdm/DisplayPort/dp.o \
			vdm/DisplayPort/dp_system_stubs.o

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

