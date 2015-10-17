ccflags-y += -DPLATFORM_LINUX
obj-m   += fusb30x_whole.o
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
			bitfield_translators.o \
			vdm.o \
			vdm_callbacks.o \
			vdm_config.o \
			dp.o \
			dp_system_stubs.o

KERNELVER               ?= $(shell uname -r)
KERNELDIR               ?= /lib/modules/$(KERNELVER)/build
INSTALL_MOD_PATH        ?= /..
PWD                     := $(shell pwd)

all: module

module:
	$(MAKE) -C $(KERNELDIR) M=$(PWD)

check:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) C=2 CF="-D__CHECK_ENDIAN__"

clean:
	rm -f *.o *~ .depend .*.cmd *.ko *.mod.c
	rm -f Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers

