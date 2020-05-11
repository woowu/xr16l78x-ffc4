obj-m := xr16l78x.o

# Overwrite this from command line when doing cross-compiling, in that
# case the dir should be a kernel source that configired for the target
# architecture.
# Hence, that means somthing like below:
# ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- KERNELDIR=/path/to/kernel/src make
#
KERNELDIR ?= /lib/modules/$(shell uname -r)/build

all default: modules
install: modules_install

modules modules_install help clean:
	$(MAKE) -C $(KERNELDIR) M=$(shell pwd) $@
