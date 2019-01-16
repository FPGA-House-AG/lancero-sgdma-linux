obj-m += lancero.o
lancero-objs := lancero-base.o lancero-user.o

#KERNELDIR := /lib/modules/2.6.32-25-generic/build

KERNELDIR ?= /lib/modules/$(shell uname -r)/build
#KERNELDIR := /home/leon/ti-ezsdk_dm816x-evm_5_01_01_80/psp/linux-2.6.37-psp04.00.00.10
PWD       := $(shell pwd)

all:
	$(MAKE) -C $(KERNELDIR) M=$(PWD)

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions *.symvers .#* *.save *.bak Modules.* modules.order Module.markers *.bin

