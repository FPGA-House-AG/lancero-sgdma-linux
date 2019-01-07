obj-m += lancero.o
lancero-objs := lancero-base.o lancero-user.o

KERNELDIR ?= /lib/modules/$(shell uname -r)/build
#KERNELDIR := /home/leon/targetfs/home/root/c6a816x-evm/lib/modules/2.6.34/build
#KERNELDIR := /home/leon/ti-ezsdk_dm816x-evm_5_01_01_80/psp/linux-2.6.37-psp04.00.00.10

#KERNELDIR := /home/leon/arrow/linux-2.6.35/usr

PWD       := $(shell pwd)

ccflags-y := -Wno-error=date-time -Wno-error=unused-function -Wno-error=unused-variable -Wno-error=unused-label

all:
	$(MAKE) -C $(KERNELDIR) M=$(PWD)

clean:
	rm -rf *.o *.o.d *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions *.symvers .#* *.save *.bak Modules.* modules.order Module.markers *.bin

