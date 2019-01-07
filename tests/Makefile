CC ?= gcc
#CC=~/sandbox/ccm/topha/openembedded/tmp/sysroots/i686-linux/usr/ppce500v2/bin/powerpc-angstrom-linux-gnuspe-gcc
#CC=/home/leon/sandbox/sidebranch/warpcomm/openembedded/tmp/sysroots/i686-linux/usr/armv5te/bin/arm-oe-linux-gnueabi-gcc
#CC=~/sandbox/ccm/topha/openembedded/tmp/sysroots/i686-linux/usr/ppce500v2/bin/powerpc-angstrom-linux-gnuspe-gcc
#CC=/home/leon/sandbox/sidebranch/openembedded/p1022rdk/openembedded/tmp/sysroots/i686-linux/usr/ppce500v2/bin/powerpc-oe-linux-gnuspe-gcc

all: mmap devalt ram stress datagenerator dma dma_to_device dma_from_device ringbuffer #lcd

adjacent: adjacent.o
	$(CC) -o $@ $<

stress: stress.o
	$(CC) -o $@ $<

mmap: mmap.o
	$(CC) -o $@ $<

dma: dma.o
	$(CC) -o $@ $< -D_FILE_OFFSET_BITS=64 -D_GNU_SOURCE -D_LARGE_FILE_SOURCE

dma_to_device: dma_to_device.o
	$(CC) -o $@ $< -D_FILE_OFFSET_BITS=64 -D_GNU_SOURCE -D_LARGE_FILE_SOURCE

dma_from_device: dma_from_device.o
	$(CC) -o $@ $< -D_FILE_OFFSET_BITS=64 -D_GNU_SOURCE -D_LARGE_FILE_SOURCE

ringbuffer: ringbuffer.o
	$(CC) -o $@ $< -D_FILE_OFFSET_BITS=64 -D_GNU_SOURCE -D_LARGE_FILE_SOURCE

lcd: lcd.o
	$(CC) -o $@ $< -D_FILE_OFFSET_BITS=64 -D_GNU_SOURCE -D_LARGE_FILE_SOURCE

devalt: devalt.o
	$(CC) -o $@ $<

ram: ram.o
	$(CC) -o $@ $<

datagenerator: datagenerator.o
	$(CC) -D_FILE_OFFSET_BITS=64 -D_GNU_SOURCE -o $@ $< -lpthread

%.o: %.c
	$(CC) -c -std=c99 -o $@ $< -D_FILE_OFFSET_BITS=64 -D_GNU_SOURCE -D_LARGE_FILE_SOURCE

clean:
	rm -rf mmap devalt ram stress datagenerator *.o *.bin dma_to_device dma_from_device lcd

