/*
 * Driver for Lancero SGDMA for FPGA logic
 *
 * Copyright (C) 2007-2012 Sidebranch
 *
 * Leon Woestenberg <leon@sidebranch.com>
 *
 */

#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/fb.h> /* frame buffer */
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/mm.h> /* mmap */
#include <linux/mm_types.h> /* mmap */
#if defined(CONFIG_MTD) && 0
#  include <linux/mtd/map.h>
#  include <linux/mtd/mtd.h>
#endif

#include <linux/pci.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>
#include <linux/uio.h>
#include <linux/version.h>

/* kernel bug: linux/aio.h depends on linux/kobject.h linux/kdev_t.h */
#include <linux/aio.h>
#include <linux/splice.h>

#include "lancero-user.h"

/* compile-time options */
#define LANCERO_GPL 0

/* optimistic back-to-back I/O chaining */
#define CHAIN_MULTIPLE_TRANSFERS 1

/* demo only */
#define LANCERO_FRAMEBUFFER 0

/* not supported */
#define LANCERO_RINGBUFFER 1

/* not supported */
#define FPGA_DESCRIPTORS 0

/* not supported */
#define PACKET_LENGTH_TABLE 0

/* for test purposes only, not in default IP! */
#define DESC_COUNTER 0

#if LANCERO_GPL
MODULE_LICENSE("Copyright (C) 2009-2011 Sidebranch");
#else
MODULE_LICENSE("GPL v2");
#endif
MODULE_AUTHOR("Leon Woestenberg <leon@sidebranch.com>");

#ifndef __devinit
#define __devinit
#endif
#ifndef __devexit
#define __devexit
#endif
#ifndef DMA_32BIT_MASK
#define DMA_32BIT_MASK DMA_BIT_MASK(32)
#endif
#ifndef DMA_64BIT_MASK
#define DMA_64BIT_MASK DMA_BIT_MASK(64)
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,1,0)
	#define AIO_COMPLETE(iocb, done, x) iocb->ki_complete(iocb, done, x)
#else
	#define AIO_COMPLETE(iocb, done, x) aio_complete(iocb, done, x)
#endif

__attribute__((unused)) static const char *cvs_revision = __FILE__ ": $Revision: 1.146 $";

#define DRV_NAME "lancero"
#define LANCERO_KNOWN_REVISION (0x01)
#define LANCERO_BAR_NUM (2)

#define LANCERO_BAR_SGDMA (1)

#define USER_BAR (0)

#define LANCERO_BAR (1)
#define LANCERO_BAR_SIZE (0x1000UL)

#define MAX_EXTRA_ADJ (15)

#if 0
/* offset of the SOPC modules on the control bus */
#define LANCERO_OFS_INT_CTRL 		(0x2000UL)
#define LANCERO_OFS_CONFIG 		(-1UL) //(0x0000UL)
#define LANCERO_OFS_DMA_WRITE 		(0x4000UL)
#define LANCERO_OFS_DMA_WRITE_PERF	(0x4100UL)
#define LANCERO_OFS_DMA_WRITE_TESTER	(0x4200UL)
#define LANCERO_OFS_DMA_READ 		(0x6000UL)
#define LANCERO_OFS_DMA_READ_PERF	(0x6100UL)
#define LANCERO_OFS_DMA_READ_TESTER	(0x6200UL)
#define LANCERO_OFS_FRAMEBUFFER		(0x8000UL)
#define LANCERO_INT_DMA_WRITE		(1 << 1)
#define LANCERO_INT_DMA_READ		(1 << 3)
#define LANCERO_INT_DMA_WRITE_PERF	(1 << 2)
#define LANCERO_INT_DMA_READ_PERF	(1 << 4)
#define LANCERO_INT_DMA_READ_TESTER	(1 << 5)
#else
/* Target internal components on BAR1 */
#define LANCERO_OFS_CONFIG 			(0x0000UL)
#define LANCERO_OFS_INT_CTRL 		(0x0100UL)

/* Scatter-Gather internal components on BAR1 */
#define LANCERO_OFS_DMA_WRITE 		(0x0200UL)
#define LANCERO_OFS_DMA_WRITE_PERF	(0x0300UL)
#define LANCERO_OFS_DMA_READ 		(0x0400UL)
#define LANCERO_OFS_DMA_READ_PERF	(0x0500UL)

/* Scatter-Gather reference design only, BAR0 */
#define LANCERO_OFS_DMA_WRITE_TESTER	(0x1000UL)
#define LANCERO_OFS_DMA_READ_TESTER		(0x2000UL)

/* Scatter-Gather framebuffer design only, BAR0 */
#define LANCERO_OFS_FRAMEBUFFER		(0x0000UL)

/* interrupts of Scatter-Gather internal components */
#define LANCERO_INT_DMA_WRITE		(1UL << 16)
#define LANCERO_INT_DMA_READ		(1UL << 18)
#define LANCERO_INT_DMA_WRITE_PERF	(1UL << 17)
#define LANCERO_INT_DMA_READ_PERF	(1UL << 19)

#if LANCERO_FRAMEBUFFER
/* interrupts of Lancero Video reference design only */
#define LANCERO_INT_FRAMEBUFFER 	(1UL << 0)
#else
/* interrupts of Scatter-Gather reference design only */
#define LANCERO_INT_DMA_READ_TESTER	(1UL << 0)
#endif

#endif

static unsigned int major = 0;
module_param(major, uint, 0644);
MODULE_PARM_DESC(major, "Character device major number, 0 for dynamic selection which is default.");

static unsigned int msi = 1;
module_param(msi, uint, 0644);
MODULE_PARM_DESC(msi, "Use 0 to disable MSI interrupting, default is 1 to use MSI interrupting.");

static unsigned int bar_map_size = 0;
module_param(bar_map_size, uint, 0644);
MODULE_PARM_DESC(bar_map_size, "BAR 0 mapping size in bytes, to map less than the full BAR size, which is default.");

static unsigned int performance_interval = 10;
module_param(performance_interval, uint, 0644);
MODULE_PARM_DESC(performance_interval, "Performance measurement interval in seconds, defaults to 10 seconds.");

static unsigned int performance_dirs = -1;
module_param(performance_dirs, uint, 0644);
MODULE_PARM_DESC(performance_dirs, "Performance measurement direction(s), 0 = from device, 1 = to device, 2 = both, default -1 disabled.");

static unsigned int bridge_bar = LANCERO_BAR_SGDMA;
module_param(bridge_bar, uint, 0644);
MODULE_PARM_DESC(bridge_bar, "BAR number for the target bridge, default 0.");

/* Requires CONFIG_MTD_COMPLEX_MAPPINGS */
static int flash_addr = -1;
module_param(flash_addr, int, 0644);
MODULE_PARM_DESC(flash_addr, "Address of the FlashROM to probe for; default -1 disables probe.");

static unsigned int target_addr = -1;
module_param(target_addr, uint, 0644);
MODULE_PARM_DESC(target_addr, "Address offset to test read/write through target_bridge; default -1 disables test.");

static unsigned int tester_offset = -1;
module_param(tester_offset, uint, 0644);
MODULE_PARM_DESC(tester_offset, "Descriptor tester address offset; default -1 assumes no tester present.");

static unsigned int desc_num = 0;
module_param(desc_num, uint, 0644);
MODULE_PARM_DESC(desc_num, "Number of descriptors; default 0 disables test.");

static int desc_offset = 0;
module_param(desc_offset, int, 0644);
MODULE_PARM_DESC(desc_offset, "Offset of the first descriptor to a 4096 bytes boundary (-4095, 4095); default 0.");

static unsigned int seed = 0;
module_param(seed, uint, 0644);
MODULE_PARM_DESC(seed, "If non-zero, runs semi-random tests using the descriptor tester module; default -1 disables test.");

static unsigned int test_to_dev = 1;
module_param(test_to_dev, uint, 0644);
MODULE_PARM_DESC(test_to_dev, "If tests are run, determine direction (test_to_device = 1 means towards end point).");

static unsigned int test_dma_runs = 10000;
module_param(test_dma_runs, uint, 0644);
MODULE_PARM_DESC(test_dma_runs, "Number of DMA test runs");

static int engine_first = -1;
module_param(engine_first, int, 0644);
MODULE_PARM_DESC(engine_first, "Performance test, first engine in the performance test (0 = write, 1 = read)");

static int engine_last = -1;
module_param(engine_last, int, 0644);
MODULE_PARM_DESC(engine_last, "Performance test, last engine in the performance test (0 = write, 1 = read)");

static int test_dma_size = -1;
module_param(test_dma_size, int, 0644);
MODULE_PARM_DESC(test_dma_size, "Fixed-sized DMA transfers, defaults to -1 which means random.");

static int test_dma_boundary_offset = -1;
module_param(test_dma_boundary_offset, int, 0644);
MODULE_PARM_DESC(test_dma_boundary_offset, "DMA Offset against 4096 bytes boundary, defaults to -1 which means random.");

static unsigned int config_offset = LANCERO_OFS_CONFIG;
module_param(config_offset, uint, 0644);
MODULE_PARM_DESC(config_offset, "Configuration module address offset.");

static unsigned int payload = 0;
module_param(payload, uint, 0644);
MODULE_PARM_DESC(payload, "Payload.");

static const struct pci_device_id pci_ids[] = {
	{ PCI_DEVICE(0x1172, 0x0004), },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, pci_ids);

/**
 * Minimum sizes of the mappings of the device BAR resources, zero means do not
 * map. If the actual BAR length is less, this is an error; check the Altera
 * PCIe core configuration and Lancero manual.
 */
static resource_size_t bar_map_sizes[LANCERO_BAR_NUM] =
	{ 16 * 1024, LANCERO_BAR_SIZE, 0, 0, 0, 0 };

/* testing purposes; request interrupt on each descriptor */
#define FORCE_IR_DESC_COMPLETED 0
/* testing purposes; sustained hardware throughput benchmark */
#define LANCERO_PERFORMANCE_TEST 0
/* descriptor, ioread/write, scatter-gather debugging */
#define dbg_desc(fmt, ...) printk(KERN_DEBUG fmt, ##__VA_ARGS__)
#define dbg_io(fmt, ...) printk(KERN_DEBUG fmt, ##__VA_ARGS__)
#define dbg_perf(fmt, ...) printk(KERN_DEBUG fmt, ##__VA_ARGS__)
#define dbg_sg(fmt, ...) printk(KERN_DEBUG fmt, ##__VA_ARGS__)
#define dbg_tfr(fmt, ...) printk(KERN_INFO fmt, ##__VA_ARGS__)

/* comment-out to enable debugging */
#define dbg_desc(...)
#if 0
#define dbg_io(...)
//#define dbg_perf(...)
#define dbg_sg(...)
#define dbg_tfr(...)
#endif


#if !FPGA_DESCRIPTORS
/* maximum number of bytes per transfer request */
#  define LANCERO_TRANSFER_MAX_BYTES (2048 * 4096)
#else
/* bound by on-chip memory size of 4kB (minus two pages) */
#  define LANCERO_TRANSFER_MAX_BYTES (126 * 4096)
#  error Micron Optics
#endif

/* maximum size of a single DMA transfer descriptor; 1<<16 = 64 KB */
#define LANCERO_DESC_MAX_BYTES ((1 << 18) - 1)

/** bits of the SG DMA control register */
#define LANCERO_CTRL_RUN_STOP (1UL << 0)
#define LANCERO_CTRL_IE_DESCRIPTOR_STOPPED (1UL << 1)
#define LANCERO_CTRL_IE_DESCRIPTOR_COMPLETED (1UL << 2)
#define LANCERO_CTRL_IE_MAGIC_STOPPED (1UL << 4)
#define LANCERO_CTRL_IE_IDLE_STOPPED (1UL << 6)
#define LANCERO_CTRL_IE_NONALIGNED_STOPPED (1UL << 9)
#define LANCERO_CTRL_RST (1UL << 31)

/** bits of the SG DMA status register */
#define LANCERO_STAT_BUSY (1UL << 0)
#define LANCERO_STAT_DESCRIPTOR_STOPPED (1UL << 1)
#define LANCERO_STAT_DESCRIPTOR_COMPLETED (1UL << 2)
#define LANCERO_STAT_MAGIC_STOPPED (1UL << 4)
#define LANCERO_STAT_FETCH_STOPPED (1UL << 5)
#define LANCERO_STAT_IDLE_STOPPED (1UL << 6)
#define LANCERO_STAT_PAYLOAD_MISMATCH (1UL << 7)
#define LANCERO_STAT_MAXREAD_MISMATCH (1UL << 8)
#define LANCERO_STAT_NONALIGNED_STOPPED (1UL << 9)
#define LANCERO_STAT_SEQUENCE_STOPPED (1UL << 10)

/** bits of the SG DMA descriptor control field */
#define LANCERO_DESC_STOP (1UL << 0)
#define LANCERO_DESC_COMPLETED (1UL << 1)

#define CHAR_USER 0
#define CHAR_CTRL 1
#define CHAR_EVENTS 2
#define CHAR_SGDMA 3

#define MAGIC_ENGINE 0xEEEEEEEEUL
#define MAGIC_DEVICE 0xDDDDDDDDUL
#define MAGIC_CHAR 0xCCCCCCCCUL

#define LANCERO_FB_X 800
#define LANCERO_FB_Y 600

#define LANCERO_FB_BPP 2

/* horizontal front porch; time from sync to picture */
#define LANCERO_FB_HFP 100
/* horizontal back porch; time from picture to sync */
#define LANCERO_FB_HBP 100
#define LANCERO_FB_HSYNC 56

#define LANCERO_FB_VFP 10
#define LANCERO_FB_VBP 10
#define LANCERO_FB_VSYNC 5

#define LANCERO_FB_BUFNUM 3

/**
 * SG DMA Controller status and control registers
 *
 * These registers make the control interface for DMA transfers.
 *
 * It sits in End Point (FPGA) memory BAR[0] for 32-bit or BAR[0:1] for 64-bit.
 * It references the first descriptor which exists in Root Complex (PC) memory.
 *
 * @note The registers must be accessed using 32-bit (PCI DWORD) read/writes,
 * and their values are in little-endian byte ordering.
 */
struct engine_regs {
	/* identifier */
	u32 identifier;
	/* status register */
	u32 status;
	/* control register */
	u32 control;
	/* bus address to first descriptor in Root Complex memory */
	u32 first_desc; /* 0x0c */
	/* number of adjacent descriptors at first_desc */
	u32 first_desc_adjacent; /* 0x10 */
	/* number of completed descriptors */
	u32 completed_desc_count; /* 0x14 */
	u32 completed_desc_bytes; /* 0x18 */
	u32 first_desc_hi; /* 0x1c */
#if PACKET_LENGTH_TABLE
	u32 length_table_lo;
	u32 length_table_hi;
	u32 length_table_len;
#endif
} __attribute__ ((packed));

/* Performance counter for Avalon Streaming
 */
struct performance_regs {
	/* identifier 0xc34900xx */
	u32 identifier;
	/* control register */
	u32 control;
	/* status register */
	u32 status;
	/* 64-bit period in 8 ns units (low 32-bit) */
	u32 period_low;
	/* period (high 32-bit) */
	u32 period_high;
	/* 64-bit performance counter in 8-byte units (low 32-bit) */
	u32 performance_low;
	/* performance (high 32-bit) */
	u32 performance_high;
	/* 64-bit wait counter in 8-byte units (low 32-bit) */
	u32 wait_low;
	/* wait (high 32-bit) */
	u32 wait_high;
} __attribute__ ((packed));
#define PERF_CTRL_RUN 1
#define PERF_CTRL_IE 2

#define PERF_STAT_BUSY 1
#define PERF_STAT_IRQ 2

/* Incremental data tester
 */
struct tester_regs {
	/* identifier 0xae2300xx */
	u32 identifier;
	/* control register */
	u32 control;
	/* status register */
	u32 status;
	/* counter register */
	u32 counter;
};

/** Lancero frame buffer
 */
struct lancerofb_regs {
	/* identifier 0xfe387dxx (0 / 0x00) */
	u32 identifier;
	/* control register (1 / 0x04) */
	u32 control;
	/* status register (2 / 0x08) */
	u32 status;
	/* horizontal front porch (pixels) (3 / 0x0c) */
	u32 hor_front_porch;
	/* horizontal sync width (pixels) (4 / 0x10) */
	u32 hor_sync_width;
	/* horizontal back porch (pixels) (5 / 0x14) */
	u32 hor_back_porch;
	/* horizontal pixels (pixels) (6 / 0x18) */
	u32 hor_pixels;
	/* vertical front porch (lines) (7 / 0x1c) */
	u32 ver_front_porch;
	/* vertical sync width (lines) (8 / 0x20) */
	u32 ver_sync_width;
	/* vertical back porch (lines) (9 / 0x24) */
	u32 ver_back_porch;
	/* vertical lines (10 / 0x28) */
	u32 ver_lines;
	u32 fader;
	/* frame count (0x30) */
	u32 count;
};

struct lancero_packet_generator_regs {
	u32 control;
};

struct lancero_latency_tester_regs {
	u32 id;
	u32 control;
	u32 status;
	u32 delay;
	u32 data;
	u32 counter;
};

/**
 * Descriptor for a single contiguous memory block transfer.
 *
 * Multiple descriptors are linked by means of the next pointer. An additional
 * extra adjacent number gives the amount of extra contiguous descriptors.
 *
 * The descriptors are in root complex memory, and the bytes in the 32-bit
 * words must be in little-endian byte ordering.
 */
struct lancero_desc {
	/* descriptor control field (0 / 0x00) */
	u32 control;
	/* number of bytes in the transfer (1 / 0x04) */
	u32 bytes;
	/* FPGA bus address */
	u32 fpga_addr;
	u32 fpga_addr_pad;
	/* PCIe bus address */
	u32 pcie_addr_lo;
	u32 pcie_addr_hi;
	/* next descriptor in the single-linked list of descriptors;
	 * this is the bus address of the next descriptor in the
	 * root complex memory. */
	u32 next_lo;
	u32 next_hi;
} __attribute__ ((packed));

/**
 * Describes a (SG DMA) single transfer for the engine.
 */
struct lancero_transfer {
#if 0
	struct lancero_dev *lro;
#endif
	/* queue of non-completed transfers */
	struct list_head entry;
	/* virtual address of the first descriptor */
	struct lancero_desc *desc_virt;
	/* bus address of the first descriptor */
	dma_addr_t desc_bus;
	/* number of descriptors adjacent in memory at desc_bus address */
	int desc_adjacent;
	/* number of descriptors involved in the transfer */
	int desc_num;
	/* whether the direction of the transfer is to the device */
	int dir_to_dev;
	/* wait queue for synchronously waiting threads */
	wait_queue_head_t wq;
	/* completion buffer for asynchronous I/O, NULL if not used */
	struct kiocb *iocb;
	/* number of descriptors at desc_virt address */
	int sgl_nents;
	/* user space scatter gather mapper, NULL if not used */
	struct sg_mapping_t *sgm;
	/* user space pages were gotten? */
	int userspace;
	/* state of the transfer */
	int state;
	/* cyclic transfer? */
	int cyclic;
	/* last transfer within an I/O request? */
	int last_in_request;
	/* last transfer within an I/O request? */
	ssize_t size_of_request;
};

#define TRANSFER_STATE_NEW 0
#define TRANSFER_STATE_SUBMITTED 1
#define TRANSFER_STATE_COMPLETED 3
#define TRANSFER_STATE_FAILED 4

struct lancero_engine {
	/* MAGIC_ENGINE == 0xEEEEEEEE */
	unsigned long magic;
	/* parent device */
	struct lancero_dev *lro;

	/* character device major:minor */
	dev_t cdevno;
	/* character device (embedded struct) */
	struct cdev cdev;

	/* protects concurrent access interrupt context */
	spinlock_t lock;
	/* remember CPU# of (last) locker */
	int prev_cpu;
	/* queue of transfers to be completed */
	struct list_head transfer_list;
	/* address offset of the engine in its BAR */
	struct engine_regs *regs;
	/* direction of this engine */
	int dir_to_dev;
	/* whether the driver has started the engine */
	int running;
	/* whether the engine stopped accepting new requests */
	int shutdown;
/* engine requested to shutdown */
#define ENGINE_SHUTDOWN_REQUEST 1
/* engine has been shutdown and is idle */
#define ENGINE_SHUTDOWN_IDLE 2
/* stay idle preparing for driver unload */
#define ENGINE_SHUTDOWN_TEARDOWN 4
	/* wait queue for synchronously waiting threads */
	wait_queue_head_t shutdown_wq;
	/* last known status of device */
	u32 status;
#if 0
	/* last set control of device */
	u32 control;
#endif
	/* interrupt */
	u32 interrupt;
	/* name of this engine */
	char *name;
	/* version of this engine */
	int version;
	/* descriptor prefetch capability */
	int max_extra_adj;
	/* user space scatter gather mapper */
	struct sg_mapping_t *sgm;
	/* total number of descriptors of completed transfers in this run */
	int desc_dequeued;
	/* engine service work */
	struct work_struct work;
};
/*
 * Lancero Avalon bus specific book keeping. Each bus has a character device,
 * the control bus has no SGDMA engines attached to it.
 */
struct lancero_char {
	/* MAGIC_CHAR == 0xCCCCCCCC */
	unsigned long magic;
	/* parent device */
	struct lancero_dev *lro;
	/* character device major:minor */
	dev_t cdevno;
	/* character device (embedded struct) */
	struct cdev cdev;
	/* for character devices on a control bus, -1 otherwise */
	int bar;
	/* for character device interfaces to the data bus, the SG DMA read engine */
	struct lancero_engine *read_engine;
	/* for character device interfaces to the data bus, the SG DMA write engine */
	struct lancero_engine *write_engine;
	/* sysfs device */
	struct device *sys_device;
};

/*
 * Lancero PCIe device specific bookkeeping
 */
struct lancero_dev {
	/* MAGIC_DEVICE == 0xDDDDDDDD */
	unsigned long magic;
	/* the kernel pci device data structure provided by probe() */
	struct pci_dev *pci_dev;

	/*
	 * kernel virtual address of the mapped BAR regions. See (un)map_bars().
	 */
	void *__iomem bar[LANCERO_BAR_NUM];
	/*
	 * bus address of the descriptor list in Root Complex memory
	 */
	dma_addr_t table_bus;
	/* if the device regions could not be allocated, assume and remember it
	 * is in use by another driver; this driver must not disable the device.
	 */
	int regions_in_use;
	/* whether msi was enabled for the device */
	int msi_enabled;
	/* whether this driver could obtain the regions */
	int got_regions;
	/* irq line succesfully requested by this driver, -1 otherwise */
	int irq_line;
	/* board revision */
	u8 revision;
	/* core capabilities */
	u32 capabilities;
#define CAP_INT_EVENTS 1
#define CAP_64BIT_DMA 2
#define CAP_64BIT_DESC 4
#define CAP_ENGINE_WRITE 8
#define CAP_ENGINE_READ 16
	/* interrupt count, incremented by the interrupt handler */
	int irq_count;
	/* character device major:minor base */
	dev_t cdevno_base;
	/* character device structures */
	struct lancero_char *user_char_dev;
	struct lancero_char *ctrl_char_dev;
	struct lancero_char *events_char_dev;
	struct lancero_char *sgdma_char_dev;
	/* number of engines in the system */
	int engines_num;
	struct lancero_engine *engine[6];
	/* alignment rules for both directions dir_to_dev = { 0, 1 } */
	int align[2];
	/* cumulated (OR'd) irq events */
	u32 events_irq;
	/* wait queue for synchronously waiting threads */
	wait_queue_head_t events_wq;
	/* sys filesystem */
	struct class *lancero_class;
#if LANCERO_FRAMEBUFFER
	/* frame buffer */
	struct fb_info *fb_info;
	struct lancero_transfer *fb_transfer[3];
        u32 fb_count;
        int fb_active;
#endif /*LANCERO_FRAMEBUFFER*/
};

struct lancero_target_bridge {
	/* 0xBBBBBBBB */
	unsigned long magic;
};

#if 0
#define write_register(value, iomem) iowrite32(cpu_to_le32(value), iomem)
#define read_register(iomem) le32_to_cpu(ioread32(iomem))
#else
#define write_register(value, iomem) do { iowrite32(value, iomem); } while(0)
#define read_register(iomem) ioread32(iomem)
#endif

/* prototypes */
static struct lancero_transfer *create_transfer_kernel(struct lancero_dev *lro, const char *start, size_t count, u32 ep_addr, int dir_to_dev);
static void free_transfer(struct lancero_dev *lro, struct lancero_transfer *transfer);
static struct lancero_desc *lancero_desc_alloc(struct pci_dev *dev,
  int number, dma_addr_t *desc_bus_p, struct lancero_desc **desc_last_p);
static void lancero_desc_link(struct lancero_desc *first, struct lancero_desc *second,
	dma_addr_t second_bus);
static void lancero_desc_set(struct lancero_desc *desc,
  dma_addr_t rc_bus_addr, u32 ep_addr, int len);
static void lancero_desc_control(struct lancero_desc *first, u32 control_field);
static int queue_transfer(struct lancero_engine *engine,
	struct lancero_transfer *transfer);
static void lancero_transfer_cyclic(struct lancero_transfer *transfer);
static struct lancero_char *create_sg_char(struct lancero_dev *lro, int bar,
	struct lancero_engine *read_engine, struct lancero_engine *write_engine, int type);
static int destroy_sg_char(struct lancero_char *lro_char);
static void dump_transfer(struct lancero_transfer *transfer);
static u32 lancero_read_status(struct lancero_engine *engine, int clear);

/* enable_interrupts -- Enable the interrupts we are interested in */
static int enable_interrupts(struct lancero_dev *lro, int interrupts_offset, u32 mask)
{
	void *reg = lro->bar[bridge_bar] + interrupts_offset;
	u32 w;
	int rc = 0;
	//printk(KERN_DEBUG "Read register at BAR %d, address 0x%08x.\n", bridge_bar, reg);
	w = read_register(reg + 0x00);
	if (((w & 0xffffff00UL) != 0xd2870000UL) &&
	    ((w & 0x00ffff00UL) != 0x00b10000UL)) {
		printk(KERN_DEBUG "Interrupt controller identifier not found (found 0x%08x expected 0xd2870001).\n", w);
		rc = -1;
		goto fail_identifier;
	}
	/* since version 2, an event register is present */
	if ((w & 0xffUL) >= 2UL) lro->capabilities |= CAP_INT_EVENTS;
	/* enable selected interrupts */
	w = read_register(reg + 0x04);
	printk(KERN_DEBUG "Current interrupt controller enable mask: 0x%08x.\n", w);
	w |= mask;
	printk(KERN_DEBUG "New interrupt controller enable mask: 0x%08x.\n", w);
	write_register(w, reg + 0x04);
	/* read back (flushes write) */
	w = read_register(reg + 0x00);
fail_identifier:
	return rc;
}

static int interrupts_disable(struct lancero_dev *lro, int interrupts_offset, u32 mask)
{
	void *reg = lro->bar[bridge_bar] + interrupts_offset;
	u32 w;
	int rc = 0;
	//printk(KERN_DEBUG "Read register at BAR %d, address 0x%08x.\n", bridge_bar, reg);
	w = read_register(reg + 0x00);
	if (((w & 0xffffff00UL) != 0xd2870000UL) &&
	    ((w & 0x00ffff00UL) != 0x00b10000UL)) {
		printk(KERN_DEBUG "Interrupt controller identifier not found (found 0x%08x expected 0xd2870001).\n", w);
		rc = -1;
		goto fail_identifier;
	}
	/* disable selected interrupts */
	w = read_register(reg + 0x04);
	w &= ~mask;
	//printk(KERN_DEBUG "Set interrupt controller enable mask: 0x%08x.\n", w);
	write_register(w, reg + 0x04);
	/* flush previous writes using a dummy read */
	w = read_register(reg + 0x04);
fail_identifier:
	return rc;
}

/* read_interrupts -- Print the interrupt controller status */
static u32 read_interrupts(struct lancero_dev *lro, int interrupts_offset)
{
	void *reg = lro->bar[bridge_bar] + interrupts_offset;
	u32 w;
	w = read_register(reg + 0x04);
	//printk(KERN_DEBUG "interrupt controller enable mask: 0x%08x.\n", w);
	w = read_register(reg + 0x08);
	//printk(KERN_DEBUG "interrupt controller request mask: 0x%08x.\n", w);
	w = read_register(reg + 0x0c);
	//printk(KERN_DEBUG "interrupt controller event mask: 0x%08x.\n", w);
	return w;
}

/* allocate reserved vmalloc()ed 32-bit addressable memory */
#if LANCERO_FRAMEBUFFER
static void *rvmalloc(unsigned long size)
{
	void *mem;
	unsigned long adr;

	size = PAGE_ALIGN(size);
	mem = vmalloc_32(size);
	if (!mem)
		return NULL;

	adr = (unsigned long)mem;
	while (size > 0) {
		SetPageReserved(vmalloc_to_page((void *)adr));
		adr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}
	return mem;
}

/* free reserved vmalloc()ed memory */
static void rvfree(void *mem, unsigned long size)
{
	unsigned long adr;
	if (!mem)
		return;
	adr = (unsigned long)mem;
	while ((long) size > 0) {
		ClearPageReserved(vmalloc_to_page((void *)adr));
		adr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}
	vfree(mem);
}

/* set parameters */
static int lancerofb_set_par(struct fb_info *info)
{
#if 1
	struct fb_var_screeninfo *var = &info->var;
	struct fb_fix_screeninfo *fix = &info->fix;
	printk(KERN_DEBUG "lancerofb_set_par()\n");

	/* no acceleration */
	fix->accel = FB_ACCEL_NONE;
	/* only truecolor */
	fix->visual = FB_VISUAL_TRUECOLOR;
	/* bytes per line */
	fix->line_length = LANCERO_FB_X * LANCERO_FB_BPP;
#endif
	return 0;
}



/* check and correct variable settings */
static int lancerofb_check_var(struct fb_var_screeninfo *var,
struct fb_info *info)
{
	//printk(KERN_DEBUG "lancerofb_check_var()\n");

        //printk(KERN_DEBUG "yoffset = %u\n", var->yoffset);
	var->xres = LANCERO_FB_X;
	var->yres = LANCERO_FB_Y;

	var->xres_virtual = LANCERO_FB_X;
	var->yres_virtual = LANCERO_FB_Y * LANCERO_FB_BUFNUM;

	var->pixclock = 15000;                 /* pixel clock in ps (pico seconds) */
	var->left_margin = LANCERO_FB_HFP;              /* time from sync to picture    */
	var->right_margin = LANCERO_FB_HBP;             /* time from picture to sync    */
	var->upper_margin = LANCERO_FB_VFP;             /* time from sync to picture    */
	var->lower_margin = LANCERO_FB_VBP;
	var->hsync_len = LANCERO_FB_HSYNC;                /* length of horizontal sync    */
	var->vsync_len = LANCERO_FB_VSYNC;

	var->bits_per_pixel = LANCERO_FB_BPP * 8;

#if (LANCERO_FB_BPP == 4)
	/* ARGB8888 */
	var->transp.length = 8;
	var->red.length = 8;
	var->green.length = 8;
	var->blue.length = 8;

	var->transp.offset = 24;
	var->red.offset = 16;
	var->green.offset = 8;
	var->blue.offset = 0;
#else
	/* RGB565 */
	var->transp.length = 0;
	var->red.length = 5;
	var->green.length = 6;
	var->blue.length = 5;

	var->transp.offset = 0;
	var->red.offset = 11;
	var->green.offset = 5;
	var->blue.offset = 0;
#endif

	var->transp.msb_right = 0;
	var->red.msb_right = 0;
	var->green.msb_right = 0;
	var->blue.msb_right = 0;
	return 0;
}

static int lancerofb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	unsigned long start = vma->vm_start;
	unsigned long size = vma->vm_end - vma->vm_start;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	unsigned long page, pos;
 	printk(KERN_DEBUG "lancerofb_mmap()\n");
        /* @TODO + PAGE_SIZE */
 	printk(KERN_DEBUG "info->fix.smem_len = %d\n", info->fix.smem_len);
	if ((offset + size) > (info->fix.smem_len + PAGE_SIZE)) {
                printk(KERN_DEBUG "offset = %u, size = %u, info->fix.smem_len = %u, too long\n", offset, size, info->fix.smem_len);
		return -EINVAL;
	}

	pos = (unsigned long)info->fix.smem_start + offset;

	while (size > 0) {
                /* calculate pfn */
		page = vmalloc_to_pfn((void *)pos);
		if (remap_pfn_range(vma, start, page, PAGE_SIZE, PAGE_SHARED)) {
                        printk(KERN_DEBUG "remap_pfn_range()\n");
			return -EAGAIN;
		}
		start += PAGE_SIZE;
		pos += PAGE_SIZE;
		if (size > PAGE_SIZE)
			size -= PAGE_SIZE;
		else
			size = 0;
	}
	/* prevent swap-out of this VMA */
#ifndef VM_RESERVED
	vma->vm_flags |= VM_DONTEXPAND | VM_DONTDUMP;
#else
	vma->vm_flags |= VM_RESERVED;
#endif
 	printk(KERN_DEBUG "lancerofb_mmap() = 0\n");

	return 0;
}


static int old_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	unsigned long size, offset;
	int rc;
	printk(KERN_DEBUG "lancerofb_mmap()\n");

	size = vma->vm_end - vma->vm_start;
	offset = vma->vm_pgoff << PAGE_SHIFT;
	if (offset + size > info->fix.smem_len)
		return -EINVAL;

	offset += info->fix.smem_start;
	rc = remap_pfn_range(vma, vma->vm_start, vmalloc_to_pfn(offset) /* offset >> PAGE_SHIFT*/,
		size, PAGE_SHARED /*vma->vm_page_prot*/);
	if (rc) {
		printk(KERN_DEBUG "remap_pfn_range() = %d\n", rc);
		return -EAGAIN;
	}
	printk(KERN_DEBUG "lancerofb: mmap framebuffer P(0x%lx)->V(0x%lx)\n",
		offset, vma->vm_start);
	return 0;
}

static uint32_t pseudo_palette[16];

static int lancerofb_blank(int mode, struct fb_info *info)
{
	printk(KERN_DEBUG "lancerofb_blank()\n");
	return 0;
}

static int lancerofb_setcolreg(u_int color_index,
	u_int red, u_int green, u_int blue,
	u_int transp, struct fb_info *info)
{
	printk(KERN_DEBUG "lancerofb_setcolreg()\n");

	if ((info->fix.visual == FB_VISUAL_TRUECOLOR) && (color_index < 16)) {
		//printk(KERN_DEBUG "FB_VISUAL_TRUECOLOR\n");
		/* Do any required translations to convert red, blue, green and
		transp, to values that can be directly fed to the hardware */
		/* ... */
		((u32 *)(info->pseudo_palette))[color_index] =
		(red << info->var.red.offset) |
		(green << info->var.green.offset) |
		(blue << info->var.blue.offset) |
		(transp << info->var.transp.offset);
	}
	return 0;
}

/* ioctl */
static int lancerofb_ioctl(struct fb_info *info, unsigned int cmd,
	unsigned long arg)
{
	return 0;
}

static int lancerofb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
        int fb_new;
        struct lancero_dev *lro = (struct lancero_dev *)dev_get_drvdata(info->device);
	WARN_ON(lro->magic != MAGIC_DEVICE);
        //printk(KERN_DEBUG "lancerofb_pan_display() var->yoffset = %u, info->var.yoffset = %u, info->var.yres = %u\n", var->yoffset, info->var.yoffset, info->var.yres);
 
        /* calculate one of the buffers */
        fb_new = (var->yoffset / LANCERO_FB_Y) % LANCERO_FB_BUFNUM;
        //printk(KERN_DEBUG "lro->fb_active = %d, fb_new = %d\n", lro->fb_active, fb_new);
        /* make newly active buffer cyclic */
        lancero_transfer_cyclic(lro->fb_transfer[fb_new]);
        
        /* initially, we do not know the active buffer -- in fact the
         * driver loops through them for test purposes so all are active */
        if (lro->fb_active == -1) {
                printk(KERN_DEBUG "lancerofb_pan_display(): init\n");
                int i;
                /* link all other buffers to the newly activated buffer */
                for (i = 0; i < LANCERO_FB_BUFNUM; i++) {
                        lancero_desc_link(lro->fb_transfer[(fb_new + 1 + i) % LANCERO_FB_BUFNUM]->desc_virt + lro->fb_transfer[(fb_new + 1 + i) % LANCERO_FB_BUFNUM]->desc_num - 1,
                                lro->fb_transfer[fb_new]->desc_virt, lro->fb_transfer[fb_new]->desc_bus);
                }
        } else {
                /* link last activated buffer to new activated buffer, so that
                * the video DMA will reach this last and cycle there */
                lancero_desc_link(lro->fb_transfer[lro->fb_active]->desc_virt + lro->fb_transfer[lro->fb_active]->desc_num - 1,
                        lro->fb_transfer[fb_new]->desc_virt, lro->fb_transfer[fb_new]->desc_bus);
                printk(KERN_DEBUG "lancerofb_pan_display(): linking active %d to new %d\n", lro->fb_active, fb_new);
        }
        /* remember this */                
        lro->fb_active = fb_new;
        //printk(KERN_DEBUG "lancerofb_pan_display() var->yoffset = %u, info->var.yoffset = %u\n", var->yoffset, info->var.yoffset);
	//pxafb_schedule_work(fbi, C_CHANGE_DMA_BASE);
	return 0;
}


/* frame buffer operations */
static struct fb_ops lancerofb_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = lancerofb_check_var,
	.fb_set_par = lancerofb_set_par,
	.fb_setcolreg = lancerofb_setcolreg,
	.fb_blank = lancerofb_blank,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_mmap = lancerofb_mmap,
//	.fb_ioctl = lancerofb_ioctl,
        .fb_pan_display = lancerofb_pan_display,
};

static int lancerofb_pattern(u32 *p)
{
	unsigned int x = 0, y = 0;
	u32 colorbar[8] = { 0x00FFFFFFUL, 0x00FFFF00UL,
		0x000FFFFUL, 0x0000FF00UL, 0x00FF00FFUL, 0x00FF0000UL,
		0x00000FFUL, 0x00000000UL };
	/* first line, first pixel */
	*(p + y * LANCERO_FB_X + x) = 0x00888888UL;
	/* first line, remaining pixels */
	for (x = 1; x < LANCERO_FB_X; x++) {
		int color = x * 8 / LANCERO_FB_X;
		*(p + y * LANCERO_FB_X + x) = 0x00FFFFFFUL;
	}
	/* remaining lines */
	for (y = 1; y < LANCERO_FB_Y; y++) {
		x = 0;
		/* first pixel on the line */
		*(p + y * LANCERO_FB_X + x) = 0x00AAAAAAUL;
		for (x = 1; x < LANCERO_FB_X; x++) {
			int color = x * 8 / LANCERO_FB_X;
			*(p + y * LANCERO_FB_X + x) = colorbar[color];
		}
	}
}

static int lancerofb_pattern16(u16 *p)
{
	unsigned int x = 0, y = 0, slice = 0;
	u16 levels[7] = { 0x00, 0x20, 0x30, 0x38, 0x3c, 0x3e, 0x3f };
	u16 offsets[3] = { 0, 5, 11 };
	u32 colorbar[8] = { 0x00FFFFFFUL, 0x00FFFF00UL,
		0x000FFFFUL, 0x0000FF00UL, 0x00FF00FFUL, 0x00FF0000UL,
		0x00000FFUL, 0x00000000UL };
	/* first line, first pixel */
	*(p + y * LANCERO_FB_X + x) = 0x0888UL;
	/* first line, remaining pixels */
	for (x = 1; x < LANCERO_FB_X; x++) {
		int color = x * 8 / LANCERO_FB_X;
		*(p + y * LANCERO_FB_X + x) = 0xFFFFUL;
	}
	y = 1;
	for (slice = 0; slice < 3; slice++) 
	{
	/* remaining lines */
	for (; y < LANCERO_FB_Y * (slice + 1) / 3; y++) {
		x = 0;
		/* first pixel on the line */
		*(p + y * LANCERO_FB_X + x) = 0x00AAAAAAUL;
		for (x = 1; x < LANCERO_FB_X; x++) {
			int color = x * 7 / LANCERO_FB_X;
			*(p + y * LANCERO_FB_X + x) = levels[color] << offsets[slice];
		}
	}
	}
}

static int lancerofb_clear(u8 *p, uint32_t argb)
{
	unsigned int x = 0, y = 0;
#if (LANCERO_FB_BPP == 4)
        u32 *q = (u32 *)p;
	for (y = 0; y < LANCERO_FB_Y; y++) {
		for (x = 0; x < LANCERO_FB_X; x++) {
			int color = x * 8 / LANCERO_FB_X;
			*(q + y * LANCERO_FB_X + x) = argb;
		}
	}
#elif (LANCERO_FB_BPP == 2)
        u16 *q = (u16 *)p;
	for (y = 0; y < LANCERO_FB_Y; y++) {
		for (x = 0; x < LANCERO_FB_X; x++) {
			int color = x * 8 / LANCERO_FB_X;
			*(q + y * LANCERO_FB_X + x) = (u16)argb;
		}
	}
        *q = 0xFFFFUL;
#endif
}

static int lancerofb_start(struct lancero_dev *lro, int offset)
{
	struct lancerofb_regs *reg = (struct lancerofb_regs *)(lro->bar[USER_BAR] + offset);
	int rc = 0;
	u32 w;

	dbg_io("ioread32(0x%p).\n", reg);
	w = read_register(&reg->identifier);
	dbg_io("ioread32(0x%p) returned 0x%08x.\n", &reg->identifier, w);
	if (w != 0xfe387d02) {
		printk(KERN_DEBUG "Performance identifier not found (found 0x%08x expected 0xfe387d01).\n", w);
		rc = -1;
		goto fail_identifier;
	}

	printk(KERN_DEBUG "About to perform iowrite32(0x%x @ 0x%p).\n", w, reg);
	write_register(LANCERO_FB_HFP, (void *)&reg->hor_front_porch);
	write_register(LANCERO_FB_HSYNC, (void *)&reg->hor_sync_width);
	write_register(LANCERO_FB_HBP, (void *)&reg->hor_back_porch);
	write_register(LANCERO_FB_X, (void *)&reg->hor_pixels);
	write_register(LANCERO_FB_HFP, (void *)&reg->ver_front_porch);
	write_register(LANCERO_FB_HSYNC, (void *)&reg->ver_sync_width);
	write_register(LANCERO_FB_HBP, (void *)&reg->ver_back_porch);
	write_register(LANCERO_FB_Y, (void *)&reg->ver_lines);

	w = 1/*run*/ | 0/*not big endian*/ | 0x10 /*interrupt enable*/ | 0x30 /*RGB*/;
	printk(KERN_DEBUG "About to perform iowrite32(0x%x @ 0x%p).\n", w, &reg->control);
	write_register(w, &reg->control);
	read_register(&reg->status);
fail_identifier:
good:
	return rc;
}

static int lancerofb_stop(struct lancero_dev *lro, int offset)
{
	struct lancerofb_regs *reg = (struct lancerofb_regs *)(lro->bar[USER_BAR] + offset);
	int rc = 0;
	u32 w;
	w = 0;
	printk(KERN_DEBUG "About to perform iowrite32(0x%x @ 0x%p).\n", w, &reg->control);
	write_register(w, &reg->control);
	read_register(&reg->status);
	return rc;
}


static struct fb_fix_screeninfo lancerofb_fix __initdata = {
	.id =		DRV_NAME,
	.type =		FB_TYPE_PACKED_PIXELS,
	.visual =	FB_VISUAL_TRUECOLOR,
	.accel =	FB_ACCEL_NONE,
	.ypanstep =	1,
	.ywrapstep =	1,
};

static int __init lancerofb_create(struct lancero_dev *lro)
{
	int rc = 0;
	/* bus address */
	dma_addr_t desc_bus;
        dma_addr_t fb_smem_bus;
	/* allocate framebuffer structure */
	lro->fb_info = framebuffer_alloc(0, &lro->pci_dev->dev);
	/* could not allocate framebuffer structure? */
	if (!lro->fb_info) {
		printk(KERN_DEBUG "framebuffer_alloc() failed\n");
		rc = -ENOMEM;
		goto fail_alloc;
	}

	/* initialize info->fix */
	lro->fb_info->fix = lancerofb_fix;
	/* initialize info->var */
	lancerofb_check_var(&lro->fb_info->var, lro->fb_info);

	/* set frame buffer operations */
	lro->fb_info->fbops = &lancerofb_ops;

	fb_alloc_cmap(&lro->fb_info->cmap, 16, 0);

	lro->fb_info->fix.smem_len = LANCERO_FB_X * LANCERO_FB_Y * LANCERO_FB_BPP * LANCERO_FB_BUFNUM;


	/* allocate a frame buffer in virtual memory */
	lro->fb_info->screen_base = rvmalloc(lro->fb_info->fix.smem_len);
	//lro->fb_info->screen_base = pci_alloc_consistent(lro->pci_dev, lro->fb_info->fix.smem_len, &fb_smem_bus);
	printk(KERN_DEBUG "rvmalloc(size=%d) = %p\n",
		lro->fb_info->fix.smem_len, lro->fb_info->screen_base);
	//lro->fb_info->fix.smem_start =
	if (!lro->fb_info->screen_base) {
		printk(KERN_DEBUG "rvmalloc() failed\n");
		rc = -ENOMEM;
		goto fail_rvmalloc;
	}
#if 0        
	/* @todo bus address in &lro->fb_info->fix.smem_start - NOT CORRECT? */
	lro->fb_info->fix.smem_start = virt_to_phys(lro->fb_info->screen_base);
#endif
	lro->fb_info->fix.smem_start = lro->fb_info->screen_base;
	printk(KERN_DEBUG "rinfo->fix.smem_start = 0x%lx\n", lro->fb_info->fix.smem_start);

	lro->fb_info->pseudo_palette = pseudo_palette;

        /* create a default test pattern */
	lancerofb_pattern16((u16 *)(lro->fb_info->screen_base + LANCERO_FB_X * LANCERO_FB_Y * LANCERO_FB_BPP * 0));
	lancerofb_pattern16((u16 *)(lro->fb_info->screen_base + LANCERO_FB_X * LANCERO_FB_Y * LANCERO_FB_BPP * 1));
	lancerofb_pattern16((u16 *)(lro->fb_info->screen_base + LANCERO_FB_X * LANCERO_FB_Y * LANCERO_FB_BPP * 2));

#if 0
	//lancerofb_clear((u32 *)lro->fb_info->screen_base, 0x00FF0000UL);
	lancerofb_clear((u8 *)lro->fb_info->screen_base + LANCERO_FB_X * LANCERO_FB_Y * LANCERO_FB_BPP * 0, 0x0000F800UL);
	lancerofb_clear((u8 *)lro->fb_info->screen_base + LANCERO_FB_X * LANCERO_FB_Y * LANCERO_FB_BPP * 1, 0x000007e0UL);
	lancerofb_clear((u8 *)lro->fb_info->screen_base + LANCERO_FB_X * LANCERO_FB_Y * LANCERO_FB_BPP * 2, 0x0000001FUL);
#endif

	/* create transfer */
	lro->fb_transfer[0] = create_transfer_kernel(lro,
		lro->fb_info->screen_base + LANCERO_FB_X * LANCERO_FB_Y * LANCERO_FB_BPP * 0, LANCERO_FB_X * LANCERO_FB_Y * LANCERO_FB_BPP,
		0/*ep addr*/, 1/*dir_to_dev*/);
	if (!lro->fb_transfer[0]) {
		printk(KERN_DEBUG "sg_create_mapper() failed\n");
		rc = -ENOMEM;
		goto fail_transfer;
	}

	/* create transfer */
	lro->fb_transfer[1] = create_transfer_kernel(lro,
		lro->fb_info->screen_base + LANCERO_FB_X * LANCERO_FB_Y * LANCERO_FB_BPP * 1, LANCERO_FB_X * LANCERO_FB_Y * LANCERO_FB_BPP,
		1 * LANCERO_FB_X * LANCERO_FB_Y * LANCERO_FB_BPP/*ep addr*/, 1/*dir_to_dev*/);
	if (!lro->fb_transfer[1]) {
		printk(KERN_DEBUG "sg_create_mapper() failed\n");
		rc = -ENOMEM;
		goto fail_transfer;
	}

	/* create transfer */
	lro->fb_transfer[2] = create_transfer_kernel(lro,
		lro->fb_info->screen_base + LANCERO_FB_X * LANCERO_FB_Y * LANCERO_FB_BPP * 2, LANCERO_FB_X * LANCERO_FB_Y * LANCERO_FB_BPP,
		2 * LANCERO_FB_X * LANCERO_FB_Y * LANCERO_FB_BPP/*ep addr*/, 1/*dir_to_dev*/);
	if (!lro->fb_transfer[2]) {
		printk(KERN_DEBUG "sg_create_mapper() failed\n");
		rc = -ENOMEM;
		goto fail_transfer;
	}

	/* make transfer cyclic */
	lancero_transfer_cyclic(lro->fb_transfer[0]);
	/* make transfer cyclic */
	lancero_transfer_cyclic(lro->fb_transfer[1]);
	/* make transfer cyclic */
	lancero_transfer_cyclic(lro->fb_transfer[2]);
	/* do not stop engine and optionally request IRQ on last descriptor  */
	lancero_desc_control(lro->fb_transfer[0]->desc_virt + lro->fb_transfer[0]->desc_num - 1,
		0 /*| LANCERO_DESC_COMPLETED*/);
	lancero_desc_control(lro->fb_transfer[1]->desc_virt + lro->fb_transfer[1]->desc_num - 1,
		0 /*| LANCERO_DESC_COMPLETED*/);
	lancero_desc_control(lro->fb_transfer[2]->desc_virt + lro->fb_transfer[2]->desc_num - 1,
		0 /*| LANCERO_DESC_COMPLETED*/);

#if 0
        /* link transfer 0 to 1 */
	lancero_desc_link(lro->fb_transfer[0]->desc_virt + lro->fb_transfer[0]->desc_num - 1,
          lro->fb_transfer[1]->desc_virt, lro->fb_transfer[1]->desc_bus);

        /* link transfer 1 to 2 */
	lancero_desc_link(lro->fb_transfer[1]->desc_virt + lro->fb_transfer[1]->desc_num - 1,
          lro->fb_transfer[2]->desc_virt, lro->fb_transfer[2]->desc_bus);

        /* link transfer 2 to 0 */
	lancero_desc_link(lro->fb_transfer[2]->desc_virt + lro->fb_transfer[2]->desc_num - 1,
          lro->fb_transfer[0]->desc_virt, lro->fb_transfer[0]->desc_bus);
#endif
#if 0
        dump_transfer(lro->fb_transfer[0]);
        dump_transfer(lro->fb_transfer[1]);
        dump_transfer(lro->fb_transfer[2]);
#endif 
        lro->fb_active = -1;

	/* queue on read engine */
	queue_transfer(lro->engine[1], lro->fb_transfer[2]);

	/* wait a bit */
	msleep(1);

	lancerofb_start(lro, LANCERO_OFS_FRAMEBUFFER);

	/* set info->var */
	rc = lancerofb_set_par(lro->fb_info);
	if (rc) {
		printk(KERN_DEBUG "lancerofb_set_par() failed, rc = %d. !rc=%d\n", rc, !rc);
		goto fail_set_par;
	}
#if 1
	printk(KERN_DEBUG "register_framebuffer()\n");
	/* register the framebuffer */
	rc = register_framebuffer(lro->fb_info);
	if (rc < 0) {
		printk(KERN_DEBUG "register_framebuffer() failed, rc = %d\n", rc);
		goto fail_register;
	}
#endif
	goto good;
fail_register:
fail_set_par:
	lro->fb_transfer[0]->sgm->mapped_pages = 0;
	/* destroy the scatter gather list */
	free_transfer(lro, lro->fb_transfer[0]);
fail_transfer:
	/* free frame buffer */
	rvfree(lro->fb_info->screen_base, lro->fb_info->fix.smem_len);
fail_rvmalloc:
	/* free frame buffer */
	framebuffer_release(lro->fb_info);
	lro->fb_info = NULL;
fail_alloc:
good:
	return rc;
}

static int lancerofb_destroy(struct lancero_dev *lro)
{
	printk(KERN_DEBUG "lancerofb_destroy()\n");
	WARN_ON(!lro);
	WARN_ON(!lro->fb_info);
	/* destroy the scatter gather list */
	lro->fb_transfer[0]->sgm->mapped_pages = 0;
	free_transfer(lro, lro->fb_transfer[0]);
	/* unregister the framebuffer */
	unregister_framebuffer(lro->fb_info);
	fb_dealloc_cmap(&lro->fb_info->cmap);
	/* free frame buffer */
	rvfree(lro->fb_info->screen_base, lro->fb_info->fix.smem_len);
	lro->fb_info->screen_base = NULL;
	/* free frame buffer data structure */
	framebuffer_release(lro->fb_info);
	lro->fb_info = NULL;
}
#endif /*LANCERO_FRAMEBUFFER*/

static struct lancero_char *create_sg_char(struct lancero_dev *lro, int bar,
	struct lancero_engine *read_engine, struct lancero_engine *write_engine, int type);
static int destroy_sg_char(struct lancero_char *lro_char);

/* test_target_bridge -- write and read back some address locations
 *
 * @return zero on success, non-zero on failure
 */
static int __devinit test_target_bridge(struct lancero_dev *lro, int offset)
{
	void *reg = lro->bar[bridge_bar] + offset;
	u32 w;
	int failed = 0;
#if 0 /* 16-bit access test */
	u16 hw;
	printk(KERN_DEBUG "About to perform ioread16(0x%p).\n", reg);
	hw = read_register(reg);
	printk(KERN_DEBUG "Address 0x%p reads data 0x%08x.\n", reg, (unsigned int)hw);
	hw = 0xbabe;
	printk(KERN_DEBUG "About to perform iowrite16(0x%x @ 0x%p).\n", (unsigned int)hw, reg);
	write_register(hw, reg);
#else /* 32-bit accesses test */
	printk(KERN_DEBUG "bar = 0x%p\n", lro->bar[bridge_bar]);
	printk(KERN_DEBUG "offset = %d\n", offset);
	printk(KERN_DEBUG "About to perform ioread32(0x%p).\n", reg);
	w = read_register(reg);
	printk(KERN_DEBUG "Address 0x%p reads data 0x%08x.\n", reg, w);

	w = 0xdeadbeef;
	printk(KERN_DEBUG "About to perform iowrite32(0x%x @ 0x%p).\n", w, reg);
	write_register(w, reg);
	printk(KERN_DEBUG "About to perform ioread32(0x%p).\n", reg);
	w = read_register(reg);
	printk(KERN_DEBUG "Address 0x%p reads data 0x%08x.\n", reg, w);
	failed |= (w != 0xdeadbeef);

	w = 0xbabecafe;
	printk(KERN_DEBUG "About to perform iowrite32(0x%x @ 0x%p).\n", w, reg);
	write_register(w, reg);
	printk(KERN_DEBUG "About to perform ioread32(0x%p).\n", reg);
	w = read_register(reg);
	printk(KERN_DEBUG "Address 0x%p reads data 0x%08x.\n", reg, w);
	failed |= (w != 0xbabecafe);

	w = 0xc0dec0de;
	printk(KERN_DEBUG "About to perform iowrite32(0x%x @ 0x%p).\n", w, reg);
	write_register(w, reg);
	printk(KERN_DEBUG "About to perform ioread32(0x%p).\n", reg);
	w = read_register(reg);
	printk(KERN_DEBUG "Address 0x%p reads data 0x%08x.\n", reg, w);
	failed |= (w != 0xc0dec0de);

	w = 0xabcdeffe;
	printk(KERN_DEBUG "About to perform iowrite32(0x%x @ 0x%p).\n", w, reg);
	write_register(w, reg);
	printk(KERN_DEBUG "About to perform ioread32(0x%p).\n", reg);
	w = read_register(reg);
	printk(KERN_DEBUG "Address 0x%p reads data 0x%08x.\n", reg, w);
	failed |= (w != 0xabcdeffe);
#endif
	/* return 0 on success */
	return failed;
}

/* tester_reset -- read status then reset the tester for a new run
 */
static int tester_reset(struct lancero_dev *lro, int offset)
{
	int rc = 0;
	u32 w;
	struct tester_regs *reg = (struct tester_regs *)(lro->bar[0] + offset);
	dbg_io("ioread32(0x%p).\n", reg);
	w = read_register(&reg->identifier);
	dbg_io("ioread32(0x%p) returned 0x%08x.\n", &reg->identifier, w);
	if (((w & 0xffff0001) != 0xae230001) &&
	    ((w & 0x00ffffff) != 0x00d10001) &&
	    ((w & 0x00ffffff) != 0x00d20001)) {
		printk(KERN_DEBUG "Tester identifier not found (found 0x%08x expected 0xae230001).\n", w);
		rc = -1;
		goto fail_identifier;
	}
	w = read_register(&reg->status);
	dbg_io("ioread32(0x%p) returned 0x%08x.\n", &reg->status, w);
	w = 3;
	dbg_perf("iowrite32(0x%08x to 0x%p) (control)\n", w,
		(void *)&reg->control);
	write_register(w, &reg->control);
	/* read counter register of read tester */
	w = read_register(&reg->counter);
	dbg_io("ioread32(0x%p) returned 0x%08x.\n", &reg->counter, w);
fail_identifier:
	return rc;
}

/* tester_status -- read status then reset the tester for a new run
 */
static int tester_status(struct lancero_dev *lro, int offset, u32 expected)
{
	int rc = 0;
	u32 w;
	struct tester_regs *reg = (struct tester_regs *)(lro->bar[0] + offset);
	dbg_io("ioread32(0x%p).\n", reg);
	w = read_register(&reg->identifier);
	dbg_io("ioread32(0x%p) returned 0x%08x.\n", &reg->identifier, w);
	if (((w & 0xffff0001) != 0xae230001) &&
	    ((w & 0x00ffffff) != 0x00d20001)) {
		printk(KERN_DEBUG "Tester identifier not found (found 0x%08x expected 0xae230001).\n", w);
		rc = -1;
		goto fail_identifier;
	}
	w = read_register(&reg->status);
	dbg_io("ioread32(0x%p) returned 0x%08x.\n", &reg->status, w);
	if (w & 1) {
		printk(KERN_DEBUG "Read tester detected counter error.\n");
		rc = -1;
	}
	/* read counter register of read tester */
	w = read_register(&reg->counter);
	dbg_io("ioread32(0x%p) returned 0x%08x.\n", &reg->counter, w);
	if (w != expected) {
		printk(KERN_DEBUG "Read tester counter was %08x, expected %08x",
			w, expected);
		rc = -1;
	}
fail_identifier:
	return rc;
}

/* performance_start -- start the performance measurement module
 */
static int performance_start(struct lancero_dev *lro, int offset, u32 period_msecs)
{
	int rc = 0;
	u32 w;
	/* 125 MHz clock has 8 ns per cycle, 125000 cycles per millisecond */
	u64 period = period_msecs * 125000;
	struct performance_regs *reg = (struct performance_regs *)(lro->bar[bridge_bar] + offset);
	dbg_io("ioread32(0x%p).\n", reg);
	w = read_register(&reg->identifier);
	dbg_io("ioread32(0x%p) returned 0x%08x.\n", &reg->identifier, w);
	w &= 0x00ffff00UL;
	if ((w != 0x00490000) && (w != 0x00d30000) && (w != 0x00d20000)) {
		printk(KERN_DEBUG "Performance identifier not found (found 0x%08x expected 0x00d20000 / 0x00d30000).\n", w);
		rc = -1;
		goto fail_identifier;
	}

	w = 0;
	dbg_perf("iowrite32(0x%08x to 0x%p) (control)\n", w,
		(void *)&reg->control);
	write_register(w, &reg->control);

	w = period & 0xffffffffUL;
	dbg_perf("iowrite32(0x%08x to 0x%p) (period_low)\n", w,
		(void *)&reg->period_low);
	write_register(w, &reg->period_low);
	w = period >> 32;
	dbg_perf("iowrite32(0x%08x to 0x%p) (period_high)\n", w,
		(void *)&reg->period_high);
	write_register(w, &reg->period_high);
	/* ensure previous writes are before the next */
	wmb();
	/* dummy write-flushing read */
	w = ioread32(&reg->identifier);

	w = PERF_CTRL_RUN | PERF_CTRL_IE;
	dbg_perf("iowrite32(0x%08x to 0x%p) (control)\n", w,
		(void *)&reg->control);
	write_register(w, &reg->control);
	/* dummy write-flushing read */
	w = read_register(&reg->identifier);
fail_identifier:
	return rc;
}

static int performance_submit(struct lancero_dev *lro, int dir_to_dev)
{
	u8 *buffer_virt;
	/* bus address */
	dma_addr_t buffer_bus;
	int size = 32 * PAGE_SIZE;
	struct lancero_transfer *transfer;
	int fpga_addr = 0;
	buffer_virt = pci_alloc_consistent(lro->pci_dev, size, &buffer_bus);
	//printk(KERN_DEBUG "buffer_virt = %p\n", buffer_virt);

	/* allocate transfer data structure */
	transfer = kzalloc(sizeof(struct lancero_transfer), GFP_KERNEL);
	BUG_ON(!transfer);
	/* 0 = write engine (to_dev=0) , 1 = read engine (to_dev=1) */
	transfer->dir_to_dev = dir_to_dev;
	/* set number of descriptors */
	transfer->desc_num = 1;
	/* allocate descriptor list */
	transfer->desc_virt = lancero_desc_alloc(lro->pci_dev,
		transfer->desc_num, &transfer->desc_bus, NULL);
	BUG_ON(!transfer->desc_virt);
	/* fill in descriptor entry with transfer details */
	lancero_desc_set(transfer->desc_virt, buffer_bus,
		fpga_addr, size);
	/* stop engine and request interrupt on last descriptor */
	lancero_desc_control(transfer->desc_virt, 0 /*LANCERO_DESC_COMPLETED | LANCERO_DESC_STOP*/);
	/* create a linked loop */
	lancero_desc_link(transfer->desc_virt, transfer->desc_virt, transfer->desc_bus);
	transfer->cyclic = 1;
	/* dump transfer for debugging */
	dump_transfer(transfer);
	/* initialize wait queue */
	init_waitqueue_head(&transfer->wq);
#if 0
	/* make transfer cyclic */
	lancero_transfer_cyclic(transfer);
#endif
	/* */
	printk(KERN_DEBUG "Queueing SGDMA I/O %s request for %d seconds performance measurement.\n", dir_to_dev? "write (to device)" : "read (from device)", performance_interval);
	queue_transfer(lro->engine[dir_to_dev], transfer);
	return 0;
}

static int performance_run(struct lancero_dev *lro, int dir_to_dev)
{
  printk(KERN_DEBUG "dir_to_dev = %d\n", dir_to_dev);

  if (dir_to_dev < 0) return -1;
  if (dir_to_dev < 2) {
    /* start gated performance measurement */
    performance_start(lro, dir_to_dev?LANCERO_OFS_DMA_READ_PERF:LANCERO_OFS_DMA_WRITE_PERF, performance_interval * 1000);
    /* submit work on selected engine */
    performance_submit(lro, dir_to_dev);
    lancero_read_status(lro->engine[dir_to_dev], 0);
    printk(KERN_DEBUG "Engine status = 0x%08lx\n", lro->engine[dir_to_dev]->status);
  } else {
    /* start dual gated performance measurement */
	  performance_start(lro, LANCERO_OFS_DMA_WRITE_PERF, performance_interval * 1000);
	  performance_start(lro, LANCERO_OFS_DMA_READ_PERF, performance_interval * 1000);
    /* submit work on each engine */
    performance_submit(lro, /*dir_to_dev =*/0);
	  performance_submit(lro, /*dir_to_dev =*/1);
  }
  return 0;
}

/* performance_read -- read the performance measurement module
 */
static int performance_read(struct lancero_dev *lro, int offset)
{
	int rc = 0;
	u32 w;
	u64 performance, period, bandwidth, wait;
	struct performance_regs *reg = (struct performance_regs *)(lro->bar[bridge_bar] + offset);

	dbg_perf("ioread32(0x%p).\n", &reg->identifier);
	w = read_register(&reg->identifier);
	dbg_perf("ioread32(0x%p) returned 0x%08x.\n", &reg->identifier, w);
	w &= 0x00ffff00UL;
	if ((w != 0x00490000) && (w != 0x00d30000) && (w != 0x00d20000)) {
		printk(KERN_DEBUG "Performance identifier not found (found 0x%08x expected 0x00d*0001).\n", w);
		rc = -1;
		goto fail_identifier;
	}
	w = read_register(&reg->performance_high);
	dbg_perf("ioread32(0x%p) returned 0x%08x.\n", &reg->performance_high, w);
	performance = (u64)w;
	performance <<= 32;
	w = read_register(&reg->performance_low);
	dbg_perf("ioread32(0x%p) returned 0x%08x.\n", &reg->performance_low, w);
	performance |= (u64)w;
	printk(KERN_DEBUG "Performance counter = %llu\n", (unsigned long long)performance);

	w = read_register(&reg->period_high);
	dbg_perf("ioread32(0x%p) returned 0x%08x.\n", &reg->period_high, w);
	period = (u64)w;
	period <<= 32;

	w = read_register(&reg->period_low);
	dbg_perf("ioread32(0x%p) returned 0x%08x.\n", &reg->period_low, w);
	period |= (u64)w;

	bandwidth = performance;
	do_div(bandwidth, performance_interval);
#if 1
	printk(KERN_DEBUG "%s performance = %llu bytes in %u seconds = %llu bytes/second.\n",
		(offset == LANCERO_OFS_DMA_READ_PERF)? "Read" : "Write",
		(unsigned long long)performance, performance_interval,
		(unsigned long long)bandwidth);
#endif
	w = read_register(&reg->wait_high);
	dbg_perf("ioread32(0x%p) returned 0x%08x.\n", &reg->wait_high, w);
	wait = (u64)w;
	wait <<= 32;

	w = read_register(&reg->period_low);
	dbg_perf("ioread32(0x%p) returned 0x%08x.\n", &reg->wait_low, w);
	wait |= (u64)w;
	printk(KERN_DEBUG "wait = %llu\n", (unsigned long long)wait);

	w = 0;
	dbg_perf("iowrite32(0x%08x to 0x%p) (control)\n", w,
		(void *)&reg->control);
	write_register(w, &reg->control);

	w = 0x1UL;
	dbg_perf("iowrite32(0x%08x to 0x%p) (status)\n", w,
		(void *)&reg->status);
	write_register(w, &reg->status);
	ioread32(&reg->status);
fail_identifier:
	return rc;
}
/**
 * lancero_read_status() - read status of SG DMA engine (optionally reset)
 *
 * Stores status in engine->status.
 *
 * @return -1 on failure, status register otherwise
 */
static u32 lancero_read_status(struct lancero_engine *engine, int clear)
{
	u32 value, w, desc_completed;
	BUG_ON(!engine);
	dbg_io("ioread32(0x%p).\n", &engine->regs->identifier);
	w = read_register(&engine->regs->identifier);
	dbg_io("ioread32(0x%p) returned 0x%08x.\n", &engine->regs->identifier, w);
	w &= 0x00ffff00UL;
	if ((w != 0x004b0000UL) && (w != 0x00c10000UL) && (w != 0x00c20000UL)) {
		printk(KERN_ERR "SGDMA controller identifier not found (found 0x%08x expected 0xad4bXX01).\n", w);
		value = 0xffffffff;
		goto fail_identifier;
	}
	/* extra debugging; inspect complete engine set of registers */
	w = read_register(&engine->regs->status);
	dbg_io("ioread32(0x%p) returned 0x%08x.\n", &engine->regs->status, w);
	w = read_register(&engine->regs->control);
	dbg_io("ioread32(0x%p) returned 0x%08x.\n", &engine->regs->control, w);
	w = read_register(&engine->regs->first_desc);
	dbg_io("ioread32(0x%p) returned 0x%08x.\n", &engine->regs->first_desc, w);
	w = read_register(&engine->regs->first_desc_adjacent);
	dbg_io("ioread32(0x%p) returned 0x%08x.\n", &engine->regs->first_desc_adjacent, w);
	w = read_register(&engine->regs->completed_desc_count);
	dbg_io("ioread32(0x%p) returned 0x%08x.\n", &engine->regs->completed_desc_count, w);
	w = read_register(&engine->regs->completed_desc_bytes);
	dbg_io("ioread32(0x%p) returned 0x%08x.\n", &engine->regs->completed_desc_bytes, w);
	w = read_register(&engine->regs->first_desc_hi);
	dbg_io("ioread32(0x%p) returned 0x%08x.\n", &engine->regs->first_desc_hi, w);

	/* read status register */
	dbg_io("Status of SG DMA %s engine:\n", engine->name);
	dbg_io("ioread32(0x%p).\n", &engine->regs->status);
	value = engine->status = read_register(&engine->regs->status);
	dbg_io("status = 0x%08x: %s%s%s%s%s%s%s%s\n", (u32)engine->status,
		(value & LANCERO_STAT_BUSY) ? "BUSY ": "IDLE ",
		(value & LANCERO_STAT_DESCRIPTOR_STOPPED) ? "DESCRIPTOR_STOPPED ": "",
		(value & LANCERO_STAT_DESCRIPTOR_COMPLETED) ? "DESCRIPTOR_COMPLETED ": "",
		(value & LANCERO_STAT_MAGIC_STOPPED) ? "MAGIC_STOPPED ": "",
		(value & LANCERO_STAT_FETCH_STOPPED) ? "FETCH_STOPPED ": "",
		(value & LANCERO_STAT_IDLE_STOPPED) ? "IDLE_STOPPED": "",
		(value & LANCERO_STAT_NONALIGNED_STOPPED) ? "NONALIGNED_STOPPED": "",
		(value & LANCERO_STAT_SEQUENCE_STOPPED) ? "SEQUENCE_STOPPED": "");

	if (value & LANCERO_STAT_BUSY) {
		/* read number of completed descriptors after engine start */
		desc_completed = read_register(&engine->regs->completed_desc_count);
		dbg_io("desc_completed = %d\n", desc_completed);
	}

	/* clear hardware status register */
	if (clear) {
		dbg_io("Clearing status of SG DMA %s engine:\n", engine->name);
		w = LANCERO_STAT_BUSY |
			LANCERO_STAT_DESCRIPTOR_COMPLETED |
			LANCERO_STAT_DESCRIPTOR_STOPPED |
			LANCERO_STAT_MAGIC_STOPPED |
			LANCERO_STAT_FETCH_STOPPED |
			LANCERO_STAT_IDLE_STOPPED |
			LANCERO_STAT_NONALIGNED_STOPPED |
			LANCERO_STAT_SEQUENCE_STOPPED;
		dbg_io("iowrite32(%08x, 0x%p).\n", w, &engine->regs->status);
		/* clear status register bits */
		write_register(w, &engine->regs->status);
	}
fail_identifier:
	return value;
}

/**
 * lancero_stop() - stop an SG DMA engine
 *
 */
static void lancero_stop(struct lancero_engine *engine)
{
	u32 w;
	printk(KERN_DEBUG "lancero_stop(engine=%p)\n", engine);
	BUG_ON(!engine);
	/* reset RUN_STOP bit */
	w = read_register(&engine->regs->control);
	printk(KERN_DEBUG "ioread32() done\n");

	w &= ~LANCERO_CTRL_RUN_STOP;
	w |= LANCERO_CTRL_IE_IDLE_STOPPED;
	dbg_io("Stopping SG DMA %s engine; writing 0x%08x to 0x%p.\n",
		engine->name, w, (u32 *)&engine->regs->control);
	write_register(w, &engine->regs->control);
	/* dummy read of status register to flush all previous writes */
	read_register(&engine->regs->status);
	printk(KERN_DEBUG "lancero_stop(%s) done\n", engine->name);
}

/**
 * engine_reset() - reset an SG DMA engine
 *
 */
static void engine_reset(struct lancero_engine *engine)
{
	u32 w;

	struct lancero_transfer *transfer = 0;

	/* lock the engine */
	spin_lock(&engine->lock);
	printk(KERN_DEBUG "engine_reset(engine=%p)\n", engine);
	BUG_ON(!engine);
	w = LANCERO_CTRL_RST;
	dbg_io("Resetting SG DMA %s engine; writing 0x%08x to 0x%p.\n",
		engine->name, w, (u32 *)&engine->regs->control);
	write_register(w, &engine->regs->control);
	/* flush previous writes */
	read_register(&engine->regs->status);
	/* zero RUN_STOP bit */
	write_register(0, &engine->regs->control);
	/* read and clear engine status to remove pending interrupts */
	lancero_read_status(engine, 1);
	printk(KERN_DEBUG "engine_reset(%s) done\n", engine->name);

	/* transfers on queue? */
	if (!list_empty(&engine->transfer_list)) {
		printk(KERN_DEBUG "Removing transfer queue from this engine.\n");
		/* pick first transfer on the queue (was submitted to the engine) */
		transfer = list_entry(engine->transfer_list.next,
			struct lancero_transfer, entry);
	}
	/* iterate over all the transfers completed by the engine,
	 * except for the last (i.e. use > instead of >=). */
	while (transfer) {
		/* remove completed transfer from list */
		list_del(engine->transfer_list.next);
		/* mark transfer as succesfully completed */
		transfer->state = TRANSFER_STATE_FAILED;
		/* asynchronous I/O? */
		if ((transfer->iocb) && (transfer->last_in_request)) {
				struct kiocb *iocb = transfer->iocb;
				ssize_t done = transfer->size_of_request;
				printk(KERN_DEBUG "Freeing (async I/O request) last transfer %p, iocb %p\n", transfer, transfer->iocb);
				free_transfer(engine->lro, transfer);
				transfer = NULL;
				printk(KERN_DEBUG "Completing async I/O iocb %p with size %d\n", iocb, (int)done);
				/* indicate I/O completion XXX res, res2 */
				AIO_COMPLETE(iocb, done, 0);
		/* synchronous I/O? */
		} else {
			/* awake task on transfer's wait queue */
			wake_up_interruptible(&transfer->wq);
		}
		/* if exists, get the next transfer on the list */
		if (!list_empty(&engine->transfer_list)) {
			transfer = list_entry(engine->transfer_list.next,
				struct lancero_transfer, entry);
			printk(KERN_DEBUG "Non-completed transfer %p\n", transfer);
		/* no further transfers? */
		} else {
			transfer = NULL;
		}
	}
	engine->running = 0;
	/* unlock the engine */
	spin_unlock(&engine->lock);
}

/**
 * engine_cyclic_stop() - stop a cyclic transfer running on an SG DMA engine
 *
 */
static struct lancero_transfer *engine_cyclic_stop(struct lancero_engine *engine)
{
	u32 w;
	/* lock the engine */
	spin_lock(&engine->lock);

	struct lancero_transfer *transfer = 0;
	/* transfers on queue? */
	if (!list_empty(&engine->transfer_list)) {
		/* pick first transfer on the queue (was submitted to the engine) */
		transfer = list_entry(engine->transfer_list.next, struct lancero_transfer, entry);
    		if (transfer->cyclic) {
				printk(KERN_DEBUG "Stopping cyclic transfer on %s engine\n", engine->name);
				/* make sure the service handler sees the correct transfer state */
				transfer->cyclic = 0;
				/* set STOP flag and interrupt on completion, on the last descriptor */
				lancero_desc_control(transfer->desc_virt + transfer->desc_num - 1, LANCERO_DESC_COMPLETED | LANCERO_DESC_STOP);
    		}
  	}
	/* unlock the engine */
	spin_unlock(&engine->lock);
    return transfer;
}

/* engine_shutdown -- request engine comes to full stop */
static void engine_shutdown(struct lancero_engine *engine)
{
	/* lock the engine */
	spin_lock(&engine->lock);
	/* make sure engine goes full stop */
	engine->shutdown |= ENGINE_SHUTDOWN_REQUEST;
	/* unlock the engine */
	spin_unlock(&engine->lock);
}

static void engine_teardown(struct lancero_engine *engine)
{
	/* lock the engine */
	spin_lock(&engine->lock);
	/* make sure engine does not start again */
	engine->shutdown |= ENGINE_SHUTDOWN_REQUEST | ENGINE_SHUTDOWN_TEARDOWN;
	/* unlock the engine */
	spin_unlock(&engine->lock);
}

static void engine_enable(struct lancero_engine *engine)
{
	/* lock the engine */
	spin_lock(&engine->lock);
	engine->shutdown = 0;
	/* unlock the engine */
	spin_unlock(&engine->lock);
}

/**
 * engine_start() - start an idle engine with its first transfer on queue
 *
 * The engine will run and process all transfers that are queued using
 * transfer_queue() and thus have their descriptor lists chained.
 *
 * During the run, new transfers will be processed if transfer_queue() has
 * chained the descriptors before the hardware fetches the last descriptor.
 * A transfer that was chained too late will invoke a new run of the engine
 * initiated from the engine_service() routine.
 *
 * The engine must be idle and at least one transfer must be queued.
 * This function does not take locks; the engine spinlock must already be taken.
 *
 */
static struct lancero_transfer *engine_start(struct lancero_engine *engine)
{
	struct lancero_transfer *transfer;
	u32 w;
	int extra_adj = 0;
	/* engine must be idle */
	BUG_ON(engine->running);
	/* engine transfer queue must not be empty */
	BUG_ON(list_empty(&engine->transfer_list));
	/* inspect first transfer queued on the engine */
	transfer = list_entry(engine->transfer_list.next,
		struct lancero_transfer, entry);
	BUG_ON(!transfer);

	/* engine is no longer shutdown */
	engine->shutdown = 0;

	printk(KERN_DEBUG "engine_start(%s): transfer=0x%p.\n", engine->name, transfer);
	/* XXX make sure bus address in within 4GB address space XXX */
	WARN_ON((transfer->desc_bus >> 16) >> 16);
	/* write bus address of first descriptor in Root Complex memory */
	w = (u32)transfer->desc_bus;

#if FPGA_DESCRIPTORS
	/* on-chip */
	w = 0;
#endif

	/* initialize number of descriptors of dequeued transfers */
	engine->desc_dequeued = 0;

#if 0 /* test Rd request completion timeout in Control Bridge */
	/* @todo XXX Test with invalid address to test Rd request completion
	 * timeout in the descriptor Control Bridge */
	w = 0xd1fffff0;
#endif
	dbg_io("iowrite32(0x%08x to 0x%p) (first_desc)\n", w,
		(void *)&engine->regs->first_desc);
	/* write bus address of transfer first descriptor */
	write_register(w, &engine->regs->first_desc);
	write_register(0, &engine->regs->first_desc_hi);

	if (transfer->desc_adjacent > 0) {
		extra_adj = transfer->desc_adjacent - 1;
		if (extra_adj > MAX_EXTRA_ADJ)
			extra_adj = MAX_EXTRA_ADJ;
	}
	dbg_io("iowrite32(0x%08x to 0x%p) (first_desc_adjacent)\n",
		extra_adj, (void *)&engine->regs->first_desc_adjacent);
	write_register(extra_adj, &engine->regs->first_desc_adjacent);

	dbg_io("ioread32(0x%p) (dummy read flushes writes).\n", &engine->regs->status);
	/* dummy read of status register to flush all previous writes */
	read_register(&engine->regs->status);
	wmb();

	/* write control register of SG DMA engine */
	w = (u32)LANCERO_CTRL_RUN_STOP;
	w |= (u32)LANCERO_CTRL_IE_DESCRIPTOR_STOPPED;
	w |= (u32)LANCERO_CTRL_IE_DESCRIPTOR_COMPLETED;
	w |= (u32)LANCERO_CTRL_IE_MAGIC_STOPPED;
	w |= (u32)LANCERO_CTRL_IE_IDLE_STOPPED;
	w |= (u32)LANCERO_CTRL_IE_NONALIGNED_STOPPED;

	dbg_io("iowrite32(0x%08x to 0x%p) (control)\n", w,
		(void *)&engine->regs->control);
	/* start the engine */
	write_register(w, &engine->regs->control);

	/* dummy read of status register to flush all previous writes */
	w = read_register(&engine->regs->status);
	dbg_io("ioread32(0x%p) = 0x%lx (dummy read flushes writes).\n",
		&engine->regs->status, (unsigned long)w);
#if 1
	/* read status register and report */
	lancero_read_status(engine, 0);
#endif
	printk(KERN_DEBUG "%s engine 0x%p now running\n", engine->name, engine);
	/* remember the engine is running */
	engine->running = 1;
	return transfer;
}

/* engine_initialize -- Initialize the engine for use, read capabilities */
static int engine_initialize(struct lancero_dev *lro, int interrupts_offset)
{
	void *reg = lro->bar[bridge_bar] + interrupts_offset;
	u32 w;
	int rc = 0;
	printk(KERN_DEBUG "Read register at BAR %d, address 0x%08x.\n", bridge_bar, reg);
	w = read_register(reg + 0x00);
	/* not a write nor a read engine found? */
	if (((w & 0x00ffff00UL) != 0x00c10000UL) && ((w & 0x00ffff00UL) != 0x00c20000UL)) {
		printk(KERN_DEBUG "Engine identifier not found (found 0x%08x expected 0xC100/0xC200).\n", w);
		rc = -1;
		goto fail_identifier;
	}
	printk(KERN_DEBUG "Engine identifier found 0x%08x with version %u.\n", w, w & 0xffUL);

	/* before version 2, 64-bit DMA is not available */
	if ((w & 0xffUL) < 2UL) lro->capabilities &= ~(CAP_64BIT_DMA | CAP_64BIT_DESC);
	/* clear all interrupt event enables, stop engine */
	w = 0x0UL;
	printk(KERN_DEBUG "Set engine controller enable mask: 0x%08x.\n", w);
	write_register(w, reg + 0x04);
fail_identifier:
	return rc;
}

static int engine_version(struct lancero_dev *lro, int engine_offset)
{
	void *reg = lro->bar[bridge_bar] + engine_offset;
	u32 w;
	int rc = 0;
	w = read_register(reg + 0x00);
	/* not a write nor a read engine found? */
	if (((w & 0x00ffff00UL) != 0x00c10000UL) && ((w & 0x00ffff00UL) != 0x00c20000UL)) {
		printk(KERN_DEBUG "Engine identifier not found (found 0x%08x expected 0xC100/0xC200).\n", w);
		rc = -1;
		goto fail_identifier;
	}
	/* engine version */
	rc = w & 0xff;
fail_identifier:
	return rc;
}

/**
 * engine_service() - service an SG DMA engine
 *
 * @engine pointer to struct lancero_engine
 *
 */
static int engine_service(struct lancero_engine *engine)
{
	u32 desc_completed;
	struct lancero_transfer *transfer = 0, *transfer_started;
	int lock_contended = spin_is_locked(&engine->lock);
	int cpu;

	printk(KERN_DEBUG "service() spinlock is %s\n", lock_contended? "locked": "unlocked");

	/* lock the engine */
	spin_lock(&engine->lock);
	printk(KERN_DEBUG "service() got lock\n");
	/* determine current cpu */
	cpu = get_cpu();
	put_cpu();
	printk(KERN_DEBUG "service() got cpu\n");
	/* engine was not started by the driver? */
	if (!engine->running) {
		engine->prev_cpu = cpu;
		printk(KERN_DEBUG "service() engine was not running\n");
		spin_unlock(&engine->lock);
		return 0;
	}

	printk(KERN_DEBUG "service() got lock\n");

	dbg_perf("service(): spinlock was %scontended, previous owner cpu #%d, this cpu #%d.\n",
		lock_contended?"":"not ", engine->prev_cpu, cpu);

	/* read status register */
	lancero_read_status(engine, 1);

	/* engine was running but is no longer busy? */
	if (engine->running && !(engine->status & LANCERO_STAT_BUSY)) {
		dbg_tfr("service(): engine just went idle, resetting RUN_STOP.\n");
		lancero_stop(engine);
		engine->running = 0;
	}

#define LANCERO_STAT (LANCERO_STAT_BUSY | \
LANCERO_STAT_DESCRIPTOR_COMPLETED | \
LANCERO_STAT_DESCRIPTOR_STOPPED | \
LANCERO_STAT_MAGIC_STOPPED | \
LANCERO_STAT_NONALIGNED_STOPPED | \
LANCERO_STAT_SEQUENCE_STOPPED)

	/* engine event which can forward our work? */
	if (engine->status & (LANCERO_STAT_DESCRIPTOR_COMPLETED |
		LANCERO_STAT_DESCRIPTOR_STOPPED | LANCERO_STAT_MAGIC_STOPPED |
		LANCERO_STAT_IDLE_STOPPED | LANCERO_STAT_NONALIGNED_STOPPED | LANCERO_STAT_SEQUENCE_STOPPED)) {

#if 0
		/* completed descriptor with irq flag and not busy? */
		if ((engine->status & LANCERO_STAT) == LANCERO_STAT_DESCRIPTOR_COMPLETED) {
			dbg_tfr("%s engine completed irq descriptor and stopped.\n", engine->name);
		/* completed chain and not busy? */
		} else if ((engine->status & LANCERO_STAT) == LANCERO_STAT_DESCRIPTOR_STOPPED) {
			dbg_tfr("%s engine completed chain and stopped.\n", engine->name);
		/* completed descriptor with irq flag and still busy? */
		} else if ((engine->status & LANCERO_STAT) == (LANCERO_STAT_DESCRIPTOR_COMPLETED | LANCERO_STAT_BUSY)) {
			dbg_tfr("%s engine completed irq descriptor while running.\n", engine->name);
		} else {
			dbg_tfr("%s engine interrupts, status 0x%08x.\n",
				engine->name, engine->status);
		}
#endif
		/* read number of completed descriptors after engine start */
		desc_completed = read_register(&engine->regs->completed_desc_count);
		dbg_tfr("engine->regs->completed_desc_count = %d\n", desc_completed);

		/* transfers on queue? */
		if (!list_empty(&engine->transfer_list)) {
			/* pick first transfer on the queue (was submitted to the engine) */
			transfer = list_entry(engine->transfer_list.next,
				struct lancero_transfer, entry);
#if 0
			dbg_io("read32(0x%p) (first_desc) = 0x%08x, transfer->desc_bus = 0x%08x\n",
				(void *)&engine->regs->first_desc, value, transfer->desc_bus);
#endif
			dbg_tfr("head of queue transfer 0x%p has %d descriptors, engine completed %d desc, %d not yet dequeued.\n",
				transfer, (int)transfer->desc_num, (int)desc_completed, (int)desc_completed - engine->desc_dequeued);
		} else {
			dbg_tfr("no transfers on queue, but engine completed %d descriptors?!\n",
				 (int)desc_completed);
		}

		/* account for already dequeued transfers during this engine run */
		desc_completed -= engine->desc_dequeued;

		/* iterate over all the transfers completed by the engine,
		 * except for the last (i.e. use > instead of >=). */
		while (transfer && (!transfer->cyclic) &&
			(desc_completed > transfer->desc_num)) {
			/* remove this transfer from desc_completed */
			desc_completed -= transfer->desc_num;
			dbg_tfr("%s engine completed non-cyclic transfer 0x%p (%d desc).\n",
				engine->name, transfer, transfer->desc_num);
			/* remove completed transfer from list */
			list_del(engine->transfer_list.next);
			/* add to dequeued number of descriptors during this run */
			engine->desc_dequeued += transfer->desc_num;
			/* mark transfer as succesfully completed */
			transfer->state = TRANSFER_STATE_COMPLETED;
			/* asynchronous I/O? */
			if ((transfer->iocb) && (transfer->last_in_request)) {
					struct kiocb *iocb = transfer->iocb;
					ssize_t done = transfer->size_of_request;
					printk(KERN_DEBUG "Freeing (async I/O request) last transfer %p, iocb %p\n", transfer, transfer->iocb);
					free_transfer(engine->lro, transfer);
					transfer = NULL;
					printk(KERN_DEBUG "Completing async I/O iocb %p with size %d\n", iocb, (int)done);
					/* indicate I/O completion XXX res, res2 */
					AIO_COMPLETE(iocb, done, 0);
			/* synchronous I/O? */
			} else {
				/* awake task on transfer's wait queue */
				wake_up_interruptible(&transfer->wq);
			}
			/* if exists, get the next transfer on the list */
			if (!list_empty(&engine->transfer_list)) {
				transfer = list_entry(engine->transfer_list.next,
					struct lancero_transfer, entry);
				printk(KERN_DEBUG "Non-completed transfer %p\n", transfer);
			/* no further transfers? */
			} else {
				transfer = NULL;
			}
		}
		/* inspect the current transfer */
		if (transfer) {
			/* engine stopped? (i.e. not busy and stop reason known? */
			if (((engine->status & LANCERO_STAT_BUSY) == 0) &&
				(engine->status & (LANCERO_STAT_MAGIC_STOPPED |
				LANCERO_STAT_DESCRIPTOR_STOPPED |
				LANCERO_STAT_IDLE_STOPPED |
				LANCERO_STAT_NONALIGNED_STOPPED | LANCERO_STAT_SEQUENCE_STOPPED))) {
				dbg_tfr("running %s engine has stopped\n", engine->name);
			}

			/* the engine still working on current transfer? */
			if (engine->status & LANCERO_STAT_BUSY) {
				dbg_tfr("running %s engine was %d descriptors into transfer 0x%p (with %d desc)\n",
					engine->name, desc_completed, transfer, transfer->desc_num);
			/* engine has stopped  */
			} else {
				/* the engine failed on current transfer? */
				if (engine->status & (LANCERO_STAT_MAGIC_STOPPED | LANCERO_STAT_NONALIGNED_STOPPED | LANCERO_STAT_SEQUENCE_STOPPED))  {
					dbg_tfr("aborted %s engine was %d descriptors into transfer 0x%p (with %d desc)\n",
						engine->name, desc_completed, transfer, transfer->desc_num);
					/* mark transfer as succesfully completed */
					transfer->state = TRANSFER_STATE_FAILED;
					lancero_stop(engine);
				/* the engine stopped on current transfer? */
				} else {
					if (desc_completed < transfer->desc_num) {
						transfer->state = TRANSFER_STATE_FAILED;
						printk(KERN_DEBUG "Engine stopped half-way transfer %p\n", transfer);
					} else {
						dbg_tfr("stopped %s engine completed transfer 0x%p (%d desc), desc_completed = %d\n",
							engine->name, transfer, transfer->desc_num, desc_completed);
						if (!transfer->cyclic) {
							/* if the engine stopped on this transfer, it should be the last */
							WARN_ON(desc_completed > transfer->desc_num);
						}
						/* mark transfer as succesfully completed */
						transfer->state = TRANSFER_STATE_COMPLETED;
					}
				}
				/* remove completed transfer from list */
				list_del(engine->transfer_list.next);
				/* add to dequeued number of descriptors during this run */
				engine->desc_dequeued += transfer->desc_num;
				/* asynchronous I/O? */
				if ((transfer->iocb) && (transfer->last_in_request)) {
					struct kiocb *iocb = transfer->iocb;
					ssize_t done = transfer->size_of_request;
					printk(KERN_DEBUG "Freeing (async I/O request) last transfer %p, iocb %p\n", transfer, transfer->iocb);
					free_transfer(engine->lro, transfer);
					transfer = NULL;
					printk(KERN_DEBUG "Completing async I/O iocb %p with size %d\n", iocb, (int)done);
					/* indicate I/O completion XXX res, res2 */
					AIO_COMPLETE(iocb, done, 0);
				/* synchronous I/O? */
				} else {
					/* awake task on transfer's wait queue */
					wake_up_interruptible(&transfer->wq);
				}
			}
		}
		/* engine stopped? */
		if (!engine->running) {
			/* engine was requested to be shutdown? */
			if (engine->shutdown & ENGINE_SHUTDOWN_REQUEST) {
				engine->shutdown |= ENGINE_SHUTDOWN_IDLE;
				/* awake task on engine's shutdown wait queue */
				wake_up_interruptible(&engine->shutdown_wq);
			/* more pending transfers? */
			} else if (!list_empty(&engine->transfer_list)) {
				/* (re)start engine */
				transfer_started = engine_start(engine);
				dbg_tfr("re-started %s engine with pending transfer 0x%08x\n",
					engine->name, (u32)transfer_started);
			} else {
				dbg_tfr("no pending transfers, %s engine remains idle.\n", engine->name);
			}
		/* engine is still running? */
		} else {
			if (list_empty(&engine->transfer_list)) {
#if LANCERO_PERFORMANCE_TEST
				dbg_tfr("no transfers on queue but %s engine is running; cyclic?\n", engine->name);
				//lancero_stop(engine);
#else
				dbg_tfr("no transfers on queue but %s engine is running?! BUSY is stuck?!\n", engine->name);
				WARN_ON(1);
#endif
			}
		}
	/* engine did not complete a transfer */
	} else {
		dbg_tfr("%s engine triggered unknown interrupt 0x%08x\n", engine->name, engine->status);
	}
	/* remember last lock holder */
	engine->prev_cpu = cpu;
	spin_unlock(&engine->lock);
	return 0;
}

/* engine_service_work */
static void engine_service_work(struct work_struct *work)
{
	struct lancero_engine *engine;
	engine = container_of(work, struct lancero_engine, work);
	BUG_ON(engine->magic != MAGIC_ENGINE);
	printk(KERN_DEBUG "engine_service_work() for %s engine %p\n", engine->name, engine);
	engine_service(engine);
	enable_interrupts(engine->lro, LANCERO_OFS_INT_CTRL, engine->interrupt);
}

#if LANCERO_FRAMEBUFFER
static u32 lancerofb_isr(struct lancero_dev *lro)
{
	u32 w;
	//printk(KERN_DEBUG "lancerofb_isr(0x%08x)\n", lro);
	static int limit_count = 0;
	struct lancerofb_regs *reg = (struct lancerofb_regs *)(lro->bar[USER_BAR] + LANCERO_OFS_FRAMEBUFFER);
	void *reg2 = lro->bar[bridge_bar] + LANCERO_OFS_INT_CTRL;

#if 0
	printk(KERN_DEBUG "reg=0x%08x\n", reg);

	w = read_register(&reg->status);
	printk(KERN_INFO "frame status = 0x%08x\n", w);
#endif
	/* clear frame interrupt */
	write_register(1/*FRAME_IRQ*/, &reg->status);
	/* read frame counter */
	lro->fb_count = read_register(&reg->count);
	if (limit_count < 4) {
		printk(KERN_DEBUG "FB IRQ, FRAME #%d\n", lro->fb_count);
		limit_count++;
		if (limit_count >= 4) {
			printk(KERN_DEBUG "(WILL NOT REPORT FURTHER FB IRQs)\n");
		}
	}
#if 0
	w = read_register(&reg->status);
	printk(KERN_INFO "frame status = 0x%08x\n", w);

	/* stop on error */
	if (w & 2) {
		write_register(0, &reg->control);
	}
	/* read interrupt controller events */
	w = read_register(reg2 + 0x08);
	printk(KERN_DEBUG "interrupt request = 0x%08x\n", w);
#endif

	return lro->fb_count;
}
#endif

/*
 * lancero_isr() - Interrupt handler
 *
 * @dev_id pointer to lancero_dev
 */
static irqreturn_t lancero_isr(int irq, void *dev_id)
{
	u32 w, fc;
	struct lancero_dev *lro;
	BUG_ON(!dev_id);
	lro = (struct lancero_dev *)dev_id;
	if (!lro)
		return IRQ_NONE;

	w = read_interrupts(lro, LANCERO_OFS_INT_CTRL);

	/* disable engine interrupts that fired, the work will enable these again */
	interrupts_disable(lro, LANCERO_OFS_INT_CTRL, w & (LANCERO_INT_DMA_READ | LANCERO_INT_DMA_WRITE));
#if 1
	if (lro->irq_count < 10)
		printk(KERN_DEBUG "IRQ #%d, interrupts 0x%08lx\n", lro->irq_count, w);
#endif
	/* write engine? */
	if (w & LANCERO_INT_DMA_WRITE) {
		printk(KERN_DEBUG "schedule_work(engine=0x%p->work))\n", lro->engine[0]);
		schedule_work(&lro->engine[0]->work);
	}
	/* read engine? */
	if (w & LANCERO_INT_DMA_READ) {
		//printk(KERN_DEBUG "Read Engine interrupt.\n");

		//lancero_read_status(lro->engine[1], 0);

		printk(KERN_DEBUG "schedule_work(engine=0x%p->work))\n", lro->engine[1]);
		schedule_work(&lro->engine[1]->work);
	}

	/* write performance module? */
	if (w & LANCERO_INT_DMA_WRITE_PERF) {
		printk(KERN_DEBUG "Write performance interrupt.\n");
		lancero_read_status(lro->engine[0], 0);
		engine_cyclic_stop(lro->engine[0]);
		performance_read(lro, LANCERO_OFS_DMA_WRITE_PERF);
	}
	/* read performance module? */
	if (w & LANCERO_INT_DMA_READ_PERF) {
		lancero_read_status(lro->engine[1], 0);
		engine_cyclic_stop(lro->engine[1]);
		printk(KERN_DEBUG "Read performance interrupt.\n");
		performance_read(lro, LANCERO_OFS_DMA_READ_PERF);
	}
#if LANCERO_FRAMEBUFFER
	/* framebuffer? */
	if (w & LANCERO_INT_FRAMEBUFFER) {
                fc = lancerofb_isr(lro);
	}
#else
	/* read performance module? */
	if (w & LANCERO_INT_DMA_READ_TESTER) {
		printk(KERN_DEBUG "Read tester found data mismatch.\n");
	}
#endif
	/* new irq events? */
	if ((lro->events_irq | w) != lro->events_irq) {
        //printk(KERN_DEBUG "wake_up_interruptible() because 0x%08lx | 0x%08lx = 0x%08lx\n", lro->events_irq, w, lro->events_irq | w);
		/* accumulate events into the pending mask */
		lro->events_irq |= w;
		wake_up_interruptible(&lro->events_wq);
	}
#if LANCERO_FRAMEBUFFER
        //printk(KERN_DEBUG "FRAME #%d\n", fc);
#endif
	lro->irq_count++;
	return IRQ_HANDLED;
}

/*
 * Unmap the BAR regions that had been mapped earlier using map_bars()
 */
static void unmap_bars(struct lancero_dev *lro, struct pci_dev *dev)
{
	int i;
	for (i = 0; i < LANCERO_BAR_NUM; i++) {
		/* is this BAR mapped? */
		if (lro->bar[i]) {
			/* unmap BAR */
			pci_iounmap(dev, lro->bar[i]);
			/* mark as unmapped */
			lro->bar[i] = NULL;
		}
	}
}

/* map_bars() -- map device regions into kernel virtual address space
 *
 * Map the device memory regions into kernel virtual address space after
 * verifying their sizes respect the minimum sizes needed, given by the
 * bar_map_sizes[] array.
 */
static int __devinit map_bars(struct lancero_dev *lro, struct pci_dev *dev)
{
	int rc;
	int i;

  /* map full user BAR by default */
  bar_map_sizes[USER_BAR] = pci_resource_len(dev, USER_BAR);
  
  /* use specified map size instead for user BAR? */
  if (bar_map_size > 0) {
    /* within BAR size? */
    if (bar_map_size < bar_map_sizes[USER_BAR])
      bar_map_sizes[USER_BAR] = bar_map_size;
    else {
	     printk(KERN_DEBUG "Cannot map more than the actual BAR size, ignoring "
        "the bar_map_size option to %llu bytes.\n", (unsigned long long)bar_map_sizes[USER_BAR]);
      return -1;
    }
  }

	/* iterate through all the BARs */
	for (i = 0; i < LANCERO_BAR_NUM; i++) {
		resource_size_t bar_start = pci_resource_start(dev, i);
		resource_size_t bar_end = pci_resource_end(dev, i);
		resource_size_t bar_length = pci_resource_len(dev, i);
		lro->bar[i] = NULL;
		printk(KERN_DEBUG "BAR%d: %llu bytes to be mapped.\n", i, (unsigned long long)bar_map_sizes[USER_BAR]); 
		/* do not map, and skip, specified BAR mapping with length 0 */
		if (!bar_map_sizes[i])
			continue;
		/* do not map BARs with length 0. Note that start MAY be 0! */
		if (!bar_length) {
			printk(KERN_DEBUG "BAR #%d is not present???\n", i);
			printk(KERN_DEBUG "pci_resource_start(... , %d) = 0x%llx.\n", i, (unsigned long long)bar_start);
			printk(KERN_DEBUG "pci_resource_end(... , %d) = 0x%llx.\n", i, (unsigned long long)bar_end);
			rc = -1;
			goto fail;
		}
		/* BAR length is less than driver requires? */
		if (bar_length < bar_map_sizes[i]) {
			printk(KERN_DEBUG "BAR #%d length = %llu bytes but "
				" driver requires at least %llu bytes\n", i,
				(unsigned long long)bar_length,
				(unsigned long long)bar_map_sizes[i]);
			rc = -1;
			goto fail;
		}
		/* map the full device memory or IO region into kernel virtual
		 * address space */
		lro->bar[i] = pci_iomap(dev, i, bar_map_sizes[i]);
		if (!lro->bar[i]) {
			printk(KERN_DEBUG "Could not map BAR #%d. See bar_map_size option to reduce the map size.\n", i);
			rc = -1;
			goto fail;
		}
		printk(KERN_DEBUG "BAR[%d] at 0x%llx mapped at 0x%p with length "
			"%llu(/%llu).\n", i, (unsigned long long)bar_start,
			lro->bar[i],
			(unsigned long long)bar_map_sizes[i],
			(unsigned long long)bar_length);
	}
	/* succesfully mapped all required BAR regions */
	rc = 0;
	goto success;
fail:
	/* unwind; unmap any BARs that we did map */
	unmap_bars(lro, dev);
success:
	return rc;
}

#if defined(CONFIG_MTD) && 0
#  ifdef CONFIG_MTD_COMPLEX_MAPPINGS) && de
static map_word pci_read16(struct map_info *map, unsigned long ofs)
{
	map_word val;
	val.x[0]= ioread16(map->virt + ofs);
	printk(KERN_DEBUG "pci_read16 : %08lx => %02x\n", ofs, val.x[0]);
	return val;
}

static void pci_write16(struct map_info *map, map_word val, unsigned long ofs)
{
	printk(KERN_DEBUG "pci_write16 : %08lx <= %02x\n", ofs, val.x[0]);
	iowrite16(val.x[0], map->virt + ofs);
}

static void pci_copyfrom(struct map_info *map, void *to, unsigned long from, ssize_t len)
{
	printk(KERN_DEBUG "pci_copyfrom : virt= 0x%p, %08p <= %08x, %d\n",
		map->virt, to, from, len);
	memcpy_fromio(to, map->virt + from, len);
}

static void pci_copyto(struct map_info *map, unsigned long to, const void *from, ssize_t len)
{
	printk(KERN_DEBUG "pci_copyfrom : virt=0x%p, %08p => %08x, %d\n",
		map->virt, from, to, len);
	memcpy_toio(map->virt + to, from, len);
}

static int probe_flash(struct lancero_dev *lro, int offset)
{
	/* MTD info after probing */
	struct mtd_info *mtd;
	struct map_info *map;
	int rc;
	map = kzalloc(sizeof(struct map_info), GFP_KERNEL);
	if (!map) {
		printk(KERN_DEBUG "Failed to allocate map_info.\n");
		goto fail_map;
	}

	map->name = dev_name(&lro->pci_dev->dev);
	map->phys = pci_resource_start(lro->pci_dev, 0) + 0x800000;
	map->size = 0x800000;
	map->bankwidth = 2;
	map->virt = lro->bar[0] + 0x800000;

#if 1
	map->read = pci_read16;
	map->write = pci_write16;
	map->copy_from = pci_copyfrom;
	map->copy_to = pci_copyto,
#else
	simple_map_init(map);
#endif

#if 0
	probe_type = rom_probe_types;
	for (; mtd == NULL && *probe_type != NULL; probe_type++)
		mtd = do_map_probe(*probe_type, &map[i]);
#endif
	mtd = do_map_probe("cfi_probe", map);
	printk(KERN_DEBUG "mtd = 0x%p\n", mtd);
	if (!mtd) {
		goto fail_mtd;
	}
	rc = add_mtd_device(mtd);
	printk(KERN_DEBUG "add_mtd_device() = %d\n", rc);
	return 0;
fail_mtd:
	kfree(map);
fail_map:
	return -1;
}
#endif /* ifdef CONFIG_MTD_COMPLEX_MAPPINGS */
#endif /* ifdef CONFIG_MTD */

/* obtain the 32 most significant (high) bits of a 32-bit or 64-bit address */
#define pci_dma_h(addr) ((addr >> 16) >> 16)
/* obtain the 32 least significant (low) bits of a 32-bit or 64-bit address */
#define pci_dma_l(addr) (addr & 0xffffffffUL)

static void dump_desc(struct lancero_desc *desc_virt)
{
	int j;
	u32 *p = (u32 *)desc_virt;
	const char *field_name[] = { "magic|extra_adjacent|control", "bytes",
		"fpga_addr", "fpga_addr_pad", "pcie_addr_lo", "pcie_addr_hi",
		"next_addr", "next_addr_pad" };
	for (j = 0; j < 8; j += 1) {
		printk(KERN_DEBUG "0x%08x/0x%02x: 0x%08x 0x%08x %s\n",
			(u32)p, (u32)p & 15, *p, le32_to_cpu(*p), field_name[j]);
		p++;
	}
	printk(KERN_DEBUG "\n");
}

static void dump_transfer(struct lancero_transfer *transfer)
{
	int i;
	struct lancero_desc *desc_virt = transfer->desc_virt;
	printk(KERN_DEBUG "Descriptor Entry (Pre-Transfer)\n");
	for (i = 0; i < transfer->desc_num; i += 1) {
		dump_desc(desc_virt + i);
	}
}

int adjacent_bound(u32 address)
{
  return 0x1000 - (address & 0x0fffUL) >> 5;
}

/* lancero_desc_alloc() - Allocate cache-coherent array of N descriptors.
 *
 * Allocates an array of 'number' descriptors in contiguous PCI bus addressable
 * memory. Chains the descriptors as a singly-linked list; the descriptor's next
 * pointer specifies the bus address of the next descriptor.
 *
 *
 * @dev Pointer to pci_dev
 * @number Number of descriptors to be allocated
 * @desc_bus_p Pointer where to store the first descriptor bus address
 * @desc_last_p Pointer where to store the last descriptor virtual address,
 * or NULL.
 *
 * @return Virtual address of the first descriptor
 *
 */
static struct lancero_desc *lancero_desc_alloc(struct pci_dev *dev,
int number, dma_addr_t *desc_bus_p, struct lancero_desc **desc_last_p) {
	/* virtual address */
	struct lancero_desc *desc_virt;
	/* bus address */
	dma_addr_t desc_bus;
	int i, adj = number - 1, extra_adj;

	BUG_ON(number < 1);

	/* allocate a set of cache-coherent contiguous pages */
	desc_virt = (struct lancero_desc *)pci_alloc_consistent(dev,
		number * sizeof(struct lancero_desc), desc_bus_p);
	if (!desc_virt) return NULL;
	/* get bus address of the first descriptor */
	desc_bus = *desc_bus_p;
	WARN_ON((desc_bus >> 16) >> 16);

	/* create singly-linked list for SG DMA controller */
	for (i = 0; i < number - 1; i++) {
		/* increment bus address to next in array */
		desc_bus += sizeof(struct lancero_desc);
		/* XXX assert not using >4GB addresses for descriptors XXX */
		WARN_ON((desc_bus >> 16) >> 16);

		/* singly-linked list uses bus addresses */
		desc_virt[i].next_lo = cpu_to_le32(pci_dma_l(desc_bus));
		desc_virt[i].next_hi = cpu_to_le32(pci_dma_h(desc_bus));
		desc_virt[i].bytes = cpu_to_le32(0);

		/* any adjacent descriptors? */
		if (adj > 0) {
#if 0 /* done in IP core */
			/* see max adjacent due to 4K boundary */
			int boundary_adj = adjacent_bound(pci_dma_l(desc_bus));
			/* limited by 4K boundary? */
			if (adj > boundary_adj) {
				extra_adj = boundary_adj - 1;
				printk("boundary_adj = %d\n", boundary_adj);
			/* not limited by 4K boundary */
			} else
#endif
			{
				extra_adj = adj - 1;
			}
			if (extra_adj > MAX_EXTRA_ADJ) 
				extra_adj = MAX_EXTRA_ADJ;
			adj--;
		} else
			extra_adj = 0;
		//printk(KERN_DEBUG "extra_adj = %d\n", extra_adj);
#if DESC_COUNTER
		desc_virt[i].control = cpu_to_le32(0xAD4B0000UL |
			((i & 0xf) << 12) | (extra_adj << 8));
#else
		desc_virt[i].control = cpu_to_le32(0xAD4B0000UL | (extra_adj << 8));
#endif
	}
	/* { i = number - 1 } */
	/* zero the last descriptor next pointer */
	desc_virt[i].next_lo = cpu_to_le32(0);
	desc_virt[i].next_hi = cpu_to_le32(0);
	desc_virt[i].bytes = cpu_to_le32(0);
#if DESC_COUNTER
	desc_virt[i].control = cpu_to_le32(0xAD4B0000UL | ((i & 0xf) << 12));
#else
	desc_virt[i].control = cpu_to_le32(0xAD4B0000UL);
#endif
	/* caller wants a pointer to last descriptor? */
	if (desc_last_p) {
		*desc_last_p = desc_virt + i;
	}

	/* return the virtual address of the first descriptor */
	return desc_virt;
}

/* lancero_desc_link() - Link two descriptors
 *
 * Link the first descriptor to a second descriptor, or terminate the first.
 *
 * @first first descriptor
 * @second second descriptor, or NULL if first descriptor must be set as last.
 * @second_bus bus address of second descriptor
 */
static void lancero_desc_link(struct lancero_desc *first, struct lancero_desc *second,
dma_addr_t second_bus) {
	/* remember reserved control in first descriptor, but zero extra_adjacent! */
	u32 control = le32_to_cpu(first->control) & 0x0000f0ffUL;
	/* second descriptor given? */
	if (second) {
		/* link last descriptor of 1st array to first descriptor of 2nd array */
		first->next_lo = cpu_to_le32(pci_dma_l(second_bus));
		first->next_hi = cpu_to_le32(pci_dma_h(second_bus));
		WARN_ON(first->next_hi);
	/* no second descriptor given */
	} else {
		/* first descriptor is the last */
		first->next_lo = 0;
		first->next_hi = 0;
	}
	/* merge magic, extra_adjacent and control field */
	control |= 0xAD4B0000UL;

	/* write bytes and next_num */
	first->control = cpu_to_le32(control);
}

static void lancero_transfer_cyclic(struct lancero_transfer *transfer)
{
	/* make transfer cyclic */
	lancero_desc_link(transfer->desc_virt + transfer->desc_num - 1,
                transfer->desc_virt, transfer->desc_bus);
	/* remember transfer is cyclic */
	transfer->cyclic = 1;
}

/* lancero_desc_adjacent -- Set how many descriptors are adjacent to this one */
static void lancero_desc_adjacent(struct lancero_desc *desc, int next_adjacent)
{
	int extra_adj = 0;
	/* remember reserved and control bits */
	u32 control = le32_to_cpu(desc->control) & 0x0000f0ffUL;
	if (next_adjacent > 0) {
		extra_adj =  next_adjacent - 1;
		if (extra_adj > MAX_EXTRA_ADJ)
			extra_adj = MAX_EXTRA_ADJ;
	}
	/* merge adjacent and control field */
	control |= 0xAD4B0000UL | (extra_adj << 8);
	/* write control and next_adjacent */
	desc->control = cpu_to_le32(control);
}

/* lancero_desc_control -- Set complete control field of a descriptor. */
static void lancero_desc_control(struct lancero_desc *first, u32 control_field)
{
	/* remember magic and adjacent number */
	u32 control = le32_to_cpu(first->control) & 0xffffff00UL;
	BUG_ON(control_field & 0xffffff00UL);
	/* merge adjacent and control field */
	control |= control_field;
	/* write control and next_adjacent */
	first->control = cpu_to_le32(control);
}

/* lancero_desc_clear -- Clear bits in control field of a descriptor. */
static void lancero_desc_control_clear(struct lancero_desc *first, u32 clear_mask)
{
	/* remember magic and adjacent number */
	u32 control = le32_to_cpu(first->control);
	BUG_ON(clear_mask & 0xffffff00UL);
	/* merge adjacent and control field */
	control &= (~clear_mask);
	/* write control and next_adjacent */
	first->control = cpu_to_le32(control);
}

/* lancero_desc_clear -- Set bits in control field of a descriptor. */
static void lancero_desc_control_set(struct lancero_desc *first, u32 set_mask)
{
	/* remember magic and adjacent number */
	u32 control = le32_to_cpu(first->control);
	BUG_ON(set_mask & 0xffffff00UL);
	/* merge adjacent and control field */
	control |= set_mask;
	/* write control and next_adjacent */
	first->control = cpu_to_le32(control);
}


/* lancero_desc_free - Free cache-coherent linked list of N descriptors.
 *
 * @dev Pointer to pci_dev
 * @number Number of descriptors to be allocated
 * @desc_virt Pointer to (i.e. virtual address of) first descriptor in list
 * @desc_bus Bus address of first descriptor in list
 */
static void lancero_desc_free(struct pci_dev *dev,
int number, struct lancero_desc *desc_virt, dma_addr_t desc_bus) {
	BUG_ON(!desc_virt);
	BUG_ON(number < 0);
	/* free contiguous list */
	pci_free_consistent(dev, number * sizeof(struct lancero_desc),
		desc_virt, desc_bus);
}

/* lancero_desc() - Fill a descriptor with the transfer details
 *
 * @desc pointer to descriptor to be filled
 * @addr root complex address
 * @ep_addr end point address
 * @len number of bytes, must be a (non-negative) multiple of 4.
 * @dir_to_dev If non-zero, source is root complex address and destination
 * is the end point address. If zero, vice versa.
 *
 * Does not modify the next pointer
 */
static void lancero_desc_set(struct lancero_desc *desc,
dma_addr_t rc_bus_addr, u32 ep_addr, int len)
{
	/* length (in bytes) must be a non-negative multiple of four */
	BUG_ON(len & 3);
	//BUG_ON(len <= 0);
	/* transfer length */
	desc->bytes = cpu_to_le32(len);
	/* address on the PCIe bus */
	desc->pcie_addr_lo = cpu_to_le32(pci_dma_l(rc_bus_addr));
	desc->pcie_addr_hi = cpu_to_le32(pci_dma_h(rc_bus_addr));
#if 0
	if (pci_dma_h(rc_bus_addr))
  	printk(KERN_DEBUG "DMA requires 64-bit support.\n");
#endif
	/* address on the fpga bus  */
	desc->fpga_addr = cpu_to_le32(pci_dma_l(ep_addr));

	/* zero all padding fields */
	desc->fpga_addr_pad = cpu_to_le32(0);
}

#if FPGA_DESCRIPTORS
static int lancero_desc_copylocal(struct lancero_dev *lro, int offset, struct lancero_desc *src)
{
	int addr = 0;
	struct lancero_desc *dst = (struct lancero_desc *)(lro->bar[0] + offset);
	while (src) {
		/* copy descriptor */
		*dst = *src;
		if (src->next_lo || src-> next_hi) {
			dst->next_lo = (addr + 32);
			dst->next_hi = 0;

			src++;
			dst++;
		} else {
			dst->next_lo = 0;
			dst->next_hi = 0;
			/* stop */
			src = 0;
		}
		/* dump this local descriptor */
		dump_desc(lro->bar[0] + addr);
		addr += 32;
	}
}
#endif /* FPGA_DESCRIPTORS */

/* queue_transfer() - Queue a DMA transfer on the engine
 *
 * @engine DMA engine doing the transfer
 * @transfer DMA transfer submitted to the engine
 *
 * Takes and releases the engine spinlock
 */
static int queue_transfer(struct lancero_engine *engine,
struct lancero_transfer *transfer)
{
	int rc = 0;
	struct lancero_transfer *transfer_started;
	BUG_ON(!engine);
	BUG_ON(!transfer);
	BUG_ON(transfer->desc_num == 0);
	dbg_tfr("queue_transfer(transfer=0x%p).\n", transfer);

	/* lock the engine state */
	spin_lock(&engine->lock);
	engine->prev_cpu = get_cpu();
	put_cpu();

	/* engine is being shutdown; do not accept new transfers */
	if (engine->shutdown & ENGINE_SHUTDOWN_REQUEST) {
		rc = -1;
		goto shutdown;
	}

	/* either the engine is still busy and we will end up in the
	 * service handler later, or the engine is idle and we have to
	 * start it with this transfer here */

#if CHAIN_MULTIPLE_TRANSFERS
	/* queue is not empty? try to chain the descriptor lists */
	if (!list_empty(&engine->transfer_list)) {
		struct lancero_transfer *last;
		dbg_tfr("queue_transfer(): list not empty\n");
		/* get last transfer queued on the engine */
		last = list_entry(engine->transfer_list.prev,
			struct lancero_transfer, entry);
		/* @only when non-cyclic transfer */
		/* link the last transfer's last descriptor to this transfer */
		lancero_desc_link(last->desc_virt + last->desc_num - 1,
			transfer->desc_virt, transfer->desc_bus);
		/* do not stop now that there is a linked transfers */
		lancero_desc_control_clear(last->desc_virt + last->desc_num - 1, LANCERO_DESC_STOP);

		dbg_tfr("queue_transfer(transfer=0x%p, desc=%d) chained after 0x%p with engine %s.\n",
			transfer, transfer->desc_num, last, engine->running? "running" : "idle");
#if 0 /* debug proper linking between transfer descriptor lists? */
		dbg_tfr("last->desc_bus = 0x%p, transfer->desc_bus = 0x%p\n", last->desc_bus, transfer->desc_bus);
		if (last->desc_num > 1)
			dump_desc(last->desc_virt + last->desc_num - 2);
		dump_desc(last->desc_virt + last->desc_num - 1);
		dump_desc(transfer->desc_virt);
		dump_desc(transfer->desc_virt + 1);
		dump_desc(transfer->desc_virt + transfer->desc_num - 1);
#endif
	/* queue is empty */
	} else {
		if (engine->running)
			dbg_tfr("queue_transfer(): queue empty, but engine seems to be running?!!\n");
		else
			dbg_tfr("queue_transfer(): queue empty and engine idle.\n");
		/* engine should not be running */
		WARN_ON(engine->running);
	}
#endif

	/* mark the transfer as submitted */
	transfer->state = TRANSFER_STATE_SUBMITTED;
	/* add transfer to the tail of the engine transfer queue */
	list_add_tail(&transfer->entry, &engine->transfer_list);

	/* engine is idle? */
	if (!engine->running) {
		/* start engine */
		dbg_tfr("queue_transfer(): starting %s engine.\n", engine->name);
		transfer_started = engine_start(engine);
		dbg_tfr("queue_transfer(transfer=0x%p) started %s engine with transfer 0x%p.\n", transfer, engine->name, transfer_started);
	} else {
		dbg_tfr("queue_transfer(transfer=0x%p) queued, with %s engine running.\n", transfer, engine->name);
	}
shutdown:
	/* unlock the engine state */
	printk(KERN_DEBUG "engine->running = %d\n", engine->running);
	spin_unlock(&engine->lock);
	return rc;
};

/* create_engine() - Create an SG DMA engine bookkeeping data structure
 *
 * An SG DMA engine consists of the resources for a single-direction transfer
 * queue; the SG DMA hardware, the software queue and interrupt handling.
 *
 * @dev Pointer to pci_dev
 * @offset byte address offset in BAR[bridge_bar] resource for the SG DMA
 * controller registers.
 * @dir_to_dev Whether the engine transfers to the device (PCIe Rd).
 */
static struct lancero_engine *create_engine(struct lancero_dev *lro, int offset,
int dir_to_dev)
{
	/* allocate data structure for engine book keeping */
	struct lancero_engine *engine =
		kzalloc(sizeof(struct lancero_engine), GFP_KERNEL);
	/* memory allocation failure? */
	if (!engine) return NULL;
	/* set magic */
	engine->magic = MAGIC_ENGINE;
	/* create virtual memory mapper */
	engine->sgm = sg_create_mapper(LANCERO_TRANSFER_MAX_BYTES);
	if (!engine->sgm) {
		kfree(engine);
		return NULL;
	}
	/* initialize spinlock */
	spin_lock_init(&engine->lock);
	/* initialize transfer_list */
	INIT_LIST_HEAD(&engine->transfer_list);
	/* parent */
	engine->lro = lro;
	/* register address */
	engine->regs = (lro->bar[bridge_bar] + offset);
	/* remember SG DMA direction */
	engine->dir_to_dev = dir_to_dev;
	engine->interrupt = dir_to_dev? LANCERO_INT_DMA_READ: LANCERO_INT_DMA_WRITE;
	/* initialize the deferred work for transfer completion */
	INIT_WORK(&engine->work, engine_service_work);
	/* initialize wait queue */
	init_waitqueue_head(&engine->shutdown_wq);
	return engine;
}

/* free_transfer() - free transfer */
static void free_transfer(struct lancero_dev *lro, struct lancero_transfer *transfer)
{
	/* user space buffer was locked in on account of transfer? */
	if (transfer->sgm) {
		/* unmap scatterlist */
		dbg_tfr("free_transfer(): pci_unmap_sg()\n");
		/* the direction is needed to synchronize caches */
		pci_unmap_sg(lro->pci_dev, transfer->sgm->sgl, transfer->sgm->mapped_pages,
			transfer->dir_to_dev? DMA_TO_DEVICE: DMA_FROM_DEVICE);
		if (transfer->userspace) {
			/* dirty and unlock the pages */
			sgm_put_user_pages(transfer->sgm, transfer->dir_to_dev? 0 : 1);
			dbg_tfr("free_transfer(): sgm_put_user_pages()\n");
		}
		transfer->sgm->mapped_pages = 0;
		sg_destroy_mapper(transfer->sgm);
	}
	/* free descriptors */
	lancero_desc_free(lro->pci_dev,
		transfer->sgl_nents, transfer->desc_virt, transfer->desc_bus);
	/* free transfer */
	kfree(transfer);
}

#define CONFIG_WDMA_256 (1 << 4)			
#define CONFIG_WDMA_128 (1 << 3)			
#define CONFIG_WDMA_64 (1 << 2)			
#define CONFIG_WDMA_32 (1 << 1)			
#define CONFIG_WDMA_EN (1 << 0)	

#define CONFIG_RDMA_256 (1 << 4)			
#define CONFIG_RDMA_128 (1 << 3)			
#define CONFIG_RDMA_64 (1 << 2)			
#define CONFIG_RDMA_32 (1 << 1)			
#define CONFIG_RDMA_EN (1 << 0)	

/* lancero_config -- Inspect the Lancero IP Core configuration */
static int lancero_config(struct lancero_dev *lro, unsigned int config_offset)
{
	void *reg = lro->bar[bridge_bar] + config_offset;
	u32 w, payload, maxread;
	int rc = 0;
	int version = 0;

	/* read identifier of the configuration inspector */
	w = read_register(reg + 0x00);
	/* read version of the configuration inspector */
	version = w & 0x000000ffUL;
	printk(KERN_ERR "Configuration Inspector identifier is 0x%08x.\n", w);
	/* configuration inspector not found? */
	if ((w & 0xffffff00UL) != 0x00b20000UL) {
		printk(KERN_ERR "Configuration Inspector identifier not found (found 0x%08x expected 0x00b20001).\n", w);
		rc = -1;
		goto fail_identifier;
	}

	payload = read_register(reg + 0x08);
	printk(KERN_ERR "Payload = 0x%08x.\n", payload);
	maxread = read_register(reg + 0x0c);
	printk(KERN_ERR "MaxRead =  0x%08x.\n", maxread);

	/* read Lancero System identifier */
	w = read_register(reg + 0x10) & 0x0000ffffUL;
	/* Lancero Target Bridge */
	if (w == 0xFF01UL)
		printk(KERN_DEBUG "Lancero Target Bridge\n");
	/* Lancero SGDMA */
	else if (w == 0xFF02UL) {
		/* versions lower than 2 had a fixed configuration */
		if (version < 2) {
			/* read and write engines always present */
			lro->capabilities |= CAP_ENGINE_WRITE | CAP_ENGINE_READ;
			lro->align[/*dir_to_dev=*/0] = lro->align[1] = 8;
		/* engine capabilities in the inspector since version 2 */
		} else {
			/* inspect write engine capabilities */
			w = read_register(reg + 0x1c);
			printk(KERN_DEBUG "WDMA = 0x%08lx\n", w);
			if (w & CONFIG_WDMA_EN) {
				lro->capabilities |= CAP_ENGINE_WRITE;
				/* determine address and size alignment for write DMA */
				if (w & CONFIG_WDMA_32) lro->align[/*dir_to_dev=*/0] = 4;
				else if (w & CONFIG_WDMA_128) lro->align[/*dir_to_dev=*/0] = 16;
				else if (w & CONFIG_WDMA_256) lro->align[/*dir_to_dev=*/0] = 32;
				else lro->align[/*dir_to_dev=*/0] = 8;
				printk(KERN_DEBUG "Write Engine requires %d byte alignment.\n", lro->align[/*dir_to_dev=*/0]);
			} 
			/* inspect read engine capabilities */
			w = read_register(reg + 0x20);
			printk(KERN_DEBUG "RDMA = 0x%08lx\n", w);
			if (w & CONFIG_RDMA_EN) {
				lro->capabilities |= CAP_ENGINE_READ;
				/* determine address and size alignment for read DMA */
				if (w & CONFIG_RDMA_32) lro->align[/*dir_to_dev=*/1] = 4;
				else if (w & CONFIG_RDMA_128) lro->align[/*dir_to_dev=*/1] = 16;
				else if (w & CONFIG_RDMA_256) lro->align[/*dir_to_dev=*/1] = 32;
				else lro->align[/*dir_to_dev=*/1] = 8;
				printk(KERN_DEBUG "Read Engine requires %d byte alignment.\n", lro->align[/*dir_to_dev=*/0]);
			} 
		}
		/* any engine present? */
		if (lro->capabilities & (CAP_ENGINE_READ | CAP_ENGINE_WRITE)) {
			printk(KERN_DEBUG "Lancero Scatter-Gather%s%s\n",
				(lro->capabilities & CAP_ENGINE_READ)?" Read":"",
				(lro->capabilities & CAP_ENGINE_WRITE)?" Write":"");
		} else {
			printk(KERN_DEBUG "Lancero Target Bridge (Scatter-Gather Engines disabled in SOPC).\n");
		}
	}
	else
		printk(KERN_DEBUG "Lancero System ID = 0x%04x.\n", w);
	w = read_register(reg + 0x04);
	/* bus, device and function */
	printk(KERN_DEBUG "bus:dev.fn = %02x:%02x.%1x, payload = %d bytes, maxread = %d bytes\n",
		(w >> 8) & 0x0f/*bus*/, (w >> 3) & 0x1f/* device*/, w & 0x07/*function*/,
		(unsigned int)payload, (unsigned int)maxread);
fail_identifier:
	return rc;
}


/* test_desc_bridge --
 *
 * A transfer is created which uses a list of descriptors laid in host memory
 * at a given address offset and with a given number of descriptors.
 *
 */
static int test_desc_bridge(struct lancero_dev *lro, int tester_offset, int desc_num, int desc_offset, int number)
{
	void *reg = lro->bar[bridge_bar] + tester_offset;
	u8 *buffer_virt = 0;
	/* virtual address of the descriptor */
	struct lancero_desc *desc_virt = 0;
	/* bus address of the allocated buffer and descriptor */
	dma_addr_t buffer_bus = 0;
	struct lancero_transfer *transfer;
	int i, res = 0, j, rc = 0;
	u32 w;

	printk(KERN_DEBUG "test_desc_bridge()\n");
	printk(KERN_DEBUG "lro->bar[bridge_bar] = 0x%p\n", lro->bar[bridge_bar]);
	printk(KERN_DEBUG "reg = 0x%p, offset = %d\n", reg, tester_offset);

	while (desc_offset < 0) desc_offset += 4096;
	BUG_ON(desc_offset & 3);

	/* allocate transfer data structure */
	transfer = kzalloc(sizeof(struct lancero_transfer), GFP_KERNEL);
	if (!transfer) return -1;

	/* allocate and map coherently-cached memory for a DMA-able buffer */
	/* @see Documentation/DMA-mapping.txt */
	buffer_virt = (u8 *)pci_alloc_consistent(lro->pci_dev, desc_num * PAGE_SIZE, &buffer_bus);
	if (!buffer_virt) {
		printk(KERN_DEBUG "Could not allocate coherent DMA buffer.\n");
		res = -ENOMEM;
		goto fail_pci_alloc;
	}
#if 0
	printk(KERN_DEBUG "Allocated cache-coherent DMA buffer of %lu bytes;\n"
		"virtual address = 0x%016Lx;\n    bus address = 0x%016Lx).\n",
		desc_num * PAGE_SIZE, (unsigned long long)(int)buffer_virt, (unsigned long long)(int)buffer_bus);
#endif
	/* allocate descriptor list, plus additional PAGE_SIZE */
	desc_virt = transfer->desc_virt = lancero_desc_alloc(lro->pci_dev,
		desc_num + PAGE_SIZE / 32, &transfer->desc_bus, NULL);
	if (!desc_virt) {
		printk(KERN_DEBUG "Could not allocate descriptors.\n");
		res = -ENOMEM;
		goto fail_desc;
	}
#if 0
	printk(KERN_DEBUG "Allocated cache-coherent descriptor list of %lu bytes;\n"
		"virtual address = 0x%016Lx;\n    bus address = 0x%016Lx).\n",
		desc_num * 32 + PAGE_SIZE, (unsigned long long)(int)desc_virt, (unsigned long long)(int)transfer->desc_bus);
#endif

	if (desc_offset > 0) {
		/* apply the offset for the test */
		transfer->desc_virt = (struct lancero_desc *)((char *)transfer->desc_virt + desc_offset);
		transfer->desc_bus = (char *)transfer->desc_bus + desc_offset;

#if 0
		printk(KERN_DEBUG "   Nudged cache-coherent descriptor list of %lu bytes;\n"
			"virtual address = 0x%016Lx;\n    bus address = 0x%016Lx).\n",
			desc_num * 32 + PAGE_SIZE, (unsigned long long)(int)transfer->desc_virt, (unsigned long long)(int)transfer->desc_bus);
#endif
	}
	/* iterate over all descriptors */
	for (i = 0; i < desc_num - 1; i++) {
		/* fill in descriptor entry i with transfer details */
		lancero_desc_set(transfer->desc_virt + i, buffer_bus + i * PAGE_SIZE,
			0 + i * PAGE_SIZE, PAGE_SIZE);
		/* link descriptor i to descriptor i + 1 */
		lancero_desc_link(transfer->desc_virt + i, transfer->desc_virt + i + 1,
			transfer->desc_bus + (i + 1) * 32);
	}
	/* fill in descriptor entry i with transfer details */
	lancero_desc_set(transfer->desc_virt + i, buffer_bus + i * PAGE_SIZE,
		0 + i * PAGE_SIZE, PAGE_SIZE);
	/* terminate the last descriptor i */
	lancero_desc_link(transfer->desc_virt + i, 0, 0);

	transfer->desc_num = i + 1;
//	dbg_sg("transfer 0x%p has %d descriptors\n", transfer, transfer->desc_num);

	for (j = 0; j < 8 * desc_num; j++) {
		*((u32 *)transfer->desc_virt + j) = ((number << 8) | j);
	}

	//dump_transfer(transfer);
	/* initialize wait queue */
	init_waitqueue_head(&transfer->wq);

	w = read_register(reg + 0x00);
	if ((w & 0xffffffffUL) != 0xae230001UL) {
		printk(KERN_DEBUG "Descriptor Tester identifier not found (found 0x%08x expected 0xae230001).", w);
		goto fail_tester;
	}

	w = transfer->desc_bus;
//	printk(KERN_DEBUG "About to perform iowrite32(0x%x @ 0x%p).\n", w, reg + 0x0c);
	write_register(w, reg + 0x0c);
	w = desc_num * 8;
//	printk(KERN_DEBUG "About to perform iowrite32(0x%x @ 0x%p).\n", w, reg + 0x10);
	write_register(w, reg + 0x10);
#if 1
	/* flush writes */
	w = read_register(reg + 0x10);
#endif
	w = 0x00000003;
	printk(KERN_DEBUG "About to perform iowrite32(0x%x @ 0x%p).\n", w, reg + 0x04);
	write_register(w, reg + 0x04);
#if 1
	/* flush writes */
	w = read_register(reg + 0x04);
	if (w & 0xfffffffc) {
		printk(KERN_DEBUG "Flush read status is 0x%08x.\n", w);
		rc = -5;
		goto fail_tester;
	}
		/* no longer busy? */
#endif

#if 0
	/* re-write length in an attempt to produce errors */
	for (j = 0; j < 100; j++) {
		w = desc_num * 8;
		write_register(w, reg + 0x10);
		udelay(10);
	}
#endif

#if 1 /* poll status */
	for (j = 0; j < 100; j++) {
		/* read the config module */
		if (config_offset != -1U) {
			int bus = lancero_config(lro, config_offset);
			/* is the bus number still two? */
			if (bus != 2) {
				printk(KERN_DEBUG "BUS IS NO LONGER 2.\n");
				rc = -5;
				goto fail_tester;
			}
		}
		w = read_register(reg + 0x08);
		/* still busy after one loop iteration? */
//		if ((j > 0) && (w & 0x1))
		/* not zero, BUSY, OR IRQ|BUSY? */
		if ((w != 0x0) && (w != 0x1) && (w != 0x3)) {
			printk(KERN_DEBUG "Status 0x%08x after %d microseconds.\n", w, j * 10);
			rc = -4;
			goto fail_tester;
		}
		/* no longer busy? */
		if ((w & 0x1) == 0x0) break;
		udelay(10);
	}
	/* still busy? */
	if (w & 0x1) {
		rc = -1;
		printk(KERN_DEBUG "Timed out after %d microseconds.\n", j * 10);
		goto fail_tester;
	} else {
		printk(KERN_DEBUG "Completed after %d microseconds.\n", j * 10);
	}
#endif /* poll status */

#if 1 /* verify tester buffer against expected data */
//	printk(KERN_DEBUG "Reading buffer.\n");
	for (j = 0; j < 8 * desc_num; j++) {
		w = read_register(reg + 0x200 + j * 4);
		if (w != ((number << 8) | j)) {
			printk(KERN_DEBUG "buffer offset 0x%04x: Expected 0x%08x but read 0x%08x.\n", j * 4, (0x11223300 | j), w);
			rc = -2;
			goto fail_tester;
		}
	}
#endif
#if 1 /* read remainder of buffer */
//	printk(KERN_DEBUG "Reading remainder of buffer.\n");
	for (; j < 128; j++) {
		w = read_register(reg + 0x200 + j * 4);
		if (w != (0xdeaddead)) {
			printk(KERN_DEBUG "Read 0x%08x at 0x%08x.\n", w, 0x200 + j * 4);
			rc = -3;
			goto fail_tester;
		}
	}
#endif
	read_interrupts(lro, LANCERO_OFS_INT_CTRL);

fail_tester:
	if (rc != 0)
		printk(KERN_DEBUG "test_desc_bridge(..., num = %d, offset = %d) = %d.\n",
			desc_num, desc_offset, rc);

#if 1 /* stay in memory for further tests using descriptor tester */
	/* free descriptor list, plus additional PAGE_SIZE */
	lancero_desc_free(lro->pci_dev, desc_num + PAGE_SIZE / 32,
		desc_virt, transfer->desc_bus);
#endif
/* failed allocating descriptors */
fail_desc:
	/* unmap and free coherently-cached memory for a DMA-able buffer */
	pci_free_consistent(lro->pci_dev, desc_num * PAGE_SIZE, buffer_virt, buffer_bus);
/* failed allocating pci consistent memory */
fail_pci_alloc:
	/* free the transfer data structure */
	kfree(transfer);
	return rc;
}

static int test_dma_performance(struct lancero_dev *lro)
{
	/* range of engines to test, 0 = write, 1 = read.
         * 0,0 means only test write engine
         * 1,1 means only test read engine
         * 0,1 means test both engines concurrently
         */
#define PERF_TRANSFER_NUM (2)
	/* virtual addresses of the DMA buffers */
	u8 *buffer_virt[PERF_TRANSFER_NUM] = { 0, 0 };
	/* bus address of the allocated buffer and descriptor */
	dma_addr_t buffer_bus[PERF_TRANSFER_NUM] = { 0, 0 };
	struct lancero_transfer *transfer[PERF_TRANSFER_NUM] = { 0, 0 };
	int i, res = 0, rc = 0, size = 8 * PAGE_SIZE, bus;
	unsigned int fpga_addr = 0x00000000UL;

	printk(KERN_DEBUG "test_dma_performance(engines %d through %d)\n", engine_first, engine_last);

	/* iterate over all descriptors */
	for (i = engine_first; i <= engine_last; i++) {
		/* allocate transfer data structure */
		transfer[i] = kzalloc(sizeof(struct lancero_transfer), GFP_KERNEL);
		if (!transfer[i]) {
			rc = -1;
			goto transfer_fail;
		}
		/* 0 = write engine (to_dev=0) , 1 = read engine (to_dev=1) */
		transfer[i]->dir_to_dev = i;
		/* set number of descriptors */
		transfer[i]->desc_num = 1;
		/* allocate descriptor list */
		transfer[i]->desc_virt = lancero_desc_alloc(lro->pci_dev,
			transfer[i]->desc_num, &transfer[i]->desc_bus, NULL);
		/* could not allocate descriptor list? */
		if (!transfer[i]->desc_virt) {
			printk(KERN_DEBUG "Could not allocate descriptor.\n");
			res = -ENOMEM;
			goto fail_desc;
		}
#if 1
		printk(KERN_DEBUG "Allocated cache-coherent descriptor list of %d bytes;\n"
			"virtual address = 0x%016Lx;\n    bus address = 0x%016Lx).\n",
			transfer[i]->desc_num * 8, (unsigned long long)transfer[i]->desc_virt,
			(unsigned long long)transfer[i]->desc_bus);
#endif
		/* allocate and map coherently-cached memory for a DMA-able buffer */
		/* @see Documentation/DMA-mapping.txt */
		buffer_virt[i] = (u8 *)pci_alloc_consistent(lro->pci_dev, size, &buffer_bus[i]);
		/* could not allocate and map coherently-cached memory for a DMA-able buffer? */
		if (!buffer_virt[i]) {
			printk(KERN_DEBUG "Could not allocate coherent DMA buffer size %d.\n", size);
			res = -ENOMEM;
			goto fail_pci_alloc;
		}
		printk(KERN_DEBUG "Allocated cache-coherent buffer of %d bytes;\n"
			"virtual address = 0x%016Lx;\n    bus address = 0x%016Lx).\n",
			size, (unsigned long long)buffer_virt[i],
			(unsigned long long)buffer_bus[i]);
#if 0
		printk(KERN_DEBUG "%03d: virt=0x%08x, bus=0x%08x, fpga_addr=0x%08x, size=0x%08x.\n",
			i, buffer_virt[i], buffer_bus[i], fpga_addr, size);
#endif
		/* fill in descriptor entry with transfer details */
		lancero_desc_set(transfer[i]->desc_virt, buffer_bus[i],
			fpga_addr, size);
		fpga_addr += 0x80000000UL;
		/* stop engine and request interrupt on last descriptor */
		lancero_desc_control(transfer[i]->desc_virt, 0);
		/* create a linked loop */
		lancero_desc_link(transfer[i]->desc_virt, transfer[i]->desc_virt, transfer[i]->desc_bus);
		transfer[i]->cyclic = 1;
		/* dump transfer for debugging */
		//dump_transfer(transfer[i]);
		/* initialize wait queue */
		init_waitqueue_head(&transfer[i]->wq);
	}
	/* queue */
	for (i = engine_first; i <= engine_last; i++) {
		printk(KERN_DEBUG "queue on engine %d\n", i);
		/* queue on write (0) resp. read (1) engine */
		queue_transfer(lro->engine[i], transfer[i]);
	}
	/* start measurement */
	for (i = engine_first; i <= engine_last; i++) {
		performance_start(lro, i? LANCERO_OFS_DMA_READ_PERF: LANCERO_OFS_DMA_WRITE_PERF, performance_interval * 1000);
	}

	printk(KERN_DEBUG "test_dma(): Waiting for the SGDMA engines to complete.\n");
	/* iterate over all descriptors */
	for (i = engine_first; i <= engine_last; i++) {
		/* the function servicing the engine will wake us */
		rc = wait_event_interruptible(transfer[i]->wq, transfer[i]->state != TRANSFER_STATE_SUBMITTED);
		if (rc) {
			printk(KERN_ERR "wait_event_interruptible() = %d\n", rc);
			//dump_transfer(transfer);
			lancero_read_status(lro->engine[i], 0);
		}
	}
	for (i = engine_first; i <= engine_last; i++) {
#if 0
		printk(KERN_DEBUG "%03d: virt=0x%16llx, bus=0x%08x.%08x, size=0x%08x.\n",
			i, (unsigned long long)buffer_virt[i],
			transfer[i]->desc_virt->pcie_addr_hi,
			transfer[i]->desc_virt->pcie_addr_lo,
			transfer[i]->desc_virt->bytes);
#endif
		/* free buffer for DMA */
		pci_free_consistent(lro->pci_dev, transfer[i]->desc_virt->bytes,
			buffer_virt[i], buffer_bus[i]);
	}
fail_verify:
fail_bus:
fail_pci_alloc:
	for (i = engine_first; i <= engine_last; i++) {
		/* free descriptor list */
		lancero_desc_free(lro->pci_dev, transfer[i]->desc_num,
			transfer[i]->desc_virt, transfer[i]->desc_bus);
	}
/* failed allocating pci consistent memory */
fail_desc:
	/* free the transfer data structure */
	for (i = engine_first; i <= engine_last; i++) {
		kfree(transfer[i]);
	}
/* failed allocating transfer */
transfer_fail:
	return rc;
}


static int test_latency(struct lancero_dev *lro)
{
	/* virtual addresses of the DMA buffers */
#if 0
	u8 *length_buffer_virt = 0;
#endif
	volatile u8 *packet_buffer_virt = 0;
	/* bus address of the allocated buffer and descriptor */
#if 0
	dma_addr_t length_buffer_bus = 0;
#endif
	dma_addr_t packet_buffer_bus = 0;
	/* transfer */
	//struct lancero_transfer *length_transfer = 0;
	struct lancero_transfer *packet_transfer = 0;
	struct lancero_engine *engine = lro->sgdma_char_dev->write_engine;

	//struct lancero_packet_generator_regs *reg = (struct lancero_packet_generator_regs *)(lro->bar[USER_BAR] + 0/*offset*/);
	struct lancero_latency_tester_regs *reg = (struct lancero_latency_tester_regs *)(lro->bar[USER_BAR] + 0/*offset*/);

	int i, res = 0, rc = 0, size = PAGE_SIZE, bus, pollcount = 0;
	u32 w;
	unsigned int fpga_addr = 0x00000000UL;
	unsigned long irq_flags;
	void *gpio = 0;

	printk(KERN_DEBUG "test_latency()\n");

#if 0
	gpio = ioremap_nocache(0xffe0f000, 0x1000);
	if (!gpio) {
		printk(KERN_DEBUG "Could not remap GPIO memory.\n");
		goto ioremap_fail;
	}
	out_be32(gpio + 0xc00, 0x061f0000);
	out_be32(gpio + 0xc08, 0xeaef0000);
#endif
	/* allocate transfer data structure */
	packet_transfer = kzalloc(sizeof(struct lancero_transfer), GFP_KERNEL);
	if (!packet_transfer) {
		rc = -1;
		goto transfer_fail;
	}
	/* 0 = write engine (to_dev=0) , 1 = read engine (to_dev=1) */
	//length_transfer[i]->dir_to_dev = 0;
	packet_transfer->dir_to_dev = 0;
	/* set number of descriptors */
	//length_transfer[i]->desc_num = 1;
	packet_transfer->desc_num = 1;
	/* allocate descriptor list for the packet data */
	packet_transfer->desc_virt = lancero_desc_alloc(lro->pci_dev,
		packet_transfer->desc_num, &packet_transfer->desc_bus, NULL);

	printk(KERN_DEBUG "Allocated cache-coherent descriptor list of %d bytes;\n"
		"virtual address = 0x%016Lx;\n    bus address = 0x%016Lx).\n",
		packet_transfer->desc_num * 8, (unsigned long long)packet_transfer->desc_virt,
		(unsigned long long)packet_transfer->desc_bus);

	/* could not allocate descriptor list? */
	if (!packet_transfer->desc_virt) {
		printk(KERN_DEBUG "Could not allocate descriptor.\n");
		res = -ENOMEM;
		goto fail_desc;
	}
#if 0

	/* allocate and map coherently-cached memory for a DMA-able buffer */
	/* @see Documentation/DMA-mapping.txt */
	length_buffer_virt = (u8 *)pci_alloc_consistent(lro->pci_dev, size, &length_buffer_bus);
	/* could not allocate and map coherently-cached memory for a DMA-able buffer? */
	if (!length_buffer_virt) {
		printk(KERN_DEBUG "Could not allocate coherent DMA buffer size %d.\n", size);
		res = -ENOMEM;
		goto fail_length_pci_alloc;
	}
#endif
	
	/* allocate and map coherently-cached memory for a DMA-able buffer */
	/* @see Documentation/DMA-mapping.txt */
	packet_buffer_virt = (u8 *)pci_alloc_consistent(lro->pci_dev, size, &packet_buffer_bus);
	/* could not allocate and map coherently-cached memory for a DMA-able buffer? */
	if (!packet_buffer_virt) {
		printk(KERN_DEBUG "Could not allocate coherent DMA buffer size %d.\n", size);
		res = -ENOMEM;
		goto fail_packet_pci_alloc;
	}

	/* fill in packet descriptor entry with transfer details */
	lancero_desc_set(packet_transfer->desc_virt, packet_buffer_bus,
		fpga_addr, 128);
	fpga_addr += 0x80000000UL;
	lancero_desc_control(packet_transfer->desc_virt, /*LANCERO_DESC_STOP |*/ LANCERO_DESC_COMPLETED);
#if 0
	/* create a linked loop of this single descriptor */
	lancero_desc_link(packet_transfer->desc_virt, packet_transfer->desc_virt, packet_transfer->desc_bus);
#endif
	//packet_transfer->cyclic = 1;
	dump_transfer(packet_transfer);

#if 0
	/* invalidate the length table, reset */
	iowrite32(0, &engine->regs->length_table_len);
	iowrite32(pci_dma_l(length_buffer_bus), &engine->regs->length_table_lo);
	iowrite32(pci_dma_h(length_buffer_bus), &engine->regs->length_table_hi);
	iowrite32(size, &engine->regs->length_table_len);
#endif

	/* initialize wait queue */
	init_waitqueue_head(&packet_transfer->wq);

	w = read_register(&reg->id);
	printk(KERN_DEBUG "latency tester id = 0x%04x.\n", w);

#if 0
	*(u32 *)length_buffer_virt = 0;
#endif
	*(u32 *)packet_buffer_virt = 0;

#if 0
	printk(KERN_DEBUG "*(u32 *)length_buffer_virt = 0x%08x.\n",*(u32 *)length_buffer_virt);
#endif
	printk(KERN_DEBUG "*(u32 *)packet_buffer_virt = 0x%08x.\n",*(u32 *)packet_buffer_virt);

	/* clear latency tester */
	w = 1;
	write_register(w, &reg->control);
	while (w)
		w = read_register(&reg->control);

	/* write data value */
	w = 0xBABECAFE;
	write_register(w, &reg->data);

	/* 1 ms delay */
	w = 125000;
	write_register(w, &reg->delay);
	read_register(&reg->delay);

#if 1
	/* disable interrupts */
	local_irq_save(irq_flags);
#endif

	/* queue on write (0) resp. read (1) engine */
	queue_transfer(lro->engine[0], packet_transfer);

	while (pollcount < 0x7fffffff) {
		if (*(volatile u32 *)packet_buffer_virt) break;
//		printk(KERN_DEBUG "pollcount = %d, *(u32 *)packet_buffer_virt = 0x%08x.\n",
//			pollcount, *(u32 *)packet_buffer_virt);
		pollcount++;
	}
#if 0
	out_be32(gpio + 0xc08, 0xeaff0000);
#endif
	//in_be32(gpio + 0xc08);
	// stop latency counter */
	w = 2;
	write_register(w, &reg->control);

	if ((pollcount > 0) && (pollcount < 0x7fffffff))
		printk(KERN_DEBUG "POLL SUCCESFULL\n");
	else
		printk(KERN_DEBUG "POLL FAILED (UNDER/OVERRUN)\n");

	w = read_register(&reg->counter);
	printk(KERN_DEBUG "COUNTER = %u.\n", w);

#if 1
	/* restore interrupts */
	local_irq_restore(irq_flags);
#endif
	printk(KERN_DEBUG "*(u32 *)packet_buffer_virt = 0x%08x.\n",*(u32 *)packet_buffer_virt);
	printk(KERN_DEBUG "Waiting for transfer to complete, pollcount = %u.\n", pollcount);
	

	/* the function servicing the engine will wake us */
	rc = wait_event_interruptible(packet_transfer->wq, packet_transfer->state != TRANSFER_STATE_SUBMITTED);
	printk(KERN_DEBUG "Transfer completed.\n");
#if 0
	printk(KERN_DEBUG "*(u32 *)length_buffer_virt = 0x%08x.\n",*(u32 *)length_buffer_virt);
#endif

	w = 0;
	//printk(KERN_DEBUG "About to perform iowrite32(0x%x @ 0x%p).\n", w, &reg->control);
	write_register(w, &reg->control);

fail_packet_pci_alloc:
#if 0
	/* free buffer for length DMA buffer */
	pci_free_consistent(lro->pci_dev, PAGE_SIZE, length_buffer_virt, length_buffer_bus);
fail_length_pci_alloc:
	/* fail length */
#endif
/* failed allocating pci consistent memory */
fail_desc:
	kfree(packet_transfer);

/* failed allocating transfer */
transfer_fail:
	if (gpio) {
		iounmap(gpio);
	}
ioremap_fail:
	return rc;
}

/**
 * see http://www.jstatsoft.org/v08/i14/ for algorithm and choice of constants
 */
static unsigned int xorshift_random(unsigned int x)
{
	x ^= (x << 3);
	x ^= (x >> 13);
	x ^= (x << 7);
	return x;
}


#define TEST_DMA_MAX_DESC (128)
	/* virtual addresses of the DMA buffers */
static	u8 *buffer_virt[TEST_DMA_MAX_DESC];

	/* bus address of the allocated buffer and descriptor */
static	dma_addr_t buffer_bus[TEST_DMA_MAX_DESC] = { 0 };
/* test_dma -- test SGDMA correctness
 *
 * A transfer is created which uses a list of descriptors laid in host memory
 * at a given address offset and with a given number of descriptors.
 *
 */
static int test_dma(struct lancero_dev *lro, unsigned int seed, int dir_to_dev)
{
	struct lancero_transfer *transfer;
	int i, res = 0, rc = 0, boundary_offset[TEST_DMA_MAX_DESC], size, bus;
	unsigned int fpga_addr = 0x00000000UL;
	u32 read_tester_counter = 0UL;

	printk(KERN_DEBUG "test_dma(seed = %u)\n", seed);

	/* allocate transfer data structure */
	transfer = kzalloc(sizeof(struct lancero_transfer), GFP_KERNEL);
	if (!transfer) {
		rc = -1;
		goto transfer_fail;
	}
	/* set transfer direction */
	transfer->dir_to_dev = dir_to_dev;

	/* determine number of descriptors */
	seed = xorshift_random(seed);
	transfer->desc_num = seed & (TEST_DMA_MAX_DESC - 1);
	if (transfer->desc_num == 0) transfer->desc_num = 1;

	/* override with fixed value? */
	if (desc_num >= 1) transfer->desc_num = desc_num;
	if (desc_num > TEST_DMA_MAX_DESC) transfer->desc_num = TEST_DMA_MAX_DESC;

	dbg_sg("transfer 0x%p has %d descriptors\n", transfer, transfer->desc_num);

	/* allocate descriptor list */
	transfer->desc_virt = lancero_desc_alloc(lro->pci_dev,
		transfer->desc_num, &transfer->desc_bus, NULL);
	if (!transfer->desc_virt) {
		printk(KERN_DEBUG "Could not allocate %d descriptors.\n", transfer->desc_num);
		res = -ENOMEM;
		goto fail_desc;
	}
#if 1
	printk(KERN_DEBUG "Allocated cache-coherent descriptor list of %d bytes;\n"
		"virtual address = 0x%016Lx;\n    bus address = 0x%016Lx).\n",
		transfer->desc_num * 8, (unsigned long long)transfer->desc_virt,
		(unsigned long long)transfer->desc_bus);
#endif
	/* iterate over all descriptors */
	for (i = 0; i < transfer->desc_num; i++) {
		int j;
		seed = xorshift_random(seed);
		size = seed & LANCERO_DESC_MAX_BYTES;
		seed = xorshift_random(seed);
		/* stay on the safe side of the address space */
		fpga_addr = seed & 0x3fffffffUL;
		fpga_addr &= ~(lro->align[dir_to_dev] - 1);

		/* truncate size to alignment */
		size &= ~(lro->align[dir_to_dev] - 1);
		/* if truncated to zero, pick minimum size */
		if (!size) size = lro->align[dir_to_dev];

		/* override with fixed value */
		if (test_dma_size >= 0) size = test_dma_size;

		/* allocate and map coherently-cached memory for a DMA-able buffer */
		/* @see Documentation/DMA-mapping.txt */
		buffer_virt[i] = (u8 *)pci_alloc_consistent(lro->pci_dev, size + PAGE_SIZE, &buffer_bus[i]);
		if (!buffer_virt[i]) {
			printk(KERN_DEBUG "Could not allocate coherent DMA buffer size %d + %d.\n", size, PAGE_SIZE);
			res = -ENOMEM;
			goto fail_pci_alloc;
		}
		printk(KERN_DEBUG "Allocated cache-coherent buffer of %d + %d bytes;\n"
			"virtual address = 0x%016Lx;\n    bus address = 0x%016Lx).\n",
			size, PAGE_SIZE, (unsigned long long)buffer_virt[i],
			(unsigned long long)buffer_bus[i]);
		seed = xorshift_random(seed);
		/* boundary in [0, 4096) */
		boundary_offset[i] = seed & (4096 - 1);
		/* align */
		boundary_offset[i] &= ~(lro->align[dir_to_dev] - 1);
		/* override with fixed value */
		if (test_dma_boundary_offset >= 0) boundary_offset[i] = test_dma_boundary_offset;
#if 0
		printk(KERN_DEBUG "%03d: virt=0x%08x, bus=0x%016llx, offset=0x%016llx, fpga_addr=0x%08x, size=0x%08x.\n",
			i, (unsigned long long)buffer_virt[i],
			(unsigned long long)buffer_bus[i],
			boundary_offset[i], fpga_addr, size);
#endif
		/* fill in descriptor entry with transfer details */
		lancero_desc_set(transfer->desc_virt + i, buffer_bus[i] + boundary_offset[i],
			fpga_addr, size);
		fpga_addr += size;

		/* initialize buffer content; j is the offset in the transfer */
		for (j = 0; j < (transfer->desc_virt + i)->bytes; j += 4) {
			u32 *p = (u32 *)(buffer_virt[i] + boundary_offset[i] + j);
			*p = (read_tester_counter++);
#if 0 /* inject an error to test the read tester fault detection functionality */
			/* test read tester by inserting errors */
			if (j == 0x100) *p = 0xdeadbeef;
#endif
		}
		printk(KERN_DEBUG "read_tester_counter = %u.\n", read_tester_counter);
	}
	i -= 1;
	/* terminate the last descriptor */
	lancero_desc_link(transfer->desc_virt + i, 0, 0);
	/* stop engine and request interrupt on last descriptor */
	lancero_desc_control(transfer->desc_virt + i, LANCERO_DESC_STOP | LANCERO_DESC_COMPLETED);
#if 0
	dump_transfer(transfer);
#endif
	/* initialize wait queue */
	init_waitqueue_head(&transfer->wq);
	/* reset the incremental data tester */
	tester_reset(lro, dir_to_dev? LANCERO_OFS_DMA_READ_TESTER: LANCERO_OFS_DMA_WRITE_TESTER);
#if 1
	/* queue on 0 = write engine, 1 = read engine */
	queue_transfer(lro->engine[dir_to_dev], transfer);
#if 0
	/* read the config module */
	if (config_offset != -1U) {
		bus = test_pcie_config(lro, config_offset);
		/* is the bus number still two? */
		if (bus != 2) {
			printk(KERN_CRIT "BUS IS NO LONGER 2.\n");
			rc = -5;
			goto fail_bus;
		}
	}

#endif /* config */
	printk(KERN_DEBUG "test_dma(): Waiting for the SGDMA engine to complete.\n");
	/* the function servicing the engine will wake us */
	rc = wait_event_interruptible(transfer->wq, transfer->state != TRANSFER_STATE_SUBMITTED);
	if (rc) {
		printk(KERN_ERR "wait_event_interruptible() = %d\n", rc);
		//dump_transfer(transfer);
		lancero_read_status(lro->engine[dir_to_dev], 0);
	}

#if 1


	/* data direction towards host? (i.e. write engine?) */
	if (!dir_to_dev) {
		read_tester_counter = 0UL;
		/* verify tester buffer against expected data */
		for (i = 0; i < transfer->desc_num; i++) {
			int j;
			/* j is the offset in the transfer */
			for (j = 0; j < (transfer->desc_virt + i)->bytes; j += 4) {
				u32 *p = (u32 *)(buffer_virt[i] + boundary_offset[i] + j);
				/* read word in the buffer */
				u32 w = le32_to_cpu(*p);
#if 0 /* print data verification even if matched? */
				printk(KERN_DEBUG "data @%p: 0x%08x matches  0x%08x\n",
					p, le32_to_cpu(*p), read_tester_counter);
#endif
#if 1 /* only print errors on mismatch */
				/* data mismatch? */
				if (w != read_tester_counter) {
					printk(KERN_ERR "data @%p: 0x%08x expected 0x%08x !!!\n",
						p, le32_to_cpu(*p), read_tester_counter);
					rc = -6;
					goto fail_verify;
				}
				read_tester_counter++;
#endif /* print verify */
			}
		}
	}
	/* in case of read engine, verify that the tester received expected data */
	if (dir_to_dev) {
		/* reset the incremental data tester */
		rc = tester_status(lro, dir_to_dev? LANCERO_OFS_DMA_READ_TESTER: LANCERO_OFS_DMA_WRITE_TESTER, read_tester_counter);
	}
	//printk(KERN_ERR "data verify ALL OK\n");
#endif /* verify */
#endif /* queue */
fail_verify:
	for (i = 0; i < transfer->desc_num; i++) {
#if 0
		printk(KERN_DEBUG "%03d: virt=0x%016llx, bus=0x%016llx, size=0x%016lx.\n",
			i, (unsigned long long)buffer_virt[i], (unsigned long long)buffer_bus[i],
			(transfer->desc_virt + i)->bytes + PAGE_SIZE);
#endif
		/* free DMA-able buffer */
		pci_free_consistent(lro->pci_dev, (transfer->desc_virt + i)->bytes + PAGE_SIZE,
			buffer_virt[i], buffer_bus[i]);
	}
fail_bus:
fail_pci_alloc:
	/* free descriptor list */
	lancero_desc_free(lro->pci_dev, transfer->desc_num,
		transfer->desc_virt, transfer->desc_bus);
/* failed allocating pci consistent memory */
fail_desc:
	/* free the transfer data structure */
	kfree(transfer);
/* failed allocating transfer */
transfer_fail:
	return rc;
}

/* test the write DMA engine */
static int test_dma_vectors(struct lancero_dev *lro, unsigned int seed, int dir_to_dev)
{
	int i = 0, rc;
	int runs = 10000;
	if (test_dma_runs) runs = test_dma_runs;
	printk(KERN_DEBUG "test_dma_runs = %d\n", test_dma_runs);
	/* number of test runs */
	while (i < runs) {
		/* perform SG DMA test */
		rc = test_dma(lro, seed, dir_to_dev);
		/* test failed, break loop */
		if (rc) break;
		/* advance seed */
		seed = xorshift_random(seed);
		i++;
		printk(KERN_DEBUG "test_dma_vectors() run %d of %d completed.\n", i, runs);
	}
	if (rc) {
		printk(KERN_ERR "test_dma_vectors() FAILED\n");
	} else {
		printk(KERN_ERR "test_dma_vectors() ALL GOOD\n");
	}
	return rc;
}

/* test the descriptor bridge using the bridge exerciser */
static int test_desc_vectors(struct lancero_dev *lro, unsigned int seed)
{
	int i = 0, rc;
	/* number of test vectors */
	while (i < 10000)
	{
		int number, offset;
		seed = xorshift_random(seed);
		/* number in [0, 16) */
		number = seed & ((1 << 4/* 4 bits field */) - 1);
		/* number in [1, 16] @todo +=1 */
		number &= 7;
		number = 1;
		/* offset in [0, 1024) */
		offset = (seed >> 6) & ((1 << 10) - 1);
		/* offset in [0, 4092), multiple of 4 */
		offset *= 4;
		offset = 0;
		//printk(KERN_DEBUG "i = %d, number = %d, offset = %d\n", i, number, offset);
		/* create and submit descriptor list */
		rc = test_desc_bridge(lro, tester_offset, number, offset, i);
		if (rc < 0) {
			printk(KERN_DEBUG "i = %d, number = %d, offset = %d, rc = %d\n", i, number, offset, rc);
			break;
		}
#if 0 /* delay inbetween descriptor tests? */
		msleep(1);
#endif
		i++;
	}
	printk(KERN_DEBUG "vector test rc = %d\n", rc);
	return rc;
}

/* create_transfer_user() - Create a DMA transfer to/from a user space buffer
 *
 * Allocates a transfer data structure and an array of descriptors. Builds a
 * descriptor list from the scatter gather list, coalescing adjacent entries.
 *
 * The scatterlist must have been mapped by pci_map_sg(sgm->sgl).
 *
 * @sgl scatterlist.
 * @nents Number of entries in the scatterlist after mapping by pci_map_sg().
 * @first Start index in the scatterlist sgm->sgl.
 *
 * Returns Number of entries in the table on success, -1 on error.
 */
static struct lancero_transfer *create_transfer_user(struct lancero_dev *lro, const char *start, size_t count, u32 ep_addr, int dir_to_dev)
{
	int i = 0, j = 0, new_desc, rc;
	dma_addr_t cont_addr, addr;
	unsigned int cont_len, len, cont_max_len = 0;
	struct scatterlist *sgl;

	/* allocate transfer data structure */
	struct lancero_transfer *transfer =
		kzalloc(sizeof(struct lancero_transfer), GFP_KERNEL);

	printk(KERN_INFO "create_transfer_user()\n");

	if (!transfer) return NULL;
	/* remember direction of transfer */
	transfer->dir_to_dev = dir_to_dev;

	/* create virtual memory mapper */
	transfer->sgm = sg_create_mapper(count);
	BUG_ON(!transfer->sgm);
	transfer->userspace = 1;

	/* lock user pages in memory and create a scatter gather list */
	rc = sgm_get_user_pages(transfer->sgm, start, count, !dir_to_dev);
	BUG_ON(rc < 0);

	sgl = transfer->sgm->sgl;

	dbg_sg("mapped_pages=%d.\n", transfer->sgm->mapped_pages);
	dbg_sg("sgl = 0x%p.\n", transfer->sgm->sgl);
	BUG_ON(!lro->pci_dev);
	BUG_ON(!transfer->sgm->sgl);
	BUG_ON(!transfer->sgm->mapped_pages);
	/* map all SG entries into DMA memory */
	transfer->sgl_nents = pci_map_sg(lro->pci_dev, transfer->sgm->sgl,
		transfer->sgm->mapped_pages, dir_to_dev? DMA_TO_DEVICE: DMA_FROM_DEVICE);
	dbg_sg("hwnents=%d.\n", transfer->sgl_nents);

	/* verify if the page start address got into the first sg entry */
	dbg_sg("sg_page(&sgl[0])=0x%p.\n", sg_page(&transfer->sgm->sgl[0]));
	dbg_sg("sg_dma_address(&sgl[0])=0x%016llx.\n", (u64)sg_dma_address(&transfer->sgm->sgl[0]));
	dbg_sg("sg_dma_len(&sgl[0])=0x%08x.\n", sg_dma_len(&transfer->sgm->sgl[0]));

	/* allocate descriptor list */
	transfer->desc_virt = lancero_desc_alloc(lro->pci_dev,
		transfer->sgl_nents, &transfer->desc_bus, NULL);
	WARN_ON((transfer->desc_bus >> 16) >> 16);
	dbg_sg("create_transfer_user():\n");
	dbg_sg("transfer->desc_bus = 0x%llx.\n", (u64)transfer->desc_bus);

	/* start first contiguous block */
	cont_addr = addr = sg_dma_address(&transfer->sgm->sgl[i]);
	cont_len = 0;

	/* iterate over all remaining entries but the last */
	for (i = 0; i < transfer->sgl_nents - 1; i++) {
		/* bus address of next entry i + 1 */
		dma_addr_t next = sg_dma_address(&sgl[i + 1]);
		/* length of this entry i */
		len = sg_dma_len(&sgl[i]);
		dbg_desc("SGLE %04d: addr=0x%016llx length=0x%08x\n", i, (u64)addr, len);

		/* add entry i to current contiguous block length */
		cont_len += len;

		new_desc = 0;
		/* entry i + 1 is non-contiguous with entry i? */
		if (next != addr + len) {
			dbg_desc("NON-CONTIGUOUS WITH DESC %d\n", i + 1);
			new_desc = 1;
		}
		/* entry i reached maximum transfer size? */
		else if (cont_len > (LANCERO_DESC_MAX_BYTES - PAGE_SIZE)) {
			dbg_desc("BREAK\n");
			new_desc = 1;
		}
		if (new_desc) {
			/* fill in descriptor entry j with transfer details */
			lancero_desc_set(transfer->desc_virt + j, cont_addr,
				ep_addr, cont_len);
#if FORCE_IR_DESC_COMPLETED
			lancero_desc_control(transfer->desc_virt + j, LANCERO_DESC_COMPLETED);
#endif
			if (cont_len > cont_max_len) {
				cont_max_len = cont_len;
				printk(KERN_DEBUG "LONGEST CONTIGUOUS LENGTH (SO FAR) = %d\n",
					cont_max_len);
			}
			dbg_desc("DESC %4d: cont_addr=0x%016llx cont_len=0x%08x, ep_addr=0x%lx\n",
				j, (u64)cont_addr, cont_len, (unsigned long)ep_addr);
			/* proceed EP address for next contiguous block */
			ep_addr += cont_len;
			/* start new contiguous block */
			cont_addr = next;
			cont_len = 0;
			j++;
		}
		/* goto entry i + 1 */
		addr = next;
	}
	/* i is the last entry in the scatterlist, add it to the last block */
	len = sg_dma_len(&sgl[i]);
	cont_len += len;
	BUG_ON(j > transfer->sgl_nents);

	/* j is the index of the last descriptor */

	dbg_desc("SGLE %04d: addr=0x%016llx length=0x%08x\n", i, (u64)addr, len);
	dbg_desc("DESC %4d: cont_addr=0x%016llx cont_len=0x%08x, ep_addr=0x%lx\n",
		j, (u64)cont_addr, cont_len, (unsigned long)ep_addr);

	/* XXX to test error condition, set cont_len = 0 */

	/* fill in last descriptor entry j with transfer details */
	lancero_desc_set(transfer->desc_virt + j, cont_addr,
		ep_addr, cont_len);
#if LANCERO_PERFORMANCE_TEST
	/* create a linked loop */
	lancero_desc_link(transfer->desc_virt + j, transfer->desc_virt, transfer->desc_bus);
#  if FORCE_IR_DESC_COMPLETED
	/* request IRQ on last descriptor */
	lancero_desc_control(transfer->desc_virt + j, LANCERO_DESC_COMPLETED);
#  endif
#else
	/* terminate last descriptor */
	lancero_desc_link(transfer->desc_virt + j, 0, 0);
	/* request IRQ on last descriptor */
	lancero_desc_control(transfer->desc_virt + j, LANCERO_DESC_STOP | LANCERO_DESC_COMPLETED);
#endif

	j++;
	/* j is the number of descriptors */
	transfer->desc_num = transfer->desc_adjacent = j;

	dbg_sg("transfer 0x%p has %d descriptors\n", transfer, transfer->desc_num);
	/* fill in adjacent numbers */
	for (i = 0; i < transfer->desc_num; i++) {
		lancero_desc_adjacent(transfer->desc_virt + i,
			transfer->desc_num - i - 1);
	}

	/* initialize wait queue */
	init_waitqueue_head(&transfer->wq);

	return transfer;
}

/* create_transfer_kernel() - Create a DMA transfer to/from a kernel buffer
 *
 * Allocates a transfer data structure and an array of descriptors. Builds a
 * descriptor list from the scatter gather list, coalescing adjacent entries.
 *
 * The scatterlist must have been mapped by pci_map_sg(sgm->sgl).
 *
 * vmalloc_to_pfn
 *
 * @sgl scatterlist.
 * @nents Number of entries in the scatterlist after mapping by pci_map_sg().
 * @first Start index in the scatterlist sgm->sgl.
 *
 * Returns Number of entries in the table on success, -1 on error.
 */
static struct lancero_transfer *create_transfer_kernel(struct lancero_dev *lro, const char *start, size_t count, u32 ep_addr, int dir_to_dev)
{
	int i = 0, j = 0, new_desc, rc;
	dma_addr_t cont_addr, addr;
	unsigned int cont_len, len, cont_max_len = 0;
	struct scatterlist *sgl;

	/* allocate transfer data structure */
	struct lancero_transfer *transfer =
		kzalloc(sizeof(struct lancero_transfer), GFP_KERNEL);

	printk(KERN_INFO "create_transfer_kernel()\n");

	if (!transfer) return NULL;
	/* remember direction of transfer */
	transfer->dir_to_dev = dir_to_dev;

	/* create virtual memory mapper */
	transfer->sgm = sg_create_mapper(count);
	BUG_ON(!transfer->sgm);

	/* create a scatter gather list */
	rc = sgm_kernel_pages(transfer->sgm, start, count, !dir_to_dev);
	BUG_ON(rc < 0);

	sgl = transfer->sgm->sgl;

	dbg_sg("mapped_pages=%d.\n", transfer->sgm->mapped_pages);
	dbg_sg("sgl = 0x%p.\n", transfer->sgm->sgl);
	BUG_ON(!lro->pci_dev);
	BUG_ON(!transfer->sgm->sgl);
	BUG_ON(!transfer->sgm->mapped_pages);
	/* map all SG entries into DMA memory */
	transfer->sgl_nents = pci_map_sg(lro->pci_dev, transfer->sgm->sgl,
		transfer->sgm->mapped_pages, dir_to_dev? DMA_TO_DEVICE : DMA_FROM_DEVICE);
	dbg_sg("hwnents=%d.\n", transfer->sgl_nents);

	/* verify if the page start address got into the first sg entry */
	dbg_sg("sg_page(&sgl[0])=0x%p.\n", sg_page(&transfer->sgm->sgl[0]));
	dbg_sg("sg_dma_address(&sgl[0])=0x%016llx.\n", (u64)sg_dma_address(&transfer->sgm->sgl[0]));
	dbg_sg("sg_dma_len(&sgl[0])=0x%08x.\n", sg_dma_len(&transfer->sgm->sgl[0]));

	/* allocate descriptor list */
	transfer->desc_virt = lancero_desc_alloc(lro->pci_dev,
		transfer->sgl_nents, &transfer->desc_bus, NULL);
	dbg_sg("create_transfer_user():\n");
	dbg_sg("transfer->desc_bus = 0x%llx.\n", (u64)transfer->desc_bus);

	/* start first contiguous block */
	cont_addr = addr = sg_dma_address(&transfer->sgm->sgl[i]);
	cont_len = 0;

	/* iterate over all remaining entries but the last */
	for (i = 0; i < transfer->sgl_nents - 1; i++) {
		/* bus address of next entry i + 1 */
		dma_addr_t next = sg_dma_address(&sgl[i + 1]);
		/* length of this entry i */
		len = sg_dma_len(&sgl[i]);
		dbg_desc("SGLE %04d: addr=0x%016llx length=0x%08x\n", i, (u64)addr, len);

		/* add entry i to current contiguous block length */
		cont_len += len;

		new_desc = 0;
		/* entry i + 1 is non-contiguous with entry i? */
		if (next != addr + len) {
			dbg_desc("NON CONTIGUOUS\n");
			new_desc = 1;
		}
		/* entry i reached maximum transfer size? */
		else if (cont_len > (LANCERO_DESC_MAX_BYTES - PAGE_SIZE)) {
			dbg_desc("BREAK\n");
			new_desc = 1;
		}
#if LANCERO_FRAMEBUFFER		
		new_desc = 1;
#endif		
		if (new_desc) {
			/* fill in descriptor entry j with transfer details */
			lancero_desc_set(transfer->desc_virt + j, cont_addr,
				ep_addr, cont_len);
#if FORCE_IR_DESC_COMPLETED
			lancero_desc_control(transfer->desc_virt + j, LANCERO_DESC_COMPLETED);
#endif
			if (cont_len > cont_max_len) {
				cont_max_len = cont_len;
				printk(KERN_DEBUG "DESC %4d: cont_addr=0x%016llx cont_len=0x%08x, ep_addr=0x%lx\n",
				j, (u64)cont_addr, cont_len, (unsigned long)ep_addr);
			}
			dbg_desc("DESC %4d: cont_addr=0x%016llx cont_len=0x%08x, ep_addr=0x%lx\n",
				j, (u64)cont_addr, cont_len, (unsigned long)ep_addr);
			/* proceed EP address for next contiguous block */
			ep_addr += cont_len;
			/* start new contiguous block */
			cont_addr = next;
			cont_len = 0;
			j++;
		}
		/* goto entry i + 1 */
		addr = next;
	}
	/* i is the last entry in the scatterlist, add it to the last block */
	len = sg_dma_len(&sgl[i]);
	cont_len += len;
	BUG_ON(j > transfer->sgl_nents);

	/* j is the index of the last descriptor */

	dbg_desc("SGLE %04d: addr=0x%016llx length=0x%08x\n", i, (u64)addr, len);
	dbg_desc("DESC %4d: cont_addr=0x%016llx cont_len=0x%08x, ep_addr=0x%lx\n",
		j, (u64)cont_addr, cont_len, (unsigned long)ep_addr);

	/* XXX to test error condition, set cont_len = 0 */

	/* fill in last descriptor entry j with transfer details */
	lancero_desc_set(transfer->desc_virt + j, cont_addr,
		ep_addr, cont_len);
#if LANCERO_PERFORMANCE_TEST
	/* create a linked loop */
	lancero_desc_link(transfer->desc_virt + j, transfer->desc_virt, transfer->desc_bus);
#else
	/* terminate last descriptor */
	lancero_desc_link(transfer->desc_virt + j, 0, 0);
	/* request IRQ on last descriptor */
	lancero_desc_control(transfer->desc_virt + j, LANCERO_DESC_STOP | LANCERO_DESC_COMPLETED);
#endif

	j++;
	/* j is the number of descriptors */
	transfer->desc_num = transfer->desc_adjacent = j;

	dbg_sg("transfer 0x%p has %d descriptors\n", transfer, transfer->desc_num);
	/* fill in adjacent numbers */
	for (i = 0; i < transfer->desc_num; i++) {
		lancero_desc_adjacent(transfer->desc_virt + i,
			transfer->desc_num - i - 1);
	}

	/* initialize wait queue */
	init_waitqueue_head(&transfer->wq);

	return transfer;
}

/* sg_aio_read_write() -- Read from or write to the device
 *
 * @buf userspace buffer
 * @count number of bytes in the userspace buffer
 * @pos byte-address in device
 * @dir_to_device If !0, a write to the device is performed
 *
 * Iterate over the userspace buffer, taking at most 255 * PAGE_SIZE bytes for
 * each DMA transfer.
 *
 * For each transfer, get the user pages, build a sglist, map, build a
 * descriptor table. submit the transfer. wait for the interrupt handler
 * to wake us on completion.
 */
static ssize_t sg_aio_read_write(struct kiocb *iocb, const struct iovec *iov,
	unsigned long nr_segs, loff_t pos, int dir_to_dev)
{
	/* fetch file this io request acts on */
	struct file *file = iocb->ki_filp;
	loff_t *ppos = &iocb->ki_pos;
	size_t total_done = 0;
	unsigned long seg;

	struct lancero_char *lro_char;
	struct lancero_dev *lro;
	struct lancero_engine *engine;

	/* fetch device specific data stored earlier during open */
	lro_char = (struct lancero_char *)file->private_data;
	BUG_ON(!lro_char);
	BUG_ON(lro_char->magic != MAGIC_CHAR);

	lro = lro_char->lro;
	BUG_ON(!lro);
	BUG_ON(lro->magic != MAGIC_DEVICE);

	printk(KERN_INFO "sg_aio_read_write(iocb=0x%p, iov=0x%p, nr_segs=%ld, pos=%llu, dir_to_dev=%d) %s request\n",
		iocb, iov, nr_segs, (u64)pos, dir_to_dev, dir_to_dev? "write" : "read");

	engine = dir_to_dev? lro_char->read_engine : lro_char->write_engine;
	/* XXX detect not supported direction */
	BUG_ON(!engine);
	BUG_ON(engine->magic != MAGIC_ENGINE);
	/* iterate over all vector segments */
	for (seg = 0; seg < nr_segs; seg++) {
		const char __user *buf = iov[seg].iov_base;
		size_t count = iov[seg].iov_len;
		size_t remaining = count, done = 0;
		char *transfer_addr = (char *)buf;
		/* assert correct base address alignment - lower bits must be zero */
		if ((int)buf & (lro->align[dir_to_dev] - 1)) return -EINVAL;
		/* assert size alignment - lower bits must be zero */
		if (count & (lro->align[dir_to_dev] - 1)) return -EINVAL;
		printk(KERN_INFO "seg %lu: buf=0x%p, count=%lld, pos=%llu\n",
			seg, buf, (s64)count, (u64)pos);
		/* anything left to transfer? */
		while (remaining > 0) {
			struct lancero_transfer *transfer;
			/* DMA transfer size, multiple if necessary */
			size_t transfer_len = (remaining > LANCERO_TRANSFER_MAX_BYTES) ?
				LANCERO_TRANSFER_MAX_BYTES : remaining;

			/* build device-specific descriptor tables */
			transfer = create_transfer_user(lro, transfer_addr, transfer_len,
				pos, dir_to_dev);
			dbg_sg("segment:%lu transfer=0x%p.\n", seg, transfer);
			BUG_ON(!transfer);

			if (!transfer) {
				remaining = 0;
				done = 0;
				printk(KERN_DEBUG "Could not allocate memory for transfer!");
				return -ENOMEM;
			}
			/* remember I/O context for later completion */
			transfer->iocb = iocb;
			/* last transfer for the given request? */
			if (transfer_len >= remaining) {
				/* mark as last transfer, using request size */
				transfer->last_in_request = 1;
				transfer->size_of_request = done + transfer_len;
			}
			/* queue the transfer on the hardware */
			queue_transfer(engine, transfer);
			/* calculate the next transfer */
			transfer_addr += transfer_len;
			remaining -= transfer_len;
			done += transfer_len;
			printk(KERN_DEBUG "remaining = %lld, done = %lld.\n",
				(s64)remaining, (s64)done);
		}
		total_done += done;
	}
	printk(KERN_INFO "sg_aio_read_write() queued a total of %lld bytes, returns -EIOCBQUEUED.\n", (s64)total_done);
	return -EIOCBQUEUED;
}

/**
* sg_aio_read_write - generic asynchronous read routine
* @iocb:       kernel I/O control block
* @iov:        io vector request
* @nr_segs:    number of segments in the iovec
* @pos:        current file position
*
*/
ssize_t sg_aio_read(struct kiocb *iocb, const struct iovec *iov,
unsigned long nr_segs, loff_t pos)
{
        printk(KERN_DEBUG "sg_aio_read()\n");
        return sg_aio_read_write(iocb, iov, nr_segs, pos, 0/*dir_to_dev = 0*/);
}

ssize_t sg_aio_write(struct kiocb *iocb, const struct iovec *iov,
unsigned long nr_segs, loff_t pos)
{
        printk(KERN_DEBUG "sg_aio_write()\n");
        return sg_aio_read_write(iocb, iov, nr_segs, pos, 1/*dir_to_dev = 1*/);
}

/* @TODO make dependent on kernel version, as this is an internal API change */
static ssize_t sg_read_iter(struct kiocb *iocb, struct iov_iter *to)
{
    return sg_aio_read_write(iocb, to->iov, to->nr_segs, iocb->ki_pos, 0);
}
static ssize_t sg_write_iter(struct kiocb *iocb, struct iov_iter *to)
{
    return sg_aio_read_write(iocb, to->iov, to->nr_segs, iocb->ki_pos, 1);
}


static ssize_t char_sgdma_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
}

static ssize_t char_cyclic_read_write(struct file *file, char __user *buf,
size_t count, loff_t *pos, int dir_to_dev)
{
	char *transfer_addr = (char *)buf;
 	unsigned long address_low_bits = 0;
	size_t length_low_bits = 0;
	struct lancero_char *lro_char;
	struct lancero_dev *lro;
	struct lancero_engine *engine;
	struct lancero_transfer *transfer;

	printk(KERN_DEBUG "char_ring_read_write().\n");

	/* fetch device specific data stored earlier during open */
	lro_char = (struct lancero_char *)file->private_data;
	lro = lro_char->lro;

	engine = dir_to_dev? lro_char->read_engine : lro_char->write_engine;
	address_low_bits = (unsigned long)buf & (lro->align[dir_to_dev] - 1);

	/* assert correct base address alignment - lower bits must be zero */
	if (address_low_bits)
	{
		return -EINVAL;
	}
	/* assert size alignment - lower bits must be zero */
	length_low_bits = count & ((size_t)lro->align[dir_to_dev] - 1);
	/* low bits are set? */
	if (length_low_bits) {
		return -EINVAL;
	}
        /* more than allowed for a single descriptor list? */
	if (count > LANCERO_TRANSFER_MAX_BYTES) {
		return -EINVAL;
        }

	/* build device-specific descriptor tables */
	transfer = create_transfer_user(lro, transfer_addr,
		count, *pos, dir_to_dev);
	WARN_ON(!transfer);
	if (!transfer) {
		return -EIO;
	}
        /* request consisting of a single transfer */
	transfer->last_in_request = 1;
	transfer->size_of_request = count;
        /* make this a cyclic transfer */
        lancero_transfer_cyclic(transfer);
	/* only interrupt on wrap */
	lancero_desc_control(transfer->desc_virt + transfer->desc_num - 1, LANCERO_DESC_COMPLETED);
	/* let the device read from the host */
	queue_transfer(engine, transfer);
	return count;
}


/* sg_read_write() -- Read from or write to the device
 *
 * @buf userspace buffer
 * @count number of bytes in the userspace buffer
 * @pos byte-address in device
 * @dir_to_device If !0, a write to the device is performed
 *
 * Iterate over the userspace buffer, taking at most 255 * PAGE_SIZE bytes for
 * each DMA transfer.
 *
 * For each transfer, get the user pages, build a sglist, map, build a
 * descriptor table. submit the transfer. wait for the interrupt handler
 * to wake us on completion.
 */

static ssize_t char_sgdma_read_write(struct file *file, char __user *buf,
size_t count, loff_t *pos, int dir_to_dev)
{
	int rc;
	ssize_t res = 0;
	static int counter = 0;
	int seq = counter++;
	ssize_t remaining = count, done = 0;
	char *transfer_addr = (char *)buf;
 	unsigned long address_low_bits = 0;
	size_t length_low_bits = 0;
#if LANCERO_PERFORMANCE_TEST
	int performance_period = 1000;
#endif
	struct lancero_char *lro_char;
	struct lancero_dev *lro;
	struct lancero_engine *engine;

	/* fetch device specific data stored earlier during open */
	lro_char = (struct lancero_char *)file->private_data;
	BUG_ON(!lro_char);
	BUG_ON(lro_char->magic != MAGIC_CHAR);

	lro = lro_char->lro;
	BUG_ON(!lro);
	BUG_ON(lro->magic != MAGIC_DEVICE);

	engine = dir_to_dev? lro_char->read_engine : lro_char->write_engine;
	/* XXX detect non-supported directions XXX */
	BUG_ON(!engine);
	BUG_ON(engine->magic != MAGIC_ENGINE);

	printk(KERN_INFO "seq:%d sg_read_write(file=0x%p, buf=0x%p, count=%lld, pos=%llu, dir_to_dev=%d) %s request\n",
		seq, file, buf, (s64)count, (u64)*pos, dir_to_dev, dir_to_dev? "write" : "read");
	printk(KERN_INFO "engine = 0x%p\n", engine);
	printk(KERN_INFO "lro = 0x%p\n", lro);

	address_low_bits = (unsigned long)buf & (lro->align[dir_to_dev] - 1);

	/* assert correct base address alignment - lower bits must be zero */
	if (address_low_bits)
	{
#if PACKET_LENGTH_TABLE
		printk(KERN_DEBUG "address low bits are 0x%06x\n", (int)address_low_bits);
		printk(KERN_DEBUG "PACKET_LENGTH_TABLE: dir_to_dev=%d\n", dir_to_dev);
		/* read engine will use the address as the Avalon packet length */
		if (dir_to_dev) {
			printk(KERN_DEBUG "Sending packet with Ethernet length %d, Avalon length %d\n", (int)buf, count);
			/* write engine does not support non-aligned transfers */
		} else {
			return -EINVAL;
		}
#else
		return -EINVAL;
#endif
	}
	/* assert size alignment - lower bits must be zero */
	length_low_bits = count & ((size_t)lro->align[dir_to_dev] - 1);
	/* strip the low bits off the count */
	count &= ~(lro->align[dir_to_dev] - 1);
	remaining = count;
	/* low bits are set? */
	if (length_low_bits) {
#if PACKET_LENGTH_TABLE
	if (length_low_bits > 2)
		return -EINVAL;
	/* for setting the packet length table buffer, use length offset 1 */
	if ((length_low_bits == 1) && (dir_to_dev == 0)) {
		printk(KERN_DEBUG "length low bits are 0x%06llx\n", (unsigned long long)length_low_bits);
		/*  */
		if (count) {
			printk(KERN_DEBUG "PACKET_LENGTH_TABLE: length low bits are 1, setting length table buffer.\n");
			printk(KERN_DEBUG "PACKET_LENGTH_TABLE virtual address = 0x%16llx, length = 0x%8lx\n",
				(unsigned long long)buf, count);
			/* be lazy, create a transfer so that the corresponding user
			* pages are mapped */
			struct lancero_transfer *transfer;
			transfer = create_transfer_user(lro, buf, count, 0, 0/* = dir_to_dev */);
			WARN_ON(!transfer);
			printk(KERN_DEBUG "PACKET_LENGTH_TABLE pcie bus address = 0x%08lx.%08lx, count = 0x%08lx\n",
				transfer->desc_virt->pcie_addr_hi, transfer->desc_virt->pcie_addr_lo, count);
			/* invalidate the length table, reset */
			iowrite32(0, &engine->regs->length_table_len);
			iowrite32(transfer->desc_virt->pcie_addr_lo, &engine->regs->length_table_lo);
			iowrite32(transfer->desc_virt->pcie_addr_hi, &engine->regs->length_table_hi);
			iowrite32(count, &engine->regs->length_table_len);
		} else {
			iowrite32(0, &engine->regs->length_table_len);
		}
	}	
#else
		return -EINVAL;
#endif
	}

#if LANCERO_RINGBUFFER        
    /* start a non-blocking and cyclic read or write operation? */
	if (file->f_flags & O_NONBLOCK) {
		return char_cyclic_read_write(file, buf, count, pos, dir_to_dev);
	}
#endif        

	/* still good and anything left to transfer? */
	while ((res == 0) && (remaining > 0)) {
		struct lancero_transfer *transfer;
		/* DMA transfer size, multiple if necessary */
		size_t transfer_len = (remaining > LANCERO_TRANSFER_MAX_BYTES) ?
			LANCERO_TRANSFER_MAX_BYTES : remaining;

		/* build device-specific descriptor tables */
		transfer = create_transfer_user(lro, transfer_addr,
			transfer_len, *pos, dir_to_dev);
		printk(KERN_INFO "seq:%d transfer=0x%p.\n", seq, transfer);
		BUG_ON(!transfer);
		if (!transfer) {
			remaining = 0;
			res = -EIO;
			continue;
		}
#if PACKET_LENGTH_TABLE
	if ((length_low_bits == 1) && (dir_to_dev == 0)) {
		printk(KERN_DEBUG "PACKET_LENGTH_TABLE returning count = 0x%08lx\n", (count | length_low_bits));
		return (count | length_low_bits);
	} else

	/* length offset 2 for ringbuffer */
	if ((length_low_bits == 2) && (dir_to_dev == 0)) {
		/* not supported, we want a single cyclic buffer */
		WARN_ON(transfer_len < remaining);
		/* create a linked loop */
		printk(KERN_DEBUG "PACKET_LENGTH_TABLE: length low bits are 2, creating cyclic transfer.\n");
			lancero_desc_link(transfer->desc_virt + transfer->desc_num - 1,
		transfer->desc_virt, transfer->desc_bus);
		transfer->cyclic = 1;
		lancero_desc_control(transfer->desc_virt + transfer->desc_num - 1, 0);
	}
#endif /* PACKET_LENGTH_TABLE */
		/* last transfer for the given request? */
		if (transfer_len >= remaining) {
			transfer->last_in_request = 1;
			transfer->size_of_request = done + transfer_len;
		}
		//dump_transfer(transfer);
#if FPGA_DESCRIPTORS
		printk(KERN_DEBUG "on-chip descriptor list:\n");
		lancero_desc_copylocal(lro, 0, transfer->desc_virt);
#endif /* FPGA_DESCRIPTORS */

#if LANCERO_PERFORMANCE_TEST
		/* start measurement, will commence on first SOP, will
		 * end after performance period and will then interrupt
		 * us. */
		performance_start(lro, dir_to_dev? LANCERO_OFS_DMA_READ_PERF: LANCERO_OFS_DMA_WRITE_PERF, performance_interval * 1000);
#endif
#if 1 /* start SGDMA */
		/* let the device read from the host */
		queue_transfer(engine, transfer);
#else /* simulate SGDMA */
		transfer->state = TRANSFER_STATE_COMPLETED;
#endif
#if 0
#if LANCERO_PERFORMANCE_TEST
		/* wait a bit longer */
		printk(KERN_DEBUG "SLEEPING %d ms\n", performance_period + 100);
		msleep(performance_period + 100);
		/* then stop on the first descriptor */
		//lancero_desc_control(transfer->desc_virt, LANCERO_DESC_STOP | LANCERO_DESC_COMPLETED);
		printk(KERN_DEBUG "DONE SLEEPING.\n");
		lancero_stop(engine);
		printk(KERN_DEBUG "DONE STOPPING.\n");
		//dump_transfer(transfer);
#endif
#endif
		/* the function servicing the engine will wake us */
		rc = wait_event_interruptible(transfer->wq, transfer->state != TRANSFER_STATE_SUBMITTED);
		if (rc) printk(KERN_INFO "seq:%d wait_event_interruptible() = %d\n", seq, rc);

#if PACKET_LENGTH_TABLE
		/* stop cyclic transfer */
		transfer->cyclic = 0;
		lancero_desc_control(transfer->desc_virt + transfer->desc_num - 1, LANCERO_DESC_COMPLETED | LANCERO_DESC_STOP);
//		lancero_desc_link(transfer->desc_virt + transfer->desc_num - 1, NULL, 0);
		printk(KERN_DEBUG "engine is being stopped\n");
		engine_reset(engine);
#endif

		/* transfer was taken off the engine? */
		if (transfer->state != TRANSFER_STATE_SUBMITTED) {
			/* transfer failed? */
			if (transfer->state != TRANSFER_STATE_COMPLETED) {
				transfer_len = remaining = 0;
				res = -EIO;
			}
			free_transfer(lro, transfer);
		/* wait_event_interruptible() was interrupted by a signal? */
		} else if (rc == -ERESTARTSYS) {
			printk(KERN_DEBUG "wait_event_interruptible(transfer->wq) == ERESTARTSYS\n");
			/* transfer can still be in-flight */
			lancero_read_status(engine, 0);
			read_interrupts(lro, LANCERO_OFS_INT_CTRL);
			transfer_len = remaining = 0;

#if 0
			/* request engine to shutdown */
			printk(KERN_DEBUG "requesting engine shutdown, waiting for completion\n");
			engine_shutdown(engine);
			/* wait until engine is shutdown */
			rc = wait_event_interruptible(engine->shutdown_wq, engine->shutdown & 2);
			printk(KERN_DEBUG "engine is shutdown\n");
#else
			printk(KERN_DEBUG "waiting for transfer 0x%p to complete.\n", transfer);
			rc = wait_event_interruptible(transfer->wq, transfer->state != TRANSFER_STATE_SUBMITTED);
			printk(KERN_DEBUG "transfer 0x%p has completed, transfer->state = %d\n", transfer->state);
#endif
			res = -ERESTARTSYS;
			if (transfer->state != TRANSFER_STATE_SUBMITTED) {
				free_transfer(lro, transfer);
			}
		}
		/* calculate the next transfer */
		transfer_addr += transfer_len;
		remaining -= transfer_len;
		done += transfer_len;
		printk(KERN_DEBUG "remaining = %lld, done = %lld.\n",
			(s64)remaining, (s64)done);
	}
	/* return error or else number of bytes */
	res = res ? res : done;
	printk(KERN_INFO "seq:%d sg_read_write() returns %lld.\n", seq, (s64)res);
	return res;
}

/* sg_write() -- Write to the device
 *
 * @buf userspace buffer
 * @count number of bytes in the userspace buffer
 * @pos byte-address in device
 */
static ssize_t char_sgdma_write(struct file *file, const char __user *buf,
size_t count, loff_t *pos)
{
	return char_sgdma_read_write(file, (char *)buf, count, pos, 1/*dir_to_dev = 1*/);
}

/* char_sgdma_read() - Read from the device
 *
 * @buf userspace buffer
 * @count number of bytes in the userspace buffer
 *
 * Iterate over the userspace buffer, taking at most 255 * PAGE_SIZE bytes for
 * each DMA transfer.
 *
 * For each transfer, get the user pages, build a sglist, map, build a
 * descriptor table, submit the transfer, wait for the interrupt handler
 * to wake us on completion, free the sglist and descriptors.
 */
static ssize_t char_sgdma_read(struct file *file, char __user *buf,
size_t count, loff_t *pos)
{
	return char_sgdma_read_write(file, buf, count, pos, 0/*dir_to_dev = 0*/);
}

/*
 * Called when the device goes from unused to used.
 */
static int char_open(struct inode *inode, struct file *file)
{
	struct lancero_char *lro_char;
	printk(KERN_DEBUG DRV_NAME "_open(0x%p, 0x%p)\n", inode, file);
	/* pointer to containing data structure of the character device inode */
	lro_char = container_of(inode->i_cdev, struct lancero_char, cdev);
	BUG_ON(lro_char->magic != MAGIC_CHAR);
	printk(KERN_DEBUG "lro_char = 0x%p\n", lro_char);
	printk(KERN_DEBUG "lro_char->read_engine = 0x%p\n", lro_char->read_engine);
	printk(KERN_DEBUG "lro_char->write_engine = 0x%p\n", lro_char->write_engine);

	/* create a reference to our char device in the opened file */
	file->private_data = lro_char;
	return 0;
}

/*
 * Called when the device goes from used to unused.
 */
static int char_close(struct inode *inode, struct file *file)
{
	struct lancero_dev *lro;
	struct lancero_char *lro_char = (struct lancero_char *)file->private_data;
	BUG_ON(!lro_char);
	BUG_ON(lro_char->magic != MAGIC_CHAR);

	/* fetch device specific data stored earlier during open */
	lro = lro_char->lro;
	BUG_ON(!lro);
	BUG_ON(lro->magic != MAGIC_DEVICE);
	printk(KERN_DEBUG DRV_NAME "_close(0x%p, 0x%p)\n", inode, file);
#if LANCERO_RINGBUFFER
        /* stop cyclic transfer on the write engine */
        if (lro_char->write_engine) {
                engine_cyclic_stop(lro_char->write_engine);
        }
        /* stop cyclic transfer on the read engine */
        if (lro_char->read_engine) {
                engine_cyclic_stop(lro_char->read_engine);
        }
#endif
	return 0;
}

static int __devinit probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	int rc = 0;
	u16 dcr;
	struct lancero_dev *lro = NULL;

	printk(KERN_DEBUG "probe(pdev = 0x%p, pci_id = 0x%p)\n", pdev, id);

	/* allocate zeroed device book keeping structure */
	lro = kzalloc(sizeof(struct lancero_dev), GFP_KERNEL);
	if (!lro) {
		printk(KERN_DEBUG "Could not kzalloc(lancero_dev).\n");
		goto err_alloc;
	}
	lro->magic = MAGIC_DEVICE;
	/* create a device to driver reference */
	dev_set_drvdata(&pdev->dev, lro);
	/* create a driver to device reference */
	lro->pci_dev = pdev;
	printk(KERN_DEBUG "probe() lro = 0x%p\n", lro);

	printk(KERN_DEBUG "pci_enable_device()\n");
	rc = pci_enable_device(pdev);
	if (rc) {
		printk(KERN_DEBUG "pci_enable_device() failed, rc = %d.\n", rc);
		goto err_enable;
	}

	/* enable bus master capability */
	printk(KERN_DEBUG "pci_set_master()\n");
	pci_set_master(pdev);

	if (msi) {
		/* enable message signaled interrupts */
		printk(KERN_DEBUG "pci_enable_msi()\n");
		rc = pci_enable_msi(pdev);
		/* could not use MSI? */
		if (rc) {
			printk(KERN_DEBUG "Could not enable MSI interrupting; rc = %d.\n", rc);
#if 0
			rc = -1;
			goto err_msi;
#endif
		} else {
			lro->msi_enabled = 1;
		}
	}

	printk(KERN_DEBUG "pci_read_config_byte(PCI_REVISION_ID).\n");
	pci_read_config_byte(pdev, PCI_REVISION_ID, &lro->revision);
	/* is the revision number supported by this driver? */
	if (lro->revision > LANCERO_KNOWN_REVISION) {
		printk(KERN_DEBUG "Revision 0x%02x is not supported.\n", lro->revision);
		rc = -ENODEV;
		goto err_rev;
	}
	printk(KERN_DEBUG "Device Revision: 0x%02x.\n", lro->revision);

	/* known root complex's max read request sizes */
#ifdef CONFIG_ARCH_TI816X
	printk(KERN_DEBUG "TI816X RC detected: limiting MaxReadReq size to 128 bytes.\n");
	pcie_set_readrq(pdev, 128);
#endif

	pci_read_config_word(pdev, 0x88, &dcr);
	printk(KERN_DEBUG "Device Control Register: 0x%04x.\n", dcr);
	if (payload == 256) {
		dcr |= 0x0020U;
		pci_write_config_word(pdev, 0x88, dcr);
		/* read back */
		pci_read_config_word(pdev, 0x88, &dcr);
		printk(KERN_DEBUG "Device Control Register: 0x%04x.\n", dcr);
	}

	printk(KERN_DEBUG "pci_request_regions()\n");
	rc = pci_request_regions(pdev, DRV_NAME);
	/* could not request all regions? */
	if (rc) {
		printk(KERN_DEBUG "pci_request_regions() = %d, device in use?\n", rc);
		/* assume device is in use so do not disable it later */
		lro->regions_in_use = 1;
		goto err_regions;
	}
	lro->got_regions = 1;

	printk(KERN_DEBUG "map_bars()\n");
	/* map BARs */
	rc = map_bars(lro, pdev);
	if (rc)
		goto err_map;

	printk(KERN_DEBUG "lancero_config()\n");
	/* determine Lancero system configuration */
	rc = lancero_config(lro, 0x000);
	if (rc)
		goto err_system;
	
	/* any engine available? */
	if (lro->capabilities & (CAP_ENGINE_WRITE | CAP_ENGINE_READ)) {
		/* assume 64-bit DMA and descriptor capability, later
		 * engine initialization possibly clears it */
		lro->capabilities |= CAP_64BIT_DESC | CAP_64BIT_DMA;
	}
	/* write engine available? */
	if (lro->capabilities & CAP_ENGINE_WRITE) {	
		rc = engine_initialize(lro, LANCERO_OFS_DMA_WRITE);
	}
	/* read engine available? */
	if (lro->capabilities & CAP_ENGINE_READ) {	
		rc = engine_initialize(lro, LANCERO_OFS_DMA_READ);
	}
	printk("Lancero IP Core does %ssupport 64-bit DMA.\n",
		(lro->capabilities & CAP_64BIT_DMA)?"":"not ");

	printk("sizeof(dma_addr_t) == %d\n", sizeof(dma_addr_t));
	/* 64-bit addressing capability for SGDMA? */
	if ((lro->capabilities & CAP_64BIT_DMA) &&
		(!pci_set_dma_mask(pdev, DMA_64BIT_MASK))) {
		/* query for DMA transfer */
		/* @see Documentation/DMA-mapping.txt */
		printk(KERN_DEBUG "pci_set_dma_mask()\n");
		/* use 64-bit DMA */
		printk(KERN_DEBUG "Using a 64-bit DMA mask.\n");
		/* use 32-bit DMA for descriptors */
		pci_set_consistent_dma_mask(pdev, DMA_32BIT_MASK);
		/* use 64-bit DMA, 32-bit for consistent */
	} else
	if (!pci_set_dma_mask(pdev, DMA_32BIT_MASK)) {
		printk(KERN_DEBUG "Could not set 64-bit DMA mask.\n");
		pci_set_consistent_dma_mask(pdev, DMA_32BIT_MASK);
		/* use 32-bit DMA */
		printk(KERN_DEBUG "Using a 32-bit DMA mask.\n");
	} else {
		printk(KERN_DEBUG "No suitable DMA possible.\n");
		/** @todo Choose proper error return code */
		rc = -1;
		goto err_mask;
	}
	lro->irq_line = -1;
	/* request irq, MSI interrupts are not shared */
	printk(KERN_DEBUG "request_irq()\n");
	rc = request_irq(pdev->irq, lancero_isr,
		lro->msi_enabled? 0: IRQF_SHARED, DRV_NAME, (void *)lro);
	if (rc) {
		printk(KERN_DEBUG "Could not request IRQ #%d; rc = %d.\n",
			pdev->irq, rc);
		lro->irq_line = -1;
		goto err_irq;
	}
	/* remember the allocated irq */
	lro->irq_line = (int)pdev->irq;
	printk(KERN_DEBUG "Succesfully requested IRQ #%d with dev_id 0x%p\n",
		lro->irq_line, lro);
#if 0
#  ifdef CONFIG_MTD_COMPLEX_MAPPINGS
	if (flash_addr != -1) {
		printk(KERN_DEBUG "Probing the Flash ROM.\n");
		rc = probe_flash(lro, flash_addr);
	}
#  endif
#endif
#if LANCERO_GPL
	lro->lancero_class = class_create(THIS_MODULE, DRV_NAME);
	if (IS_ERR(lro->lancero_class)) {
		printk(KERN_DEBUG DRV_NAME ": failed to create class");
		// goto err_class;
	}
#endif
#if 1
	/* initialize user character device */
	lro->user_char_dev = create_sg_char(lro, 0/*bar*/, NULL, NULL, CHAR_USER);
	if (!lro->user_char_dev) {
		printk(KERN_DEBUG "create_char(user_char_dev) failed\n");
		goto err_user_cdev;
	}
#endif
#if 1
	/* initialize control character device */
	lro->ctrl_char_dev = create_sg_char(lro, 1/*bar*/, NULL, NULL, CHAR_CTRL);
	if (!lro->ctrl_char_dev) {
		printk(KERN_DEBUG "create_char(ctrl_char_dev) failed\n");
		goto err_ctrl_cdev;
	}
#endif
#if 1
	/* initialize wait queue for events */
	init_waitqueue_head(&lro->events_wq);
	/* initialize events character device */
	lro->events_char_dev = create_sg_char(lro, -1/*bar*/, NULL, NULL, CHAR_EVENTS);
	if (!lro->events_char_dev) {
		printk(KERN_DEBUG "create_char(events_char_dev) failed\n");
		goto err_events_cdev;
	}
#endif
#if 1
	rc = enable_interrupts(lro, LANCERO_OFS_INT_CTRL, 0x00ffffffUL);
#endif
	/* system indicates write engine */
	if (lro->capabilities & CAP_ENGINE_WRITE) {
		/* allocate and initialize write engine */
		lro->engine[0] = create_engine(lro, LANCERO_OFS_DMA_WRITE, 0/*to_dev*/);
		if (!lro->engine[0])
			goto err_engine;
		lro->engine[0]->name = "write";
		lro->engine[0]->version = engine_version(lro, LANCERO_OFS_DMA_WRITE);
		if (lro->engine[0]->version >= 4) lro->engine[0]->max_extra_adj = MAX_EXTRA_ADJ;
		printk(KERN_DEBUG "%s engine at 0x%p with max_extra_adj = %d\n", lro->engine[0]->name, lro->engine[0], lro->engine[0]->max_extra_adj);
		engine_reset(lro->engine[0]);
	}
	/* system indicates read engine */
	if (lro->capabilities & CAP_ENGINE_READ) {
		/* allocate and initialize read engine */
		lro->engine[1] = create_engine(lro, LANCERO_OFS_DMA_READ, 1/*to_dev*/);
		if (!lro->engine[1])
			goto err_engine;
		lro->engine[1]->name = "read";
		lro->engine[1]->version = engine_version(lro, LANCERO_OFS_DMA_READ);
		if (lro->engine[1]->version >= 4) lro->engine[1]->max_extra_adj = MAX_EXTRA_ADJ;
		printk(KERN_DEBUG "%s engine at 0x%p with max_extra_adj = %d\n", lro->engine[1]->name, lro->engine[1], lro->engine[1]->max_extra_adj);
		engine_reset(lro->engine[1]);
	}
	/* any engines? */
	if (lro->capabilities & (CAP_ENGINE_WRITE | CAP_ENGINE_READ)) {
		/* initialize SG DMA character device */
		lro->sgdma_char_dev = create_sg_char(lro, -1/*bar*/, lro->engine[1], lro->engine[0], CHAR_SGDMA);
		if (!lro->sgdma_char_dev) {
			printk(KERN_DEBUG "create_char(sgdma_char_dev) failed\n");
			goto err_sgdma_cdev;
		}
	}
#if LANCERO_FRAMEBUFFER
	lancerofb_create(lro);
#endif /*LANCERO_FRAMEBUFFER*/
#if PACKET_LENGTH_TABLE
	printk(KERN_INFO "Lancero built with PACKET_LENGTH_TABLE.\n");
#endif
	if ((performance_dirs >= 0) && (performance_dirs <= 2)) {
		performance_run(lro, performance_dirs);
	}
	rc = 0;
#if 0
	printk(KERN_DEBUG "target_addr=%d\n", (int)target_addr);
	/* perform target bridge test? */
	if (target_addr != -1) {
		printk(KERN_DEBUG "target_addr=%d, starting read/write test.\n", (int)target_addr);
		rc = test_target_bridge(lro, target_addr);
	}
#endif
#if 0
	/* perform descriptor bridge test? */
	if ((desc_num > 0) && (tester_offset != -1)) {
		BUG_ON(desc_offset < -4096);
		BUG_ON(desc_offset > 4096);
		printk(KERN_DEBUG "desc_num=%d, desc_offset=%d.\n", (int)desc_num, (int)desc_offset);
		rc = test_desc_bridge(lro, tester_offset, desc_num, desc_offset, 0);
	}
#endif
	/* perform descriptor bridge self test using a random vector list? */
	if (seed > 0) {
		/* descriptor bridge tester present? */
		if (tester_offset != -1)
			rc = test_desc_vectors(lro, seed);
		else
			rc = test_dma_vectors(lro, seed, test_to_dev);
		rc = 0;
	}

	if ((engine_first >= 0) && (engine_last >= 0)) {
		printk(KERN_DEBUG "Commencing test_dma_performance().\n");
		msleep(500);
		test_dma_performance(lro);
	}

#if PACKET_LENGTH_TABLE
#if 1
	printk(KERN_DEBUG "Performing low latency round trip or GPIO trigger test\n");
	test_latency(lro);
#endif
#endif

	if (rc == 0)
		goto end;

	/* remove SG DMA character device */
	if (lro->sgdma_char_dev)
		destroy_sg_char(lro->sgdma_char_dev);
err_sgdma_cdev:
	/* destroy_engine */
err_engine:
	/* remove events character device */
	if (lro->events_char_dev)
		destroy_sg_char(lro->events_char_dev);
err_events_cdev:
	/* remove control character device */
	if (lro->ctrl_char_dev)
		destroy_sg_char(lro->ctrl_char_dev);
err_ctrl_cdev:
	/* remove user character device */
	if (lro->user_char_dev)
		destroy_sg_char(lro->user_char_dev);
err_user_cdev:
#if LANCERO_GPL
	/* remove lancero sysfs class */
	if (lro->lancero_class)
		class_destroy(lro->lancero_class);
#endif
//err_class:
	/* unmap the BARs */
	unmap_bars(lro, pdev);
err_irq:
	if (lro->msi_enabled)
		pci_disable_msi(pdev);
err_mask:
	/* disable the device only if it was not in use */
	if (!lro->regions_in_use)
		pci_disable_device(pdev);
	/* free allocated irq */
	if (lro->irq_line > -1)
		free_irq(lro->irq_line, (void *)lro);
	/* disable message signaled interrupts */
	if (lro->msi_enabled)
    	pci_disable_msi(pdev);
err_system:
	/* unmap the BARs */
	unmap_bars(lro, pdev);
err_map:
	if (lro->got_regions)
		pci_release_regions(pdev);
err_regions:
err_rev:
err_msi:
/* clean up everything before device enable() */
err_enable:
	kfree(lro);
err_alloc:
end:
	return rc;
}

static void __devexit remove(struct pci_dev *pdev)
{
	struct lancero_dev *lro;
	printk(KERN_DEBUG "remove(0x%p)\n", pdev);
	if ((pdev == 0) || (dev_get_drvdata(&pdev->dev) == 0)) {
		printk(KERN_DEBUG
			"remove(dev = 0x%p) pdev->dev.driver_data = 0x%p\n",
			pdev, dev_get_drvdata(&pdev->dev));
		return;
	}
	lro = (struct lancero_dev *)dev_get_drvdata(&pdev->dev);
	printk(KERN_DEBUG
		"remove(dev = 0x%p) where pdev->dev.driver_data = 0x%p\n",
		pdev, lro);
	if (lro->pci_dev != pdev) {
		printk(KERN_DEBUG
		"pdev->dev.driver_data->pci_dev (0x%08lx) != pdev (0x%08lx)\n",
		(unsigned long)lro->pci_dev, (unsigned long)pdev);
	}
 	/* disable all interrupts */
	interrupts_disable(lro, LANCERO_OFS_INT_CTRL, 0x00ffffffUL);
	if (lro->engine[0]) {
		/* reset SGDMA write engine */
		engine_reset(lro->engine[0]);
	}
	if (lro->engine[1]) {
		/* reset SGDMA read engine */
		engine_reset(lro->engine[1]);
	}
#if LANCERO_FRAMEBUFFER
	/* stop framebuffer logic */
	lancerofb_stop(lro, LANCERO_OFS_FRAMEBUFFER);
	/* make sure engines have stopped */
	msleep(100);
	/* remove frame buffer device */
	lancerofb_destroy(lro);
#endif
	/* remove SG DMA character device */
	if (lro->sgdma_char_dev) {
		destroy_sg_char(lro->sgdma_char_dev);
		lro->sgdma_char_dev = 0;
	}
	/* remove events character device */
	if (lro->events_char_dev) {
		destroy_sg_char(lro->events_char_dev);
		lro->events_char_dev = 0;
	}
	/* remove control character device */
	if (lro->ctrl_char_dev) {
		destroy_sg_char(lro->ctrl_char_dev);
		lro->ctrl_char_dev = 0;
	}
	/* remove user character device */
	if (lro->user_char_dev) {
		destroy_sg_char(lro->user_char_dev);
		lro->user_char_dev = 0;
	}
#if LANCERO_GPL
	if (lro->lancero_class)
		class_destroy(lro->lancero_class);
#endif
	/* free IRQ */
	if (lro->irq_line >= 0) {
		printk(KERN_DEBUG "Freeing IRQ #%d for dev_id 0x%08lx.\n",
			lro->irq_line, (unsigned long)lro);
		free_irq(lro->irq_line, (void *)lro);
	}
	/* MSI was enabled? */
	if (lro->msi_enabled) {
		/* Disable MSI @see Documentation/MSI-HOWTO.txt */
		printk(KERN_DEBUG "Disabling MSI interrupting.\n");
		pci_disable_msi(pdev);
		lro->msi_enabled = 0;
	}
	/* unmap the BARs */
	unmap_bars(lro, pdev);
	printk(KERN_DEBUG "Unmapping BARs.\n");
	if (!lro->regions_in_use) {
		printk(KERN_DEBUG "Disabling device.\n");
		pci_disable_device(pdev);
	}
	if (lro->got_regions)
		/* to be called after pci_disable_device()! */
		pci_release_regions(pdev);
}

#if 0
static void mm_open(struct vm_area_struct *vma)
{
	printk(KERN_DEBUG "mm_open()\n");
}
static void mm_close(struct vm_area_struct *vma)
{
	printk(KERN_DEBUG "mm_close()\n");
}

static struct vm_operations_struct vm_ops = {
	.open = mm_open,
	.close = mm_close,
};
#endif

static int bridge_mmap(struct file *file, struct vm_area_struct *vma)
{
	int rc;
	struct lancero_dev *lro;
	struct lancero_char *lro_char = (struct lancero_char *)file->private_data;
	unsigned long off;
	unsigned long phys;
	unsigned long vsize;
	unsigned long psize;
	BUG_ON(!lro_char);
	BUG_ON(lro_char->magic != MAGIC_CHAR);
	lro = lro_char->lro;
	BUG_ON(!lro);
	BUG_ON(lro->magic != MAGIC_DEVICE);

	off = vma->vm_pgoff << PAGE_SHIFT;
	/* BAR physical address */
	phys = pci_resource_start(lro->pci_dev, lro_char->bar) + off;
	vsize = vma->vm_end - vma->vm_start;
	/* complete resource */
	psize = pci_resource_end(lro->pci_dev, lro_char->bar) - pci_resource_start(lro->pci_dev, lro_char->bar) + 1 - off;

	printk(KERN_DEBUG "mmap(): lro_char = 0x%08lx\n", (unsigned long)lro_char);
	printk(KERN_DEBUG "mmap(): lro_char->bar = %d\n", lro_char->bar);
	printk(KERN_DEBUG "mmap(): lro = 0x%p\n", lro);
	printk(KERN_DEBUG "mmap(): pci_dev = 0x%08lx\n", (unsigned long)lro->pci_dev);

	printk(KERN_DEBUG "off = 0x%lx\n", off);
	printk(KERN_DEBUG "start = 0x%llx\n", (unsigned long long)pci_resource_start(lro->pci_dev, lro_char->bar));
	printk(KERN_DEBUG "phys = 0x%lx\n", phys);

	if (vsize > psize)
		return -EINVAL;
	/* pages must not be cached as this would result in cache line sized
	   accesses to the end point */
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	/* prevent touching the pages (byte access) for swap-in,
	   and prevent the pages from being swapped out */
#ifndef VM_RESERVED
	vma->vm_flags |= VM_IO | VM_DONTEXPAND | VM_DONTDUMP;
#else
	vma->vm_flags |= VM_IO | VM_RESERVED;
#endif
	/* make MMIO accessible to user space */
	rc = io_remap_pfn_range(vma, vma->vm_start, phys >> PAGE_SHIFT,
		vsize, vma->vm_page_prot);
	printk(KERN_DEBUG "io_remap_pfn_range(vma=0x%p, vma->vm_start=0x%lx, phys=0x%lx, size=%lu) = %d\n",
		vma, vma->vm_start, phys >> PAGE_SHIFT, vsize, rc);
	if (rc)
		return -EAGAIN;
	//vma->vm_ops = &vm_ops;
	return 0;
}

static ssize_t char_ctrl_read(struct file *file, char __user *buf,
	size_t count, loff_t *pos)
{
	u32 w;
	void *reg;
	struct lancero_dev *lro;
	struct lancero_char *lro_char = (struct lancero_char *)file->private_data;

	BUG_ON(!lro_char);
	BUG_ON(lro_char->magic != MAGIC_CHAR);
	lro = lro_char->lro;
	BUG_ON(!lro);
	BUG_ON(lro->magic != MAGIC_DEVICE);

	/* only 32-bit aligned and 32-bit multiples */
#if 0
	if (count & 3) return -EPROTO;
#endif
	if (*pos & 3) return -EPROTO;
	/* first address is BAR base plus file position offset */
	reg = lro->bar[lro_char->bar] + *pos;
	w = read_register(reg);
	printk(KERN_DEBUG "char_ctrl_read(@%p, count=%d, pos=%d) value = 0x%08x\n", reg, (int)count, (int)*pos, w);
	copy_to_user(buf, &w, 4);
	*pos += 4;
	return 4;
}

static ssize_t char_ctrl_write(struct file *file, const char __user *buf,
	size_t count, loff_t *pos)
{
	u32 w;
	void *reg;
	struct lancero_dev *lro;
	struct lancero_char *lro_char = (struct lancero_char *)file->private_data;

	BUG_ON(!lro_char);
	BUG_ON(lro_char->magic != MAGIC_CHAR);
	lro = lro_char->lro;
	BUG_ON(!lro);
	BUG_ON(lro->magic != MAGIC_DEVICE);

	/* only 32-bit aligned and 32-bit multiples */
#if 0
	if (count & 3) return -EPROTO;
#endif
	if (*pos & 3) return -EPROTO;
	/* first address is BAR base plus file position offset */
	reg = lro->bar[lro_char->bar] + *pos;
	copy_from_user(&w, buf, 4);
	printk(KERN_DEBUG "char_ctrl_write(0x%08x @%p, count=%d, pos=%d)\n", w, reg, (int)count, (int)*pos);
	write_register(w, reg);
	*pos += 4;
	return 4;
}

static ssize_t char_llseek(struct file *file, loff_t *offset, int whence)
{
	loff_t newpos = 0;
	if (whence == 0)
		newpos = *offset;
	file->f_pos = newpos;
	printk(KERN_DEBUG "llseek: pos=%lld\n", (signed long long)newpos);
	return newpos;
}

/*
 * character device file operations for events
 */
static ssize_t char_events_read(struct file *file, char __user *buf,
	size_t count, loff_t *pos)
{
	int rc;
	struct lancero_dev *lro;
	struct lancero_char *lro_char = (struct lancero_char *)file->private_data;

	BUG_ON(!lro_char);
	BUG_ON(lro_char->magic != MAGIC_CHAR);
	lro = lro_char->lro;
	BUG_ON(!lro);
	BUG_ON(lro->magic != MAGIC_DEVICE);

	if (count != 4) return -EPROTO;
	if (*pos & 3) return -EPROTO;

	/* the function servicing the engine will wake us */
	rc = wait_event_interruptible(lro->events_wq, lro->events_irq != 0);
	if (rc) printk(KERN_INFO "wait_event_interruptible() = %d\n", rc);
	/* wait_event_interruptible() was interrupted by a signal */
	if (rc == -ERESTARTSYS) {
		return -ERESTARTSYS;
	}
	//printk(KERN_INFO "wait_event_interruptible() = %d, events_irq = 0x%08lx\n", rc, lro->events_irq);
	copy_to_user(buf, &lro->events_irq, 4);
	/* should be atomic with the above */
	lro->events_irq = 0;
	return 4;
}


/*
 * character device file operations for SG DMA engine
 */
static struct file_operations sg_fops = {
	.owner = THIS_MODULE,
	.open = char_open,
	.release = char_close,
	.read = char_sgdma_read,
	.write = char_sgdma_write,
	//.llseek = char_llseek,
#if 1
	/* asynchronous */
        .read_iter = sg_read_iter,
        .write_iter = sg_write_iter,
#elif 0 /* @TODO make Linux kernel version dependent */
	.aio_read = sg_aio_read,
	.aio_write = sg_aio_write,
#endif
};

/*
 * character device file operations for control bus (through control bridge)
 */
static struct file_operations ctrl_fops = {
	.owner = THIS_MODULE,
	.open = char_open,
	.release = char_close,
	.read = char_ctrl_read,
	.write = char_ctrl_write,
#if 0
	.llseek = char_llseek,
#endif
#if 1
	.mmap = bridge_mmap,
#endif
};

/*
 * character device file operations for the irq events
 */
static struct file_operations events_fops = {
	.owner = THIS_MODULE,
	.open = char_open,
	.release = char_close,
	.read = char_events_read,
};


static int destroy_sg_char(struct lancero_char *lro_char)
{
	BUG_ON(!lro_char);
	BUG_ON(lro_char->magic != MAGIC_CHAR);
	BUG_ON(!lro_char->lro);
#if LANCERO_GPL
	BUG_ON(!lro_char->lro->lancero_class);
	BUG_ON(!lro_char->sys_device);
	if (lro_char->sys_device)
		device_destroy(lro_char->lro->lancero_class, lro_char->cdevno);
#endif
	cdev_del(&lro_char->cdev);
	unregister_chrdev_region(lro_char->cdevno, 1/*count*/);
	kfree(lro_char);
	return 0;
}

#define LANCERO_MINOR_BASE (0)

/* create_char() -- create a character device interface to data or control bus
 *
 * If at least one SG DMA engine is specified, the character device interface
 * is coupled to the SG DMA file operations which operate on the data bus. If
 * no engines are specified, the interface is coupled with the control bus.
 */
static struct lancero_char *create_sg_char(struct lancero_dev *lro, int bar,
	struct lancero_engine *read_engine, struct lancero_engine *write_engine, int type)
{
	struct lancero_char *lro_char;
	int rc;
	static const char *names[4] = { "lancero_user", "lancero_control", "lancero_irq", "lancero_sgdma" };

	printk(KERN_DEBUG DRV_NAME " create_sg_char(lro = 0x%p, read_engine = 0x%p, write_engine = 0x%p)\n",
		lro, read_engine, write_engine);
	/* at least one engine must be specified */
	//BUG_ON(!read_engine && !write_engine);
	/* allocate book keeping data structure */
	lro_char = kzalloc(sizeof(struct lancero_char), GFP_KERNEL);
	if (!lro_char)
		return NULL;
	lro_char->magic = MAGIC_CHAR;
	if (major == 0) {
		/* allocate a dynamically allocated character device node */
		rc = alloc_chrdev_region(&lro_char->cdevno, LANCERO_MINOR_BASE,
			1, DRV_NAME);
	} else {
		lro_char->cdevno = MKDEV(major, type);
		rc = register_chrdev_region(lro_char->cdevno, 1, names[type]);
	}

	if (rc < 0) {
		printk(KERN_DEBUG "alloc/register_chrdev_region() = %d\n", rc);
		goto fail_alloc;
	}
	/* are we dealing with a SG DMA character device interface? */
	if (type == CHAR_SGDMA) {
		/* couple the SG DMA device file operations to the character device */
		cdev_init(&lro_char->cdev, &sg_fops);
		/* @todo */
		kobject_set_name(&lro_char->cdev.kobj, DRV_NAME "_sgdma");
		printk(KERN_DEBUG DRV_NAME "_sgdma = %d:%d\n",
			MAJOR(lro_char->cdevno), MINOR(lro_char->cdevno));
		/* remember engines */
		lro_char->read_engine = read_engine;
		lro_char->write_engine = write_engine;
	/* user character device interface? */
	} else if (type == CHAR_USER) {
		BUG_ON(read_engine || write_engine);
		/* remember BAR we are attached to */
		lro_char->bar = bar;
		/* couple the control device file operations to the character device */
		cdev_init(&lro_char->cdev, &ctrl_fops);
		kobject_set_name(&lro_char->cdev.kobj, DRV_NAME "_user");
		printk(KERN_DEBUG DRV_NAME "_user = %d:%d\n",
			MAJOR(lro_char->cdevno), MINOR(lro_char->cdevno));
	/* control character device interface */
	} else if (type == CHAR_CTRL) {
		BUG_ON(read_engine || write_engine);
		/* remember BAR we are attached to */
		lro_char->bar = bar;
		/* couple the control device file operations to the character device */
		cdev_init(&lro_char->cdev, &ctrl_fops);
		kobject_set_name(&lro_char->cdev.kobj, DRV_NAME "_control");
		printk(KERN_DEBUG DRV_NAME "_control = %d:%d\n",
			MAJOR(lro_char->cdevno), MINOR(lro_char->cdevno));
	} else if (type == CHAR_EVENTS) {
		BUG_ON(read_engine || write_engine);
		/* couple the events device file operations to the character device */
		cdev_init(&lro_char->cdev, &events_fops);
		kobject_set_name(&lro_char->cdev.kobj, DRV_NAME "_events");
		printk(KERN_DEBUG DRV_NAME "_events = %d:%d\n",
			MAJOR(lro_char->cdevno), MINOR(lro_char->cdevno));

	}
	lro_char->cdev.owner = THIS_MODULE;

	/* remember parent */
	lro_char->lro = lro;

	/* bring character device live */
	rc = cdev_add(&lro_char->cdev, lro_char->cdevno, 1/*count*/);
	if (rc < 0) {
		printk(KERN_DEBUG "cdev_add() = %d\n", rc);
		goto fail_add;
	}
#if LANCERO_GPL
	/* create device on our class */
	if (lro->lancero_class) {
		/* this must match the enumeration of CHAR_ */
		const char *name[4] = { "user", "control", "irq", "sgdma" };
		lro_char->sys_device = device_create(lro->lancero_class,
			&lro->pci_dev->dev, lro_char->cdevno, NULL,
			"%s", name[type]);
		if (!lro_char->sys_device) {
			goto fail_device;
		}
	}
#endif

	goto success;
#if LANCERO_GPL
fail_device:
#endif
	cdev_del(&lro_char->cdev);
fail_add:
	unregister_chrdev_region(lro_char->cdevno, 1/*count*/);
fail_alloc:
	kfree(lro_char);
success:
	return lro_char;
}

static struct pci_driver pci_driver = {
	.name = DRV_NAME,
	.id_table = pci_ids,
	.probe = probe,
	.remove = remove,
	/* resume, suspend are optional */
};

static int __init lancero_init(void)
{
	int rc = 0;
	printk(KERN_INFO DRV_NAME " init()\n");
#if 0
	printk(KERN_INFO DRV_NAME " built " __DATE__ " " __TIME__ "\n");
#endif
	rc = pci_register_driver(&pci_driver);
	return rc;
}

static void __exit lancero_exit(void)
{
	printk(KERN_INFO DRV_NAME" exit()\n");
	/* unregister this driver from the PCI bus driver */
	pci_unregister_driver(&pci_driver);
}

module_init(lancero_init);
module_exit(lancero_exit);
