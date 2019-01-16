/*
 * Driver for Lancero SGDMA for FPGA logic
 *
 * Copyright (C) 2007-2014 Sidebranch
 *
 * Leon Woestenberg <leon@sidebranch.com>
 *
 */

#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mm_types.h>

#include <linux/pci.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/uio.h>
#include <linux/workqueue.h>

#include <linux/aio.h>
#include <linux/splice.h>

#include "lancero-user.h"

/* from include/linux/dma-mapping.h */
#ifndef DMA_BIT_MASK
#  define DMA_BIT_MASK(n) (((n) == 64) ? ~0ULL : ((1ULL<<(n))-1))
#endif
#ifndef __devinit
#  define __devinit
#endif
#ifndef __devexit
#  define __devexit
#endif

/* compile-time options */
#define CHAIN_MULTIPLE_TRANSFERS 1

MODULE_LICENSE("Copyright (C) 2009-2014 Sidebranch");

#define DRV_NAME "lancero"
#define LANCERO_KNOWN_REVISION (0x01)
#define LANCERO_BAR_NUM (2)

#define LANCERO_BAR_SGDMA (1)

#define USER_BAR (0)

#define LANCERO_BAR (1)
#define LANCERO_BAR_SIZE (0x1000UL)

#define LANCERO_RINGBUFFER 0
#define FORCE_IR_DESC_COMPLETED 0

#define MAX_EXTRA_ADJ (15)

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

/* interrupts of Scatter-Gather internal components */
#define LANCERO_INT_DMA_WRITE		(1UL << 16)
#define LANCERO_INT_DMA_READ		(1UL << 18)
#define LANCERO_INT_DMA_WRITE_PERF	(1UL << 17)
#define LANCERO_INT_DMA_READ_PERF	(1UL << 19)
/* interrupts of Scatter-Gather reference design only */
#define LANCERO_INT_DMA_READ_TESTER	(1UL << 0)

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

static const unsigned int bridge_bar = LANCERO_BAR_SGDMA;

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
	{ 16 * 1024, LANCERO_BAR_SIZE }; //, 0, 0, 0, 0 };

/* maximum number of bytes per transfer request */
#define LANCERO_TRANSFER_MAX_BYTES (2048 * 4096)
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

const static char *version="lancero-base.c $Ver$";

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
	/* bus address (lowest 32-bit of address) to first descriptor in Root Complex memory */
	u32 first_desc; /* 0x0c */
	/* number of adjacent descriptors at first_desc */
	u32 first_desc_adjacent; /* 0x10 */
	/* number of completed descriptors */
	u32 completed_desc_count; /* 0x14 */
	u32 completed_desc_bytes; /* 0x18 */
	/* bus address (highest 32-bit of address) to first descriptor in Root Complex memory */
	u32 first_desc_hi; /* 0x1c */
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
	/* whether the service routine must free the transfer */
	int dealloc_in_service;
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

	/* protects concurrent access from interrupt context */
	spinlock_t lock;
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
	/* name of this engine */
	char *name;
	/* user space scatter gather mapper */
	struct sg_mapping_t *sgm;
	/* total number of descriptors of completed transfers in this run */
	int desc_dequeued;
	/* engine service work */
	struct work_struct work;
	/* the single bit mask representing the engine interrupt */
	u32 irq;
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
	/* protects concurrent access from interrupt context */
	spinlock_t irq_lock;
	/* shadow register for interrupt enable, spinlock protected */
	u32 irq_enabled_shadow;
};

struct lancero_target_bridge {
	/* 0xBBBBBBBB */
	unsigned long magic;
};

/* prototypes */
static struct lancero_transfer *create_transfer_kernel(struct lancero_dev *lro, const char *start, size_t count, u32 ep_addr, int dir_to_dev);
static void free_transfer(struct lancero_dev *lro, struct lancero_transfer *transfer);
static struct lancero_desc *lancero_desc_alloc(struct pci_dev *dev,
  int number, dma_addr_t *desc_bus_p, struct lancero_desc **desc_last_p);
static void lancero_desc_link(struct lancero_desc *first, struct lancero_desc *second,
	dma_addr_t second_bus);
static void lancero_desc_set(struct lancero_desc *desc,
  dma_addr_t rc_bus_addr, u32 ep_addr, int len);
static void lancero_desc_control(struct lancero_desc *first, u32 control_mask);
static int queue_transfer(struct lancero_engine *engine,
	struct lancero_transfer *transfer);
static void lancero_transfer_cyclic(struct lancero_transfer *transfer);
static struct lancero_char *create_sg_char(struct lancero_dev *lro, int bar,
	struct lancero_engine *read_engine, struct lancero_engine *write_engine, int type);
static int destroy_sg_char(struct lancero_char *lro_char);
static void dump_transfer(struct lancero_transfer *transfer);
static u32 lancero_read_status(struct lancero_engine *engine, int clear);

static int interrupts_enable(struct lancero_dev *lro, int interrupts_offset, uint32_t ints);
static int interrupts_disable(struct lancero_dev *lro, int interrupts_offset, uint32_t ints);

/* performance_start -- start the performance measurement module
 */
static int performance_start(struct lancero_dev *lro, int offset, u32 period_msecs)
{
	int rc = 0;
	u32 w;
	/* 125 MHz clock has 8 ns per cycle, 125000 cycles per millisecond */
	u64 period = period_msecs * 125000;
	struct performance_regs *reg = (struct performance_regs *)(lro->bar[bridge_bar] + offset);
	w = ioread32(&reg->identifier) & 0x00ffff00UL;
	if ((w != 0x00490000) && (w != 0x00d30000) && (w != 0x00d20000)) {
		printk(KERN_DEBUG "Performance identifier not found (found 0x%08x expected 0x00d20000 / 0x00d30000).\n", w);
		rc = -1;
		goto fail_identifier;
	}

	w = 0;
	iowrite32(w, &reg->control);
	w = period & 0xffffffffUL;
	iowrite32(w, &reg->period_low);
	w = period >> 32;
	iowrite32(w, &reg->period_high);
	/* ensure previous writes are before the next */
	wmb();
	/* dummy write-flushing read */
	w = ioread32(&reg->identifier);

	w = PERF_CTRL_RUN | PERF_CTRL_IE;
	iowrite32(w, &reg->control);
	/* dummy write-flushing read */
	w = ioread32(&reg->identifier);
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
	lancero_desc_control(transfer->desc_virt, 0);
	/* create a linked loop */
	lancero_desc_link(transfer->desc_virt, transfer->desc_virt, transfer->desc_bus);
	transfer->cyclic = 1;
	/* dump transfer for debugging */
	//dump_transfer(transfer);
	/* initialize wait queue */
	init_waitqueue_head(&transfer->wq);
#if 0
	/* make transfer cyclic */
	lancero_transfer_cyclic(transfer);
#endif
	/* */
	printk(KERN_DEBUG "Queueing SGDMA I/O %s request for %d seconds performance measurement.\n", dir_to_dev? "write (to device)" : "read (from device)", performance_interval);
	queue_transfer(lro->engine[dir_to_dev], transfer);
}

static int performance_run(struct lancero_dev *lro, int dir_to_dev)
{
  if (dir_to_dev < 2) {
    /* start gated performance measurement */
    performance_start(lro, dir_to_dev?LANCERO_OFS_DMA_READ_PERF:LANCERO_OFS_DMA_WRITE_PERF, performance_interval * 1000);
    /* submit work on selected engine */
    performance_submit(lro, dir_to_dev);
    //lancero_read_status(lro->engine[dir_to_dev], 0);
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
	u64 performance, period, bandwidth;
	struct performance_regs *reg = (struct performance_regs *)(lro->bar[bridge_bar] + offset);

	w = ioread32(&reg->identifier) & 0x00ffff00UL;
	if ((w != 0x00490000) && (w != 0x00d30000) && (w != 0x00d20000)) {
		printk(KERN_DEBUG "Performance identifier not found (found 0x%08x expected 0x00d20000 / 0x00d30000).\n", w);
		rc = -1;
		goto fail_identifier;
	}
	w = ioread32(&reg->performance_high);
	performance = (u64)w;
	performance <<= 32;
	w = ioread32(&reg->performance_low);
	performance |= (u64)w;
	//printk(KERN_DEBUG "Performance counter = %llu\n", (unsigned long long)performance);

	w = ioread32(&reg->period_high);
	period = (u64)w;
	period <<= 32;

	w = ioread32(&reg->period_low);
	period |= (u64)w;

	bandwidth = performance;
	do_div(bandwidth, performance_interval);
	printk(KERN_DEBUG "SGDMA performance %s device = %llu bytes in %u seconds = %llu bytes/second.\n",
		(offset == LANCERO_OFS_DMA_READ_PERF)? "to" : "from",
		(unsigned long long)performance, performance_interval,
		(unsigned long long)bandwidth);

	ioread32(&reg->status);
	/* de-assert RUN and IE */
	w = 0;
	iowrite32(w, &reg->control);
	ioread32(&reg->status);
	/* write IRQ bit in status register to clear interrupt */
	w = 0x2UL;
	iowrite32(w, &reg->status);
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
	u32 value, w;
	BUG_ON(!engine);
	w = ioread32(&engine->regs->identifier) & 0x00ffff00;
	if ((w != 0x004b0000UL) && (w != 0x00c10000UL) && (w != 0x00c20000UL)) {
		printk(KERN_ERR "SGDMA controller identifier not found (found 0x%08x expected 0x00c10000/0x00c20000).\n", w);
		value = 0xffffffff;
		goto fail_identifier;
	}
	/* read status register */
	value = engine->status = ioread32(&engine->regs->status);

	/* clear hardware status register */
	if (clear) {
		w = LANCERO_STAT_BUSY |
			LANCERO_STAT_DESCRIPTOR_COMPLETED |
			LANCERO_STAT_DESCRIPTOR_STOPPED |
			LANCERO_STAT_MAGIC_STOPPED |
			LANCERO_STAT_FETCH_STOPPED |
			LANCERO_STAT_IDLE_STOPPED |
			LANCERO_STAT_NONALIGNED_STOPPED;
		/* clear status register bits */
		iowrite32(w, &engine->regs->status);
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
	/* reset RUN_STOP bit */
	w = ioread32(&engine->regs->control);
	w &= ~LANCERO_CTRL_RUN_STOP;
	w |= LANCERO_CTRL_IE_IDLE_STOPPED;
	iowrite32(w, &engine->regs->control);
	/* dummy read of status register to flush all previous writes */
	ioread32(&engine->regs->status);
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
	/* set RST bit */
	w = LANCERO_CTRL_RST;
	iowrite32(w, &engine->regs->control);
        /* flush */
	ioread32(&engine->regs->status);
	w = 0;
	iowrite32(w, &engine->regs->control);
        /* read and clear engine status to remove pending interrupts */
	lancero_read_status(engine, 1);
	/* transfers on queue? */
	if (!list_empty(&engine->transfer_list)) {
		//printk(KERN_DEBUG "Removing transfer queue from this engine.\n");
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
				free_transfer(engine->lro, transfer);
				transfer = NULL;
				/* indicate I/O completion XXX res, res2 */
				aio_complete(iocb, done, 0);
		/* synchronous I/O? */
		} else {
			/* awake task on transfer's wait queue */
			wake_up_interruptible(&transfer->wq);
		}
		/* if exists, get the next transfer on the list */
		if (!list_empty(&engine->transfer_list)) {
			transfer = list_entry(engine->transfer_list.next,
				struct lancero_transfer, entry);
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
	//u32 w;

	struct lancero_transfer *transfer = 0;

	/* lock the engine */
	spin_lock(&engine->lock);

    /* transfers on queue? */
	if (!list_empty(&engine->transfer_list)) {
		/* pick first transfer on the queue (was submitted to the engine) */
		transfer = list_entry(engine->transfer_list.next, struct lancero_transfer, entry);
                if (transfer->cyclic) {
#if 0
				printk(KERN_DEBUG "Stopping cyclic transfer on %s engine\n", engine->name);
#endif				
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

	/* XXX make sure bus address in within 4GB address space XXX */
	WARN_ON((transfer->desc_bus >> 16) >> 16);
	/* write bus address of first descriptor in Root Complex memory */
	w = (u32)transfer->desc_bus;

	/* initialize number of descriptors of dequeued transfers */
	engine->desc_dequeued = 0;

	/* write bus address of first descriptor of transfer */
	iowrite32(w, &engine->regs->first_desc);
	iowrite32(0, &engine->regs->first_desc_hi);

	if (transfer->desc_adjacent > 0) {
		extra_adj = transfer->desc_adjacent - 1;
		if (extra_adj > MAX_EXTRA_ADJ)
			extra_adj = MAX_EXTRA_ADJ;
	}
	iowrite32(extra_adj, &engine->regs->first_desc_adjacent);

	/* dummy read of status register to flush all previous writes */
	ioread32(&engine->regs->status);
	wmb();

	/* write control register of SG DMA engine */
	w = (u32)LANCERO_CTRL_RUN_STOP;
	w |= (u32)LANCERO_CTRL_IE_DESCRIPTOR_STOPPED;
	w |= (u32)LANCERO_CTRL_IE_DESCRIPTOR_COMPLETED;
	w |= (u32)LANCERO_CTRL_IE_MAGIC_STOPPED;
	w |= (u32)LANCERO_CTRL_IE_IDLE_STOPPED;
	w |= (u32)LANCERO_CTRL_IE_NONALIGNED_STOPPED;

	/* start the engine */
	iowrite32(w, &engine->regs->control);

	/* dummy read of status register to flush all previous writes */
	ioread32(&engine->regs->status);

	engine->running = 1;
	return transfer;
}

/* engine_initialize -- Initialize the engine for use, read capabilities */
static int engine_initialize(struct lancero_dev *lro, int interrupts_offset)
{
	void *reg = lro->bar[bridge_bar] + interrupts_offset;
	u32 w;
	int rc = 0;
	w = ioread32(reg + 0x00);
	/* not a write nor a read engine found? */
	if (((w & 0x00ffff00UL) != 0x00c10000UL) && ((w & 0x00ffff00UL) != 0x00c20000UL)) {
		rc = -1;
		goto fail_identifier;
	}

	/* before version 2, 64-bit DMA is not available */
	if ((w & 0xffUL) < 2UL) lro->capabilities &= ~(CAP_64BIT_DMA | CAP_64BIT_DESC);
	/* clear all interrupt event enables, stop engine */
	w = 0x0UL;
	iowrite32(w, reg + 0x04);
fail_identifier:
	return rc;
}

/**
 * engine_service() - service an SGDMA engine
 *
 * @engine pointer to struct lancero_engine
 *
 */
static int engine_service(struct lancero_engine *engine)
{
	u32 desc_completed;
	struct lancero_transfer *transfer = 0, *transfer_started, *transfer_next;

	/* lock the engine */
	spin_lock(&engine->lock);

	/* engine was not started by the driver? */
	if (!engine->running) {
		spin_unlock(&engine->lock);
		return 0;
	}

	/* read status register */
	lancero_read_status(engine, 1);

	/* engine was running but is no longer busy? */
	if (engine->running && !(engine->status & LANCERO_STAT_BUSY)) {
		lancero_stop(engine);
		engine->running = 0;
	}

#define LANCERO_STAT (LANCERO_STAT_BUSY | \
LANCERO_STAT_DESCRIPTOR_COMPLETED | \
LANCERO_STAT_DESCRIPTOR_STOPPED | \
LANCERO_STAT_MAGIC_STOPPED | \
LANCERO_STAT_NONALIGNED_STOPPED)

	/* engine event which can forward our work? */
	if (engine->status & (LANCERO_STAT_DESCRIPTOR_COMPLETED |
		LANCERO_STAT_DESCRIPTOR_STOPPED | LANCERO_STAT_MAGIC_STOPPED |
		LANCERO_STAT_IDLE_STOPPED | LANCERO_STAT_NONALIGNED_STOPPED)) {

		/* read number of completed descriptors after engine start */
		desc_completed = ioread32(&engine->regs->completed_desc_count);

		/* transfers on queue? */
		if (!list_empty(&engine->transfer_list)) {
			/* pick first transfer on the queue (was submitted to the engine) */
			transfer = list_entry(engine->transfer_list.next,
				struct lancero_transfer, entry);
		}

		/* account for already dequeued transfers during this engine run */
		desc_completed -= engine->desc_dequeued;

		/* iterate over all the transfers completed by the engine,
		 * except for the last (i.e. use > instead of >=). */
		while (transfer && (!transfer->cyclic) &&
			(desc_completed > transfer->desc_num)) {
			/* remove this transfer from desc_completed */
			desc_completed -= transfer->desc_num;
			/* remove completed transfer from list */
			list_del(engine->transfer_list.next);

			/* remember next transfer */
			if (!list_empty(&engine->transfer_list)) {
				/* remember next transfer on the queue (was submitted to the engine) */
				transfer_next = list_entry(engine->transfer_list.next,
					struct lancero_transfer, entry);
			} else { transfer_next = NULL; }

			/* add to dequeued number of descriptors during this run */
			engine->desc_dequeued += transfer->desc_num;
			/* asynchronous I/O? nobody cares for the transfer, only the result */
			if ((transfer->iocb) && (transfer->last_in_request)) {
				struct kiocb *iocb = transfer->iocb;
				ssize_t done = transfer->size_of_request;
				//printk(KERN_DEBUG "Freeing (async I/O request) last transfer %p, iocb %p\n", transfer, transfer->iocb);
				free_transfer(engine->lro, transfer);
				//printk(KERN_DEBUG "Completing async I/O iocb %p with size %d\n", iocb, (int)done);
				/* indicate I/O completion XXX res, res2 */
				aio_complete(iocb, done, 0);
			/* synchronous I/O? */
			} else {
				/* initiator no longer cares or waits for the transfer */
				if (transfer->dealloc_in_service) {
					free_transfer(engine->lro, transfer);
				    	printk(KERN_DEBUG "transfer->dealloc_in_service; freeing transfer.\n");
				/* normal case, blocked initiator waits for transfer */
				} else {
					/* mark transfer as succesfully completed */
					transfer->state = TRANSFER_STATE_COMPLETED;
					/* awake task on transfer's wait queue */
					wake_up_interruptible(&transfer->wq);
				}
			}
			transfer = transfer_next;
		}
		/* inspect the transfer that is not completed yet */
		if (transfer) {
			/* engine stopped? (i.e. not busy and stop reason known? */
			if (((engine->status & LANCERO_STAT_BUSY) == 0) &&
				(engine->status & (LANCERO_STAT_MAGIC_STOPPED |
				LANCERO_STAT_DESCRIPTOR_STOPPED |
				LANCERO_STAT_IDLE_STOPPED |
				LANCERO_STAT_NONALIGNED_STOPPED))) {
			}

			/* the engine still working on current transfer? */
			if (engine->status & LANCERO_STAT_BUSY) {
			/* engine has stopped  */
			} else {
				/* the engine failed on current transfer? */
				if (engine->status & (LANCERO_STAT_MAGIC_STOPPED | LANCERO_STAT_NONALIGNED_STOPPED)) {
					/* mark transfer as succesfully completed */
					transfer->state = TRANSFER_STATE_FAILED;
					lancero_stop(engine);
				/* the engine stopped on current transfer? */
				} else {
					if (desc_completed < transfer->desc_num) {
						transfer->state = TRANSFER_STATE_FAILED;
					} else {
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
					free_transfer(engine->lro, transfer);
					transfer = NULL;
					/* indicate I/O completion XXX res, res2 */
					aio_complete(iocb, done, 0);
				/* synchronous I/O? */
				} else {
					/* initiator no longer cares or waits for the transfer */
					if (transfer->dealloc_in_service) {
						free_transfer(engine->lro, transfer);
					    	printk(KERN_DEBUG "transfer->dealloc_in_service; freeing transfer.\n");
					/* normal case, blocked initiator waits for transfer */
					} else {
						/* mark transfer as succesfully completed */
						transfer->state = TRANSFER_STATE_COMPLETED;
						/* awake task on transfer's wait queue */
						wake_up_interruptible(&transfer->wq);
					}
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
			}
		}
	/* engine did not complete a transfer */
	} else {
	    	printk(KERN_DEBUG "Spurious interrupt.\n");
	}
	/* enable the interrupt for this engine */
	interrupts_enable(engine->lro, LANCERO_OFS_INT_CTRL, engine->irq);
	/* unlock the engine spinlock*/
	spin_unlock(&engine->lock);
	return 0;
}

/* engine_service_work */
static void engine_service_work(struct work_struct *work)
{
	struct lancero_engine *engine;
	engine = container_of(work, struct lancero_engine, work);
	BUG_ON(engine->magic != MAGIC_ENGINE);
	engine_service(engine);
}

/* read_interrupts -- Print the interrupt controller status */
static u32 read_interrupts(struct lancero_dev *lro, int interrupts_offset)
{
	void *reg = lro->bar[bridge_bar] + interrupts_offset;
	u32 w;
	w = ioread32(reg + 0x08);
	return w;
}

/*
 * lancero_isr() - Interrupt handler
 *
 * @dev_id pointer to lancero_dev
 */
static irqreturn_t lancero_isr(int irq, void *dev_id)
{
	u32 w;
	struct lancero_dev *lro;
	BUG_ON(!dev_id);
	void *reg;
	lro = (struct lancero_dev *)dev_id;
	if (!lro)
		return IRQ_NONE;
	reg = lro->bar[bridge_bar];

#if 0
	iowrite32(0, reg + 0x308);
	ioread32(reg + 0x308);
#endif
	w = read_interrupts(lro, LANCERO_OFS_INT_CTRL);
#if 0
	printk(KERN_DEBUG "IRQ #%d, mask 0x%08x\n", lro->irq_count, w);
#endif

	/* disable all interrupts that fired */
	interrupts_disable(lro, LANCERO_OFS_INT_CTRL, w & (LANCERO_INT_DMA_READ | LANCERO_INT_DMA_WRITE));

	/* new irq events? */
	if ((lro->events_irq | w) != lro->events_irq) {
		/* accumulate events into the pending mask */
		lro->events_irq |= w;
		wake_up_interruptible(&lro->events_wq);
	}
	/* write engine? */
	if (w & LANCERO_INT_DMA_WRITE) {
		schedule_work(&lro->engine[0]->work);
	}
	/* read engine? */
	if (w & LANCERO_INT_DMA_READ) {
		schedule_work(&lro->engine[1]->work);
	}
#if 0
	/* write performance module? */
	if (w & LANCERO_INT_DMA_WRITE_PERF) {
		engine_cyclic_stop(lro->engine[0]);
		performance_read(lro, LANCERO_OFS_DMA_WRITE_PERF);
	}
	/* read performance module? */
	if (w & LANCERO_INT_DMA_READ_PERF) {
		/* make the transfer non-cyclic */
		engine_cyclic_stop(lro->engine[1]);
		performance_read(lro, LANCERO_OFS_DMA_READ_PERF);
	}
#endif
#if 0
	/* read datagenerator module? */
	if (w & LANCERO_INT_DMA_READ_TESTER) {
		printk(KERN_DEBUG "Read tester found data mismatch, disabling interrupt.\n");
		interrupts_disable(lro, LANCERO_OFS_INT_CTRL, LANCERO_INT_DMA_READ_TESTER);
	}
#endif
	lro->irq_count++;
#if 0
	if (lro->irq_count > 10000) {
		printk(KERN_DEBUG "IRQ #%d, mask 0x%08x\n", lro->irq_count, w);
		printk(KERN_DEBUG "irq_count == 10000, stopping all interrupts.\n");
		interrupts_disable(lro, LANCERO_OFS_INT_CTRL, 0xffffffffUL);
	}
#endif
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

	/* create singly-linked list for SG DMA controller */
	for (i = 0; i < number - 1; i++) {
		/* increment bus address to next in array */
		desc_bus += sizeof(struct lancero_desc);
		/* singly-linked list uses bus addresses */
		desc_virt[i].next_lo = cpu_to_le32(pci_dma_l(desc_bus));
		desc_virt[i].next_hi = cpu_to_le32(0);
		desc_virt[i].bytes = cpu_to_le32(0);

		if (adj > 0) {
			extra_adj = adj - 1;
			if (extra_adj > MAX_EXTRA_ADJ)
				extra_adj = MAX_EXTRA_ADJ;
			adj--;
		} else
			extra_adj = 0;
		desc_virt[i].control = cpu_to_le32(0xAD4B0000UL | (extra_adj << 8));
	}
	/* { i = number - 1 } */
	/* zero the last descriptor next pointer */
	desc_virt[i].next_lo = cpu_to_le32(0);
	desc_virt[i].next_hi = cpu_to_le32(0);
	desc_virt[i].bytes = cpu_to_le32(0);
	desc_virt[i].control = cpu_to_le32(0xAD4B0000UL);
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
	/* extra adjacent descriptors after the next descriptor */
	u32 extra_adj = 0;
	/* remember control in first descriptor */
	u32 control = le32_to_cpu(first->control) & 0x000000ffUL;
	/* second descriptor given? */
	if (second) {
		/* get number of extra contiguous descriptors after second descriptor */
		extra_adj = ((le32_to_cpu(second->control) >> 8) & 0xf);
		/* if adjacent, add */
		if (extra_adj > 0) {
			if (++extra_adj > MAX_EXTRA_ADJ)
				extra_adj = MAX_EXTRA_ADJ;
		}
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
	control |= 0xAD4B0000UL | (extra_adj << 8);

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
	/* remember control bits */
	u32 control = le32_to_cpu(desc->control) & 0x000000ffUL;
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

/* lancero_desc_control -- Set control field of a descriptor. */
static void lancero_desc_control(struct lancero_desc *first, u32 control_mask)
{
	/* remember magic and adjacent number */
	u32 control = le32_to_cpu(first->control) & 0xffff0f00UL;
	/* merge adjacent and control field */
	control |= control_mask;
	/* write control and next_adjacent */
	first->control = cpu_to_le32(control);
}

/* lancero_desc_clear -- Set control field of a descriptor. */
static void lancero_desc_clear(struct lancero_desc *first, u32 clear_mask)
{
	/* remember magic and adjacent number */
	u32 control = le32_to_cpu(first->control) & 0xffff0f00UL;
	/* merge adjacent and control field */
	control &= (~clear_mask);
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
#if 0
	static chained = 0;
	static nonchained = 0;
#endif
	int rc = 0;
	struct lancero_transfer *transfer_started;
	BUG_ON(!engine);
	BUG_ON(!transfer);
	BUG_ON(transfer->desc_num == 0);

	/* lock the engine state */
	spin_lock(&engine->lock);

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
		/* get last transfer queued on the engine */
		last = list_entry(engine->transfer_list.prev,
			struct lancero_transfer, entry);
		/* @only when non-cyclic transfer */
		/* link the last transfer's last descriptor to this transfer */
		lancero_desc_link(last->desc_virt + last->desc_num - 1,
			transfer->desc_virt, transfer->desc_bus);
		/* do not stop now that there is a linked transfers */
		lancero_desc_clear(last->desc_virt + last->desc_num - 1, LANCERO_DESC_STOP);
#if 0
		if (nonchained) {
			printk("Last %d transfers could not be chained.\n", nonchained);
			nonchained = 0;
		}
		chained++;
	} else {
		if (chained) {
			printk("Last %d transfers were chained.\n", chained);
			chained = 0;
		}
		nonchained++;
#endif
	}
#endif

	/* mark the transfer as submitted */
	transfer->state = TRANSFER_STATE_SUBMITTED;
	/* add transfer to the tail of the engine transfer queue */
	list_add_tail(&transfer->entry, &engine->transfer_list);

	/* engine is idle? */
	if (!engine->running) {
		/* start engine */
		transfer_started = engine_start(engine);
	}
shutdown:
	/* unlock the engine state */
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
	/* engine interrupt request bit */
	engine->irq = dir_to_dev? LANCERO_INT_DMA_READ: LANCERO_INT_DMA_WRITE;
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
		/* the direction is needed to synchronize caches */
		pci_unmap_sg(lro->pci_dev, transfer->sgm->sgl, transfer->sgm->mapped_pages,
			transfer->dir_to_dev? DMA_TO_DEVICE: DMA_FROM_DEVICE);
		if (transfer->userspace) {
			/* dirty and unlock the pages */
			sgm_put_user_pages(transfer->sgm, transfer->dir_to_dev? 0 : 1);
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

/* interrupts_probe -- Probe the interrupt controller */
static int interrupts_probe(struct lancero_dev *lro, int interrupts_offset)
{
	void *reg = lro->bar[bridge_bar] + interrupts_offset;
	u32 w;
	int rc = 0;
	w = ioread32(reg + 0x00);
	if (((w & 0xffffff00UL) != 0xd2870000UL) &&
	    ((w & 0x00ffff00UL) != 0x00b10000UL)) {
		printk(KERN_DEBUG "Interrupt controller identifier not found (found 0x%08x expected 0x00b100..).\n", w);
		rc = -1;
		goto fail_identifier;
	}
	/* since version 2, an event register is present */
	if ((w & 0xffUL) >= 2UL) lro->capabilities |= CAP_INT_EVENTS;
fail_identifier:
	return rc;
}

/* interrupts_enable -- Enable the interrupts we are interested in */
static int interrupts_enable(struct lancero_dev *lro, int interrupts_offset, u32 ints)
{
	void *reg = lro->bar[bridge_bar] + interrupts_offset;
	u32 w;
	int rc = 0;
	unsigned long flags;
	spin_lock_irqsave(&lro->irq_lock, flags);
	/* update desired interrupt enable mask in shadow register */
	lro->irq_enabled_shadow |= ints;
	/* write interrupt enable mask */
	iowrite32(lro->irq_enabled_shadow, reg + 0x04);
	/* read back (flushes write) */
	w = ioread32(reg + 0x04);
	spin_unlock_irqrestore(&lro->irq_lock, flags);
	/* verify all interrupt were enabled */
	if (w != lro->irq_enabled_shadow) {
		printk(KERN_DEBUG "Could not set interrupt enable register?!\n");
		rc = -1;
	}
	return rc;
}

/* interrupts_disable -- Enable the interrupts we are interested in */
static int interrupts_disable(struct lancero_dev *lro, int interrupts_offset, u32 ints)
{
	void *reg = lro->bar[bridge_bar] + interrupts_offset;
	u32 w;
	int rc = 0;
	unsigned long flags;
	spin_lock_irqsave(&lro->irq_lock, flags);
	/* update desired interrupt enable mask in shadow register */
	lro->irq_enabled_shadow &= ~ints;
	/* write interrupt enable mask */
	iowrite32(lro->irq_enabled_shadow, reg + 0x04);
	/* read back (flushes write) */
	w = ioread32(reg + 0x04);
	spin_unlock_irqrestore(&lro->irq_lock, flags);
	/* verify all interrupt were enabled */
	if (w != lro->irq_enabled_shadow) {
		printk(KERN_DEBUG "Could not set interrupt enable register?!\n");
		rc = -1;
	}
	return rc;
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
	int off = 0x100;
	u32 w, payload, maxread;
	int rc = 0;
	int version = 0;

	/* read identifier of the configuration inspector */
	w = ioread32(reg + 0x00);
	/* read version of the configuration inspector */
	version = w & 0x000000ffUL;
	/* configuration inspector not found? */
	if ((w & 0xffffff00UL) != 0x00b20000UL) {
		printk(KERN_ERR "Configuration Inspector identifier not found (found 0x%08x expected 0x00b20001).\n", w);
		rc = -1;
		goto fail_identifier;
	}

	payload = ioread32(reg + 0x08);
	maxread = ioread32(reg + 0x0c);
	/* read Lancero System identifier */
	w = ioread32(reg + 0x10) & 0x0000ffffUL;
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
			w = ioread32(reg + 0x1c);
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
			w = ioread32(reg + 0x20);
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
	w = ioread32(reg + 0x04);
	/* bus, device and function */
	printk(KERN_DEBUG "bus:dev.fn = %02x:%02x.%1x, payload = %d bytes, maxread = %d bytes\n",
		(w >> 8) & 0x0f/*bus*/, (w >> 3) & 0x1f/* device*/, w & 0x07/*function*/,
		(unsigned int)payload, (unsigned int)maxread);
fail_identifier:
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
	WARN_ON(!transfer);
	if (!transfer) {
		return NULL;
	}
	/* remember direction of transfer */
	transfer->dir_to_dev = dir_to_dev;

	/* create virtual memory mapper */
	transfer->sgm = sg_create_mapper(count);
	WARN_ON(!transfer->sgm);
	if (!transfer->sgm) {
		goto fail_sgm;
	}
	transfer->userspace = 1;

	/* lock user pages in memory and create a scatter gather list */
	rc = sgm_get_user_pages(transfer->sgm, start, count, !dir_to_dev);
	//WARN_ON(rc < 0);
	if (rc < 0) {
		/* XXX */
		goto fail_sgm;
	}
	sgl = transfer->sgm->sgl;

	/* map all SG entries into DMA memory */
	transfer->sgl_nents = pci_map_sg(lro->pci_dev, transfer->sgm->sgl,
		transfer->sgm->mapped_pages, dir_to_dev? DMA_TO_DEVICE: DMA_FROM_DEVICE);
	WARN_ON(!transfer->sgl_nents);
	if (!transfer->sgl_nents)
		goto fail_map;

	/* allocate descriptor list */
	transfer->desc_virt = lancero_desc_alloc(lro->pci_dev,
		transfer->sgl_nents, &transfer->desc_bus, NULL);
	WARN_ON(!transfer->desc_virt);
	if (!transfer->desc_virt)
		goto fail_desc;

	/* start first contiguous block */
	cont_addr = addr = sg_dma_address(&transfer->sgm->sgl[i]);
	cont_len = 0;

	/* iterate over all remaining entries but the last */
	for (i = 0; i < transfer->sgl_nents - 1; i++) {
		/* bus address of next entry i + 1 */
		dma_addr_t next = sg_dma_address(&sgl[i + 1]);
		/* length of this entry i */
		len = sg_dma_len(&sgl[i]);

		/* add entry i to current contiguous block length */
		cont_len += len;

		new_desc = 0;
		/* entry i + 1 is non-contiguous with entry i? */
		if (next != addr + len) {
			new_desc = 1;
		}
		/* entry i reached maximum transfer size? */
		else if (cont_len > (LANCERO_DESC_MAX_BYTES - PAGE_SIZE)) {
			new_desc = 1;
		}
		if (new_desc) {
			/* fill in descriptor entry j with transfer details */
			lancero_desc_set(transfer->desc_virt + j, cont_addr,
				ep_addr, cont_len);
			if (cont_len > cont_max_len) {
				cont_max_len = cont_len;
			}
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

	/* fill in last descriptor entry j with transfer details */
	lancero_desc_set(transfer->desc_virt + j, cont_addr,
		ep_addr, cont_len);
	/* terminate last descriptor */
	lancero_desc_link(transfer->desc_virt + j, 0, 0);
	/* request IRQ on last descriptor */
	lancero_desc_control(transfer->desc_virt + j, LANCERO_DESC_STOP | LANCERO_DESC_COMPLETED);

	j++;
	/* j is the number of descriptors */
	transfer->desc_num = transfer->desc_adjacent = j;

	/* fill in adjacent numbers */
	for (i = 0; i < transfer->desc_num; i++) {
		lancero_desc_adjacent(transfer->desc_virt + i,
			transfer->desc_num - i - 1);
	}

	/* initialize wait queue */
	init_waitqueue_head(&transfer->wq);

	return transfer;
fail_desc:
	pci_unmap_sg(lro->pci_dev, transfer->sgm->sgl,
		transfer->sgm->mapped_pages, dir_to_dev? DMA_TO_DEVICE: DMA_FROM_DEVICE);
fail_map:
	/* free mapper */
	sg_destroy_mapper(transfer->sgm);
fail_sgm:
	/* free transfer */
	kfree(transfer);
	return NULL;
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

	printk(KERN_DEBUG "mapped_pages=%d.\n", transfer->sgm->mapped_pages);
	printk(KERN_DEBUG "sgl = 0x%p.\n", transfer->sgm->sgl);
	BUG_ON(!lro->pci_dev);
	BUG_ON(!transfer->sgm->sgl);
	BUG_ON(!transfer->sgm->mapped_pages);
	/* map all SG entries into DMA memory */
	transfer->sgl_nents = pci_map_sg(lro->pci_dev, transfer->sgm->sgl,
		transfer->sgm->mapped_pages, dir_to_dev? DMA_TO_DEVICE : DMA_FROM_DEVICE);
	printk(KERN_DEBUG "hwnents=%d.\n", transfer->sgl_nents);

	/* verify if the page start address got into the first sg entry */
	printk(KERN_DEBUG "sg_page(&sgl[0])=0x%p.\n", sg_page(&transfer->sgm->sgl[0]));
	printk(KERN_DEBUG "sg_dma_address(&sgl[0])=0x%016llx.\n", (u64)sg_dma_address(&transfer->sgm->sgl[0]));
	printk(KERN_DEBUG "sg_dma_len(&sgl[0])=0x%08x.\n", sg_dma_len(&transfer->sgm->sgl[0]));

	/* allocate descriptor list */
	transfer->desc_virt = lancero_desc_alloc(lro->pci_dev,
		transfer->sgl_nents, &transfer->desc_bus, NULL);
	printk(KERN_DEBUG "create_transfer_user():\n");
	printk(KERN_DEBUG "transfer->desc_bus = 0x%llx.\n", (u64)transfer->desc_bus);

	/* start first contiguous block */
	cont_addr = addr = sg_dma_address(&transfer->sgm->sgl[i]);
	cont_len = 0;

	/* iterate over all remaining entries but the last */
	for (i = 0; i < transfer->sgl_nents - 1; i++) {
		/* bus address of next entry i + 1 */
		dma_addr_t next = sg_dma_address(&sgl[i + 1]);
		/* length of this entry i */
		len = sg_dma_len(&sgl[i]);
		printk(KERN_DEBUG "SGLE %04d: addr=0x%016llx length=0x%08x\n", i, (u64)addr, len);

		/* add entry i to current contiguous block length */
		cont_len += len;

		new_desc = 0;
		/* entry i + 1 is non-contiguous with entry i? */
		if (next != addr + len) {
			printk(KERN_DEBUG "NON CONTIGUOUS\n");
			new_desc = 1;
		}
		/* entry i reached maximum transfer size? */
		else if (cont_len > (LANCERO_DESC_MAX_BYTES - PAGE_SIZE)) {
			printk(KERN_DEBUG "BREAK\n");
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
				printk(KERN_DEBUG "DESC %4d: cont_addr=0x%016llx cont_len=0x%08x, ep_addr=0x%lx\n",
				j, (u64)cont_addr, cont_len, (unsigned long)ep_addr);
			}
			printk(KERN_DEBUG "DESC %4d: cont_addr=0x%016llx cont_len=0x%08x, ep_addr=0x%lx\n",
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

	printk(KERN_DEBUG "SGLE %04d: addr=0x%016llx length=0x%08x\n", i, (u64)addr, len);
	printk(KERN_DEBUG "DESC %4d: cont_addr=0x%016llx cont_len=0x%08x, ep_addr=0x%lx\n",
		j, (u64)cont_addr, cont_len, (unsigned long)ep_addr);

	/* XXX to test error condition, set cont_len = 0 */

	/* fill in last descriptor entry j with transfer details */
	lancero_desc_set(transfer->desc_virt + j, cont_addr,
		ep_addr, cont_len);
#if 0 //LANCERO_PERFORMANCE_TEST
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

	printk(KERN_DEBUG "transfer 0x%p has %d descriptors\n", transfer, transfer->desc_num);
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
	lro = lro_char->lro;

	engine = dir_to_dev? lro_char->read_engine : lro_char->write_engine;
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
		/* anything left to transfer? */
		while (remaining > 0) {
			struct lancero_transfer *transfer;
			/* DMA transfer size, multiple if necessary */
			size_t transfer_len = (remaining > LANCERO_TRANSFER_MAX_BYTES) ?
				LANCERO_TRANSFER_MAX_BYTES : remaining;

			/* build device-specific descriptor tables */
			transfer = create_transfer_user(lro, transfer_addr, transfer_len,
				pos, dir_to_dev);

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
		}
		total_done += done;
	}
	return -EIOCBQUEUED;
}

/**
* sg_aio_read - generic asynchronous read routine
* @iocb:       kernel I/O control block
* @iov:        io vector request
* @nr_segs:    number of segments in the iovec
* @pos:        current file position
*/
ssize_t sg_aio_read(struct kiocb *iocb, const struct iovec *iov,
unsigned long nr_segs, loff_t pos)
{
        return sg_aio_read_write(iocb, iov, nr_segs, pos, 0/*dir_to_dev = 0*/);
}

ssize_t sg_aio_write(struct kiocb *iocb, const struct iovec *iov,
unsigned long nr_segs, loff_t pos)
{
        return sg_aio_read_write(iocb, iov, nr_segs, pos, 1/*dir_to_dev = 1*/);
}

#if 0
static ssize_t char_sgdma_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
}
#endif

static ssize_t char_cyclic_read_write(struct file *file, char __user *buf,
size_t count, loff_t *pos, int dir_to_dev)
{
	char *transfer_addr = (char *)buf;
 	int address_low_bits = 0;
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
	address_low_bits = (int)buf & (lro->align[dir_to_dev] - 1);

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
	//WARN_ON(!transfer);
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
 	int address_low_bits = 0;
	size_t length_low_bits = 0;
	struct lancero_char *lro_char;
	struct lancero_dev *lro;
	struct lancero_engine *engine;

	/* fetch device specific data stored earlier during open */
	lro_char = (struct lancero_char *)file->private_data;
	lro = lro_char->lro;

	engine = dir_to_dev? lro_char->read_engine : lro_char->write_engine;
	address_low_bits = (int)buf & (lro->align[dir_to_dev] - 1);

	/* assert correct base address alignment - lower bits must be zero */
	if (address_low_bits)
	{
		return -EINVAL;
	}
	/* assert size alignment - lower bits must be zero */
	length_low_bits = count & ((size_t)lro->align[dir_to_dev] - 1);
	/* strip the low bits off the count */
	count &= ~(lro->align[dir_to_dev] - 1);
	remaining = count;
	/* low bits are set? */
	if (length_low_bits) {
		return -EINVAL;
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
		//WARN_ON(!transfer);
		if (!transfer) {
			remaining = 0;
			res = -EIO;
			continue;
		}
		/* last transfer for the given request? */
		if (transfer_len >= remaining) {
			transfer->last_in_request = 1;
			transfer->size_of_request = done + transfer_len;
		}
		/* let the device read from the host */
		queue_transfer(engine, transfer);

		/* the function servicing the engine will wake us */
		rc = wait_event_interruptible(transfer->wq, transfer->state != TRANSFER_STATE_SUBMITTED);

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
			/* lock the engine */
			spin_lock(&engine->lock);
			/* engine has completed the transfer? */
			if (transfer->state != TRANSFER_STATE_SUBMITTED) {
				free_transfer(lro, transfer);
			/* engine still working on transfer? */
			} else {
				transfer->dealloc_in_service = 1;
				printk(KERN_DEBUG "-ERESTARTSYS, TRANSFER_STATE_SUBMITTED; ask engine to free the transfer.\n");
			}
			spin_unlock(&engine->lock);
			transfer_len = remaining = 0;
			res = -ERESTARTSYS;
		} else {
			printk(KERN_DEBUG "transfer->state = %d, rc = %d", transfer->state, rc);
		}
		/* calculate the next transfer */
		transfer_addr += transfer_len;
		remaining -= transfer_len;
		done += transfer_len;
		*pos += transfer_len;
	}
	/* return error or else number of bytes */
	res = res ? res : done;
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

#if 0
/* Vectored SGDMA writev
 *
 * struct iovec {
 *   void __user *iov_base;
 *   __kernel_size_t iov_len;	
 * }
*/
static ssize_t char_sgdma_writev(struct file *file, const struct iovec *iov, 
size_t count, loff_t *pos)
{
	/* fetch file this io request acts on */
	struct file *file = iocb->ki_filp;
	loff_t *ppos = &iocb->ki_pos;
	size_t total_done = 0;
	unsigned long seg;

	struct lancero_char *lro_char;
	struct lancero_dev *lro;
	struct lancero_engine *engine;

    struct lancero_transfer *transfer[5];

    int rc;

	/* fetch device specific data stored earlier during open */
	lro_char = (struct lancero_char *)file->private_data;
	lro = lro_char->lro;

	if (nr_segs > 5) {
		printk(KERN_DEBUG "Only up to 5 segments are supported.\n");
		return -EINVAL;
	}

	engine = dir_to_dev? lro_char->read_engine : lro_char->write_engine;
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

		if (remaining > LANCERO_TRANSFER_MAX_BYTES) {
			rc = -EINVAL;
			goto fail_size;
		}

		/* build device-specific descriptor tables */
		transfer[seg] = create_transfer_user(lro, transfer_addr, transfer_len,
			pos, dir_to_dev);
		/* could not allocate transfer? */
		if (!transfer[seg]) {
			printk(KERN_DEBUG "Could not allocate memory for transfer!");
			rc = -ENOMEM;
			goto fail_transfer_alloc;
		}
	}
#if 0
		/* anything left to transfer? */
		while (remaining > 0) {
			/* DMA transfer size, multiple if necessary */
			size_t transfer_len = (remaining > LANCERO_TRANSFER_MAX_BYTES) ?
				LANCERO_TRANSFER_MAX_BYTES : remaining;
#endif
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
		}
		total_done += done;
	}
fail_size:
fail_transfer_alloc:

	return -EIOCBQUEUED;
}
#endif

/*
 * Called when the device goes from unused to used.
 */
static int char_open(struct inode *inode, struct file *file)
{
	struct lancero_char *lro_char;
	/* pointer to containing data structure of the character device inode */
	lro_char = container_of(inode->i_cdev, struct lancero_char, cdev);
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
	/* fetch device specific data stored earlier during open */
	lro = lro_char->lro;
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
	struct lancero_dev *lro = NULL;

	/* allocate zeroed device book keeping structure */
	lro = kzalloc(sizeof(struct lancero_dev), GFP_KERNEL);
	if (!lro) {
		goto err_alloc;
	}
	/* initialize the spin lock */
	spin_lock_init(&lro->irq_lock);
	lro->magic = MAGIC_DEVICE;
	/* create a device to driver reference */
	dev_set_drvdata(&pdev->dev, lro);
	/* create a driver to device reference */
	lro->pci_dev = pdev;
	rc = pci_enable_device(pdev);
	if (rc) {
		goto err_enable;
	}

	/* enable bus master capability */
	pci_set_master(pdev);

	if (msi) {
		/* enable message signaled interrupts */
		rc = pci_enable_msi(pdev);
		/* could not use MSI? */
		if (rc) {
		} else {
			lro->msi_enabled = 1;
		}
	}

#if (defined(CONFIG_ARCH_TI814X) || defined(CONFIG_ARCH_TI816X))
	printk(KERN_DEBUG "TI81xx PCIe RC detected: limiting PCIe MaxReadReq size to 256 bytes.\n");
	pcie_set_readrq(pdev, 256);
#endif

	rc = pci_request_regions(pdev, DRV_NAME);
	/* could not request all regions? */
	if (rc) {
		/* assume device is in use so do not disable it later */
		lro->regions_in_use = 1;
		goto err_regions;
	}
	lro->got_regions = 1;

	/* map BARs */
	rc = map_bars(lro, pdev);
	if (rc)
		goto err_map;

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
	printk(KERN_DEBUG "Lancero IP Core does %ssupport 64-bit DMA.\n",
		(lro->capabilities & CAP_64BIT_DMA)?"":"not ");

	/* 64-bit addressing capability for SGDMA? */
	if ((lro->capabilities & CAP_64BIT_DMA) &&
		(!pci_set_dma_mask(pdev, DMA_BIT_MASK(64)))) {
		/* query for DMA transfer */
		/* @see Documentation/DMA-mapping.txt */
		/* use 64-bit DMA */
		//printk(KERN_DEBUG "Using a 64-bit DMA mask.\n");
		/* use 32-bit DMA for descriptors */
		pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(32));
		/* use 64-bit DMA, 32-bit for consistent */
	} else
	if (!pci_set_dma_mask(pdev, DMA_BIT_MASK(32))) {
		pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(32));
		/* use 32-bit DMA, 32-bit for consistent */
	} else {
		rc = -1;
		goto err_mask;
	}
	lro->irq_line = -1;
	/* request irq, MSI interrupts are not shared */
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
		goto err_ctrl_cdev;
	}
#endif

#if 1
	/* initialize wait queue for events */
	init_waitqueue_head(&lro->events_wq);
	/* initialize events character device */
	lro->events_char_dev = create_sg_char(lro, -1/*no bar*/, NULL, NULL, CHAR_EVENTS);
	if (!lro->events_char_dev) {
		goto err_events_cdev;
	}
#endif
#if 1
	/* probe the interrupt controller and read its capabilities */
	rc = interrupts_probe(lro, LANCERO_OFS_INT_CTRL);
	if (rc)
		goto err_interrupts;
	/* enable the interrupts */
	rc = interrupts_enable(lro, LANCERO_OFS_INT_CTRL, 0x00ffffffUL);
	if (rc)
		goto err_interrupts;
#endif
	/* system indicates write engine */
	if (lro->capabilities & CAP_ENGINE_WRITE) {
		/* allocate and initialize write engine */
		lro->engine[0] = create_engine(lro, LANCERO_OFS_DMA_WRITE, 0/*to_dev*/);
		if (!lro->engine[0])
			goto err_engine;
		lro->engine[0]->name = "write";
		lancero_stop(lro->engine[0]);
	}
	/* system indicates read engine */
	if (lro->capabilities & CAP_ENGINE_READ) {
		/* allocate and initialize read engine */
		lro->engine[1] = create_engine(lro, LANCERO_OFS_DMA_READ, 1/*to_dev*/);
		if (!lro->engine[1])
			goto err_engine;
		lro->engine[1]->name = "read";
		lancero_stop(lro->engine[1]);
	}
	/* any engines? */
	if (lro->capabilities & (CAP_ENGINE_WRITE | CAP_ENGINE_READ)) {
		/* initialize SG DMA character device */
		lro->sgdma_char_dev = create_sg_char(lro, -1/*bar*/, lro->engine[1], lro->engine[0], CHAR_SGDMA);
		if (!lro->sgdma_char_dev) {
			goto err_sgdma_cdev;
		}
	}
	if ((performance_dirs >= 0) && (performance_dirs <= 2)) {
		performance_run(lro, performance_dirs);
	}
	rc = 0;
	
	if (rc == 0)
		goto end;

	/* remove SG DMA character device */
	if (lro->sgdma_char_dev)
		destroy_sg_char(lro->sgdma_char_dev);
err_sgdma_cdev:
	/* destroy_engine */
err_engine:
err_interrupts:
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
	if ((pdev == 0) || (dev_get_drvdata(&pdev->dev) == 0)) {
		return;
	}
	lro = (struct lancero_dev *)dev_get_drvdata(&pdev->dev);

	/* disable all interrupts */
	interrupts_disable(lro, LANCERO_OFS_INT_CTRL, 0xffffffffUL);
	if (lro->engine[0]) {
		/* stop SGDMA write engine */
		engine_reset(lro->engine[0]);
	}
	if (lro->engine[1]) {
		/* stop SGDMA read engine */
		engine_reset(lro->engine[1]);
	}
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
	/* free IRQ */
	if (lro->irq_line >= 0) {
		free_irq(lro->irq_line, (void *)lro);
	}
	/* MSI was enabled? */
	if (lro->msi_enabled) {
		/* Disable MSI @see Documentation/MSI-HOWTO.txt */
		pci_disable_msi(pdev);
		lro->msi_enabled = 0;
	}
	/* unmap the BARs */
	unmap_bars(lro, pdev);
	if (!lro->regions_in_use) {
		pci_disable_device(pdev);
	}
	if (lro->got_regions)
		/* to be called after pci_disable_device()! */
		pci_release_regions(pdev);
}

static int bridge_mmap(struct file *file, struct vm_area_struct *vma)
{
	int rc;
	struct lancero_dev *lro;
	struct lancero_char *lro_char = (struct lancero_char *)file->private_data;
	unsigned long off;
	unsigned long phys;
	unsigned long vsize;
	unsigned long psize;
	lro = lro_char->lro;

	off = vma->vm_pgoff << PAGE_SHIFT;
	/* BAR physical address */
	phys = pci_resource_start(lro->pci_dev, lro_char->bar) + off;
	vsize = vma->vm_end - vma->vm_start;
	/* complete resource */
	psize = pci_resource_end(lro->pci_dev, lro_char->bar) - pci_resource_start(lro->pci_dev, lro_char->bar) + 1 - off;

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
	if (rc)
		return -EAGAIN;
	return 0;
}

static ssize_t char_ctrl_read(struct file *file, char __user *buf,
	size_t count, loff_t *pos)
{
	u32 w;
	void *reg;
	struct lancero_dev *lro;
	struct lancero_char *lro_char = (struct lancero_char *)file->private_data;

	lro = lro_char->lro;

	/* only 32-bit aligned and 32-bit multiples */
	if (count & 3) return -EPROTO;
	if (*pos & 3) return -EPROTO;
	/* first address is BAR base plus file position offset */
	reg = lro->bar[lro_char->bar] + *pos;
	//w = le32_to_cpu(ioread32(reg));
	w = ioread32(reg);
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

	lro = lro_char->lro;

	/* only 32-bit aligned and 32-bit multiples */
	if (count & 3) return -EPROTO;
	if (*pos & 3) return -EPROTO;
	/* first address is BAR base plus file position offset */
	reg = lro->bar[lro_char->bar] + *pos;
	copy_from_user(&w, buf, 4);
	//w = cpu_to_le32(w);
	iowrite32(w, reg);
	*pos += 4;
	return 4;
}

/* seek into DMA file */
static ssize_t char_llseek(struct file *file, char __user *buf,
	size_t count, loff_t *pos)
{
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

	lro = lro_char->lro;

	if (count != 4) return -EPROTO;
	if (*pos & 3) return -EPROTO;

	/* the function servicing the engine will wake us */
	rc = wait_event_interruptible(lro->events_wq, lro->events_irq != 0);
	/* wait_event_interruptible() was interrupted by a signal */
	if (rc == -ERESTARTSYS) {
		return -ERESTARTSYS;
	}
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
#if 0 /* 0 means vectored I/O implemented called as aio_write(), 1 means writev() support */
	.writev = char_sgdma_writev,
#endif
#if 0
        .ioctl = char_sgdma_ioctl,
#endif
#if 1
	/* asynchronous */
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
		/* remember BAR we are attached to */
		lro_char->bar = bar;
		/* couple the control device file operations to the character device */
		cdev_init(&lro_char->cdev, &ctrl_fops);
		kobject_set_name(&lro_char->cdev.kobj, DRV_NAME "_control");
		printk(KERN_DEBUG DRV_NAME "_control = %d:%d\n",
			MAJOR(lro_char->cdevno), MINOR(lro_char->cdevno));
	} else if (type == CHAR_EVENTS) {
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
		goto fail_add;
	}

	goto success;
fail_device:
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
	printk(KERN_INFO DRV_NAME " built " __DATE__ " " __TIME__ "\n");
	rc = pci_register_driver(&pci_driver);
	return rc;
}

static void __exit lancero_exit(void)
{
	/* unregister this driver from the PCI bus driver */
	pci_unregister_driver(&pci_driver);
}

module_init(lancero_init);
module_exit(lancero_exit);
