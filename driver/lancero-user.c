/**
 * Driver module exercising scatterlist interfaces
 *
 * Copyright (C) 2007-2012 Sidebranch
 *
 * Leon Woestenberg <leon@sidebranch.com>
 *
 */

//#define DEBUG

#include <linux/cdev.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uio.h>

#include <asm/byteorder.h>
#include <asm/cacheflush.h>
#include <asm/delay.h>
#include <asm/pci.h>

#include "lancero-user.h"

int lancero_pages_currently_mapped = 0;

/*
 * sg_create_mapper() - Create a mapper for virtual memory to scatterlist.
 *
 * @max_len: Maximum number of bytes that can be mapped at a time.
 *
 * Allocates a book keeping structure, array to page pointers and a scatter
 * list to map virtual user memory into.
 *
 */
struct sg_mapping_t *sg_create_mapper(unsigned long max_len)
{
	struct sg_mapping_t *sgm;
  WARN_ON(max_len == 0);
	if (max_len == 0) {
		return NULL;
  }
	/* allocate bookkeeping */
	sgm = kcalloc(1, sizeof(struct sg_mapping_t), GFP_KERNEL);
  WARN_ON(sgm == NULL);
	if (sgm == NULL)
		return NULL;
	/* upper bound of pages */
	sgm->max_pages = max_len / PAGE_SIZE + 2;
	/* allocate an array of struct page pointers */
	sgm->pages = kcalloc(sgm->max_pages, sizeof(*sgm->pages), GFP_KERNEL);
  WARN_ON(sgm->pages == NULL);
	if (sgm->pages == NULL) {
		kfree(sgm);
		return NULL;
	}
	/* allocate a scatter gather list */
	sgm->sgl = kcalloc(sgm->max_pages, sizeof(struct scatterlist), GFP_KERNEL);
  WARN_ON(sgm->sgl == NULL);
	if (sgm->sgl == NULL) {
		kfree(sgm->pages);
		kfree(sgm);
		return NULL;
	}
	sg_init_table(sgm->sgl, sgm->max_pages);
	return sgm;
};

/*
 * sg_destroy_mapper() - Destroy a mapper for virtual memory to scatterlist.
 *
 * @sgm scattergather mapper handle.
 */
void sg_destroy_mapper(struct sg_mapping_t *sgm)
{
	/* user failed to call sgm_unmap_user_pages() */
	BUG_ON(sgm->mapped_pages > 0);
	/* free scatterlist */
	kfree(sgm->sgl);
	/* free page array */
	kfree(sgm->pages);
	/* free mapper handle */
	kfree(sgm);
};

#if 0
long get_user_pages(struct task_struct *tsk, struct mm_struct *mm,
                    unsigned long start, unsigned long nr_pages,
                    unsigned int gup_flags, struct page **pages,
                    struct vm_area_struct **vmas);
long get_user_pages_locked(struct task_struct *tsk, struct mm_struct *mm,
                    unsigned long start, unsigned long nr_pages,
                    unsigned int gup_flags, struct page **pages, int *locked);
long __get_user_pages_unlocked(struct task_struct *tsk, struct mm_struct *mm,
                               unsigned long start, unsigned long nr_pages,
                               struct page **pages, unsigned int gup_flags);
long get_user_pages_unlocked(struct task_struct *tsk, struct mm_struct *mm,
                    unsigned long start, unsigned long nr_pages,
                    struct page **pages, unsigned int gup_flags);
int get_user_pages_fast(unsigned long start, int nr_pages, int write,
                        struct page **pages);
#endif

/*
 * sgm_map_user_pages() - Get user pages and build a scatterlist.
 *
 * @sgm scattergather mapper handle.
 * @start User space buffer (virtual memory) address.
 * @count Number of bytes in buffer.
 * @to_user !0 if data direction is from device to user space.
 *
 * Returns Number of entries in the table on success, -1 on error.
 */
int sgm_get_user_pages(struct sg_mapping_t *sgm, const char *start, size_t count, int to_user)
{
	//u64 boe = (u64)start;
#if 1
	/* calculate page frame number @todo use macro's */
	const unsigned long first = ((unsigned long)start & PAGE_MASK) >> PAGE_SHIFT;
	const unsigned long last  = (((unsigned long)start + count - 1) & PAGE_MASK) >> PAGE_SHIFT;
#else
	/* XXX does not work, virt_to_page() not allowed on user space? */
	const unsigned long first = page_to_pfn(virt_to_page(start));
	const unsigned long last  = page_to_pfn(virt_to_page(start + count - 1));
#endif

	/* the number of pages we want to map in */
	const int nr_pages = last - first + 1;
	int rc, i;
	struct scatterlist *sgl = sgm->sgl;
	/* pointer to array of page pointers */
	struct page **pages = sgm->pages;

	/* no pages should currently be mapped */
	if (start + count < start) {
		printk(KERN_DEBUG "sgm_get_user_pages(): start + count < start?!\n");
		return -EINVAL;
	}
	if (nr_pages > sgm->max_pages) {
		printk(KERN_DEBUG "sgm_get_user_pages(): nr_pages > sgm->max_pages?!\n");
		return -EINVAL;
	}
	if (count == 0) {
		printk(KERN_DEBUG "sgm_get_user_pages(): count = 0?!\n");
		return 0;
	}
	/* initialize scatter gather list */
	sg_init_table(sgl, nr_pages);

	for (i = 0; i < nr_pages - 1; i++) {
		pages[i] = NULL;
	}

	/* try to fault in all of the necessary pages */
	down_read(&current->mm->mmap_sem);

#if 0 /* older API */
	/* to_user != 0 means read from device, write into user space buffer memory */
	rc = get_user_pages(current, current->mm, (unsigned long)start, nr_pages, to_user,
		0 /* don't force */, pages, NULL);
#else /* newer API, @TODO use kernel version */
	/* to_user != 0 means read from device, write into user space buffer memory */
	rc = get_user_pages(current, current->mm, (unsigned long)start, nr_pages, to_user? FOLL_WRITE: 0,
		pages, NULL);
#endif
	up_read(&current->mm->mmap_sem);

	/* errors and no page mapped should return here */
	if (rc < nr_pages) {
		printk(KERN_DEBUG "sgm_get_user_pages(): rc = %d, nr_pages = %d\n", rc, nr_pages);
		if (rc > 0) sgm->mapped_pages = rc;
		goto out_unmap;
	}
	sgm->mapped_pages = rc;

	/* XXX: scsi/st.c is mentioning this as FIXME */
	for (i = 0; i < nr_pages; i++) {
		flush_dcache_page(pages[i]);
	}

	/* populate the scatter/gather list */
	sg_set_page(&sgl[0], pages[0], 0 /*length*/, offset_in_page(start));

	/* multiple pages? */
	if (nr_pages > 1) {
		/* offset was already set above */
		sgl[0].length = PAGE_SIZE - sgl[0].offset;
		count -= sgl[0].length;
		/* iterate over further pages, except the last page */
		for (i = 1; i < nr_pages - 1; i++) {
			BUG_ON(count < PAGE_SIZE);
			/* set the scatter gather entry i */
			sg_set_page(&sgl[i], pages[i], PAGE_SIZE, 0/*offset*/);
			count -= PAGE_SIZE;
		}
		/* count bytes at offset 0 in the page */
		sg_set_page(&sgl[i], pages[i], count, 0);
	}
	/* single page */
	else {
		/* limit the count */
		sgl[0].length = count;
	}

	lancero_pages_currently_mapped += nr_pages;
	//printk(KERN_DEBUG "sgm_get_user_pages(): nr_pages = %d, mapped = %d\n", nr_pages, lancero_pages_currently_mapped);
	return nr_pages;

out_unmap:
	/* { rc < 0 means errors, >= 0 means not all pages could be mapped } */
	/* did we map any pages? */
	for (i = 0; i < sgm->mapped_pages; i++)
		put_page(pages[i]);
	rc = -ENOMEM;
	sgm->mapped_pages = 0;
	return rc;
}

/*
 * sgm_unmap_user_pages() - Mark the pages dirty and release them.
 *
 * Pages mapped earlier with sgm_map_user_pages() are released here.
 * after being marked dirtied by the DMA.
 *
 */
int sgm_put_user_pages(struct sg_mapping_t *sgm, int dirtied)
{
	int i;

	lancero_pages_currently_mapped -= sgm->mapped_pages;
	//printk(KERN_DEBUG "sgm_put_user_pages(sgm->mapped_pages=%d), mapped = %d\n", sgm->mapped_pages, lancero_pages_currently_mapped);

	/* mark page dirty */
	if (dirtied)
		sgm_dirty_pages(sgm);
	/* put (i.e. release) pages */
	for (i = 0; i < sgm->mapped_pages; i++) {
		put_page(sgm->pages[i]);
	}
	/* remember we have zero pages */
	sgm->mapped_pages = 0;
	return 0;
}

void sgm_dirty_pages(struct sg_mapping_t *sgm)
{
	int i;
	/* iterate over all mapped pages */
	for (i = 0; i < sgm->mapped_pages; i++) {
		/* mark page dirty */
		SetPageDirty(sgm->pages[i]);
	}
}

/* sgm_kernel_pages() -- create a sgm map from a vmalloc()ed memory */
int sgm_kernel_pages(struct sg_mapping_t *sgm, const char *start, size_t count, int to_user)
{
	/* calculate page frame number @todo use macro's */
	const unsigned long first = ((unsigned long)start & PAGE_MASK) >> PAGE_SHIFT;
	const unsigned long last  = (((unsigned long)start + count - 1) & PAGE_MASK) >> PAGE_SHIFT;

	/* the number of pages we want to map in */
	const int nr_pages = last - first + 1;
	int rc, i;
	struct scatterlist *sgl = sgm->sgl;
	/* pointer to array of page pointers */
	struct page **pages = sgm->pages;
	unsigned char *virt = (unsigned char *)start;

	/* no pages should currently be mapped */
	if (start + count < start)
		return -EINVAL;
	if (nr_pages > sgm->max_pages)
		return -EINVAL;
	if (count == 0)
		return 0;
	/* initialize scatter gather list */
	sg_init_table(sgl, nr_pages);


	/* get pages belonging to vmalloc()ed space */
	for (i = 0; i < nr_pages; i++, virt += PAGE_SIZE) {
		pages[i] = vmalloc_to_page(virt);
		if (pages[i] == NULL)
			goto err;
		/* make sure page was allocated using vmalloc_32() */
		BUG_ON(PageHighMem(pages[i]));
	}
	sgm->mapped_pages = nr_pages;

	/* XXX: scsi/st.c is mentioning this as FIXME */
	for (i = 0; i < nr_pages; i++) {
		flush_dcache_page(pages[i]);
	}

	/* set first page */
	sg_set_page(&sgl[0], pages[0], 0 /*length*/, offset_in_page(start));

	/* multiple pages? */
	if (nr_pages > 1) {
		/* { sgl[0].offset is already set } */
		sgl[0].length = PAGE_SIZE - sgl[0].offset;
		count -= sgl[0].length;
		/* iterate over further pages, except the last page */
		for (i = 1; i < nr_pages - 1; i++) {
			/* set the scatter gather entry i */
			sg_set_page(&sgl[i], pages[i], PAGE_SIZE, 0/*offset*/);
			count -= PAGE_SIZE;
		}
		/* 'count' bytes remaining at offset 0 in the page */
		sg_set_page(&sgl[i], pages[i], count, 0);
	}
	/* single page */
	else {
		/* limit the count */
		sgl[0].length = count;
	}
	return nr_pages;
err:
	rc = -ENOMEM;
	sgm->mapped_pages = 0;
	return rc;
}

#if 0
/*
 * sgm_map_user_pages_vectored() - Get user pages and build a scatterlist.
 *
 * Calculate how many consecutive pages to map for each I/O vector.
 *
 *
 * @sgm scattergather mapper handle.
 *
 *
 *
 * @to_user !0 if data direction is from device to user space.
 *
 * Returns Number of entries in the table on success, -1 on error.
 */
int sgm_get_user_pages_vectored(struct sg_mapping_t *sgm, const struct iovec *iov, unsigned long iov_count, int to_user)
{
	/* user space buffers bases and sizes */
    char *start[MAX_VECTOR];
    size_t count[MAX_VECTOR];
    /* page numbers */
	unsigned long first[MAX_VECTOR];
	unsigned long last[MAX_VECTOR];
	int nr_pages[MAX_VECTOR], nr_pages_total = 0;
	int rc, i, v, mapped_pages;
	struct scatterlist *sgl = sgm->sgl;
	/* pointer to array of page pointers */
	//struct page **pages = sgm->pages;

	if (iov_count == 0) {
		printk(KERN_DEBUG "sgm_get_user_pages_vectored(): iov_count = 0?!\n");
		return 0;
	}
	/* local modifiable copy of user space buffers bases and sizes */
	for (v = 0; v < count; v++) {
		start[v] = (char *)iov[v]->iov_base;
		count[v] = (size_t)iov[v]->iov_len;
		if (count[v] == 0) {
			printk(KERN_DEBUG "sgm_get_user_pages_vectored(): count[v%d] = 0?!\n", v);
		return 0;
	}

	for (v = 0; v < count; v++) {
    	first[v] = ((unsigned long)iov[v]->iov_base & PAGE_MASK) >> PAGE_SHIFT;
    	last[v] = (((unsigned long)iov[v]->iov_base + iov[v]->iov_len - 1) & PAGE_MASK) >> PAGE_SHIFT;
  		/* the number of pages we want to map in */
		nr_pages[v] = last[v] - first[v] + 1;
		nr_pages_total += nr_pages[v];
	}

#if 0
	/* no pages should currently be mapped */
	if (start + count < start) {
		printk(KERN_DEBUG "sgm_get_user_pages(): start + count < start?!\n");
		return -EINVAL;
	}
#endif
	if (nr_pages_total > sgm->max_pages) {
		printk(KERN_DEBUG "sgm_get_user_pages(): nr_pages > sgm->max_pages?!\n");
		return -EINVAL;
	}
	/* initialize scatter gather list */
	sg_init_table(sgl, nr_pages_total);

	for (i = 0; i < nr_pages_total - 1; i++) {
		sgm->pages[i] = NULL;
	}
	 = 0;
	sgm->mapped_pages = 0;

	/* iterate over vectors, and map pages for each user buffer */
	for (v = 0; v < count; v++) {
		/* try to fault in all of the necessary pages */
		down_read(&current->mm->mmap_sem);
		/* to_user != 0 means read from device, write into user space buffer memory */
		mapped_pages = get_user_pages(current, current->mm, (unsigned long)iov[v]->iov_base, nr_pages[v], to_user,
			0 /* don't force */, &sgm->pages[sgm->mapped_pages], NULL);
		up_read(&current->mm->mmap_sem);

		/* errors and no page mapped should return here */
		if (mapped_pages < nr_pages_total) {
			printk(KERN_DEBUG "sgm_get_user_pages(): rc = %d, nr_pages = %d\n", mapped_pages, nr_pages);
			/* list of pages was partially mapped??! */
			if (mapped_pages > 0) sgm->mapped_pages += mapped_pages;
			goto out_unmap;
		}
		sgm->mapped_pages += mapped_pages;
	}

	/* XXX: scsi/st.c is mentioning this as FIXME */
	for (i = 0; i < nr_pages_total; i++) {
		flush_dcache_page(sgm->pages[i]);
	}
#if 0
	for ()

	/* populate the scatter/gather list */
	sg_set_page(&sgl[0], sgm->pages[0], 0 /*length*/, offset_in_page(start));

	/* multiple pages? */
	if (nr_pages > 1) {
		/* offset was already set above */
		sgl[0].length = PAGE_SIZE - sgl[0].offset;
		count -= sgl[0].length;
		/* iterate over further pages, except the last page */
		for (i = 1; i < nr_pages - 1; i++) {
			BUG_ON(count < PAGE_SIZE);
			/* set the scatter gather entry i */
			sg_set_page(&sgl[i], sgm->pages[i], PAGE_SIZE, 0/*offset*/);
			count -= PAGE_SIZE;
		}
		/* count bytes at offset 0 in the page */
		sg_set_page(&sgl[i], sgm->pages[i], count, 0);
	}
	/* single page */
	else {
		/* limit the count */
		sgl[0].length = count;
	}

#endif

	lancero_pages_currently_mapped += nr_pages_total;
	//printk(KERN_DEBUG "sgm_get_user_pages(): nr_pages_total = %d, mapped = %d\n", nr_pages_total, lancero_pages_currently_mapped);
	return nr_pages;

out_unmap:
	/* { rc < 0 means errors, >= 0 means not all pages could be mapped } */
	/* did we map any pages? */
	for (i = 0; i < sgm->mapped_pages; i++)
		put_page(sgm->pages[i]);
	rc = -ENOMEM;
	sgm->mapped_pages = 0;
	return rc;
}
#endif
