/**
 * Driver module exercising scatterlist interfaces
 *
 * Copyright (C) 2007-2020 Sidebranch, Leon Woestenberg <leon@sidebranch.com>
 * Copyright (C) 2020-2023 BrightAI BV, Leon Woestenberg <leon@brightai.nl>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <linux/pagemap.h>
#include <linux/scatterlist.h>

/* describes a mapping from a virtual memory user buffer to scatterlist */
struct sg_mapping_t {
	/* scatter gather list used to map in the relevant user pages */
	struct scatterlist *sgl;
	/* pointer to array of page pointers */
	struct page **pages;
	/* maximum amount of pages in the scatterlist and page array */
	int max_pages;
	/* current amount of mapped pages in the scatterlist and page array */
	int mapped_pages;
};

struct sg_mapping_t *sg_create_mapper(unsigned long max_len);
void sg_destroy_mapper(struct sg_mapping_t *sgm);

int sgm_get_user_pages(struct sg_mapping_t *sgm, const char *start, size_t count, int to_user);
int sgm_put_user_pages(struct sg_mapping_t *sgm, int dirtied);
void sgm_dirty_pages(struct sg_mapping_t *sgm);

int sgm_get_user_pages_vectored(struct sg_mapping_t *sgm, const struct iovec *iov, unsigned long iov_count, int to_user);


int sgm_kernel_pages(struct sg_mapping_t *sgm, const char *start, size_t count, int to_user);


