/* Broadcom FlexRM Mailbox Driver
 *
 * Copyright (C) 2016 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * FlexRM descriptor library
 */

#ifndef __FLEXRM_DESC_H__
#define __FLEXRM_DESC_H__

#include <linux/types.h>
#include <linux/device.h>
#include <linux/mailbox/brcm-message.h>

extern u64 flexrm_read_desc(void *desc_ptr);

extern void flexrm_write_desc(void *desc_ptr, u64 desc);

extern u32 flexrm_cmpl_desc_to_reqid(u64 cmpl_desc);

extern int flexrm_cmpl_desc_to_error(u64 cmpl_desc);

extern bool flexrm_is_next_table_desc(void *desc_ptr);

extern u64 flexrm_next_table_desc(u32 toggle, dma_addr_t next_addr);

extern u64 flexrm_null_desc(u32 toogle);

extern u32 flexrm_estimate_header_desc_count(u32 nhcnt);

extern bool flexrm_sanity_check(struct brcm_message *msg);

extern u32 flexrm_estimate_nonheader_desc_count(struct brcm_message *msg);

extern int flexrm_dma_map(struct device *dev, struct brcm_message *msg);

extern void flexrm_dma_unmap(struct device *dev, struct brcm_message *msg);

extern void *flexrm_write_descs(struct brcm_message *msg, u32 nhcnt,
				 u32 reqid, void *desc_ptr, u32 toggle,
				 void *start_desc, void *end_desc);

#endif /* __FLEXRM_DESC_H__ */
