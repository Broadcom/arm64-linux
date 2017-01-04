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

#include <asm/barrier.h>
#include <asm/byteorder.h>
#include <linux/dma-mapping.h>
#include <linux/printk.h>

#include "flexrm-desc.h"

/* Completion descriptor format */
#define CMPL_OPAQUE_SHIFT			0
#define CMPL_OPAQUE_MASK			0xffff
#define CMPL_ENGINE_STATUS_SHIFT		16
#define CMPL_ENGINE_STATUS_MASK			0xffff
#define CMPL_DME_STATUS_SHIFT			32
#define CMPL_DME_STATUS_MASK			0xffff
#define CMPL_RM_STATUS_SHIFT			48
#define CMPL_RM_STATUS_MASK			0xffff

/* Completion DME status code */
#define DME_STATUS_MEM_COR_ERR			BIT(0)
#define DME_STATUS_MEM_UCOR_ERR			BIT(1)
#define DME_STATUS_FIFO_UNDERFLOW		BIT(2)
#define DME_STATUS_FIFO_OVERFLOW		BIT(3)
#define DME_STATUS_RRESP_ERR			BIT(4)
#define DME_STATUS_BRESP_ERR			BIT(5)
#define DME_STATUS_ERROR_MASK			(DME_STATUS_MEM_COR_ERR | \
						 DME_STATUS_MEM_UCOR_ERR | \
						 DME_STATUS_FIFO_UNDERFLOW | \
						 DME_STATUS_FIFO_OVERFLOW | \
						 DME_STATUS_RRESP_ERR | \
						 DME_STATUS_BRESP_ERR)

/* Completion RM status code */
#define RM_STATUS_CODE_SHIFT			0
#define RM_STATUS_CODE_MASK			0x3ff
#define RM_STATUS_CODE_GOOD			0x0
#define RM_STATUS_CODE_AE_TIMEOUT		0x3ff

/* General descriptor format */
#define DESC_TYPE_SHIFT				60
#define DESC_TYPE_MASK				0xf
#define DESC_PAYLOAD_SHIFT			0
#define DESC_PAYLOAD_MASK			0x0fffffffffffffff

/* Null descriptor format  */
#define NULL_TYPE				0
#define NULL_TOGGLE_SHIFT			58
#define NULL_TOGGLE_MASK			0x1

/* Header descriptor format */
#define HEADER_TYPE				1
#define HEADER_TOGGLE_SHIFT			58
#define HEADER_TOGGLE_MASK			0x1
#define HEADER_ENDPKT_SHIFT			57
#define HEADER_ENDPKT_MASK			0x1
#define HEADER_STARTPKT_SHIFT			56
#define HEADER_STARTPKT_MASK			0x1
#define HEADER_BDCOUNT_SHIFT			36
#define HEADER_BDCOUNT_MASK			0x1f
#define HEADER_BDCOUNT_MAX			HEADER_BDCOUNT_MASK
#define HEADER_FLAGS_SHIFT			16
#define HEADER_FLAGS_MASK			0xffff
#define HEADER_OPAQUE_SHIFT			0
#define HEADER_OPAQUE_MASK			0xffff

/* Source (SRC) descriptor format */
#define SRC_TYPE				2
#define SRC_LENGTH_SHIFT			44
#define SRC_LENGTH_MASK				0xffff
#define SRC_ADDR_SHIFT				0
#define SRC_ADDR_MASK				0x00000fffffffffff

/* Destination (DST) descriptor format */
#define DST_TYPE				3
#define DST_LENGTH_SHIFT			44
#define DST_LENGTH_MASK				0xffff
#define DST_ADDR_SHIFT				0
#define DST_ADDR_MASK				0x00000fffffffffff

/* Immediate (IMM) descriptor format */
#define IMM_TYPE				4
#define IMM_DATA_SHIFT				0
#define IMM_DATA_MASK				0x0fffffffffffffff

/* Next pointer (NPTR) descriptor format */
#define NPTR_TYPE				5
#define NPTR_TOGGLE_SHIFT			58
#define NPTR_TOGGLE_MASK			0x1
#define NPTR_ADDR_SHIFT				0
#define NPTR_ADDR_MASK				0x00000fffffffffff

/* Mega source (MSRC) descriptor format */
#define MSRC_TYPE				6
#define MSRC_LENGTH_SHIFT			44
#define MSRC_LENGTH_MASK			0xffff
#define MSRC_ADDR_SHIFT				0
#define MSRC_ADDR_MASK				0x00000fffffffffff

/* Mega destination (MDST) descriptor format */
#define MDST_TYPE				7
#define MDST_LENGTH_SHIFT			44
#define MDST_LENGTH_MASK			0xffff
#define MDST_ADDR_SHIFT				0
#define MDST_ADDR_MASK				0x00000fffffffffff

/* Source with tlast (SRCT) descriptor format */
#define SRCT_TYPE				8
#define SRCT_LENGTH_SHIFT			44
#define SRCT_LENGTH_MASK			0xffff
#define SRCT_ADDR_SHIFT				0
#define SRCT_ADDR_MASK				0x00000fffffffffff

/* Destination with tlast (DSTT) descriptor format */
#define DSTT_TYPE				9
#define DSTT_LENGTH_SHIFT			44
#define DSTT_LENGTH_MASK			0xffff
#define DSTT_ADDR_SHIFT				0
#define DSTT_ADDR_MASK				0x00000fffffffffff

/* Immediate with tlast (IMMT) descriptor format */
#define IMMT_TYPE				10
#define IMMT_DATA_SHIFT				0
#define IMMT_DATA_MASK				0x0fffffffffffffff

/* Descriptor helper macros */
#define DESC_DEC(_d, _s, _m)			(((_d) >> (_s)) & (_m))
#define DESC_ENC(_d, _v, _s, _m)		\
			do { \
				(_d) &= ~((u64)(_m) << (_s)); \
				(_d) |= (((u64)(_v) & (_m)) << (_s)); \
			} while (0)

u64 flexrm_read_desc(void *desc_ptr)
{
	return le64_to_cpu(*((u64 *)desc_ptr));
}

void flexrm_write_desc(void *desc_ptr, u64 desc)
{
	*((u64 *)desc_ptr) = cpu_to_le64(desc);
}

u32 flexrm_cmpl_desc_to_reqid(u64 cmpl_desc)
{
	return (u32)(cmpl_desc & CMPL_OPAQUE_MASK);
}

int flexrm_cmpl_desc_to_error(u64 cmpl_desc)
{
	u32 status;

	status = DESC_DEC(cmpl_desc, CMPL_DME_STATUS_SHIFT,
			  CMPL_DME_STATUS_MASK);
	if (status & DME_STATUS_ERROR_MASK)
		return -EIO;

	status = DESC_DEC(cmpl_desc, CMPL_RM_STATUS_SHIFT,
			  CMPL_RM_STATUS_MASK);
	status &= RM_STATUS_CODE_MASK;
	if (status == RM_STATUS_CODE_AE_TIMEOUT)
		return -ETIMEDOUT;

	return 0;
}

bool flexrm_is_next_table_desc(void *desc_ptr)
{
	u64 desc = flexrm_read_desc(desc_ptr);
	u32 type = DESC_DEC(desc, DESC_TYPE_SHIFT, DESC_TYPE_MASK);

	return (type == NPTR_TYPE) ? true : false;
}

u64 flexrm_next_table_desc(u32 toggle, dma_addr_t next_addr)
{
	u64 desc = 0;

	DESC_ENC(desc, NPTR_TYPE, DESC_TYPE_SHIFT, DESC_TYPE_MASK);
	DESC_ENC(desc, toggle, NPTR_TOGGLE_SHIFT, NPTR_TOGGLE_MASK);
	DESC_ENC(desc, next_addr, NPTR_ADDR_SHIFT, NPTR_ADDR_MASK);

	return desc;
}

u64 flexrm_null_desc(u32 toggle)
{
	u64 desc = 0;

	DESC_ENC(desc, NULL_TYPE, DESC_TYPE_SHIFT, DESC_TYPE_MASK);
	DESC_ENC(desc, toggle, NULL_TOGGLE_SHIFT, NULL_TOGGLE_MASK);

	return desc;
}

u32 flexrm_estimate_header_desc_count(u32 nhcnt)
{
	u32 hcnt = nhcnt / HEADER_BDCOUNT_MAX;

	if (!(nhcnt % HEADER_BDCOUNT_MAX))
		hcnt += 1;

	return hcnt;
}

static void flexrm_flip_header_toogle(void *desc_ptr)
{
	u64 desc = flexrm_read_desc(desc_ptr);

	if (desc & ((u64)0x1 << HEADER_TOGGLE_SHIFT))
		desc &= ~((u64)0x1 << HEADER_TOGGLE_SHIFT);
	else
		desc |= ((u64)0x1 << HEADER_TOGGLE_SHIFT);

	flexrm_write_desc(desc_ptr, desc);
}

static u64 flexrm_header_desc(u32 toggle, u32 startpkt, u32 endpkt,
			       u32 bdcount, u32 flags, u32 opaque)
{
	u64 desc = 0;

	DESC_ENC(desc, HEADER_TYPE, DESC_TYPE_SHIFT, DESC_TYPE_MASK);
	DESC_ENC(desc, toggle, HEADER_TOGGLE_SHIFT, HEADER_TOGGLE_MASK);
	DESC_ENC(desc, startpkt, HEADER_STARTPKT_SHIFT, HEADER_STARTPKT_MASK);
	DESC_ENC(desc, endpkt, HEADER_ENDPKT_SHIFT, HEADER_ENDPKT_MASK);
	DESC_ENC(desc, bdcount, HEADER_BDCOUNT_SHIFT, HEADER_BDCOUNT_MASK);
	DESC_ENC(desc, flags, HEADER_FLAGS_SHIFT, HEADER_FLAGS_MASK);
	DESC_ENC(desc, opaque, HEADER_OPAQUE_SHIFT, HEADER_OPAQUE_MASK);

	return desc;
}

static void flexrm_enqueue_desc(u32 nhpos, u32 nhcnt, u32 reqid,
				 u64 desc, void **desc_ptr, u32 *toggle,
				 void *start_desc, void *end_desc)
{
	u64 d;
	u32 nhavail, _toggle, _startpkt, _endpkt, _bdcount;

	/* Sanity check */
	if (nhcnt <= nhpos)
		return;

	/*
	 * Each request or packet start with a HEADER descriptor followed
	 * by one or more non-HEADER descriptors (SRC, SRCT, MSRC, DST,
	 * DSTT, MDST, IMM, and IMMT). The number of non-HEADER descriptors
	 * following a HEADER descriptor is represented by BDCOUNT field
	 * of HEADER descriptor. The max value of BDCOUNT field is 31 which
	 * means we can only have 31 non-HEADER descriptors following one
	 * HEADER descriptor.
	 *
	 * In general use, number of non-HEADER descriptors can easily go
	 * beyond 31. To tackle this situation, we have packet (or request)
	 * extenstion bits (STARTPKT and ENDPKT) in the HEADER descriptor.
	 *
	 * To use packet extension, the first HEADER descriptor of request
	 * (or packet) will have STARTPKT=1 and ENDPKT=0. The intermediate
	 * HEADER descriptors will have STARTPKT=0 and ENDPKT=0. The last
	 * HEADER descriptor will have STARTPKT=0 and ENDPKT=1. Also, the
	 * TOGGLE bit of the first HEADER will be set to invalid state to
	 * ensure that FlexRM does not start fetching descriptors till all
	 * descriptors are enqueued. The user of this function will flip
	 * the TOGGLE bit of first HEADER after all descriptors are
	 * enqueued.
	 */

	if ((nhpos % HEADER_BDCOUNT_MAX == 0) && (nhcnt - nhpos)) {
		/* Prepare the header descriptor */
		nhavail = (nhcnt - nhpos);
		_toggle = (nhpos == 0) ? !(*toggle) : (*toggle);
		_startpkt = (nhpos == 0) ? 0x1 : 0x0;
		_endpkt = (nhavail <= HEADER_BDCOUNT_MAX) ? 0x1 : 0x0;
		_bdcount = (nhavail <= HEADER_BDCOUNT_MAX) ?
				nhavail : HEADER_BDCOUNT_MAX;
		if (nhavail <= HEADER_BDCOUNT_MAX)
			_bdcount = nhavail;
		else
			_bdcount = HEADER_BDCOUNT_MAX;
		d = flexrm_header_desc(_toggle, _startpkt, _endpkt,
					_bdcount, 0x0, reqid);

		/* Write header descriptor */
		flexrm_write_desc(*desc_ptr, d);

		/* Point to next descriptor */
		*desc_ptr += sizeof(desc);
		if (*desc_ptr == end_desc)
			*desc_ptr = start_desc;

		/* Skip next pointer descriptors */
		while (flexrm_is_next_table_desc(*desc_ptr)) {
			*toggle = (*toggle) ? 0 : 1;
			*desc_ptr += sizeof(desc);
			if (*desc_ptr == end_desc)
				*desc_ptr = start_desc;
		}
	}

	/* Write desired descriptor */
	flexrm_write_desc(*desc_ptr, desc);

	/* Point to next descriptor */
	*desc_ptr += sizeof(desc);
	if (*desc_ptr == end_desc)
		*desc_ptr = start_desc;

	/* Skip next pointer descriptors */
	while (flexrm_is_next_table_desc(*desc_ptr)) {
		*toggle = (*toggle) ? 0 : 1;
		*desc_ptr += sizeof(desc);
		if (*desc_ptr == end_desc)
			*desc_ptr = start_desc;
	}
}

static u64 flexrm_src_desc(dma_addr_t addr, unsigned int length)
{
	u64 desc = 0;

	DESC_ENC(desc, SRC_TYPE, DESC_TYPE_SHIFT, DESC_TYPE_MASK);
	DESC_ENC(desc, length, SRC_LENGTH_SHIFT, SRC_LENGTH_MASK);
	DESC_ENC(desc, addr, SRC_ADDR_SHIFT, SRC_ADDR_MASK);

	return desc;
}

static u64 flexrm_msrc_desc(dma_addr_t addr, unsigned int length_div_16)
{
	u64 desc = 0;

	DESC_ENC(desc, MSRC_TYPE, DESC_TYPE_SHIFT, DESC_TYPE_MASK);
	DESC_ENC(desc, length_div_16, MSRC_LENGTH_SHIFT, MSRC_LENGTH_MASK);
	DESC_ENC(desc, addr, MSRC_ADDR_SHIFT, MSRC_ADDR_MASK);

	return desc;
}

static u64 flexrm_dst_desc(dma_addr_t addr, unsigned int length)
{
	u64 desc = 0;

	DESC_ENC(desc, DST_TYPE, DESC_TYPE_SHIFT, DESC_TYPE_MASK);
	DESC_ENC(desc, length, DST_LENGTH_SHIFT, DST_LENGTH_MASK);
	DESC_ENC(desc, addr, DST_ADDR_SHIFT, DST_ADDR_MASK);

	return desc;
}

static u64 flexrm_mdst_desc(dma_addr_t addr, unsigned int length_div_16)
{
	u64 desc = 0;

	DESC_ENC(desc, MDST_TYPE, DESC_TYPE_SHIFT, DESC_TYPE_MASK);
	DESC_ENC(desc, length_div_16, MDST_LENGTH_SHIFT, MDST_LENGTH_MASK);
	DESC_ENC(desc, addr, MDST_ADDR_SHIFT, MDST_ADDR_MASK);

	return desc;
}

static u64 flexrm_imm_desc(u64 data)
{
	u64 desc = 0;

	DESC_ENC(desc, IMM_TYPE, DESC_TYPE_SHIFT, DESC_TYPE_MASK);
	DESC_ENC(desc, data, IMM_DATA_SHIFT, IMM_DATA_MASK);

	return desc;
}

static u64 flexrm_srct_desc(dma_addr_t addr, unsigned int length)
{
	u64 desc = 0;

	DESC_ENC(desc, SRCT_TYPE, DESC_TYPE_SHIFT, DESC_TYPE_MASK);
	DESC_ENC(desc, length, SRCT_LENGTH_SHIFT, SRCT_LENGTH_MASK);
	DESC_ENC(desc, addr, SRCT_ADDR_SHIFT, SRCT_ADDR_MASK);

	return desc;
}

static u64 flexrm_dstt_desc(dma_addr_t addr, unsigned int length)
{
	u64 desc = 0;

	DESC_ENC(desc, DSTT_TYPE, DESC_TYPE_SHIFT, DESC_TYPE_MASK);
	DESC_ENC(desc, length, DSTT_LENGTH_SHIFT, DSTT_LENGTH_MASK);
	DESC_ENC(desc, addr, DSTT_ADDR_SHIFT, DSTT_ADDR_MASK);

	return desc;
}

static u64 flexrm_immt_desc(u64 data)
{
	u64 desc = 0;

	DESC_ENC(desc, IMMT_TYPE, DESC_TYPE_SHIFT, DESC_TYPE_MASK);
	DESC_ENC(desc, data, IMMT_DATA_SHIFT, IMMT_DATA_MASK);

	return desc;
}

static bool flexrm_spu_sanity_check(struct brcm_message *msg)
{
	struct scatterlist *sg;

	if (!msg->spu.src || !msg->spu.dst)
		return false;
	for (sg = msg->spu.src; sg; sg = sg_next(sg)) {
		if (sg->length & 0xf) {
			if (sg->length > SRC_LENGTH_MASK)
				return false;
		} else {
			if (sg->length > (MSRC_LENGTH_MASK * 16))
				return false;
		}
	}
	for (sg = msg->spu.dst; sg; sg = sg_next(sg)) {
		if (sg->length & 0xf) {
			if (sg->length > DST_LENGTH_MASK)
				return false;
		} else {
			if (sg->length > (MDST_LENGTH_MASK * 16))
				return false;
		}
	}

	return true;
}

static u32 flexrm_spu_estimate_nonheader_desc_count(struct brcm_message *msg)
{
	u32 cnt = 0;
	unsigned int dst_target = 0;
	struct scatterlist *src_sg = msg->spu.src, *dst_sg = msg->spu.dst;

	while (src_sg || dst_sg) {
		if (src_sg) {
			cnt++;
			dst_target = src_sg->length;
			src_sg = sg_next(src_sg);
		} else
			dst_target = UINT_MAX;

		while (dst_target && dst_sg) {
			cnt++;
			if (dst_sg->length < dst_target)
				dst_target -= dst_sg->length;
			else
				dst_target = 0;
			dst_sg = sg_next(dst_sg);
		}
	}

	return cnt;
}

static int flexrm_spu_dma_map(struct device *dev, struct brcm_message *msg)
{
	int rc;

	rc = dma_map_sg(dev, msg->spu.src, sg_nents(msg->spu.src),
			DMA_TO_DEVICE);
	if (rc < 0)
		return rc;

	rc = dma_map_sg(dev, msg->spu.dst, sg_nents(msg->spu.dst),
			DMA_FROM_DEVICE);
	if (rc < 0) {
		dma_unmap_sg(dev, msg->spu.src, sg_nents(msg->spu.src),
			     DMA_TO_DEVICE);
		return rc;
	}

	return 0;
}

static void flexrm_spu_dma_unmap(struct device *dev, struct brcm_message *msg)
{
	dma_unmap_sg(dev, msg->spu.dst, sg_nents(msg->spu.dst),
		     DMA_FROM_DEVICE);
	dma_unmap_sg(dev, msg->spu.src, sg_nents(msg->spu.src),
		     DMA_TO_DEVICE);
}

static void *flexrm_spu_write_descs(struct brcm_message *msg, u32 nhcnt,
				     u32 reqid, void *desc_ptr, u32 toggle,
				     void *start_desc, void *end_desc)
{
	u64 d;
	u32 nhpos = 0;
	void *orig_desc_ptr = desc_ptr;
	unsigned int dst_target = 0;
	struct scatterlist *src_sg = msg->spu.src, *dst_sg = msg->spu.dst;

	while (src_sg || dst_sg) {
		if (src_sg) {
			if (sg_dma_len(src_sg) & 0xf)
				d = flexrm_src_desc(sg_dma_address(src_sg),
						     sg_dma_len(src_sg));
			else
				d = flexrm_msrc_desc(sg_dma_address(src_sg),
						      sg_dma_len(src_sg)/16);
			flexrm_enqueue_desc(nhpos, nhcnt, reqid,
					     d, &desc_ptr, &toggle,
					     start_desc, end_desc);
			nhpos++;
			dst_target = sg_dma_len(src_sg);
			src_sg = sg_next(src_sg);
		} else
			dst_target = UINT_MAX;

		while (dst_target && dst_sg) {
			if (sg_dma_len(dst_sg) & 0xf)
				d = flexrm_dst_desc(sg_dma_address(dst_sg),
						     sg_dma_len(dst_sg));
			else
				d = flexrm_mdst_desc(sg_dma_address(dst_sg),
						      sg_dma_len(dst_sg)/16);
			flexrm_enqueue_desc(nhpos, nhcnt, reqid,
					     d, &desc_ptr, &toggle,
					     start_desc, end_desc);
			nhpos++;
			if (sg_dma_len(dst_sg) < dst_target)
				dst_target -= sg_dma_len(dst_sg);
			else
				dst_target = 0;
			dst_sg = sg_next(dst_sg);
		}
	}

	/* Null descriptor with invalid toggle bit */
	flexrm_write_desc(desc_ptr, flexrm_null_desc(!toggle));

	/* Ensure that descriptors have been written to memory */
	wmb();

	/* Flip toggle bit in header */
	flexrm_flip_header_toogle(orig_desc_ptr);

	return desc_ptr;
}

static bool flexrm_sba_sanity_check(struct brcm_message *msg)
{
	u32 i;

	if (!msg->sba.cmds || !msg->sba.cmds_count)
		return false;

	for (i = 0; i < msg->sba.cmds_count; i++) {
		if (((msg->sba.cmds[i].flags & BRCM_SBA_CMD_TYPE_B) ||
		     (msg->sba.cmds[i].flags & BRCM_SBA_CMD_TYPE_C)) &&
		    (msg->sba.cmds[i].flags & BRCM_SBA_CMD_HAS_OUTPUT))
			return false;
		if ((msg->sba.cmds[i].flags & BRCM_SBA_CMD_TYPE_B) &&
		    (msg->sba.cmds[i].data_len > SRCT_LENGTH_MASK))
			return false;
		if ((msg->sba.cmds[i].flags & BRCM_SBA_CMD_TYPE_C) &&
		    (msg->sba.cmds[i].data_len > SRCT_LENGTH_MASK))
			return false;
		if ((msg->sba.cmds[i].flags & BRCM_SBA_CMD_HAS_RESP) &&
		    (msg->sba.cmds[i].resp_len > DSTT_LENGTH_MASK))
			return false;
		if ((msg->sba.cmds[i].flags & BRCM_SBA_CMD_HAS_OUTPUT) &&
		    (msg->sba.cmds[i].data_len > DSTT_LENGTH_MASK))
			return false;
	}

	return true;
}

static u32 flexrm_sba_estimate_nonheader_desc_count(struct brcm_message *msg)
{
	u32 i, cnt;

	cnt = 0;
	for (i = 0; i < msg->sba.cmds_count; i++) {
		cnt++;

		if ((msg->sba.cmds[i].flags & BRCM_SBA_CMD_TYPE_B) ||
		    (msg->sba.cmds[i].flags & BRCM_SBA_CMD_TYPE_C))
			cnt++;

		if (msg->sba.cmds[i].flags & BRCM_SBA_CMD_HAS_RESP)
			cnt++;

		if (msg->sba.cmds[i].flags & BRCM_SBA_CMD_HAS_OUTPUT)
			cnt++;
	}

	return cnt;
}

static void *flexrm_sba_write_descs(struct brcm_message *msg, u32 nhcnt,
				     u32 reqid, void *desc_ptr, u32 toggle,
				     void *start_desc, void *end_desc)
{
	u64 d;
	u32 i, nhpos = 0;
	struct brcm_sba_command *c;
	void *orig_desc_ptr = desc_ptr;

	/* Convert SBA commands into descriptors */
	for (i = 0; i < msg->sba.cmds_count; i++) {
		c = &msg->sba.cmds[i];

		if ((c->flags & BRCM_SBA_CMD_HAS_RESP) &&
		    (c->flags & BRCM_SBA_CMD_HAS_OUTPUT)) {
			/* Destination response descriptor */
			d = flexrm_dst_desc(c->resp, c->resp_len);
			flexrm_enqueue_desc(nhpos, nhcnt, reqid,
					     d, &desc_ptr, &toggle,
					     start_desc, end_desc);
			nhpos++;
		} else if (c->flags & BRCM_SBA_CMD_HAS_RESP) {
			/* Destination response with tlast descriptor */
			d = flexrm_dstt_desc(c->resp, c->resp_len);
			flexrm_enqueue_desc(nhpos, nhcnt, reqid,
					     d, &desc_ptr, &toggle,
					     start_desc, end_desc);
			nhpos++;
		}

		if (c->flags & BRCM_SBA_CMD_HAS_OUTPUT) {
			/* Destination with tlast descriptor */
			d = flexrm_dstt_desc(c->data, c->data_len);
			flexrm_enqueue_desc(nhpos, nhcnt, reqid,
					     d, &desc_ptr, &toggle,
					     start_desc, end_desc);
			nhpos++;
		}

		if (c->flags & BRCM_SBA_CMD_TYPE_B) {
			/* Command as immediate descriptor */
			d = flexrm_imm_desc(c->cmd);
			flexrm_enqueue_desc(nhpos, nhcnt, reqid,
					     d, &desc_ptr, &toggle,
					     start_desc, end_desc);
			nhpos++;
		} else {
			/* Command as immediate descriptor with tlast */
			d = flexrm_immt_desc(c->cmd);
			flexrm_enqueue_desc(nhpos, nhcnt, reqid,
					     d, &desc_ptr, &toggle,
					     start_desc, end_desc);
			nhpos++;
		}

		if ((c->flags & BRCM_SBA_CMD_TYPE_B) ||
		    (c->flags & BRCM_SBA_CMD_TYPE_C)) {
			/* Source with tlast descriptor */
			d = flexrm_srct_desc(c->data, c->data_len);
			flexrm_enqueue_desc(nhpos, nhcnt, reqid,
					     d, &desc_ptr, &toggle,
					     start_desc, end_desc);
			nhpos++;
		}
	}

	/* Null descriptor with invalid toggle bit */
	flexrm_write_desc(desc_ptr, flexrm_null_desc(!toggle));

	/* Ensure that descriptors have been written to memory */
	wmb();

	/* Flip toggle bit in header */
	flexrm_flip_header_toogle(orig_desc_ptr);

	return desc_ptr;
}

bool flexrm_sanity_check(struct brcm_message *msg)
{
	if (!msg)
		return false;

	switch (msg->type) {
	case BRCM_MESSAGE_SPU:
		return flexrm_spu_sanity_check(msg);
	case BRCM_MESSAGE_SBA:
		return flexrm_sba_sanity_check(msg);
	default:
		return false;
	};
}

u32 flexrm_estimate_nonheader_desc_count(struct brcm_message *msg)
{
	if (!msg)
		return 0;

	switch (msg->type) {
	case BRCM_MESSAGE_SPU:
		return flexrm_spu_estimate_nonheader_desc_count(msg);
	case BRCM_MESSAGE_SBA:
		return flexrm_sba_estimate_nonheader_desc_count(msg);
	default:
		return 0;
	};
}

int flexrm_dma_map(struct device *dev, struct brcm_message *msg)
{
	if (!dev || !msg)
		return -EINVAL;

	switch (msg->type) {
	case BRCM_MESSAGE_SPU:
		return flexrm_spu_dma_map(dev, msg);
	default:
		break;
	}

	return 0;
}

void flexrm_dma_unmap(struct device *dev, struct brcm_message *msg)
{
	if (!dev || !msg)
		return;

	switch (msg->type) {
	case BRCM_MESSAGE_SPU:
		flexrm_spu_dma_unmap(dev, msg);
		break;
	default:
		break;
	}
}

void *flexrm_write_descs(struct brcm_message *msg, u32 nhcnt,
			  u32 reqid, void *desc_ptr, u32 toggle,
			  void *start_desc, void *end_desc)
{
	if (!msg || !desc_ptr || !start_desc || !end_desc)
		return ERR_PTR(-ENOTSUPP);

	if ((desc_ptr < start_desc) || (end_desc <= desc_ptr))
		return ERR_PTR(-ERANGE);

	switch (msg->type) {
	case BRCM_MESSAGE_SPU:
		return flexrm_spu_write_descs(msg, nhcnt, reqid,
					       desc_ptr, toggle,
					       start_desc, end_desc);
	case BRCM_MESSAGE_SBA:
		return flexrm_sba_write_descs(msg, nhcnt, reqid,
					       desc_ptr, toggle,
					       start_desc, end_desc);
	default:
		return ERR_PTR(-ENOTSUPP);
	};
}
