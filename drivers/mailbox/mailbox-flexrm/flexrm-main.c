/* Broadcom FlexRM Mailbox Driver
 *
 * Copyright (C) 2016 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Each Broadcom FlexSparx4 offload engine is implemented as an
 * extension to Broadcom FlexRM ring manager. The FlexRM ring
 * manager provides a set of rings which can be used to submit
 * work to a FlexSparx4 offload engine.
 *
 * This driver creates a mailbox controller using a set of FlexRM
 * rings where each mailbox channel represents a separate FlexRM ring.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/err.h>
#include <linux/idr.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mailbox_controller.h>
#include <linux/mailbox_client.h>
#include <linux/mailbox/brcm-message.h>
#include <linux/module.h>
#include <linux/msi.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#include "flexrm-desc.h"

/* FlexRM configuration */
#define RING_REGS_SIZE					0x10000
#define RING_DESC_SIZE					8
#define RING_DESC_INDEX(offset)				\
			((offset) / RING_DESC_SIZE)
#define RING_DESC_OFFSET(index)				\
			((index) * RING_DESC_SIZE)
#define RING_MAX_REQ_COUNT				1024
#define RING_BD_ALIGN_ORDER				12
#define RING_BD_ALIGN_CHECK(addr)			\
			(!((addr) & ((0x1 << RING_BD_ALIGN_ORDER) - 1)))
#define RING_BD_TOGGLE_INVALID(offset)			\
			(((offset) >> RING_BD_ALIGN_ORDER) & 0x1)
#define RING_BD_TOGGLE_VALID(offset)			\
			(!RING_BD_TOGGLE_INVALID(offset))
#define RING_BD_DESC_PER_REQ				32
#define RING_BD_DESC_COUNT				\
			(RING_MAX_REQ_COUNT * RING_BD_DESC_PER_REQ)
#define RING_BD_SIZE					\
			(RING_BD_DESC_COUNT * RING_DESC_SIZE)
#define RING_CMPL_ALIGN_ORDER				13
#define RING_CMPL_DESC_COUNT				RING_MAX_REQ_COUNT
#define RING_CMPL_SIZE					\
			(RING_CMPL_DESC_COUNT * RING_DESC_SIZE)
#define RING_VER_MAGIC					0x76303031

/* Per-Ring register offsets */
#define RING_VER					0x000
#define RING_BD_START_ADDR				0x004
#define RING_BD_READ_PTR				0x008
#define RING_BD_WRITE_PTR				0x00c
#define RING_BD_READ_PTR_DDR_LS				0x010
#define RING_BD_READ_PTR_DDR_MS				0x014
#define RING_CMPL_START_ADDR				0x018
#define RING_CMPL_WRITE_PTR				0x01c
#define RING_NUM_REQ_RECV_LS				0x020
#define RING_NUM_REQ_RECV_MS				0x024
#define RING_NUM_REQ_TRANS_LS				0x028
#define RING_NUM_REQ_TRANS_MS				0x02c
#define RING_NUM_REQ_OUTSTAND				0x030
#define RING_CONTROL					0x034
#define RING_FLUSH_DONE					0x038
#define RING_MSI_ADDR_LS				0x03c
#define RING_MSI_ADDR_MS				0x040
#define RING_MSI_CONTROL				0x048
#define RING_BD_READ_PTR_DDR_CONTROL			0x04c
#define RING_MSI_DATA_VALUE				0x064

/* Register RING_BD_START_ADDR fields */
#define BD_LAST_UPDATE_HW_SHIFT				28
#define BD_LAST_UPDATE_HW_MASK				0x1
#define BD_START_ADDR_VALUE(pa)				\
	((u32)((((dma_addr_t)(pa)) >> RING_BD_ALIGN_ORDER) & 0x0fffffff))
#define BD_START_ADDR_DECODE(val)			\
	((dma_addr_t)((val) & 0x0fffffff) << RING_BD_ALIGN_ORDER)

/* Register RING_CMPL_START_ADDR fields */
#define CMPL_START_ADDR_VALUE(pa)			\
	((u32)((((u64)(pa)) >> RING_CMPL_ALIGN_ORDER) & 0x03ffffff))

/* Register RING_CONTROL fields */
#define CONTROL_MASK_DISABLE_CONTROL			12
#define CONTROL_FLUSH_SHIFT				5
#define CONTROL_ACTIVE_SHIFT				4
#define CONTROL_RATE_ADAPT_MASK				0xf
#define CONTROL_RATE_DYNAMIC				0x0
#define CONTROL_RATE_FAST				0x8
#define CONTROL_RATE_MEDIUM				0x9
#define CONTROL_RATE_SLOW				0xa
#define CONTROL_RATE_IDLE				0xb

/* Register RING_FLUSH_DONE fields */
#define FLUSH_DONE_MASK					0x1

/* Register RING_MSI_CONTROL fields */
#define MSI_TIMER_VAL_SHIFT				16
#define MSI_TIMER_VAL_MASK				0xffff
#define MSI_ENABLE_SHIFT				15
#define MSI_ENABLE_MASK					0x1
#define MSI_COUNT_SHIFT					0
#define MSI_COUNT_MASK					0x3ff

/* Register RING_BD_READ_PTR_DDR_CONTROL fields */
#define BD_READ_PTR_DDR_TIMER_VAL_SHIFT			16
#define BD_READ_PTR_DDR_TIMER_VAL_MASK			0xffff
#define BD_READ_PTR_DDR_ENABLE_SHIFT			15
#define BD_READ_PTR_DDR_ENABLE_MASK			0x1

struct flexrm_ring {
	/* Unprotected members */
	int num;
	struct flexrm_mbox *mbox;
	void __iomem *regs;
	bool irq_requested;
	unsigned int irq;
	unsigned int msi_timer_val;
	unsigned int msi_count_threshold;
	struct ida requests_ida;
	struct brcm_message *requests[RING_MAX_REQ_COUNT];
	void *bd_base;
	dma_addr_t bd_dma_base;
	u32 bd_write_offset;
	void *cmpl_base;
	dma_addr_t cmpl_dma_base;
	/* Protected members */
	spinlock_t lock;
	struct brcm_message *last_pending_msg;
	u32 cmpl_read_offset;
};

struct flexrm_mbox {
	struct device *dev;
	void __iomem *regs;
	u32 num_rings;
	struct flexrm_ring *rings;
	u64 dma_mask;
	struct dma_pool *bd_pool;
	struct dma_pool *cmpl_pool;
	struct mbox_controller controller;
};

static int flexrm_new_request(struct flexrm_ring *ring,
				struct brcm_message *batch_msg,
				struct brcm_message *msg)
{
	void *next;
	unsigned long flags;
	u32 val, count, nhcnt;
	u32 read_offset, write_offset;
	bool exit_cleanup = false;
	int ret = 0, reqid;

	/* Do sanity check on message */
	if (!flexrm_sanity_check(msg))
		return -EIO;
	msg->error = 0;

	/* If no requests possible then save data pointer and goto done. */
	reqid = ida_simple_get(&ring->requests_ida, 0,
				RING_MAX_REQ_COUNT, GFP_KERNEL);
	if (reqid < 0) {
		spin_lock_irqsave(&ring->lock, flags);
		if (batch_msg)
			ring->last_pending_msg = batch_msg;
		else
			ring->last_pending_msg = msg;
		spin_unlock_irqrestore(&ring->lock, flags);
		return 0;
	}
	ring->requests[reqid] = msg;

	/* Do DMA mappings for the message */
	ret = flexrm_dma_map(ring->mbox->dev, msg);
	if (ret < 0) {
		ring->requests[reqid] = NULL;
		ida_simple_remove(&ring->requests_ida, reqid);
		return ret;
	}

	/* If last_pending_msg is already set then goto done with error */
	spin_lock_irqsave(&ring->lock, flags);
	if (ring->last_pending_msg)
		ret = -ENOSPC;
	spin_unlock_irqrestore(&ring->lock, flags);
	if (ret < 0) {
		dev_warn(ring->mbox->dev, "no space in ring %d\n", ring->num);
		exit_cleanup = true;
		goto exit;
	}

	/* Determine current HW BD read offset */
	read_offset = readl_relaxed(ring->regs + RING_BD_READ_PTR);
	val = readl_relaxed(ring->regs + RING_BD_START_ADDR);
	read_offset *= RING_DESC_SIZE;
	read_offset += (u32)(BD_START_ADDR_DECODE(val) - ring->bd_dma_base);

	/*
	 * Number required descriptors = number of non-header descriptors +
	 *				 number of header descriptors +
	 *				 1x null descriptor
	 */
	nhcnt = flexrm_estimate_nonheader_desc_count(msg);
	count = flexrm_estimate_header_desc_count(nhcnt) + nhcnt + 1;

	/* Check for available descriptor space. */
	write_offset = ring->bd_write_offset;
	while (count) {
		if (!flexrm_is_next_table_desc(ring->bd_base + write_offset))
			count--;
		write_offset += RING_DESC_SIZE;
		if (write_offset == RING_BD_SIZE)
			write_offset = 0x0;
		if (write_offset == read_offset)
			break;
	}
	if (count) {
		spin_lock_irqsave(&ring->lock, flags);
		if (batch_msg)
			ring->last_pending_msg = batch_msg;
		else
			ring->last_pending_msg = msg;
		spin_unlock_irqrestore(&ring->lock, flags);
		ret = 0;
		exit_cleanup = true;
		goto exit;
	}

	/* Write descriptors to ring */
	next = flexrm_write_descs(msg, nhcnt, reqid,
			ring->bd_base + ring->bd_write_offset,
			RING_BD_TOGGLE_VALID(ring->bd_write_offset),
			ring->bd_base, ring->bd_base + RING_BD_SIZE);
	if (IS_ERR(next)) {
		ret = PTR_ERR(next);
		exit_cleanup = true;
		goto exit;
	}

	/* Save ring BD write offset */
	ring->bd_write_offset = (unsigned long)(next - ring->bd_base);

exit:
	/* Update error status in message */
	msg->error = ret;

	/* Cleanup if we failed */
	if (exit_cleanup) {
		flexrm_dma_unmap(ring->mbox->dev, msg);
		ring->requests[reqid] = NULL;
		ida_simple_remove(&ring->requests_ida, reqid);
	}

	return ret;
}

static int flexrm_process_completions(struct flexrm_ring *ring)
{
	u64 desc;
	int err, count = 0;
	unsigned long flags;
	struct brcm_message *msg = NULL;
	u32 reqid, cmpl_read_offset, cmpl_write_offset;
	struct mbox_chan *chan = &ring->mbox->controller.chans[ring->num];

	spin_lock_irqsave(&ring->lock, flags);

	/* Check last_pending_msg */
	if (ring->last_pending_msg) {
		msg = ring->last_pending_msg;
		ring->last_pending_msg = NULL;
	}

	/*
	 * Get current completion read and write offset
	 *
	 * Note: We should read completion write pointer atleast once
	 * after we get a MSI interrupt because HW maintains internal
	 * MSI status which will allow next MSI interrupt only after
	 * completion write pointer is read.
	 */
	cmpl_write_offset = readl_relaxed(ring->regs + RING_CMPL_WRITE_PTR);
	cmpl_write_offset *= RING_DESC_SIZE;
	cmpl_read_offset = ring->cmpl_read_offset;
	ring->cmpl_read_offset = cmpl_write_offset;

	spin_unlock_irqrestore(&ring->lock, flags);

	/* If last_pending_msg was set then queue it back */
	if (msg)
		mbox_send_message(chan, msg);

	/* For each completed request notify mailbox clients */
	reqid = 0;
	while (cmpl_read_offset != cmpl_write_offset) {
		/* Dequeue next completion descriptor */
		desc = *((u64 *)(ring->cmpl_base + cmpl_read_offset));

		/* Next read offset */
		cmpl_read_offset += RING_DESC_SIZE;
		if (cmpl_read_offset == RING_CMPL_SIZE)
			cmpl_read_offset = 0;

		/* Decode error from completion descriptor */
		err = flexrm_cmpl_desc_to_error(desc);
		if (err < 0) {
			dev_warn(ring->mbox->dev,
				 "got completion desc=0x%lx with error %d",
				 (unsigned long)desc, err);
		}

		/* Determine request id from completion descriptor */
		reqid = flexrm_cmpl_desc_to_reqid(desc);

		/* Determine message pointer based on reqid */
		msg = ring->requests[reqid];
		if (!msg) {
			dev_warn(ring->mbox->dev,
				 "null msg pointer for completion desc=0x%lx",
				 (unsigned long)desc);
			continue;
		}

		/* Release reqid for recycling */
		ring->requests[reqid] = NULL;
		ida_simple_remove(&ring->requests_ida, reqid);

		/* Unmap DMA mappings */
		flexrm_dma_unmap(ring->mbox->dev, msg);

		/* Give-back message to mailbox client */
		msg->error = err;
		mbox_chan_received_data(chan, msg);

		/* Increment number of completions processed */
		count++;
	}

	return count;
}

static irqreturn_t flexrm_irq_event(int irq, void *dev_id)
{
	/* We only have MSI for completions so just wakeup IRQ thread */
	/* Ring related errors will be informed via completion descriptors */

	return IRQ_WAKE_THREAD;
}

static irqreturn_t flexrm_irq_thread(int irq, void *dev_id)
{
	flexrm_process_completions(dev_id);

	return IRQ_HANDLED;
}

static int flexrm_send_data(struct mbox_chan *chan, void *data)
{
	int i, rc;
	struct flexrm_ring *ring = chan->con_priv;
	struct brcm_message *msg = data;

	if (msg->type == BRCM_MESSAGE_BATCH) {
		for (i = msg->batch.msgs_queued;
		     i < msg->batch.msgs_count; i++) {
			rc = flexrm_new_request(ring, msg,
						 &msg->batch.msgs[i]);
			if (rc) {
				msg->error = rc;
				return rc;
			}
			msg->batch.msgs_queued++;
		}
		return 0;
	}

	return flexrm_new_request(ring, NULL, data);
}

static bool flexrm_peek_data(struct mbox_chan *chan)
{
	int cnt = flexrm_process_completions(chan->con_priv);

	return (cnt > 0) ? true : false;
}

static int flexrm_startup(struct mbox_chan *chan)
{
	u64 d;
	u32 val, off;
	int ret = 0;
	dma_addr_t next_addr;
	struct flexrm_ring *ring = chan->con_priv;

	/* Allocate BD memory */
	ring->bd_base = dma_pool_alloc(ring->mbox->bd_pool,
				       GFP_KERNEL, &ring->bd_dma_base);
	if (!ring->bd_base) {
		dev_err(ring->mbox->dev, "can't allocate BD memory\n");
		ret = -ENOMEM;
		goto fail;
	}

	/* Configure next table pointer entries in BD memory */
	for (off = 0; off < RING_BD_SIZE; off += RING_DESC_SIZE) {
		next_addr = off + RING_DESC_SIZE;
		if (next_addr == RING_BD_SIZE)
			next_addr = 0;
		next_addr += ring->bd_dma_base;
		if (RING_BD_ALIGN_CHECK(next_addr))
			d = flexrm_next_table_desc(RING_BD_TOGGLE_VALID(off),
						    next_addr);
		else
			d = flexrm_null_desc(RING_BD_TOGGLE_INVALID(off));
		flexrm_write_desc(ring->bd_base + off, d);
	}

	/* Allocate completion memory */
	ring->cmpl_base = dma_pool_alloc(ring->mbox->cmpl_pool,
					 GFP_KERNEL, &ring->cmpl_dma_base);
	if (!ring->cmpl_base) {
		dev_err(ring->mbox->dev, "can't allocate completion memory\n");
		ret = -ENOMEM;
		goto fail_free_bd_memory;
	}
	memset(ring->cmpl_base, 0, RING_CMPL_SIZE);

	/* Request IRQ */
	if (ring->irq == UINT_MAX) {
		dev_err(ring->mbox->dev, "ring IRQ not available\n");
		ret = -ENODEV;
		goto fail_free_cmpl_memory;
	}
	ret = request_threaded_irq(ring->irq,
				   flexrm_irq_event,
				   flexrm_irq_thread,
				   0, dev_name(ring->mbox->dev), ring);
	if (ret) {
		dev_err(ring->mbox->dev, "failed to request ring IRQ\n");
		goto fail_free_cmpl_memory;
	}
	ring->irq_requested = true;

	/* Disable/inactivate ring */
	writel_relaxed(0x0, ring->regs + RING_CONTROL);

	/* Program BD start address */
	val = BD_START_ADDR_VALUE(ring->bd_dma_base);
	writel_relaxed(val, ring->regs + RING_BD_START_ADDR);

	/* BD write pointer will be same as HW write pointer */
	ring->bd_write_offset =
			readl_relaxed(ring->regs + RING_BD_WRITE_PTR);
	ring->bd_write_offset *= RING_DESC_SIZE;

	/* Program completion start address */
	val = CMPL_START_ADDR_VALUE(ring->cmpl_dma_base);
	writel_relaxed(val, ring->regs + RING_CMPL_START_ADDR);

	/* Ensure last pending message is cleared */
	ring->last_pending_msg = NULL;

	/* Completion read pointer will be same as HW write pointer */
	ring->cmpl_read_offset =
			readl_relaxed(ring->regs + RING_CMPL_WRITE_PTR);
	ring->cmpl_read_offset *= RING_DESC_SIZE;

	/* Read ring Tx, Rx, and Outstanding counts to clear */
	readl_relaxed(ring->regs + RING_NUM_REQ_RECV_LS);
	readl_relaxed(ring->regs + RING_NUM_REQ_RECV_MS);
	readl_relaxed(ring->regs + RING_NUM_REQ_TRANS_LS);
	readl_relaxed(ring->regs + RING_NUM_REQ_TRANS_MS);
	readl_relaxed(ring->regs + RING_NUM_REQ_OUTSTAND);

	/* Configure RING_MSI_CONTROL */
	val = 0;
	val |= (ring->msi_timer_val << MSI_TIMER_VAL_SHIFT);
	val |= BIT(MSI_ENABLE_SHIFT);
	val |= (ring->msi_count_threshold & MSI_COUNT_MASK) << MSI_COUNT_SHIFT;
	writel_relaxed(val, ring->regs + RING_MSI_CONTROL);

	/* Enable/activate ring */
	val = BIT(CONTROL_ACTIVE_SHIFT);
	writel_relaxed(val, ring->regs + RING_CONTROL);

	return 0;

fail_free_cmpl_memory:
	dma_pool_free(ring->mbox->cmpl_pool,
		      ring->cmpl_base, ring->cmpl_dma_base);
	ring->cmpl_base = NULL;
fail_free_bd_memory:
	dma_pool_free(ring->mbox->bd_pool,
		      ring->bd_base, ring->bd_dma_base);
	ring->bd_base = NULL;
fail:
	return ret;
}

static void flexrm_shutdown(struct mbox_chan *chan)
{
	u32 reqid;
	unsigned int timeout;
	struct brcm_message *msg;
	struct flexrm_ring *ring = chan->con_priv;

	/* Disable/inactivate ring */
	writel_relaxed(0x0, ring->regs + RING_CONTROL);

	/* Flush ring with timeout of 1s */
	timeout = 1000;
	writel_relaxed(BIT(CONTROL_FLUSH_SHIFT),
			ring->regs + RING_CONTROL);
	do {
		if (readl_relaxed(ring->regs + RING_FLUSH_DONE) &
		    FLUSH_DONE_MASK)
			break;
		mdelay(1);
	} while (timeout--);

	/* Abort all in-flight requests */
	for (reqid = 0; reqid < RING_MAX_REQ_COUNT; reqid++) {
		msg = ring->requests[reqid];
		if (!msg)
			continue;

		/* Release reqid for recycling */
		ring->requests[reqid] = NULL;
		ida_simple_remove(&ring->requests_ida, reqid);

		/* Unmap DMA mappings */
		flexrm_dma_unmap(ring->mbox->dev, msg);

		/* Give-back message to mailbox client */
		msg->error = -EIO;
		mbox_chan_received_data(chan, msg);
	}

	/* Release IRQ */
	if (ring->irq_requested) {
		free_irq(ring->irq, ring);
		ring->irq_requested = false;
	}

	/* Free-up completion descriptor ring */
	if (ring->cmpl_base) {
		dma_pool_free(ring->mbox->cmpl_pool,
			      ring->cmpl_base, ring->cmpl_dma_base);
		ring->cmpl_base = NULL;
	}

	/* Free-up BD descriptor ring */
	if (ring->bd_base) {
		dma_pool_free(ring->mbox->bd_pool,
			      ring->bd_base, ring->bd_dma_base);
		ring->bd_base = NULL;
	}
}

static bool flexrm_last_tx_done(struct mbox_chan *chan)
{
	bool ret;
	unsigned long flags;
	struct flexrm_ring *ring = chan->con_priv;

	spin_lock_irqsave(&ring->lock, flags);
	ret = (ring->last_pending_msg) ? false : true;
	spin_unlock_irqrestore(&ring->lock, flags);

	return ret;
}

static const struct mbox_chan_ops flexrm_mbox_chan_ops = {
	.send_data	= flexrm_send_data,
	.startup	= flexrm_startup,
	.shutdown	= flexrm_shutdown,
	.last_tx_done	= flexrm_last_tx_done,
	.peek_data	= flexrm_peek_data,
};

static void flexrm_mbox_msi_write(struct msi_desc *desc, struct msi_msg *msg)
{
	struct device *dev = msi_desc_to_dev(desc);
	struct flexrm_mbox *mbox = dev_get_drvdata(dev);
	struct flexrm_ring *ring = &mbox->rings[desc->platform.msi_index];

	/* Configure per-Ring MSI registers */
	writel_relaxed(msg->address_lo, ring->regs + RING_MSI_ADDR_LS);
	writel_relaxed(msg->address_hi, ring->regs + RING_MSI_ADDR_MS);
	writel_relaxed(msg->data, ring->regs + RING_MSI_DATA_VALUE);
}

static struct mbox_chan *flexrm_mbox_of_xlate(struct mbox_controller *cntlr,
					const struct of_phandle_args *pa)
{
	struct mbox_chan *chan;
	struct flexrm_ring *ring;

	if (pa->args_count < 3)
		return ERR_PTR(-EINVAL);

	if (pa->args[0] >= cntlr->num_chans)
		return ERR_PTR(-ENOENT);

	if (pa->args[1] > MSI_COUNT_MASK)
		return ERR_PTR(-EINVAL);

	if (pa->args[2] > MSI_TIMER_VAL_MASK)
		return ERR_PTR(-EINVAL);

	chan = &cntlr->chans[pa->args[0]];
	ring = chan->con_priv;
	ring->msi_count_threshold = pa->args[1];
	ring->msi_timer_val = pa->args[2];

	return chan;
}

static int flexrm_mbox_probe(struct platform_device *pdev)
{
	int index, ret = 0;
	void __iomem *regs;
	void __iomem *regs_end;
	struct msi_desc *desc;
	struct resource *iomem;
	struct flexrm_ring *ring;
	struct flexrm_mbox *mbox;
	struct device *dev = &pdev->dev;

	/* Allocate driver mailbox struct */
	mbox = devm_kzalloc(dev, sizeof(*mbox), GFP_KERNEL);
	if (!mbox) {
		ret = -ENOMEM;
		goto fail;
	}
	mbox->dev = dev;
	platform_set_drvdata(pdev, mbox);

	/* Get resource for registers */
	iomem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!iomem || (resource_size(iomem) < RING_REGS_SIZE)) {
		ret = -ENODEV;
		goto fail;
	}

	/* Map registers of all rings */
	mbox->regs = devm_ioremap_resource(&pdev->dev, iomem);
	if (IS_ERR(mbox->regs)) {
		ret = PTR_ERR(mbox->regs);
		dev_err(&pdev->dev, "Failed to remap mailbox regs: %d\n", ret);
		goto fail;
	}
	regs_end = mbox->regs + resource_size(iomem);

	/* Scan and count available rings */
	mbox->num_rings = 0;
	for (regs = mbox->regs; regs < regs_end; regs += RING_REGS_SIZE) {
		if (readl_relaxed(regs + RING_VER) == RING_VER_MAGIC)
			mbox->num_rings++;
	}
	if (!mbox->num_rings) {
		ret = -ENODEV;
		goto fail;
	}

	/* Allocate driver ring structs */
	ring = devm_kcalloc(dev, mbox->num_rings, sizeof(*ring), GFP_KERNEL);
	if (!ring) {
		ret = -ENOMEM;
		goto fail;
	}
	mbox->rings = ring;

	/* Initialize members of driver ring structs */
	regs = mbox->regs;
	for (index = 0; index < mbox->num_rings; index++) {
		ring = &mbox->rings[index];
		ring->num = index;
		ring->mbox = mbox;
		while ((regs < regs_end) &&
		       (readl_relaxed(regs + RING_VER) != RING_VER_MAGIC))
			regs += RING_REGS_SIZE;
		if (regs_end <= regs) {
			ret = -ENODEV;
			goto fail;
		}
		ring->regs = regs;
		regs += RING_REGS_SIZE;
		ring->irq = UINT_MAX;
		ring->irq_requested = false;
		ring->msi_timer_val = MSI_TIMER_VAL_MASK;
		ring->msi_count_threshold = 0x1;
		ida_init(&ring->requests_ida);
		memset(ring->requests, 0, sizeof(ring->requests));
		ring->bd_base = NULL;
		ring->bd_dma_base = 0;
		ring->cmpl_base = NULL;
		ring->cmpl_dma_base = 0;
		spin_lock_init(&ring->lock);
		ring->last_pending_msg = NULL;
		ring->cmpl_read_offset = 0;
	}

	/* FlexRM is capable of 40-bit physical addresses only */
	mbox->dma_mask = DMA_BIT_MASK(40);
	dev->dma_mask = &mbox->dma_mask;

	/* Create DMA pool for ring BD memory */
	mbox->bd_pool = dma_pool_create("bd", dev, RING_BD_SIZE,
					1 << RING_BD_ALIGN_ORDER, 0);
	if (!mbox->bd_pool) {
		ret = -ENOMEM;
		goto fail;
	}

	/* Create DMA pool for ring completion memory */
	mbox->cmpl_pool = dma_pool_create("cmpl", dev, RING_CMPL_SIZE,
					  1 << RING_CMPL_ALIGN_ORDER, 0);
	if (!mbox->cmpl_pool) {
		ret = -ENOMEM;
		goto fail_destroy_bd_pool;
	}

	/* Allocate platform MSIs for each ring */
	ret = platform_msi_domain_alloc_irqs(dev, mbox->num_rings,
						flexrm_mbox_msi_write);
	if (ret)
		goto fail_destroy_cmpl_pool;

	/* Save alloced IRQ numbers for each ring */
	for_each_msi_entry(desc, dev) {
		ring = &mbox->rings[desc->platform.msi_index];
		ring->irq = desc->irq;
	}

	/* Initialize mailbox controller */
	mbox->controller.txdone_irq = false;
	mbox->controller.txdone_poll = true;
	mbox->controller.txpoll_period = 1;
	mbox->controller.ops = &flexrm_mbox_chan_ops;
	mbox->controller.dev = dev;
	mbox->controller.num_chans = mbox->num_rings;
	mbox->controller.of_xlate = flexrm_mbox_of_xlate;
	mbox->controller.chans = devm_kcalloc(dev, mbox->num_rings,
				sizeof(*mbox->controller.chans), GFP_KERNEL);
	if (!mbox->controller.chans) {
		ret = -ENOMEM;
		goto fail_free_msis;
	}
	for (index = 0; index < mbox->num_rings; index++)
		mbox->controller.chans[index].con_priv = &mbox->rings[index];

	/* Register mailbox controller */
	ret = mbox_controller_register(&mbox->controller);
	if (ret)
		goto fail_free_msis;

	dev_info(dev, "registered flexrm mailbox with %d channels\n",
			mbox->controller.num_chans);

	return 0;

fail_free_msis:
	platform_msi_domain_free_irqs(dev);
fail_destroy_cmpl_pool:
	dma_pool_destroy(mbox->cmpl_pool);
fail_destroy_bd_pool:
	dma_pool_destroy(mbox->bd_pool);
fail:
	return ret;
}

static int flexrm_mbox_remove(struct platform_device *pdev)
{
	int index;
	struct device *dev = &pdev->dev;
	struct flexrm_ring *ring;
	struct flexrm_mbox *mbox = platform_get_drvdata(pdev);

	mbox_controller_unregister(&mbox->controller);

	platform_msi_domain_free_irqs(dev);

	dma_pool_destroy(mbox->cmpl_pool);
	dma_pool_destroy(mbox->bd_pool);

	for (index = 0; index < mbox->num_rings; index++) {
		ring = &mbox->rings[index];
		ida_destroy(&ring->requests_ida);
	}

	return 0;
}

static const struct of_device_id flexrm_mbox_of_match[] = {
	{ .compatible = "brcm,iproc-flexrm-mbox", },
	{},
};
MODULE_DEVICE_TABLE(of, flexrm_mbox_of_match);

static struct platform_driver flexrm_mbox_driver = {
	.driver = {
		.name = "brcm-flexrm-mbox",
		.of_match_table = flexrm_mbox_of_match,
	},
	.probe		= flexrm_mbox_probe,
	.remove		= flexrm_mbox_remove,
};
module_platform_driver(flexrm_mbox_driver);

MODULE_AUTHOR("Anup Patel <anup.patel@broadcom.com>");
MODULE_DESCRIPTION("Broadcom FlexRM mailbox driver");
MODULE_LICENSE("GPL v2");
