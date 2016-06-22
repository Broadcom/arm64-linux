/*
 * snps_udc.c - Synopsys USB 2.0 Device Controller driver
 *
 * Copyright (C) 2016 Broadcom
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/extcon.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/phy/phy.h>
#include <linux/proc_fs.h>
#include <linux/types.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/version.h>
#include "snps_udc.h"

#define DRIVER_DESC "Driver for Synopsys Designware core UDC"

static void ep0_setup_init(struct snps_udc_ep *ep, int status)
{
	struct snps_udc *udc = ep->udc;

	ep->dma.virt->setup.status = DMA_STS_BUF_HOST_READY;
	ep->dirn = USB_DIR_OUT;
	ep->stopped = 0;

	if (!status) {
		clear_ep_nak(udc->regs, ep->num, USB_DIR_OUT);
		clear_ep_nak(udc->regs, ep->num, USB_DIR_IN);
	} else {
		enable_ep_stall(udc->regs, ep->num, USB_DIR_IN);
		enable_ep_stall(udc->regs, ep->num, USB_DIR_OUT);
	}

	enable_udc_ep_irq(udc->regs, ep->num, USB_DIR_OUT);
	enable_ep_dma(udc->regs, ep->num, USB_DIR_OUT);

	dev_dbg(udc->dev, "%s setup buffer initialized\n", ep->name);
}

static void ep_dma_init(struct snps_udc_ep *ep)
{
	struct snps_udc *udc = ep->udc;
	u32 desc_cnt = (DESC_CNT - 1);
	u32 i;

	ep->dma.virt = &ep->udc->dma.virt->ep[ep->num];
	ep->dma.phys = &ep->udc->dma.phys->ep[ep->num];

	ep->dma.virt->setup.status = DMA_STS_BUF_HOST_BUSY;
	set_setup_buf_ptr(udc->regs, ep->num, USB_DIR_OUT,
			  &ep->dma.phys->setup);

	for (i = 0; i < DESC_CNT; i++) {
		ep->dma.virt->desc[i].status = DMA_STS_BUF_HOST_BUSY;
		ep->dma.virt->desc[i].next_desc_addr =
				(dma_addr_t)&ep->dma.phys->desc[i + 1];
	}
	ep->dma.virt->desc[desc_cnt].next_desc_addr =
				(dma_addr_t)&ep->dma.phys->desc[0];

	set_data_desc_ptr(udc->regs, ep->num, USB_DIR_OUT,
			  &ep->dma.phys->desc[0]);
	set_data_desc_ptr(udc->regs, ep->num, USB_DIR_IN,
			  &ep->dma.phys->desc[0]);

	dev_dbg(udc->dev, " %s dma initialized\n", ep->name);
}

static void ep_data_dma_init(struct snps_udc_ep *ep)
{
	struct ep_xfer_req *ep_req;

	dev_dbg(ep->udc->dev, "enter: %s\n", __func__);

	ep_req = list_first_entry(&ep->queue, struct ep_xfer_req, queue);

	if (ep_req->dma_aligned) {
		ep_req->dma_addr_orig = ep_req->usb_req.dma;
		ep_req->usb_req.dma = ep->dma.aligned_addr;
		if (ep->dirn == USB_DIR_IN)
			memcpy(ep->dma.aligned_buf, ep_req->usb_req.buf,
			       ep_req->usb_req.length);
	}

	ep->dma.done = 0;
	ep->dma.len_done = 0;
	ep->dma.len_rem = ep->dma.usb_req->length;
	ep->dma.buf_addr = ep->dma.usb_req->dma;
	ep->dma.status = DMA_STS_RX_SUCCESS;

	if ((ep->dirn == USB_DIR_IN) &&
	    (ep->type != USB_ENDPOINT_XFER_ISOC)) {
		if (in_bf_mode)
			ep->dma.len_max = ep->dma.usb_req->length;
		else
			ep->dma.len_max = ep->usb_ep.maxpacket;
	} else {
		if (out_bf_mode)
			ep->dma.len_max = ep->dma.usb_req->length;
		else
			ep->dma.len_max = ep->usb_ep.maxpacket;
	}

	dma_desc_chain_reset(ep);
}

static void ep_data_dma_finish(struct snps_udc_ep *ep)
{
	struct snps_udc *udc = ep->udc;
	struct ep_xfer_req *ep_req;

	disable_udc_ep_irq(udc->regs, ep->num, ep->dirn);
	disable_ep_dma(udc->regs, ep->num, ep->dirn);

	ep_req = list_first_entry(&ep->queue, struct ep_xfer_req, queue);

	if (ep_req->dma_aligned) {
		if (ep->dirn == USB_DIR_OUT)
			memcpy(ep_req->usb_req.buf,
			       ep->dma.aligned_buf, ep_req->usb_req.length);
		ep_req->usb_req.dma = ep_req->dma_addr_orig;
	}
	dev_dbg(udc->dev, "%s dma finished\n", ep->name);
}

static void ep_data_dma_add(struct snps_udc_ep *ep)
{
	struct data_desc *desc = NULL;
	u32 status;
	u32 len;

	if (!ep->dma.len_rem)
		ep->dma.usb_req->zero = 1;

	ep->dma.last = ep->dma.usb_req->zero;

	while (!dma_desc_chain_is_full(ep) &&
	       (ep->dma.len_rem || ep->dma.usb_req->zero)) {
		desc = dma_desc_chain_alloc(ep);
		len = (ep->dma.len_rem < ep->dma.len_max) ?
			ep->dma.len_rem : ep->dma.len_max;
		ep->dma.len_rem -= len;
		status = 0;

		if (len <= ep->dma.len_max ||
		    (out_bf_mode && (len <= ep->dma.len_max))) {
			if (in_bf_mode ||
			    !((ep->dirn == USB_DIR_IN) &&
			      (ep->type == USB_ENDPOINT_XFER_BULK) &&
			      (len != 0) &&
			      (len % ep->usb_ep.maxpacket == 0)))
				ep->dma.usb_req->zero = 0;
		}

		if ((ep->dirn == USB_DIR_IN) &&
		    (ep->type == USB_ENDPOINT_XFER_ISOC)) {
			ep->dma.frame_num += ep->dma.frame_incr;
			dev_dbg(ep->udc->dev, "%s: DMA started: frame_num=%d.%d\n",
				ep->name, (ep->dma.frame_num >> 3),
				(ep->dma.frame_num & 0x7));
			status |= ((ep->dma.frame_num <<
				  DMA_STS_FRAME_NUM_SHIFT)
				  & DMA_STS_FRAME_NUM_MASK);
		}

		desc->buf_addr = ep->dma.buf_addr;
		status |= (len << DMA_STS_BYTE_CNT_SHIFT);
		desc->status = status | DMA_STS_BUF_HOST_READY;
		/* Ensure all writes are done before going for next descriptor*/
		wmb();
		ep->dma.buf_addr += len;

		if ((ep->dirn == USB_DIR_IN) &&
		    (ep->type == USB_ENDPOINT_XFER_ISOC))
			break;
	}

	if (desc)
		desc->status |= DMA_STS_LAST_DESC;

	dev_dbg(ep->udc->dev, "%s dma data added\n", ep->name);
}

static void ep_data_dma_remove(struct snps_udc_ep *ep)
{
	struct data_desc *desc;
	u32 status;
	u32 len = 0;

	while (!dma_desc_chain_is_empty(ep)) {
		desc = dma_desc_chain_head(ep);
		status = desc->status;
		desc->status = DMA_STS_BUF_HOST_BUSY;
		/* Ensure all writes are done before going for next descriptor*/
		wmb();
		len = (status & DMA_STS_NISO_BYTE_CNT_MASK) >>
			DMA_STS_NISO_BYTE_CNT_SHIFT;

		if ((ep->dirn == USB_DIR_IN) || (status &
					DMA_STS_LAST_DESC)) {
			ep->dma.len_done += len;
			ep->dma.usb_req->actual += len;
		}

		if ((status & DMA_STS_RX_MASK) != DMA_STS_RX_SUCCESS) {
			ep->dma.status = status & DMA_STS_RX_MASK;
			ep->dma.usb_req->status = -EIO;
			dev_warn(ep->udc->dev, "%s: DMA error\n", ep->name);
		}

		if ((ep->dirn == USB_DIR_IN) &&
		    (ep->type == USB_ENDPOINT_XFER_ISOC)) {
			if (ep->dma.usb_req->actual ==
					ep->dma.usb_req->length)
				ep->dma.usb_req->status = 0;
			dma_desc_chain_reset(ep);
		} else {
			dma_desc_chain_free(ep);
		}
	}

	if ((!ep->dma.len_rem || (len < ep->usb_ep.maxpacket)) &&
	    (ep->dma.usb_req->status == -EINPROGRESS))
		ep->dma.usb_req->status = 0;

	dev_dbg(ep->udc->dev, "%s dma data removed\n", ep->name);
}

static int fifo_ram_alloc(struct snps_udc_ep *ep, u32 max_pkt_size)
{
	u32 rx_cnt;
	u32 tx_cnt;

	switch (EP_DIRN_TYPE(ep->dirn, ep->type)) {
	case EP_DIRN_TYPE(USB_DIR_OUT, USB_ENDPOINT_XFER_BULK):
	case EP_DIRN_TYPE(USB_DIR_OUT, USB_ENDPOINT_XFER_INT):
	case EP_DIRN_TYPE(USB_DIR_OUT, USB_ENDPOINT_XFER_ISOC):
		rx_cnt = FIFO_SZ_U8(max_pkt_size);
		tx_cnt = 0;
		break;

	case EP_DIRN_TYPE(USB_DIR_IN, USB_ENDPOINT_XFER_BULK):
	case EP_DIRN_TYPE(USB_DIR_IN, USB_ENDPOINT_XFER_INT):
		rx_cnt = 0;
		tx_cnt = FIFO_SZ_U8(max_pkt_size);
		break;

	case EP_DIRN_TYPE(USB_DIR_IN, USB_ENDPOINT_XFER_ISOC):
		rx_cnt = 0;
		tx_cnt = 2 * FIFO_SZ_U8(max_pkt_size);
		break;

	case EP_DIRN_TYPE(USB_DIR_IN,  USB_ENDPOINT_XFER_CONTROL):
	case EP_DIRN_TYPE(USB_DIR_OUT, USB_ENDPOINT_XFER_CONTROL):
		rx_cnt = FIFO_SZ_U8(max_pkt_size);
		tx_cnt = rx_cnt;
		break;

	default:
		dev_err(ep->udc->dev, "%s: invalid EP attributes\n", ep->name);
		return -ENODEV;
	}

	dev_dbg(ep->udc->dev, "rx req=%u free=%u: tx req=%u free=%u\n",
		rx_cnt, ep->udc->rx_fifo_space, tx_cnt, ep->udc->tx_fifo_space);

	if ((ep->udc->rx_fifo_space < rx_cnt) ||
	    (ep->udc->tx_fifo_space < tx_cnt)) {
		dev_err(ep->udc->dev, "%s: fifo alloc failed\n", ep->name);
		return -ENOSPC;
	}

	ep->rx_fifo_size = rx_cnt;
	ep->tx_fifo_size = tx_cnt;

	if (mrx_fifo)
		ep->udc->rx_fifo_space -= rx_cnt;

	ep->udc->tx_fifo_space -= tx_cnt;

	return 0;
}

static void fifo_ram_free(struct snps_udc_ep *ep)
{
	if (mrx_fifo)
		ep->udc->rx_fifo_space += ep->rx_fifo_size;

	ep->udc->tx_fifo_space += ep->tx_fifo_size;

	ep->rx_fifo_size = 0;
	ep->tx_fifo_size = 0;
}

static int ep_cfg(struct snps_udc_ep *ep, u32 type,
		  u32 max_pkt_size)
{
	struct snps_udc *udc = ep->udc;

	ep->type = type;
	if (fifo_ram_alloc(ep, max_pkt_size) != 0)
		return -ENOSPC;

	ep->type = type;
	ep->usb_ep.maxpacket = max_pkt_size;

	if (ep->udc->conn_type)
		init_ep_reg(udc->regs, ep->num, ep->type, ep->dirn,
			    max_pkt_size);
	dev_dbg(udc->dev, "ep_cfg: %s: type=%u dirn=0x%x pkt=%u\n",
		ep->usb_ep.name, type, ep->dirn, max_pkt_size);

	return 0;
}

static void epreq_xfer_done(struct snps_udc_ep *ep,
			    struct ep_xfer_req *ep_req, int status)
{
	struct snps_udc *udc = ep->udc;
	u32 stopped;

	list_del_init(&ep_req->queue);

	if (ep_req->usb_req.status == -EINPROGRESS)
		ep_req->usb_req.status = status;

	if (ep_req->dma_aligned) {
		ep_req->dma_aligned = 0;
	} else if (ep_req->dma_mapped) {
		dma_unmap_single(ep->udc->gadget.dev.parent,
				 ep_req->usb_req.dma,
				 (ep_req->usb_req.length ?
				 ep_req->usb_req.length : 1),
				 (ep->dirn == USB_DIR_IN ? DMA_TO_DEVICE :
				 DMA_FROM_DEVICE));
		ep_req->dma_mapped = 0;
		ep_req->usb_req.dma = DMA_ADDR_INVALID;
	}

	dev_dbg(udc->dev, "%s xfer done req=0x%p buf=0x%p len=%d actual=%d\n",
		ep->name, &ep_req->usb_req, ep_req->usb_req.buf,
		ep_req->usb_req.length, ep_req->usb_req.actual);

	stopped = ep->stopped;
	ep->stopped = 1;
	spin_unlock(&ep->udc->lock);
	ep_req->usb_req.complete(&ep->usb_ep, &ep_req->usb_req);
	spin_lock(&ep->udc->lock);
	ep->stopped = stopped;
}

static void epreq_xfer_process(struct snps_udc_ep *ep)
{
	struct snps_udc *udc = ep->udc;
	struct ep_xfer_req *ep_req;

	dev_dbg(udc->dev, "%s: xfer request\n", ep->name);

	if (!ep->dma.usb_req) {
		dev_dbg(udc->dev, "%s: No dma usb request\n", ep->name);
		return;
	}

	disable_ep_dma(udc->regs, ep->num, ep->dirn);
	ep_data_dma_remove(ep);

	if (ep->dma.usb_req->status != -EINPROGRESS) {
		ep_data_dma_finish(ep);

		if ((ep->type == USB_ENDPOINT_XFER_CONTROL) &&
		    (ep->dirn == USB_DIR_IN) &&
		    (ep->dma.usb_req->status == 0)) {
			ep->dirn = USB_DIR_OUT;
			ep->b_ep_addr = ep->num | ep->dirn;
			ep->dma.usb_req->status = -EINPROGRESS;
			ep->dma.usb_req->actual = 0;
			ep->dma.usb_req->length = 0;
			ep_data_dma_init(ep);
		} else {
			if (in_bf_mode && is_ep_in() && is_ep_bulk() &&
			    (ep->dma.usb_req->length != 0) &&
			    (ep->dma.usb_req->length %
			    ep->usb_ep.maxpacket == 0) &&
			    (ep->dma.last)) {
				ep->dma.usb_req->status = -EINPROGRESS;
				ep->dma.usb_req->actual = 0;
				ep->dma.usb_req->length = 0;
			} else if (!list_empty(&ep->queue))
				epreq_xfer_done(ep,
						list_first_entry(&ep->queue,
								 struct
								 ep_xfer_req,
								 queue), 0);

			if (ep->type == USB_ENDPOINT_XFER_CONTROL)
				ep0_setup_init(ep, 0);

			if (is_ep_in() && is_ep_bulk() &&
			    !list_empty(&ep->queue)) {
				ep->in_xfer_done = true;
				clear_ep_nak(udc->regs, ep->num, ep->dirn);
				enable_udc_ep_irq(udc->regs, ep->num, ep->dirn);
				return;
			}

			if (list_empty(&ep->queue)) {
				ep->dma.usb_req = NULL;
			} else {
				ep_req = list_first_entry(&ep->queue,
							  struct ep_xfer_req,
							  queue);
				ep->dma.usb_req = &ep_req->usb_req;
				ep_data_dma_init(ep);
			}
		}
	}

	if (ep->dma.usb_req) {
		ep_data_dma_add(ep);
		enable_udc_ep_irq(udc->regs, ep->num, ep->dirn);
		clear_ep_nak(udc->regs, ep->num, ep->dirn);
		enable_ep_dma(udc->regs, ep->num, ep->dirn);
	}
}

static void epreq_xfer_error(struct snps_udc_ep *ep, int status)
{
	if (!ep->dma.usb_req) {
		dev_err(ep->udc->dev, "%s: No DMA usb request\n", ep->name);
		return;
	}

	ep->dma.usb_req->status = status;
	epreq_xfer_process(ep);
}

static void epreq_xfer_add(struct snps_udc_ep *ep,
			   struct ep_xfer_req *ep_req)
{
	struct snps_udc *udc = ep->udc;

	list_add_tail(&ep_req->queue, &ep->queue);
	if (ep->stopped)
		return;

	if ((ep->dirn == USB_DIR_IN) &&
	    (ep->type == USB_ENDPOINT_XFER_ISOC) &&
	    (ep->dma.usb_req) &&
	    (ep->dma.frame_num == FRAME_NUM_INVALID)) {
		ep_data_dma_finish(ep);
		ep->dma.usb_req = NULL;
		epreq_xfer_done(ep,
				list_first_entry(&ep->queue,
						 struct ep_xfer_req,
						 queue),
				-EREMOTEIO);
	}

	if (ep->dma.usb_req) {
		dev_dbg(udc->dev, "%s: busy\n", ep->name);
	} else if (!in_isoc_delay_disabled && (ep->dirn == USB_DIR_IN) &&
		   (ep->type == USB_ENDPOINT_XFER_ISOC) &&
		   (ep->dma.frame_num == FRAME_NUM_INVALID)) {
		dev_dbg(udc->dev, "%s: ISOC delay xfer start\n", ep->name);
		ep->dma.usb_req = &(list_first_entry(&ep->queue,
				struct ep_xfer_req, queue))->usb_req;
		ep_data_dma_init(ep);
		clear_ep_nak(udc->regs, ep->num, ep->dirn);
		enable_udc_ep_irq(udc->regs, ep->num, ep->dirn);

	} else {
		if (in_isoc_delay_disabled && (ep->dirn == USB_DIR_IN) &&
		    (ep->type == USB_ENDPOINT_XFER_ISOC) &&
		    (ep->dma.frame_num == FRAME_NUM_INVALID)) {
			ep->dma.frame_num = get_last_rx_frnum(udc->regs);
		}

		if (is_ep_in() && is_ep_bulk() && !ep->dma.usb_req) {
			ep->in_xfer_done = true;
			clear_ep_nak(udc->regs, ep->num, ep->dirn);
			enable_udc_ep_irq(udc->regs, ep->num, ep->dirn);
			return;
		}

		ep_req = list_first_entry(&ep->queue,
					  struct ep_xfer_req, queue);
		ep->dma.usb_req = &ep_req->usb_req;
		ep_data_dma_init(ep);
		ep_data_dma_add(ep);
		enable_udc_ep_irq(udc->regs, ep->num, ep->dirn);
		clear_ep_nak(udc->regs, ep->num, ep->dirn);
		enable_ep_dma(udc->regs, ep->num, ep->dirn);
	}

	dev_dbg(udc->dev, "%s: xfer add ep request\n", ep->name);
}

static void epreq_queue_flush(struct snps_udc_ep *ep, int status)
{
	struct snps_udc *udc = ep->udc;
	struct ep_xfer_req *ep_req;

	ep->stopped = 1;

	while (!list_empty(&ep->queue)) {
		ep_req = list_first_entry(&ep->queue,
					  struct ep_xfer_req, queue);
		epreq_xfer_done(ep, ep_req, status);
	}

	ep->dma.usb_req = NULL;
	if ((is_ep_in() && is_ep_bulk()) || !ep->num) {
		set_ep_fifo_flush(udc->regs, ep->num, ep->dirn);
		clear_ep_fifo_flush(udc->regs, ep->num, ep->dirn);
	}

	dev_dbg(udc->dev, "%s: EP queue flushed\n", ep->usb_ep.name);
}

static void ep0_setup_rx(struct snps_udc_ep *ep,
			 struct usb_ctrlrequest *setup)
{
	struct snps_udc *udc = ep->udc;
	int status;
	u32 val;
	u32 idx;
	u32 len;

	val = le16_to_cpu(setup->wValue);
	idx = le16_to_cpu(setup->wIndex);
	len = le16_to_cpu(setup->wLength);

	ep->dirn = setup->bRequestType & USB_ENDPOINT_DIR_MASK;

	dev_dbg(udc->dev, "%s: SETUP %02x.%02x v%04x i%04x l %04x\n",
		ep->name, setup->bRequestType, setup->bRequest,
		val, idx, len);

	if (ep->num != 0) {
		status = -EOPNOTSUPP;
	} else {
		spin_unlock(&udc->lock);
		status = udc->gadget_driver->setup(&udc->gadget, setup);
		spin_lock(&udc->lock);
	}

	if (status < 0)
		ep0_setup_init(ep, status);
	else if (len == 0)
		ep0_setup_init(ep, 0);
}

static void irq_ep_out_setup(struct snps_udc_ep *ep)
{
	struct setup_desc *desc = &ep->dma.virt->setup;
	u32 status = desc->status;

	dev_dbg(ep->udc->dev, "irq set up %s desc status: 0x%x\n",
		ep->name, status);

	if ((status & DMA_STS_BUF_MASK) != DMA_STS_BUF_DMA_DONE) {
		ep0_setup_init(ep, 0);
	} else if ((status & DMA_STS_RX_MASK) != DMA_STS_RX_SUCCESS) {
		ep0_setup_init(ep, 0);
	} else {
		desc->status = (status & ~DMA_STS_BUF_MASK)
					| DMA_STS_BUF_HOST_BUSY;
		ep0_setup_rx(ep, (struct usb_ctrlrequest *)&desc->data1);
	}
}

static void irq_process_epout(struct snps_udc_ep *ep)
{
	struct snps_udc *udc = ep->udc;
	u32 status;

	status = get_ep_status(udc->regs, ep->num, USB_DIR_OUT);
	clear_ep_status(udc->regs, ep->num, USB_DIR_OUT, status);

	status &= EP_STS_ALL;

	if (!status)
		return;

	if ((ep->dirn != USB_DIR_OUT) &&
	    (ep->type != USB_ENDPOINT_XFER_CONTROL)) {
		dev_err(udc->dev, "%s: unexpected interrupt\n", ep->name);
		return;
	}

	if (status & OUT_DMA_DATA_DONE) {
		status &= ~OUT_DMA_DATA_DONE;
		epreq_xfer_process(ep);
	}

	if (status & OUT_DMA_SETUP_DONE) {
		status &= ~OUT_DMA_SETUP_DONE;
		irq_ep_out_setup(ep);
	}

	if (status & DMA_BUF_NOT_AVAIL) {
		status &= ~DMA_BUF_NOT_AVAIL;
		dev_dbg(udc->dev, "%s: DMA BUF NOT AVAIL\n", ep->name);
		epreq_xfer_process(ep);
	}

	if (status & DMA_ERROR) {
		status &= ~DMA_ERROR;
		dev_err(udc->dev, "%s: DMA ERROR\n", ep->usb_ep.name);
		epreq_xfer_error(ep, -EIO);
	}

	if (status)
		dev_err(udc->dev, "%s: unknown status=0x%x\n",
			ep->name, status);
}

static void irq_process_epin(struct snps_udc_ep *ep)
{
	struct snps_udc *udc = ep->udc;
	struct ep_xfer_req *ep_req;
	u32 status;

	status = get_ep_status(udc->regs, ep->num, USB_DIR_IN);
	clear_ep_status(udc->regs, ep->num, USB_DIR_IN, status);

	if (!status)
		return;

	if (ep->dirn != USB_DIR_IN) {
		dev_err(udc->dev, "%s: unexpected OUT endpoint\n", ep->name);
		return;
	}

	if ((ep->type == USB_ENDPOINT_XFER_ISOC) &&
	    (status & (IN_XFER_DONE | DMA_BUF_NOT_AVAIL))) {
		dev_warn(ep->udc->dev, "%s: ISOC IN unexpected status=0x%x\n",
			 ep->name, status);
	}

	if (status & IN_TOKEN_RX) {
		status &= ~IN_TOKEN_RX;
		if (!ep->dma.usb_req && list_empty(&ep->queue))
			enable_ep_nak(udc->regs, ep->num, USB_DIR_IN);

		if (ep->type == USB_ENDPOINT_XFER_ISOC) {
			ep->dma.frame_num = get_frnum_last_rx(udc->regs);
			dev_dbg(udc->dev, "%s: ISOC IN\n", ep->name);
			if (ep->dma.usb_req) {
				ep->dma.usb_req->status = -EREMOTEIO;
				epreq_xfer_process(ep);
			}
		}
	}

	if (is_ep_bulk() && !list_empty(&ep->queue) &&
	    ep->in_xfer_done) {
		ep->in_xfer_done = false;
		ep_req = list_first_entry(&ep->queue,
					  struct ep_xfer_req, queue);
		ep->dma.usb_req = &ep_req->usb_req;

		ep_data_dma_init(ep);
		ep_data_dma_add(ep);
		clear_ep_nak(udc->regs, ep->num, ep->dirn);
		enable_udc_ep_irq(udc->regs, ep->num, ep->dirn);
		enable_ep_dma(udc->regs, ep->num, ep->dirn);
	}

	if (status & IN_DMA_DONE) {
		status &= ~IN_DMA_DONE;
		clear_ep_nak(udc->regs, ep->num, USB_DIR_IN);

		if (ep->type == USB_ENDPOINT_XFER_ISOC) {
			dev_dbg(udc->dev, "%s: ISOC IN\n", ep->usb_ep.name);
			epreq_xfer_process(ep);
		} else if (ep->dma.done & IN_XFER_DONE) {
			dev_dbg(udc->dev, "%s: late IN DMA done rec'd\n",
				ep->name);
			epreq_xfer_process(ep);
		} else {
			ep->dma.done = IN_DMA_DONE;
		}
	}

	if (status & IN_XFER_DONE) {
		status &= ~(IN_XFER_DONE);
		status &= ~(IN_FIFO_EMPTY);

		if (ep->dma.done & IN_DMA_DONE)
			epreq_xfer_process(ep);
		else
			ep->dma.done = IN_XFER_DONE;
	}

	status &= ~(IN_FIFO_EMPTY);

	if (status & DMA_BUF_NOT_AVAIL) {
		dev_err(udc->dev, "%s: DMA BUF NOT AVAIL\n", ep->name);
		status &= ~(DMA_BUF_NOT_AVAIL);
		epreq_xfer_process(ep);
	}

	if (status & DMA_ERROR) {
		status &= ~DMA_ERROR;
		dev_err(udc->dev, "%s: DMA ERROR\n", ep->name);
		epreq_xfer_error(ep, -EIO);
	}

	if (status)
		dev_err(udc->dev, "%s: unknown status=0x%x\n",
			ep->name, status);
}

static void ep_irq_process(struct snps_udc *udc, u32 irq_in, u32 irq_out)
{
	u32 mask = 1;
	u32 num;

	for (num = 0; num < UDC_MAX_EP; num++) {
		if (irq_in & mask)
			irq_process_epin(&udc->ep[num]);

		if (irq_out & mask)
			irq_process_epout(&udc->ep[num]);

		mask <<= 1;
	}
}

static void irq_process_set_intf(struct snps_udc *udc)
{
	struct usb_ctrlrequest setup;
	u32 ep_num;
	u16 intf;
	u16 alt;

	intf = (uint16_t)get_intf_num(udc->regs);
	alt =  (uint16_t)get_alt_num(udc->regs);

	setup.bRequestType = USB_DIR_OUT | USB_TYPE_STANDARD
					 | USB_RECIP_INTERFACE;
	setup.bRequest = USB_REQ_SET_INTERFACE;
	setup.wValue = cpu_to_le16(alt);
	setup.wIndex = cpu_to_le16(intf);
	setup.wLength = 0;

	for (ep_num = 0; ep_num < UDC_MAX_EP; ep_num++) {
		set_ep_alt_num(udc->regs, ep_num, alt);
		set_ep_intf_num(udc->regs, ep_num, intf);
	}
	dev_info(udc->dev, "SET INTF=%d ALT=%d\n", intf, alt);

	ep0_setup_rx(&udc->ep[0], &setup);
	set_setup_done(udc->regs);
}

static void irq_process_set_cfg(struct snps_udc *udc)
{
	struct usb_ctrlrequest setup;
	u32 ep_num;
	u16 cfg;

	cfg = (u16)get_cfg_num(udc->regs);

	setup.bRequestType = USB_DIR_OUT | USB_TYPE_STANDARD
					 | USB_RECIP_DEVICE;
	setup.bRequest = USB_REQ_SET_CONFIGURATION;
	setup.wValue = cpu_to_le16(cfg);
	setup.wIndex = 0;
	setup.wLength = 0;

	for (ep_num = 0; ep_num < UDC_MAX_EP; ep_num++)
		set_epcfg_reg(udc->regs, ep_num, cfg);

	dev_info(udc->dev, "SET CFG=%d\n", cfg);

	ep0_setup_rx(&udc->ep[0], &setup);
	set_setup_done(udc->regs);
}

static void irq_process_speed_enum(struct snps_udc *udc)
{
	u32 speed = udc->gadget.speed;

	switch (get_enum_speed(udc->regs)) {
	case SPEED_HIGH:
		dev_info(udc->dev, "HIGH SPEED\n");
		udc->gadget.speed = USB_SPEED_HIGH;
		break;
	case SPEED_FULL:
		dev_info(udc->dev, "FULL SPEED\n");
		udc->gadget.speed = USB_SPEED_FULL;
		break;
	case SPEED_LOW:
		dev_warn(udc->dev, "LOW SPEED not supported\n");
		udc->gadget.speed = USB_SPEED_LOW;
		break;
	default:
		dev_err(udc->dev, "Unknown SPEED = 0x%x\n",
			get_enum_speed(udc->regs));
		break;
	}

	if ((speed == USB_SPEED_UNKNOWN) &&
	    (udc->gadget.speed != USB_SPEED_UNKNOWN)) {
		ep0_setup_init(&udc->ep[0], 0);
		clear_devnak(udc->regs);
	}
}

static void irq_process_bus_idle(struct snps_udc *udc)
{
	int num;

	for (num = 0; num < UDC_MAX_EP; num++) {
		set_ep_fifo_flush(udc->regs, num, EP_DIRN_IN);
		clear_ep_fifo_flush(udc->regs, num, EP_DIRN_IN);
	}
}

static void dev_irq_process(struct snps_udc *udc, u32 irq)
{
	if (irq & IRQ_BUS_RESET)
		dev_info(udc->dev, "BUS RESET\n");

	if (irq & IRQ_BUS_SUSPEND)
		dev_dbg(udc->dev, "BUS SUSPEND\n");

	if (irq & IRQ_BUS_IDLE) {
		dev_dbg(udc->dev, "BUS IDLE\n");
		irq_process_bus_idle(udc);
	}

	if (irq & IRQ_SPEED_ENUM_DONE) {
		dev_dbg(udc->dev, "BUS speed enum done\n");
		irq_process_speed_enum(udc);
	}

	if (irq & IRQ_SET_CFG) {
		dev_dbg(udc->dev, "SET CFG\n");
		irq_process_set_cfg(udc);
	}

	if (irq & IRQ_SET_INTF) {
		dev_dbg(udc->dev, "SET INTF\n");
		irq_process_set_intf(udc);
	}
}

static irqreturn_t snps_udc_irq(int irq, void *dev)
{
	struct snps_udc *udc = (struct snps_udc *)dev;
	u32 devintr, epin_intr, epout_intr;
	unsigned long flags;

	spin_lock_irqsave(&udc->lock, flags);

	devintr = get_irq_active(udc->regs);
	epin_intr = get_ep_irq_active(udc->regs, USB_DIR_IN);
	epout_intr = get_ep_irq_active(udc->regs, USB_DIR_OUT);

	clear_udc_dev_irq(udc->regs, devintr);
	clear_udc_ep_irq_list(udc->regs, USB_DIR_IN, epin_intr);
	clear_udc_ep_irq_list(udc->regs, USB_DIR_OUT, epout_intr);

	if (!udc->gadget_driver) {
		spin_unlock_irqrestore(&udc->lock, flags);
		return IRQ_NONE;
	}

	/* SET_CFG and SET_INTF interrupts are handled last */
	dev_irq_process(udc, devintr & ~(IRQ_SET_CFG | IRQ_SET_INTF));
	ep_irq_process(udc, epin_intr, epout_intr);
	dev_irq_process(udc, devintr & (IRQ_SET_CFG | IRQ_SET_INTF));

	spin_unlock_irqrestore(&udc->lock, flags);
	dev_dbg(udc->dev, "UDC interrupts: Dev=0x%x EpIn=0x%x EpOut=0x%x\n",
		devintr, epin_intr, epout_intr);

	return IRQ_HANDLED;
}

static int snps_ep_enable(struct usb_ep *usb_ep,
			  const struct usb_endpoint_descriptor *desc)
{
	struct snps_udc_ep *ep;
	struct snps_udc *udc;
	unsigned long flags;
	u32 max_pkt_size;
	u32 xfertype;

	ep = container_of(usb_ep, struct snps_udc_ep, usb_ep);
	udc = ep->udc;

	if (!usb_ep || (ep->b_ep_addr != desc->bEndpointAddress)) {
		dev_err(udc->dev, "invalid endpoint (%p)\n", usb_ep);
		return -EINVAL;
	}

	if (!desc || (desc->bDescriptorType != USB_DT_ENDPOINT)) {
		dev_err(udc->dev, "ep%d: invalid descriptor=%p\n",
			ep->num, desc);
		return -EINVAL;
	}

	if (desc == ep->desc) {
		dev_err(udc->dev, "ep%d: already enabled\n", ep->num);
		return -EEXIST;
	}

	if (ep->desc) {
		dev_err(udc->dev, "ep%d:already enabled wth other descr\n",
			ep->num);
		return -EBUSY;
	}

	if (!udc->gadget_driver) {
		dev_warn(udc->dev, "%s: invalid device state\n", ep->name);
		return -ESHUTDOWN;
	}

	xfertype = desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK;
	max_pkt_size = le16_to_cpu(desc->wMaxPacketSize) & 0x7FF;

	if (!max_pkt_size || (max_pkt_size > ep->max_pkt_size)) {
		dev_err(udc->dev, "%s: invalid max pkt size\n", ep->name);
		return -ERANGE;
	}

	if ((ep->dirn == USB_DIR_IN) &&
	    (xfertype == USB_ENDPOINT_XFER_ISOC)) {
		if ((desc->bInterval < 1) || (desc->bInterval > 16)) {
			dev_err(udc->dev, "%s: invalid binterval\n", ep->name);
			return -ERANGE;
		}
		ep->dma.frame_num = FRAME_NUM_INVALID;
		ep->dma.frame_incr = 1 << (desc->bInterval - 1);
	}

	spin_lock_irqsave(&udc->lock, flags);

	if (ep_cfg(ep, xfertype, max_pkt_size) != 0) {
		spin_unlock_irqrestore(&udc->lock, flags);
		dev_err(udc->dev, "%s: not enough FIFO space\n", ep->name);
		return -ENOSPC;
	}

	set_epcfg_reg(udc->regs, ep->num, get_cfg_num(udc->regs));

	ep->desc = desc;
	ep->stopped = 0;
	ep->usb_ep.maxpacket = max_pkt_size;

	spin_unlock_irqrestore(&udc->lock, flags);

	dev_dbg(udc->dev, "%s: enabled: type: 0x%x, max_pkt_size: %d\n",
		ep->name, xfertype, max_pkt_size);

	return 0;
}

static int snps_ep_disable(struct usb_ep *usb_ep)
{
	struct snps_udc_ep *ep;
	struct snps_udc *udc;
	unsigned long flags;

	ep = container_of(usb_ep, struct snps_udc_ep, usb_ep);
	udc = ep->udc;

	if (!usb_ep || !ep->desc) {
		dev_err(udc->dev, "%s: invalid endpoint\n", ep->usb_ep.name);
		return -EINVAL;
	}

	spin_lock_irqsave(&udc->lock, flags);

	epreq_queue_flush(ep, -ESHUTDOWN);
	ep->desc = NULL;
	ep->usb_ep.maxpacket = ep->max_pkt_size;
	fifo_ram_free(ep);

	spin_unlock_irqrestore(&udc->lock, flags);

	return 0;
}

static struct usb_request *
snps_ep_alloc_request(struct usb_ep *usb_ep, gfp_t gfp_flags)
{
	struct ep_xfer_req *ep_req;

	if (!usb_ep)
		return NULL;

	ep_req = kzalloc(sizeof(*ep_req), gfp_flags);
	if (ep_req) {
		INIT_LIST_HEAD(&ep_req->queue);
		ep_req->usb_req.dma = DMA_ADDR_INVALID;
		pr_debug("%s: ep alloc req\n", usb_ep->name);
		return &ep_req->usb_req;
	}

	return NULL;
}

static void snps_ep_free_request(struct usb_ep *usb_ep,
				 struct usb_request *usb_req)
{
	struct ep_xfer_req *ep_req;

	ep_req = container_of(usb_req, struct ep_xfer_req, usb_req);

	if (usb_req) {
		pr_debug("%s: freed\n", usb_ep->name);
		kfree(ep_req);
	}
}

static int snps_ep_queue(struct usb_ep *usb_ep,
			 struct usb_request *usb_req, gfp_t gfp_flags)
{
	struct ep_xfer_req *ep_req;
	struct snps_udc_ep *ep;
	struct snps_udc *udc;
	unsigned long flags;

	ep = container_of(usb_ep, struct snps_udc_ep, usb_ep);
	ep_req = container_of(usb_req, struct ep_xfer_req, usb_req);

	dev_dbg(ep->udc->dev, "%s: %s\n", __func__, ep->usb_ep.name);
	if (!usb_ep || !usb_req || !ep_req->usb_req.complete ||
	    !ep_req->usb_req.buf || !list_empty(&ep_req->queue)) {
		dev_dbg(ep->udc->dev, "%s:invalid queue request\n", ep->name);
		return -EINVAL;
	}

	if (!ep->desc && (ep->num != 0)) {
		dev_err(ep->udc->dev, "%s: invalid EP state\n", ep->name);
		return -EFAULT;
	}

	if ((ep->type == USB_ENDPOINT_XFER_CONTROL) &&
	    !list_empty(&ep->queue)) {
		dev_err(ep->udc->dev, "%s: EP queue not empty\n", ep->name);
		return -EPERM;
	}

	if (usb_req->length > 0xffff) {
		dev_err(ep->udc->dev, "%s: request too big\n", ep->name);
		return -E2BIG;
	}

	if ((ep->type == USB_ENDPOINT_XFER_ISOC) &&
	    (ep->dirn == USB_DIR_IN) &&
	    (usb_req->length > ep->usb_ep.maxpacket)) {
		dev_err(ep->udc->dev, "%s: request > scheduled bandwidth, length=%u\n",
			ep->name, usb_req->length);
		return -EFBIG;
	}

	udc = ep->udc;
	if (!udc->gadget_driver) {
		dev_err(udc->dev, "%s: invalid device state\n", ep->name);
		return -ESHUTDOWN;
	}

	if (((unsigned long)ep_req->usb_req.buf) & 0x3UL) {
		dev_dbg(udc->dev, "%s: invalid buffer alignment: addr=0x%p\n",
			ep->usb_ep.name, ep_req->usb_req.buf);

		if ((ep->dma.aligned_buf) &&
		    (ep->dma.aligned_len < ep_req->usb_req.length)) {
			dma_free_coherent(NULL, ep->dma.aligned_len,
					  ep->dma.aligned_buf,
					  ep->dma.aligned_addr);
			ep->dma.aligned_buf = NULL;
		}

		if (!ep->dma.aligned_buf) {
			ep->dma.aligned_len = ep_req->usb_req.length;
			ep->dma.aligned_buf = dma_alloc_coherent(NULL,
				ep->dma.aligned_len, &ep->dma.aligned_addr,
				GFP_ATOMIC);
		}

		if (!ep->dma.aligned_buf) {
			dev_err(udc->dev, "%s: ep dma alloc failed\n",
				ep->name);
			return -ENOMEM;
		}

		ep_req->dma_aligned = 1;
	} else if ((ep_req->usb_req.dma == DMA_ADDR_INVALID) ||
		   (ep_req->usb_req.dma == 0)) {
		ep_req->dma_mapped = 1;
		ep_req->usb_req.dma = dma_map_single(
					ep->udc->gadget.dev.parent,
					ep_req->usb_req.buf,
					(ep_req->usb_req.length ?
					ep_req->usb_req.length : 1),
		(ep->dirn == USB_DIR_IN ? DMA_TO_DEVICE : DMA_FROM_DEVICE));
		if (dma_mapping_error(ep->udc->gadget.dev.parent,
				      ep_req->usb_req.dma)) {
			dev_err(ep->udc->gadget.dev.parent,
				"failed to map buffer\n");
			return -EFAULT;
		}
	}

	spin_lock_irqsave(&udc->lock, flags);

	ep_req->usb_req.status = -EINPROGRESS;
	ep_req->usb_req.actual = 0;

	if ((ep->type == USB_ENDPOINT_XFER_CONTROL) &&
	    (ep->dirn == USB_DIR_OUT) &&
	    (ep_req->usb_req.length == 0)) {
		epreq_xfer_done(ep, ep_req, 0);
	} else {
		if (ep_req->usb_req.length == 0)
			ep_req->usb_req.zero = 1;

		epreq_xfer_add(ep, ep_req);
	}

	spin_unlock_irqrestore(&udc->lock, flags);

	return 0;
}

static int snps_ep_dequeue(struct usb_ep *usb_ep,
			   struct usb_request *usb_req)
{
	struct ep_xfer_req *ep_req;
	struct snps_udc_ep *ep;
	unsigned long flags;

	ep = container_of(usb_ep, struct snps_udc_ep, usb_ep);
	ep_req = container_of(usb_req, struct ep_xfer_req, usb_req);

	if (!usb_ep || !usb_req) {
		dev_err(ep->udc->dev, "%s: invalid dequeue request\n",
			ep->name);
		return -EINVAL;
	}

	spin_lock_irqsave(&ep->udc->lock, flags);

	list_for_each_entry(ep_req, &ep->queue, queue) {
		if (&ep_req->usb_req == usb_req)
			break;
	}

	if (&ep_req->usb_req != usb_req) {
		spin_unlock_irqrestore(&ep->udc->lock, flags);
		dev_err(ep->udc->dev, "%s: request not queued\n", ep->name);
		return -ENOLINK;
	}

	epreq_xfer_done(ep, ep_req, -ECONNRESET);
	spin_unlock_irqrestore(&ep->udc->lock, flags);

	dev_dbg(ep->udc->dev, "%s: req=0x%p\n", ep->name, usb_req);
	return 0;
}

static int snps_ep_set_halt(struct usb_ep *usb_ep, int halt)
{
	struct snps_udc_ep *ep;
	unsigned long flags;
	struct snps_udc *udc;

	ep = container_of(usb_ep, struct snps_udc_ep, usb_ep);
	udc = ep->udc;
	if (!usb_ep) {
		dev_err(udc->dev, "%s: invalid halt request\n", ep->name);
		return -EINVAL;
	}

	if (ep->type == USB_ENDPOINT_XFER_ISOC) {
		dev_err(udc->dev, "%s: unsupported halt req\n", ep->name);
		return -EOPNOTSUPP;
	}

	if (halt && (ep->dirn == USB_DIR_IN) &&
	    !list_empty(&ep->queue)) {
		dev_err(udc->dev, "%s: EP IN queue not empty\n", ep->name);
		return -EAGAIN;
	}

	if (!halt && (ep->type == USB_ENDPOINT_XFER_CONTROL)) {
		dev_err(udc->dev, "%s: CTRL HALT clear\n", ep->name);
		return -EPROTO;
	}

	spin_lock_irqsave(&ep->udc->lock, flags);

	if (!halt) {
		disable_ep_stall(udc->regs, ep->num, ep->dirn);
	} else if (ep->type != USB_ENDPOINT_XFER_CONTROL) {
		enable_ep_stall(udc->regs, ep->num, ep->dirn);
	} else {
		enable_ep_stall(udc->regs, ep->num, USB_DIR_IN);
		enable_ep_stall(udc->regs, ep->num, USB_DIR_OUT);
	}

	spin_unlock_irqrestore(&ep->udc->lock, flags);

	dev_dbg(udc->dev, "%s: HALT %s done\n", ep->name,
		halt ? "SET" : "CLR");

	return 0;
}

static struct usb_ep_ops snps_ep_ops = {
	.enable		= snps_ep_enable,
	.disable	= snps_ep_disable,

	.alloc_request	= snps_ep_alloc_request,
	.free_request	= snps_ep_free_request,

	.queue		= snps_ep_queue,
	.dequeue	= snps_ep_dequeue,

	.set_halt	= snps_ep_set_halt,
};

static int eps_init(struct snps_udc *udc)
{
	struct snps_udc_ep *ep;
	int i, ret;

	/* Initialize Endpoint 0 */
	ep = &udc->ep[0];
	ep->udc = udc;
	ep->num = 0;
	ep->in_xfer_done = true;
	ep->dirn = USB_DIR_OUT;
	ep->b_ep_addr = ep->num | ep->dirn;
	strncpy(ep->name, "ep0", sizeof(ep->name));
	ep->usb_ep.name = ep->name;
	ep->max_pkt_size = EP_CTRL_MAX_PKT_SIZE;
	usb_ep_set_maxpacket_limit(&ep->usb_ep, EP_CTRL_MAX_PKT_SIZE);
	ep->usb_ep.ops = &snps_ep_ops;
	ep->stopped = 0;
	ep->usb_ep.caps.type_control = true;
	ep->usb_ep.caps.dir_in = true;
	ep->usb_ep.caps.dir_out = true;
	INIT_LIST_HEAD(&ep->queue);
	ep->type = USB_ENDPOINT_XFER_CONTROL;
	ep->usb_ep.maxpacket = EP_CTRL_MAX_PKT_SIZE;

	if (udc->conn_type)
		ep_dma_init(ep);

	dev_dbg(udc->dev, "%s: type: 0x%x, Dir:0x%x, Max Size: %d\n",
		ep->name, ep->type, ep->dirn, ep->max_pkt_size);

	/* Initialize remaining endpoints */
	for (i = 1; i < UDC_MAX_EP; i++) {
		ep = &udc->ep[i];
		ep->udc = udc;
		ep->max_pkt_size = EP_MAX_PKT_SIZE;
		usb_ep_set_maxpacket_limit(&ep->usb_ep, EP_MAX_PKT_SIZE);
		ep->usb_ep.ops = &snps_ep_ops;
		ep->in_xfer_done = true;
		ep->num = i;
		if (i % 2) {
			snprintf(ep->name, sizeof(ep->name), "ep%din", i);
			ep->dirn = EP_DIRN_IN;
			ep->usb_ep.caps.dir_in = true;
		} else {
			snprintf(ep->name, sizeof(ep->name), "ep%dout", i);
			ep->dirn = EP_DIRN_OUT;
			ep->usb_ep.caps.dir_out = true;
		}
		ep->usb_ep.name = ep->name;
		ep->b_ep_addr = ep->num | ep->dirn;

		ep->usb_ep.caps.type_iso = true;
		ep->usb_ep.caps.type_bulk = true;
		ep->usb_ep.caps.type_int = true;
		ep->stopped = 0;
		ep->usb_ep.maxpacket = EP_MAX_PKT_SIZE;

		INIT_LIST_HEAD(&ep->queue);
		if (udc->conn_type)
			ep_dma_init(ep);

		dev_dbg(udc->dev, "%s: type: 0x%x, Dir: 0x%x, Max Size: %d\n",
			ep->name, ep->type, ep->dirn, ep->max_pkt_size);
	}

	udc->rx_fifo_space = OUT_RX_FIFO_MEM_SIZE;
	udc->tx_fifo_space = IN_TX_FIFO_MEM_SIZE;
	ret = ep_cfg(&udc->ep[0], USB_ENDPOINT_XFER_CONTROL,
		     EP_CTRL_MAX_PKT_SIZE);
	if (ret) {
		dev_err(udc->dev, "Synopsys-UDC: error configuring endpoints\n");
		return ret;
	}

	dev_dbg(udc->dev, "Synopsys UDC Endpoints initialized\n");
	return 0;
}

static void start_udc(struct snps_udc *udc)
{
	int i;

	init_udc_reg(udc->regs);

	udc->rx_fifo_space = OUT_RX_FIFO_MEM_SIZE;
	udc->tx_fifo_space = IN_TX_FIFO_MEM_SIZE;

	eps_init(udc);
	enable_self_pwr(udc->regs);

	enable_udc_dev_irq(udc->regs, IRQ_SPEED_ENUM_DONE | IRQ_BUS_SUSPEND |
			   IRQ_BUS_IDLE | IRQ_BUS_RESET | IRQ_SET_INTF |
			   IRQ_SET_CFG);

	for (i = 0; i < UDC_MAX_EP; ++i) {
		if (udc->ep[i].usb_ep.name) {
			enable_udc_ep_irq(udc->regs,
					  udc->ep[i].num, USB_DIR_OUT);
			enable_udc_ep_irq(udc->regs,
					  udc->ep[i].num, USB_DIR_IN);
		}
	}

	clear_devnak(udc->regs);
	enable_ctrl_dma(udc->regs);
	bus_connect(udc->regs);

	dev_dbg(udc->dev, "Synopsys UDC started\n");
}

static void stop_udc(struct snps_udc *udc)
{
	finish_udc(udc->regs);

	udc->gadget.speed = USB_SPEED_UNKNOWN;
	epreq_queue_flush(&udc->ep[0], -ESHUTDOWN);
	udc->ep[0].desc = NULL;

	bus_disconnect(udc->regs);

	if (udc->gadget_driver && udc->gadget_driver->disconnect) {
		spin_unlock(&udc->lock);
		udc->gadget_driver->disconnect(&udc->gadget);
		spin_lock(&udc->lock);
	}

	dev_dbg(udc->dev, "Synopsys UDC stopped\n");
}

static int snps_gadget_pullup(struct usb_gadget *gadget, int is_on)
{
	struct snps_udc *udc;
	unsigned long flags;

	udc = container_of(gadget, struct snps_udc, gadget);

	spin_lock_irqsave(&udc->lock, flags);

	if (!udc->gadget_driver) {
		spin_unlock_irqrestore(&udc->lock, flags);
		return 0;
	}

	if (is_on && udc->pullup_on) {
		start_udc(udc);
		udc->ep[0].stopped = 0;
		dev_info(udc->dev, "Synopsys UDC device connected\n");
	} else if (!is_on && !udc->pullup_on) {
		stop_udc(udc);
		udc->ep[0].stopped = 1;
		dev_info(udc->dev, "Synopsys UDC device Disconnected\n");
	}

	spin_unlock_irqrestore(&udc->lock, flags);

	return 0;
}

static int snps_gadget_start(struct usb_gadget *gadget,
			     struct usb_gadget_driver *driver)
{
	struct snps_udc *udc;
	unsigned long flags;

	udc = container_of(gadget, struct snps_udc, gadget);

	if (udc->gadget_driver)
		return -EBUSY;

	spin_lock_irqsave(&udc->lock, flags);

	driver->driver.bus = NULL;
	udc->gadget_driver = driver;
	udc->gadget.dev.driver = &driver->driver;
	udc->ep[0].stopped = 0;

	spin_unlock_irqrestore(&udc->lock, flags);

	/* when cable is connected at boot time */
	if (udc->conn_type)
		schedule_delayed_work(&udc->drd_work, USBD_WQ_DELAY_MS);
	dev_dbg(udc->dev, "%s: Done\n", __func__);

	return 0;
}

static int snps_gadget_stop(struct usb_gadget *gadget)
{
	struct snps_udc_ep *ep;
	struct snps_udc *udc;
	unsigned long flags;

	udc = container_of(gadget, struct snps_udc, gadget);

	spin_lock_irqsave(&udc->lock, flags);
	stop_udc(udc);
	udc->gadget.dev.driver = NULL;
	udc->gadget_driver = NULL;

	list_for_each_entry(ep, &udc->gadget.ep_list, usb_ep.ep_list) {
		epreq_queue_flush(ep, -ESHUTDOWN);
		if (ep->desc)
			ep->desc = NULL;
	}

	spin_unlock_irqrestore(&udc->lock, flags);

	dev_dbg(udc->dev, "%s: Done\n", __func__);

	return 0;
}

static struct usb_gadget_ops snps_gadget_ops = {
	.pullup		= snps_gadget_pullup,
	.udc_start	= snps_gadget_start,
	.udc_stop	= snps_gadget_stop,
};

void snps_udc_drd_work(struct work_struct *work)
{
	struct snps_udc *udc;

	udc = container_of(to_delayed_work(work),
			   struct snps_udc, drd_work);

	if (udc->conn_type) {
		dev_dbg(udc->dev, "idle -> device\n");
		if (udc->gadget_driver) {
			udc->pullup_on = 1;
			snps_gadget_pullup(&udc->gadget, 1);
		}
	} else {
		dev_dbg(udc->dev, "device -> idle\n");
		udc->pullup_on = 0;
		snps_gadget_pullup(&udc->gadget, 0);
	}
}

static int usbd_connect_notify(struct notifier_block *self,
			       unsigned long event, void *ptr)
{
	struct snps_udc *udc = container_of(self, struct snps_udc, nb);

	dev_dbg(udc->dev, "%s: event: %lu\n", __func__, event);

	udc->conn_type = event;

	schedule_delayed_work(&udc->drd_work, USBD_WQ_DELAY_MS);

	return NOTIFY_OK;
}

static void free_udc_dma(struct platform_device *pdev, struct snps_udc *udc)
{
	u32 num;

	dma_free_coherent(&pdev->dev, sizeof(struct ep_desc_array),
			  udc->dma.virt, (dma_addr_t)udc->dma.phys);

	for (num = 0; num < UDC_MAX_EP; num++) {
		if (udc->ep[num].dma.aligned_buf) {
			dma_free_coherent(NULL, udc->ep[num].dma.aligned_len,
					  udc->ep[num].dma.aligned_buf,
					  udc->ep[num].dma.aligned_addr);
			udc->ep[num].dma.aligned_buf = NULL;
		}
	}
}

static int snps_udc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct snps_udc *udc;
	int i, ret;

	udc = devm_kzalloc(dev, sizeof(*udc), GFP_KERNEL);
	if (!udc)
		return -ENOMEM;

	spin_lock_init(&udc->lock);
	udc->dev = dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	udc->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(udc->regs))
		return PTR_ERR(udc->regs);

	udc->irq = irq_of_parse_and_map(dev->of_node, 0);
	if (udc->irq <= 0) {
		dev_err(dev, "Can't parse and map interrupt\n");
		return -EINVAL;
	}

	udc->udc_phy = devm_phy_get(dev, "usb2drd");
	if (IS_ERR(udc->udc_phy)) {
		dev_err(dev, "Failed to obtain phy from device tree\n");
		return PTR_ERR(udc->udc_phy);
	}

	ret = phy_init(udc->udc_phy);
	if (ret) {
		dev_err(dev, "UDC phy init failed");
		return ret;
	}

	ret = phy_power_on(udc->udc_phy);
	if (ret) {
		dev_err(dev, "UDC phy power on failed");
		phy_exit(udc->udc_phy);
		return ret;
	}

	udc->edev = extcon_get_edev_by_phandle(dev, 0);
	if (IS_ERR(udc->edev)) {
		if (PTR_ERR(udc->edev) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		dev_err(dev, "Invalid or missing extcon\n");
		ret = PTR_ERR(udc->edev);
		goto exit_phy;
	}

	udc->nb.notifier_call = usbd_connect_notify;
	ret = extcon_register_notifier(udc->edev, EXTCON_USB, &udc->nb);
	if (ret < 0) {
		dev_err(dev, "Can't register extcon device\n");
		goto exit_phy;
	}

	ret = extcon_get_cable_state_(udc->edev, EXTCON_USB);
	if (ret < 0) {
		dev_err(dev, "Can't get cable state\n");
		goto exit_extcon;
	} else if (ret) {
		udc->conn_type = ret;
	}

	udc->dma.virt = dma_alloc_coherent(&pdev->dev,
				sizeof(struct ep_desc_array),
				(dma_addr_t *)&udc->dma.phys,
				 GFP_KERNEL);
	if (!udc->dma.virt) {
		dev_err(dev, "Failed to allocate memory for ep\n");
		ret = -ENOMEM;
		goto exit_extcon;
	}

	INIT_DELAYED_WORK(&udc->drd_work, snps_udc_drd_work);

	ret = devm_request_irq(dev, udc->irq, snps_udc_irq, IRQF_SHARED,
			       "snps-udc", udc);
	if (ret < 0) {
		dev_err(dev, "Request irq %d failed for UDC\n", udc->irq);
		goto exit_dma;
	}

	/* Gagdet structure init */
	udc->gadget.name	= "snps-udc";
	udc->gadget.speed	= USB_SPEED_UNKNOWN;
	udc->gadget.max_speed	= USB_SPEED_HIGH;
	udc->gadget.ops		= &snps_gadget_ops;
	udc->gadget.ep0		= &udc->ep[0].usb_ep;
	INIT_LIST_HEAD(&udc->gadget.ep_list);

	eps_init(udc);
	for (i = 1; i < UDC_MAX_EP; i++) {
		list_add_tail(&udc->ep[i].usb_ep.ep_list,
			      &udc->gadget.ep_list);
	}

	ret = usb_add_gadget_udc(&pdev->dev, &udc->gadget);
	if (ret) {
		dev_err(dev, "Error adding gadget udc: %d\n", ret);
		goto exit_dma;
	}

	platform_set_drvdata(pdev, udc);
	dev_info(dev, "Synopsys UDC driver probe successful\n");

	return 0;
exit_dma:
	free_udc_dma(pdev, udc);
exit_extcon:
	extcon_unregister_notifier(udc->edev, EXTCON_USB, &udc->nb);
exit_phy:
	phy_power_off(udc->udc_phy);
	phy_exit(udc->udc_phy);

	return ret;
}

static int snps_udc_remove(struct platform_device *pdev)
{
	struct snps_udc *udc;

	udc = platform_get_drvdata(pdev);

	usb_del_gadget_udc(&udc->gadget);

	platform_set_drvdata(pdev, NULL);

	if (udc->drd_wq) {
		flush_workqueue(udc->drd_wq);
		destroy_workqueue(udc->drd_wq);
	}

	free_udc_dma(pdev, udc);
	phy_power_off(udc->udc_phy);
	phy_exit(udc->udc_phy);
	extcon_unregister_notifier(udc->edev, EXTCON_USB, &udc->nb);

	dev_info(&pdev->dev, "Synopsys UDC driver removed\n");

	return 0;
}

static void snps_udc_shutdown(struct platform_device *pdev)
{
	struct snps_udc *udc = platform_get_drvdata(pdev);

	snps_gadget_stop(&udc->gadget);
}

#ifdef CONFIG_PM_SLEEP
static int snps_udc_suspend(struct device *dev)
{
	struct snps_udc *udc;

	udc = dev_get_drvdata(dev);

	if (extcon_get_cable_state_(udc->edev, EXTCON_USB) > 0) {
		dev_dbg(udc->dev, "device -> idle\n");
		snps_gadget_pullup(&udc->gadget, 0);
	}
	phy_power_off(udc->udc_phy);
	phy_exit(udc->udc_phy);

	return 0;
}

static int snps_udc_resume(struct device *dev)
{
	struct snps_udc *udc;
	int ret;

	udc = dev_get_drvdata(dev);

	ret = phy_init(udc->udc_phy);
	if (ret) {
		dev_err(udc->dev, "UDC phy init failure");
		return ret;
	}

	ret = phy_power_on(udc->udc_phy);
	if (ret) {
		dev_err(udc->dev, "UDC phy power on failure");
		phy_exit(udc->udc_phy);
		return ret;
	}

	if (extcon_get_cable_state_(udc->edev, EXTCON_USB) > 0) {
		dev_dbg(udc->dev, "idle -> device\n");
		snps_gadget_pullup(&udc->gadget, 1);
	}

	return 0;
}

static const struct dev_pm_ops snps_udc_pm_ops = {
	.suspend	= snps_udc_suspend,
	.resume		= snps_udc_resume,
};
#endif

static const struct of_device_id of_udc_match[] = {
	{ .compatible = "snps,dw-ahb-udc", },
	{ }
};

MODULE_DEVICE_TABLE(of, of_udc_match);

static struct platform_driver snps_udc_driver = {
	.probe		= snps_udc_probe,
	.remove		= snps_udc_remove,
	.shutdown	= snps_udc_shutdown,
	.driver		= {
		.name	= "snps-udc",
		.of_match_table = of_match_ptr(of_udc_match),
#ifdef CONFIG_PM_SLEEP
		.pm	= &snps_udc_pm_ops,
#endif
	},
};

module_platform_driver(snps_udc_driver);

MODULE_ALIAS("platform:snps-udc");
MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL v2");
