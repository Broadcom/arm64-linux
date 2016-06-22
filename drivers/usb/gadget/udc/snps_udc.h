/*
 * Copyright (C) 2016 Broadcom
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __SNPS_UDC_H
#define __SNPS_UDC_H

/* UDC speeds */
#define SPEED_UNKNOWN			(0)
#define SPEED_LOW			(1)
#define SPEED_FULL			(2)
#define SPEED_HIGH			(3)

/* Endpoint directions */
#define EP_DIRN_IN			(0x80)
#define EP_DIRN_OUT			(0x00)
#define EP_DIRN_MASK			(0x80)

/* Endpoint types */
#define EP_TYPE_CTRL			(0)
#define EP_TYPE_ISOC			(1)
#define EP_TYPE_BULK			(2)
#define EP_TYPE_INTR			(3)
#define EP_TYPE_MASK			(0x03)

/* Max supported endpoints */
#define UDC_MAX_EP			(10)

#define EP_MAX_PKT_SIZE			512
#define EP_CTRL_MAX_PKT_SIZE		64
#define OUT_RX_FIFO_MEM_SIZE		4096
#define IN_TX_FIFO_MEM_SIZE		4096

#define is_ep_in()			((ep->dirn) == USB_DIR_IN)
#define is_ep_out()			((ep->dirn) == USB_DIR_OUT)
#define is_ep_bulk()			((ep->type) == USB_ENDPOINT_XFER_BULK)

#define DESC_CNT (1)

#define EP_DMA_DESC_IDX_MASK		(DESC_CNT - 1)
#define EP_DMA_DESC_IDX(num)		((num) & EP_DMA_DESC_IDX_MASK)

#define USB_MODE_IDLE			(1)
#define USB_MODE_DEVICE			(2)

#define FIFO_SZ_U32(pkt_sz)		(((pkt_sz) + 3) / sizeof(u32))
#define FIFO_SZ_U8(sz)			(FIFO_SZ_U32(sz) * sizeof(u32))
#define USBD_WQ_DELAY_MS		msecs_to_jiffies(100)
/* Register Masks and definitions */

/* Endpoint Control Registers*/
#define EP_CTRL_OUT_FLUSH_ENABLE	BIT(12)
#define EP_CTRL_OUT_CLOSE_DESC		BIT(11)
#define EP_CTRL_IN_SEND_NULL		BIT(10)
#define EP_CTRL_OUT_DMA_ENABLE		BIT(9)
#define EP_CTRL_NAK_CLEAR		BIT(8)
#define EP_CTRL_NAK_SET			BIT(7)
#define EP_CTRL_NAK_IN_PROGRESS		BIT(6)
#define EP_CTRL_TYPE_SHIFT		(4)
#define EP_CTRL_TYPE_MASK		(3 << EP_CTRL_TYPE_SHIFT)
#define EP_CTRL_IN_DMA_ENABLE		BIT(3)
#define EP_CTRL_SNOOP_ENABLE		BIT(2)
#define EP_CTRL_IN_FLUSH_ENABLE		BIT(1)
#define EP_CTRL_STALL_ENABLE		BIT(0)

/* Endpoint Status Registers */
#define EP_STS_CLOSE_DESC_CLEAR		BIT(28)
#define EP_STS_IN_XFER_DONE		BIT(27)
#define EP_STS_STALL_SET_RX		BIT(26)
#define EP_STS_STALL_CLEAR_RX		BIT(25)
#define EP_STS_IN_FIFO_EMPTY		BIT(24)
#define EP_STS_IN_DMA_DONE		BIT(10)
#define EP_STS_AHB_BUS_ERROR		BIT(9)
#define EP_STS_OUT_FIFO_EMPTY		BIT(8)
#define EP_STS_DMA_BUF_NOT_AVAIL	BIT(7)
#define EP_STS_IN_TOKEN_RX		BIT(6)
#define EP_STS_OUT_DMA_SETUP_DONE	BIT(5)
#define EP_STS_OUT_DMA_DATA_DONE	BIT(4)

/* Buffer Regs for EP In, Receive Packet Frame Num Regs for EP Out */
#define EP_REG2_OUT_ISOC_PID_SHIFT	(16)
#define EP_REG2_OUT_ISOC_PID_MASK	(3 << EP_REG2_OUT_ISOC_PID_SHIFT)
#define EP_REG2_IN_DEPTH_SHIFT		(0)
#define EP_REG2_IN_DEPTH_MASK		(0xffff << EP_REG2_IN_DEPTH_SHIFT)
#define EP_REG2_OUT_FRAME_NUM_SHIFT	EP_REG2_IN_DEPTH_SHIFT
#define EP_REG2_OUT_FRAME_NUM_MASK	EP_REG2_IN_DEPTH_MASK

/* Max Packet Size Regs for EP In, Buffer Size Regs for EP Out */
#define EP_REG3_OUT_DEPTH_SHIFT		(16)
#define EP_REG3_OUT_DEPTH_MASK		(0xffff << EP_REG3_OUT_DEPTH_SHIFT)
#define EP_REG3_PKT_MAX_SHIFT		(0)
#define EP_REG3_PKT_MAX_MASK		(0xffff << EP_REG3_PKT_MAX_SHIFT)

/* Endpoint Config Registers */
#define EP_CFG_DIRN_IN			BIT(4)
#define EP_CFG_DIRN_OUT			(0)
#define EP_CFG_PKT_MAX_SHIFT		(19)
#define EP_CFG_PKT_MAX_MASK		(0x7ff << EP_CFG_PKT_MAX_SHIFT)
#define EP_CFG_ALT_NUM_SHIFT		(15)
#define EP_CFG_ALT_NUM_MASK		(0xf << EP_CFG_ALT_NUM_SHIFT)
#define EP_CFG_INTF_NUM_SHIFT		(11)
#define EP_CFG_INTF_NUM_MASK		(0xf << EP_CFG_INTF_NUM_SHIFT)
#define EP_CFG_CFG_NUM_SHIFT		(7)
#define EP_CFG_CFG_NUM_MASK		(0xf << EP_CFG_CFG_NUM_SHIFT)
#define EP_CFG_TYPE_SHIFT		(5)
#define EP_CFG_TYPE_MASK		(0x3 << EP_CFG_TYPE_SHIFT)
#define EP_CFG_FIFO_NUM_SHIFT		(0)
#define EP_CFG_FIFO_NUM_MASK		(0xf << EP_CFG_FIFO_NUM_SHIFT)

/* Endpoint Interrupt Registers */
#define EP_INTR_OUT_SHIFT		(16)
#define EP_INTR_OUT_MASK		(0xffff << EP_INTR_OUT_SHIFT)
#define EP_INTR_IN_SHIFT		(0)
#define EP_INTR_IN_MASK			(0xffff << EP_INTR_IN_SHIFT)

/* Device Config Register */
#define CFG_ULPI_DDR_ENABLE		BIT(19)
#define CFG_SET_DESCRIPTOR_ENABLE	BIT(18)
#define CFG_CSR_PROGRAM_ENABLE		BIT(17)
#define CFG_HALT_STALL_ENABLE		BIT(16)
#define CFG_HS_TIMEOUT_CALIB_SHIFT	(13)
#define CFG_HS_TIMEOUT_CALIB_MASK	(7 << CFG_HS_TIMEOUT_CALIB_SHIFT)
#define CFG_FS_TIMEOUT_CALIB_SHIFT	(10)
#define CFG_FS_TIMEOUT_CALIB_MASK	(7 << CFG_FS_TIMEOUT_CALIB_SHIFT)
#define CFG_STS_1_ENABLE		BIT(8)
#define CFG_STS_ENABLE			BIT(7)
#define CFG_UTMI_BI_DIRN_ENABLE		BIT(6)
#define CFG_UTMI_8BIT_ENABLE		BIT(5)
#define CFG_SYNC_FRAME_ENABLE		BIT(4)
#define CFG_SELF_PWR_ENABLE		BIT(3)
#define CFG_REMOTE_WAKEUP_ENABLE	BIT(2)
#define CFG_SPD_SHIFT			(0)
#define CFG_SPD_MASK			(3 << CFG_SPD_SHIFT)
#define CFG_SPD_HS			(0 << CFG_SPD_SHIFT)
#define CFG_SPD_FS			BIT(0)
#define CFG_SPD_LS			(2 << CFG_SPD_SHIFT)
#define CFG_SPD_FS_48MHZ		(3 << CFG_SPD_SHIFT)

/* Device Control Register*/
#define CTRL_DMA_OUT_THRESH_LEN_SHIFT	(24)
#define CTRL_DMA_OUT_THRESH_LEN_MASK	(0xff << CTRL_DMA_OUT_THRESH_LEN_SHIFT)
#define CTRL_DMA_BURST_LEN_SHIFT	(16)
#define CTRL_DMA_BURST_LEN_MASK		(0xff << CTRL_DMA_BURST_LEN_SHIFT)
#define CTRL_OUT_FIFO_FLUSH_ENABLE	BIT(14)
#define CTRL_CSR_DONE			BIT(13)
#define CTRL_OUT_ALL_NAK		BIT(12)
#define CTRL_DISCONNECT_ENABLE		BIT(10)
#define CTRL_DMA_MODE_ENABLE		BIT(9)
#define CTRL_DMA_BURST_ENABLE		BIT(8)
#define CTRL_DMA_OUT_THRESH_ENABLE	BIT(7)
#define CTRL_DMA_BUFF_FILL_MODE_ENABLE	BIT(6)
#define CTRL_ENDIAN_BIG_ENABLE		BIT(5)
#define CTRL_DMA_DESC_UPDATE_ENABLE	BIT(4)
#define CTRL_DMA_IN_ENABLE		BIT(3)
#define CTRL_DMA_OUT_ENABLE		BIT(2)
#define CTRL_RESUME_SIGNAL_ENABLE	BIT(0)
#define CTRL_LE_ENABLE			(0)

/* Device Status Register */
#define STS_SOF_FRAME_NUM_SHIFT		(18)
#define STS_SOF_FRAME_NUM_MASK		(0x3ffff << STS_SOF_FRAME_NUM_SHIFT)
#define STS_REMOTE_WAKEUP_ALLOWED	BIT(17)
#define STS_PHY_ERROR			BIT(16)
#define STS_OUT_FIFO_EMPTY		BIT(15)
#define STS_SPD_SHIFT			(13)
#define STS_SPD_MASK			(3 << STS_SPD_SHIFT)
#define STS_SPD_HS			(0 << STS_SPD_SHIFT)
#define STS_SPD_FS			BIT(13)
#define STS_SPD_LS			(2 << STS_SPD_SHIFT)
#define STS_SPD_FS_48MHZ		(3 << STS_SPD_SHIFT)
#define STS_BUS_SUSPENDED		BIT(12)
#define STS_ALT_NUM_SHIFT		(8)
#define STS_ALT_NUM_MASK		(0xf << STS_SPD_SHIFT)
#define STS_INTF_NUM_SHIFT		(4)
#define STS_INTF_NUM_MASK		(0xf << STS_INTF_NUM_SHIFT)
#define STS_CFG_NUM_SHIFT		(0)
#define STS_CFG_NUM_MASK		(0xf << STS_CFG_NUM_SHIFT)

/* Device Interrupt Register */
#define INTR_REMOTE_WAKEUP_DELTA	BIT(7)
#define INTR_SPD_ENUM_DONE		BIT(6)
#define INTR_SOF_RX			BIT(5)
#define INTR_BUS_SUSPEND		BIT(4)
#define INTR_BUS_RESET			BIT(3)
#define INTR_BUS_IDLE			BIT(2)
#define INTR_SET_INTF_RX		BIT(1)
#define INTR_SET_CFG_RX			BIT(0)

#define DMA_STS_BUF_SHIFT		(30)
#define DMA_STS_BUF_HOST_READY		(0 << DMA_STS_BUF_SHIFT)
#define DMA_STS_BUF_DMA_BUSY		BIT(30)
#define DMA_STS_BUF_DMA_DONE		(2 << DMA_STS_BUF_SHIFT)
#define DMA_STS_BUF_HOST_BUSY		(3 << DMA_STS_BUF_SHIFT)
#define DMA_STS_BUF_MASK		(3 << DMA_STS_BUF_SHIFT)
#define DMA_STS_RX_SHIFT		(28)
#define DMA_STS_RX_SUCCESS		(0 << DMA_STS_RX_SHIFT)
#define DMA_STS_RX_ERR_DESC		BIT(28)
#define DMA_STS_RX_ERR_BUF		(3 << DMA_STS_RX_SHIFT)
#define DMA_STS_RX_MASK			(3 << DMA_STS_RX_SHIFT)
#define DMA_STS_CFG_NUM_SHIFT		(24)
#define DMA_STS_CFG_NUM_MASK		(0xf << DMA_STS_CFG_NUM_SHIFT)
#define DMA_STS_INTF_NUM_SHIFT		(20)
#define DMA_STS_INTF_NUM_MASK		(0xf << DMA_STS_INTF_NUM_SHIFT)
#define DMA_STS_LAST_DESC		BIT(27)
#define DMA_STS_FRAME_NUM_SHIFT		(16)
#define DMA_STS_FRAME_NUM_MASK		(0x7ff << DMA_STS_FRAME_NUM_SHIFT)
#define DMA_STS_BYTE_CNT_SHIFT		(0)
#define DMA_STS_ISO_PID_SHIFT		(14)
#define DMA_STS_ISO_PID_MASK		(0x3 << DMA_STS_ISO_PID_SHIFT)
#define DMA_STS_ISO_BYTE_CNT_SHIFT	(DMA_STS_BYTE_CNT_SHIFT)
#define DMA_STS_ISO_BYTE_CNT_MASK	(0x3fff << DMA_STS_ISO_BYTE_CNT_SHIFT)
#define DMA_STS_NISO_BYTE_CNT_SHIFT	(DMA_STS_BYTE_CNT_SHIFT)
#define DMA_STS_NISO_BYTE_CNT_MASK	(0xffff << DMA_STS_NISO_BYTE_CNT_SHIFT)

/* UDC Interrupts */
#define UDC_IRQ_ALL			(IRQ_REMOTEWAKEUP_DELTA | \
					IRQ_SPEED_ENUM_DONE | \
					IRQ_BUS_SUSPEND | \
					IRQ_BUS_RESET | \
					IRQ_BUS_IDLE | \
					IRQ_SET_INTF | \
					IRQ_SET_CFG)
#define IRQ_REMOTEWAKEUP_DELTA		INTR_REMOTE_WAKEUP_DELTA
#define IRQ_SPEED_ENUM_DONE		INTR_SPD_ENUM_DONE
#define IRQ_SOF_DETECTED		INTR_SOF_RX
#define IRQ_BUS_SUSPEND			INTR_BUS_SUSPEND
#define IRQ_BUS_RESET			INTR_BUS_RESET
#define IRQ_BUS_IDLE			INTR_BUS_IDLE
#define IRQ_SET_INTF			INTR_SET_INTF_RX
#define IRQ_SET_CFG			INTR_SET_CFG_RX

/* Endpoint status */
#define EP_STS_ALL			(DMA_ERROR | \
					DMA_BUF_NOT_AVAIL | \
					IN_TOKEN_RX | \
					IN_DMA_DONE | \
					IN_XFER_DONE | \
					OUT_DMA_DATA_DONE | \
					OUT_DMA_SETUP_DONE)

#define DMA_ERROR			EP_STS_AHB_BUS_ERROR
#define DMA_BUF_NOT_AVAIL		EP_STS_DMA_BUF_NOT_AVAIL
#define IN_TOKEN_RX			EP_STS_IN_TOKEN_RX
#define IN_DMA_DONE			EP_STS_IN_DMA_DONE
#define IN_FIFO_EMPTY			EP_STS_IN_FIFO_EMPTY
#define IN_XFER_DONE			EP_STS_IN_XFER_DONE
#define OUT_DMA_DATA_DONE		EP_STS_OUT_DMA_DATA_DONE
#define OUT_DMA_SETUP_DONE		EP_STS_OUT_DMA_SETUP_DONE

#define DMA_ADDR_INVALID	(~(dma_addr_t)0)
#define DIRN_STR(dirn)		((dirn) == USB_DIR_IN ? "IN" : "OUT")
#define EP_DIRN_TYPE(d, t)	(((d) << 8) | (t))

/* Used for ISOC IN transfers for frame alignment. */
#define FRAME_NUM_INVALID	(~(u32)0)

/* UDC config parameters */

/* If multiple RX FIFO controllers are implemented for
 * OUT Endpoints, MRX_FIFO is enabled.
 * Multi RX FIFO controllers are not implemented in RTL.
 */
#define MRX_FIFO 0
#if MRX_FIFO
static bool mrx_fifo = true;
#else
static bool mrx_fifo;
#endif

/* Buffer Fill mode is enabled for IN transfers,
 * disabled for OUT transfers.
 */
#define IN_DMA_BUF_FILL_EN 1
#if IN_DMA_BUF_FILL_EN
static bool in_bf_mode = true;
#else
static bool in_bf_mode;
#endif

#define OUT_DMA_BUF_FILL_EN 0
#if OUT_DMA_BUF_FILL_EN
static bool out_bf_mode = true;
#else
static bool out_bf_mode;
#endif
/*
 * If it desired that frames start being DMA'd w/o frame
 * alignment, define ISOC_IN_XFER_DELAY_DISABLE.
 * If frame alignment is used, this delay is not disabled.
 */
#define ISOC_IN_XFER_DELAY_DISABLE 0
#if ISOC_IN_XFER_DELAY_DISABLE
static bool in_isoc_delay_disabled = true;
#else
static bool in_isoc_delay_disabled;
#endif

/* Endpoint IN/OUT registers
 * Register space is reserved for 16 endpoints, but the controller
 * actually supports 10 endpoints only.
 */
#define EP_CNT	(16)
struct snps_ep_regs {
	u32 ctrl;	/* EP control */
	u32 status;	/* EP status */
	u32 epreg2;	/* Buffer for IN, Rec Pkt Frame num for OUT */
	u32 epreg3;	/* Max pkt size for IN, Buf size for OUT */
	u32 setupbuf;	/* Rsvd for IN, EP setup buffer ptr for OUT */
	u32 datadesc;	/* EP data descriptor pointer */
	u32 rsvd[2];
};

/* UDC registers */
struct snps_udc_regs {
	struct snps_ep_regs ep_in[EP_CNT];
	struct snps_ep_regs ep_out[EP_CNT];
	u32 devcfg;
	u32 devctrl;
	u32 devstatus;
	u32 devintrstat;
	u32 devintrmask;
	u32 epintrstat;
	u32 epintrmask;
	u32 testmode;
	u32 releasenum;
	u32 rsvd[56];
	u32 epcfg[EP_CNT];
	u32 rsvd1[175];
	u32 rx_fifo[256];
	u32 tx_fifo[256];
	u32 strap;
};

/* Endpoint SETUP buffer */
struct setup_desc {
	u32 status;
	u32 reserved;
	u32 data1;
	u32 data2;
};

/* Endpoint In/Out data descriptor */
struct data_desc {
	u32 status;
	u32 reserved;
	u32 buf_addr;
	u32 next_desc_addr;
};

/* Endpoint descriptor layout. */
struct ep_dma_desc {
	struct setup_desc setup;
	struct data_desc  desc[DESC_CNT];
};

/* Endpoint descriptor array for Synopsys UDC */
struct ep_desc_array {
	struct ep_dma_desc ep[UDC_MAX_EP];
};

struct snps_udc;

/* Endpoint data structure (for each endpoint) */
struct snps_udc_ep {
	struct usb_ep usb_ep;
	const struct usb_endpoint_descriptor *desc;
	struct list_head queue;
	struct snps_udc *udc;
	char name[14];
	bool in_xfer_done;
	u32 num;
	u32 dirn;
	u32 type;			/* USB_ENDPOINT_XFER_xxx */
	u32 b_ep_addr;			/* dirn | type */
	u32 max_pkt_size;
	u32 rx_fifo_size;		/* Rx FIFO ram allocated */
	u32 tx_fifo_size;		/* Tx FIFO ram allocated */
	u32 stopped:1;
	struct {
		struct ep_dma_desc *virt;
		struct ep_dma_desc *phys;
		struct usb_request *usb_req;/* Current request being DMA'd */
		u32 len_max;		/* to use with a descriptor */
		u32 len_done;		/* Length of request DMA'd so far */
		u32 len_rem;		/* Length of request left to DMA */
		u32 add_idx;		/* descriptor chain index */
		u32 remove_idx;		/* descriptor chain index */
		u32 buf_addr;		/* Location in request to DMA */
		u32 frame_num;		/* Frame number for ISOC transfers */
		u32 frame_incr;		/* Frame number increment (period) */
		u32 status;
		u32 done;		/* DMA/USB xfer completion indication */
		void *aligned_buf;	/* used if usb_req buf not aligned */
		dma_addr_t aligned_addr;/* Aligned buffer physical address */
		u32 aligned_len;	/* Aligned buffer length */
		u32 last;
	} dma;
};

/* Endpoint xfer request structure */
struct ep_xfer_req {
	struct usb_request	usb_req;
	struct list_head	queue;
	dma_addr_t		dma_addr_orig;
	u32			dma_mapped:1;
	u32			dma_aligned:1;
};

/* Controller data structure */
struct snps_udc {
	struct usb_gadget		gadget;
	struct usb_gadget_driver	*gadget_driver;
	struct device			*dev;
	void __iomem			*regs;
	int				irq;
	struct completion		*dev_release;
	spinlock_t			lock; /* UDC spin lock variable */
	u32				rx_fifo_space;
	u32				tx_fifo_space;
	struct snps_udc_ep		ep[UDC_MAX_EP];
	struct {
		struct ep_desc_array	*virt;
		struct ep_desc_array	*phys;
	} dma;
	struct gpio_desc		*vbus_gpiod;
	u32				vbus_active:1;
	u32				pullup_on:1;
	struct phy			*udc_phy;
	u32				mode;
	struct extcon_dev		*edev;
	struct extcon_specific_cable_nb	extcon_nb;
	struct notifier_block		nb;
	struct delayed_work		drd_work;
	struct workqueue_struct		*drd_wq;
	u32				conn_type;
};

#define REG_WR(reg, val)		writel(val, &reg)
#define REG_MOD_AND(reg, val)		writel(val & readl(&reg), &reg)
#define REG_MOD_OR(reg, val)		writel(val | readl(&reg), &reg)
#define REG_MOD_MASK(reg, mask, val)	writel(val | (mask & readl(&reg)), &reg)
#define REG_RD(reg)			readl(&reg)

static inline void dump_regs(struct snps_udc_regs *regs)
{
	pr_debug("DEVCFG: 0x%x\n", REG_RD(regs->devcfg));
	pr_debug("DEVCTRL: 0x%x\n", REG_RD(regs->devctrl));
	pr_debug("DEVSTS: 0x%x\n", REG_RD(regs->devstatus));
	pr_debug("DEVINTRMASK: 0x%x\n", REG_RD(regs->devintrmask));
	pr_debug("DEVINTRSTS: 0x%x\n", REG_RD(regs->devintrstat));
	pr_debug("EPINTRMASK: 0x%x\n", REG_RD(regs->epintrmask));
	pr_debug("EPINTRSTS: 0x%x\n", REG_RD(regs->epintrstat));
}

static inline void bus_connect(struct snps_udc_regs *regs)
{
	REG_MOD_AND(regs->devctrl, ~CTRL_DISCONNECT_ENABLE);
}

static inline void bus_disconnect(struct snps_udc_regs *regs)
{
	REG_MOD_OR(regs->devctrl, CTRL_DISCONNECT_ENABLE);
}

static inline bool is_bus_suspend(struct snps_udc_regs *regs)
{
	return REG_RD(regs->devstatus) &
		STS_BUS_SUSPENDED ? true : false;
}

static inline u32 get_alt_num(struct snps_udc_regs *regs)
{
	return (REG_RD(regs->devstatus) & STS_ALT_NUM_MASK)
						>> STS_ALT_NUM_SHIFT;
}

static inline u32 get_cfg_num(struct snps_udc_regs *regs)
{
	return (REG_RD(regs->devstatus) & STS_CFG_NUM_MASK)
						>> STS_CFG_NUM_SHIFT;
}

static inline u32 get_intf_num(struct snps_udc_regs *regs)
{
	return (REG_RD(regs->devstatus) & STS_INTF_NUM_MASK)
						>> STS_INTF_NUM_SHIFT;
}

static inline void disable_ctrl_dma(struct snps_udc_regs *regs)
{
	REG_MOD_AND(regs->devctrl, ~(CTRL_DMA_IN_ENABLE |
					     CTRL_DMA_OUT_ENABLE));
}

static inline void enable_ctrl_dma(struct snps_udc_regs *regs)
{
		REG_MOD_OR(regs->devctrl, (CTRL_DMA_IN_ENABLE |
					   CTRL_DMA_OUT_ENABLE));
}

static inline bool is_ctrl_dma_enable(struct snps_udc_regs *regs)
{
	return REG_RD(regs->devctrl) &
				CTRL_DMA_OUT_ENABLE ? true : false;
}

static inline void disable_epin_dma(struct snps_udc_regs *regs)
{
	REG_MOD_AND(regs->devctrl, ~(CTRL_DMA_IN_ENABLE));
}

static inline void enable_epin_dma(struct snps_udc_regs *regs)
{
	REG_MOD_OR(regs->devctrl, (CTRL_DMA_IN_ENABLE));
}

static inline bool is_epin_dma_enable(struct snps_udc_regs *regs)
{
	return REG_RD(regs->devctrl) &
				CTRL_DMA_IN_ENABLE ? true : false;
}

static inline void disable_epout_dma(struct snps_udc_regs *regs)
{
	REG_MOD_AND(regs->devctrl, ~(CTRL_DMA_OUT_ENABLE));
}

static inline void enable_epout_dma(struct snps_udc_regs *regs)
{
	REG_MOD_OR(regs->devctrl, (CTRL_DMA_OUT_ENABLE));
}

static inline bool is_epout_dma_enable(struct snps_udc_regs *regs)
{
	return REG_RD(regs->devctrl) &
				CTRL_DMA_OUT_ENABLE ? true : false;
}

static inline u32 get_frnum_last_rx(struct snps_udc_regs *regs)
{
	return (REG_RD(regs->devstatus) &
		STS_SOF_FRAME_NUM_MASK) >> STS_SOF_FRAME_NUM_SHIFT;
}

static inline u32 get_irq_active(struct snps_udc_regs *regs)
{
	return REG_RD(regs->devintrstat);
}

static inline void clear_udc_dev_irq(struct snps_udc_regs *regs, u32 mask)
{
	REG_WR(regs->devintrstat, mask);
}

static inline void disable_udc_dev_irq(struct snps_udc_regs *regs, u32 mask)
{
	REG_MOD_OR(regs->devintrmask, mask);
}

static inline void enable_udc_dev_irq(struct snps_udc_regs *regs, u32 mask)
{
	REG_MOD_AND(regs->devintrmask, ~mask);
}

static inline u32 mask_irq(struct snps_udc_regs *regs)
{
	return (~REG_RD(regs->devintrmask)) & UDC_IRQ_ALL;
}

static inline void clear_devnak(struct snps_udc_regs *regs)
{
	REG_MOD_AND(regs->devctrl, ~CTRL_OUT_ALL_NAK);
}

static inline void set_devnak(struct snps_udc_regs *regs)
{
	REG_MOD_OR(regs->devctrl, CTRL_OUT_ALL_NAK);
}

static inline bool is_phy_error(struct snps_udc_regs *regs)
{
	return REG_RD(regs->devstatus) &
		STS_PHY_ERROR ? true : false;
}

static inline bool is_rmtwkp(struct snps_udc_regs *regs)
{
	return REG_RD(regs->devstatus) &
		STS_REMOTE_WAKEUP_ALLOWED ? true : false;
}

static inline void clear_rmtwkup(struct snps_udc_regs *regs)
{
	REG_MOD_AND(regs->devcfg, ~CFG_REMOTE_WAKEUP_ENABLE);
}

static inline void set_rmtwkp(struct snps_udc_regs *regs)
{
	REG_MOD_OR(regs->devcfg, CFG_REMOTE_WAKEUP_ENABLE);
}

static inline void start_rmtwkp(struct snps_udc_regs *regs)
{
	REG_MOD_OR(regs->devctrl, CTRL_RESUME_SIGNAL_ENABLE);
}

static inline void stop_rmtwkp(struct snps_udc_regs *regs)
{
	REG_MOD_AND(regs->devctrl, ~CTRL_RESUME_SIGNAL_ENABLE);
}

static inline void disable_self_pwr(struct snps_udc_regs *regs)
{
	REG_MOD_AND(regs->devcfg, ~CFG_SELF_PWR_ENABLE);
}

static inline void enable_self_pwr(struct snps_udc_regs *regs)
{
	REG_MOD_OR(regs->devcfg, CFG_SELF_PWR_ENABLE);
}

static inline void disable_set_desc(struct snps_udc_regs *regs)
{
	REG_MOD_AND(regs->devcfg, ~CFG_SET_DESCRIPTOR_ENABLE);
}

static inline void enable_set_desc(struct snps_udc_regs *regs)
{
	REG_MOD_OR(regs->devcfg, CFG_SET_DESCRIPTOR_ENABLE);
}

static inline void set_setup_done(struct snps_udc_regs *regs)
{
	REG_MOD_OR(regs->devctrl, CTRL_CSR_DONE);
}

static inline u32 get_enum_speed(struct snps_udc_regs *regs)
{
	switch (REG_RD(regs->devstatus) & STS_SPD_MASK) {
	case STS_SPD_LS:
		return SPEED_LOW;
	case STS_SPD_HS:
		return SPEED_HIGH;
	case STS_SPD_FS:
	case STS_SPD_FS_48MHZ:
		return SPEED_FULL;
	default:
		return 0;
	}
}

static inline void set_speed_requested(struct snps_udc_regs *regs, u32 speed)
{
	REG_MOD_AND(regs->devcfg, ~CFG_SPD_MASK);

	switch (speed) {
	case SPEED_LOW:
		REG_MOD_OR(regs->devcfg, CFG_SPD_LS);
		break;

	case SPEED_HIGH:
		REG_MOD_OR(regs->devcfg, CFG_SPD_HS);
		break;

	case SPEED_FULL:
	default:
		REG_MOD_OR(regs->devcfg, CFG_SPD_FS);
		break;
	}
}

static inline void init_ep_reg(struct snps_udc_regs *regs, u32 num, u32 type,
			       u32 dirn, u32 max_pkt_size)
{
	if ((type == EP_TYPE_CTRL) || (dirn == EP_DIRN_OUT)) {
		REG_WR(regs->ep_out[num].ctrl,
		       (type << EP_CTRL_TYPE_SHIFT));
		REG_WR(regs->ep_out[num].status,
		       regs->ep_out[num].status);
		REG_WR(regs->ep_out[num].epreg2,  0);
		REG_WR(regs->ep_out[num].epreg3,
		       ((max_pkt_size >> 2) << 16) | max_pkt_size);

		if (mrx_fifo)
			REG_MOD_OR(regs->ep_out[num].epreg3,
				   (FIFO_SZ_U32(max_pkt_size) <<
				   EP_REG3_OUT_DEPTH_SHIFT));
	}
	if ((type == EP_TYPE_CTRL) || (dirn == EP_DIRN_IN)) {
		REG_WR(regs->ep_in[num].ctrl,
		       (type << EP_CTRL_TYPE_SHIFT));
		REG_WR(regs->ep_in[num].epreg3,
		       (max_pkt_size << EP_REG3_PKT_MAX_SHIFT));
		REG_WR(regs->ep_in[num].epreg2,
		       (max_pkt_size >> 2));
		REG_MOD_OR(regs->ep_in[num].ctrl,
			   EP_CTRL_IN_FLUSH_ENABLE);
		REG_MOD_AND(regs->ep_in[num].ctrl,
			    ~EP_CTRL_IN_FLUSH_ENABLE);
		REG_MOD_AND(regs->ep_in[num].ctrl,
			    EP_CTRL_NAK_SET);
	}
	REG_WR(regs->epcfg[num],
	       (num << EP_CFG_FIFO_NUM_SHIFT) |
	       (type << EP_CFG_TYPE_SHIFT) |
	       (max_pkt_size << EP_CFG_PKT_MAX_SHIFT) |
	       ((dirn == EP_DIRN_OUT) ? EP_CFG_DIRN_OUT : EP_CFG_DIRN_IN));
}

static inline void set_ep_alt_num(struct snps_udc_regs *regs, u32 num, u32 alt)
{
	REG_MOD_MASK(regs->epcfg[num], ~EP_CFG_ALT_NUM_MASK,
		     (alt << EP_CFG_ALT_NUM_SHIFT));
}

static inline void set_epcfg_reg(struct snps_udc_regs *regs, u32 num, u32 cfg)
{
	REG_MOD_MASK(regs->epcfg[num], ~EP_CFG_CFG_NUM_MASK,
		     (cfg << EP_CFG_CFG_NUM_SHIFT));
}

static inline void set_ep_intf_num(struct snps_udc_regs *regs, u32 num,
				   u32 intf)
{
	REG_MOD_MASK(regs->epcfg[num], ~EP_CFG_INTF_NUM_MASK,
		     (intf << EP_CFG_INTF_NUM_SHIFT));
}

static inline void disable_ep_dma(struct snps_udc_regs *regs, u32 num, u32 dirn)
{
	if (dirn == EP_DIRN_OUT) {
		if (mrx_fifo)
			REG_MOD_AND(regs->ep_out[num].ctrl,
				    ~EP_CTRL_OUT_DMA_ENABLE);
	} else {
		REG_MOD_AND(regs->ep_in[num].ctrl,
			    ~EP_CTRL_IN_DMA_ENABLE);
	}
}

static inline void enable_ep_dma(struct snps_udc_regs *regs,
				 u32 num, u32 dirn)
{
	if (dirn == EP_DIRN_OUT) {
		if (mrx_fifo)
			REG_MOD_OR(regs->ep_out[num].ctrl,
				   EP_CTRL_OUT_DMA_ENABLE);
		else
			REG_MOD_OR(regs->devctrl,
				   CTRL_DMA_OUT_ENABLE);
	} else
		REG_MOD_OR(regs->ep_in[num].ctrl,
			   EP_CTRL_IN_DMA_ENABLE);
}

static inline void set_setup_buf_ptr(struct snps_udc_regs *regs,
				     u32 num, u32 dirn, void *addr)
{
	if (dirn == EP_DIRN_OUT)
		REG_WR(regs->ep_out[num].setupbuf, (dma_addr_t)addr);
}

static inline void set_data_desc_ptr(struct snps_udc_regs *regs,
				     u32 num, u32 dirn, void *addr)
{
	if (dirn == EP_DIRN_OUT)
		REG_WR(regs->ep_out[num].datadesc, (dma_addr_t)addr);
	else
		REG_WR(regs->ep_in[num].datadesc, (dma_addr_t)addr);
}

static inline bool is_ep_fifo_empty(struct snps_udc_regs *regs,
				    u32 num, u32 dirn)
{
	if (dirn == EP_DIRN_OUT) {
		if (mrx_fifo)
			return REG_RD(regs->ep_out[num].status) &
				EP_STS_OUT_FIFO_EMPTY ? true : false;
		else
			return REG_RD(regs->devstatus) &
				STS_OUT_FIFO_EMPTY ? true : false;
	}
	return REG_RD(regs->ep_in[num].status) &
			EP_STS_IN_FIFO_EMPTY ? true : false;
}

static inline void clear_ep_fifo_flush(struct snps_udc_regs *regs,
				       u32 num, u32 dirn)
{
	if (dirn == EP_DIRN_OUT) {
		if (mrx_fifo)
			REG_MOD_AND(regs->ep_out[num].ctrl,
				    ~EP_CTRL_OUT_FLUSH_ENABLE);
		else
			REG_MOD_AND(regs->devctrl,
				    ~CTRL_OUT_FIFO_FLUSH_ENABLE);
	} else {
		REG_MOD_AND(regs->ep_in[num].ctrl,
			    ~EP_CTRL_IN_FLUSH_ENABLE);
	}
}

static inline void set_ep_fifo_flush(struct snps_udc_regs *regs,
				     u32 num, u32 dirn)
{
	if (dirn == EP_DIRN_OUT) {
		if (mrx_fifo)
			REG_MOD_OR(regs->ep_out[num].ctrl,
				   EP_CTRL_OUT_FLUSH_ENABLE);
		else
			REG_MOD_OR(regs->devctrl,
				   CTRL_OUT_FIFO_FLUSH_ENABLE);
	} else {
		REG_MOD_OR(regs->ep_in[num].ctrl,
			   EP_CTRL_IN_FLUSH_ENABLE);
	}
}

static inline u32 get_ep_frnum(struct snps_udc_regs *regs,
			       u32 num, u32 dirn)
{
	if (dirn == EP_DIRN_OUT)
		return (regs->ep_out[num].epreg2 &
				EP_REG2_OUT_FRAME_NUM_MASK) >>
				EP_REG2_OUT_FRAME_NUM_SHIFT;
	return 0;
}

static inline void clear_udc_ep_irq(struct snps_udc_regs *regs,
				    u32 num, u32 dirn)
{
	if (dirn == EP_DIRN_OUT)
		REG_WR(regs->epintrstat, (1 << num) <<
		       EP_INTR_OUT_SHIFT);
	else
		REG_WR(regs->epintrstat, (1 << num) <<
		       EP_INTR_IN_SHIFT);
}

static inline void disable_udc_ep_irq(struct snps_udc_regs *regs,
				      u32 num, u32 dirn)
{
	if (dirn == EP_DIRN_OUT) {
		REG_MOD_OR(regs->epintrmask, ((1 << num) <<
			   EP_INTR_OUT_SHIFT));
	} else {
		REG_MOD_OR(regs->epintrmask, ((1 << num) <<
			   EP_INTR_IN_SHIFT));
	}
}

static inline void enable_udc_ep_irq(struct snps_udc_regs *regs,
				     u32 num, u32 dirn)
{
	if (dirn == EP_DIRN_OUT) {
		REG_MOD_AND(regs->epintrmask, ~((1 << num) <<
			    EP_INTR_OUT_SHIFT));
	} else {
		REG_MOD_AND(regs->epintrmask, ~((1 << num) <<
			    EP_INTR_IN_SHIFT));
	}
}

static inline u32 get_ep_irq_active(struct snps_udc_regs *regs, u32 dirn)
{
	if (dirn == EP_DIRN_OUT)
		return (REG_RD(regs->epintrstat) & EP_INTR_OUT_MASK)
				 >> EP_INTR_OUT_SHIFT;

	return (REG_RD(regs->epintrstat) & EP_INTR_IN_MASK)
				 >> EP_INTR_IN_SHIFT;
}

static inline void clear_udc_ep_irq_list(struct snps_udc_regs *regs,
					 u32 dirn, u32 mask)
{
	if (dirn == EP_DIRN_OUT)
		REG_WR(regs->epintrstat, (mask << EP_INTR_OUT_SHIFT));
	else
		REG_WR(regs->epintrstat, (mask << EP_INTR_IN_SHIFT));
}

static inline u32 get_ep_status(struct snps_udc_regs *regs, u32 num, u32 dirn)
{
	if (dirn == EP_DIRN_OUT)
		return REG_RD(regs->ep_out[num].status);

	return REG_RD(regs->ep_in[num].status);
}

static inline void clear_ep_status(struct snps_udc_regs *regs,
				   u32 num, u32 dirn, u32 mask)
{
	if (dirn == EP_DIRN_OUT)
		REG_WR(regs->ep_out[num].status, mask);
	else
		REG_WR(regs->ep_in[num].status, mask);
}

static inline void clear_ep_nak(struct snps_udc_regs *regs,
				u32 num, u32 dirn)
{
	if (dirn == EP_DIRN_OUT)
		REG_MOD_OR(regs->ep_out[num].ctrl, EP_CTRL_NAK_CLEAR);
	else
		REG_MOD_OR(regs->ep_in[num].ctrl, EP_CTRL_NAK_CLEAR);
}

static inline void enable_ep_nak(struct snps_udc_regs *regs,
				 u32 num, u32 dirn)
{
	if (dirn == EP_DIRN_OUT)
		REG_MOD_OR(regs->ep_out[num].ctrl, EP_CTRL_NAK_SET);
	else
		REG_MOD_OR(regs->ep_in[num].ctrl, EP_CTRL_NAK_SET);
}

static inline void disable_ep_nak(struct snps_udc_regs *regs,
				  u32 num, u32 dirn)
{
	if (dirn == EP_DIRN_OUT)
		REG_MOD_AND(regs->ep_out[num].ctrl, ~EP_CTRL_NAK_SET);
	else
		REG_MOD_AND(regs->ep_in[num].ctrl, ~EP_CTRL_NAK_SET);
}

static inline bool is_ep_nak_inprog(struct snps_udc_regs *regs,
				    u32 num, u32 dirn)
{
	if (dirn == EP_DIRN_OUT)
		return REG_RD(regs->ep_out[num].ctrl) &
			EP_CTRL_NAK_IN_PROGRESS ? true : false;

	return REG_RD(regs->ep_in[num].ctrl) &
			EP_CTRL_NAK_IN_PROGRESS ? true : false;
}

static inline void disable_ep_stall(struct snps_udc_regs *regs,
				    u32 num, u32 dirn)
{
	if (dirn == EP_DIRN_OUT)
		REG_MOD_AND(regs->ep_out[num].ctrl,
			    ~EP_CTRL_STALL_ENABLE);
	else
		REG_MOD_AND(regs->ep_in[num].ctrl,
			    ~EP_CTRL_STALL_ENABLE);
}

static inline void enable_ep_stall(struct snps_udc_regs *regs,
				   u32 num, u32 dirn)
{
	if (mrx_fifo && !(REG_RD(regs->ep_out[num].status) &
			EP_STS_OUT_FIFO_EMPTY))
		return;
	else if (!mrx_fifo && !(REG_RD(regs->devstatus) &
			STS_OUT_FIFO_EMPTY))
		return;

	if (dirn == EP_DIRN_OUT)
		REG_MOD_OR(regs->ep_out[num].ctrl,
			   EP_CTRL_STALL_ENABLE);
	else
		REG_MOD_OR(regs->ep_in[num].ctrl,
			   EP_CTRL_STALL_ENABLE);
}

static inline u32 get_last_rx_frnum(struct snps_udc_regs *regs)
{
	return (REG_RD(regs->devstatus) & STS_SOF_FRAME_NUM_MASK)
			>> STS_SOF_FRAME_NUM_SHIFT;
}

static inline void finish_udc(struct snps_udc_regs *regs)
{
	u32 ep_num;

	disable_ctrl_dma(regs);
	disable_udc_dev_irq(regs, UDC_IRQ_ALL);
	clear_udc_dev_irq(regs, UDC_IRQ_ALL);

	for (ep_num = 0; ep_num < UDC_MAX_EP; ep_num++) {
		disable_udc_ep_irq(regs, ep_num, EP_DIRN_IN);
		clear_udc_ep_irq(regs, ep_num, EP_DIRN_IN);
		clear_ep_status(regs, ep_num, EP_DIRN_IN,
				get_ep_status(regs, ep_num,
					      EP_DIRN_IN));

		disable_udc_ep_irq(regs, ep_num, EP_DIRN_OUT);
		clear_udc_ep_irq(regs, ep_num, EP_DIRN_OUT);
		clear_ep_status(regs, ep_num, EP_DIRN_OUT,
				get_ep_status(regs, ep_num,
					      EP_DIRN_OUT));
	}
}

static inline void init_udc_reg(struct snps_udc_regs *regs)
{
	finish_udc(regs);
	REG_WR(regs->devcfg, CFG_SET_DESCRIPTOR_ENABLE
					| CFG_UTMI_8BIT_ENABLE
					| CFG_CSR_PROGRAM_ENABLE
					| CFG_SPD_HS);
	REG_WR(regs->devctrl, CTRL_LE_ENABLE
					| CTRL_DISCONNECT_ENABLE
					| CTRL_DMA_MODE_ENABLE
					| CTRL_DMA_DESC_UPDATE_ENABLE
					| CTRL_OUT_ALL_NAK
					| CTRL_DMA_OUT_THRESH_LEN_MASK
					| CTRL_DMA_BURST_LEN_MASK
					| CTRL_DMA_BURST_ENABLE
					| CTRL_OUT_FIFO_FLUSH_ENABLE
			);

	if (mrx_fifo)
		REG_MOD_AND(regs->devctrl, ~CTRL_OUT_FIFO_FLUSH_ENABLE);

	if (out_bf_mode)
		REG_MOD_OR(regs->devctrl, CTRL_DMA_BUFF_FILL_MODE_ENABLE);

	REG_WR(regs->devintrmask, IRQ_BUS_IDLE | IRQ_SOF_DETECTED);
	REG_WR(regs->epintrmask, 0);
}

static inline struct data_desc *dma_desc_chain_alloc(struct snps_udc_ep *ep)
{
	u32 idx;

	idx = ep->dma.add_idx++;

	return &ep->dma.virt->desc[EP_DMA_DESC_IDX(idx)];
}

static inline int dma_desc_chain_is_empty(struct snps_udc_ep *ep)
{
	return ep->dma.add_idx == ep->dma.remove_idx;
}

static inline void dma_desc_chain_free(struct snps_udc_ep *ep)
{
	ep->dma.remove_idx++;
}

static inline int dma_desc_chain_is_full(struct snps_udc_ep *ep)
{
	return !dma_desc_chain_is_empty(ep) &&
		(EP_DMA_DESC_IDX(ep->dma.add_idx) ==
		 EP_DMA_DESC_IDX(ep->dma.remove_idx));
}

static inline struct data_desc *dma_desc_chain_head(struct snps_udc_ep *ep)
{
	u32 index = EP_DMA_DESC_IDX(ep->dma.remove_idx);

	return &ep->dma.virt->desc[index];
}

static inline void dma_desc_chain_reset(struct snps_udc_ep *ep)
{
	ep->dma.add_idx = 0;
	ep->dma.remove_idx = 0;
}
#endif
