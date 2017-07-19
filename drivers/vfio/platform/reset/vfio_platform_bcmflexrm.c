/*
 * Copyright (C) 2017 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * This driver provides reset support for Broadcom FlexRM ring manager
 * to VFIO platform.
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include "vfio_platform_private.h"

/* FlexRM configuration */
#define RING_REGS_SIZE					0x10000
#define RING_VER_MAGIC					0x76303031

/* Per-Ring register offsets */
#define RING_VER					0x000
#define RING_CONTROL					0x034
#define RING_FLUSH_DONE					0x038

/* Register RING_CONTROL fields */
#define CONTROL_FLUSH_SHIFT				5
#define CONTROL_ACTIVE_SHIFT				4

/* Register RING_FLUSH_DONE fields */
#define FLUSH_DONE_MASK					0x1

static void vfio_platform_bcmflexrm_shutdown(void __iomem *ring)
{
	unsigned int timeout;

	/* Disable/inactivate ring */
	writel_relaxed(0x0, ring + RING_CONTROL);

	/* Flush ring with timeout of 1s */
	timeout = 1000;
	writel_relaxed(BIT(CONTROL_FLUSH_SHIFT), ring + RING_CONTROL);
	do {
		if (readl_relaxed(ring + RING_FLUSH_DONE) & FLUSH_DONE_MASK)
			break;
		mdelay(1);
	} while (timeout--);

	if (!timeout)
		pr_warn("VFIO FlexRM shutdown timedout\n");
}

static int vfio_platform_bcmflexrm_reset(struct vfio_platform_device *vdev)
{
	void __iomem *ring;
	struct vfio_platform_region *reg = &vdev->regions[0];

	/* Map FlexRM ring registers if not mapped */
	if (!reg->ioaddr) {
		reg->ioaddr = ioremap_nocache(reg->addr, reg->size);
		if (!reg->ioaddr)
			return -ENOMEM;
	}

	/* Discover and shutdown each FlexRM ring */
	for (ring = reg->ioaddr;
	     ring < (reg->ioaddr + reg->size); ring += RING_REGS_SIZE) {
		if (readl_relaxed(ring + RING_VER) == RING_VER_MAGIC)
			vfio_platform_bcmflexrm_shutdown(ring);
	}

	return 0;
}

module_vfio_reset_handler("brcm,iproc-flexrm-mbox",
			  vfio_platform_bcmflexrm_reset);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Anup Patel <anup.patel@broadcom.com>");
MODULE_DESCRIPTION("Reset support for Broadcom FlexRM VFIO platform device");
