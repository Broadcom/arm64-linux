/*
 * Copyright (C) 2015 Broadcom Corporation
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

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/phy/iproc_mdio_phy.h>

#define MDIO_TIMEOUT_MSEC             10

#define MDC_CTRL_OFFSET               0x000

#define MDC_CTRL_DIV_SHIFT            0
#define MDC_CTRL_DIV_WIDTH            7
#define MDC_MAX_DIV                   (BIT(MDC_CTRL_DIV_WIDTH) - 1)
#define MDC_CTRL_PRE_SHIFT            7
#define MDC_CTRL_BUSY_SHIFT           8
#define MDC_CTRL_EXT_SHIFT            9
#define MDC_CTRL_BTP_SHIFT            10

#define MDC_DATA_OFFSET               0x004

#define MDC_DATA_SHIFT                0
#define MDC_DATA_MASK                 0xffff

#define MDC_DATA_TA_SHIFT             16
#define MDC_DATA_TA_VAL               2

#define MDC_DATA_RA_SHIFT             18
#define MDC_DATA_RA_WIDTH             5
#define MDC_MAX_RA                    (BIT(MDC_DATA_RA_WIDTH) - 1)

#define MDC_DATA_PA_SHIFT             23
#define MDC_DATA_PA_WIDTH             5
#define MDC_MAX_PA                    (BIT(MDC_DATA_PA_WIDTH) - 1)

#define MDC_DATA_OP_SHIFT             28
#define MDC_DATA_OP_WRITE             1
#define MDC_DATA_OP_READ              2

#define MDC_DATA_SB_SHIFT             30

/**
 * struct iproc_mdc - iProc MDC/MDIO device
 * @dev: pointer to device
 * @base: MDC controller register base pointer
 * @lock: mutex to protect access to the MDC device
 * @is_ready: flag to indicate the driver has been initialized and is ready for
 * use
 */
struct iproc_mdc {
	struct device *dev;
	void __iomem *base;
	struct mutex lock;
	bool is_ready;
};

static struct iproc_mdc iproc_mdc;

static inline int iproc_mdio_check_params(unsigned clk_div, unsigned pa,
					  unsigned ra)
{
	if (clk_div > MDC_MAX_DIV || pa > MDC_MAX_PA || ra > MDC_MAX_RA)
		return -EINVAL;

	return 0;
}

static inline int iproc_mdio_wait_for_idle(void __iomem *base)
{
	u32 val;
	unsigned long timeout = jiffies + msecs_to_jiffies(MDIO_TIMEOUT_MSEC);

	while (!time_after(jiffies, timeout)) {
		val = readl(base + MDC_CTRL_OFFSET);
		if ((val & BIT(MDC_CTRL_BUSY_SHIFT)) == 0)
			return 0;

		cpu_relax();
	}

	return -ETIMEDOUT;
}

static inline void iproc_mdio_config_clk(void __iomem *base, unsigned clk_div)
{
	u32 val;

	val = (clk_div << MDC_CTRL_DIV_SHIFT) | BIT(MDC_CTRL_PRE_SHIFT);
	writel(val, base + MDC_CTRL_OFFSET);
}

static int iproc_mdio_read_write(void __iomem *base, bool dir_is_write,
				 unsigned clk_div, unsigned pa, unsigned ra,
				 u16 *data)
{
	int ret;
	u32 val;

	ret = iproc_mdio_check_params(clk_div, pa, ra);
	if (ret)
		return ret;

	ret = iproc_mdio_wait_for_idle(base);
	if (ret)
		return ret;

	iproc_mdio_config_clk(base, clk_div);

	ret = iproc_mdio_wait_for_idle(base);
	if (ret)
		return ret;

	val = (MDC_DATA_TA_VAL << MDC_DATA_TA_SHIFT) |
		(ra << MDC_DATA_RA_SHIFT) |
		(pa << MDC_DATA_PA_SHIFT) |
		BIT(MDC_DATA_SB_SHIFT);

	if (dir_is_write) {
		val |= (MDC_DATA_OP_WRITE << MDC_DATA_OP_SHIFT);
		val |= ((u32)(*data) & MDC_DATA_MASK);
	} else {
		val |= (MDC_DATA_OP_READ << MDC_DATA_OP_SHIFT);
	}

	writel(val, base + MDC_DATA_OFFSET);

	ret = iproc_mdio_wait_for_idle(base);
	if (ret)
		return ret;

	if (!dir_is_write) {
		val = readl(base + MDC_DATA_OFFSET) & MDC_DATA_MASK;
		*data = (u16)val;
	}

	return 0;
}


/**
 * iproc_mdio_read() - read from a PHY register through the MDC interface
 *
 * @clk_div: MDC clock divisor
 * @phy_addr: MDC PHY address
 * @reg_addr: PHY register address
 * @data: pointer to the memory where data will be stored
 */
int iproc_mdio_read(unsigned clk_div, unsigned phy_addr, unsigned reg_addr,
		    u16 *data)
{
	int ret;
	struct iproc_mdc *mdc = &iproc_mdc;

	if (!mdc->is_ready)
		return -ENODEV;

	mutex_lock(&mdc->lock);
	ret = iproc_mdio_read_write(mdc->base, false, clk_div, phy_addr,
				    reg_addr, data);
	if (ret)
		dev_err(mdc->dev, "mdio read failed\n");
	mutex_unlock(&mdc->lock);

	return ret;
}
EXPORT_SYMBOL(iproc_mdio_read);

/**
 * iproc_mdio_write() - write to a PHY register through the MDC interface
 *
 * @clk_div: MDC clock divisor
 * @phy_addr: MDC PHY address
 * @reg_addr: PHY register address
 * @data: data value to be written to the PHY register
 */
int iproc_mdio_write(unsigned clk_div, unsigned phy_addr, unsigned reg_addr,
		     u16 data)
{
	int ret;
	struct iproc_mdc *mdc = &iproc_mdc;

	if (!mdc->is_ready)
		return -ENODEV;

	mutex_lock(&mdc->lock);
	ret = iproc_mdio_read_write(mdc->base, true, clk_div, phy_addr,
				    reg_addr, &data);
	if (ret)
		dev_err(mdc->dev, "mdio write failed\n");
	mutex_unlock(&mdc->lock);

	return ret;
}
EXPORT_SYMBOL(iproc_mdio_write);

static int iproc_mdio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iproc_mdc *mdc = &iproc_mdc;
	struct resource *res;

	mdc->dev = dev;
	mutex_init(&mdc->lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mdc->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(mdc->base))
		return PTR_ERR(mdc->base);

	mdc->is_ready = true;
	return 0;
}

static const struct of_device_id iproc_mdio_match_table[] = {
	{ .compatible = "brcm,iproc-mdio" },
	{ }
};

static struct platform_driver iproc_mdio_driver = {
	.driver = {
		.name = "iproc-mdio",
		.of_match_table = iproc_mdio_match_table,
		.suppress_bind_attrs = true,
	},
	.probe	= iproc_mdio_probe,
};

static int __init iproc_mdio_init(void)
{
	return platform_driver_register(&iproc_mdio_driver);
}
arch_initcall_sync(iproc_mdio_init);

MODULE_AUTHOR("Ray Jui <rjui@broadcom.com>");
MODULE_DESCRIPTION("Broadcom iPROC MDC/MDIO driver");
MODULE_LICENSE("GPL v2");
