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

#include <linux/delay.h>
#include <linux/extcon.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#define ICFG_DRD_AFE		0x0
#define ICFG_MISC_STAT		0x18
#define ICFG_DRD_P0CTL		0x1C
#define ICFG_STRAP_CTRL		0x20
#define ICFG_FSM_CTRL		0x24

#define IDM_RST_BIT		BIT(0)
#define AFE_CORERDY_VDDC	BIT(18)
#define PHY_PLL_RESETB		BIT(15)
#define PHY_RESETB		BIT(14)
#define PHY_PLL_LOCK		BIT(0)

#define DRD_DEV_MODE		BIT(20)
#define OHCI_OVRCUR_POL		BIT(11)
#define ICFG_OFF_MODE		BIT(6)
#define PLL_LOCK_RETRY		1000

#define EVT_DEVICE		0
#define EVT_HOST		1
#define EVT_IDLE		2

#define DRD_HOST_MODE		(BIT(2) | BIT(3))
#define DRD_DEVICE_MODE		(BIT(4) | BIT(5))
#define DRD_HOST_VAL		0x803
#define DRD_DEV_VAL		0x807
#define GPIO_DELAY		20
#define PHY_WQ_DELAY		msecs_to_jiffies(600)

struct ns2_phy_data;
struct ns2_phy_driver {
	void __iomem *icfgdrd_regs;
	void __iomem *idmdrd_rst_ctrl;
	void __iomem *crmu_usb2_ctrl;
	void __iomem *usb2h_strap_reg;
	spinlock_t lock; /* spin lock for phy driver */
	bool host_mode;
	struct ns2_phy_data *data;
	struct extcon_specific_cable_nb extcon_dev;
	struct extcon_specific_cable_nb extcon_host;
	struct notifier_block host_nb;
	struct notifier_block dev_nb;
	struct delayed_work conn_work;
	struct extcon_dev *edev;
	struct gpio_desc *vbus_gpiod;
	struct gpio_desc *id_gpiod;
	int id_irq;
	int vbus_irq;
	unsigned long debounce_jiffies;
	struct delayed_work wq_extcon;
};

struct ns2_phy_data {
	struct ns2_phy_driver *driver;
	struct phy *phy;
	int new_state;
	bool poweron;
};

static const unsigned int usb_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_NONE,
};

static inline int pll_lock_stat(u32 usb_reg, int reg_mask,
				struct ns2_phy_driver *driver)
{
	int retry = PLL_LOCK_RETRY;
	u32 val;

	do {
		udelay(1);
		val = readl(driver->icfgdrd_regs + usb_reg);
		if (val & reg_mask)
			return 0;
	} while (--retry > 0);

	return -EBUSY;
}

static int ns2_drd_phy_init(struct phy *phy)
{
	struct ns2_phy_data *data = phy_get_drvdata(phy);
	struct ns2_phy_driver *driver = data->driver;
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&driver->lock, flags);

	val = readl(driver->icfgdrd_regs + ICFG_FSM_CTRL);

	if (data->new_state == EVT_HOST) {
		val &= ~DRD_DEVICE_MODE;
		val |= DRD_HOST_MODE;
	} else {
		val &= ~DRD_HOST_MODE;
		val |= DRD_DEVICE_MODE;
	}
	writel(val, driver->icfgdrd_regs + ICFG_FSM_CTRL);

	spin_unlock_irqrestore(&driver->lock, flags);
	return 0;
}

static int ns2_drd_phy_shutdown(struct phy *phy)
{
	struct ns2_phy_data *data = phy_get_drvdata(phy);
	struct ns2_phy_driver *driver = data->driver;
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&driver->lock, flags);
	if (!data->poweron)
		goto exit;

	val = readl(driver->crmu_usb2_ctrl);
	val &= ~AFE_CORERDY_VDDC;
	writel(val, driver->crmu_usb2_ctrl);

	driver->host_mode = 0;
	val = readl(driver->crmu_usb2_ctrl);
	val &= ~DRD_DEV_MODE;
	writel(val, driver->crmu_usb2_ctrl);

	/* Disable Host and Device Mode */
	val = readl(driver->icfgdrd_regs + ICFG_FSM_CTRL);
	val &= ~(DRD_HOST_MODE | DRD_DEVICE_MODE | ICFG_OFF_MODE);
	writel(val, driver->icfgdrd_regs + ICFG_FSM_CTRL);

	data->poweron = 0;
exit:
	spin_unlock_irqrestore(&driver->lock, flags);
	return 0;
}

static int ns2_drd_phy_poweron(struct phy *phy)
{
	struct ns2_phy_data *data = phy_get_drvdata(phy);
	struct ns2_phy_driver *driver = data->driver;
	u32 extcon_event = data->new_state;
	unsigned long flags;
	int ret;
	u32 val;

	spin_lock_irqsave(&driver->lock, flags);
	if (extcon_event == EVT_DEVICE) {
		if (data->poweron)
			goto exit;

		writel(DRD_DEV_VAL, driver->icfgdrd_regs + ICFG_DRD_P0CTL);

		val = readl(driver->icfgdrd_regs + ICFG_FSM_CTRL);
		val &= ~(DRD_HOST_MODE | ICFG_OFF_MODE);
		val |= DRD_DEVICE_MODE;
		writel(val, driver->icfgdrd_regs + ICFG_FSM_CTRL);

		val = readl(driver->idmdrd_rst_ctrl);
		val &= ~IDM_RST_BIT;
		writel(val, driver->idmdrd_rst_ctrl);

		val = readl(driver->crmu_usb2_ctrl);
		val |= (AFE_CORERDY_VDDC | DRD_DEV_MODE);
		writel(val, driver->crmu_usb2_ctrl);

		/* Bring PHY and PHY_PLL out of Reset */
		val = readl(driver->crmu_usb2_ctrl);
		val |= (PHY_PLL_RESETB | PHY_RESETB);
		writel(val, driver->crmu_usb2_ctrl);

		ret = pll_lock_stat(ICFG_MISC_STAT, PHY_PLL_LOCK, driver);
		if (ret < 0) {
			dev_err(&phy->dev, "Phy PLL lock failed\n");
			goto err_shutdown;
		}
	} else {
		if (data->poweron && driver->host_mode)
			goto exit;

		writel(DRD_HOST_VAL, driver->icfgdrd_regs + ICFG_DRD_P0CTL);

		val = readl(driver->icfgdrd_regs + ICFG_FSM_CTRL);
		val &= ~(DRD_DEVICE_MODE | ICFG_OFF_MODE);
		val |= DRD_HOST_MODE;
		writel(val, driver->icfgdrd_regs + ICFG_FSM_CTRL);

		val = readl(driver->crmu_usb2_ctrl);
		val |= AFE_CORERDY_VDDC;
		writel(val, driver->crmu_usb2_ctrl);

		ret = pll_lock_stat(ICFG_MISC_STAT, PHY_PLL_LOCK, driver);
		if (ret < 0) {
			dev_err(&phy->dev, "Phy PLL lock failed\n");
			goto err_shutdown;
		}

		val = readl(driver->idmdrd_rst_ctrl);
		val &= ~IDM_RST_BIT;
		writel(val, driver->idmdrd_rst_ctrl);

		/* port over current Polarity */
		val = readl(driver->usb2h_strap_reg);
		val |= OHCI_OVRCUR_POL;
		writel(val, driver->usb2h_strap_reg);

		driver->host_mode = 1;
	}

	data->poweron = 1;
exit:
	spin_unlock_irqrestore(&driver->lock, flags);
	return 0;

err_shutdown:
	data->poweron = 1;
	spin_unlock_irqrestore(&driver->lock, flags);
	ns2_drd_phy_shutdown(phy);
	return ret;
}

static void connect_work(struct work_struct *work)
{
	struct ns2_phy_driver *driver;
	struct ns2_phy_data *data;
	u32 extcon_event;

	driver  = container_of(to_delayed_work(work),
			       struct ns2_phy_driver, conn_work);
	data = driver->data;
	extcon_event = data->new_state;

	if (extcon_event == EVT_DEVICE || extcon_event == EVT_HOST) {
		ns2_drd_phy_init(data->phy);
		ns2_drd_phy_poweron(data->phy);
	} else if (extcon_event == EVT_IDLE) {
		ns2_drd_phy_shutdown(data->phy);
	}
}

static int drd_device_notify(struct notifier_block *self,
			     unsigned long event, void *ptr)
{
	struct ns2_phy_driver *driver = container_of(self,
					struct ns2_phy_driver, dev_nb);

	if (event) {
		pr_debug("Device connected\n");
		driver->data->new_state = EVT_DEVICE;
		schedule_delayed_work(&driver->conn_work, 0);
	} else {
		pr_debug("Device disconnected\n");
		driver->data->new_state = EVT_IDLE;
		schedule_delayed_work(&driver->conn_work, PHY_WQ_DELAY);
	}

	return NOTIFY_DONE;
}

static int drd_host_notify(struct notifier_block *self,
			   unsigned long event, void *ptr)
{
	struct ns2_phy_driver *driver = container_of(self,
					struct ns2_phy_driver, host_nb);

	if (event) {
		pr_debug("Host connected\n");
		driver->data->new_state = EVT_HOST;
		schedule_delayed_work(&driver->conn_work, 0);
	} else {
		pr_debug("Host disconnected\n");
		driver->data->new_state = EVT_IDLE;
		schedule_delayed_work(&driver->conn_work, PHY_WQ_DELAY);
	}

	return NOTIFY_DONE;
}

static void extcon_work(struct work_struct *work)
{
	struct ns2_phy_driver *driver;
	int vbus;
	int id;

	driver  = container_of(to_delayed_work(work),
			       struct ns2_phy_driver, wq_extcon);

	id = gpiod_get_value_cansleep(driver->id_gpiod);
	vbus = gpiod_get_value_cansleep(driver->vbus_gpiod);

	if (!id && vbus) {
		extcon_set_cable_state_(driver->edev, EXTCON_USB_HOST, true);
	} else if (id && !vbus) {
		extcon_set_cable_state_(driver->edev, EXTCON_USB_HOST, false);
		extcon_set_cable_state_(driver->edev, EXTCON_USB, false);
	} else if (id && vbus) {
		extcon_set_cable_state_(driver->edev, EXTCON_USB, true);
	}
}

static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
	struct ns2_phy_driver *driver = dev_id;

	queue_delayed_work(system_power_efficient_wq, &driver->wq_extcon,
			   driver->debounce_jiffies);

	return IRQ_HANDLED;
}

static int register_extcon_notifier(struct ns2_phy_driver *phy_driver,
				    struct device *dev)
{
	struct extcon_dev *edev;
	int ret;

	phy_driver->host_nb.notifier_call = drd_host_notify;
	phy_driver->dev_nb.notifier_call = drd_device_notify;

	edev = phy_driver->edev;

	/* Register for device change notification */
	ret = extcon_register_notifier(edev, EXTCON_USB,
				       &phy_driver->dev_nb);
	if (ret < 0) {
		dev_err(dev, "can't register extcon_dev for %s\n", edev->name);
		return ret;
	}

	/* Register for host change notification */
	ret = extcon_register_notifier(edev, EXTCON_USB_HOST,
				       &phy_driver->host_nb);
	if (ret < 0) {
		dev_err(dev, "can't register extcon_dev for %s\n", edev->name);
		goto err_dev;
	}

	/* Check the device cable connect state */
	ret = extcon_get_cable_state_(edev, EXTCON_USB);
	if (ret < 0) {
		dev_err(dev, "can't get extcon_dev state for %s\n", edev->name);
		goto err_host;
	} else if (ret) {
		phy_driver->data->new_state = EVT_DEVICE;
	}

	/* Check the host cable connect state */
	ret = extcon_get_cable_state_(edev, EXTCON_USB_HOST);
	if (ret < 0) {
		dev_err(dev, "can't get extcon_dev state for %s\n", edev->name);
		goto err_host;
	} else if (ret) {
		phy_driver->data->new_state = EVT_HOST;
	}

	return 0;

err_host:
	ret = extcon_unregister_notifier(edev, EXTCON_USB_HOST,
					&phy_driver->host_nb);
err_dev:
	ret = extcon_unregister_notifier(edev, EXTCON_USB,
					&phy_driver->dev_nb);
	return ret;
}

static struct phy_ops ops = {
	.init		= ns2_drd_phy_init,
	.power_on	= ns2_drd_phy_poweron,
	.power_off	= ns2_drd_phy_shutdown,
	.owner		= THIS_MODULE,
};

static const struct of_device_id ns2_drd_phy_dt_ids[] = {
	{ .compatible = "brcm,ns2-drd-phy", },
	{ }
};

static int ns2_drd_phy_remove(struct platform_device *pdev)
{
	struct ns2_phy_driver *driver = dev_get_drvdata(&pdev->dev);

	if (driver->edev) {
		extcon_unregister_notifier(driver->edev, EXTCON_USB_HOST,
					  &driver->host_nb);
		extcon_unregister_notifier(driver->edev, EXTCON_USB,
					  &driver->dev_nb);
	}

	return 0;
}
static int ns2_drd_phy_probe(struct platform_device *pdev)
{
	struct phy_provider *phy_provider;
	struct device *dev = &pdev->dev;
	struct ns2_phy_driver *driver;
	struct ns2_phy_data *data;
	struct resource *res;
	int ret;
	u32 val;

	driver = devm_kzalloc(dev, sizeof(struct ns2_phy_driver),
			      GFP_KERNEL);
	if (!driver)
		return -ENOMEM;

	driver->data = devm_kzalloc(dev, sizeof(struct ns2_phy_data),
				  GFP_KERNEL);
	if (!driver->data)
		return -ENOMEM;

	spin_lock_init(&driver->lock);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "icfg");
	driver->icfgdrd_regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(driver->icfgdrd_regs))
		return PTR_ERR(driver->icfgdrd_regs);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "rst-ctrl");
	driver->idmdrd_rst_ctrl = devm_ioremap_resource(dev, res);
	if (IS_ERR(driver->idmdrd_rst_ctrl))
		return PTR_ERR(driver->idmdrd_rst_ctrl);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "crmu-ctrl");
	driver->crmu_usb2_ctrl = devm_ioremap_resource(dev, res);
	if (IS_ERR(driver->crmu_usb2_ctrl))
		return PTR_ERR(driver->crmu_usb2_ctrl);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "usb2-strap");
	driver->usb2h_strap_reg = devm_ioremap_resource(dev, res);
	if (IS_ERR(driver->usb2h_strap_reg))
		return PTR_ERR(driver->usb2h_strap_reg);

	 /* create extcon */
	driver->id_gpiod = devm_gpiod_get(&pdev->dev, "id", GPIOD_IN);
	if (IS_ERR(driver->id_gpiod)) {
		dev_err(dev, "failed to get ID GPIO\n");
		return PTR_ERR(driver->id_gpiod);
	}
	driver->vbus_gpiod = devm_gpiod_get(&pdev->dev, "vbus", GPIOD_IN);
	if (IS_ERR(driver->vbus_gpiod)) {
		dev_err(dev, "failed to get VBUS GPIO\n");
		return PTR_ERR(driver->vbus_gpiod);
	}

	driver->edev = devm_extcon_dev_allocate(dev, usb_extcon_cable);
	if (IS_ERR(driver->edev)) {
		dev_err(dev, "failed to allocate extcon device\n");
		return -ENOMEM;
	}

	ret = devm_extcon_dev_register(dev, driver->edev);
	if (ret < 0) {
		dev_err(dev, "failed to register extcon device\n");
		goto extcon_free;
	}

	ret = gpiod_set_debounce(driver->id_gpiod, GPIO_DELAY * 1000);
	if (ret < 0)
		driver->debounce_jiffies = msecs_to_jiffies(GPIO_DELAY);

	INIT_DELAYED_WORK(&driver->wq_extcon, extcon_work);

	driver->id_irq = gpiod_to_irq(driver->id_gpiod);
	if (driver->id_irq < 0) {
		dev_err(dev, "failed to get ID IRQ\n");
		ret = driver->id_irq;
		goto extcon_unregister;
	}
	driver->vbus_irq = gpiod_to_irq(driver->vbus_gpiod);
	if (driver->vbus_irq < 0) {
		dev_err(dev, "failed to get ID IRQ\n");
		ret = driver->vbus_irq;
		goto extcon_unregister;
	}

	ret = devm_request_threaded_irq(dev, driver->id_irq, NULL,
					gpio_irq_handler,
					IRQF_TRIGGER_RISING |
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					"usb_id", driver);
	if (ret < 0) {
		dev_err(dev, "failed to request handler for ID IRQ\n");
		goto extcon_unregister;
	}
	ret = devm_request_threaded_irq(dev, driver->vbus_irq, NULL,
					gpio_irq_handler,
					IRQF_TRIGGER_RISING |
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					"usb_vbus", driver);
	if (ret < 0) {
		dev_err(dev, "failed to request handler for VBUS IRQ\n");
		goto extcon_unregister;
	}

	dev_set_drvdata(dev, driver);
	driver->host_mode = 0;

	/* Shutdown all ports. They can be powered up as required */
	val = readl(driver->crmu_usb2_ctrl);
	val &= ~(AFE_CORERDY_VDDC | PHY_RESETB);
	writel(val, driver->crmu_usb2_ctrl);

	data = driver->data;
	data->phy = devm_phy_create(dev, dev->of_node, &ops);
	if (IS_ERR(data->phy)) {
		dev_err(dev, "Failed to create usb drd phy\n");
		ret = PTR_ERR(data->phy);
		goto extcon_unregister;
	}

	data->driver = driver;
	data->poweron = 0;
	phy_set_drvdata(data->phy, data);

	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (IS_ERR(phy_provider)) {
		dev_err(dev, "Failed to register as phy provider\n");
		ret = PTR_ERR(phy_provider);
		goto extcon_unregister;
	}

	INIT_DELAYED_WORK(&driver->conn_work, connect_work);
	platform_set_drvdata(pdev, driver);

	ret = register_extcon_notifier(driver, dev);
	if (ret < 0) {
		dev_err(dev, "register extcon notifier failed (%d)\n", ret);
		goto extcon_unregister;
	}
	dev_info(dev, "Registered %s\n", driver->edev->name);
	queue_delayed_work(system_power_efficient_wq, &driver->wq_extcon,
			   driver->debounce_jiffies);

	return 0;

extcon_unregister:
	devm_extcon_dev_unregister(dev, driver->edev);
extcon_free:
	devm_extcon_dev_free(dev, driver->edev);
	return ret;
}

MODULE_DEVICE_TABLE(of, ns2_drd_phy_dt_ids);

static struct platform_driver ns2_drd_phy_driver = {
	.probe = ns2_drd_phy_probe,
	.remove = ns2_drd_phy_remove,
	.driver = {
		.name = "bcm-ns2-usbphy",
		.of_match_table = of_match_ptr(ns2_drd_phy_dt_ids),
	},
};
module_platform_driver(ns2_drd_phy_driver);

MODULE_ALIAS("platform:bcm-ns2-drd-phy");
MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom NS2 USB2 PHY driver");
MODULE_LICENSE("GPL v2");
