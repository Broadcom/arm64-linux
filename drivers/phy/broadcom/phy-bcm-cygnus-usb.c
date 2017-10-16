/*
 * Copyright 2017 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as
 * published by the Free Software Foundation (the "GPL").
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License version 2 (GPLv2) for more details.
 *
 * You should have received a copy of the GNU General Public License
 * version 2 (GPLv2) along with this source code.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/phy/phy.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/extcon.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>

#define CDRU_USBPHY_CLK_RST_SEL_OFFSET			0x0
#define CDRU_USBPHY2_HOST_DEV_SEL_OFFSET		0x4
#define CDRU_USB_DEV_SUSPEND_RESUME_CTRL_OFFSET		0x5C
#define CDRU_USBPHY_P0_STATUS_OFFSET			0x1C
#define CDRU_USBPHY_P1_STATUS_OFFSET			0x34
#define CDRU_USBPHY_P2_STATUS_OFFSET			0x4C
#define CRMU_USB_PHY_AON_CTRL_OFFSET			0x0

#define CDRU_USBPHY_USBPHY_ILDO_ON_FLAG			BIT(1)
#define CDRU_USBPHY_USBPHY_PLL_LOCK			BIT(0)
#define CDRU_USB_DEV_SUSPEND_RESUME_CTRL_DISABLE	BIT(0)

#define PHY2_DEV_HOST_CTRL_SEL_DEVICE			0
#define PHY2_DEV_HOST_CTRL_SEL_HOST			1
#define PHY2_DEV_HOST_CTRL_SEL_IDLE			2
#define CRMU_USBPHY_P0_AFE_CORERDY_VDDC			BIT(1)
#define CRMU_USBPHY_P0_RESETB				BIT(2)
#define CRMU_USBPHY_P1_AFE_CORERDY_VDDC			BIT(9)
#define CRMU_USBPHY_P1_RESETB				BIT(10)
#define CRMU_USBPHY_P2_AFE_CORERDY_VDDC			BIT(17)
#define CRMU_USBPHY_P2_RESETB				BIT(18)

#define USB2_IDM_IDM_IO_CONTROL_DIRECT_OFFSET		0x0408
#define USB2_IDM_IDM_IO_CONTROL_DIRECT_CLK_ENABLE	BIT(0)
#define SUSPEND_OVERRIDE_0				13
#define SUSPEND_OVERRIDE_1				14
#define SUSPEND_OVERRIDE_2				15
#define USB2_IDM_IDM_RESET_CONTROL_OFFSET		0x0800
#define USB2_IDM_IDM_RESET_CONTROL__RESET		0
#define USB2D_IDM_IDM_IO_SS_CLEAR_NAK_NEMPTY_EN_I	BIT(24)

#define PLL_LOCK_RETRY_COUNT				1000
#define MAX_REGULATOR_NAME_LEN				25
#define DUAL_ROLE_PHY					2

#define USBPHY_WQ_DELAY_MS		msecs_to_jiffies(500)
#define USB2_SEL_DEVICE			0
#define USB2_SEL_HOST			1
#define USB2_SEL_IDLE			2
#define USB_CONNECTED			1
#define USB_DISCONNECTED		0
#define MAX_NUM_PHYS			3

static const int status_reg[] = {CDRU_USBPHY_P0_STATUS_OFFSET,
				CDRU_USBPHY_P1_STATUS_OFFSET,
				CDRU_USBPHY_P2_STATUS_OFFSET};

struct cygnus_phy_instance;

struct cygnus_phy_driver {
	void __iomem *cdru_usbphy_regs;
	void __iomem *crmu_usbphy_aon_ctrl_regs;
	void __iomem *usb2h_idm_regs;
	void __iomem *usb2d_idm_regs;
	int num_phys;
	bool idm_host_enabled;
	struct cygnus_phy_instance *instances;
	int phyto_src_clk;
	struct platform_device *pdev;
};

struct cygnus_phy_instance {
	struct cygnus_phy_driver *driver;
	struct phy *generic_phy;
	int port;
	int new_state;		/* 1 - Host, 0 - device, 2 - idle*/
	bool power;		/* 1 - powered_on 0 - powered off */
	struct regulator *vbus_supply;
	spinlock_t lock;
	struct extcon_dev *edev;
	struct notifier_block	device_nb;
	struct notifier_block	host_nb;
};

static const unsigned int usb_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_NONE,
};

static inline int phy_pll_lock_stat(u32 usb_reg, int bit_mask,
				    struct cygnus_phy_driver *phy_driver)
{
	/* Wait for the PLL lock status */
	int retry = PLL_LOCK_RETRY_COUNT;
	u32 reg_val;

	do {
		udelay(1);
		reg_val = readl(phy_driver->cdru_usbphy_regs +
				usb_reg);
		if (reg_val & bit_mask)
			return 0;
	} while (--retry > 0);

	return -EBUSY;
}

static struct phy *cygnus_phy_xlate(struct device *dev,
				    struct of_phandle_args *args)
{
	struct cygnus_phy_driver *phy_driver = dev_get_drvdata(dev);
	struct cygnus_phy_instance *instance_ptr;

	if (!phy_driver)
		return ERR_PTR(-EINVAL);

	if (WARN_ON(args->args[0] >= phy_driver->num_phys))
		return ERR_PTR(-ENODEV);

	if (WARN_ON(args->args_count < 1))
		return ERR_PTR(-EINVAL);

	instance_ptr = &phy_driver->instances[args->args[0]];
	if (instance_ptr->port > phy_driver->phyto_src_clk)
		phy_driver->phyto_src_clk = instance_ptr->port;

	if (instance_ptr->port == DUAL_ROLE_PHY)
		goto ret_p2;
	phy_driver->instances[instance_ptr->port].new_state =
						PHY2_DEV_HOST_CTRL_SEL_HOST;

ret_p2:
	return phy_driver->instances[instance_ptr->port].generic_phy;
}

static void cygnus_usbp2_dev_clock_init(struct phy *generic_phy, bool enable)
{
	u32 reg_val;
	struct cygnus_phy_instance *instance_ptr = phy_get_drvdata(generic_phy);
	struct cygnus_phy_driver *phy_driver = instance_ptr->driver;

	reg_val = readl(phy_driver->usb2d_idm_regs +
			USB2_IDM_IDM_IO_CONTROL_DIRECT_OFFSET);

	if (enable)
		reg_val |= USB2_IDM_IDM_IO_CONTROL_DIRECT_CLK_ENABLE;
	else
		reg_val &= ~USB2_IDM_IDM_IO_CONTROL_DIRECT_CLK_ENABLE;

	writel(reg_val, phy_driver->usb2d_idm_regs +
		USB2_IDM_IDM_IO_CONTROL_DIRECT_OFFSET);

	reg_val = readl(phy_driver->usb2d_idm_regs +
			USB2_IDM_IDM_RESET_CONTROL_OFFSET);

	if (enable)
		reg_val &= ~BIT(USB2_IDM_IDM_RESET_CONTROL__RESET);
	else
		reg_val |= BIT(USB2_IDM_IDM_RESET_CONTROL__RESET);

	writel(reg_val, phy_driver->usb2d_idm_regs +
		USB2_IDM_IDM_RESET_CONTROL_OFFSET);
}

static void cygnus_phy_clk_reset_src_switch(struct phy *generic_phy,
					    u32 src_phy)
{
	u32 reg_val;
	struct cygnus_phy_instance *instance_ptr = phy_get_drvdata(generic_phy);
	struct cygnus_phy_driver *phy_driver = instance_ptr->driver;

	writel(src_phy, phy_driver->cdru_usbphy_regs +
		CDRU_USBPHY_CLK_RST_SEL_OFFSET);

	/* Force the clock/reset source phy to not suspend */
	reg_val = readl(phy_driver->usb2h_idm_regs +
			USB2_IDM_IDM_IO_CONTROL_DIRECT_OFFSET);
	reg_val &= ~(BIT(SUSPEND_OVERRIDE_0) |
			BIT(SUSPEND_OVERRIDE_1) |
			BIT(SUSPEND_OVERRIDE_2));
	reg_val |= BIT(SUSPEND_OVERRIDE_0 + src_phy);

	writel(reg_val, phy_driver->usb2h_idm_regs +
		USB2_IDM_IDM_IO_CONTROL_DIRECT_OFFSET);
}

static void cygnus_phy_dual_role_init(struct cygnus_phy_instance *instance_ptr)
{
	u32 reg_val;
	struct cygnus_phy_driver *phy_driver = instance_ptr->driver;

	if (instance_ptr->new_state == PHY2_DEV_HOST_CTRL_SEL_HOST) {
		writel(PHY2_DEV_HOST_CTRL_SEL_HOST,
			phy_driver->cdru_usbphy_regs +
			CDRU_USBPHY2_HOST_DEV_SEL_OFFSET);
	} else {
		/*
		 * Disable suspend/resume signals to device controller
		 * when a port is in device mode
		 */
		writel(PHY2_DEV_HOST_CTRL_SEL_DEVICE,
			phy_driver->cdru_usbphy_regs +
			CDRU_USBPHY2_HOST_DEV_SEL_OFFSET);
		reg_val = readl(phy_driver->cdru_usbphy_regs +
				CDRU_USB_DEV_SUSPEND_RESUME_CTRL_OFFSET);
		reg_val |= CDRU_USB_DEV_SUSPEND_RESUME_CTRL_DISABLE;
		writel(reg_val, phy_driver->cdru_usbphy_regs +
				CDRU_USB_DEV_SUSPEND_RESUME_CTRL_OFFSET);
	}
}

static int cygnus_phy_init(struct phy *generic_phy)
{
	struct cygnus_phy_instance *instance_ptr = phy_get_drvdata(generic_phy);

	if (instance_ptr->port == DUAL_ROLE_PHY)
		cygnus_phy_dual_role_init(instance_ptr);

	return 0;
}

static int cygnus_phy_shutdown(struct phy *generic_phy)
{
	u32 reg_val;
	int i, ret;
	unsigned long flags;
	bool power_off_flag = true;
	struct cygnus_phy_instance *instance_ptr = phy_get_drvdata(generic_phy);
	struct cygnus_phy_driver *phy_driver = instance_ptr->driver;

	if (instance_ptr->vbus_supply) {
		ret = regulator_disable(instance_ptr->vbus_supply);
		if (ret) {
			dev_err(&generic_phy->dev,
					"Failed to disable regulator\n");
			return ret;
		}
	}

	spin_lock_irqsave(&instance_ptr->lock, flags);

	/* power down the phy */
	reg_val = readl(phy_driver->crmu_usbphy_aon_ctrl_regs +
			CRMU_USB_PHY_AON_CTRL_OFFSET);
	if (instance_ptr->port == 0) {
		reg_val &= ~CRMU_USBPHY_P0_AFE_CORERDY_VDDC;
		reg_val &= ~CRMU_USBPHY_P0_RESETB;
	} else if (instance_ptr->port == 1) {
		reg_val &= ~CRMU_USBPHY_P1_AFE_CORERDY_VDDC;
		reg_val &= ~CRMU_USBPHY_P1_RESETB;
	} else if (instance_ptr->port == 2) {
		reg_val &= ~CRMU_USBPHY_P2_AFE_CORERDY_VDDC;
		reg_val &= ~CRMU_USBPHY_P2_RESETB;
	}
	writel(reg_val, phy_driver->crmu_usbphy_aon_ctrl_regs +
		CRMU_USB_PHY_AON_CTRL_OFFSET);

	instance_ptr->power = false;

	/* Determine whether all the phy's are powered off */
	for (i = 0; i < phy_driver->num_phys; i++) {
		if (phy_driver->instances[i].power == true) {
			power_off_flag = false;
			break;
		}
	}

	/* Disable clock to USB device and keep the USB device in reset */
	if (instance_ptr->port == DUAL_ROLE_PHY)
		cygnus_usbp2_dev_clock_init(instance_ptr->generic_phy,
					    false);

	/*
	 * Put the host controller into reset state and
	 * disable clock if all the phy's are powered off
	 */
	if (power_off_flag) {
		reg_val = readl(phy_driver->usb2h_idm_regs +
			USB2_IDM_IDM_IO_CONTROL_DIRECT_OFFSET);
		reg_val &= ~USB2_IDM_IDM_IO_CONTROL_DIRECT_CLK_ENABLE;
		writel(reg_val, phy_driver->usb2h_idm_regs +
				USB2_IDM_IDM_IO_CONTROL_DIRECT_OFFSET);

		reg_val = readl(phy_driver->usb2h_idm_regs +
				 USB2_IDM_IDM_RESET_CONTROL_OFFSET);
		reg_val |= BIT(USB2_IDM_IDM_RESET_CONTROL__RESET);
		writel(reg_val, phy_driver->usb2h_idm_regs +
				USB2_IDM_IDM_RESET_CONTROL_OFFSET);
		phy_driver->idm_host_enabled = false;
	}
	spin_unlock_irqrestore(&instance_ptr->lock, flags);
	return 0;
}

static int cygnus_phy_poweron(struct phy *generic_phy)
{
	int ret;
	unsigned long flags;
	u32 reg_val;
	struct cygnus_phy_instance *instance_ptr = phy_get_drvdata(generic_phy);
	struct cygnus_phy_driver *phy_driver = instance_ptr->driver;
	u32 extcon_event = instance_ptr->new_state;

	/*
	 * Switch on the regulator only if in HOST mode
	 */
	if (instance_ptr->vbus_supply) {
		ret = regulator_enable(instance_ptr->vbus_supply);
		if (ret) {
			dev_err(&generic_phy->dev,
				"Failed to enable regulator\n");
			goto err_shutdown;
		}
	}

	spin_lock_irqsave(&instance_ptr->lock, flags);
	/* Bring the AFE block out of reset to start powering up the PHY */
	reg_val = readl(phy_driver->crmu_usbphy_aon_ctrl_regs +
			CRMU_USB_PHY_AON_CTRL_OFFSET);
	if (instance_ptr->port == 0)
		reg_val |= CRMU_USBPHY_P0_AFE_CORERDY_VDDC;
	else if (instance_ptr->port == 1)
		reg_val |= CRMU_USBPHY_P1_AFE_CORERDY_VDDC;
	else if (instance_ptr->port == 2)
		reg_val |= CRMU_USBPHY_P2_AFE_CORERDY_VDDC;
	writel(reg_val, phy_driver->crmu_usbphy_aon_ctrl_regs +
		CRMU_USB_PHY_AON_CTRL_OFFSET);

	/* Check for power on and PLL lock */
	ret = phy_pll_lock_stat(status_reg[instance_ptr->port],
				CDRU_USBPHY_USBPHY_ILDO_ON_FLAG, phy_driver);
	if (ret < 0) {
		dev_err(&generic_phy->dev,
			"Timed out waiting for USBPHY_ILDO_ON_FLAG on port %d",
			instance_ptr->port);
		spin_unlock_irqrestore(&instance_ptr->lock, flags);
		goto err_shutdown;
	}
	ret = phy_pll_lock_stat(status_reg[instance_ptr->port],
				CDRU_USBPHY_USBPHY_PLL_LOCK, phy_driver);
	if (ret < 0) {
		dev_err(&generic_phy->dev,
			"Timed out waiting for USBPHY_PLL_LOCK on port %d",
			instance_ptr->port);
		spin_unlock_irqrestore(&instance_ptr->lock, flags);
		goto err_shutdown;
	}

	instance_ptr->power = true;

	/* Enable clock to USB device and take the USB device out of reset */
	if (instance_ptr->port == DUAL_ROLE_PHY)
		cygnus_usbp2_dev_clock_init(instance_ptr->generic_phy, true);

	/* Set clock source provider to be the last powered on phy */
	if (instance_ptr->port == phy_driver->phyto_src_clk)
		cygnus_phy_clk_reset_src_switch(generic_phy,
				instance_ptr->port);

	if (phy_driver->idm_host_enabled != true &&
		extcon_event == PHY2_DEV_HOST_CTRL_SEL_HOST) {
		/* Enable clock to USB host and take the host out of reset */
		reg_val = readl(phy_driver->usb2h_idm_regs +
				USB2_IDM_IDM_IO_CONTROL_DIRECT_OFFSET);
		reg_val |= USB2_IDM_IDM_IO_CONTROL_DIRECT_CLK_ENABLE;
		writel(reg_val, phy_driver->usb2h_idm_regs +
				USB2_IDM_IDM_IO_CONTROL_DIRECT_OFFSET);

		reg_val = readl(phy_driver->usb2h_idm_regs +
				USB2_IDM_IDM_RESET_CONTROL_OFFSET);
		reg_val &= ~BIT(USB2_IDM_IDM_RESET_CONTROL__RESET);
		writel(reg_val, phy_driver->usb2h_idm_regs +
				 USB2_IDM_IDM_RESET_CONTROL_OFFSET);
		phy_driver->idm_host_enabled = true;
	}
	spin_unlock_irqrestore(&instance_ptr->lock, flags);

	return 0;
err_shutdown:
	cygnus_phy_shutdown(generic_phy);
	return ret;
}

static int usbd_connect_notify(struct notifier_block *self,
			       unsigned long event, void *ptr)
{
	struct cygnus_phy_instance *instance_ptr = container_of(self,
					struct cygnus_phy_instance, device_nb);

	if (event) {
		instance_ptr->new_state = PHY2_DEV_HOST_CTRL_SEL_DEVICE;
		cygnus_phy_dual_role_init(instance_ptr);
	}

	return NOTIFY_OK;
}

static int usbh_connect_notify(struct notifier_block *self,
			       unsigned long event, void *ptr)
{
	struct cygnus_phy_instance *instance_ptr = container_of(self,
					struct cygnus_phy_instance, host_nb);
	if (event) {
		instance_ptr->new_state = PHY2_DEV_HOST_CTRL_SEL_HOST;
		cygnus_phy_dual_role_init(instance_ptr);
	}

	return NOTIFY_OK;
}

static int cygnus_register_extcon_notifiers(
				struct cygnus_phy_instance *instance_ptr)
{
	int ret = 0;
	struct device *dev = &instance_ptr->generic_phy->dev;

	if (of_property_read_bool(dev->of_node, "extcon")) {
		instance_ptr->edev = extcon_get_edev_by_phandle(dev, 0);
		if (IS_ERR(instance_ptr->edev)) {
			if (PTR_ERR(instance_ptr->edev) == -EPROBE_DEFER)
				return -EPROBE_DEFER;
			ret = PTR_ERR(instance_ptr->edev);
			goto err;
		}

		instance_ptr->device_nb.notifier_call = usbd_connect_notify;
		ret = extcon_register_notifier(instance_ptr->edev, EXTCON_USB,
						&instance_ptr->device_nb);
		if (ret < 0) {
			dev_err(dev, "Can't register extcon device\n");
			goto err;
		}

		if (extcon_get_state(instance_ptr->edev, EXTCON_USB) == true) {
			instance_ptr->new_state = PHY2_DEV_HOST_CTRL_SEL_DEVICE;
			cygnus_phy_dual_role_init(instance_ptr);
		}

		instance_ptr->host_nb.notifier_call = usbh_connect_notify;
		ret = extcon_register_notifier(instance_ptr->edev,
						EXTCON_USB_HOST,
						&instance_ptr->host_nb);
		if (ret < 0) {
			dev_err(dev, "Can't register extcon device\n");
			goto err;
		}

		if (extcon_get_state(instance_ptr->edev, EXTCON_USB_HOST)
					== true) {
			instance_ptr->new_state = PHY2_DEV_HOST_CTRL_SEL_HOST;
			cygnus_phy_dual_role_init(instance_ptr);
		}
	} else {
		dev_err(dev, "Extcon device handle not found\n");
		return -EINVAL;
	}

	return 0;
err:
	return ret;
}

static const struct phy_ops ops = {
	.init		= cygnus_phy_init,
	.power_on	= cygnus_phy_poweron,
	.power_off	= cygnus_phy_shutdown,
};

static int cygnus_phy_instance_create(struct cygnus_phy_driver *phy_driver)
{
	struct device_node *child;
	char *vbus_name;
	struct platform_device *pdev = phy_driver->pdev;
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct cygnus_phy_instance *instance_ptr;
	unsigned int id, ret;

	for_each_available_child_of_node(node, child) {
		if (of_property_read_u32(child, "reg", &id)) {
			dev_err(dev, "missing reg property for %s\n",
				child->name);
			ret = -EINVAL;
			goto put_child;
		}

		if (id >= MAX_NUM_PHYS) {
			dev_err(dev, "invalid PHY id: %u\n", id);
			ret = -EINVAL;
			goto put_child;
		}

		instance_ptr = &phy_driver->instances[id];
		instance_ptr->driver = phy_driver;
		instance_ptr->port = id;
		spin_lock_init(&instance_ptr->lock);

		if (instance_ptr->generic_phy) {
			dev_err(dev, "duplicated PHY id: %u\n", id);
			ret = -EINVAL;
			goto put_child;
		}

		instance_ptr->generic_phy =
				devm_phy_create(dev, child, &ops);
		if (IS_ERR(instance_ptr->generic_phy)) {
			dev_err(dev, "Failed to create usb phy %d", id);
			ret = PTR_ERR(instance_ptr->generic_phy);
			goto put_child;
		}

		vbus_name = devm_kzalloc(&instance_ptr->generic_phy->dev,
					MAX_REGULATOR_NAME_LEN,
					GFP_KERNEL);
		if (!vbus_name) {
			ret = -ENOMEM;
			goto put_child;
		}

		/* regulator use is optional */
		sprintf(vbus_name, "vbus-p%d", id);
		instance_ptr->vbus_supply =
			devm_regulator_get(&instance_ptr->generic_phy->dev,
					vbus_name);
		if (IS_ERR(instance_ptr->vbus_supply))
			instance_ptr->vbus_supply = NULL;
		devm_kfree(&instance_ptr->generic_phy->dev, vbus_name);
		phy_set_drvdata(instance_ptr->generic_phy, instance_ptr);
	}

	return 0;

put_child:
	of_node_put(child);
	return ret;
}

static int cygnus_phy_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct cygnus_phy_driver *phy_driver;
	struct phy_provider *phy_provider;
	int i, ret;
	u32 reg_val;
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;

	/* allocate memory for each phy instance */
	phy_driver = devm_kzalloc(dev, sizeof(struct cygnus_phy_driver),
				  GFP_KERNEL);
	if (!phy_driver)
		return -ENOMEM;

	phy_driver->num_phys = of_get_child_count(node);

	if (phy_driver->num_phys == 0) {
		dev_err(dev, "PHY no child node\n");
		return -ENODEV;
	}

	phy_driver->instances = devm_kcalloc(dev, phy_driver->num_phys,
					     sizeof(struct cygnus_phy_instance),
					     GFP_KERNEL);
	phy_driver->pdev = pdev;
	platform_set_drvdata(pdev, phy_driver);

	ret = cygnus_phy_instance_create(phy_driver);
	if (ret)
		return ret;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					   "crmu-usbphy-aon-ctrl");
	phy_driver->crmu_usbphy_aon_ctrl_regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(phy_driver->crmu_usbphy_aon_ctrl_regs))
		return PTR_ERR(phy_driver->crmu_usbphy_aon_ctrl_regs);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					   "cdru-usbphy");
	phy_driver->cdru_usbphy_regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(phy_driver->cdru_usbphy_regs))
		return PTR_ERR(phy_driver->cdru_usbphy_regs);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "usb2h-idm");
	phy_driver->usb2h_idm_regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(phy_driver->usb2h_idm_regs))
		return PTR_ERR(phy_driver->usb2h_idm_regs);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "usb2d-idm");
	phy_driver->usb2d_idm_regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(phy_driver->usb2d_idm_regs))
		return PTR_ERR(phy_driver->usb2d_idm_regs);

	reg_val = readl(phy_driver->usb2d_idm_regs);
	writel((reg_val | USB2D_IDM_IDM_IO_SS_CLEAR_NAK_NEMPTY_EN_I),
		   phy_driver->usb2d_idm_regs);
	phy_driver->idm_host_enabled = false;

	/*
	 * Shutdown all ports. They can be powered up as
	 * required
	 */
	reg_val = readl(phy_driver->crmu_usbphy_aon_ctrl_regs +
			CRMU_USB_PHY_AON_CTRL_OFFSET);
	reg_val &= ~CRMU_USBPHY_P0_AFE_CORERDY_VDDC;
	reg_val &= ~CRMU_USBPHY_P0_RESETB;
	reg_val &= ~CRMU_USBPHY_P1_AFE_CORERDY_VDDC;
	reg_val &= ~CRMU_USBPHY_P1_RESETB;
	reg_val &= ~CRMU_USBPHY_P2_AFE_CORERDY_VDDC;
	reg_val &= ~CRMU_USBPHY_P2_RESETB;
	writel(reg_val, phy_driver->crmu_usbphy_aon_ctrl_regs +
		CRMU_USB_PHY_AON_CTRL_OFFSET);

	phy_provider = devm_of_phy_provider_register(dev, cygnus_phy_xlate);
	if (IS_ERR(phy_provider)) {
		dev_err(dev, "Failed to register as phy provider\n");
		ret = PTR_ERR(phy_provider);
		return ret;
	}

	for (i = 0; i < phy_driver->num_phys; i++) {
		if (phy_driver->instances[i].port == DUAL_ROLE_PHY) {
			ret = cygnus_register_extcon_notifiers(
				&phy_driver->instances[DUAL_ROLE_PHY]);
			if (ret) {
				dev_err(dev, "Failed to register notifier\n");
				return ret;
			}
		}
	}

	return 0;
}

static const struct of_device_id cygnus_phy_dt_ids[] = {
	{ .compatible = "brcm,cygnus-usb-phy", },
	{ }
};
MODULE_DEVICE_TABLE(of, bcm_phy_dt_ids);

static struct platform_driver cygnus_phy_driver = {
	.probe = cygnus_phy_probe,
	.driver = {
		.name = "bcm-cygnus-usbphy",
		.of_match_table = of_match_ptr(cygnus_phy_dt_ids),
	},
};
module_platform_driver(cygnus_phy_driver);

MODULE_ALIAS("platform:bcm-cygnus-usbphy");
MODULE_AUTHOR("Raveendra Padasalagi <Raveendra.padasalagi@broadcom.com");
MODULE_DESCRIPTION("Broadcom Cygnus USB PHY driver");
MODULE_LICENSE("GPL v2");
