/*
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/shared_mdio.h>
#include <linux/io.h>
#include <linux/phy.h>
#include <linux/of_mdio.h>

struct iproc_mdio_priv {
	struct shared_mdio_master *master;
	struct mii_bus *mii_bus;
};

static int iproc_mdio_mii_read(struct mii_bus *bus, int phyadr, int reg)
{
	struct iproc_mdio_priv *priv = (struct iproc_mdio_priv *)bus->priv;

	return shared_mdio_read(priv->master, phyadr, reg);
}

static int iproc_mdio_mii_write(struct mii_bus *bus, int phyadr, int reg,
				u16 val)
{
	struct iproc_mdio_priv *priv = bus->priv;

	return shared_mdio_write(priv->master, phyadr, reg, val);
}

static int iproc_mdio_probe(struct shared_mdio_master *master)
{
	struct device *dev = &master->dev;
	struct iproc_mdio_priv *priv;
	struct mii_bus *bus;
	struct device_node *dn = dev->of_node;
	int rc = 0;

	if (of_get_child_count(dn) == 0)
		return -ENODEV;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->master = master;

	priv->mii_bus = mdiobus_alloc();
	if (!priv->mii_bus) {
		dev_err(dev, "mdio bus alloc failed\n");
		return -ENOMEM;
	}

	bus = priv->mii_bus;
	bus->priv = priv;
	bus->name = "iProc MDIO bus";
	snprintf(bus->id, MII_BUS_ID_SIZE, "%s-%d", dev_name(&master->dev),
							master->dev_num);
	bus->parent = &master->dev;
	bus->read = iproc_mdio_mii_read;
	bus->write = iproc_mdio_mii_write;

	rc = of_mdiobus_register(bus, master->dev.of_node);
	if (rc) {
		dev_err(dev, "mdio bus registration failed\n");
		goto mdiobus_err;
	}

	shared_mdio_set_drvdata(master, priv);
	dev_info(dev, "iProc MDIO bus registered\n");
	return 0;

mdiobus_err:
	mdiobus_free(bus);
	return rc;
}

int iproc_mdio_remove(struct shared_mdio_master *master)
{
	struct iproc_mdio_priv *priv = shared_mdio_get_drvdata(master);

	mdiobus_unregister(priv->mii_bus);
	mdiobus_free(priv->mii_bus);

	return 0;
}

static const struct of_device_id iproc_mdio_of_match[] = {
	{ .compatible = "brcm,iproc-mdio-master-eth" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, iproc_mdio_of_match);

static struct shared_mdio_driver iproc_mdio_driver = {
	.driver = {
		.name = "iproc-mdio-master-eth",
		.of_match_table = iproc_mdio_of_match,
	},
	.probe = iproc_mdio_probe,
	.remove = iproc_mdio_remove,
};
module_shared_mdio_driver(iproc_mdio_driver);

MODULE_DESCRIPTION("Broadcom iProc shared mdio master driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Pramod Kumar <pramod.kumar@broadcom.com>");
