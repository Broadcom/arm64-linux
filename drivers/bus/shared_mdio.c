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
 *
 * Shared MDIO Bus
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/errno.h>
#include <linux/idr.h>
#include <linux/of_device.h>
#include <linux/stddef.h>
#include <linux/module.h>
#include <linux/shared_mdio.h>
#include <linux/slab.h>

static DEFINE_IDA(shared_mdio_ida);

static void shared_mdio_release_master(struct device *dev)
{
	struct shared_mdio_master *master = to_shared_mdio_master(dev);

	ida_simple_remove(&shared_mdio_ida, master->dev_num);
	kfree(master);
}

struct shared_mdio_master *shared_mdio_alloc_master(struct device *parent,
						    struct device_node *node)
{
	int ret = 0;
	struct shared_mdio_master *master;

	master = kzalloc(sizeof(*master), GFP_KERNEL);
	if (!master) {
		ret = -ENOMEM;
		goto fail1;
	}

	master->dev.parent = parent;
	master->dev.bus = &shared_mdio_bus;
	master->dev.release = shared_mdio_release_master;

	device_initialize(&master->dev);

	ret = ida_simple_get(&shared_mdio_ida, 0, 0, GFP_KERNEL);
	if (ret < 0)
		goto fail2;
	master->dev_num = ret;

	dev_set_name(&master->dev, "shared-mdio-master%d", master->dev_num);

	of_node_get(node);
	master->dev.of_node = node;

	return master;

fail2:
	kfree(master);
fail1:
	return ERR_PTR(ret);
}
EXPORT_SYMBOL(shared_mdio_alloc_master);

int shared_mdio_add_master(struct shared_mdio_master *master)
{
	if (!master)
		return -EINVAL;

	return device_add(&master->dev);
}
EXPORT_SYMBOL(shared_mdio_add_master);

static int shared_mdio_driver_probe(struct device *dev)
{
	int rc;
	struct shared_mdio_master *master = to_shared_mdio_master(dev);
	struct shared_mdio_driver *drv = to_shared_mdio_driver(dev->driver);

	if (!drv->probe)
		return -ENODEV;

	rc = drv->probe(master);
	if (rc)
		return rc;

	return 0;
}

static int shared_mdio_driver_remove(struct device *dev)
{
	struct shared_mdio_driver *drv = to_shared_mdio_driver(dev->driver);

	if (drv->remove)
		return drv->remove(to_shared_mdio_master(dev));

	return 0;
}

static void shared_mdio_driver_shutdown(struct device *dev)
{
	struct shared_mdio_driver *drv = to_shared_mdio_driver(dev->driver);

	if (drv->shutdown)
		drv->shutdown(to_shared_mdio_master(dev));
}

int __shared_mdio_register_driver(struct shared_mdio_driver *drv,
				  struct module *owner)
{
	int rc;

	drv->driver.bus = &shared_mdio_bus;
	drv->driver.owner = owner;
	drv->driver.probe = shared_mdio_driver_probe;
	drv->driver.remove = shared_mdio_driver_remove;
	drv->driver.shutdown = shared_mdio_driver_shutdown;

	rc = driver_register(&drv->driver);
	if (rc) {
		pr_err("driver_register() failed for %s, error: %d\n",
			drv->driver.name, rc);
		return rc;
	}

	return 0;
}
EXPORT_SYMBOL(__shared_mdio_register_driver);

static int shared_mdio_bus_match(struct device *dev, struct device_driver *drv)
{
	/* Attempt an OF style match */
	if (of_driver_match_device(dev, drv))
		return 1;

	return 0;
}

struct bus_type shared_mdio_bus = {
	.name	= "shared_mdio",
	.match	= shared_mdio_bus_match,
};
EXPORT_SYMBOL(shared_mdio_bus);

static int __init shared_mdio_init(void)
{
	int rc;

	rc = bus_register(&shared_mdio_bus);
	if (rc) {
		pr_err("Failed to register shared_mdio bus, error: %d\n", rc);
		return rc;
	}

	return 0;
}

static void __exit shared_mdio_exit(void)
{
	bus_unregister(&shared_mdio_bus);

	ida_destroy(&shared_mdio_ida);
}

subsys_initcall(shared_mdio_init);
module_exit(shared_mdio_exit);

MODULE_DESCRIPTION("Shared MDIO Bus");
MODULE_AUTHOR("Anup Patel <anup.patel@broadcom.com>");
MODULE_AUTHOR("Pramod Kumar <pramod.kumar@broadcom.com>");
MODULE_LICENSE("GPL v2");
