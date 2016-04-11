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

#ifndef __SHARED_MDIO_H__
#define __SHARED_MDIO_H__

#include <linux/device.h>
#include <linux/types.h>

extern struct bus_type shared_mdio_bus;

/*
 * This structure represets the mdio master that control the MDIO bus
 * to access the phy attached on it.
 * @dev Underlying device for mdio master
 * @dev_num Unique device number of mdio master
 * @mdio_write Write callback of mdio master
 * @mdio_read Read callback for mdio master
 */
struct shared_mdio_master {
	struct device dev;
	int dev_num;
	void *priv;

	int (*mdio_write)(struct shared_mdio_master *master,
				u16 phyid, u16 reg, u16 data);
	int (*mdio_read)(struct shared_mdio_master *master, u16 phyid, u16 reg);
};
#define to_shared_mdio_master(d)		\
			container_of(d, struct shared_mdio_master, dev)

struct shared_mdio_driver {
	int (*probe)(struct shared_mdio_master *master);
	int (*remove)(struct shared_mdio_master *master);
	void (*shutdown)(struct shared_mdio_master *master);

	struct device_driver driver;
};
#define to_shared_mdio_driver(d)	\
			container_of(d, struct shared_mdio_driver, driver)

struct shared_mdio_master *shared_mdio_alloc_master(struct device *parent,
						    struct device_node *node);

int shared_mdio_add_master(struct shared_mdio_master *master);

static inline void shared_mdio_remove_master(struct shared_mdio_master *master)
{
		device_unregister(&master->dev);
}

int __shared_mdio_register_driver(struct shared_mdio_driver *drv,
				  struct module *owner);

/* use a define to avoid include chaining to get THIS_MODULE & friends */
#define shared_mdio_register_driver(drv) \
	__shared_mdio_register_driver(drv, THIS_MODULE)

static inline void shared_mdio_unregister_driver(
					struct shared_mdio_driver *drv)
{
		driver_unregister(&drv->driver);
}

/**
 * module_shared_mdio_driver() - Helper macro for registering a shared_mdio
 * driver
 * @__shared_mdio_driver: shared_mdio_driver struct
 *
 * Helper macro for shared mdio drivers which do not do anything special in
 * module init/exit. This eliminates a lot of boilerplate. Each module
 * may only use this macro once, and calling it replaces module_init()
 * and module_exit().
 */
#define module_shared_mdio_driver(__shared_mdio_driver) \
	module_driver(__shared_mdio_driver, shared_mdio_register_driver, \
		       shared_mdio_unregister_driver)


static inline int shared_mdio_write(struct shared_mdio_master *master,
						u16 phy, u16 reg, u16 data)
{
	if (master->mdio_write)
		return master->mdio_write(master, phy, reg, data);

	return -ENOTSUPP;
}

static inline int shared_mdio_read(struct shared_mdio_master *master,
							u16 phy_id, u16 reg)
{
	if (master->mdio_read)
		return master->mdio_read(master, phy_id, reg);

	return -ENOTSUPP;
}

/*
 * Use the following functions to manipulate shared_mdio's per-master
 * driver-specific data.
 */
static inline void *shared_mdio_get_drvdata(struct shared_mdio_master *master)
{
	return dev_get_drvdata(&master->dev);
}

static inline void shared_mdio_set_drvdata(struct shared_mdio_master *master,
					   void *data)
{
	dev_set_drvdata(&master->dev, data);
}

#endif
