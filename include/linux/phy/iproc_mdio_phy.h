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

#ifndef __IPROC_MDIO_PHY_H
#define __IPROC_MDIO_PHY_H

int iproc_mdio_read(unsigned clk_div, unsigned phy_addr, unsigned reg_addr,
		    u16 *data);
int iproc_mdio_write(unsigned clk_div, unsigned phy_addr, unsigned reg_addr,
		     u16 data);

#endif /* __IPROC_MDIO_PHY_H */
