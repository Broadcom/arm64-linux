/*
 * Copyright (C) 2014-2015 Broadcom Corporation
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

#ifndef _PCIE_IPROC_H
#define _PCIE_IPROC_H

#define IPROC_PCIE_MAX_NUM_IRQS 6

/**
 * iProc PCIe outbound mapping
 * @set_oarr_size: indicates the OARR size bit needs to be set
 * @axi_offset: offset from the AXI address to the internal address used by
 * the iProc PCIe core
 * @window_size: outbound window size
 */
struct iproc_pcie_ob {
	bool set_oarr_size;
	resource_size_t axi_offset;
	resource_size_t window_size;
};

/**
 * iProc PCIe device
 *
 * @dev: pointer to device data structure
 * @base: PCIe host controller I/O register base
 * @resources: linked list of all PCI resources
 * @sysdata: Per PCI controller data
 * @root_bus: pointer to root bus
 * @phy: optional PHY device that controls the Serdes
 * @need_ob_cfg: indicates SW needs to configure the outbound mapping window
 * @ob: outbound mapping parameters
 */
struct iproc_pcie {
	struct device *dev;
	void __iomem *base;
	struct list_head *resources;
#ifdef CONFIG_ARM
	struct pci_sys_data sysdata;
#endif
	struct pci_bus *root_bus;
	struct phy *phy;
	bool need_ob_cfg;
	struct iproc_pcie_ob ob;
};

int iproc_pcie_setup(struct iproc_pcie *pcie);
int iproc_pcie_remove(struct iproc_pcie *pcie);
struct msi_controller *iproc_pcie_msi_init(struct iproc_pcie *pcie,
					   struct device_node *node);

#endif /* _PCIE_IPROC_H */
