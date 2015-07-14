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
 * iProc PCIe device
 * @sysdata: Per PCI controller data. This needs to be kept at the beginning of
 * struct iproc_pcie, to enable support of both ARM32 and ARM64 platforms with
 * minimal changes in the iProc PCIe core driver
 * @dev: pointer to device data structure
 * @base: PCIe host controller I/O register base
 * @resources: linked list of all PCI resources
 * @root_bus: pointer to root bus
 * @phy: optional PHY device that controls the Serdes
 * @irqs: interrupt IDs
 */
struct iproc_pcie {
#ifdef CONFIG_ARM
	struct pci_sys_data sysdata;
#endif
	struct device *dev;
	void __iomem *base;
	struct pci_bus *root_bus;
	struct phy *phy;
	int irqs[IPROC_PCIE_MAX_NUM_IRQS];
	int (*map_irq)(const struct pci_dev *, u8, u8);
};

int iproc_pcie_setup(struct iproc_pcie *pcie, struct list_head *res);
int iproc_pcie_remove(struct iproc_pcie *pcie);

#endif /* _PCIE_IPROC_H */
