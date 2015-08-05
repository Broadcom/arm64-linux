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

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/msi.h>
#include <linux/of_irq.h>
#include <linux/of_pci.h>
#include <linux/pci.h>
#include <linux/platform_device.h>

#include "pcie-iproc.h"

#define SYS_EQ_PAGE_OFFSET           0x200
#define SYS_MSI_PAGE_OFFSET          0x204
#define SYS_MSI_INTS_EN_OFFSET       0x208
#define SYS_MSI_CTRL_0_OFFSET        0x210
#define SYS_MSI_INTR_EN_SHIFT        11
#define SYS_MSI_INTR_EN              BIT(SYS_MSI_INTR_EN_SHIFT)
#define SYS_MSI_INT_N_EVENT_SHIFT    1
#define SYS_MSI_INT_N_EVENT          BIT(SYS_MSI_INT_N_EVENT_SHIFT)
#define SYS_MSI_EQ_EN_SHIFT          0
#define SYS_MSI_EQ_EN                BIT(SYS_MSI_EQ_EN_SHIFT)

#define SYS_EQ_HEAD_0_OFFSET         0x250
#define SYS_EQ_TAIL_0_OFFSET         0x254
#define SYS_EQ_MASK                  0x3f

#define SYS_MSI_CTRL(eq)             (SYS_MSI_CTRL_0_OFFSET + (eq) * 4)
#define SYS_EQ_HEAD(eq)              (SYS_EQ_HEAD_0_OFFSET + (eq) * 8)
#define SYS_EQ_TAIL(eq)              (SYS_EQ_TAIL_0_OFFSET + (eq) * 8)

#define SYS_EQ_HEAD_MASK             0x3f
#define SYS_EQ_TAIL_MASK             0x3f

#define SYS_EQ_SIZE                  64

/**
 * iProc event queue based MSI
 *
 * Only meant to be used on legacy platforms without MSI support integrated
 * into the GIC
 *
 * @pcie: pointer to iProc PCIe data
 * @chip: MSI controller
 * @irqs: pointer to an array that contains the interrupt IDs
 * @nirqs: number of total interrupts
 * @has_inten_reg: indicates the MSI interrupt enable register needs to be
 * set explicitly
 * @used: bitmap to track usage of MSI
 * @domain: IRQ domain
 * @bitmap_lock: lock to protect access to the IRQ bitmap
 * @eq_page: page memory for the event queue
 * @msi_page: page memory for MIS posted writes
 */
struct iproc_msi {
	struct iproc_pcie *pcie;
	struct msi_controller chip;
	int *irqs;
	int nirqs;
	bool has_inten_reg;
	DECLARE_BITMAP(used, IPROC_PCIE_MAX_NUM_IRQS);
	struct irq_domain *domain;
	struct mutex bitmap_lock;
	unsigned long eq_page;
	unsigned long msi_page;
};

static inline struct iproc_msi *to_iproc_msi(struct msi_controller *chip)
{
	return container_of(chip, struct iproc_msi, chip);
}

static int iproc_msi_alloc(struct iproc_msi *msi)
{
	int irq;

	mutex_lock(&msi->bitmap_lock);

	irq = find_first_zero_bit(msi->used, msi->nirqs);
	if (irq < msi->nirqs)
		set_bit(irq, msi->used);
	else
		irq = -ENOSPC;

	mutex_unlock(&msi->bitmap_lock);

	return irq;
}

static void iproc_msi_free(struct iproc_msi *msi, unsigned long irq)
{
	struct device *dev = msi->chip.dev;

	mutex_lock(&msi->bitmap_lock);

	if (!test_bit(irq, msi->used))
		dev_warn(dev, "trying to free unused MSI IRQ#%lu\n", irq);
	else
		clear_bit(irq, msi->used);

	mutex_unlock(&msi->bitmap_lock);
}

static int iproc_msi_setup_irq(struct msi_controller *chip,
			       struct pci_dev *pdev, struct msi_desc *desc)
{
	struct iproc_msi *msi = to_iproc_msi(chip);
	struct msi_msg msg;
	unsigned int irq;
	int hwirq;

	hwirq = iproc_msi_alloc(msi);
	if (hwirq < 0)
		return hwirq;

	irq = irq_create_mapping(msi->domain, hwirq);
	if (!irq) {
		iproc_msi_free(msi, hwirq);
		return -EINVAL;
	}

	irq_set_msi_desc(irq, desc);

	msg.address_lo = virt_to_phys((void *)msi->msi_page) | (hwirq * 4);
	msg.address_hi = 0;
	msg.data = hwirq;

	pci_write_msi_msg(irq, &msg);

	return 0;
}

static void iproc_msi_teardown_irq(struct msi_controller *chip,
				   unsigned int irq)
{
	struct iproc_msi *msi = to_iproc_msi(chip);
	struct irq_data *d = irq_get_irq_data(irq);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);

	irq_dispose_mapping(irq);
	iproc_msi_free(msi, hwirq);
}

static struct irq_chip iproc_msi_irq_chip = {
	.name = "iProc PCIe MSI",
	.irq_enable = pci_msi_unmask_irq,
	.irq_disable = pci_msi_mask_irq,
	.irq_mask = pci_msi_mask_irq,
	.irq_unmask = pci_msi_unmask_irq,
};

static int iproc_msi_map(struct irq_domain *domain, unsigned int irq,
			 irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &iproc_msi_irq_chip, handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);
	set_irq_flags(irq, IRQF_VALID);

	return 0;
}

static const struct irq_domain_ops msi_domain_ops = {
	.map = iproc_msi_map,
};

static void iproc_msi_enable(struct iproc_msi *msi)
{
	struct iproc_pcie *pcie = msi->pcie;
	int eq;
	u32 val;

	writel(virt_to_phys((void *)msi->eq_page),
	       pcie->base + SYS_EQ_PAGE_OFFSET);
	writel(virt_to_phys((void *)msi->msi_page),
	       pcie->base + SYS_MSI_PAGE_OFFSET);

	for (eq = 0; eq < msi->nirqs; eq++) {
		/* enable MSI event queue */
		val = SYS_MSI_INTR_EN | SYS_MSI_INT_N_EVENT | SYS_MSI_EQ_EN;
		writel(val, pcie->base + SYS_MSI_CTRL(eq));

		/*
		 * Some legacy platforms require the MSI interrupt enable
		 * register to be set explicitly
		 */
		if (msi->has_inten_reg) {
			val = readl(pcie->base + SYS_MSI_INTS_EN_OFFSET);
			val |= BIT(eq);
			writel(val, pcie->base + SYS_MSI_INTS_EN_OFFSET);
		}
	}
}

static void iproc_msi_handler(unsigned int irq, struct irq_desc *desc)
{
	struct irq_chip *irq_chip = irq_desc_get_chip(desc);
	struct iproc_msi *msi;
	struct iproc_pcie *pcie;
	u32 eq, head, tail, num_events;
	int virq;

	chained_irq_enter(irq_chip, desc);

	msi = irq_get_handler_data(irq);
	pcie = msi->pcie;

	eq = irq - msi->irqs[0];
	virq = irq_find_mapping(msi->domain, eq);
	head = readl(pcie->base + SYS_EQ_HEAD(eq)) & SYS_EQ_HEAD_MASK;
	do {
		tail = readl(pcie->base + SYS_EQ_TAIL(eq)) & SYS_EQ_TAIL_MASK;

		num_events = (tail < head) ?
			(SYS_EQ_SIZE - (head - tail)) : (tail - head);
		if (!num_events)
			break;

		generic_handle_irq(virq);

		head++;
		head %= SYS_EQ_SIZE;
		writel(head, pcie->base + SYS_EQ_HEAD(eq));
	} while (true);

	chained_irq_exit(irq_chip, desc);
}

struct msi_controller *iproc_pcie_msi_init(struct iproc_pcie *pcie,
					   struct device_node *node)
{
	struct iproc_msi *msi;
	struct msi_controller *chip;
	int i, ret;

	if (!of_find_property(node, "msi-controller", NULL))
		return ERR_PTR(-ENODEV);

	msi = devm_kzalloc(pcie->dev, sizeof(*msi), GFP_KERNEL);
	if (!msi)
		return ERR_PTR(-ENOMEM);
	msi->pcie = pcie;

	mutex_init(&msi->bitmap_lock);

	if (of_find_property(node, "brcm,pcie-msi-inten", NULL))
		msi->has_inten_reg = true;

	msi->nirqs = of_irq_count(node);
	if (!msi->nirqs) {
		dev_err(pcie->dev, "found no MSI interrupt in DT\n");
		return ERR_PTR(-ENODEV);
	}
	if (msi->nirqs > IPROC_PCIE_MAX_NUM_IRQS) {
		dev_warn(pcie->dev, "too many MSI interrupts defined %d\n",
			 msi->nirqs);
		msi->nirqs = IPROC_PCIE_MAX_NUM_IRQS;
	}
	msi->irqs = devm_kcalloc(pcie->dev, msi->nirqs, sizeof(*msi->irqs),
				 GFP_KERNEL);
	if (!msi->irqs)
		return ERR_PTR(-ENOMEM);

	for (i = 0; i < msi->nirqs; i++) {
		msi->irqs[i] = irq_of_parse_and_map(node, i);
		if (!msi->irqs[i]) {
			dev_err(pcie->dev, "unable to parse/map interrupt\n");
			return ERR_PTR(-ENODEV);
		}
	}

	chip = &msi->chip;
	chip->dev = pcie->dev;
	chip->setup_irq = iproc_msi_setup_irq;
	chip->teardown_irq = iproc_msi_teardown_irq;
	msi->domain = irq_domain_add_linear(pcie->dev->of_node, msi->nirqs,
					    &msi_domain_ops, chip);
	if (!msi->domain) {
		dev_err(pcie->dev, "failed to create IRQ domain\n");
		return ERR_PTR(-ENXIO);
	}

	/* reserve page for iProc MSI event queue */
	msi->eq_page = __get_free_pages(GFP_KERNEL, 0);
	if (!msi->eq_page) {
		dev_err(pcie->dev,
			"failed to allocate memory for MSI event queue\n");
		ret = -ENOMEM;
		goto err_rm_irq_domain;
	}

	/* reserve page for MSI post writes */
	msi->msi_page = __get_free_pages(GFP_KERNEL, 0);
	if (!msi->msi_page) {
		dev_err(pcie->dev, "failed to allocate memory for MSI\n");
		ret = -ENOMEM;
		goto err_free_eq_page;
	}

	for (i = 0; i < msi->nirqs; i++) {
		irq_set_handler_data(msi->irqs[i], msi);
		irq_set_chained_handler(msi->irqs[i], iproc_msi_handler);
	}

	iproc_msi_enable(msi);

	return chip;

err_free_eq_page:
	free_pages(msi->eq_page, 0);

err_rm_irq_domain:
	irq_domain_remove(msi->domain);

	return ERR_PTR(ret);
}
EXPORT_SYMBOL(iproc_pcie_msi_init);
