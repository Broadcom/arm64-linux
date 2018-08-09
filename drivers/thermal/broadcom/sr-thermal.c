// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 Broadcom
 */

#include <linux/acpi.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>

#define TMON_CRIT_TEMP          105000 /* temp in millidegree C */
#define SR_TMON_MAX_LIST        6

/*
 * In stingray thermal IO memory,
 * Total Number of available TMONs MASK is at offset 0
 * temperature registers BASE is at 4 byte offset.
 * Each TMON temperature register size is 4.
 */
#define SR_TMON_TEMP_BASE(id)   ((id) * 0x4)

static const char * const sr_tmon_names[SR_TMON_MAX_LIST] = {
	"sr_tmon_ihost0",
	"sr_tmon_ihost1",
	"sr_tmon_ihost2",
	"sr_tmon_ihost3",
	"sr_tmon_crmu",
	"sr_tmon_nitro",
};

struct sr_tmon {
	struct thermal_zone_device *tz;
	unsigned int crit_temp;
	unsigned int tmon_id;
	struct sr_thermal *priv;
};

struct sr_thermal {
	struct device *dev;
	void __iomem *regs;
	struct sr_tmon tmon[SR_TMON_MAX_LIST];
};

static int sr_get_temp(struct thermal_zone_device *tz, int *temp)
{
	struct sr_tmon *tmon = tz->devdata;
	struct sr_thermal *sr_thermal = tmon->priv;

	*temp = readl(sr_thermal->regs + SR_TMON_TEMP_BASE(tmon->tmon_id));

	return 0;
}

static int sr_get_trip_type(struct thermal_zone_device *tz, int trip,
					enum thermal_trip_type *type)
{
	struct sr_tmon *tmon = tz->devdata;
	struct sr_thermal *sr_thermal = tmon->priv;

	switch (trip) {
	case 0:
		*type = THERMAL_TRIP_CRITICAL;
		break;
	default:
		dev_dbg(sr_thermal->dev,
			"Driver does not support more than 1 trip point\n");
		return -EINVAL;
	}
	return 0;
}

static int sr_get_trip_temp(struct thermal_zone_device *tz, int trip, int *temp)
{
	struct sr_tmon *tmon = tz->devdata;
	struct sr_thermal *sr_thermal = tmon->priv;

	switch (trip) {
	case 0:
		*temp = tmon->crit_temp;
		break;
	default:
		dev_dbg(sr_thermal->dev,
			"Driver does not support more than 1 trip point\n");
		return -EINVAL;
	}
	return 0;
}

static int sr_set_trip_temp(struct thermal_zone_device *tz, int trip, int temp)
{
	struct sr_tmon *tmon = tz->devdata;

	switch (trip) {
	case 0:
		/*
		 * Allow the user to change critical temperature
		 * as per their requirement, could be for debug
		 * purpose, even if it's more than the recommended
		 * critical temperature.
		 */
		tmon->crit_temp = temp;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static struct thermal_zone_device_ops sr_thermal_ops = {
	.get_temp = sr_get_temp,
	.get_trip_type = sr_get_trip_type,
	.get_trip_temp = sr_get_trip_temp,
	.set_trip_temp = sr_set_trip_temp,
};

static int sr_thermal_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sr_thermal *sr_thermal;
	struct sr_tmon *tmon;
	struct resource *res;
	uint32_t sr_tmon_list = 0;
	unsigned int i;
	int ret;

	sr_thermal = devm_kzalloc(dev, sizeof(*sr_thermal), GFP_KERNEL);
	if (!sr_thermal)
		return -ENOMEM;

	sr_thermal->dev = dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	sr_thermal->regs = (void __iomem *)devm_memremap(&pdev->dev, res->start,
					 resource_size(res), MEMREMAP_WB);
	if (IS_ERR(sr_thermal->regs)) {
		dev_err(dev, "failed to get io address\n");
		return PTR_ERR(sr_thermal->regs);
	}

	ret = device_property_read_u32(dev, "brcm,tmon-mask", &sr_tmon_list);
	if (ret)
		return ret;

	for (i = 0; i < SR_TMON_MAX_LIST; i++) {

		if (!(sr_tmon_list & BIT(i)))
			continue;

		/* Flush temperature registers */
		writel(0, sr_thermal->regs + SR_TMON_TEMP_BASE(i));
		tmon = &sr_thermal->tmon[i];
		tmon->crit_temp = TMON_CRIT_TEMP;
		tmon->tmon_id = i;
		tmon->priv = sr_thermal;
		tmon->tz = thermal_zone_device_register(sr_tmon_names[i],
				1, 1,
				tmon,
				&sr_thermal_ops,
				NULL, 1000, 1000);
		if (IS_ERR(tmon->tz))
			goto err_exit;

		dev_info(dev, "%s: registered\n", sr_tmon_names[i]);
	}
	platform_set_drvdata(pdev, sr_thermal);

	return 0;

err_exit:
	while (--i >= 0) {
		if (sr_thermal->tmon[i].tz)
			thermal_zone_device_unregister(sr_thermal->tmon[i].tz);
	}

	return PTR_ERR(tmon->tz);
}

static int sr_thermal_remove(struct platform_device *pdev)
{
	struct sr_thermal *sr_thermal = platform_get_drvdata(pdev);
	unsigned int i;

	for (i = 0; i < SR_TMON_MAX_LIST; i++)
		if (sr_thermal->tmon[i].tz)
			thermal_zone_device_unregister(sr_thermal->tmon[i].tz);

	return 0;
}

static const struct of_device_id sr_thermal_of_match[] = {
	{ .compatible = "brcm,sr-thermal", },
	{},
};
MODULE_DEVICE_TABLE(of, sr_thermal_of_match);

static const struct acpi_device_id sr_thermal_acpi_ids[] = {
	{ .id = "BRCM0500" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(acpi, sr_thermal_acpi_ids);

static struct platform_driver sr_thermal_driver = {
	.probe		= sr_thermal_probe,
	.remove		= sr_thermal_remove,
	.driver = {
		.name = "sr-thermal",
		.of_match_table = sr_thermal_of_match,
		.acpi_match_table = ACPI_PTR(sr_thermal_acpi_ids),
	},
};
module_platform_driver(sr_thermal_driver);

MODULE_AUTHOR("Pramod Kumar <pramod.kumar@broadcom.com>");
MODULE_DESCRIPTION("Stingray thermal driver");
MODULE_LICENSE("GPL v2");
