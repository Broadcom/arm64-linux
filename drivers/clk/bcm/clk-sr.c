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

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>

#include <dt-bindings/clock/bcm-sr.h>
#include "clk-iproc.h"

#define REG_VAL(o, s, w) { .offset = o, .shift = s, .width = w, }

#define AON_VAL(o, pw, ps, is) { .offset = o, .pwr_width = pw, \
	.pwr_shift = ps, .iso_shift = is }

#define SW_CTRL_VAL(o, s) { .offset = o, .shift = s, }

#define RESET_VAL(o, rs, prs) { .offset = o, .reset_shift = rs, \
	.p_reset_shift = prs }

#define DF_VAL(o, kis, kiw, kps, kpw, kas, kaw) { .offset = o, .ki_shift = kis,\
	.ki_width = kiw, .kp_shift = kps, .kp_width = kpw, .ka_shift = kas,    \
	.ka_width = kaw }

#define VCO_CTRL_VAL(uo, lo) { .u_offset = uo, .l_offset = lo }

#define ENABLE_VAL(o, es, hs, bs) { .offset = o, .enable_shift = es, \
	.hold_shift = hs, .bypass_shift = bs }


static const struct iproc_pll_ctrl genpll0 = {
	.flags = IPROC_CLK_AON | IPROC_CLK_PLL_HAS_NDIV_FRAC |
		IPROC_CLK_PLL_NEEDS_SW_CFG,
	.aon = AON_VAL(0x0, 5, 1, 0),
	.reset = RESET_VAL(0x0, 12, 11),
	.dig_filter = DF_VAL(0x0, 4, 3, 0, 4, 7, 3),
	.sw_ctrl = SW_CTRL_VAL(0x10, 31),
	.ndiv_int = REG_VAL(0x10, 20, 10),
	.ndiv_frac = REG_VAL(0x10, 0, 20),
	.pdiv = REG_VAL(0x14, 0, 4),
	.status = REG_VAL(0x30, 12, 1),
};

static const struct iproc_clk_ctrl genpll0_clk[] = {
	[BCM_SR_GENPLL0_SATA_CLK] = {
		.channel = BCM_SR_GENPLL0_SATA_CLK,
		.flags = IPROC_CLK_AON,
		.enable = ENABLE_VAL(0x4, 6, 0, 12),
		.mdiv = REG_VAL(0x18, 0, 9),
	},
	[BCM_SR_GENPLL0_SCR_CLK] = {
		.channel = BCM_SR_GENPLL0_SCR_CLK,
		.flags = IPROC_CLK_AON,
		.enable = ENABLE_VAL(0x4, 7, 1, 13),
		.mdiv = REG_VAL(0x18, 10, 9),
	},
	[BCM_SR_GENPLL0_250M_CLK] = {
		.channel = BCM_SR_GENPLL0_250M_CLK,
		.flags = IPROC_CLK_AON,
		.enable = ENABLE_VAL(0x4, 8, 2, 14),
		.mdiv = REG_VAL(0x18, 20, 9),
	},
	[BCM_SR_GENPLL0_PCIE_AXI_CLK] = {
		.channel = BCM_SR_GENPLL0_PCIE_AXI_CLK,
		.flags = IPROC_CLK_AON,
		.enable = ENABLE_VAL(0x4, 9, 3, 15),
		.mdiv = REG_VAL(0x1c, 0, 9),
	},
	[BCM_SR_GENPLL0_PAXC_AXI_X2_CLK] = {
		.channel = BCM_SR_GENPLL0_PAXC_AXI_X2_CLK,
		.flags = IPROC_CLK_AON,
		.enable = ENABLE_VAL(0x4, 10, 4, 16),
		.mdiv = REG_VAL(0x1c, 10, 9),
	},
	[BCM_SR_GENPLL0_PAXC_AXI_CLK] = {
		.channel = BCM_SR_GENPLL0_PAXC_AXI_CLK,
		.flags = IPROC_CLK_AON,
		.enable = ENABLE_VAL(0x4, 11, 5, 17),
		.mdiv = REG_VAL(0x1c, 20, 9),
	},
};

static void __init sr_genpll0_clk_init(struct device_node *node)
{
	iproc_pll_clk_setup(node, &genpll0, NULL, 0, genpll0_clk,
				ARRAY_SIZE(genpll0_clk));
}
CLK_OF_DECLARE(sr_genpll0_clk, "brcm,sr-genpll0",
				sr_genpll0_clk_init);

static const struct iproc_pll_ctrl genpll3 = {
	.flags = IPROC_CLK_AON | IPROC_CLK_PLL_HAS_NDIV_FRAC |
		IPROC_CLK_PLL_NEEDS_SW_CFG,
	.aon = AON_VAL(0x0, 1, 19, 18),
	.reset = RESET_VAL(0x0, 12, 11),
	.dig_filter = DF_VAL(0x0, 4, 3, 0, 4, 7, 3),
	.sw_ctrl = SW_CTRL_VAL(0x10, 31),
	.ndiv_int = REG_VAL(0x10, 20, 10),
	.ndiv_frac = REG_VAL(0x10, 0, 20),
	.pdiv = REG_VAL(0x14, 0, 4),
	.status = REG_VAL(0x30, 12, 1),
};

static const struct iproc_clk_ctrl genpll3_clk[] = {
	[BCM_SR_GENPLL3_HSLS_CLK] = {
		.channel = BCM_SR_GENPLL3_HSLS_CLK,
		.flags = IPROC_CLK_AON,
		.enable = ENABLE_VAL(0x4, 6, 0, 12),
		.mdiv = REG_VAL(0x18, 0, 9),
	},
	[BCM_SR_GENPLL3_SDIO_CLK] = {
		.channel = BCM_SR_GENPLL3_SDIO_CLK,
		.flags = IPROC_CLK_AON,
		.enable = ENABLE_VAL(0x4, 7, 1, 13),
		.mdiv = REG_VAL(0x18, 10, 9),
	},
};

static void __init sr_genpll3_clk_init(struct device_node *node)
{
	iproc_pll_clk_setup(node, &genpll3, NULL, 0, genpll3_clk,
				ARRAY_SIZE(genpll3_clk));
}
CLK_OF_DECLARE(sr_genpll3_clk, "brcm,sr-genpll3",
			sr_genpll3_clk_init);

static const struct iproc_pll_ctrl genpll4 = {
	.flags = IPROC_CLK_AON | IPROC_CLK_PLL_HAS_NDIV_FRAC |
		IPROC_CLK_PLL_NEEDS_SW_CFG,
	.aon = AON_VAL(0x0, 1, 25, 24),
	.reset = RESET_VAL(0x0, 12, 11),
	.dig_filter = DF_VAL(0x0, 4, 3, 0, 4, 7, 3),
	.sw_ctrl = SW_CTRL_VAL(0x10, 31),
	.ndiv_int = REG_VAL(0x10, 20, 10),
	.ndiv_frac = REG_VAL(0x10, 0, 20),
	.pdiv = REG_VAL(0x14, 0, 4),
	.status = REG_VAL(0x30, 12, 1),
};

static const struct iproc_clk_ctrl genpll4_clk[] = {
	[BCM_SR_GENPLL4_CCN_CLK] = {
		.channel = BCM_SR_GENPLL4_CCN_CLK,
		.flags = IPROC_CLK_AON,
		.enable = ENABLE_VAL(0x4, 6, 0, 12),
		.mdiv = REG_VAL(0x18, 0, 9),
	},
};

static void __init sr_genpll4_clk_init(struct device_node *node)
{
	iproc_pll_clk_setup(node, &genpll4, NULL, 0, genpll4_clk,
				ARRAY_SIZE(genpll4_clk));
}
CLK_OF_DECLARE(sr_genpll4_clk, "brcm,sr-genpll4",
			sr_genpll4_clk_init);

static const struct iproc_pll_ctrl genpll5 = {
	.flags = IPROC_CLK_AON | IPROC_CLK_PLL_HAS_NDIV_FRAC |
		IPROC_CLK_PLL_NEEDS_SW_CFG,
	.aon = AON_VAL(0x0, 1, 1, 0),
	.reset = RESET_VAL(0x0, 12, 11),
	.dig_filter = DF_VAL(0x0, 4, 3, 0, 4, 7, 3),
	.sw_ctrl = SW_CTRL_VAL(0x10, 31),
	.ndiv_int = REG_VAL(0x10, 20, 10),
	.ndiv_frac = REG_VAL(0x10, 0, 20),
	.pdiv = REG_VAL(0x14, 0, 4),
	.status = REG_VAL(0x30, 12, 1),
};

static const struct iproc_clk_ctrl genpll5_clk[] = {
	[BCM_SR_GENPLL5_FS_CLK] = {
		.channel = BCM_SR_GENPLL5_FS_CLK,
		.flags = IPROC_CLK_AON,
		.enable = ENABLE_VAL(0x4, 6, 0, 12),
		.mdiv = REG_VAL(0x18, 0, 9),
	},
	[BCM_SR_GENPLL5_SPU_CLK] = {
		.channel = BCM_SR_GENPLL5_SPU_CLK,
		.flags = IPROC_CLK_AON,
		.enable = ENABLE_VAL(0x4, 6, 0, 12),
		.mdiv = REG_VAL(0x18, 10, 9),
	},
};

static void __init sr_genpll5_clk_init(struct device_node *node)
{
	iproc_pll_clk_setup(node, &genpll5, NULL, 0, genpll5_clk,
				ARRAY_SIZE(genpll5_clk));
}
CLK_OF_DECLARE(sr_genpll5_clk, "brcm,sr-genpll5",
		sr_genpll5_clk_init);

static const struct iproc_pll_ctrl lcpll0 = {
	.flags = IPROC_CLK_AON | IPROC_CLK_PLL_NEEDS_SW_CFG,
	.aon = AON_VAL(0x0, 2, 19, 18),
	.reset = RESET_VAL(0x0, 31, 30),
	.sw_ctrl = SW_CTRL_VAL(0x4, 31),
	.ndiv_int = REG_VAL(0x4, 16, 10),
	.pdiv = REG_VAL(0x4, 26, 4),
	.status = REG_VAL(0x38, 12, 1),
};

static const struct iproc_clk_ctrl lcpll0_clk[] = {
	[BCM_SR_LCPLL0_SATA_REF_CLK] = {
		.channel = BCM_SR_LCPLL0_SATA_REF_CLK,
		.flags = IPROC_CLK_AON,
		.enable = ENABLE_VAL(0x0, 7, 1, 13),
		.mdiv = REG_VAL(0x14, 0, 9),
	},
	[BCM_SR_LCPLL0_USB_REF_CLK] = {
		.channel = BCM_SR_LCPLL0_USB_REF_CLK,
		.flags = IPROC_CLK_AON,
		.enable = ENABLE_VAL(0x0, 8, 2, 14),
		.mdiv = REG_VAL(0x14, 10, 9),
	},
	[BCM_SR_LCPLL0_SATA_REFPN_CLK] = {
		.channel = BCM_SR_LCPLL0_SATA_REFPN_CLK,
		.flags = IPROC_CLK_AON,
		.enable = ENABLE_VAL(0x0, 9, 3, 15),
		.mdiv = REG_VAL(0x14, 20, 9),
	},
};

static void __init sr_lcpll0_clk_init(struct device_node *node)
{
	iproc_pll_clk_setup(node, &lcpll0, NULL, 0, lcpll0_clk,
				ARRAY_SIZE(lcpll0_clk));
}
CLK_OF_DECLARE(sr_lcpll0_clk, "brcm,sr-lcpll0",
			sr_lcpll0_clk_init);

static const struct iproc_pll_ctrl lcpll1 = {
	.flags = IPROC_CLK_AON | IPROC_CLK_PLL_NEEDS_SW_CFG,
	.aon = AON_VAL(0x0, 2, 22, 21),
	.reset = RESET_VAL(0x0, 31, 30),
	.sw_ctrl = SW_CTRL_VAL(0x4, 31),
	.ndiv_int = REG_VAL(0x4, 16, 10),
	.pdiv = REG_VAL(0x4, 26, 4),
	.status = REG_VAL(0x38, 12, 1),
};

static const struct iproc_clk_ctrl lcpll1_clk[] = {
	[BCM_SR_LCPLL1_WAN_CLK] = {
		.channel = BCM_SR_LCPLL1_WAN_CLK,
		.flags = IPROC_CLK_AON,
		.enable = ENABLE_VAL(0x0, 7, 1, 13),
		.mdiv = REG_VAL(0x14, 0, 9),
	},
};

static void __init sr_lcpll1_clk_init(struct device_node *node)
{
	iproc_pll_clk_setup(node, &lcpll1, NULL, 0, lcpll1_clk,
				ARRAY_SIZE(lcpll1_clk));
}
CLK_OF_DECLARE(sr_lcpll1_clk, "brcm,sr-lcpll1",
			sr_lcpll1_clk_init);

static const struct iproc_pll_ctrl lcpll_pcie = {
	.flags = IPROC_CLK_AON | IPROC_CLK_PLL_NEEDS_SW_CFG,
	.aon = AON_VAL(0x0, 2, 25, 24),
	.reset = RESET_VAL(0x0, 31, 30),
	.sw_ctrl = SW_CTRL_VAL(0x4, 31),
	.ndiv_int = REG_VAL(0x4, 16, 10),
	.pdiv = REG_VAL(0x4, 26, 4),
	.status = REG_VAL(0x38, 12, 1),
};

static const struct iproc_clk_ctrl lcpll_pcie_clk[] = {
	[BCM_SR_LCPLL_PCIE_PHY_REF_CLK] = {
		.channel = BCM_SR_LCPLL_PCIE_PHY_REF_CLK,
		.flags = IPROC_CLK_AON,
		.enable = ENABLE_VAL(0x0, 7, 1, 13),
		.mdiv = REG_VAL(0x14, 0, 9),
	},
};

static void __init sr_lcpll_pcie_clk_init(struct device_node *node)
{
	iproc_pll_clk_setup(node, &lcpll_pcie, NULL, 0, lcpll_pcie_clk,
		ARRAY_SIZE(lcpll_pcie_clk));
}
CLK_OF_DECLARE(sr_lcpll_pcie_clk, "brcm,sr-lcpll-pcie",
		sr_lcpll_pcie_clk_init);
