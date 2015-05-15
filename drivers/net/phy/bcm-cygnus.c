/*
 * Copyright (C) 2015 Broadcom Corporation
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

/* Broadcom Cygnus SoC internal transceivers support. */

#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/brcmphy.h>
#include <linux/mdio.h>

/* Broadcom Cygnus Phy specific registers */
#define MII_BCM_CYGNUS_CORE_BASE1E       0x1E /* Core BASE1E register */
#define MII_BCM_CYGNUS_DSP_TAP10         0x0A /* DSP TAP10 register */
#define MII_BCM_CYGNUS_EXPB0             0xB0 /* EXPB0 register */
#define MII_BCM_CYGNUS_EXPB1             0xB1 /* EXPB1 register */

#define MII_BCM_CYGNUS_AFE_VDAC_ICTRL_0  0x91E5 /* VDAL Control register */

#define BCM_CYGNUS_APD_CLR_MASK          0xFF7F /* clear bits 5 & 6 */
#define BCM_CYGNUS_ANEG_APD_EN           0x0020 /* bits 5 & 6 */
#define BCM_CYGNUS_NO_ANEG_APD_EN        0x0060 /* bits 5 & 6 */
#define BCM_CYGNUS_APD_SINGLELP_EN       0x0100 /* Bit 8 */

#define BCM_CL45VEN_EEE_ADV              0x3c

#define MII_BCMCYGNUS_CHANNEL_WIDTH      0x2000

static int bcm_exp_write(struct phy_device *phydev, u16 reg, u16 val)
{
	int rc;

	rc = phy_write(phydev, MII_BCM54XX_EXP_SEL, reg);
	if (rc < 0)
		return rc;

	return phy_write(phydev, MII_BCM54XX_EXP_DATA, val);
}

static int bcm_misc_write(struct phy_device *phydev,
			  u16 reg, u16 chl, u16 value)
{
	int rc;
	int tmp;

	rc = phy_write(phydev, MII_BCM54XX_AUX_CTL,
		       MII_BCM54XX_AUXCTL_SHDWSEL_MISC);
	if (rc < 0)
		return rc;

	tmp = phy_read(phydev, MII_BCM54XX_AUX_CTL);
	tmp |= MII_BCM54XX_AUXCTL_ACTL_SMDSP_ENA;
	rc = phy_write(phydev, MII_BCM54XX_AUX_CTL, tmp);
	if (rc < 0)
		return rc;

	tmp = (chl * MII_BCMCYGNUS_CHANNEL_WIDTH) | reg;
	rc = phy_write(phydev, MII_BCM54XX_EXP_SEL, tmp);
	if (rc < 0)
		return rc;

	rc = phy_write(phydev, MII_BCM54XX_EXP_DATA, value);

	return rc;
}

static int bcm_cygnus_afe_config(struct phy_device *phydev)
{
	int rc;

	/* ensure smdspclk is enabled */
	rc = phy_write(phydev, MII_BCM54XX_AUX_CTL, 0x0c30);
	if (rc < 0)
		return rc;

	/* AFE_VDAC_ICTRL_0 bit 7:4 Iq=1100 for 1g 10bt, normal modes */
	rc = bcm_misc_write(phydev, 0x39, 0x01, 0xA7C8);
	if (rc < 0)
		return rc;

	/* AFE_HPF_TRIM_OTHERS bit11=1, short cascode enable for all modes*/
	rc = bcm_misc_write(phydev, 0x3A, 0x00, 0x0803);
	if (rc < 0)
		return rc;

	/* AFE_TX_CONFIG_1 bit 7:4 Iq=1100 for test modes */
	rc = bcm_misc_write(phydev, 0x3A, 0x01, 0xA740);
	if (rc < 0)
		return rc;

	/* AFE TEMPSEN_OTHERS rcal_HT, rcal_LT 10000 */
	rc = bcm_misc_write(phydev, 0x3A, 0x03, 0x8400);
	if (rc < 0)
		return rc;

	/* AFE_FUTURE_RSV bit 2:0 rccal <2:0>=100 */
	rc = bcm_misc_write(phydev, 0x3B, 0x00, 0x0004);
	if (rc < 0)
		return rc;

	/* Adjust bias current trim to overcome digital offSet */
	rc = phy_write(phydev, MII_BCM_CYGNUS_CORE_BASE1E, 0x02);
	if (rc < 0)
		return rc;

	/* make rcal=100, since rdb default is 000 */
	rc = bcm_exp_write(phydev, MII_BCM_CYGNUS_EXPB1, 0x10);
	if (rc < 0)
		return rc;

	/* CORE_EXPB0, Reset R_CAL/RC_CAL Engine */
	rc = bcm_exp_write(phydev, MII_BCM_CYGNUS_EXPB0, 0x10);
	if (rc < 0)
		return rc;

	/* CORE_EXPB0, Disable Reset R_CAL/RC_CAL Engine */
	rc = bcm_exp_write(phydev, MII_BCM_CYGNUS_EXPB0, 0x00);

	return 0;
}

static int bcm_cygnus_apd_config(struct phy_device *phydev)
{
	int val;

	val = bcm54xx_shadow_read(phydev, BCM54XX_SHD_APD);
	if (val < 0)
		return val;

	/* Clear APD bits */
	val &= BCM_CYGNUS_APD_CLR_MASK;

	if (AUTONEG_ENABLE == phydev->autoneg)
		val |= BCM_CYGNUS_ANEG_APD_EN;
	else
		val |= BCM_CYGNUS_NO_ANEG_APD_EN;

	/* Enable energy detect single link pulse for easy wakeup */
	val |= BCM_CYGNUS_APD_SINGLELP_EN;

	/* Enable Auto Power-Down (APD) for the PHY */
	return bcm54xx_shadow_write(phydev, BCM54XX_SHD_APD, val);
}

static int bcm_cygnus_eee_enable(struct phy_device *phydev)
{
	int val;

	/* Enable EEE at PHY level */
	val = phy_read_mmd_indirect(phydev, BRCM_CL45VEN_EEE_CONTROL,
				    MDIO_MMD_AN, phydev->addr);
	if (val < 0)
		return val;

	val |= LPI_FEATURE_EN | LPI_FEATURE_EN_DIG1000X;

	phy_write_mmd_indirect(phydev, BRCM_CL45VEN_EEE_CONTROL,
			       MDIO_MMD_AN,  phydev->addr, (u32)val);

	/* Advertise EEE */
	val = phy_read_mmd_indirect(phydev, BCM_CL45VEN_EEE_ADV,
				    MDIO_MMD_AN, phydev->addr);
	if (val < 0)
		return val;

	val |= (MDIO_AN_EEE_ADV_100TX | MDIO_AN_EEE_ADV_1000T);

	phy_write_mmd_indirect(phydev, BCM_CL45VEN_EEE_ADV,
			       MDIO_MMD_AN,  phydev->addr, (u32)val);

	return 0;
}

static int bcm_cygnus_config_init(struct phy_device *phydev)
{
	int reg, rc;

	reg = phy_read(phydev, MII_BCM54XX_ECR);
	if (reg < 0)
		return reg;

	/* Mask interrupts globally. */
	reg |= MII_BCM54XX_ECR_IM;
	rc = phy_write(phydev, MII_BCM54XX_ECR, reg);
	if (rc)
		return rc;

	/* Unmask events of interest */
	reg = ~(MII_BCM54XX_INT_DUPLEX |
		MII_BCM54XX_INT_SPEED |
		MII_BCM54XX_INT_LINK);
	rc = phy_write(phydev, MII_BCM54XX_IMR, reg);
	if (rc)
		return rc;

	/* Apply AFE settings for the PHY */
	rc = bcm_cygnus_afe_config(phydev);
	if (rc)
		return rc;

	/* Advertise EEE */
	rc = bcm_cygnus_eee_enable(phydev);
	if (rc)
		return rc;

	/* Enable APD */
	return bcm_cygnus_apd_config(phydev);
}

static int bcm_cygnus_ack_interrupt(struct phy_device *phydev)
{
	int reg;

	/* Clear pending interrupts.  */
	reg = phy_read(phydev, MII_BCM54XX_ISR);
	if (reg < 0)
		return reg;

	return 0;
}

static int bcm_cygnus_config_intr(struct phy_device *phydev)
{
	int reg;

	reg = phy_read(phydev, MII_BCM54XX_ECR);
	if (reg < 0)
		return reg;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED)
		reg &= ~MII_BCM54XX_ECR_IM;
	else
		reg |= MII_BCM54XX_ECR_IM;

	return phy_write(phydev, MII_BCM54XX_ECR, reg);
}

static int bcm_cygnus_resume(struct phy_device *phydev)
{
	int rc;
	struct net_device *ndev = phydev->attached_dev;

	if (ndev)
		/* Power on the PHY only if interface is running
		 * This check is required so that the PHY remains off
		 * if the interface is down while going to suspend
		 */
		if (netif_running(ndev))
			genphy_resume(phydev);

	/* Re-initialize the PHY to apply AFE work-arounds and
	 * configurations when coming out of suspend
	 */
	rc = bcm_cygnus_config_init(phydev);
	if (rc)
		return rc;

	/* restart auto negotiation with the new settings */
	return genphy_config_aneg(phydev);
}

static struct phy_driver bcm_cygnus_phy_driver[] = {
{
	.phy_id        = PHY_ID_BCM_CYGNUS,
	.phy_id_mask   = 0xfffffff0,
	.name          = "Broadcom Cygnus PHY",
	.features      = PHY_GBIT_FEATURES |
			SUPPORTED_Pause | SUPPORTED_Asym_Pause,
	.config_init   = bcm_cygnus_config_init,
	.config_aneg   = genphy_config_aneg,
	.read_status   = genphy_read_status,
	.ack_interrupt = bcm_cygnus_ack_interrupt,
	.config_intr   = bcm_cygnus_config_intr,
	.suspend       = genphy_suspend,
	.resume        = bcm_cygnus_resume,
} };

module_phy_driver(bcm_cygnus_phy_driver);

MODULE_DESCRIPTION("Broadcom Cygnus internal PHY driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Broadcom Corporation");
