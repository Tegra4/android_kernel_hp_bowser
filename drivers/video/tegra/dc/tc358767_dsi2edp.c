/*
 * drivers/video/tegra/dc/tc358767_dsi2edp.c
 *
 * Copyright (c) 2012-2013, NVIDIA CORPORATION. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/swab.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/bowser_resolution.h>

#include <mach/dc.h>

#include "dc_priv.h"
#include "tc358767_dsi2edp.h"
#include "dsi.h"

#define COLOR_BAR_TEST			0
#define PRBS7_PATTERN_TEST		0
#define D10_2_PATTERN_TEST		0
#define DBG_DUMP_REG			0

#if DBG_DUMP_REG
#define DUMP_REG(dsi2edp, reg, val)	\
	{	\
		tc358767_reg_read(dsi2edp, reg, &val); \
		printk("%s: %s 0x%08x\n", __func__, #reg, val);	\
	}
#else
#define DUMP_REG(dsi2edp, reg, val)	{}
#endif

extern int bowser_panel_resolution;

static struct tegra_dc_dsi2edp_data *tc358767_dsi2edp;
static struct i2c_client *tc358767_i2c_client;

enum i2c_transfer_type {
	I2C_WRITE,
	I2C_READ,
};

/*
 * TC358767 requires register address in big endian
 * and register value in little endian.
 * Regmap currently sends it all in big endian.
*/
#define TO_LITTLE_ENDIAN	(true)

static inline void tc358767_reg_write(struct tegra_dc_dsi2edp_data *dsi2edp,
					unsigned int addr, unsigned int val)
{
	regmap_write(dsi2edp->regmap, addr,
		TO_LITTLE_ENDIAN ? __swab32(val) : val);
}

static inline void tc358767_reg_read(struct tegra_dc_dsi2edp_data *dsi2edp,
					unsigned int addr, unsigned int *val)
{
	regmap_read(dsi2edp->regmap, addr, val);
	*val = TO_LITTLE_ENDIAN ? __swab32(*val) : *val;
}

static const struct regmap_config tc358767_regmap_config = {
	.reg_bits = 16,
	.val_bits = 32,
};

#define EDID_DUMP
#define EDID_SIZE 128
void tc358767_dsi2edp_get_edid(struct tegra_dc_dsi_data *dsi, char* buf){
	struct tegra_dc_dsi2edp_data *dsi2edp = tegra_dsi_get_outdata(dsi);
	int i = 0;
	//The 1-byte sum of all 128 bytes in this EDID block shall equal zero
	unsigned char checksum = 0;
	unsigned char AUO_EDID[EDID_SIZE];
	unsigned int* edid = (unsigned int*)AUO_EDID;

	memset(AUO_EDID, '\0', EDID_SIZE);

	mutex_lock(&dsi2edp->lock);

	tc358767_reg_write(dsi2edp, TC358767_AUX_CHANNEL_DPCD_ADDR, 0x00000050);
	tc358767_reg_write(dsi2edp, TC358767_AUX_CHANNEL_DPCD_WR_DATA0, 0x0);
	tc358767_reg_write(dsi2edp, TC358767_AUX_CHANNEL_CONFIG0, 0x00000104);

	//I2C read transaction, 16bytes requests in one transaction
	tc358767_reg_write(dsi2edp, TC358767_AUX_CHANNEL_CONFIG0, 0x00000F01);
	tc358767_reg_read(dsi2edp, TC358767_AUX_CHANNEL_DPCD_RD_DATA0, edid++);
	tc358767_reg_read(dsi2edp, TC358767_AUX_CHANNEL_DPCD_RD_DATA1, edid++);
	tc358767_reg_read(dsi2edp, TC358767_AUX_CHANNEL_DPCD_RD_DATA2, edid++);
	tc358767_reg_read(dsi2edp, TC358767_AUX_CHANNEL_DPCD_RD_DATA3, edid++);

	for(i = 16; i < EDID_SIZE; i += 16){
		//I2C read transaction, 16bytes requests in one transaction
		tc358767_reg_write(dsi2edp,TC358767_AUX_CHANNEL_CONFIG0, 0x00000F05);
		tc358767_reg_read(dsi2edp, TC358767_AUX_CHANNEL_DPCD_RD_DATA0, edid++);
		tc358767_reg_read(dsi2edp, TC358767_AUX_CHANNEL_DPCD_RD_DATA1, edid++);
		tc358767_reg_read(dsi2edp, TC358767_AUX_CHANNEL_DPCD_RD_DATA2, edid++);
		tc358767_reg_read(dsi2edp, TC358767_AUX_CHANNEL_DPCD_RD_DATA3, edid++);
	}
#if defined(EDID_DUMP)
	printk("EDID DUMP Start\n");
	for(i = 0; i < EDID_SIZE; i++){
		if((i % 16) == 0) printk("\n");
		printk("%02x ", AUO_EDID[i]);
		if(i == 127){
			checksum = 256 - checksum % 256;
			printk("\nEDID checksum=%x, result=%s\n", checksum, (checksum == AUO_EDID[EDID_SIZE - 1]) ? "pass" : "failed");
		}else{
			checksum += AUO_EDID[i];
		}
	}
	printk("EDID DUMP End\n");
#endif
	mutex_unlock(&dsi2edp->lock);
	memcpy(buf, AUO_EDID, EDID_SIZE);
}

static ssize_t show_edid(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *ndev = to_platform_device(dev);
	struct tegra_dc *dc = platform_get_drvdata(ndev);
	struct tegra_dc_dsi_data *dsi = tegra_dc_get_outdata(dc);

	tc358767_dsi2edp_get_edid(dsi, buf);

	return EDID_SIZE;
}
static DEVICE_ATTR(edid, 0644, show_edid, NULL);

#define DSI_PANEL_RST_GPIO		59	/* Pull high to power up dsi2edp bridge */
extern bool panel_initialized;

static int tc358767_dsi2edp_init(struct tegra_dc_dsi_data *dsi)
{
	int err = 0;

	if (tc358767_dsi2edp) {
		tegra_dsi_set_outdata(dsi, tc358767_dsi2edp);
		return err;
	}

	tc358767_dsi2edp = devm_kzalloc(&dsi->dc->ndev->dev,
					sizeof(*tc358767_dsi2edp),
					GFP_KERNEL);
	if (!tc358767_dsi2edp)
		return -ENOMEM;

	tc358767_dsi2edp->dsi = dsi;

	tc358767_dsi2edp->client_i2c = tc358767_i2c_client;

	tc358767_dsi2edp->regmap = devm_regmap_init_i2c(tc358767_i2c_client,
						&tc358767_regmap_config);
	if (IS_ERR(tc358767_dsi2edp->regmap)) {
		err = PTR_ERR(tc358767_dsi2edp->regmap);
		dev_err(&dsi->dc->ndev->dev,
				"tc358767_dsi2edp: regmap init failed\n");
		goto fail;
	}

	tc358767_dsi2edp->mode = &dsi->dc->mode;

	tegra_dsi_set_outdata(dsi, tc358767_dsi2edp);

	mutex_init(&tc358767_dsi2edp->lock);

	err = gpio_request(DSI_PANEL_RST_GPIO, "panel_rst");
	if (err < 0) {
		printk(KERN_ERR "%s(): Fail to request DSI_PANEL_RST_GPIO\n", __func__);
		goto fail;
	}

	///sys/class/graphics/fb0/device/edid
	err = device_create_file(&dsi->dc->ndev->dev, &dev_attr_edid);
	if(err < 0){
		dev_err(&dsi->dc->ndev->dev, "tc358767_dsi2edp: failed to create EDID sysfs attribute file\n");
	}

fail:
	return err;
}

static void tc358767_dsi2edp_destroy(struct tegra_dc_dsi_data *dsi)
{
	struct tegra_dc_dsi2edp_data *dsi2edp =
				tegra_dsi_get_outdata(dsi);

	if (!dsi2edp)
		return;

	tc358767_dsi2edp = NULL;
	mutex_destroy(&dsi2edp->lock);
}

static void tc358767_dsi2edp_enable(struct tegra_dc_dsi_data *dsi)
{
	struct tegra_dc_dsi2edp_data *dsi2edp = tegra_dsi_get_outdata(dsi);
	unsigned val;
	unsigned chip_id;
	int i = 0;

	if (dsi2edp && dsi2edp->dsi2edp_enabled)
		return;

	if (panel_initialized == 0) {
		gpio_direction_output(DSI_PANEL_RST_GPIO, 0);
		msleep(1);
	}

	msleep(50);
	gpio_set_value(DSI_PANEL_RST_GPIO, 1);
	msleep(2);

	mutex_lock(&dsi2edp->lock);

	/* Chip ID */
	tc358767_reg_read(dsi2edp, TC358767_CHIP_ID, &val);
	chip_id = val;
	pr_info("%s: TC358767_CHIP_ID 0x%08x\n", __func__, chip_id);

#if PRBS7_PATTERN_TEST
	pr_info("%s: TC358767 output PRBS7 pattern\n", __func__);

	/* Setup main link */
	tc358767_reg_write(dsi2edp, TC358767_DP0_LINK_TRAINING_CTRL, 0x00002087);
	tc358767_reg_write(dsi2edp, TC358767_DP1_LINK_TRAINING_CTRL, 0x00003083);
	tc358767_reg_write(dsi2edp, TC358767_SYSTEM_CLK_PARAM, 0x00000101);

	/* Setup DP-PHY/PLLs */
	tc358767_reg_write(dsi2edp, TC358767_PHY_CTRL, 0x03000007);
	tc358767_reg_write(dsi2edp, TC358767_DP0_LINK_CLK_PLL_CTRL, 0x00000005);
	tc358767_reg_write(dsi2edp, TC358767_DP0_LINK_CLK_PLL_CTRL, 0x00000005);
	tc358767_reg_write(dsi2edp, TC358767_DP1_LINK_CLK_PLL_CTRL, 0x00000005);
	msleep(70);

	/* Reset/Enable main link */
	tc358767_reg_write(dsi2edp, TC358767_PHY_CTRL, 0x03001107);
	tc358767_reg_write(dsi2edp, TC358767_PHY_CTRL, 0x03000007);
	msleep(70);
	tc358767_reg_write(dsi2edp, TC358767_PHY_CTRL, 0x03010007);
	msleep(70);

	/* Enable PRBS7 pattern */
	tc358767_reg_write(dsi2edp, TC358767_DP_PHY_CFG_WR, 0x00000101);
	tc358767_reg_write(dsi2edp, TC358767_DP_PHY_CFG_WR, 0x00010101);
	msleep(70);
	tc358767_reg_write(dsi2edp, TC358767_DP_PHY_CFG_WR, 0x00FF0101);
	msleep(70);

	goto finish;
#endif

#if D10_2_PATTERN_TEST
	pr_info("%s: TC358767 output D10.2 pattern\n", __func__);

	/* Setup main link */
	tc358767_reg_write(dsi2edp, TC358767_DP0_LINK_TRAINING_CTRL, 0x02022087);
	tc358767_reg_write(dsi2edp, TC358767_DP1_LINK_TRAINING_CTRL, 0x00003083);
	tc358767_reg_write(dsi2edp, TC358767_SYSTEM_CLK_PARAM, 0x00000101);

	/* Setup DP-PHY/PLLs */
	tc358767_reg_write(dsi2edp, TC358767_PHY_CTRL, 0x03000007);
	tc358767_reg_write(dsi2edp, TC358767_DP0_LINK_CLK_PLL_CTRL, 0x00000005);
	tc358767_reg_write(dsi2edp, TC358767_DP0_LINK_CLK_PLL_CTRL, 0x00000005);
	tc358767_reg_write(dsi2edp, TC358767_DP1_LINK_CLK_PLL_CTRL, 0x00000005);
	msleep(70);

	/* Reset/Enable main link */
	tc358767_reg_write(dsi2edp, TC358767_PHY_CTRL, 0x03001107);
	tc358767_reg_write(dsi2edp, TC358767_PHY_CTRL, 0x03000007);
	msleep(70);
	tc358767_reg_write(dsi2edp, TC358767_PHY_CTRL, 0x03010007);
	msleep(70);

	/* Enable D10.2 pattern */
	tc358767_reg_write(dsi2edp, TC358767_DP0_LINK_TRAINING_CTRL, 0x02023105);
	msleep(70);
	tc358767_reg_write(dsi2edp, TC358767_DP_PHY_CFG_WR, 0x00FF0101);
	msleep(70);

	goto finish;
#endif

#if !(COLOR_BAR_TEST)
	pr_info("%s: TC358767 normal output\n", __func__);
#else
	pr_info("%s: TC358767 output color bar pattern\n", __func__);
#endif

	if (bowser_panel_resolution == BOWSER_RESOLUTION_FHD) {
		pr_info("%s: DSI2EDP resolution set to FHD\n", __func__);
		/* Setup main link */
		tc358767_reg_write(dsi2edp, TC358767_DP0_LINK_TRAINING_CTRL, 0x3086);
		DUMP_REG(dsi2edp, TC358767_DP0_LINK_TRAINING_CTRL, val);
		tc358767_reg_write(dsi2edp, TC358767_DP1_LINK_TRAINING_CTRL, 0x0002);
		DUMP_REG(dsi2edp, TC358767_DP1_LINK_TRAINING_CTRL, val);
		tc358767_reg_write(dsi2edp, TC358767_SYSTEM_CLK_PARAM, 0x0101);
		DUMP_REG(dsi2edp, TC358767_SYSTEM_CLK_PARAM, val);

		/* Setup DP-PHY/PLLs */
		tc358767_reg_write(dsi2edp, TC358767_PHY_CTRL, 0x03000007);
		usleep_range(1000, 1200);
		DUMP_REG(dsi2edp, TC358767_PHY_CTRL, val);
		tc358767_reg_write(dsi2edp, TC358767_DP0_LINK_CLK_PLL_CTRL, 0x05);
		usleep_range(1000, 1200);
		tc358767_reg_write(dsi2edp, TC358767_DP0_LINK_CLK_PLL_CTRL, 0x05);
		usleep_range(1000, 1200);
		DUMP_REG(dsi2edp, TC358767_DP0_LINK_CLK_PLL_CTRL, val);
		tc358767_reg_write(dsi2edp, TC358767_DP1_LINK_CLK_PLL_CTRL, 0x05);
		DUMP_REG(dsi2edp, TC358767_DP1_LINK_CLK_PLL_CTRL, val);
		msleep(40);
		tc358767_reg_write(dsi2edp,
				TC358767_STREAM_CLK_PLL_PARAM, 0x01330142);
		DUMP_REG(dsi2edp, TC358767_STREAM_CLK_PLL_PARAM, val);
		tc358767_reg_write(dsi2edp,
				TC358767_STREAM_CLK_PLL_CTRL, 0x05);
		DUMP_REG(dsi2edp, TC358767_STREAM_CLK_PLL_CTRL, val);

		/* Reset/Enable main link */
		tc358767_reg_write(dsi2edp, TC358767_PHY_CTRL, 0x03001107);
		usleep_range(1000, 1200);
		DUMP_REG(dsi2edp, TC358767_PHY_CTRL, val);
		tc358767_reg_write(dsi2edp, TC358767_PHY_CTRL, 0x03000007);
		DUMP_REG(dsi2edp, TC358767_PHY_CTRL, val);
		msleep(40);

		/* Check main channel ready */
		DUMP_REG(dsi2edp, TC358767_PHY_CTRL, val);
		tc358767_reg_read(dsi2edp, TC358767_PHY_CTRL, &val);
		while (!(val & 0x10000) && (i < 10)) {
			printk("%s: main channel isn't ready (DP_PHY_Ctrl=%x), waiting.\n", __func__, val);
			msleep(5);
			i++;
		}
		if (val & 0x10000)
			printk("%s: main channel is ready (DP_PHY_Ctrl=%x)\n", __func__, val);
		else
			printk("%s: main channel isn't ready (DP_PHY_Ctrl=%x)\n", __func__, val);

		/* Read DP Rx link capability */
		tc358767_reg_write(dsi2edp,
				TC358767_AUX_CHANNEL_CONFIG1, 0x01063F);
		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_CONFIG1, val);
		tc358767_reg_write(dsi2edp,
				TC358767_AUX_CHANNEL_DPCD_ADDR, 0x01);
		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_DPCD_ADDR, val);
		tc358767_reg_write(dsi2edp,
				TC358767_AUX_CHANNEL_CONFIG0, 0x09);
		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_CONFIG0, val);

		/* Check aux channel status */
		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_STATUS, val);
		tc358767_reg_read(dsi2edp, TC358767_AUX_CHANNEL_STATUS, &val);
		if (val & (0x01 << 1)) {
			pr_err("%s: timeout, TC358767_AUX_CHANNEL_STATUS 0x%x\n", __func__, val);
			goto finish;
		} else if (val & 0x00000001)
			printk(KERN_ERR "%s(): Black-out detected\n", __func__);

		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_DPCD_RD_DATA0, val);
		tc358767_reg_write(dsi2edp,
				TC358767_AUX_CHANNEL_DPCD_ADDR, 0x02);
		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_DPCD_ADDR, val);
		tc358767_reg_write(dsi2edp,
				TC358767_AUX_CHANNEL_CONFIG0, 0x09);
		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_CONFIG0, val);
		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_STATUS, val);
		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_DPCD_RD_DATA0, val);

		/* Setup link & DPRx config for training */
		tc358767_reg_write(dsi2edp,
				TC358767_AUX_CHANNEL_DPCD_ADDR, 0x0100);
		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_DPCD_ADDR, val);
		tc358767_reg_write(dsi2edp,
				TC358767_AUX_CHANNEL_DPCD_WR_DATA0, 0x020A);
		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_DPCD_WR_DATA0, val);
		tc358767_reg_write(dsi2edp,
				TC358767_AUX_CHANNEL_CONFIG0, 0x0108);
		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_CONFIG0, val);
		tc358767_reg_write(dsi2edp,
				TC358767_AUX_CHANNEL_DPCD_ADDR, 0x0108);
		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_DPCD_ADDR, val);
		tc358767_reg_write(dsi2edp,
				TC358767_AUX_CHANNEL_DPCD_WR_DATA0, 0x01);
		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_DPCD_WR_DATA0, val);
		tc358767_reg_write(dsi2edp,
				TC358767_AUX_CHANNEL_CONFIG0, 0x08);
		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_CONFIG0, val);

		/* Set DPCD 00102h for training pattern 1 */
		tc358767_reg_write(dsi2edp,
				TC358767_LINK_TRAINING_SINK_CONFIG, 0x21);
		DUMP_REG(dsi2edp, TC358767_LINK_TRAINING_SINK_CONFIG, val);
		tc358767_reg_write(dsi2edp,
				TC358767_LINK_TRAINING_LOOP_CTRL, 0xF600000D);
		DUMP_REG(dsi2edp, TC358767_LINK_TRAINING_LOOP_CTRL, val);

		/* Set DP0 training pattern 1 */
		tc358767_reg_write(dsi2edp, TC358767_DP0_LINK_TRAINING_CTRL, 0x3187);
		DUMP_REG(dsi2edp, TC358767_DP0_LINK_TRAINING_CTRL, val);

		/* Enable DP0 to start link training */
		tc358767_reg_write(dsi2edp, TC358767_DP_CTRL, 0x01);
		DUMP_REG(dsi2edp, TC358767_DP_CTRL, val);
		DUMP_REG(dsi2edp, TC358767_LINK_TRAINING_STATUS, val);

		/* Set DPCD 00102h for training pattern 2 */
		tc358767_reg_write(dsi2edp,
				TC358767_LINK_TRAINING_SINK_CONFIG, 0x22);
		DUMP_REG(dsi2edp, TC358767_LINK_TRAINING_SINK_CONFIG, val);

		/* Set DP0 training pattern 2 */
		tc358767_reg_write(dsi2edp, TC358767_DP0_LINK_TRAINING_CTRL, 0x3287);
		DUMP_REG(dsi2edp, TC358767_DP0_LINK_TRAINING_CTRL, val);
		DUMP_REG(dsi2edp, TC358767_LINK_TRAINING_STATUS, val);

		/* Clear DPCD 00102h */
		tc358767_reg_write(dsi2edp,
				TC358767_AUX_CHANNEL_DPCD_ADDR, 0x0102);
		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_DPCD_ADDR, val);
		tc358767_reg_write(dsi2edp,
				TC358767_AUX_CHANNEL_DPCD_WR_DATA0, 0x00);
		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_DPCD_WR_DATA0, val);
		tc358767_reg_write(dsi2edp,
				TC358767_AUX_CHANNEL_CONFIG0, 0x08);
		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_CONFIG0, val);

		/* Clear DP0 training pattern */
		tc358767_reg_write(dsi2edp, TC358767_DP0_LINK_TRAINING_CTRL, 0x1087);
		DUMP_REG(dsi2edp, TC358767_DP0_LINK_TRAINING_CTRL, val);

		/* Read DPCD 0x00200-0x00204 */
		tc358767_reg_write(dsi2edp,
				TC358767_AUX_CHANNEL_DPCD_ADDR, 0x0200);
		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_DPCD_ADDR, val);
		tc358767_reg_write(dsi2edp,
				TC358767_AUX_CHANNEL_CONFIG0, 0x0409);
		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_CONFIG0, val);
		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_STATUS, val);
		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_DPCD_RD_DATA0, val);
		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_DPCD_RD_DATA1, val);

		/* ASSR configuration, 770A(reg 0x0500[0] = 1) supports ASSR,
		 * need to check the ASSR capability for eDP panel(0x0500[1] = 0).
		 */
		if (chip_id & 0x01) {
			pr_err("%s: ASSR configuration\n", __func__);
			tc358767_reg_write(dsi2edp,
					TC358767_AUX_CHANNEL_DPCD_ADDR, 0x000D);
			tc358767_reg_write(dsi2edp,
					TC358767_AUX_CHANNEL_CONFIG0, 0x0009);
			tc358767_reg_read(dsi2edp, TC358767_AUX_CHANNEL_STATUS, &val);
			tc358767_reg_read(dsi2edp,
					TC358767_AUX_CHANNEL_DPCD_RD_DATA0, &val);

			if (val & 0x01) {
				/* Enable ASSR*/
				tc358767_reg_write(dsi2edp,
						TC358767_AUX_CHANNEL_DPCD_ADDR, 0x010A);
				tc358767_reg_write(dsi2edp,
						TC358767_AUX_CHANNEL_DPCD_WR_DATA0, 0x01);
				tc358767_reg_write(dsi2edp,
						TC358767_AUX_CHANNEL_CONFIG0, 0x08);
			}
		}

#if !(COLOR_BAR_TEST)
		/* DSI0 setting */
		tc358767_reg_write(dsi2edp, TC358767_DSI0_PPI_TX_RX_TA, 0x000A000C);
		DUMP_REG(dsi2edp, TC358767_DSI0_PPI_TX_RX_TA, val);
		tc358767_reg_write(dsi2edp,
				TC358767_DSI0_PPI_LPTXTIMECNT, 0x8);
		DUMP_REG(dsi2edp, TC358767_DSI0_PPI_LPTXTIMECNT, val);
		tc358767_reg_write(dsi2edp,
				TC358767_DSI0_PPI_D0S_CLRSIPOCOUNT, 0x0D);
		DUMP_REG(dsi2edp, TC358767_DSI0_PPI_D0S_CLRSIPOCOUNT, val);
		tc358767_reg_write(dsi2edp,
				TC358767_DSI0_PPI_D1S_CLRSIPOCOUNT, 0x0D);
		DUMP_REG(dsi2edp, TC358767_DSI0_PPI_D1S_CLRSIPOCOUNT, val);
		tc358767_reg_write(dsi2edp,
				TC358767_DSI0_PPI_D2S_CLRSIPOCOUNT, 0x0D);
		DUMP_REG(dsi2edp, TC358767_DSI0_PPI_D2S_CLRSIPOCOUNT, val);
		tc358767_reg_write(dsi2edp,
				TC358767_DSI0_PPI_D3S_CLRSIPOCOUNT, 0x0D);
		DUMP_REG(dsi2edp, TC358767_DSI0_PPI_D3S_CLRSIPOCOUNT, val);
		tc358767_reg_write(dsi2edp,
				TC358767_DSI0_PPI_LANEENABLE, 0x1F);
		DUMP_REG(dsi2edp, TC358767_DSI0_PPI_LANEENABLE, val);
		tc358767_reg_write(dsi2edp,
				TC358767_DSI0_DSI_LANEENABLE, 0x1F);
		DUMP_REG(dsi2edp, TC358767_DSI0_DSI_LANEENABLE, val);
		tc358767_reg_write(dsi2edp, TC358767_DSI0_PPI_START, 0x01);
		DUMP_REG(dsi2edp, TC358767_DSI0_PPI_START, val);
		tc358767_reg_write(dsi2edp, TC358767_DSI0_DSI_START, 0x01);
		DUMP_REG(dsi2edp, TC358767_DSI0_DSI_START, val);
#endif

		/* lcd ctrl frame size */
		tc358767_reg_write(dsi2edp, TC358767_VIDEO_FRAME_CTRL, 0x064003E0);
		DUMP_REG(dsi2edp, TC358767_VIDEO_FRAME_CTRL, val);
		val = dsi2edp->mode->h_back_porch << 16 | dsi2edp->mode->h_sync_width;
		tc358767_reg_write(dsi2edp, TC358767_HORIZONTAL_TIME0, val);
		DUMP_REG(dsi2edp, TC358767_HORIZONTAL_TIME0, val);
		val = dsi2edp->mode->h_front_porch << 16 | dsi2edp->mode->h_active;
		tc358767_reg_write(dsi2edp, TC358767_HORIZONTAL_TIME1, val);
		DUMP_REG(dsi2edp, TC358767_HORIZONTAL_TIME1, val);
		val = dsi2edp->mode->v_back_porch << 16 | dsi2edp->mode->v_sync_width;
		tc358767_reg_write(dsi2edp, TC358767_VERTICAL_TIME0, val);
		DUMP_REG(dsi2edp, TC358767_VERTICAL_TIME0, val);
		val = dsi2edp->mode->v_front_porch << 16 | dsi2edp->mode->v_active;
		tc358767_reg_write(dsi2edp, TC358767_VERTICAL_TIME1, val);
		DUMP_REG(dsi2edp, TC358767_VERTICAL_TIME1, val);
		tc358767_reg_write(dsi2edp,
				TC358767_VIDEO_FRAME_UPDATE_ENABLE, 0x01);
		DUMP_REG(dsi2edp, TC358767_VIDEO_FRAME_UPDATE_ENABLE, val);

		/* DP main stream attributes */
		tc358767_reg_write(dsi2edp,
				TC358767_VIDEO_FRAME_OUTPUT_DELAY, 0x002907D0);
		DUMP_REG(dsi2edp, TC358767_VIDEO_FRAME_OUTPUT_DELAY, val);
		tc358767_reg_write(dsi2edp, TC358767_VIDEO_FRAME_SIZE, 0x045E0834);
		DUMP_REG(dsi2edp, TC358767_VIDEO_FRAME_SIZE, val);
		tc358767_reg_write(dsi2edp, TC358767_VIDEO_FRAME_START, 0x00120050);
		DUMP_REG(dsi2edp, TC358767_VIDEO_FRAME_START, val);
		tc358767_reg_write(dsi2edp,
				TC358767_VIDEO_FRAME_ACTIVE_REGION_SIZE, 0x04380780);
		DUMP_REG(dsi2edp, TC358767_VIDEO_FRAME_ACTIVE_REGION_SIZE, val);
		tc358767_reg_write(dsi2edp,
				TC358767_VIDEO_FRAME_SYNC_WIDTH, 0x80028002);
		DUMP_REG(dsi2edp, TC358767_VIDEO_FRAME_SYNC_WIDTH, val);

		/* DP flow shape & timestamp */
		tc358767_reg_write(dsi2edp, TC358767_DP_CONFIG, 0x14BF0000);
		DUMP_REG(dsi2edp, TC358767_DP_CONFIG, val);

#if COLOR_BAR_TEST
		tc358767_reg_write(dsi2edp, TC358767_D2DP_TEST_CTRL, 0x78006312);
		DUMP_REG(dsi2edp, TC358767_D2DP_TEST_CTRL, val);
#endif

		tc358767_reg_write(dsi2edp,
				TC358767_FMALUE_VIDEO_CLK_REGEN, 0x00004360);
		DUMP_REG(dsi2edp, TC358767_FMALUE_VIDEO_CLK_REGEN, val);
		tc358767_reg_write(dsi2edp,
				TC358767_NVALUE_VIDEO_CLK_REGEN, 0x00008133);
		DUMP_REG(dsi2edp, TC358767_NVALUE_VIDEO_CLK_REGEN, val);
		tc358767_reg_write(dsi2edp, TC358767_DP_CTRL, 0x41);
		DUMP_REG(dsi2edp, TC358767_DP_CTRL, val);
		tc358767_reg_write(dsi2edp, TC358767_DP_CTRL, 0x43);
		DUMP_REG(dsi2edp, TC358767_DP_CTRL, val);

#if COLOR_BAR_TEST
		tc358767_reg_write(dsi2edp, TC358767_SYSTEM_CTRL, 0x03);
		DUMP_REG(dsi2edp, TC358767_SYSTEM_CTRL, val);
#else
		tc358767_reg_write(dsi2edp, TC358767_SYSTEM_CTRL, 0x01);
		DUMP_REG(dsi2edp, TC358767_SYSTEM_CTRL, val);
#endif

		DUMP_REG(dsi2edp, TC358767_FMALUE_VIDEO_CLK_REGEN, val);
		DUMP_REG(dsi2edp, TC358767_NVALUE_VIDEO_CLK_REGEN, val);
		DUMP_REG(dsi2edp, TC358767_MVALUE_VIDEO_CLK_REGEN, val);
		DUMP_REG(dsi2edp, TC358767_SYSTEM_STAT, val);
		DUMP_REG(dsi2edp, TC358767_DSI_INTERRUPT_STATUS, val);
	} else {
		printk(KERN_INFO "%s(): DSI2EDP resolution set to HD\n", __func__);

		/* Setup main link */
		tc358767_reg_write(dsi2edp, TC358767_DP0_LINK_TRAINING_CTRL, 0x3082);
		DUMP_REG(dsi2edp, TC358767_DP0_LINK_TRAINING_CTRL, val);
		tc358767_reg_write(dsi2edp, TC358767_DP1_LINK_TRAINING_CTRL, 0x0002);
		DUMP_REG(dsi2edp, TC358767_DP1_LINK_TRAINING_CTRL, val);
		tc358767_reg_write(dsi2edp, TC358767_SYSTEM_CLK_PARAM, 0x0101);
		DUMP_REG(dsi2edp, TC358767_SYSTEM_CLK_PARAM, val);

		/* Setup DP-PHY/PLLs */
		tc358767_reg_write(dsi2edp, TC358767_PHY_CTRL, 0x03000003);
		usleep_range(1000, 1200);
		DUMP_REG(dsi2edp, TC358767_PHY_CTRL, val);
		tc358767_reg_write(dsi2edp, TC358767_DP0_LINK_CLK_PLL_CTRL, 0x05);
		usleep_range(1000, 1200);
		tc358767_reg_write(dsi2edp, TC358767_DP0_LINK_CLK_PLL_CTRL, 0x05);
		usleep_range(1000, 1200);
		DUMP_REG(dsi2edp, TC358767_DP0_LINK_CLK_PLL_CTRL, val);
		tc358767_reg_write(dsi2edp, TC358767_DP1_LINK_CLK_PLL_CTRL, 0x05);
		DUMP_REG(dsi2edp, TC358767_DP1_LINK_CLK_PLL_CTRL, val);
		msleep(40);
		tc358767_reg_write(dsi2edp,TC358767_STREAM_CLK_PLL_PARAM, 0x00330122);
		DUMP_REG(dsi2edp, TC358767_STREAM_CLK_PLL_PARAM, val);
		tc358767_reg_write(dsi2edp,
				TC358767_STREAM_CLK_PLL_CTRL, 0x05);
		DUMP_REG(dsi2edp, TC358767_STREAM_CLK_PLL_CTRL, val);

		/* Reset/Enable main link */
		tc358767_reg_write(dsi2edp, TC358767_PHY_CTRL, 0x13001103);
		usleep_range(1000, 1200);
		DUMP_REG(dsi2edp, TC358767_PHY_CTRL, val);
		tc358767_reg_write(dsi2edp, TC358767_PHY_CTRL, 0x03000003);
		DUMP_REG(dsi2edp, TC358767_PHY_CTRL, val);
		msleep(40);

		/* Check main channel ready */
		DUMP_REG(dsi2edp, TC358767_PHY_CTRL, val);
		tc358767_reg_read(dsi2edp, TC358767_PHY_CTRL, &val);
		while (!(val & 0x10000) && (i < 10)) {
			printk("%s: main channel isn't ready (DP_PHY_Ctrl=%x), waiting.\n", __func__, val);
			msleep(5);
			i++;
		}
		if (val & 0x10000)
			printk("%s: main channel is ready (DP_PHY_Ctrl=%x)\n", __func__, val);
		else
			printk("%s: main channel isn't ready (DP_PHY_Ctrl=%x)\n", __func__, val);

		/* Read DP Rx link capability */
		tc358767_reg_write(dsi2edp,
				TC358767_AUX_CHANNEL_CONFIG1, 0x01063F);
		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_CONFIG1, val);
		tc358767_reg_write(dsi2edp,
				TC358767_AUX_CHANNEL_DPCD_ADDR, 0x01);
		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_DPCD_ADDR, val);
		tc358767_reg_write(dsi2edp,
				TC358767_AUX_CHANNEL_CONFIG0, 0x09);
		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_CONFIG0, val);

		/* Check aux channel status */
		tc358767_reg_read(dsi2edp, TC358767_AUX_CHANNEL_STATUS, &val);
		if (val & (0x01 << 1)) {
			pr_err("%s: timeout, TC358767_AUX_CHANNEL_STATUS 0x%x\n", __func__, val);
			goto finish;
		} else
			DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_STATUS, val);

		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_DPCD_RD_DATA0, val);
		tc358767_reg_write(dsi2edp,
				TC358767_AUX_CHANNEL_DPCD_ADDR, 0x02);
		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_DPCD_ADDR, val);
		tc358767_reg_write(dsi2edp,
				TC358767_AUX_CHANNEL_CONFIG0, 0x09);
		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_CONFIG0, val);
		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_STATUS, val);
		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_DPCD_RD_DATA0, val);

		/* Setup link & DPRx config for training */
		tc358767_reg_write(dsi2edp,
				TC358767_AUX_CHANNEL_DPCD_ADDR, 0x0100);
		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_DPCD_ADDR, val);
		tc358767_reg_write(dsi2edp,
				TC358767_AUX_CHANNEL_DPCD_WR_DATA0, 0x010A);
		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_DPCD_WR_DATA0, val);
		tc358767_reg_write(dsi2edp,
				TC358767_AUX_CHANNEL_CONFIG0, 0x0108);
		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_CONFIG0, val);
		tc358767_reg_write(dsi2edp,
				TC358767_AUX_CHANNEL_DPCD_ADDR, 0x0108);
		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_DPCD_ADDR, val);
		tc358767_reg_write(dsi2edp,
				TC358767_AUX_CHANNEL_DPCD_WR_DATA0, 0x01);
		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_DPCD_WR_DATA0, val);
		tc358767_reg_write(dsi2edp,
				TC358767_AUX_CHANNEL_CONFIG0, 0x08);
		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_CONFIG0, val);

		/* Set DPCD 00102h for training pattern 1 */
		tc358767_reg_write(dsi2edp,
				TC358767_LINK_TRAINING_SINK_CONFIG, 0x21);
		DUMP_REG(dsi2edp, TC358767_LINK_TRAINING_SINK_CONFIG, val);
		tc358767_reg_write(dsi2edp,
				TC358767_LINK_TRAINING_LOOP_CTRL, 0xF600000D);
		DUMP_REG(dsi2edp, TC358767_LINK_TRAINING_LOOP_CTRL, val);

		/* Set DP0 training pattern 1 */
		tc358767_reg_write(dsi2edp, TC358767_DP0_LINK_TRAINING_CTRL, 0x3183);
		DUMP_REG(dsi2edp, TC358767_DP0_LINK_TRAINING_CTRL, val);

		/* Enable DP0 to start link training */
		tc358767_reg_write(dsi2edp, TC358767_DP_CTRL, 0x01);
		DUMP_REG(dsi2edp, TC358767_DP_CTRL, val);
		DUMP_REG(dsi2edp, TC358767_LINK_TRAINING_STATUS, val);

		/* Set DPCD 00102h for training pattern 2 */
		tc358767_reg_write(dsi2edp,
				TC358767_LINK_TRAINING_SINK_CONFIG, 0x22);
		DUMP_REG(dsi2edp, TC358767_LINK_TRAINING_SINK_CONFIG, val);

		/* Set DP0 training pattern 2 */
		tc358767_reg_write(dsi2edp, TC358767_DP0_LINK_TRAINING_CTRL, 0x3283);
		DUMP_REG(dsi2edp, TC358767_DP0_LINK_TRAINING_CTRL, val);
		DUMP_REG(dsi2edp, TC358767_LINK_TRAINING_STATUS, val);

		/* Clear DPCD 00102h */
		tc358767_reg_write(dsi2edp,
				TC358767_AUX_CHANNEL_DPCD_ADDR, 0x0102);
		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_DPCD_ADDR, val);
		tc358767_reg_write(dsi2edp,
				TC358767_AUX_CHANNEL_DPCD_WR_DATA0, 0x00);
		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_DPCD_WR_DATA0, val);
		tc358767_reg_write(dsi2edp,
				TC358767_AUX_CHANNEL_CONFIG0, 0x08);
		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_CONFIG0, val);

		/* Clear DP0 training pattern */
		tc358767_reg_write(dsi2edp, TC358767_DP0_LINK_TRAINING_CTRL, 0x1083);
		DUMP_REG(dsi2edp, TC358767_DP0_LINK_TRAINING_CTRL, val);

		/* Read DPCD 0x00200-0x00204 */
		tc358767_reg_write(dsi2edp,
				TC358767_AUX_CHANNEL_DPCD_ADDR, 0x0200);
		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_DPCD_ADDR, val);
		tc358767_reg_write(dsi2edp,
				TC358767_AUX_CHANNEL_CONFIG0, 0x0409);
		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_CONFIG0, val);
		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_STATUS, val);
		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_DPCD_RD_DATA0, val);
		DUMP_REG(dsi2edp, TC358767_AUX_CHANNEL_DPCD_RD_DATA1, val);

		/* ASSR configuration, 770A(reg 0x0500[0] = 1) supports ASSR,
		 * need to check the ASSR capability for eDP panel(0x0500[1] = 0).
		 */
		if (chip_id & 0x01) {
			pr_err("%s: ASSR configuration\n", __func__);
			tc358767_reg_write(dsi2edp,
					TC358767_AUX_CHANNEL_DPCD_ADDR, 0x000D);
			tc358767_reg_write(dsi2edp,
					TC358767_AUX_CHANNEL_CONFIG0, 0x0009);
			tc358767_reg_read(dsi2edp, TC358767_AUX_CHANNEL_STATUS, &val);
			tc358767_reg_read(dsi2edp,
					TC358767_AUX_CHANNEL_DPCD_RD_DATA0, &val);

			if (val & 0x01) {
				/* Enable ASSR*/
				tc358767_reg_write(dsi2edp,
						TC358767_AUX_CHANNEL_DPCD_ADDR, 0x010A);
				tc358767_reg_write(dsi2edp,
						TC358767_AUX_CHANNEL_DPCD_WR_DATA0, 0x01);
				tc358767_reg_write(dsi2edp,
						TC358767_AUX_CHANNEL_CONFIG0, 0x08);
			}
		}

#if !(COLOR_BAR_TEST)
		/* DSI0 setting */
		tc358767_reg_write(dsi2edp, TC358767_DSI0_PPI_TX_RX_TA, 0x00050006);
		DUMP_REG(dsi2edp, TC358767_DSI0_PPI_TX_RX_TA, val);
		tc358767_reg_write(dsi2edp,
				TC358767_DSI0_PPI_LPTXTIMECNT, 0x4);
		DUMP_REG(dsi2edp, TC358767_DSI0_PPI_LPTXTIMECNT, val);
		tc358767_reg_write(dsi2edp,
				TC358767_DSI0_PPI_D0S_CLRSIPOCOUNT, 0x5);
		DUMP_REG(dsi2edp, TC358767_DSI0_PPI_D0S_CLRSIPOCOUNT, val);
		tc358767_reg_write(dsi2edp,
				TC358767_DSI0_PPI_D1S_CLRSIPOCOUNT, 0x5);
		DUMP_REG(dsi2edp, TC358767_DSI0_PPI_D1S_CLRSIPOCOUNT, val);
		tc358767_reg_write(dsi2edp,
				TC358767_DSI0_PPI_D2S_CLRSIPOCOUNT, 0x5);
		DUMP_REG(dsi2edp, TC358767_DSI0_PPI_D2S_CLRSIPOCOUNT, val);
		tc358767_reg_write(dsi2edp,
				TC358767_DSI0_PPI_D3S_CLRSIPOCOUNT, 0x5);
		DUMP_REG(dsi2edp, TC358767_DSI0_PPI_D3S_CLRSIPOCOUNT, val);
		tc358767_reg_write(dsi2edp,
				TC358767_DSI0_PPI_LANEENABLE, 0x1F);
		DUMP_REG(dsi2edp, TC358767_DSI0_PPI_LANEENABLE, val);
		tc358767_reg_write(dsi2edp,
				TC358767_DSI0_DSI_LANEENABLE, 0x1F);
		DUMP_REG(dsi2edp, TC358767_DSI0_DSI_LANEENABLE, val);
		tc358767_reg_write(dsi2edp, TC358767_DSI0_PPI_START, 0x01);
		DUMP_REG(dsi2edp, TC358767_DSI0_PPI_START, val);
		tc358767_reg_write(dsi2edp, TC358767_DSI0_DSI_START, 0x01);
		DUMP_REG(dsi2edp, TC358767_DSI0_DSI_START, val);
#endif

		/* lcd ctrl frame size */
		tc358767_reg_write(dsi2edp, TC358767_VIDEO_FRAME_CTRL, 0x03E00000);
		DUMP_REG(dsi2edp, TC358767_VIDEO_FRAME_CTRL, val);
		val = dsi2edp->mode->h_back_porch << 16 | dsi2edp->mode->h_sync_width;
		tc358767_reg_write(dsi2edp, TC358767_HORIZONTAL_TIME0, val);
		DUMP_REG(dsi2edp, TC358767_HORIZONTAL_TIME0, val);
		val = dsi2edp->mode->h_front_porch << 16 | dsi2edp->mode->h_active;
		tc358767_reg_write(dsi2edp, TC358767_HORIZONTAL_TIME1, val);
		DUMP_REG(dsi2edp, TC358767_HORIZONTAL_TIME1, val);
		val = dsi2edp->mode->v_back_porch << 16 | dsi2edp->mode->v_sync_width;
		tc358767_reg_write(dsi2edp, TC358767_VERTICAL_TIME0, val);
		DUMP_REG(dsi2edp, TC358767_VERTICAL_TIME0, val);
		val = dsi2edp->mode->v_front_porch << 16 | dsi2edp->mode->v_active;
		tc358767_reg_write(dsi2edp, TC358767_VERTICAL_TIME1, val);
		DUMP_REG(dsi2edp, TC358767_VERTICAL_TIME1, val);
		tc358767_reg_write(dsi2edp,
				TC358767_VIDEO_FRAME_UPDATE_ENABLE, 0x01);
		DUMP_REG(dsi2edp, TC358767_VIDEO_FRAME_UPDATE_ENABLE, val);

		/* DP main stream attributes */
		tc358767_reg_write(dsi2edp,
				TC358767_VIDEO_FRAME_OUTPUT_DELAY, 0x00290574);
		DUMP_REG(dsi2edp, TC358767_VIDEO_FRAME_OUTPUT_DELAY, val);
		tc358767_reg_write(dsi2edp, TC358767_VIDEO_FRAME_SIZE, 0x033805B2);
		DUMP_REG(dsi2edp, TC358767_VIDEO_FRAME_SIZE, val);
		tc358767_reg_write(dsi2edp, TC358767_VIDEO_FRAME_START, 0x0026001C);
		DUMP_REG(dsi2edp, TC358767_VIDEO_FRAME_START, val);
		tc358767_reg_write(dsi2edp,
				TC358767_VIDEO_FRAME_ACTIVE_REGION_SIZE, 0x03000558);
		DUMP_REG(dsi2edp, TC358767_VIDEO_FRAME_ACTIVE_REGION_SIZE, val);
		tc358767_reg_write(dsi2edp,
				TC358767_VIDEO_FRAME_SYNC_WIDTH, 0x80028002);
		DUMP_REG(dsi2edp, TC358767_VIDEO_FRAME_SYNC_WIDTH, val);

		/* DP flow shape & timestamp */
		tc358767_reg_write(dsi2edp, TC358767_DP_CONFIG, 0x14BF0000);
		DUMP_REG(dsi2edp, TC358767_DP_CONFIG, val);

#if COLOR_BAR_TEST
		tc358767_reg_write(dsi2edp, TC358767_D2DP_TEST_CTRL, 0x78006312);
		DUMP_REG(dsi2edp, TC358767_D2DP_TEST_CTRL, val);
#endif

		tc358767_reg_write(dsi2edp,
				TC358767_FMALUE_VIDEO_CLK_REGEN, 0x0220);
		DUMP_REG(dsi2edp, TC358767_FMALUE_VIDEO_CLK_REGEN, val);
		tc358767_reg_write(dsi2edp,
				TC358767_NVALUE_VIDEO_CLK_REGEN, 0x07E9);
		DUMP_REG(dsi2edp, TC358767_NVALUE_VIDEO_CLK_REGEN, val);
		tc358767_reg_write(dsi2edp, TC358767_DP_CTRL, 0x41);
		DUMP_REG(dsi2edp, TC358767_DP_CTRL, val);
		tc358767_reg_write(dsi2edp, TC358767_DP_CTRL, 0x43);
		DUMP_REG(dsi2edp, TC358767_DP_CTRL, val);

#if COLOR_BAR_TEST
		tc358767_reg_write(dsi2edp, TC358767_SYSTEM_CTRL, 0x03);
		DUMP_REG(dsi2edp, TC358767_SYSTEM_CTRL, val);
#else
		tc358767_reg_write(dsi2edp, TC358767_SYSTEM_CTRL, 0x01);
		DUMP_REG(dsi2edp, TC358767_SYSTEM_CTRL, val);
#endif

		DUMP_REG(dsi2edp, TC358767_FMALUE_VIDEO_CLK_REGEN, val);
		DUMP_REG(dsi2edp, TC358767_NVALUE_VIDEO_CLK_REGEN, val);
		DUMP_REG(dsi2edp, TC358767_MVALUE_VIDEO_CLK_REGEN, val);
		DUMP_REG(dsi2edp, TC358767_SYSTEM_STAT, val);
		DUMP_REG(dsi2edp, TC358767_DSI_INTERRUPT_STATUS, val);
	}

	dsi2edp->dsi2edp_enabled = true;
	panel_initialized = 1;

finish:
	mutex_unlock(&dsi2edp->lock);
}

static void tc358767_dsi2edp_disable(struct tegra_dc_dsi_data *dsi)
{
	struct tegra_dc_dsi2edp_data *dsi2edp = tegra_dsi_get_outdata(dsi);

	gpio_set_value(DSI_PANEL_RST_GPIO, 0);

	dsi2edp->dsi2edp_enabled = false;
	panel_initialized = 0;
}

#ifdef CONFIG_PM
static void tc358767_dsi2edp_suspend(struct tegra_dc_dsi_data *dsi)
{
	/* To be done */
}

static void tc358767_dsi2edp_resume(struct tegra_dc_dsi_data *dsi)
{
	/* To be done */
}
#endif

struct tegra_dsi_out_ops tegra_dsi2edp_ops = {
	.init = tc358767_dsi2edp_init,
	.destroy = tc358767_dsi2edp_destroy,
	.enable = tc358767_dsi2edp_enable,
	.disable = tc358767_dsi2edp_disable,
#ifdef CONFIG_PM
	.suspend = tc358767_dsi2edp_suspend,
	.resume = tc358767_dsi2edp_resume,
#endif
};

static int tc358767_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	tc358767_i2c_client = client;

	return 0;
}

static int tc358767_i2c_remove(struct i2c_client *client)
{
	tc358767_i2c_client = NULL;

	return 0;
}

static const struct i2c_device_id tc358767_id_table[] = {
	{"tc358767_dsi2edp", 0},
	{},
};

static struct i2c_driver tc358767_i2c_drv = {
	.driver = {
		.name = "tc358767_dsi2edp",
		.owner = THIS_MODULE,
	},
	.probe = tc358767_i2c_probe,
	.remove = tc358767_i2c_remove,
	.id_table = tc358767_id_table,
};

static int __init tc358767_i2c_client_init(void)
{
	int err = 0;

	err = i2c_add_driver(&tc358767_i2c_drv);
	if (err)
		pr_err("tc358767_dsi2edp: Failed to add i2c client driver\n");

	return err;
}

static void __exit tc358767_i2c_client_exit(void)
{
	i2c_del_driver(&tc358767_i2c_drv);
}

subsys_initcall(tc358767_i2c_client_init);
module_exit(tc358767_i2c_client_exit);

MODULE_LICENSE("GPL");
