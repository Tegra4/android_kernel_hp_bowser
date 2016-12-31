/*
 * arch/arm/mach-tegra/board-bowser-sensors.c
 *
 * Copyright (c) 2012-2013 NVIDIA CORPORATION, All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of NVIDIA CORPORATION nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/mpu.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/therm_est.h>
#include <linux/nct1008.h>
#include <mach/edp.h>
#include <linux/edp.h>
#include <mach/gpio-tegra.h>
#include <mach/pinmux-t11.h>
#include <mach/pinmux.h>
#include <media/ov2722.h>
#include <generated/mach-types.h>
#include <linux/power/sbs-battery.h>

#include "gpio-names.h"
#include "board.h"
#include "board-common.h"
#include "board-bowser.h"
#include "cpu-tegra.h"
#include "devices.h"
#include "tegra-board-id.h"
#include "dvfs.h"

static struct regulator *bowser_vdd_cam_1v8 = NULL;
static struct regulator *bowser_avdd_cam1 = NULL;

static struct board_info board_info;

static struct throttle_table tj_throttle_table[] = {
	/* CPU_THROT_LOW cannot be used by other than CPU */
	/*      CPU,  C2BUS,  C3BUS,   SCLK,    EMC   */
	{ { 1810500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1785000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1759500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1734000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1708500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1683000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1657500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1632000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1606500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1581000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1555500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1530000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1504500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1479000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1453500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1428000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1402500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1377000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1351500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1326000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1300500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1275000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1249500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1224000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1198500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1173000, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1147500, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1122000, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1096500, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1071000, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1045500, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1020000, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  994500, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  969000, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  943500, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  918000, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  892500, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  867000, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  841500, 564000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  816000, 564000, NO_CAP, NO_CAP, 792000 } },
	{ {  790500, 564000, NO_CAP, 372000, 792000 } },
	{ {  765000, 564000, 468000, 372000, 792000 } },
	{ {  739500, 528000, 468000, 372000, 792000 } },
	{ {  714000, 528000, 468000, 336000, 792000 } },
	{ {  688500, 528000, 420000, 336000, 792000 } },
	{ {  663000, 492000, 420000, 336000, 792000 } },
	{ {  637500, 492000, 420000, 336000, 408000 } },
	{ {  612000, 492000, 420000, 300000, 408000 } },
	{ {  586500, 492000, 360000, 336000, 408000 } },
	{ {  561000, 420000, 420000, 300000, 408000 } },
	{ {  535500, 420000, 360000, 228000, 408000 } },
	{ {  510000, 420000, 288000, 228000, 408000 } },
	{ {  484500, 324000, 288000, 228000, 408000 } },
	{ {  459000, 324000, 288000, 228000, 408000 } },
	{ {  433500, 324000, 288000, 228000, 408000 } },
	{ {  408000, 324000, 288000, 228000, 408000 } },
};

static struct balanced_throttle tj_throttle = {
	.throt_tab_size = ARRAY_SIZE(tj_throttle_table),
	.throt_tab = tj_throttle_table,
};

static int __init bowser_throttle_init(void)
{
	if (machine_is_bowser())
		balanced_throttle_register(&tj_throttle, "tegra-balanced");
	return 0;
}
module_init(bowser_throttle_init);

static struct nct1008_platform_data bowser_nct1008_pdata = {
	.supported_hwrev = true,
	.ext_range = true,
	.conv_rate = 0x08,
	.shutdown_ext_limit = 105, /* C */
	.shutdown_local_limit = 70, /* C */

	.num_trips = 1,
	.trips = {
		{
			.cdev_type = "suspend_soctherm",
			.trip_temp = 50000,
			.trip_type = THERMAL_TRIP_ACTIVE,
			.upper = 1,
			.lower = 1,
			.hysteresis = 5000,
		},
	},
};

static struct i2c_board_info bowser_i2c4_nct1008_board_info[] = {
	{
		I2C_BOARD_INFO("nct1008", 0x4C),
		.platform_data = &bowser_nct1008_pdata,
		.irq = -1,
	}
};

#define VI_PINMUX(_pingroup, _mux, _pupd, _tri, _io, _lock, _ioreset) \
	{							\
		.pingroup	= TEGRA_PINGROUP_##_pingroup,	\
		.func		= TEGRA_MUX_##_mux,		\
		.pupd		= TEGRA_PUPD_##_pupd,		\
		.tristate	= TEGRA_TRI_##_tri,		\
		.io		= TEGRA_PIN_##_io,		\
		.lock		= TEGRA_PIN_LOCK_##_lock,	\
		.od		= TEGRA_PIN_OD_DEFAULT,		\
		.ioreset	= TEGRA_PIN_IO_RESET_##_ioreset	\
}

#ifndef _MACH_TEGRA_BOARD_BOWSER_H
static struct tegra_pingroup_config mclk_enable =
	VI_PINMUX(CAM_MCLK, VI_ALT3, NORMAL, NORMAL, OUTPUT, DEFAULT, DEFAULT);

static struct tegra_pingroup_config pbb0_enable =
	VI_PINMUX(GPIO_PBB0, VI_ALT3, NORMAL, NORMAL, OUTPUT, DEFAULT, DEFAULT);
#endif

struct sbs_platform_data sbs_pdata = {
	.poll_retry_count = 100,
	.i2c_retry_count = 2,
};

static int bowser_ov2722_power_on(void)
{
  gpio_set_value(CAM1_POWER_DWN_GPIO, 1);
  mdelay(5);

  if(bowser_vdd_cam_1v8 == NULL) {
    bowser_vdd_cam_1v8 = regulator_get(NULL, "vdd_cam_1v8");
    if(WARN_ON(IS_ERR(bowser_vdd_cam_1v8))) {
      pr_err("%s: couldn't get regulator vdd_cam_1v8: %ld\n",
                      __func__, PTR_ERR(bowser_vdd_cam_1v8));
      goto reg_alloc_fail;
    }
  }
  regulator_enable(bowser_vdd_cam_1v8);

  mdelay(5);

  /* Enable AVDD_Cam1 */
  if(bowser_avdd_cam1 == NULL) {
    bowser_avdd_cam1 = regulator_get(NULL, "avdd_cam1");
    if(WARN_ON(IS_ERR(bowser_avdd_cam1))) {
      pr_err("%s: couldn't get regulator avdd_cam1: %ld\n",
                      __func__, PTR_ERR(bowser_avdd_cam1));
      goto reg_alloc_fail;
    }
  }
  regulator_enable(bowser_avdd_cam1);

  mdelay(20);

  gpio_set_value(CAM1_POWER_DWN_GPIO, 0);

  mdelay(5);

  gpio_set_value(CAM_RSTN, 1);
  return 1;

reg_alloc_fail:
  if(bowser_vdd_cam_1v8) {
    regulator_put(bowser_vdd_cam_1v8);
    bowser_vdd_cam_1v8 = NULL;
  }

  if(bowser_avdd_cam1) {
    regulator_put(bowser_avdd_cam1);
    bowser_avdd_cam1 = NULL;
  }

  return -ENODEV;
}

static int bowser_ov2722_power_off(void)
{
  gpio_set_value(CAM1_POWER_DWN_GPIO, 1);
  mdelay(20);

  gpio_set_value(CAM_RSTN, 0);
  mdelay(5);
  regulator_disable(bowser_avdd_cam1);
  mdelay(5);
  regulator_disable(bowser_vdd_cam_1v8);
  return 1;
}

struct ov2722_platform_data bowser_ov2722_data = {
  .power_on = bowser_ov2722_power_on,
  .power_off = bowser_ov2722_power_off,
};

#ifndef _MACH_TEGRA_BOARD_BOWSER_H
static struct i2c_board_info bowser_i2c_board_info_e1625[] = {
	{
		I2C_BOARD_INFO("ov2722", 0x36),
		.platform_data = &bowser_ov2722_data,
	},
};
#endif

#ifndef _MACH_TEGRA_BOARD_BOWSER_H
static int bowser_ov2722_gpio_init(void)
{
	int ret;
	ret = gpio_request(CAM1_POWER_DWN_GPIO, "OV2722_PWD");
	if(ret < 0) {
		pr_err("%s(%d): gpio_request failed %d\n", __func__, __LINE__, ret);
		return -1;
	}
	gpio_direction_output(CAM1_POWER_DWN_GPIO, 1);
	mdelay(20);

	ret = gpio_request(CAM_RSTN, "OV2722_RSTN");
	if(ret < 0) {
		pr_err("%s(%d): gpio_request failed %d\n", __func__, __LINE__, ret);
		return -1;
	}

	gpio_direction_output(CAM_RSTN, 0);

	return 1;
}
#endif

#ifndef _MACH_TEGRA_BOARD_BOWSER_H
static int bowser_pike_init(void)
{
	int ret;
	unsigned pike_irq_gpio = PIKE_IRQ_GPIO;
	char *pike_name = PIKE_NAME;

	ret = gpio_request(CAM_RSTN, "pike_reset");

	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return ret;
	}

	/* Pike reset Active Low */
	ret = gpio_direction_output(CAM_RSTN, 1);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(CAM_RSTN);
		return ret;
	}

	/* Pike reset Active Low */
	gpio_set_value(CAM_RSTN, 1);
	gpio_export(CAM_RSTN, true);

	/* Setup IRQ */
	ret = gpio_request(pike_irq_gpio, pike_name);

	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return ret;
	}

	ret = gpio_direction_input(pike_irq_gpio);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(pike_irq_gpio);
		return ret;
	}

	/* Pike Interrupt */
	gpio_export(pike_irq_gpio, false);

	return 0;
}

static int bowser_camera_init(void)
{
	tegra_pinmux_config_table(&mclk_enable, 1);
	tegra_pinmux_config_table(&pbb0_enable, 1);

  bowser_ov2722_gpio_init();
  mdelay(20);

	i2c_register_board_info(2, bowser_i2c_board_info_e1625,
		ARRAY_SIZE(bowser_i2c_board_info_e1625));
	return 0;
}
#endif

/* MPU board file definition	*/
static struct mpu_platform_data mpu9250_gyro_data = {
	.int_config	= 0x10,
	.level_shifter	= 1,
	/* Located in board_[platformname].h */
	.orientation	= MPU_GYRO_ORIENTATION,
	.sec_slave_type	= SECONDARY_SLAVE_TYPE_COMPASS,
	.sec_slave_id	= COMPASS_ID_AK8963,
	.secondary_i2c_addr	= MPU_COMPASS_ADDR,
	.secondary_orientation	= MPU_COMPASS_ORIENTATION,
	.key		= {0x4E, 0xCC, 0x7E, 0xEB, 0xF6, 0x1E, 0x35, 0x22,
			   0x00, 0x34, 0x0D, 0x65, 0x32, 0xE9, 0x94, 0x89},
};

#define TEGRA_CAMERA_GPIO(_gpio, _label, _value)		\
	{							\
		.gpio = _gpio,					\
		.label = _label,				\
		.value = _value,				\
	}

static struct i2c_board_info __initdata inv_mpu9250_i2c2_board_info[] = {
	{
		I2C_BOARD_INFO(MPU_GYRO_NAME, MPU_GYRO_ADDR),
		.platform_data = &mpu9250_gyro_data,
	},
};

static void mpuirq_init(void)
{
	int ret = 0;
	unsigned gyro_irq_gpio = MPU_GYRO_IRQ_GPIO;
	unsigned gyro_bus_num = MPU_GYRO_BUS_NUM;
	char *gyro_name = MPU_GYRO_NAME;

	pr_info("*** MPU START *** mpuirq_init...\n");

	ret = gpio_request(gyro_irq_gpio, gyro_name);

	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return;
	}

	ret = gpio_direction_input(gyro_irq_gpio);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(gyro_irq_gpio);
		return;
	}
	pr_info("*** MPU END *** mpuirq_init...\n");

	inv_mpu9250_i2c2_board_info[0].irq = gpio_to_irq(MPU_GYRO_IRQ_GPIO);
	i2c_register_board_info(gyro_bus_num, inv_mpu9250_i2c2_board_info,
		ARRAY_SIZE(inv_mpu9250_i2c2_board_info));
}

static int __init bowser_nct1008_init(void)
{
	int nct1008_port;
	int ret = 0;

	if ((board_info.board_id == BOARD_E1612) ||
	    (board_info.board_id == BOARD_E1641) ||
	    (board_info.board_id == BOARD_E1613) ||
	    (board_info.board_id == BOARD_P2454))
	{
		/* per email from Matt 9/10/2012 */
		nct1008_port = TEGRA_GPIO_PX6;
	} else if (board_info.board_id == BOARD_E1611) {
		if (board_info.fab == 0x04)
			nct1008_port = TEGRA_GPIO_PO4;
		else
			nct1008_port = TEGRA_GPIO_PX6;
	} else {
		nct1008_port = TEGRA_GPIO_PX6;
		pr_err("Warning: nct alert_port assumed TEGRA_GPIO_PX6"
			" for unknown bowser board id E%d\n",
			board_info.board_id);
	}

	tegra_add_cdev_trips(bowser_nct1008_pdata.trips,
				&bowser_nct1008_pdata.num_trips);

	bowser_i2c4_nct1008_board_info[0].irq = gpio_to_irq(nct1008_port);
	pr_info("%s: bowser nct1008 irq %d",
			__func__, bowser_i2c4_nct1008_board_info[0].irq);

	ret = gpio_request(nct1008_port, "temp_alert");
	if (ret < 0)
		return ret;

	ret = gpio_direction_input(nct1008_port);
	if (ret < 0) {
		pr_info("%s: calling gpio_free(nct1008_port)", __func__);
		gpio_free(nct1008_port);
	}

	/* bowser has thermal sensor on GEN1-I2C i.e. instance 0 */
	i2c_register_board_info(0, bowser_i2c4_nct1008_board_info,
		ARRAY_SIZE(bowser_i2c4_nct1008_board_info));

	return ret;
}

static struct i2c_board_info __initdata bq20z45_pdata[] = {
	{
		I2C_BOARD_INFO("sbs-battery", 0x0B),
		.platform_data = &sbs_pdata,
	},
};

#ifdef CONFIG_TEGRA_SKIN_THROTTLE
static struct thermal_trip_info skin_trips[] = {
	{
		.cdev_type = "skin-balanced",
		.trip_temp = 54000,
		.trip_type = THERMAL_TRIP_PASSIVE,
		.upper = THERMAL_NO_LIMIT,
		.lower = THERMAL_NO_LIMIT,
		.hysteresis = 0,
	},
};

static struct therm_est_subdevice skin_devs[] = {
	{
		.dev_data = "Tdiode",
		.coeffs = {
			 1,  1, 0, 1,
			 0, -1,-1,-1,
			-1, -1, 0,-1,
			-1, -1,-1,-1,
			-1, -1,-3,-6
		},
	},
	{
		.dev_data = "Tboard",
		.coeffs = {
			25, 20, 14, 11,
			 9,  8,  5,  5,
			 6,  4,  4,  4,
			 2,  1,  3,  2,
			 3,  4,  6,  8
		},
	},
};

static struct therm_est_data skin_data = {
	.num_trips = ARRAY_SIZE(skin_trips),
	.trips = skin_trips,
	.toffset = -12000,
	.polling_period = 1100,
	.passive_delay = 15000,
	.tc1 = 10,
	.tc2 = 1,
	.ndevs = ARRAY_SIZE(skin_devs),
	.devs = skin_devs,
};

static struct throttle_table skin_throttle_table[] = {
	/* CPU_THROT_LOW cannot be used by other than CPU */
	/*      CPU,  C2BUS,  C3BUS,   SCLK,    EMC   */
	{ { 1810500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1785000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1759500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1734000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1708500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1683000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1657500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1632000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1606500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1581000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1555500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1530000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1504500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1479000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1453500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1428000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1402500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1377000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1351500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1326000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1300500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1275000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1249500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1224000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1198500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1173000, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1147500, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1122000, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1096500, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1071000, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1045500, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1020000, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  994500, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  969000, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  943500, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  918000, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  892500, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  867000, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  841500, 564000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  816000, 564000, NO_CAP, NO_CAP, 792000 } },
	{ {  790500, 564000, NO_CAP, 372000, 792000 } },
	{ {  765000, 564000, 468000, 372000, 792000 } },
	{ {  739500, 528000, 468000, 372000, 792000 } },
	{ {  714000, 528000, 468000, 336000, 792000 } },
	{ {  688500, 528000, 420000, 336000, 792000 } },
	{ {  663000, 492000, 420000, 336000, 792000 } },
	{ {  637500, 492000, 420000, 336000, 408000 } },
	{ {  612000, 492000, 420000, 300000, 408000 } },
	{ {  586500, 492000, 360000, 336000, 408000 } },
	{ {  561000, 420000, 420000, 300000, 408000 } },
	{ {  535500, 420000, 360000, 228000, 408000 } },
	{ {  510000, 420000, 288000, 228000, 408000 } },
	{ {  484500, 324000, 288000, 228000, 408000 } },
	{ {  459000, 324000, 288000, 228000, 408000 } },
	{ {  433500, 324000, 288000, 228000, 408000 } },
	{ {  408000, 324000, 288000, 228000, 408000 } },
};

static struct balanced_throttle skin_throttle = {
	.throt_tab_size = ARRAY_SIZE(skin_throttle_table),
	.throt_tab = skin_throttle_table,
};

static int __init bowser_skin_init(void)
{
	if (machine_is_bowser()) {
		balanced_throttle_register(&skin_throttle, "skin-balanced");
		tegra_skin_therm_est_device.dev.platform_data = &skin_data;
		platform_device_register(&tegra_skin_therm_est_device);
	}

	return 0;
}
late_initcall(bowser_skin_init);
#endif

int __init bowser_sensors_init(void)
{
	int err;

	tegra_get_board_info(&board_info);

	err = bowser_nct1008_init();
	if (err)
		return err;

#ifndef _MACH_TEGRA_BOARD_BOWSER_H
	if (is_pike_supported())
		bowser_pike_init();
	else {
	    bowser_camera_init();
#endif
	mpuirq_init();

	i2c_register_board_info(0, bq20z45_pdata,
		ARRAY_SIZE(bq20z45_pdata));

	return 0;
}

int __init bowser_camera_late_init(void)
{
	if (board_info.board_id != BOARD_E1611) {
		pr_err("%s: Bowser not found!\n", __func__);
		return 0;
	}

	return 0;
}

late_initcall(bowser_camera_late_init);
