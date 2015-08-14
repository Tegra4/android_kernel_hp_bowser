/*
 * arch/arm/mach-tegra/board-touch-raydium_spi.c
 *
 * Copyright (c) 2011, NVIDIA Corporation.
 * Copyright (c) 2012, Synaptics Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <linux/rmi.h>
#include "board.h"
#include "board-kai.h"

#define SYNAPTICS_ATTN_GPIO_KAI			TEGRA_GPIO_PZ3
#define SYNAPTICS_ATTN_GPIO_CARDHU		TEGRA_GPIO_PH4
#define SYNAPTICS_ATTN_GPIO_MAYA		TEGRA_GPIO_PK2
#define SYNAPTICS_RESET_GPIO_KAI		TEGRA_GPIO_PK4
#define SYNAPTICS_RESET_GPIO_CARDHU		TEGRA_GPIO_PK4

//#define SYNAPTICS_RESET_GPIO_MAYA		TEGRA_GPIO_PN5
// _PN5 did not work. let's try a few other possibilities.
#define SYNAPTICS_RESET_GPIO_MAYA		TEGRA_GPIO_PK4

#define SYNAPTICS_SPI_CS 0
#define SYNAPTICS_BUTTON_CODES {KEY_HOME, KEY_BACK,}

static unsigned char synaptics_button_codes[] = SYNAPTICS_BUTTON_CODES;

static struct rmi_button_map synaptics_button_map = {
	.nbuttons = ARRAY_SIZE(synaptics_button_codes),
	.map = synaptics_button_codes,
};

struct synaptics_gpio_data {
	int attn_gpio;
	int attn_polarity;
	int reset_gpio;
};

static struct synaptics_gpio_data synaptics_gpio_kai_data = {
	.attn_gpio = SYNAPTICS_ATTN_GPIO_KAI,
	.attn_polarity = RMI_ATTN_ACTIVE_LOW,
	.reset_gpio = SYNAPTICS_RESET_GPIO_KAI,
};

static struct synaptics_gpio_data synaptics_gpio_cardhu_data = {
	.attn_gpio = SYNAPTICS_ATTN_GPIO_CARDHU,
	.attn_polarity = RMI_ATTN_ACTIVE_LOW,
	.reset_gpio = SYNAPTICS_RESET_GPIO_CARDHU,
};

static struct synaptics_gpio_data synaptics_gpio_maya_data = {
	.attn_gpio = SYNAPTICS_ATTN_GPIO_MAYA,
	.attn_polarity = RMI_ATTN_ACTIVE_LOW,
	.reset_gpio = SYNAPTICS_RESET_GPIO_MAYA,
};

static int synaptics_touchpad_gpio_setup(void *gpio_data, bool configure)
{
	struct synaptics_gpio_data *syna_gpio_data =
		(struct synaptics_gpio_data *)gpio_data;

	if (!gpio_data)
		return -EINVAL;

	pr_info("%s: called with configure=%d, SYNAPTICS_ATTN_GPIO=%d\n",
		__func__, configure, syna_gpio_data->attn_gpio);

	if (configure) {
		gpio_request(syna_gpio_data->attn_gpio, "synaptics-irq");
		gpio_direction_input(syna_gpio_data->attn_gpio);

		gpio_request(syna_gpio_data->reset_gpio, "synaptics-reset");
		//gpio_direction_output(syna_gpio_data->reset_gpio, 0);
		// based on  recommendation from Quanta, the second parameter, 1
		// is to "pull up" the reset line:
		gpio_direction_output(syna_gpio_data->reset_gpio, 1);

		msleep(20);
		gpio_set_value(syna_gpio_data->reset_gpio, 1);
		msleep(100);
		/*
		 * for the MAYA platform (Dalmore derivative):
		 * enable VDD line so we have pwoer to the sensor
		 */
		pr_info("maya_touch_init: initializing 1.8v VDD\n");
		gpio_request(TEGRA_GPIO_PH5, "TS_SHDN");
		gpio_direction_output(TEGRA_GPIO_PH5, 1);
		msleep(100);
	} else {
		gpio_free(syna_gpio_data->attn_gpio);
		gpio_free(syna_gpio_data->reset_gpio);
	}
	return 0;
}

static struct rmi_device_platform_data synaptics_maya_platformdata = {
	.sensor_name   = "TM2762",
	.attn_gpio     = SYNAPTICS_ATTN_GPIO_MAYA,
	.attn_polarity = RMI_ATTN_ACTIVE_LOW,
	.gpio_data     = &synaptics_gpio_maya_data,
	.gpio_config   = synaptics_touchpad_gpio_setup,
	.spi_data = {
		.block_delay_us = 15,
		.read_delay_us = 15,
		.write_delay_us = 2,
	},
	.power_management = {
		.nosleep = RMI_F01_NOSLEEP_DEFAULT,
	},
	.f19_button_map = &synaptics_button_map,
//	.f54_direct_touch_report_size = 3024,
#ifdef CONFIG_RMI4_FWLIB
	.firmware_name = "PR1472153",
#endif
};

static struct rmi_device_platform_data synaptics_cardhu_platformdata = {
	.sensor_name   = "TM2106",
	.attn_gpio     = SYNAPTICS_ATTN_GPIO_CARDHU,
	.attn_polarity = RMI_ATTN_ACTIVE_LOW,
	.gpio_data     = &synaptics_gpio_cardhu_data,
	.gpio_config   = synaptics_touchpad_gpio_setup,
	.spi_data = {
		.block_delay_us = 15,
		.read_delay_us = 15,
		.write_delay_us = 2,
	},
	.power_management = {
		.nosleep = RMI_F01_NOSLEEP_OFF,
	},
	.f19_button_map = &synaptics_button_map,
};

static struct rmi_device_platform_data synaptics_kai_platformdata = {
	.sensor_name   = "TM2106",
	.attn_gpio     = SYNAPTICS_ATTN_GPIO_KAI,
	.attn_polarity = RMI_ATTN_ACTIVE_LOW,
	.gpio_data     = &synaptics_gpio_kai_data,
	.gpio_config   = synaptics_touchpad_gpio_setup,
	.spi_data = {
		.block_delay_us = 100,
		.read_delay_us = 100,
		.write_delay_us = 100,
	},
	.power_management = {
		.nosleep = RMI_F01_NOSLEEP_OFF,
	},
	.f19_button_map = &synaptics_button_map,
};

struct spi_board_info synaptics_2106_spi_board[] = {
	{
		.modalias = "rmi_spi",
		.bus_num = 0,
		.chip_select = 0,
		.max_speed_hz = 8*1000*1000,
		.mode = SPI_MODE_3,
		.platform_data = &synaptics_cardhu_platformdata,
	},
};

struct spi_board_info synaptics_2002_spi_board[] = {
	{
		.modalias = "rmi_spi",
		.bus_num = 0,
		.chip_select = 0,
		.max_speed_hz = 8*1000*1000,
		.mode = SPI_MODE_3,
		.platform_data = &synaptics_kai_platformdata,
	},
};

struct spi_board_info synaptics_2762_spi_board[] = {
	{
		.modalias = "rmi_spi",
		.bus_num = 3,
		.chip_select = 2,
		.max_speed_hz = 8*1000*1000,
		.mode = SPI_MODE_3,
		.platform_data = &synaptics_maya_platformdata,
	},
};

int __init touch_init_synaptics(int platform)
{
	struct spi_board_info *board_info;
	int board_info_size;

	msleep(100);
	pr_info("%s: called with platform=%d\n", __func__, platform);

	switch (platform) {
	case 0:
		board_info = synaptics_2002_spi_board;
		board_info_size = ARRAY_SIZE(synaptics_2002_spi_board);
		break;
	case 1:
		board_info = synaptics_2106_spi_board;
		board_info_size = ARRAY_SIZE(synaptics_2106_spi_board);
		break;
	case 2:
		board_info = synaptics_2762_spi_board;
		board_info_size = ARRAY_SIZE(synaptics_2762_spi_board);
		break;
	default:
		pr_info("%s: unknown boardname %d\n", __func__, platform);
		return -EINVAL;
	}

	pr_info("%s: registering synaptics_spi_board\n", __func__);
	pr_info("               modalias     = %s\n",
					board_info->modalias);
	pr_info("               bus_num      = %d\n",
					board_info->bus_num);
	pr_info("               chip_select  = %d\n",
					board_info->chip_select);
	pr_info("               irq          = %d\n",
					board_info->irq);
	pr_info("               max_speed_hz = %d\n",
					board_info->max_speed_hz);
	pr_info("               mode         = %d\n",
					board_info->mode);

	spi_register_board_info(board_info, board_info_size);
	return 0;
}
