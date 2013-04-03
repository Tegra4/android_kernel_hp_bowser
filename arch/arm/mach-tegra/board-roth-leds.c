/*
 * arch/arm/mach-tegra/board-roth-leds.c
 *
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
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

#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/leds_pwm.h>
#include <linux/pwm.h>
#include <mach/gpio-tegra.h>
#include <mach/pinmux.h>
#include <mach/pinmux-t11.h>

#include "gpio-names.h"
#include "devices.h"
#include "board.h"
#include "board-common.h"
#include "board-roth.h"
#include "tegra-board-id.h"



#ifdef CONFIG_LEDS_PWM

static const struct tegra_pingroup_config led_pinmux = {
	.pingroup	= TEGRA_PINGROUP_KB_COL3,
	.func		= TEGRA_MUX_PWM2
};

static struct led_pwm roth_led_info[] = {
	{
		.name			= "roth-led",
		.default_trigger	= "none",
		.pwm_id			= 2,
		.active_low		= 0,
		.max_brightness		= 255,
		.pwm_period_ns		= 10000000,
	},
};

static struct led_pwm_platform_data roth_leds_pdata = {
	.leds		= roth_led_info,
	.num_leds	= ARRAY_SIZE(roth_led_info),
	.init_gpio	= TEGRA_GPIO_PQ3,
	.OE_gpio	= TEGRA_GPIO_PU1,
	.mux		= &led_pinmux,
};

static struct platform_device roth_leds_pwm_device = {
	.name	= "leds_pwm",
	.id	= -1,
	.dev	= {
		.platform_data = &roth_leds_pdata,
	},
};


#else
static struct gpio_led roth_led_info[] = {
	{
		.name			= "roth-led",
		.default_trigger	= "none",
		.gpio			= TEGRA_GPIO_PQ3,
		.active_low		= 0,
		.retain_state_suspended	= 0,
		.default_state		= LEDS_GPIO_DEFSTATE_OFF,
	},
};

static struct gpio_led_platform_data roth_leds_pdata = {
	.leds		= roth_led_info,
	.num_leds	= ARRAY_SIZE(roth_led_info),
};

static struct platform_device roth_leds_gpio_device = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data = &roth_leds_pdata,
	},
};
#endif

static struct platform_device *roth_led_device[] = {
	&tegra_pwfm2_device,
};

int __init roth_led_init(void)
{
	int err;
#ifdef CONFIG_LEDS_PWM
	platform_device_register(&roth_leds_pwm_device);
#else
	platform_device_register(&roth_leds_gpio_device);
#endif
	err = platform_add_devices(roth_led_device,
			ARRAY_SIZE(roth_led_device));
	if (err < 0) {
		pr_err("LED: pwm led device registration failed\n");
		return err;
	}
	return 0;
}
