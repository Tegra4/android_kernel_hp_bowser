/*
 * PWM LED driver data - see drivers/leds/leds-pwm.c
 */
#ifndef __LINUX_LEDS_PWM_H
#define __LINUX_LEDS_PWM_H

#include <mach/pinmux.h>

#define TEGRA_LED_MAX 1

struct led_pwm {
	const char	*name;
	const char	*default_trigger;
	unsigned	pwm_id;
	u8 		active_low;
	unsigned 	max_brightness;
	unsigned	pwm_period_ns;
};

struct led_pwm_platform_data {
	int					num_leds;
	struct led_pwm				*leds;
	int					init_gpio;
	int					OE_gpio;
	const struct tegra_pingroup_config	*mux;
};

#endif
