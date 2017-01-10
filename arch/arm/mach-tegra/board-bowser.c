/*
 * arch/arm/mach-tegra/board-bowser.c
 *
 * Copyright (c) 2012-2013, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/serial_8250.h>
#include <linux/i2c.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/i2c-tegra.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/spi/spi.h>
#include <linux/spi/rm31080a_ts.h>
#include <linux/tegra_uart.h>
#include <linux/memblock.h>
#include <linux/spi-tegra.h>
#include <linux/nfc/pn544.h>
#include <linux/rfkill-gpio.h>
#include <linux/skbuff.h>
#include <linux/ti_wilink_st.h>
#include <linux/regulator/consumer.h>
#include <linux/smb349-charger.h>
#include <linux/max17048_battery.h>
#include <linux/leds.h>
#include <linux/i2c/at24.h>
#include <linux/of_platform.h>
#include <linux/edp.h>
#include <linux/i2c/i2c-hid.h>
#include <linux/bowser_resolution.h>

#include <asm/hardware/gic.h>

#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/pinmux-t11.h>
#include <mach/iomap.h>
#include <mach/io.h>
#include <mach/io_dpd.h>
#include <mach/i2s.h>
#include <mach/tegra_asoc_pdata.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/usb_phy.h>
#include <mach/gpio-tegra.h>
#include <linux/platform_data/tegra_usb_modem_power.h>
#include <mach/hardware.h>
#include <mach/xusb.h>
#include <linux/ektf3k.h>

#include "board-touch-raydium.h"
#include "board.h"
#include "board-common.h"
#include "clock.h"
#include "board-bowser.h"
#include "devices.h"
#include "gpio-names.h"
#include "fuse.h"
#include "pm.h"
#include "pm-irq.h"
#include "common.h"
#include "tegra-board-id.h"
#include "board-touch.h"
#include <sound/tpa6130a2-plat.h>

int bowser_panel_resolution;
bool panel_initialized = 0;

#if defined(CONFIG_BT_BLUESLEEP) || defined(CONFIG_BT_BLUESLEEP_MODULE)
static struct rfkill_gpio_platform_data bowser_bt_rfkill_pdata = {
		.name           = "bt_rfkill",
		.shutdown_gpio  = TEGRA_GPIO_PQ7,
		.reset_gpio	= TEGRA_GPIO_PQ6,
		.type           = RFKILL_TYPE_BLUETOOTH,
};

static struct platform_device bowser_bt_rfkill_device = {
	.name = "rfkill_gpio",
	.id             = -1,
	.dev = {
		.platform_data = &bowser_bt_rfkill_pdata,
	},
};

static struct resource bowser_bluesleep_resources[] = {
	[0] = {
		.name = "gpio_host_wake",
			.start  = TEGRA_GPIO_PU6,
			.end    = TEGRA_GPIO_PU6,
			.flags  = IORESOURCE_IO,
	},
	[1] = {
		.name = "gpio_ext_wake",
			.start  = TEGRA_GPIO_PEE1,
			.end    = TEGRA_GPIO_PEE1,
			.flags  = IORESOURCE_IO,
	},
	[2] = {
		.name = "host_wake",
			.flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_LOWEDGE,
	},
};

static struct platform_device bowser_bluesleep_device = {
	.name           = "bluesleep",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(bowser_bluesleep_resources),
	.resource       = bowser_bluesleep_resources,
};

static noinline void __init bowser_setup_bt_rfkill(void)
{
	platform_device_register(&bowser_bt_rfkill_device);
}

static noinline void __init bowser_setup_bluesleep(void)
{
	bowser_bluesleep_resources[2].start =
		bowser_bluesleep_resources[2].end =
			gpio_to_irq(TEGRA_GPIO_PU6);
	platform_device_register(&bowser_bluesleep_device);
	return;
}
#elif defined CONFIG_BLUEDROID_PM
static struct resource bowser_bluedroid_pm_resources[] = {
	[0] = {
		.name   = "shutdown_gpio",
		.start  = TEGRA_GPIO_PQ7,
		.end    = TEGRA_GPIO_PQ7,
		.flags  = IORESOURCE_IO,
	},
	[1] = {
		.name = "host_wake",
		.flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	},
	[2] = {
		.name = "gpio_ext_wake",
		.start  = TEGRA_GPIO_PEE1,
		.end    = TEGRA_GPIO_PEE1,
		.flags  = IORESOURCE_IO,
	},
	[3] = {
		.name = "gpio_host_wake",
		.start  = TEGRA_GPIO_PU6,
		.end    = TEGRA_GPIO_PU6,
		.flags  = IORESOURCE_IO,
	},
	[4] = {
		.name = "reset_gpio",
		.start  = TEGRA_GPIO_PQ6,
		.end    = TEGRA_GPIO_PQ6,
		.flags  = IORESOURCE_IO,
	},
};

static struct platform_device bowser_bluedroid_pm_device = {
	.name = "bluedroid_pm",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(bowser_bluedroid_pm_resources),
	.resource       = bowser_bluedroid_pm_resources,
};

static noinline void __init bowser_setup_bluedroid_pm(void)
{
	bowser_bluedroid_pm_resources[1].start =
		bowser_bluedroid_pm_resources[1].end =
				gpio_to_irq(TEGRA_GPIO_PU6);
	platform_device_register(&bowser_bluedroid_pm_device);
}
#endif

static __initdata struct tegra_clk_init_table bowser_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "pll_m",	NULL,		0,		false},
	{ "hda",	"pll_p",	108000000,	false},
	{ "hda2codec_2x", "pll_p",	48000000,	false},
	{ "pwm",	"pll_p",	3187500,	false},
	{ "blink",	"clk_32k",	32768,		true},
	{ "i2s1",	"pll_a_out0",	0,		false},
	{ "i2s3",	"pll_a_out0",	0,		false},
	{ "i2s4",	"pll_a_out0",	0,		false},
	{ "spdif_out",	"pll_a_out0",	0,		false},
	{ "d_audio",	"clk_m",	12000000,	false},
	{ "dam0",	"clk_m",	12000000,	false},
	{ "dam1",	"clk_m",	12000000,	false},
	{ "dam2",	"clk_m",	12000000,	false},
	{ "audio1",	"i2s1_sync",	0,		false},
	{ "audio3",	"i2s3_sync",	0,		false},
	/* Setting vi_sensor-clk to true for validation purpose, will imapact
	 * power, later set to be false.*/
	{ "vi_sensor",	"pll_p",	150000000,	false},
	{ "cilab",	"pll_p",	150000000,	false},
	{ "cilcd",	"pll_p",	150000000,	false},
	{ "cile",	"pll_p",	150000000,	false},
	{ "i2c1",	"pll_p",	3200000,	false},
	{ "i2c2",	"pll_p",	3200000,	false},
	{ "i2c3",	"pll_p",	3200000,	false},
	{ "i2c4",	"pll_p",	3200000,	false},
	{ "i2c5",	"pll_p",	3200000,	false},
	{ "clk_out_2",	"clk_m",	12000000,	true},
	{ NULL,		NULL,		0,		0},
};

static struct tegra_i2c_platform_data bowser_i2c1_platform_data = {
	.adapter_nr	= 0,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.scl_gpio		= {TEGRA_GPIO_I2C1_SCL, 0},
	.sda_gpio		= {TEGRA_GPIO_I2C1_SDA, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data bowser_i2c2_platform_data = {
	.adapter_nr	= 1,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.is_clkon_always = true,
	.scl_gpio		= {TEGRA_GPIO_I2C2_SCL, 0},
	.sda_gpio		= {TEGRA_GPIO_I2C2_SDA, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data bowser_i2c3_platform_data = {
	.adapter_nr	= 2,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.scl_gpio		= {TEGRA_GPIO_I2C3_SCL, 0},
	.sda_gpio		= {TEGRA_GPIO_I2C3_SDA, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data bowser_i2c4_platform_data = {
	.adapter_nr	= 3,
	.bus_count	= 1,
	.bus_clk_rate	= { 10000, 0 },
	.scl_gpio		= {TEGRA_GPIO_I2C4_SCL, 0},
	.sda_gpio		= {TEGRA_GPIO_I2C4_SDA, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data bowser_i2c5_platform_data = {
	.adapter_nr	= 4,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.scl_gpio		= {TEGRA_GPIO_I2C5_SCL, 0},
	.sda_gpio		= {TEGRA_GPIO_I2C5_SDA, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tpa6130a2_platform_data bowser_tpa6130a2_data __initdata_or_module = {
	.power_gpio		= TEGRA_GPIO_PR4,
};

static struct i2c_board_info __initdata rt5640_board_info[] = {
	{
		I2C_BOARD_INFO("rt5640", 0x1c),
	},
	{
		I2C_BOARD_INFO("tpa6130a2", 0x60),
		.platform_data = &bowser_tpa6130a2_data,
	},
};

#ifdef CONFIG_AUO_EDID
static struct i2c_board_info __initdata AUO101_board_info = {
       I2C_BOARD_INFO("AUO101", 0x50),
};
#endif /* CONFIG_AUO_EDID */

static void bowser_i2c_init(void)
{
	struct board_info board_info;

	tegra_get_board_info(&board_info);
	tegra11_i2c_device1.dev.platform_data = &bowser_i2c1_platform_data;
	tegra11_i2c_device2.dev.platform_data = &bowser_i2c2_platform_data;
	tegra11_i2c_device3.dev.platform_data = &bowser_i2c3_platform_data;
	tegra11_i2c_device4.dev.platform_data = &bowser_i2c4_platform_data;
	tegra11_i2c_device5.dev.platform_data = &bowser_i2c5_platform_data;

	platform_device_register(&tegra11_i2c_device5);
	platform_device_register(&tegra11_i2c_device4);
	platform_device_register(&tegra11_i2c_device3);
	platform_device_register(&tegra11_i2c_device2);
	platform_device_register(&tegra11_i2c_device1);

	i2c_register_board_info(0, rt5640_board_info, 2);
#ifdef CONFIG_AUO_EDID
	i2c_register_board_info(0, &AUO101_board_info, 1);
#endif /* CONFIG_AUO_EDID */
}

static struct platform_device *bowser_uart_devices[] __initdata = {
	&tegra_uarta_device,
	&tegra_uartb_device,
	&tegra_uartc_device,
	&tegra_uartd_device,
};
static struct uart_clk_parent uart_parent_clk[] = {
	[0] = {.name = "clk_m"},
	[1] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[2] = {.name = "pll_m"},
#endif
};

static struct tegra_uart_platform_data bowser_uart_pdata;
static struct tegra_uart_platform_data bowser_loopback_uart_pdata;

static void __init uart_debug_init(void)
{
	int debug_port_id;

	debug_port_id = uart_console_debug_init(3);
	if (debug_port_id < 0)
		return;

	bowser_uart_devices[debug_port_id] = uart_console_debug_device;
}

static void __init bowser_uart_init(void)
{
	struct clk *c;
	int i;

	for (i = 0; i < ARRAY_SIZE(uart_parent_clk); ++i) {
		c = tegra_get_clock_by_name(uart_parent_clk[i].name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("Not able to get the clock for %s\n",
						uart_parent_clk[i].name);
			continue;
		}
		uart_parent_clk[i].parent_clk = c;
		uart_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
	}
	bowser_uart_pdata.parent_clk_list = uart_parent_clk;
	bowser_uart_pdata.parent_clk_count = ARRAY_SIZE(uart_parent_clk);
	bowser_loopback_uart_pdata.parent_clk_list = uart_parent_clk;
	bowser_loopback_uart_pdata.parent_clk_count =
						ARRAY_SIZE(uart_parent_clk);
	bowser_loopback_uart_pdata.is_loopback = true;
	tegra_uarta_device.dev.platform_data = &bowser_uart_pdata;
	tegra_uartb_device.dev.platform_data = &bowser_uart_pdata;
	tegra_uartc_device.dev.platform_data = &bowser_uart_pdata;
	tegra_uartd_device.dev.platform_data = &bowser_uart_pdata;

	/* Register low speed only if it is selected */
	if (!is_tegra_debug_uartport_hs())
		uart_debug_init();

	platform_add_devices(bowser_uart_devices,
				ARRAY_SIZE(bowser_uart_devices));
}

static struct resource tegra_rtc_resources[] = {
	[0] = {
		.start = TEGRA_RTC_BASE,
		.end = TEGRA_RTC_BASE + TEGRA_RTC_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = INT_RTC,
		.end = INT_RTC,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device tegra_rtc_device = {
	.name = "tegra_rtc",
	.id   = -1,
	.resource = tegra_rtc_resources,
	.num_resources = ARRAY_SIZE(tegra_rtc_resources),
};

static struct tegra_asoc_platform_data bowser_audio_pdata = {
	.gpio_spkr_en		= TEGRA_GPIO_SPKR_EN,
	.gpio_lout_en		= TEGRA_GPIO_LOUT_EN,
	.gpio_hp_det		= TEGRA_GPIO_HP_DET,
	.gpio_hp_mute		= -1,
	.gpio_int_mic_en	= -1,
	.gpio_ext_mic_en	= -1,
	.gpio_ldo1_en		= TEGRA_GPIO_PR0,
	.gpio_codec1 = TEGRA_GPIO_CODEC1_EN,
	.gpio_codec2 = TEGRA_GPIO_CODEC2_EN,
	.gpio_codec3 = TEGRA_GPIO_CODEC3_EN,
	.i2s_param[HIFI_CODEC]	= {
		.audio_port_id	= 1,
		.is_i2s_master	= 1,
		.i2s_mode	= TEGRA_DAIFMT_I2S,
	},
	.i2s_param[BT_SCO]	= {
		.audio_port_id	= 3,
		.is_i2s_master	= 1,
		.i2s_mode	= TEGRA_DAIFMT_DSP_A,
	},
};

static struct platform_device bowser_audio_device = {
	.name	= "tegra-snd-rt5640",
	.id	= 0,
	.dev	= {
		.platform_data = &bowser_audio_pdata,
	},
};

static struct platform_device *bowser_devices[] __initdata = {
	&tegra_pmu_device,
	&tegra_rtc_device,
	&tegra_udc_device,
#if defined(CONFIG_TEGRA_IOVMM_SMMU) || defined(CONFIG_TEGRA_IOMMU_SMMU)
	&tegra_smmu_device,
#endif
#if defined(CONFIG_TEGRA_AVP)
	&tegra_avp_device,
#endif
#if defined(CONFIG_CRYPTO_DEV_TEGRA_SE)
	&tegra11_se_device,
#endif
	&tegra_ahub_device,
	&tegra_dam_device0,
	&tegra_dam_device1,
	&tegra_dam_device2,
	&tegra_i2s_device1,
	&tegra_i2s_device3,
	&tegra_i2s_device4,
	&tegra_spdif_device,
	&spdif_dit_device,
	&bluetooth_dit_device,
	&tegra_pcm_device,
	&bowser_audio_device,
	&tegra_hda_device,
#if defined(CONFIG_TEGRA_CEC_SUPPORT)
	&tegra_cec_device,
#endif
#if defined(CONFIG_CRYPTO_DEV_TEGRA_AES)
	&tegra_aes_device,
#endif
};

#ifdef CONFIG_USB_SUPPORT
static struct tegra_usb_platform_data tegra_udc_pdata = {
	.port_otg = true,
	.has_hostpc = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.unaligned_dma_buf_supported = false,
	.op_mode = TEGRA_USB_OPMODE_DEVICE,
	.u_data.dev = {
		.vbus_pmu_irq = 0,
		.vbus_gpio = -1,
		.charging_supported = false,
		.remote_wakeup_supported = false,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 8,
		.xcvr_lsfslew = 0,
		.xcvr_lsrslew = 3,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
	},
};

static struct tegra_usb_platform_data tegra_ehci1_utmi_pdata = {
	.port_otg = true,
	.has_hostpc = true,
	.unaligned_dma_buf_supported = false,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.id_det_type = TEGRA_USB_VIRTUAL_ID,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.hot_plug = false,
		.remote_wakeup_supported = true,
		.power_off_on_suspend = true,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 15,
		.xcvr_lsfslew = 0,
		.xcvr_lsrslew = 3,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
		.vbus_oc_map = 0x4,
               .xcvr_hsslew_lsb = 3,
               .xcvr_use_lsb = 1,
	},
};

static struct tegra_usb_platform_data tegra_ehci3_utmi_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.unaligned_dma_buf_supported = false,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.hot_plug = false,
		.remote_wakeup_supported = true,
		.power_off_on_suspend = true,
	},
	.u_cfg.utmi = {
	.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 8,
		.xcvr_lsfslew = 0,
		.xcvr_lsrslew = 3,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
		.vbus_oc_map = 0x5,
	},
};

static struct tegra_usb_platform_data tegra_ehci2_hsic_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.unaligned_dma_buf_supported = false,
	.phy_intf = TEGRA_USB_PHY_INTF_HSIC,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.hot_plug = false,
		.remote_wakeup_supported = true,
		.power_off_on_suspend = true,
	},
};

static struct tegra_usb_platform_data tegra_ehci3_hsic_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.unaligned_dma_buf_supported = false,
	.phy_intf = TEGRA_USB_PHY_INTF_HSIC,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.hot_plug = false,
		.remote_wakeup_supported = true,
		.power_off_on_suspend = true,
	},
};

static struct tegra_usb_otg_data tegra_otg_pdata = {
	.ehci_device = &tegra_ehci1_device,
	.ehci_pdata = &tegra_ehci1_utmi_pdata,
};

static void bowser_usb_init(void)
{
	int usb_port_owner_info = tegra_get_usb_port_owner_info();
	int ret = 0;

	/* Set USB wake sources for bowser */
	tegra_set_usb_wake_source();

	if (!(usb_port_owner_info & UTMI1_PORT_OWNER_XUSB)) {
		tegra_otg_device.dev.platform_data = &tegra_otg_pdata;
		platform_device_register(&tegra_otg_device);
		/* Setup the udc platform data */
		tegra_udc_device.dev.platform_data = &tegra_udc_pdata;
	}

	if (!(usb_port_owner_info & HSIC1_PORT_OWNER_XUSB)) {
		tegra_ehci2_device.dev.platform_data = &tegra_ehci2_hsic_pdata;
		platform_device_register(&tegra_ehci2_device);
	}

	if (!(usb_port_owner_info & UTMI2_PORT_OWNER_XUSB)) {
		tegra_ehci3_device.dev.platform_data = &tegra_ehci3_utmi_pdata;
		platform_device_register(&tegra_ehci3_device);
	}
	else if (!(usb_port_owner_info & HSIC2_PORT_OWNER_XUSB)) {
		ret = gpio_request(TEGRA_GPIO_PQ4, "gps_detect");
		if (ret < 0) {
			printk("%s: gps_detect gpio_request failed (err=%d)\n", __func__, ret);
			return;
		}

		ret = gpio_direction_input(TEGRA_GPIO_PQ4);
		if (ret < 0) {
			printk("%s: set gps_detect input failed (err=%d)\n", __func__, ret);
			gpio_free(TEGRA_GPIO_PQ4);
			return;
		}

		/* KB_COL4: 1 = WWAN, 0 = GPS */
		if (gpio_get_value(TEGRA_GPIO_PQ4)) {
			printk("%s: add ehci3_hsic\n", __func__);
			tegra_ehci3_device.dev.platform_data = &tegra_ehci3_hsic_pdata;
			platform_device_register(&tegra_ehci3_device);
		}
		gpio_free(TEGRA_GPIO_PQ4);
	}
}

static struct tegra_xusb_board_data xusb_bdata = {
	.portmap = TEGRA_XUSB_SS_P0 | TEGRA_XUSB_USB2_P1,
	/* ss_portmap[0:3] = SS0 map, ss_portmap[4:7] = SS1 map */
	.ss_portmap = (TEGRA_XUSB_SS_PORT_MAP_USB2_P1 << 0),
};

static void bowser_xusb_init(void)
{
	int usb_port_owner_info = tegra_get_usb_port_owner_info();

	if (usb_port_owner_info & UTMI2_PORT_OWNER_XUSB) {
		tegra_xusb_init(&xusb_bdata);
		tegra_xusb_register();
	}
}

static struct platform_device bowser_wwan_platform_device = {
	.name = "bowser_wwan",
	.id = -1,
};

static void bowser_modem_init(void) {
	platform_device_register(&bowser_wwan_platform_device);
}

#else
static void bowser_usb_init(void) { }
static void bowser_modem_init(void) { }
static void bowser_gps_init(void) { }
static void bowser_xusb_init(void) { }
#endif

static void bowser_audio_init(void)
{
	struct board_info board_info;

	tegra_get_board_info(&board_info);

	bowser_audio_pdata.codec_name = "rt5640.0-001c";
	bowser_audio_pdata.codec_dai_name = "rt5640-aif1";
}


static struct platform_device *bowser_spi_devices[] __initdata = {
        &tegra11_spi_device4,
};

struct spi_clk_parent spi_parent_clk_bowser[] = {
        [0] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
        [1] = {.name = "pll_m"},
        [2] = {.name = "clk_m"},
#else
        [1] = {.name = "clk_m"},
#endif
};

static struct tegra_spi_platform_data bowser_spi_pdata = {
	.max_dma_buffer         = 16 * 1024,
        .is_clkon_always        = false,
        .max_rate               = 25000000,
};

static void __init bowser_spi_init(void)
{
        int i;
        struct clk *c;
        struct board_info board_info, display_board_info;

        tegra_get_board_info(&board_info);
        tegra_get_display_board_info(&display_board_info);

        for (i = 0; i < ARRAY_SIZE(spi_parent_clk_bowser); ++i) {
                c = tegra_get_clock_by_name(spi_parent_clk_bowser[i].name);
                if (IS_ERR_OR_NULL(c)) {
                        pr_err("Not able to get the clock for %s\n",
                                                spi_parent_clk_bowser[i].name);
                        continue;
                }
                spi_parent_clk_bowser[i].parent_clk = c;
                spi_parent_clk_bowser[i].fixed_clk_rate = clk_get_rate(c);
        }
        bowser_spi_pdata.parent_clk_list = spi_parent_clk_bowser;
        bowser_spi_pdata.parent_clk_count = ARRAY_SIZE(spi_parent_clk_bowser);
		bowser_spi_pdata.is_dma_based = (tegra_revision == TEGRA_REVISION_A01)
							? false : true ;
		tegra11_spi_device4.dev.platform_data = &bowser_spi_pdata;
        platform_add_devices(bowser_spi_devices,
                                ARRAY_SIZE(bowser_spi_devices));
}

#ifndef CONFIG_MACH_BOWSER
static __initdata struct tegra_clk_init_table touch_clk_init_table[] = {
	/* name         parent          rate            enabled */
	{ "extern2",    "pll_p",        41000000,       false},
	{ "clk_out_2",  "extern2",      40800000,       false},
	{ NULL,         NULL,           0,              0},
};
#endif

struct rm_spi_ts_platform_data rm31080ts_bowser_data = {
	.gpio_reset = TOUCH_GPIO_RST_RAYDIUM_SPI,
	.config = 0,
	.platform_id = RM_PLATFORM_D010,
	.name_of_clock = "clk_out_2",
	.name_of_clock_con = "extern2",
};

static struct tegra_spi_device_controller_data dev_cdata = {
	.rx_clk_tap_delay = 0,
	.tx_clk_tap_delay = 16,
};

struct spi_board_info rm31080a_bowser_spi_board[1] = {
	{
	 .modalias = "rm_ts_spidev",
	 .bus_num = 3,
	 .chip_select = 2,
	 .max_speed_hz = 12 * 1000 * 1000,
	 .mode = SPI_MODE_0,
	 .controller_data = &dev_cdata,
	 .platform_data = &rm31080ts_bowser_data,
	 },
};

struct elan_ktf3k_ts_i2c_platform_data elan_ktf3k_ts_platform_data[] = {
	{
		.version = 0x0001,
		.abs_x_min = 0,
		.abs_x_max = 0,
		.abs_y_min = 0,
		.abs_y_max = 0,
		.intr_gpio = ELAN_KTF3K_TS_INT_GPIO,
		.rst_gpio = ELAN_KTF3K_TS_RESET_GPIO,
		.ts_vdd_name = ELAN_KTF3K_TS_VDD,
	},
};

static struct i2c_board_info elan_ktf3k_ts_i2c_devices[] = {
	{
		I2C_BOARD_INFO(ELAN_KTF3K_TS_NAME, 0x10),
		.platform_data = &elan_ktf3k_ts_platform_data,
		.irq = ELAN_KTF3K_TS_INT_GPIO,
	},
};

static int __init bowser_touch_init(void)
{
#ifdef CONFIG_MACH_BOWSER
	if (bowser_panel_resolution == BOWSER_RESOLUTION_FHD) {
		printk(KERN_ERR "%s(): Touchscreen resolution set to FHD\n",__func__);

		elan_ktf3k_ts_platform_data[0].abs_x_max = 1920;
		elan_ktf3k_ts_platform_data[0].abs_y_max = 1080;
	} else {
		printk(KERN_ERR "%s(): Touchscreen resolution set to HD\n",__func__);

		elan_ktf3k_ts_platform_data[0].abs_x_max = 1366;
		elan_ktf3k_ts_platform_data[0].abs_y_max = 768;
	}

	i2c_register_board_info(1, elan_ktf3k_ts_i2c_devices,
			ARRAY_SIZE(elan_ktf3k_ts_i2c_devices));
#else
	struct board_info board_info;

	tegra_get_display_board_info(&board_info);
	tegra_clk_init_from_table(touch_clk_init_table);
	if (board_info.board_id == BOARD_E1582)
		rm31080ts_bowser_data.platform_id = RM_PLATFORM_P005;
	else
		rm31080ts_bowser_data.platform_id = RM_PLATFORM_D010;
	mdelay(20);
	rm31080a_bowser_spi_board[0].irq = gpio_to_irq(TOUCH_GPIO_IRQ_RAYDIUM_SPI);
	touch_init_raydium(TOUCH_GPIO_IRQ_RAYDIUM_SPI,
				TOUCH_GPIO_RST_RAYDIUM_SPI,
				&rm31080ts_bowser_data,
				&rm31080a_bowser_spi_board[0],
				ARRAY_SIZE(rm31080a_bowser_spi_board));
#endif

	return 0;
}

static struct platform_device bowser_misc_device = {
	.name = "bowser_misc",
	.id = -1,
};

static void misc_init(void)
{
	platform_device_register(&bowser_misc_device);
}

#ifdef CONFIG_EDP_FRAMEWORK
static struct edp_manager battery_edp_manager = {
	.name = "battery",
	.imax = 2500
};

static void __init bowser_battery_edp_init(void)
{
	struct edp_governor *g;
	int r;

	r = edp_register_manager(&battery_edp_manager);
	if (r)
		goto err_ret;

	/* start with priority governor */
	g = edp_get_governor("priority");
	if (!g) {
		r = -EFAULT;
		goto err_ret;
	}

	r = edp_set_governor(&battery_edp_manager, g);
	if (r)
		goto err_ret;

	return;

err_ret:
	pr_err("Battery EDP init failed with error %d\n", r);
	WARN_ON(1);
}
#else
static inline void bowser_battery_edp_init(void) {}
#endif
static void __init tegra_bowser_init(void)
{
	/* Here we should determine the panel resolution */
	if(tegra_get_board_panel_id() == NvOdmBoardDisplayPanelId_AUO_14_FHD){
		bowser_panel_resolution = BOWSER_RESOLUTION_FHD;
		panel_initialized = 1;
	}
	else if (tegra_get_board_panel_id() == NvOdmBoardDisplayPanelId_AUO_14_HD){
		bowser_panel_resolution = BOWSER_RESOLUTION_HD;
		panel_initialized = 1;
	}
	else{
		bowser_panel_resolution = BOWSER_RESOLUTION_HD;
		panel_initialized = 0 ;
		pr_err("%s: Doesn't find the panel,use HD to default resolution \n",__func__);
	}
	tegra_clk_init_from_table(bowser_clk_init_table);
	tegra_clk_verify_parents();
	tegra_soc_device_init("bowser");
	tegra_enable_pinmux();
	bowser_pinmux_init();
	bowser_i2c_init();
	misc_init();
	bowser_spi_init();
#ifdef CONFIG_MACH_BOWSER
        gpio_request(TEGRA_GPIO_PK6, "HSIC_RESET");
        gpio_direction_output(TEGRA_GPIO_PK6, 1);
        usleep_range(4000, 10000);
#endif
	bowser_xusb_init();
	bowser_usb_init();
	//bowser_xusb_init();
	bowser_uart_init();
	bowser_audio_init();
	platform_add_devices(bowser_devices, ARRAY_SIZE(bowser_devices));
	tegra_ram_console_debug_init();
	tegra_io_dpd_init();
	bowser_regulator_init();
	bowser_sdhci_init();
	bowser_suspend_init();
	bowser_emc_init();
	bowser_edp_init();
	bowser_touch_init();
	bowser_panel_init();
	bowser_kbc_init();
#if defined(CONFIG_BT_BLUESLEEP) || defined(CONFIG_BT_BLUESLEEP_MODULE)
	bowser_setup_bluesleep();
	bowser_setup_bt_rfkill();
#elif defined CONFIG_BLUEDROID_PM
	bowser_setup_bluedroid_pm();
#endif
#ifdef CONFIG_TEGRA_WDT_RECOVERY
	tegra_wdt_recovery_init();
#endif
	bowser_sensors_init();
	bowser_soctherm_init();
	tegra_register_fuse();

	bowser_modem_init();
}

static void __init bowser_ramconsole_reserve(unsigned long size)
{
	tegra_ram_console_debug_reserve(SZ_1M);
}

static void __init tegra_bowser_dt_init(void)
{
#ifdef CONFIG_USE_OF
	of_platform_populate(NULL,
		of_default_bus_match_table, NULL, NULL);
#endif

	tegra_bowser_init();
}

static void __init tegra_bowser_reserve(void)
{
#if defined(CONFIG_NVMAP_CONVERT_CARVEOUT_TO_IOVMM)
	/* 1920*1200*4*2 = 18432000 bytes */
	tegra_reserve(0, SZ_16M + SZ_2M, SZ_16M + SZ_2M);
#else
	tegra_reserve(SZ_128M, SZ_16M + SZ_2M, SZ_16M + SZ_2M);
#endif
	bowser_ramconsole_reserve(SZ_1M);
}

static const char * const bowser_dt_board_compat[] = {
	"nvidia,bowser",
	NULL
};

MACHINE_START(BOWSER, "bowser")
	.atag_offset	= 0x100,
	.soc		= &tegra_soc_desc,
	.map_io		= tegra_map_common_io,
	.reserve	= tegra_bowser_reserve,
	.init_early	= tegra11x_init_early,
	.init_irq	= tegra_init_irq,
	.handle_irq	= gic_handle_irq,
	.timer		= &tegra_timer,
	.init_machine	= tegra_bowser_dt_init,
	.restart	= tegra_assert_system_reset,
	.dt_compat	= bowser_dt_board_compat,
MACHINE_END

EXPORT_SYMBOL(bowser_panel_resolution);
