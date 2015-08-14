#ifndef _LINUX_ELAN_KTF3K_TS_H
#define _LINUX_ELAN_KTF3K_TS_H

#define ELAN_KTF3K_TS_I2C_BUS		1
#define ELAN_KTF3K_TS_I2C_ADDR		0x10

#define TEGRA_GPIO_PK4			84
#define TEGRA_GPIO_PK2			82

#define ELAN_KTF3K_TS_RESET_GPIO	TEGRA_GPIO_PK4
#define ELAN_KTF3K_TS_INT_GPIO		TEGRA_GPIO_PK2
#define ELAN_KTF3K_TS_VDD		"vdd_3v3_ts"

#define ELAN_KTF3K_TS_NAME		"elan-ktf3k-ts"

struct elan_ktf3k_ts_i2c_platform_data {
	uint16_t version;
	int abs_x_min;
	int abs_x_max;
	int abs_y_min;
	int abs_y_max;
	int intr_gpio;
	int rst_gpio;
	char *ts_vdd_name;
};

extern void elan_ktf3k_ts_pre_resume(void);
extern void elan_ktf3k_ts_pre_suspend(void);

#endif /* #ifndef _LINUX_ELAN_KTF3K_TS_H */

