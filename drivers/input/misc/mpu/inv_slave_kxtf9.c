/*
* Copyright (C) 2012 nVidia Corp.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*/

/**
 *  @addtogroup  DRIVERS
 *  @brief       Hardware drivers.
 *
 *  @{
 *      @file    inv_slave_kxtf9.c
 *      @brief   A sysfs device driver for Invensense devices
 *      @details This file is part of inv_gyro driver code
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/jiffies.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/poll.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/mpu.h>

#include "inv_gyro.h"
#define KXTF9_CHIP_ID			1
#define KXTF9_RANGE_SET		0
#define KXTF9_BW_SET			4

/* range and bandwidth */



#define KXTF9_BW_800HZ        0x06
#define KXTF9_BW_400HZ        0x05
#define KXTF9_BW_200HZ        0x04
#define KXTF9_BW_100HZ        0x03
#define KXTF9_BW_50HZ         0x02
#define KXTF9_BW_25HZ         0x01
#define KXTF9_BW_12_5HZ       0x00

/*      register definitions */
#define KXTF9_XOUT_HPF_L                (0x00)	/* 0000 0000 */
#define KXTF9_XOUT_HPF_H                (0x01)	/* 0000 0001 */
#define KXTF9_YOUT_HPF_L                (0x02)	/* 0000 0010 */
#define KXTF9_YOUT_HPF_H                (0x03)	/* 0000 0011 */
#define KXTF9_ZOUT_HPF_L                (0x04)	/* 0001 0100 */
#define KXTF9_ZOUT_HPF_H                (0x05)	/* 0001 0101 */
#define KXTF9_XOUT_L                    (0x06)	/* 0000 0110 */
#define KXTF9_XOUT_H                    (0x07)	/* 0000 0111 */
#define KXTF9_YOUT_L                    (0x08)	/* 0000 1000 */
#define KXTF9_YOUT_H                    (0x09)	/* 0000 1001 */
#define KXTF9_ZOUT_L                    (0x0A)	/* 0001 1010 */
#define KXTF9_ZOUT_H                    (0x0B)	/* 0001 1011 */
#define KXTF9_ST_RESP                   (0x0C)	/* 0000 1100 */
#define KXTF9_WHO_AM_I                  (0x0F)	/* 0000 1111 */
#define KXTF9_TILT_POS_CUR              (0x10)	/* 0001 0000 */
#define KXTF9_TILT_POS_PRE              (0x11)	/* 0001 0001 */
#define KXTF9_INT_SRC_REG1              (0x15)	/* 0001 0101 */
#define KXTF9_INT_SRC_REG2              (0x16)	/* 0001 0110 */
#define KXTF9_STATUS_REG                (0x18)	/* 0001 1000 */
#define KXTF9_INT_REL                   (0x1A)	/* 0001 1010 */
#define KXTF9_CTRL_REG1                 (0x1B)	/* 0001 1011 */
#define KXTF9_CTRL_REG2                 (0x1C)	/* 0001 1100 */
#define KXTF9_CTRL_REG3                 (0x1D)	/* 0001 1101 */
#define KXTF9_INT_CTRL_REG1             (0x1E)	/* 0001 1110 */
#define KXTF9_INT_CTRL_REG2             (0x1F)	/* 0001 1111 */
#define KXTF9_INT_CTRL_REG3             (0x20)	/* 0010 0000 */
#define KXTF9_DATA_CTRL_REG             (0x21)	/* 0010 0001 */
#define KXTF9_TILT_TIMER                (0x28)	/* 0010 1000 */
#define KXTF9_WUF_TIMER                 (0x29)	/* 0010 1001 */
#define KXTF9_TDT_TIMER                 (0x2B)	/* 0010 1011 */
#define KXTF9_TDT_H_THRESH              (0x2C)	/* 0010 1100 */
#define KXTF9_TDT_L_THRESH              (0x2D)	/* 0010 1101 */
#define KXTF9_TDT_TAP_TIMER             (0x2E)	/* 0010 1110 */
#define KXTF9_TDT_TOTAL_TIMER           (0x2F)	/* 0010 1111 */
#define KXTF9_TDT_LATENCY_TIMER         (0x30)	/* 0011 0000 */
#define KXTF9_TDT_WINDOW_TIMER          (0x31)	/* 0011 0001 */
#define KXTF9_WUF_THRESH                (0x5A)	/* 0101 1010 */
#define KXTF9_TILT_ANGLE                (0x5C)	/* 0101 1100 */
#define KXTF9_HYST_SET                  (0x5F)	/* 0101 1111 */

#define KXTF9_MAX_DUR (0xFF)
#define KXTF9_MAX_THS (0xFF)
#define KXTF9_THS_COUNTS_P_G (32)

#define KXTF9_CHIP_ID_REG                       0x0F
#define KXTF9_MODE_CTRL_REG             KXTF9_CTRL_REG1


/* mode settings */

#define KXTF9_MODE_NORMAL      0
#define KXTF9_MODE_LOWPOWER    1
#define KXTF9_MODE_SUSPEND     2


/* For the MPU_SLAVE_CONFIG_IRQ_SUSPEND and MPU_SLAVE_CONFIG_IRQ_RESUME */
enum ext_slave_config_irq_type {
	MPU_SLAVE_IRQ_TYPE_NONE,
	MPU_SLAVE_IRQ_TYPE_MOTION,
	MPU_SLAVE_IRQ_TYPE_DATA_READY,
};

struct kxtf9_property {
	int range;
	int bandwidth;
	int mode;
};
struct kxtf9_config {
        unsigned long odr;      /* Output data rate mHz */
        unsigned int fsr;       /* full scale range mg */
        unsigned int ths;       /* Motion no-motion thseshold mg */
        unsigned int dur;       /* Motion no-motion duration ms */
        unsigned int irq_type;
        unsigned char reg_ths;
        unsigned char reg_dur;
        unsigned char reg_odr;
        unsigned char reg_int_cfg1;
        unsigned char reg_int_cfg2;
        unsigned char ctrl_reg1;
};
struct kxtf9_private_data {
	struct kxtf9_config suspend;
	struct kxtf9_config resume;
};

struct kxtf9_private_data private_data;

static struct kxtf9_property kxtf9_static_property = {
	.range = KXTF9_RANGE_SET,
	.bandwidth = KXTF9_BW_SET,
	.mode = KXTF9_MODE_SUSPEND
};


static int kxtf9_set_ths(struct inv_gyro_state_s *st,struct kxtf9_config *config, int apply, long ths)
{
        int result = 0;
        if ((ths * KXTF9_THS_COUNTS_P_G / 1000) > KXTF9_MAX_THS)
                ths = (long)(KXTF9_MAX_THS * 1000) / KXTF9_THS_COUNTS_P_G;

        if (ths < 0)
                ths = 0;

        config->ths =(unsigned int) ths;
        config->reg_ths = (unsigned char)((long)(ths * KXTF9_THS_COUNTS_P_G) / 1000);
        pr_info("THS: %d, 0x%02x\n", config->ths, (int)config->reg_ths);
	if(apply){
	result= inv_secondary_write(KXTF9_WUF_THRESH, config->reg_ths);
	}
        return result;
}


static int kxtf9_set_dur(struct inv_gyro_state_s *st,struct kxtf9_config *config, int apply, long dur)
{
	int result = 0;
	long reg_dur = (dur * config->odr) / 1000000L;
	config->dur = dur;

	if (reg_dur > KXTF9_MAX_DUR)
		reg_dur = KXTF9_MAX_DUR;

	config->reg_dur = (unsigned char)reg_dur;
	pr_info("DUR: %d, 0x%02x\n", config->dur, (int)config->reg_dur);
	if (apply)
		result = inv_secondary_write(KXTF9_WUF_TIMER,(unsigned char)reg_dur);
	return result;

}
/**
 * Sets the IRQ to fire when one of the IRQ events occur.  Threshold and
 * duration will not be used uless the type is MOT or NMOT.
 *
 * @param config configuration to apply to, suspend or resume
 * @param irq_type The type of IRQ.  Valid values are
 * - MPU_SLAVE_IRQ_TYPE_NONE
 * - MPU_SLAVE_IRQ_TYPE_MOTION
 * - MPU_SLAVE_IRQ_TYPE_DATA_READY
 */
static int kxtf9_set_irq(struct inv_gyro_state_s *st,struct kxtf9_config *config, int apply, long irq_type)
{
	int result = 0;
	config->irq_type = (unsigned char)irq_type;
	config->ctrl_reg1 &= ~0x22;
	if (irq_type == MPU_SLAVE_IRQ_TYPE_DATA_READY) {
		config->ctrl_reg1 |= 0x20;
		config->reg_int_cfg1 = 0x38;
		config->reg_int_cfg2 = 0x00;
	} else if (irq_type == MPU_SLAVE_IRQ_TYPE_MOTION) {
		config->ctrl_reg1 |= 0x02;
		if ((unsigned long)config ==
		    (unsigned long)&private_data.suspend)
			config->reg_int_cfg1 = 0x34;
		else
			config->reg_int_cfg1 = 0x24;
		config->reg_int_cfg2 = 0xE0;
	} else {
		config->reg_int_cfg1 = 0x00;
		config->reg_int_cfg2 = 0x00;
	}

	if (apply) {
		/* Must clear bit 7 before writing new configuration */
		result = inv_secondary_write(KXTF9_CTRL_REG1,0x40);
		result = inv_secondary_write(KXTF9_INT_CTRL_REG1,
						 config->reg_int_cfg1);
		result = inv_secondary_write(KXTF9_INT_CTRL_REG2,
						 config->reg_int_cfg2);
		result = inv_secondary_write(KXTF9_CTRL_REG1,
						 config->ctrl_reg1);
	}
	pr_info("CTRL_REG1: %lx, INT_CFG1: %lx, INT_CFG2: %lx\n",
		 (unsigned long)config->ctrl_reg1,
		 (unsigned long)config->reg_int_cfg1,
		 (unsigned long)config->reg_int_cfg2);

	return result;
}


/**
 * Set the Output data rate for the particular configuration
 *
 * @param config Config to modify with new ODR
 * @param odr Output data rate in units of 1/1000Hz
 */
static int kxtf9_set_odr(struct inv_gyro_state_s *st,struct kxtf9_config *config, int apply, long odr)
{
	unsigned char bits;
	int result = 0;

	/* Data sheet says there is 12.5 hz, but that seems to produce a single
	 * correct data value, thus we remove it from the table */
	if (odr > 400000L) {
		config->odr = 800000L;
		bits = 0x06;
	} else if (odr > 200000L) {
		config->odr = 400000L;
		bits = 0x05;
	} else if (odr > 100000L) {
		config->odr = 200000L;
		bits = 0x04;
	} else if (odr > 50000) {
		config->odr = 100000L;
		bits = 0x03;
	} else if (odr > 25000) {
		config->odr = 50000;
		bits = 0x02;
	} else if (odr != 0) {
		config->odr = 25000;
		bits = 0x01;
	} else {
		config->odr = 0;
		bits = 0;
	}

	if (odr != 0)
		config->ctrl_reg1 |= 0x80;
	else
		config->ctrl_reg1 &= ~0x80;

	config->reg_odr = bits;
	kxtf9_set_dur(st,config, apply, config->dur);
	pr_info("ODR: %ld, 0x%02x\n", config->odr, (int)config->ctrl_reg1);
	if (apply) {
        result = inv_secondary_write(KXTF9_DATA_CTRL_REG, config->reg_odr);
        result = inv_secondary_write(KXTF9_CTRL_REG1, 0x40);
        result = inv_secondary_write(KXTF9_CTRL_REG1, config->ctrl_reg1);

	}
	return result;
}

static int kxtf9_set_bandwidth(struct inv_gyro_state_s *st, long odr)
{
	int res = 0;
	unsigned char data;
	int Bandwidth = 0;
        long config_odr;
	if (odr > 400000L) {
		config_odr = 800000L;
	} else if (odr > 200000L) {
		config_odr = 400000L;
	} else if (odr > 100000L) {
		config_odr = 200000L;
	} else if (odr > 50000) {
		config_odr = 100000L;
	} else if (odr > 25000) {
		config_odr = 50000;
	} else if (odr != 0) {
		config_odr = 25000;
	} else {
		config_odr = 0;
	}
        res=kxtf9_set_odr(st,&private_data.resume,1,config_odr);
	return res;
}
/**
 * Set the full scale range of the accels
 *
 * @param config pointer to configuration
 * @param fsr requested full scale range
 */
static int kxtf9_set_fsr(struct inv_gyro_state_s *st,struct kxtf9_config *config, int apply, long fsr)
{
	int result = 0;

	config->ctrl_reg1 = (config->ctrl_reg1 & 0xE7);
	if (fsr <= 2000) {
		config->fsr = 2000;
		config->ctrl_reg1 |= 0x00;
	} else if (fsr <= 4000) {
		config->fsr = 4000;
		config->ctrl_reg1 |= 0x08;
	} else {
		config->fsr = 8000;
		config->ctrl_reg1 |= 0x10;
	}

	pr_info("FSR: %d\n", config->fsr);
	if (apply) {
		/* Must clear bit 7 before writing new configuration */
		result = inv_secondary_write(KXTF9_CTRL_REG1, 0x40);
		result = inv_secondary_write(KXTF9_CTRL_REG1,
						 config->ctrl_reg1);
	}
	return result;
}
static int kxtf9_set_range(struct inv_gyro_state_s *st, unsigned char Range)
{
	int res = 0;
        long fsr;
	if (Range >= 4)
		return -1;
	pr_info("%s: Range: %d\n", __func__, Range);
	switch (Range) {
	case 0://+-2G
                fsr=2000;
		break;
	case 1://+-4G
                fsr=4000;
		break;
	case 2://+-8G
                fsr=8000;
		break;
	case 3://+-8G
                fsr=8000;
		break;
	default:
		break;
	}
        res=kxtf9_set_fsr(st,&private_data.resume,1,fsr);
	kxtf9_static_property.range = Range;
	return res;
}

static int setup_slave_kxtf9(struct inv_gyro_state_s *st)
{
	int result;
	unsigned char data[4];
	result = set_3050_bypass(st, 1);
	if (result)
		return result;
	/*read secondary i2c ID register */
	result = inv_secondary_read(KXTF9_CHIP_ID_REG, 2, data);
	pr_info("%s: id: %x result:%x\n", __func__, data[0],result);

	if (result)
		return result;
	if (KXTF9_CHIP_ID != data[0])
		return result;


        result = inv_secondary_write(KXTF9_CTRL_REG1, 0x40);
        result = inv_secondary_write(KXTF9_DATA_CTRL_REG, 0x36);
        result = inv_secondary_write(KXTF9_CTRL_REG3, 0xcd);
        mdelay(2);
	private_data.resume.ctrl_reg1 = 0xC0;
	private_data.suspend.ctrl_reg1 = 0x40;
#if 1
	result = kxtf9_set_dur(st,&private_data.suspend,
			       false, 1000);
        if (result) return result;
        result = kxtf9_set_dur(st,&private_data.resume,
                               false, 2540);
        if (result) return result;

        result = kxtf9_set_odr(st,&private_data.suspend,
                               false, 50000);
        if (result) return result;
        result = kxtf9_set_odr(st,&private_data.resume,
                               false, 200000L);
        if (result) return result;
        result = kxtf9_set_fsr(st,&private_data.suspend,
                               false, 2000);
        if (result) return result;
        result = kxtf9_set_fsr(st,&private_data.resume,
                               false, 2000);
        if (result) return result;
        result = kxtf9_set_ths(st,&private_data.suspend,
                               false, 80);
        if (result) return result;
        result = kxtf9_set_ths(st,&private_data.resume,
                               false, 40);
        if (result) return result;

        result = kxtf9_set_irq(st,&private_data.suspend,
                               false, MPU_SLAVE_IRQ_TYPE_NONE);
        if (result) return result;
        result = kxtf9_set_irq(st,&private_data.resume,
                               false, MPU_SLAVE_IRQ_TYPE_NONE);
        if (result) return result;
#endif


	result = set_3050_bypass(st, 0);
	if (result)
		return result;
	/*AUX(accel), slave address is set inside set_3050_bypass*/
	/* bma250 x axis LSB register address is 6 */
	result = inv_i2c_single_write(st, 0x18, KXTF9_XOUT_L);
	return result;
}

static int kxtf9_set_mode(struct inv_gyro_state_s *st, unsigned char Mode)
{
	int res = 0;
	unsigned char data = 0;
        int result=0;
	if (Mode >= 3)
		return -1;
	pr_info("%s: Mode: %d\n", __func__, Mode);

	res = inv_secondary_read(KXTF9_MODE_CTRL_REG, 1, &data);
	if (res)
		return res;
	data &= 0x3F;
	switch (Mode) {
	case KXTF9_MODE_NORMAL:
	  data |= 0xC0;
		break;
	case KXTF9_MODE_LOWPOWER:
		data |= 0x40;
		break;
	case KXTF9_MODE_SUSPEND:
		data |= 0x40;
		break;
	default:
		break;
	}
	res = inv_secondary_write(KXTF9_MODE_CTRL_REG, data);
	kxtf9_static_property.mode = Mode;
	return res;
}
static int suspend_slave_kxtf9(struct inv_gyro_state_s *st)
{
	int result=0;
        unsigned char data;

	printk("suspend_slave_kxtf9\n");
	if (kxtf9_static_property.mode == KXTF9_MODE_SUSPEND)
		return 0;
#if 1
	/*set to bypass mode */
	result = set_3050_bypass(st, 1);
	if (result)
		return result;
	/* Wake up */
	result = inv_secondary_write(KXTF9_CTRL_REG1, 0x40);
	/* INT_CTRL_REG1: */
	result = inv_secondary_write(KXTF9_INT_CTRL_REG1,
					 private_data.suspend.reg_int_cfg1);

	/* WUF_THRESH: */
	result = inv_secondary_write(KXTF9_WUF_THRESH,
					 private_data.suspend.reg_ths);

	/* DATA_CTRL_REG */
	result = inv_secondary_write(KXTF9_DATA_CTRL_REG,
					 private_data.suspend.reg_odr);

	/* WUF_TIMER */
	result = inv_secondary_write(KXTF9_WUF_TIMER,
					 private_data.suspend.reg_dur);
	/* Normal operation  */
	result = inv_secondary_write(KXTF9_CTRL_REG1,
					 private_data.suspend.ctrl_reg1);

	result = inv_secondary_read(KXTF9_INT_REL, 1, &data);

	kxtf9_set_mode(st, KXTF9_MODE_SUSPEND);
	/* no need to recover to non-bypass mode because we need it now */
#endif
	return result;
}
static int resume_slave_kxtf9(struct inv_gyro_state_s *st)
{
	int result;
        unsigned char data;
        unsigned char data1[2];
	if (kxtf9_static_property.mode == KXTF9_MODE_NORMAL)
		return 0;
	/*set to bypass mode */
	result = set_3050_bypass(st, 1);
	if (result)
		return result;
        result = inv_secondary_write(KXTF9_CTRL_REG1,0x40);
	/* INT_CTRL_REG1: */
	result = inv_secondary_write(KXTF9_INT_CTRL_REG1,
					 private_data.resume.reg_int_cfg1);


	/* WUF_THRESH: */
	result = inv_secondary_write(KXTF9_WUF_THRESH,
					 private_data.resume.reg_ths);
	/* DATA_CTRL_REG */
	result = inv_secondary_write(KXTF9_DATA_CTRL_REG,
					 private_data.resume.reg_odr);
	result = inv_secondary_write(KXTF9_WUF_TIMER,
					 private_data.resume.reg_dur);
	/* Normal operation  */
	result = inv_secondary_write(KXTF9_CTRL_REG1,
					 private_data.resume.ctrl_reg1);

	kxtf9_set_mode(st, KXTF9_MODE_NORMAL);
	result = inv_secondary_read(KXTF9_INT_REL, 1, &data);


	/* recover bypass mode */
	result = set_3050_bypass(st, 0);
	return result;
}
static int combine_data_slave_kxtf9(unsigned char *in, short *out)
{
	out[0] = (in[0] | (in[1]<<8));
	out[1] = (in[2] | (in[3]<<8));
	out[2] = (in[4] | (in[5]<<8));
	return 0;
}
static int get_mode_slave_kxtf9(struct inv_gyro_state_s *st)
{
	if (kxtf9_static_property.mode == KXTF9_MODE_SUSPEND)
		return 0;
	else if (kxtf9_static_property.mode == KXTF9_MODE_NORMAL)
		return 1;
	return -1;
};
/**
 *  set_lpf_kxtf9() - set lpf value
 */

static int set_lpf_kxtf9(struct inv_gyro_state_s *st, int rate)
{
	int hz[7] = {800000L, 400000L, 200000L, 100000L, 50000L, 25000L, 0L};
	int   d[7] = {6, 5, 4, 3, 2, 1, 0};
	int i, h, data, result;
	h = rate;
	i = 0;
	while ((h < hz[i]) && (i < (ARRAY_SIZE(d) - 1)))
		i++;
	data = d[i];
	pr_info("%s:  rate%d,data:%d\n", __func__, rate,data);
	result = set_3050_bypass(st, 1);
	if (result)
		return result;
	result = kxtf9_set_bandwidth(st, rate);
	result |= set_3050_bypass(st, 0);

	return result;
}
/**
 *  set_fs_kxtf9() - set range value
 */

static int set_fs_kxtf9(struct inv_gyro_state_s *st, int fs)
{
	int result;
	result = set_3050_bypass(st, 1);
	if (result)
		return result;
	result = kxtf9_set_range(st, (unsigned char) fs);
	result |= set_3050_bypass(st, 0);
	if (result)
		return -EINVAL;
	return result;
}

static int bypass_read_kxtf9(struct inv_gyro_state_s *st, unsigned char *data)
{
	int result;
	unsigned char reg;
	result = set_3050_bypass(st, 1);
	if (result)
		return result;

	result = inv_secondary_read(KXTF9_INT_SRC_REG2, 1, &reg);
	if (result)
		return result;

	result = inv_secondary_read(KXTF9_XOUT_L, 6, data);

	result |= set_3050_bypass(st, 0);
	if (result)
		return -EINVAL;

	return result;
}

static struct inv_mpu_slave slave_kxtf9 = {
	.suspend = suspend_slave_kxtf9,
	.resume  = resume_slave_kxtf9,
	.setup   = setup_slave_kxtf9,
	.combine_data = combine_data_slave_kxtf9,
	.get_mode = get_mode_slave_kxtf9,
	.set_lpf = set_lpf_kxtf9,
	.set_fs  = set_fs_kxtf9,
	.bypass_read = bypass_read_kxtf9
};

int inv_register_kxtf9_slave(struct inv_gyro_state_s *st)
{
	st->mpu_slave = &slave_kxtf9;
	return 0;
}
