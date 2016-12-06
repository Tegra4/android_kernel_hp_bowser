/*
 * ov2722.c - ov2722 sensor driver
 *
 * Copyright (c) 2012, NVIDIA, All Rights Reserved.
 *
 * Contributors:
 *      erik lilliebjerg <elilliebjerg@nvidia.com>
 *
 * Leverage ov2722.c
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <media/ov2722.h>
#include <linux/module.h>

#define SIZEOF_I2C_TRANSBUF 32

struct ov2722_reg {
	u16 addr;
	u16 val;
};

struct ov2722_info {
	int mode;
	struct i2c_client *i2c_client;
	struct ov2722_platform_data *pdata;
	u8 i2c_trans_buf[SIZEOF_I2C_TRANSBUF];
	struct ov2722_sensordata sensor_data;
};

#define OV2722_TABLE_WAIT_MS 0
#define OV2722_TABLE_END 1
#define OV2722_MAX_RETRIES 3

static struct ov2722_reg mode_1920x1080[] = {
  {0x0103, 0x01},
  {OV2722_TABLE_WAIT_MS, 10},
  {0x3718, 0x10},
  {0x3702, 0x24},
  {0x373a, 0x60},
  {0x3715, 0x01},
  {0x3703, 0x2e},
  {0x3705, 0x2b},
  {0x3730, 0x30},
  {0x3704, 0x62},
  {0x3f06, 0x3a},
  {0x371c, 0x00},
  {0x371d, 0xc4},
  {0x371e, 0x01},
  {0x371f, 0x28},
  {0x3708, 0x61},
  {0x3709, 0x12},

  {0x3800, 0x00},
  {0x3801, 0x08},
  {0x3802, 0x00},
  {0x3803, 0x02},
  {0x3804, 0x07},
  {0x3805, 0x9b},
  {0x3806, 0x04},
  {0x3807, 0x45},
  {0x3808, 0x07},
  {0x3809, 0x80},
  {0x380a, 0x04},
  {0x380b, 0x40},
  {0x380c, 0x08},
  {0x380d, 0x5c},
  {0x380e, 0x04},
  {0x380f, 0x60},

  {0x3810, 0x00},
  {0x3811, 0x05},
  {0x3812, 0x00},
  {0x3813, 0x02},

  {0x3820, 0x80},
  {0x3821, 0x06},
  {0x3814, 0x11},
  {0x3815, 0x11},
  {0x3612, 0x4b},
  {0x3618, 0x04},

  {0x3a08, 0x01},
  {0x3a09, 0x50},
  {0x3a0a, 0x01},
  {0x3a0b, 0x18},
  {0x3a0d, 0x03},
  {0x3a0e, 0x03},
  {0x4520, 0x00},
  {0x4837, 0x1b},
  {0x3000, 0xff},
  {0x3001, 0xff},
  {0x3002, 0xf0},
  {0x3600, 0x08},
  {0x3621, 0xc0},
  {0x3632, 0x53},
  {0x3633, 0x63},
  {0x3634, 0x24},
  {0x3f01, 0x0c},
  {0x5001, 0xc1},
  {0x3614, 0xf0},
  {0x3630, 0x2d},
  {0x370b, 0x62},
  {0x3706, 0x61},
  {0x4000, 0x02},
  {0x4002, 0xc5},
  {0x4005, 0x08},
  {0x404f, 0x84},
  {0x4051, 0x00},
  {0x5000, 0xff},
  {0x3a18, 0x00},
  {0x3a19, 0x80},
  {0x3503, 0x00},
  {0x4521, 0x00},
  {0x5183, 0xb0},
  {0x5184, 0xb0},
  {0x5185, 0xb0},
  {0x370c, 0x0c},

  {0x3035, 0x00},
  {0x3036, 0x1e},
  {0x3037, 0xa1},
  {0x303e, 0x19},
  {0x3038, 0x06},
  {0x3018, 0x04},
  {0x3000, 0x00},
  {0x3001, 0x00},
  {0x3002, 0x00},
  {0x3a0f, 0x40},
  {0x3a10, 0x38},
  {0x3a1b, 0x48},
  {0x3a1e, 0x30},
  {0x3a11, 0x90},
  {0x3a1f, 0x10},
  {0x3011, 0x22},

  {0x5000, 0xcd},
  {0x3503, 0x07},

  {0x4800, 0x24},
  {0x0100, 0x01},
  {OV2722_TABLE_END, 0x0000}
};

static struct ov2722_reg mode_1280x800[] = {
  {0x0103, 0x01},
  {OV2722_TABLE_WAIT_MS, 10},
  {0x3718, 0x10},
  {0x3702, 0x24},
  {0x373a, 0x60},
  {0x3715, 0x01},
  {0x3703, 0x2e},
  {0x3705, 0x2b},
  {0x3730, 0x30},
  {0x3704, 0x62},
  {0x3f06, 0x3a},
  {0x371c, 0x00},
  {0x371d, 0xc4},
  {0x371e, 0x01},
  {0x371f, 0x28},
  {0x3708, 0x61},
  {0x3709, 0x12},


  {0x3800, 0x01},
  {0x3801, 0x46},

  {0x3802, 0x00},
  {0x3803, 0x92},

  {0x3804, 0x06},
  {0x3805, 0x55},

  {0x3806, 0x03},
  {0x3807, 0xb5},

  {0x3808, 0x05},
  {0x3809, 0x00},

  {0x380a, 0x03},
  {0x380b, 0x20},

  {0x380c, 0x08},
  {0x380d, 0x5e},
  {0x380e, 0x04},
  {0x380f, 0x60},

  {0x3810, 0x00},
  {0x3811, 0x09},
  {0x3812, 0x00},
  {0x3813, 0x02},
  {0x3820, 0x80},
  {0x3821, 0x06},
  {0x3814, 0x11},
  {0x3815, 0x11},
  {0x3612, 0x4b},
  {0x3618, 0x04},

  {0x3a08, 0x01},
  {0x3a09, 0x50},
  {0x3a0a, 0x01},
  {0x3a0b, 0x18},
  {0x3a0d, 0x03},
  {0x3a0e, 0x03},
  {0x4520, 0x00},
  {0x4837, 0x1b},
  {0x3000, 0xff},
  {0x3001, 0xff},
  {0x3002, 0xf0},
  {0x3600, 0x08},
  {0x3621, 0xc0},
  {0x3632, 0x53},
  {0x3633, 0x63},
  {0x3634, 0x24},
  {0x3f01, 0x0c},
  {0x5001, 0xc1},
  {0x3614, 0xf0},
  {0x3630, 0x2d},
  {0x370b, 0x62},
  {0x3706, 0x61},
  {0x4000, 0x02},
  {0x4002, 0xc5},
  {0x4005, 0x08},
  {0x404f, 0x84},
  {0x4051, 0x00},
  {0x5000, 0xff},
  {0x3a18, 0x00},
  {0x3a19, 0x80},
  {0x3503, 0x00},
  {0x4521, 0x00},
  {0x5183, 0xb0},
  {0x5184, 0xb0},
  {0x5185, 0xb0},
  {0x370c, 0x0c},

  {0x3035, 0x00},
  {0x3036, 0x1e},
  {0x3037, 0xa1},
  {0x303e, 0x19},
  {0x3038, 0x06},
  {0x3018, 0x04},
  {0x3000, 0x00},
  {0x3001, 0x00},
  {0x3002, 0x00},
  {0x3a0f, 0x40},
  {0x3a10, 0x38},
  {0x3a1b, 0x48},
  {0x3a1e, 0x30},
  {0x3a11, 0x90},
  {0x3a1f, 0x10},

  {0x3011, 0x22},
  {0x5000, 0xcd},
  {0x3503, 0x07},

  {0x4800, 0x24},
  {0x0100, 0x01},
  {OV2722_TABLE_END, 0x0000}
};

enum {
	OV2722_MODE_1920x1080,
	OV2722_MODE_1280x800,
};


static struct ov2722_reg *mode_table[] = {
	[OV2722_MODE_1920x1080] = mode_1920x1080,
	[OV2722_MODE_1280x800] = mode_1280x800,
};

static inline void ov2722_get_frame_length_regs(struct ov2722_reg *regs,
						u32 frame_length)
{
	regs->addr = 0x380e;
	regs->val = (frame_length >> 8) & 0xff;
	(regs + 1)->addr = 0x380f;
	(regs + 1)->val = (frame_length) & 0xff;
}

static inline void ov2722_get_coarse_time_regs(struct ov2722_reg *regs,
					       u32 coarse_time)
{
	regs->addr = 0x3500;
	regs->val = (coarse_time >> 12) & 0xff;
	(regs + 1)->addr = 0x3501;
	(regs + 1)->val = (coarse_time >> 4) & 0xff;
	(regs + 2)->addr = 0x3502;
	(regs + 2)->val = (coarse_time & 0xf) << 4;
}

static inline void ov2722_get_gain_reg(struct ov2722_reg *regs, u16 gain)
{
	regs->addr = 0x3508;
	regs->val = (gain >> 8) & 0xff;
	(regs + 1)->addr = 0x3509;
	(regs + 1)->val = (gain) & 0xff;
}

static int ov2722_read_reg(struct i2c_client *client, u16 addr, u8 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[3];

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8) (addr >> 8);;
	data[1] = (u8) (addr & 0xff);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = data + 2;

	err = i2c_transfer(client->adapter, msg, 2);

	if (err != 2)

		return -EINVAL;

	*val = data[2];

	return 0;
}

static int ov2722_write_reg(struct i2c_client *client, u16 addr, u8 val)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[3];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

	data[0] = (u8) (addr >> 8);;
	data[1] = (u8) (addr & 0xff);
	data[2] = (u8) (val & 0xff);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;

	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;
		retry++;
		pr_err("ov2722: i2c transfer failed, retrying %x %x\n",
		       addr, val);

		msleep(3);
	} while (retry <= OV2722_MAX_RETRIES);

	return err;
}

static int ov2722_write_bulk_reg(struct i2c_client *client, u8 *data, int len)
{
	int err;
	struct i2c_msg msg;

	if (!client->adapter)
		return -ENODEV;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = len;
	msg.buf = data;

	err = i2c_transfer(client->adapter, &msg, 1);
	if (err == 1)
		return 0;

	pr_err("ov2722: i2c bulk transfer failed at %x\n",
		(int)data[0] << 8 | data[1]);

	return err;
}

static int ov2722_write_table(struct ov2722_info *info,
			      const struct ov2722_reg table[],
			      const struct ov2722_reg override_list[],
			      int num_override_regs)
{
	int err;
	const struct ov2722_reg *next, *n_next;
	u8 *b_ptr = info->i2c_trans_buf;
	unsigned int buf_filled = 0;
	unsigned int i;
	u16 val;

	for (next = table; next->addr != OV2722_TABLE_END; next++) {
		if (next->addr == OV2722_TABLE_WAIT_MS) {
			msleep(next->val);
			continue;
		}

		val = next->val;
		/* When an override list is passed in, replace the reg */
		/* value to write if the reg is in the list            */
		if (override_list) {
			for (i = 0; i < num_override_regs; i++) {
				if (next->addr == override_list[i].addr) {
					val = override_list[i].val;
					break;
				}
			}
		}

		if (!buf_filled) {
			b_ptr = info->i2c_trans_buf;
			*b_ptr++ = next->addr >> 8;
			*b_ptr++ = next->addr & 0xff;
			buf_filled = 2;
		}
		*b_ptr++ = val;
		buf_filled++;

		n_next = next + 1;
		if (n_next->addr != OV2722_TABLE_END &&
			n_next->addr != OV2722_TABLE_WAIT_MS &&
			buf_filled < SIZEOF_I2C_TRANSBUF &&
			n_next->addr == next->addr + 1) {
			continue;
		}

		err = ov2722_write_bulk_reg(info->i2c_client,
			info->i2c_trans_buf, buf_filled);
		if (err)
			return err;
		buf_filled = 0;
	}
	return 0;
}

static int ov2722_set_mode(struct ov2722_info *info, struct ov2722_mode *mode)
{
	int sensor_mode;
	int err;
	struct ov2722_reg reg_list[7];

	pr_info("%s: xres %u yres %u framelength %u coarsetime %u gain %u\n",
		__func__, mode->xres, mode->yres, mode->frame_length,
		mode->coarse_time, mode->gain);

	if (mode->xres == 1920 && mode->yres == 1088)
		sensor_mode = OV2722_MODE_1920x1080;
	else if (mode->xres == 1280 && mode->yres == 800)
		sensor_mode = OV2722_MODE_1280x800;
	else {
		pr_err("%s: invalid resolution supplied to set mode %d %d\n",
		       __func__, mode->xres, mode->yres);
		return -EINVAL;
	}

	/* get a list of override regs for the asking frame length, */
	/* coarse integration time, and gain.                       */
	ov2722_get_frame_length_regs(reg_list, mode->frame_length);
	ov2722_get_coarse_time_regs(reg_list + 2, mode->coarse_time);
	ov2722_get_gain_reg(reg_list + 5, mode->gain);

	err = ov2722_write_table(info, mode_table[sensor_mode],
	reg_list, 6);
	if (err)
		return err;

	info->mode = sensor_mode;
	return 0;
}

static int ov2722_set_frame_length(struct ov2722_info *info, u32 frame_length)
{
	int ret;
	struct ov2722_reg reg_list[2];
	u8 *b_ptr = info->i2c_trans_buf;

	ov2722_get_frame_length_regs(reg_list, frame_length);

	*b_ptr++ = reg_list[0].addr >> 8;
	*b_ptr++ = reg_list[0].addr & 0xff;
	*b_ptr++ = reg_list[0].val & 0xff;
	*b_ptr++ = reg_list[1].val & 0xff;
	ret = ov2722_write_bulk_reg(info->i2c_client, info->i2c_trans_buf, 4);

	return ret;
}

static int ov2722_set_coarse_time(struct ov2722_info *info, u32 coarse_time)
{
	int ret;
	struct ov2722_reg reg_list[3];
	u8 *b_ptr = info->i2c_trans_buf;

	ov2722_get_coarse_time_regs(reg_list, coarse_time);

	*b_ptr++ = reg_list[0].addr >> 8;
	*b_ptr++ = reg_list[0].addr & 0xff;
	*b_ptr++ = reg_list[0].val & 0xff;
	*b_ptr++ = reg_list[1].val & 0xff;
	*b_ptr++ = reg_list[2].val & 0xff;
	ret = ov2722_write_bulk_reg(info->i2c_client, info->i2c_trans_buf, 5);

	return ret;
}

static int ov2722_set_gain(struct ov2722_info *info, u16 gain)
{
	int ret;
	struct ov2722_reg reg_list[2];
	int i = 0;

	ov2722_get_gain_reg(reg_list, gain);

	for (i = 0; i < 2; i++)	{
		ret = ov2722_write_reg(info->i2c_client, reg_list[i].addr,
			reg_list[i].val);
		if (ret)
			return ret;
	}

	return ret;
}

static int ov2722_set_group_hold(struct ov2722_info *info, struct ov2722_ae *ae)
{
	int ret;
	int count = 0;
	bool groupHoldEnabled = false;

	if (ae->gain_enable)
		count++;
	if (ae->coarse_time_enable)
		count++;
	if (ae->frame_length_enable)
		count++;
	if (count >= 2)
		groupHoldEnabled = true;

	if (groupHoldEnabled) {
		ret = ov2722_write_reg(info->i2c_client, 0x3208, 0x00);
		if (ret)
			return ret;
	}

	if (ae->gain_enable)
		ov2722_set_gain(info, ae->gain);
	if (ae->coarse_time_enable)
		ov2722_set_coarse_time(info, ae->coarse_time);
	if (ae->frame_length_enable)
		ov2722_set_frame_length(info, ae->frame_length);

	if (groupHoldEnabled) {
		ret = ov2722_write_reg(info->i2c_client, 0x3208, 0x10);
		if (ret)
			return ret;

		ret = ov2722_write_reg(info->i2c_client, 0x3208, 0xa0);
		if (ret)
			return ret;
	}

	return 0;
}

static int ov2722_get_sensor_id(struct ov2722_info *info)
{
	int ret = 0;
	int i;
	u8  bak = 0;

	if (info->sensor_data.fuse_id_size)
		return 0;


	for (i = 0; i <= 2; i++) {
		ret |= ov2722_read_reg(info->i2c_client, 0x300A + i, &bak);
		info->sensor_data.fuse_id[i] = bak;
		pr_info("%s reg %x value %x\n", __func__, 0x300A + i ,bak);
	}

	if (!ret)
		info->sensor_data.fuse_id_size = i;

	return ret;
}

static int ov2722_get_status(struct ov2722_info *info, u8 *status)
{
	int err;

	*status = 0;
	err = ov2722_read_reg(info->i2c_client, 0x002, status);
	return err;
}


static long ov2722_ioctl(struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	int err;
	struct ov2722_info *info = file->private_data;

	switch (cmd) {
	case OV2722_IOCTL_SET_MODE:
	{
		struct ov2722_mode mode;
		if (copy_from_user(&mode,
				   (const void __user *)arg,
				   sizeof(struct ov2722_mode))) {
			return -EFAULT;
		}

		return ov2722_set_mode(info, &mode);
	}
	case OV2722_IOCTL_SET_FRAME_LENGTH:
		return ov2722_set_frame_length(info, (u32)arg);
	case OV2722_IOCTL_SET_COARSE_TIME:
		return ov2722_set_coarse_time(info, (u32)arg);
	case OV2722_IOCTL_SET_GAIN:
		return ov2722_set_gain(info, (u16)arg);
	case OV2722_IOCTL_SET_GROUP_HOLD:
	{
		struct ov2722_ae ae;
		if (copy_from_user(&ae,
				(const void __user *)arg,
				sizeof(struct ov2722_ae))) {
			pr_info("%s %d\n", __func__, __LINE__);
			return -EFAULT;
		}
		return ov2722_set_group_hold(info, &ae);
	}
	case OV2722_IOCTL_GET_STATUS:
	{
		u8 status;

		err = ov2722_get_status(info, &status);
		if (err)
			return err;
		if (copy_to_user((void __user *)arg, &status,
				 2)) {
			return -EFAULT;
		}
		return 0;
	}
	case OV2722_IOCTL_GET_SENSORDATA:
	{
		err = ov2722_get_sensor_id(info);
		if (err) {
			pr_err("%s %d %d\n", __func__, __LINE__, err);
			return err;
		}
		if (copy_to_user((void __user *)arg,
				&info->sensor_data,
				sizeof(struct ov2722_sensordata))) {
			pr_info("%s %d\n", __func__, __LINE__);
			return -EFAULT;
		}
		return 0;
	}
	default:
		return -EINVAL;
	}
	return 0;
}

static struct ov2722_info *info;

static int ov2722_open(struct inode *inode, struct file *file)
{
	u8 status;
	int err;
	file->private_data = info;
	if (info->pdata && info->pdata->power_on)
		info->pdata->power_on();
	ov2722_get_sensor_id(info);
	err = ov2722_get_status(info, &status);
	pr_info("%s(%d): ----------------- status=0x%x, err=0x%x\n", __FUNCTION__, __LINE__, status, err);
	return 0;
}

int ov2722_release(struct inode *inode, struct file *file)
{
	if (info->pdata && info->pdata->power_off)
		info->pdata->power_off();
	file->private_data = NULL;
	return 0;
}


static const struct file_operations ov2722_fileops = {
	.owner = THIS_MODULE,
	.open = ov2722_open,
	.unlocked_ioctl = ov2722_ioctl,
	.release = ov2722_release,
};

static struct miscdevice ov2722_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ov2722",
	.fops = &ov2722_fileops,
};

static int ov2722_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err;

	pr_info("ov2722: probing sensor.\n");

	info = kzalloc(sizeof(struct ov2722_info), GFP_KERNEL);
	if (!info) {
		pr_err("ov2722: Unable to allocate memory!\n");
		return -ENOMEM;
	}

	err = misc_register(&ov2722_device);
	if (err) {
		pr_err("ov2722: Unable to register misc device!\n");
		kfree(info);
		return err;
	}

	info->pdata = client->dev.platform_data;
	info->i2c_client = client;

	i2c_set_clientdata(client, info);

	return 0;
}

static int ov2722_remove(struct i2c_client *client)
{
	struct ov2722_info *info;
	info = i2c_get_clientdata(client);
	misc_deregister(&ov2722_device);
	kfree(info);
	return 0;
}

static const struct i2c_device_id ov2722_id[] = {
	{ "ov2722", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, ov2722_id);

static struct i2c_driver ov2722_i2c_driver = {
	.driver = {
		.name = "ov2722",
		.owner = THIS_MODULE,
	},
	.probe = ov2722_probe,
	.remove = ov2722_remove,
	.id_table = ov2722_id,
};

static int __init ov2722_init(void)
{
	pr_info("ov2722 sensor driver loading\n");
	return i2c_add_driver(&ov2722_i2c_driver);
}

static void __exit ov2722_exit(void)
{
	i2c_del_driver(&ov2722_i2c_driver);
}

module_init(ov2722_init);
module_exit(ov2722_exit);
MODULE_LICENSE("GPL v2");
