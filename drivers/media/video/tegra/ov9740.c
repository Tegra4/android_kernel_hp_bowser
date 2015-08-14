/*
 * ov9740.c - ov9740 sensor driver
 *
 * Copyright (c) 2011, NVIDIA, All Rights Reserved.
 *
 * Contributors:
 *      Abhinav Sinha <absinha@nvidia.com>
 *
 * Leverage OV2710.c
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

/**
 * SetMode Sequence for 1280x720. Phase 0. Sensor Dependent.
 * This sequence should put sensor in streaming mode for 1280x720
 * This is usually given by the FAE or the sensor vendor.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <media/ov9740.h>
#include <linux/module.h>

#include "ov9740_tables.h"

#define INTEGRATION_TIME_LINE_HI 0x202
#define INTEGRATION_TIME_LINE_LO 0x203
#define VERTICAL_TOTAL_LENGTH_HI 0x340
#define VERTICAL_TOTAL_LENGTH_LO 0x341
#define MAX_FPS 30

static int ov9740_read_reg(struct i2c_client *client, u16 addr, u8 *val)
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
    data[0] = (u8) (addr >> 8);
    data[1] = (u8) (addr & 0xff);

    msg[1].addr = client->addr;
    msg[1].flags = I2C_M_RD;
    msg[1].len = 1;
    msg[1].buf = data + 2;

    err = i2c_transfer(client->adapter, msg, 2);

    if (err != 2)
        return -EINVAL;

    *val = data[2] ;

    return 0;
}

static int ov9740_write_reg(struct i2c_client *client, u16 addr, u8 val)
{
    int err;
    struct i2c_msg msg;
    unsigned char data[3];
    int retry = 0;

    if (!client->adapter)
        return -ENODEV;

    data[0] = (u8) (addr >> 8);
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
        pr_err("ov9740: i2c transfer failed, retrying %x %x\n",
               addr, val);
        msleep(3);
    } while (retry <= OV9740_MAX_RETRIES);

    return err;
}

static int ov9740_write_table(struct i2c_client *client,
                  const struct ov9740_reg table[],
                  const struct ov9740_reg override_list[],
                  int num_override_regs)
{
    int err;
    const struct ov9740_reg *next;
    int i;
    u16 val;

    pr_info("%s!\n", __func__);

    for (next = table; next->addr != OV9740_TABLE_END; next++) {
        if (next->addr == OV9740_TABLE_WAIT_MS) {
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

        err = ov9740_write_reg(client, next->addr, val);
        //pr_info("%s! ov9740_write_reg: %u\n", __func__, next->addr);

        if (err)
            return err;
    }
    return 0;
}

static int ov9740_set_mode(struct ov9740_info *info, struct ov9740_mode *mode)
{
    int sensor_mode;
    int err;

    pr_info("%s: xres %u yres %u\n", __func__, mode->xres, mode->yres);
    if (mode->xres == 1280 && mode->yres == 720)
        sensor_mode = OV9740_MODE_1280x720;
    else {
        pr_err("%s: invalid resolution supplied to set mode %d %d\n",
               __func__, mode->xres, mode->yres);
        return -EINVAL;
    }

    err = ov9740_write_table(info->i2c_client, mode_table[sensor_mode],
        NULL, 0);
    if (err)
        return err;

    // spec page70, 0x3c00, 0 (60Hz), 4 (50Hz); banding, 0 (60Hz), 1 (50Hz)
    err = ov9740_write_reg(info->i2c_client, 0x3c00, (mode->banding == 0 ? 0 : 4));
    if (err)
        return err;

    info->mode = sensor_mode;
    return 0;
}

static int ov9740_get_status(struct ov9740_info *info,
        struct ov9740_status *dev_status)
{
    int err;
    u8 model_id_H, model_id_L, frame_cnt;

    err = ov9740_read_reg(info->i2c_client, 0x000, (u8 *) &model_id_H);
    err = ov9740_read_reg(info->i2c_client, 0x001, (u8 *) &model_id_L);
    err = ov9740_read_reg(info->i2c_client, 0x003, (u8 *) &dev_status->data);
    err = ov9740_read_reg(info->i2c_client, 0x005, (u8 *) &frame_cnt);

    pr_info("%s(%d): module id: 0x%x%x, frame_cnt:%x\n", __func__, __LINE__, model_id_H, model_id_L, frame_cnt);
    return err;
}

static int ov9740_set_exposure(struct ov9740_info *info, u8 val)
{
	int err = 0;

	switch (val) {
	case OV9740_EXPOSURE_0:
		err = ov9740_write_table(info->i2c_client, exposure_table[OV9740_EXPOSURE_0], NULL, 0);
		break;
	case OV9740_EXPOSURE_NEGATIVE_1:
		err = ov9740_write_table(info->i2c_client, exposure_table[OV9740_EXPOSURE_NEGATIVE_1], NULL, 0);
		break;
	case OV9740_EXPOSURE_NEGATIVE_2:
		err = ov9740_write_table(info->i2c_client, exposure_table[OV9740_EXPOSURE_NEGATIVE_2], NULL, 0);
		break;
	case OV9740_EXPOSURE_NEGATIVE_3:
		err = ov9740_write_table(info->i2c_client, exposure_table[OV9740_EXPOSURE_NEGATIVE_3], NULL, 0);
		break;
	case OV9740_EXPOSURE_1:
		err = ov9740_write_table(info->i2c_client, exposure_table[OV9740_EXPOSURE_1], NULL, 0);
		break;
	case OV9740_EXPOSURE_2:
		err = ov9740_write_table(info->i2c_client, exposure_table[OV9740_EXPOSURE_2], NULL, 0);
		break;
	case OV9740_EXPOSURE_3:
		err = ov9740_write_table(info->i2c_client, exposure_table[OV9740_EXPOSURE_3], NULL, 0);
		break;
	default:
		dev_err(&info->i2c_client->dev, "this exposure setting not supported!\n");
		return -EINVAL;
	}

	return err;
}

static int ov9740_set_wb(struct ov9740_info *info, u8 val)
{
	int err = 0;

	switch (val) {
	case OV9740_WB_AUTO:
		err = ov9740_write_table(info->i2c_client, wb_table[OV9740_WB_AUTO], NULL, 0);
		break;
	case OV9740_WB_INCANDESCENT:
		err = ov9740_write_table(info->i2c_client, wb_table[OV9740_WB_INCANDESCENT], NULL, 0);
		break;
	case OV9740_WB_DAYLIGHT:
		err = ov9740_write_table(info->i2c_client, wb_table[OV9740_WB_DAYLIGHT], NULL, 0);
		break;
	case OV9740_WB_FLUORESCENT:
		err = ov9740_write_table(info->i2c_client, wb_table[OV9740_WB_FLUORESCENT], NULL, 0);
		break;
	case OV9740_WB_CLOUDY:
		err = ov9740_write_table(info->i2c_client, wb_table[OV9740_WB_CLOUDY], NULL, 0);
		break;
	default:
		dev_err(&info->i2c_client->dev, "this wb setting not supported!\n");
		return -EINVAL;
	}

	return err;
}

static long ov9740_ioctl(struct file *file,
             unsigned int cmd, unsigned long arg)
{
    int err;
    struct ov9740_info *info = file->private_data;

    switch (cmd) {
    case OV9740_IOCTL_SET_MODE:
    {
        struct ov9740_mode mode;
        if (copy_from_user(&mode,
                   (const void __user *)arg,
                   sizeof(struct ov9740_mode))) {
            return -EFAULT;
        }

        return ov9740_set_mode(info, &mode);
    }
    case OV9740_IOCTL_GET_STATUS:
    {
        struct ov9740_status dev_status;
        if (copy_from_user(&dev_status,
                   (const void __user *)arg,
                   sizeof(struct ov9740_status))) {
            return -EFAULT;
        }

        err = ov9740_get_status(info, &dev_status);
        if (err)
            return err;
        if (copy_to_user((void __user *)arg, &dev_status,
                 sizeof(struct ov9740_status))) {
            return -EFAULT;
        }
        return 0;
    }
    case OV9740_IOCTL_GET_EXPOSURE_TIME:
    {
      struct ov9740_exposure_time ov9740_et;
      u8 uh, ul;

      if (copy_from_user(&ov9740_et, (const void __user *)arg, sizeof(struct ov9740_exposure_time))) {
        pr_err("ov9740: fail copy from user ov9740_et\n");
        return -EFAULT;
      }

      err = ov9740_read_reg(info->i2c_client, INTEGRATION_TIME_LINE_HI, (u8 *) &uh);
      if (err < 0)
        return err;
      err = ov9740_read_reg(info->i2c_client, INTEGRATION_TIME_LINE_LO, (u8 *) &ul);
      if (err < 0)
        return err;

      ov9740_et.max_fps = MAX_FPS;
      ov9740_et.exposure_line = (uh << 0x8) | ul;

      err = ov9740_read_reg(info->i2c_client, VERTICAL_TOTAL_LENGTH_HI, (u8 *) &uh);
      if (err < 0)
        return err;
      err = ov9740_read_reg(info->i2c_client, VERTICAL_TOTAL_LENGTH_LO, (u8 *) &ul);
      if (err < 0)
        return err;

      ov9740_et.vts = (uh << 0x8) | ul;

      if (copy_to_user((void __user *)arg, &ov9740_et, sizeof(struct ov9740_exposure_time))) {
        pr_err("ov9740: fail copy to user ov9740_et\n");
        return -EFAULT;
      }
      return 0;
    }
    case OV9740_IOCTL_GET_ISO:
    {
        u8 iso_val;
        if (copy_from_user(&iso_val, (const void __user *)arg, sizeof(u8))) {
          pr_err("ov9740: fail copy from user\n");
          return -EFAULT;
        }

        // 0x205 is ANALOG_GAIN register
        err = ov9740_read_reg(info->i2c_client, 0x205, &iso_val);
        if (err)
            return err;
        if (copy_to_user((void __user *)arg, &iso_val, sizeof(u8))) {
          pr_err("ov9740: fail copy to user\n");
          return -EFAULT;
        }
        return 0;
    }
    case OV9740_IOCTL_SET_WHITE_BALANCE:
    {
        return ov9740_set_wb(info, (u8)arg);
    }
    case OV9740_IOCTL_SET_EXPOSURE_COMPENSATION:
    {
        return ov9740_set_exposure(info, (u8)arg);
    }
    case OV9740_IOCTL_SET_FRAME_LENGTH:
    case OV9740_IOCTL_SET_COARSE_TIME:
    case OV9740_IOCTL_SET_GAIN:
    case OV9740_IOCTL_SET_BINNING:
    case OV9740_IOCTL_SET_CAMERA_MODE:
    case OV9740_IOCTL_SYNC_SENSORS:
    case OV9740_IOCTL_SET_3A_LOCK:
    default:
    	pr_info("%s(%d): default cmd=0x%x\n", __FUNCTION__, __LINE__, cmd);
        return -EINVAL;
    }
    return 0;
}

static struct ov9740_info *info;

static int ov9740_open(struct inode *inode, struct file *file)
{
    struct ov9740_status dev_status;
    int err;

    file->private_data = info;
    if (info->pdata && info->pdata->power_on)
        info->pdata->power_on();

    dev_status.data = 0;
    dev_status.status = 0;
    err = ov9740_get_status(info, &dev_status);
    return err;
}

int ov9740_release(struct inode *inode, struct file *file)
{
    if (info->pdata && info->pdata->power_off)
        info->pdata->power_off();
    file->private_data = NULL;
    return 0;
}

static const struct file_operations ov9740_fileops = {
    .owner = THIS_MODULE,
    .open = ov9740_open,
    .unlocked_ioctl = ov9740_ioctl,
    .release = ov9740_release,
};

static struct miscdevice ov9740_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "ov9740",
    .fops = &ov9740_fileops,
};

static int ov9740_probe(struct i2c_client *client,
            const struct i2c_device_id *id)
{
    int err;
//    int i;
    pr_info("ov9740: probing sensor.\n");

    info = kzalloc(sizeof(struct ov9740_info), GFP_KERNEL);
    if (!info) {
        pr_err("ov9740: Unable to allocate memory!\n");
        return -ENOMEM;
    }

    err = misc_register(&ov9740_device);
    if (err) {
        pr_err("ov9740: Unable to register misc device!\n");
        kfree(info);
        return err;
    }

    info->pdata = client->dev.platform_data;
    info->i2c_client = client;

    i2c_set_clientdata(client, info);

    return 0;
}

static int ov9740_remove(struct i2c_client *client)
{
    struct ov9740_info *info;
    info = i2c_get_clientdata(client);
    misc_deregister(&ov9740_device);
    kfree(info);
    return 0;
}

static const struct i2c_device_id ov9740_id[] = {
    { "ov9740", 0 },
    { },
};

MODULE_DEVICE_TABLE(i2c, ov9740_id);

static struct i2c_driver ov9740_i2c_driver = {
    .driver = {
        .name = "ov9740",
        .owner = THIS_MODULE,
    },
    .probe = ov9740_probe,
    .remove = ov9740_remove,
    .id_table = ov9740_id,
};

static int __init ov9740_init(void)
{
    pr_info("ov9740 sensor driver loading\n");
    return i2c_add_driver(&ov9740_i2c_driver);
}

static void __exit ov9740_exit(void)
{
    i2c_del_driver(&ov9740_i2c_driver);
}

module_init(ov9740_init);
module_exit(ov9740_exit);
MODULE_LICENSE("GPL v2");
