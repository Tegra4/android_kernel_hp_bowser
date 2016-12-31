/*
 * drivers/power/tps65090-charger.c
 *
 * Battery charger driver for TI's tps65090
 *
 * Copyright (c) 2012, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/power_supply.h>
#include <linux/power/sbs-battery.h>
#include <linux/mfd/tps65090.h>
#include <linux/uaccess.h>

#define TPS65090_INTR_STS	0x00
#define TPS65090_CG_CTRL0	0x04
#define TPS65090_CG_CTRL1	0x05
#define TPS65090_CG_CTRL2	0x06
#define TPS65090_CG_CTRL3	0x07
#define TPS65090_CG_CTRL4	0x08
#define TPS65090_CG_CTRL5	0x09
#define TPS65090_CG_STATUS1	0x0a
#define TPS65090_CG_STATUS2	0x0b

#define TPS65090_NOITERM	BIT(5)
#define CHARGER_ENABLE		0x01
#define CHARGER_DISABLE		0xfe
#define TPS65090_VACG		0x02
#define IACSET			0x20
#define FASTTIME 		0x1c

#define TPS65090_CGACT		0x10
#define TPS65090_CGCPL 		0x20

#define EC_AC_ONLINE_PATH	"/sys/devices/platform/bowser_misc/get_ac_online"

struct tps65090_charger {
	struct	device	*dev;
	int	irq_base;
	int	ac_online;
	int	prev_ac_online;
	struct power_supply	ac;
	struct tps65090_charger_data *chg_pdata;
	struct delayed_work		work;
};

static enum power_supply_property tps65090_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CHARGE_NOW,
};

static int battery_status_org=1;
static char can_charge = 1;
struct input_dev *event_pipe;
static int create_event_pipe(void);
static int get_ac_online_from_ec(void);
int create_event_pipe(void)
{
	printk("%s() start\n",__func__);

	event_pipe = input_allocate_device();

	event_pipe->name = "Bowser Charger Event";

	set_bit(EV_KEY, event_pipe->evbit);
	set_bit(KEY_WAKEUP_DROPPED, event_pipe->keybit);

	input_register_device(event_pipe);

	return 1;
}

static int send_wakeup_event(void);
int send_wakeup_event(void)
{
	printk("%s() start\npush a wakeup event\n",__func__);
	input_report_key(event_pipe, KEY_WAKEUP_DROPPED,1);
	input_sync(event_pipe);

	msleep(200);

	input_report_key(event_pipe, KEY_WAKEUP_DROPPED,0);
	input_sync(event_pipe);
	return 1;
}

static int tps65090_low_chrg_current(struct tps65090_charger *charger)
{
	int ret;

	ret = tps65090_write(charger->dev->parent, TPS65090_CG_CTRL5,
			TPS65090_NOITERM);
	if (ret < 0) {
		dev_err(charger->dev, "%s(): error reading in register 0x%x\n",
			__func__, TPS65090_CG_CTRL5);
		return ret;
	}
	return 0;
}

static int tps65090_max_adapter_curr(struct tps65090_charger *charger)
{
	int ret;
	uint8_t retval = 0;

	ret = tps65090_read(charger->dev->parent, TPS65090_CG_CTRL0, &retval);
	if (ret < 0) {
		dev_err(charger->dev, "%s(): error reading in register 0x%x\n",
				__func__, TPS65090_CG_CTRL0);
		return ret;
	}

	ret = tps65090_write(charger->dev->parent, TPS65090_CG_CTRL0,
				(retval | IACSET));
	if (ret < 0) {
		dev_err(charger->dev, "%s(): error write 0x20 in register 0x%x\n",
				__func__, TPS65090_CG_CTRL0);
		return ret;
	}
	return 0;
}

static int tps65090_fastcharge_timeout(struct tps65090_charger *charger)
{
	int ret;
	uint8_t retval = 0;

	ret = tps65090_read(charger->dev->parent, TPS65090_CG_CTRL0, &retval);
	if (ret < 0) {
		dev_err(charger->dev, "%s(): error reading in register 0x%x\n",
				__func__, TPS65090_CG_CTRL0);
		return ret;
	}

	ret = tps65090_write(charger->dev->parent, TPS65090_CG_CTRL0,
				(retval | FASTTIME));
	if (ret < 0) {
		dev_err(charger->dev, "%s(): error write 0x1c in register 0x%x\n",
				__func__, TPS65090_CG_CTRL0);
		return ret;
	}
	return 0;
}

static int tps65090_enable_charging(struct tps65090_charger *charger,
	uint8_t enable)
{
	int ret;
	uint8_t retval = 0;

	ret = tps65090_read(charger->dev->parent, TPS65090_CG_CTRL0, &retval);
	if (ret < 0) {
		dev_err(charger->dev, "%s(): error reading in register 0x%x\n",
				__func__, TPS65090_CG_CTRL0);
		return ret;
	}

	if(enable)
	{
		ret = tps65090_write(charger->dev->parent, TPS65090_CG_CTRL0,
					(retval | CHARGER_ENABLE));
	}
	else
	{
		ret = tps65090_write(charger->dev->parent, TPS65090_CG_CTRL0,
					(retval & CHARGER_DISABLE));
	}

	if (ret < 0) {
		dev_err(charger->dev, "%s(): error writing in register 0x%x\n",
				__func__, TPS65090_CG_CTRL0);
		return ret;
	}
	return 0;
}

static int tps65090_config_charger(struct tps65090_charger *charger)
{
	int ret;

	ret = tps65090_low_chrg_current(charger);
	if (ret < 0) {
		dev_err(charger->dev,
			"error configuring low charge current\n");
		return ret;
	}

	ret = tps65090_max_adapter_curr(charger);
	if (ret < 0) {
		dev_err(charger->dev,
			"error configuring max adapter current\n");
		return ret;
	}

	ret = tps65090_fastcharge_timeout(charger);
	if (ret < 0) {
		dev_err(charger->dev,
			"error configuring fastcharger timeout\n");
		return ret;
	}

	return 0;
}

static int tps65090_ac_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	uint8_t _r = 0;
	int ret;
	struct tps65090_charger *charger = container_of(psy,
					struct tps65090_charger, ac);

	if (psp == POWER_SUPPLY_PROP_ONLINE) {
		ret = tps65090_read(charger->dev->parent, TPS65090_INTR_STS, &_r);
		if (ret < 0) {
			dev_err(charger->dev, "%s(): Error in reading reg 0x%x\n",
					__func__, TPS65090_INTR_STS);
			return -EINVAL;
		}
		if (_r & TPS65090_VACG)
			val->intval = 1;
		else {
			ret = get_ac_online_from_ec();

			if (ret == 1) {
				val->intval = 1;
			} else {
				val->intval = 0;
			}
		}
		return 0;
	}
	if (psp == POWER_SUPPLY_PROP_CHARGE_NOW) {
		if (can_charge == 1)
			can_charge = 0;
		else
			can_charge = 1;
		val->intval = can_charge;
		return 0;
	}

	return -EINVAL;
}

static int tps65090_delayed_work(struct work_struct *work)
{
	int ret;
	struct tps65090_charger *charger = container_of(work,
					struct tps65090_charger, work.work);

	ret = sbs_battery_status();
	if(ret == battery_status_org)
	{
		schedule_delayed_work(&charger->work, HZ*2);
		return 0;
	}

	if(ret)
		tps65090_enable_charging(charger, 0);
	else
		tps65090_enable_charging(charger, 1);

	battery_status_org = ret;
	schedule_delayed_work(&charger->work, HZ*2);
}

static irqreturn_t tps65090_charger_isr(int irq, void *dev_id)
{
	struct tps65090_charger *charger = dev_id;
	int ret;
	uint8_t retval = 0;
	uint8_t retval2 = 0;

	ret = tps65090_read(charger->dev->parent, TPS65090_CG_STATUS1, &retval);
	if (ret < 0) {
		dev_err(charger->dev, "%s(): Error in reading reg 0x%x\n",
				__func__, TPS65090_CG_STATUS1);
		goto error;
	}
	msleep(75);
	ret = tps65090_read(charger->dev->parent,0x02, &retval2);
	printk("tps65090 interrupt mask register value [%x]\n",retval2);
	ret = tps65090_read(charger->dev->parent, TPS65090_INTR_STS, &retval2);
	printk("tps65090 interrupt status register value [%x]\n",retval2);
	if (ret < 0) {
		dev_err(charger->dev, "%s(): Error in reading reg 0x%x\n",
				__func__, TPS65090_INTR_STS);
		goto error;
	}


	if (retval2 & TPS65090_VACG) {
		if (can_charge == 1)
		{
			ret = tps65090_enable_charging(charger, 1);
			if (ret < 0)
				goto error;
		}
		charger->ac_online = 1;
		msleep(3000);
	} else {
		charger->ac_online = 0;
	}

	if (charger->prev_ac_online != charger->ac_online) {
		if (charger->chg_pdata->update_status)
			charger->chg_pdata->update_status();
		power_supply_changed(&charger->ac);
	}

error:
	return IRQ_HANDLED;
}

static int get_ac_online_from_ec(void)
{
	static char buf[1];
	struct file* filp = NULL;
	mm_segment_t oldfs;
	ssize_t ret;

	oldfs = get_fs();
	set_fs(get_ds());

	filp = filp_open(EC_AC_ONLINE_PATH, O_RDONLY, 0);
	if(IS_ERR(filp)) {
		printk("%s: Failed to open %s\n", __func__, EC_AC_ONLINE_PATH);
		set_fs(oldfs);
		return -1;
	}

	ret=filp->f_op->read(filp,buf,sizeof(buf),&filp->f_pos);

	if(ret < 0) {
		printk("%s: Failed to read %s", __func__, EC_AC_ONLINE_PATH);
		filp_close(filp, NULL);
		set_fs(oldfs);
		return -1;
	}

	filp_close(filp, NULL);
	set_fs(oldfs);

	if (!strcmp(buf, "1"))
		return 1;
	else
		return 0;
}

static __devinit int tps65090_charger_probe(struct platform_device *pdev)
{
	uint8_t retval = 0;
	int ret;
	struct tps65090_charger *charger_data;
	struct tps65090_platform_data *pdata;

	pdata = dev_get_platdata(pdev->dev.parent);
	if (!pdata) {
		dev_err(&pdev->dev, "%s():no platform data available\n",
				__func__);
		return -ENODEV;
	}

	charger_data = devm_kzalloc(&pdev->dev, sizeof(*charger_data),
			GFP_KERNEL);
	if (!charger_data) {
		dev_err(&pdev->dev, "failed to allocate memory status\n");
		return -ENOMEM;
	}

	charger_data->chg_pdata = pdata->charger_pdata;
	if (!pdata) {
		dev_err(&pdev->dev, "%s()No charger data,exiting\n", __func__);
		return -ENODEV;
	}

	dev_set_drvdata(&pdev->dev, charger_data);

	charger_data->dev = &pdev->dev;

	/* Check for battery presence */
	ret = sbs_battery_detect();
	if (ret < 0) {
		dev_err(charger_data->dev, "No battery. Exiting driver\n");
		return -ENODEV;
	}

	create_event_pipe();

	if (charger_data->chg_pdata->irq_base) {
		ret = request_threaded_irq(charger_data->chg_pdata->irq_base
			+ TPS65090_VACG_IRQ,
			NULL, tps65090_charger_isr, 0, "tps65090-charger",
			charger_data);
		if (ret) {
			dev_err(charger_data->dev, "Unable to register irq %d err %d\n",
					charger_data->irq_base, ret);
			return ret;
		}
	}

	charger_data->ac.name		= "tps65090-ac";
	charger_data->ac.type		= POWER_SUPPLY_TYPE_MAINS;
	charger_data->ac.get_property	= tps65090_ac_get_property;
	charger_data->ac.properties	= tps65090_ac_props;
	charger_data->ac.num_properties	= ARRAY_SIZE(tps65090_ac_props);

        //disable all irq event but VAC event keep
        tps65090_write(charger_data->dev->parent, 0x02, 0x03);
	tps65090_read(charger_data->dev->parent,0x02,&retval);
	printk("tps65090 interrupt mask register value [%x]\n",retval);

	ret = power_supply_register(&pdev->dev, &charger_data->ac);
	if (ret) {
		dev_err(&pdev->dev, "failed: power supply register\n");
		goto fail_suppy_reg;
	}

	ret = tps65090_config_charger(charger_data);
	if (ret < 0) {
		dev_err(&pdev->dev, "charger config failed, err %d\n", ret);
		goto fail_config;
	}

	/* Check for charger presence */
	ret = tps65090_read(charger_data->dev->parent, TPS65090_CG_STATUS1,
			&retval);
	if (ret < 0) {
		dev_err(charger_data->dev, "%s(): Error in reading reg 0x%x",
			__func__, TPS65090_CG_STATUS1);
		goto fail_config;
	}

	if (retval != 0) {
		ret = tps65090_enable_charging(charger_data, 1);
		if (ret < 0) {
			dev_err(charger_data->dev, "error enabling charger\n");
			return ret;
		}
		printk("Bowser AC is online on booting\n");
		charger_data->ac_online = 1;
		power_supply_changed(&charger_data->ac);
	}

	INIT_DELAYED_WORK_DEFERRABLE(&charger_data->work, tps65090_delayed_work);
	schedule_delayed_work(&charger_data->work, HZ);

	return 0;
fail_config:
	power_supply_unregister(&charger_data->ac);

fail_suppy_reg:
	free_irq(charger_data->irq_base, charger_data);
	return ret;
}

static int __devexit tps65090_charger_remove(struct platform_device *pdev)
{
	struct tps65090_charger *charger = dev_get_drvdata(&pdev->dev);

	power_supply_unregister(&charger->ac);
	free_irq(charger->irq_base, charger);
	cancel_delayed_work_sync(&charger->work);
	return 0;
}

static struct platform_driver tps65090_charger_driver = {
	.driver	= {
		.name	= "tps65090-charger",
		.owner	= THIS_MODULE,
	},
	.probe	= tps65090_charger_probe,
	.remove = __devexit_p(tps65090_charger_remove),
};

module_platform_driver(tps65090_charger_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Syed Rafiuddin <srafiuddin@nvidia.com>");
MODULE_DESCRIPTION("tps65090 battery charger driver");
