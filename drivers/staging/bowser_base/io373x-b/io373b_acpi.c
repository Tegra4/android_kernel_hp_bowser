#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include "io373b.h"

#define FULLY_DISCHARGED	0x0010	//POWER_SUPPLY_STATUS_NOT_CHARGING (empty)
#define FULLY_CHARGED		0x0020	//POWER_SUPPLY_STATUS_FULL
#define DISCHARGING		0x0040	//POWER_SUPPLY_STATUS_DISCHARGING
#define INITIALIZED		0x0080	//POWER_SUPPLY_STATUS_UNKNOWN if not set
#define BAT_TYPE_VALID		0x20
#define BAT_IN			0x10
#define ADAPTER_IN		0x01
#define MANU_NAME_SIZE		(11 + 1)

#define MAX_BAT_NUM		2

enum kbc_command {
	BAT_TEMP		= 0x04,	//BAT_Temperature		Read	Word	0.1^oC
	BAT_VOLTAGE_NOW		= 0x05,	//BAT_Voltage			Read	Word	mV
	BAT_CURRENT_NOW		= 0x06,	//BAT_Current			Read	Word	mA
	BAT_CURRENT_AVG		= 0x07,	//BAT_AverageCurrent		Read	Word	mA
	BAT_CAPACITY_REL	= 0x09,	//BAT_RelativeStateOfCharge	Read	Word	Percent
	BAT_CAPACITY_REMAIN	= 0x0a,	//BAT_RemainingCapacity		Read	Word	mAh or 10mWh
	BAT_CAPACITY_FULL	= 0x0b,	//BAT_FullChargeCapacity	Read	Word	mAh or 10mWh
	BAT_CHARGING_CURRENT	= 0x0c,	//BAT_ChargingCurrent		Read	Word	mA
	BAT_CHARGING_VOLTAGE	= 0x0d,	//BAT_ChargingVoltage		Read	Word	mV
	BAT_STATUS		= 0x0e,	//BAT_BatteryStatus		Read	Word	bit flags
	BAT_CYCLE_COUNT		= 0x0f,	//BAT_CycleCount		Read	Word	count
	BAT_DESIGN_CAPACITY	= 0x10,	//BAT_DesignCapacity		Read	Word	mAh or 10mWh
	BAT_DESIGN_VOLTAGE	= 0x11,	//BAT_DesignVoltage		Read	Word	mV
	BAT_MANUFACTURE_DATE	= 0x12,	//BAT_ManufactureDateRead	Read	Word	unsigned int
	BAT_SERIAL_NUMBER	= 0x13,	//BAT_SerialNumber		Read	Word	number
	BAT_MANU_NAME		= 0x1C,	//ManufacturerName		Read	String	11+1
	ACDC_FLAG		= 0x14,	//ACDC_FLAG			Read	Byte	number
};

#define dev_dbg_sparse(dev, sec, format, arg...)             \
do {                                                         \
	static unsigned long last_time = 0;                  \
	if (last_time == 0 ||                                \
	    time_after(jiffies, last_time + HZ * sec)) {     \
		dev_dbg(dev, format, ##arg);                 \
		last_time = jiffies;                         \
	}                                                    \
} while (0)

struct io373b_acpi {
	struct platform_device *pdev;
	struct power_supply bat[MAX_BAT_NUM];
	struct power_supply ac[MAX_BAT_NUM];
	struct notifier_block nb;
	int num_bat;
	struct ua /* user access */
	{
		unsigned char cmd;
		int len;
		bool to_ascii;
	} ua;
	struct delayed_work work;
	int last_bat_status;
	int last_bat_cap;
};

static ssize_t user_cmd(struct device *dev, struct device_attribute *attr,
                        const char *buf, size_t count)
{
	struct io373b_acpi *acpi = dev_get_drvdata(dev);

	acpi->ua.len = 0; /* invalidate len */

	if (count == 2) {
		acpi->ua.cmd = buf[0];
		acpi->ua.len = buf[1];
		acpi->ua.to_ascii = false;
	} else {
		unsigned long cmd;
		int len;

		if ((sscanf(buf, "%lx %d", &cmd, &len) != 2) ||
		    (cmd > 0xFF)                             ||
		    (len <= 0 || 256 < len))
			return -EINVAL;

		acpi->ua.cmd = (unsigned char) cmd;
		acpi->ua.len = len;
		acpi->ua.to_ascii = true;
	}

	return count;
}
static DEVICE_ATTR(cmd, S_IWUG, NULL, user_cmd);

static ssize_t user_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct io373b_acpi *acpi = dev_get_drvdata(dev);
	int ret, i, n_char = 0;
	unsigned char data[256];

	if (acpi->ua.len <= 0 || 256 < acpi->ua.len)
		return -EINVAL;

	ret = io373b_subdev_read(&acpi->pdev->dev, acpi->ua.cmd, data, acpi->ua.len);
	if (ret < 0)
		return ret;

	if (acpi->ua.to_ascii) {
		for (i = 0; i < acpi->ua.len; i++)
			n_char += sprintf(buf + n_char, "%02x ", data[i]);
		n_char += sprintf(buf + n_char, "\n");
		return n_char;
	} else {
		memcpy(buf, data, acpi->ua.len);
		return acpi->ua.len;
	}
}
static DEVICE_ATTR(data, S_IRUGO, user_data, NULL);

static int io373b_acpi_create_dev_attrs(struct io373b_acpi *acpi)
{
	int err;

	if ((err = device_create_file(&acpi->pdev->dev, &dev_attr_cmd))) {
		dev_err(&acpi->pdev->dev, "Failed to create attr cmd - %d\n", err);
		return err;
	}
	if ((err = device_create_file(&acpi->pdev->dev, &dev_attr_data)))
	{
		dev_err(&acpi->pdev->dev, "Failed to create attr data - %d\n", err);
		device_remove_file(&acpi->pdev->dev, &dev_attr_cmd);
		return err;
	}

	return 0;
}

static void io373b_acpi_remove_dev_attrs(struct io373b_acpi *acpi)
{
	device_remove_file(&acpi->pdev->dev, &dev_attr_cmd);
	device_remove_file(&acpi->pdev->dev, &dev_attr_data);
}

static enum power_supply_property io373b_acpi_bat_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

static enum power_supply_property io373b_acpi_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int io373b_acpi_get_bat_status(struct io373b_acpi *acpi, int bat_idx)
{
	int ret;
	unsigned char cmd_flag = (bat_idx == 0) ? 0 : 0x80; /* 0x80 for the 2nd */
	unsigned char ac_dc;
	unsigned short bat_status;
	short bat_cur;

	if ((ret = io373b_subdev_read(&acpi->pdev->dev, ACDC_FLAG | cmd_flag, &ac_dc, 1)) < 0 ||
	    (ret = io373b_subdev_read(&acpi->pdev->dev, BAT_STATUS | cmd_flag, (unsigned char *) &bat_status, 2)) < 0 ||
	    (ret = io373b_subdev_read(&acpi->pdev->dev, BAT_CURRENT_NOW | cmd_flag, (unsigned char *) &bat_cur, 2)) < 0)
		return ret;

	bat_status = le16_to_cpu(bat_status);
	bat_cur = le16_to_cpu(bat_cur);

	dev_dbg_sparse(&acpi->pdev->dev, 10, "ACDC_FLAG = %02x, BAT_STATUS = %04x, BAT_CURRENT_NOW = %d mA\n", ac_dc, bat_status, bat_cur);

	if (!(ac_dc & BAT_TYPE_VALID))
		return POWER_SUPPLY_STATUS_UNKNOWN;
	else if (bat_status & FULLY_CHARGED)
		return POWER_SUPPLY_STATUS_FULL;
	else if (bat_cur == 0)
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	else if (bat_cur > 0)
		return POWER_SUPPLY_STATUS_CHARGING;
	else
		return POWER_SUPPLY_STATUS_DISCHARGING;
}

static int io373b_acpi_get_bat_cap(struct io373b_acpi *acpi, int bat_idx)
{
	int ret;
	unsigned char cmd_flag = (bat_idx == 0) ? 0 : 0x80; /* 0x80 for the 2nd */
	unsigned short data16;

	ret = io373b_subdev_read(&acpi->pdev->dev, BAT_CAPACITY_REL | cmd_flag, (unsigned char *) &data16, 2);
	if (ret < 0)
		return ret;
	data16 = le16_to_cpu(data16);
	dev_dbg_sparse(&acpi->pdev->dev, 10, "BAT_CAP_REL = %d\n", data16);

	return data16;
}

static char serial_number[4 + 1];  // +1 for null char
static char manu_name[MANU_NAME_SIZE];
static int io373b_acpi_bat_get_prop(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	unsigned short data16;
	unsigned char data8;
	int ret;
	struct io373b_acpi *acpi = dev_get_drvdata(psy->dev->parent);
	int bat_idx = (psy == &acpi->bat[0]) ? 0 : 1;
	unsigned char cmd_flag = (bat_idx == 0) ? 0 : 0x80; /* 0x80 for the 2nd */

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		ret = io373b_subdev_read(&acpi->pdev->dev, ACDC_FLAG | cmd_flag, &data8, 1);
		if (ret < 0)
			return ret;
		dev_dbg(&acpi->pdev->dev, "ACDC_FLAG = %02x\n", data8);
		val->intval = !!(data8 & BAT_IN);
		dev_dbg(&acpi->pdev->dev, "POWER_SUPPLY_PROP_PRESENT --> %d\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		ret = io373b_acpi_get_bat_status(acpi, bat_idx);
		if (ret < 0)
			return ret;
		val->intval = ret;
		dev_dbg_sparse(&acpi->pdev->dev, 10, "POWER_SUPPLY_PROP_STATUS --> %d\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		ret = io373b_subdev_read(&acpi->pdev->dev, BAT_CYCLE_COUNT | cmd_flag, (unsigned char *) &data16, 2);
		if (ret < 0)
			return ret;
		data16 = le16_to_cpu(data16);
		val->intval = data16;
		dev_dbg(&acpi->pdev->dev, "POWER_SUPPLY_PROP_CYCLE_COUNT --> %d\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = io373b_subdev_read(&acpi->pdev->dev, BAT_VOLTAGE_NOW | cmd_flag, (unsigned char *) &data16, 2);
		if (ret < 0)
			return ret;
		data16 = le16_to_cpu(data16);
		val->intval = data16 * 1000; //mA(*1000) -> uA
		dev_dbg(&acpi->pdev->dev, "POWER_SUPPLY_PROP_VOLTAGE_NOW --> %d\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = io373b_subdev_read(&acpi->pdev->dev, BAT_CURRENT_NOW | cmd_flag, (unsigned char *) &data16, 2);
		if (ret < 0)
			return ret;
		data16 = le16_to_cpu(data16);
		val->intval = ((signed short) data16) * 1000; //mA(*1000) -> uA
		dev_dbg(&acpi->pdev->dev, "POWER_SUPPLY_PROP_CURRENT_NOW --> %d\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		ret = io373b_subdev_read(&acpi->pdev->dev, BAT_CURRENT_AVG | cmd_flag, (unsigned char *) &data16, 2);
		if (ret < 0)
			return ret;
		data16 = le16_to_cpu(data16);
		val->intval = ((signed short) data16) * 1000; //mA(*1000) -> uA
		dev_dbg(&acpi->pdev->dev, "POWER_SUPPLY_PROP_CURRENT_AVG --> %d\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = io373b_acpi_get_bat_cap(acpi, bat_idx);
		if (ret < 0)
			return ret;
		val->intval = ret;
		dev_dbg_sparse(&acpi->pdev->dev, 10, "POWER_SUPPLY_PROP_CAPACITY --> %d\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = io373b_subdev_read(&acpi->pdev->dev, BAT_TEMP | cmd_flag, (unsigned char *) &data16, 2);
		if (ret < 0)
			return ret;
		data16 = le16_to_cpu(data16);
		val->intval = (data16 == 0) ? 0 : data16 - 2732; // in 0.1 Celsius
		dev_dbg(&acpi->pdev->dev, "POWER_SUPPLY_PROP_TEMP --> %d\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
		ret = io373b_subdev_read(&acpi->pdev->dev, BAT_SERIAL_NUMBER | cmd_flag, (unsigned char *) &data16, 2);
		if (ret < 0)
			return ret;
		data16 = le16_to_cpu(data16);
		sprintf(serial_number, "%04x", data16);
		val->strval = serial_number;
		dev_dbg(&acpi->pdev->dev, "POWER_SUPPLY_PROP_SERIAL_NUMBER --> %s\n", val->strval);
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		ret = io373b_subdev_read(&acpi->pdev->dev, BAT_MANU_NAME | cmd_flag, (unsigned char *) manu_name, MANU_NAME_SIZE);
		if (ret < 0)
			return ret;
		manu_name[MANU_NAME_SIZE - 1] = '\0'; // for safe, although f/w does it.
		val->strval = manu_name;
		dev_dbg(&acpi->pdev->dev, "POWER_SUPPLY_PROP_MANUFACTURER --> %s\n", val->strval);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int io373b_acpi_ac_get_prop(struct power_supply *psy,
                                   enum power_supply_property psp,
                                   union power_supply_propval *val)
{
	unsigned char data8;
	int ret;
	struct io373b_acpi *acpi = dev_get_drvdata(psy->dev->parent);
	unsigned char cmd_flag = (psy == &acpi->ac[0]) ? 0 : 0x80; /* 0x80 for the 2nd */

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = io373b_subdev_read(&acpi->pdev->dev, ACDC_FLAG | cmd_flag, &data8, 1);
		if (ret < 0)
			return ret;
		dev_dbg(&acpi->pdev->dev, "ACDC_FLAG = %02x\n", data8);
		val->intval = !!(data8 & ADAPTER_IN);
		dev_dbg(&acpi->pdev->dev, "POWER_SUPPLY_PROP_ONLINE --> %d\n", val->intval);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int io373b_acpi_notify(struct notifier_block *nb, unsigned long val, void *data)
{
/*
	struct io373b_acpi *acpi = container_of(nb, struct io373b_acpi, nb);

	if (val & IRQ_PF_CHARGER) {
		//power_supply_changed(&acpi->ac);
	}
*/
	return 0;
}

static void io373b_acpi_unregister_psy(struct io373b_acpi *acpi)
{
	int i;
	for (i = 0; i < acpi->num_bat; i++) {
		if (acpi->bat[i].name) {
			power_supply_unregister(&acpi->bat[i]);
			acpi->bat[i].name = NULL;
		}
		if (acpi->ac[i].name) {
			power_supply_unregister(&acpi->ac[i]);
			acpi->ac[i].name = NULL;
		}
	}
	acpi->num_bat = 0;
}

static void io373_acpi_delayed_work(struct work_struct *w)
{
	int bat_status, bat_cap;
	struct io373b_acpi *acpi = container_of(w, struct io373b_acpi, work.work);

	bat_status = io373b_acpi_get_bat_status(acpi, 0);
	if (bat_status < 0) {
		dev_err(&acpi->pdev->dev, "Failed to get bat status - %d\n", bat_status);
		goto reschedule;
	}

	bat_cap = io373b_acpi_get_bat_cap(acpi, 0);
	if (bat_cap < 0) {
		dev_err(&acpi->pdev->dev, "Failed to get bat cap - %d\n", bat_cap);
		goto reschedule;
	}

	if (bat_status != acpi->last_bat_status ||
	    bat_cap != acpi->last_bat_cap) {
		acpi->last_bat_status = bat_status;
		acpi->last_bat_cap = bat_cap;
		power_supply_changed(&acpi->bat[0]);
	}

reschedule:
	schedule_delayed_work(&acpi->work, HZ*2);
}

static int io373b_acpi_probe(struct platform_device *pdev)
{
	struct io373b_acpi *acpi = 0;
	int ret = 0;
	union io373b_subdev_board_info *info = dev_get_platdata(&pdev->dev);
	int i, num_bat;

	dev_dbg(&pdev->dev, "%s\n", __func__);

	if (!info)
		dev_warn(&pdev->dev, "io373b_subdev_board_info.acpi not specified.\n");

	num_bat = info ? info->acpi.num_bat : 0;
	if (num_bat <= 0 || num_bat > 2)
		dev_warn(&pdev->dev, "io373b_subdev_board_info.acpi.num_bat = %d\n", num_bat);

	acpi = kzalloc(sizeof(*acpi), GFP_KERNEL);
	if (!acpi)
		return -ENOMEM;

	acpi->pdev = pdev;
	acpi->nb.notifier_call = io373b_acpi_notify;
	acpi->num_bat = num_bat;

	acpi->last_bat_status = -1;
	acpi->last_bat_cap = -1;

	platform_set_drvdata(pdev, acpi);

	ret = io373b_acpi_create_dev_attrs(acpi);
	if (ret)
		goto fail_dev_attrs;

	for (i = 0; i < num_bat; i++) {
		acpi->bat[i].name		= info->acpi.bat_name[i];
		acpi->bat[i].type		= POWER_SUPPLY_TYPE_BATTERY;
		acpi->bat[i].get_property	= io373b_acpi_bat_get_prop;
		acpi->bat[i].properties		= io373b_acpi_bat_props;
		acpi->bat[i].num_properties	= ARRAY_SIZE(io373b_acpi_bat_props);

		ret = power_supply_register(&pdev->dev, &acpi->bat[i]);
		if (ret) {
			acpi->bat[i].name = NULL; /*invalidate*/
			dev_err(&pdev->dev, "Failed to power supply register battery - %d\n", ret);
			goto fail_power;
		}

		acpi->ac[i].name		= info->acpi.charger_name[i];
		acpi->ac[i].type		= POWER_SUPPLY_TYPE_MAINS;
		acpi->ac[i].get_property	= io373b_acpi_ac_get_prop;
		acpi->ac[i].properties		= io373b_acpi_ac_props;
		acpi->ac[i].num_properties	= ARRAY_SIZE(io373b_acpi_ac_props);

		ret = power_supply_register(&pdev->dev, &acpi->ac[i]);
		if (ret) {
			acpi->ac[i].name = NULL;
			dev_err(&pdev->dev, "Failed to power supply register ac - %d\n", ret);
			goto fail_power;
		}
	}
	ret = io373b_register_notifier(pdev->dev.parent, &acpi->nb);
	if (ret)
		goto fail_notifier;

	INIT_DELAYED_WORK_DEFERRABLE(&acpi->work, io373_acpi_delayed_work);
	if (acpi->num_bat)
		schedule_delayed_work(&acpi->work, HZ*3/2);

	return 0;

fail_notifier:
fail_power:

	io373b_acpi_unregister_psy(acpi);

	io373b_acpi_remove_dev_attrs(acpi);

fail_dev_attrs:

	kfree(acpi);

	return ret;
}

static int io373b_acpi_remove(struct platform_device *pdev)
{
	struct io373b_acpi *acpi = platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "%s\n", __func__);

	cancel_delayed_work_sync(&acpi->work);

	io373b_unregister_notifier(pdev->dev.parent, &acpi->nb);

	io373b_acpi_unregister_psy(acpi);

	io373b_acpi_remove_dev_attrs(acpi);

	kfree(acpi);

	return 0;
}

static int io373b_acpi_suspend(struct platform_device *pdev,
		pm_message_t state)
{
	struct io373b_acpi *acpi = platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "%s\n", __func__);

	cancel_delayed_work_sync(&acpi->work);

	return 0;
}

static int io373b_acpi_resume(struct platform_device *pdev)
{
	struct io373b_acpi *acpi = platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "%s\n", __func__);

	if (acpi->num_bat)
		schedule_delayed_work(&acpi->work, HZ);

	return 0;
}

static struct platform_driver io373b_acpi_driver = {
	.driver	= {
		.name	= "io373b-acpi",
		.owner	= THIS_MODULE,
	},
	.probe		= io373b_acpi_probe,
	.remove		= io373b_acpi_remove,
	.suspend	= io373b_acpi_suspend,
	.resume		= io373b_acpi_resume,
};

static int __init io373b_acpi_init(void)
{
	printk(KERN_INFO "%s\n", __func__);
	return platform_driver_register(&io373b_acpi_driver);
}
module_init(io373b_acpi_init);

static void __exit io373b_acpi_exit(void)
{
	printk(KERN_INFO "%s\n", __func__);
	platform_driver_unregister(&io373b_acpi_driver);
}
module_exit(io373b_acpi_exit);

MODULE_AUTHOR("ENE");
MODULE_DESCRIPTION("IO373X-B battery/charger (ACPI info) driver");
MODULE_LICENSE("GPL");
