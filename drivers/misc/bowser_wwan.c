#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/delay.h>


#define TEGRA_GPIO_PP0		120
#define TEGRA_GPIO_PP2		122
#define WWAN_MU736_RESET	TEGRA_GPIO_PP0
#define WWAN_MU736_POWER_ON	TEGRA_GPIO_PP2

static struct regulator *vdd_modem_3v3;

static int bowser_wwan_probe(struct platform_device *dev)
{
	int ret = 0;

	/* Get resource */
	vdd_modem_3v3 = regulator_get(NULL, "vdd_modem_3v3");
	if (IS_ERR_OR_NULL(vdd_modem_3v3)) {
		printk(KERN_ERR "Fail to get regulator vdd_modem_3v3\n");
		ret = PTR_ERR(vdd_modem_3v3);
		vdd_modem_3v3 = NULL;
		return ret;
	}

	ret = gpio_request(WWAN_MU736_POWER_ON, "modem_power_on");
	if (ret) {
		printk(KERN_ERR "Fail to request gpio TEGRA_GPIO_PP2\n");
		return ret;
	}

	ret = gpio_request(WWAN_MU736_RESET, "modem_reset");
	if (ret) {
		printk(KERN_ERR "Fail to request gpio TEGRA_GPIO_PP0\n");
		return ret;
	}

	/* Power on sequence */
	regulator_enable(vdd_modem_3v3);
	msleep(50);
	gpio_direction_output(WWAN_MU736_RESET, 1);
	msleep(5);
	gpio_direction_output(WWAN_MU736_POWER_ON, 1);

	return ret;
}

static int bowser_wwan_shutdown(struct platform_device *dev){
	/* Power off sequence */
	gpio_direction_output(WWAN_MU736_POWER_ON, 0);
	msleep(510);
	gpio_direction_output(WWAN_MU736_RESET, 0);

	return 0;
}

static struct platform_driver bowser_wwan_platform_driver = {
	.probe		= bowser_wwan_probe,
	.shutdown	= bowser_wwan_shutdown,
	.driver = {
		.name = "bowser_wwan",
	}
};

static void __init bowser_wwan_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&bowser_wwan_platform_driver);
	if (ret)
		printk(KERN_ERR "Fail to register bowser wwan platform driver\n");

	return ret;
}

static void __exit bowser_wwan_exit(void)
{
	platform_driver_unregister(&bowser_wwan_platform_driver);
}


module_init(bowser_wwan_init);
module_exit(bowser_wwan_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Chris Cheng <chris.cheng@quantatw.com>");
MODULE_DESCRIPTION("Bowser WWAN driver");
