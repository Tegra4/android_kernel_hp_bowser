/*
 *  ENE IO373X-B PS/2 driver
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/serio.h>
#include "io373b.h"

#define	CMD_WRITE_BYTE          0xD2
#define	CMD_READ_RSP            0xD1
#define	CMD_READ_PKT            0xD0
#define	CMD_PS2_MODE            0xE1

enum { KBD, AUX };

struct kbpkt { /* key or ps2 response */
	unsigned char smb_cnt;
	unsigned char data_cnt;
	unsigned char data[7];
};

struct response { /* response in private format */
	unsigned char cnt; /*5*/
	unsigned char n_rsp : 2;
	unsigned char resv : 4;
	unsigned char is_ps2_mode : 1;
	unsigned char is_cmd_done : 1;
	unsigned char last_cmd;
	unsigned char rsp0;
	unsigned char rsp1;
	unsigned char rsp2;
};

struct io373b_ps2 {
	struct platform_device *pdev;
	struct serio *serio;
	bool is_kbd;
	struct notifier_block nb;
};

static int io373b_ps2_notify(struct notifier_block *nb, unsigned long val, void *data)
{
	struct io373b_ps2 *ps2 = container_of(nb, struct io373b_ps2, nb);
	int ret, i;

	if (ps2->is_kbd && (val & IRQ_PF_PS2_KBD)) {
		struct kbpkt pkt;

		ret = io373b_subdev_read(&ps2->pdev->dev, CMD_READ_PKT, (unsigned char *) &pkt, sizeof(pkt));
		if (ret < 0)
			return ret;
		if (pkt.data_cnt > sizeof(pkt.data))
			return -EIO;
#if 1
		for (i = 0; i < pkt.data_cnt; i++)
			dev_dbg(&ps2->pdev->dev, "IN : %02X\n", pkt.data[i]);
#endif
		if (ps2->serio) {
			for (i = 0; i < pkt.data_cnt; i++)
				serio_interrupt(ps2->serio, pkt.data[i], 0);
		}
	} else if (!ps2->is_kbd && (val & IRQ_PF_PS2_AUX)) { /* aux not supported yet */
		return -ENXIO;
	}

	return 0;
}

static int io373b_ps2_write(struct serio *serio, unsigned char byte)
{
	struct io373b_ps2 *ps2 = serio->port_data;

	dev_dbg(&ps2->pdev->dev, "OUT: %02X\n", byte);

	return io373b_subdev_write(&ps2->pdev->dev, CMD_WRITE_BYTE, &byte, 1);
}

static int io373b_ps2_register_port(struct io373b_ps2 *ps2)
{
	struct serio *serio;

	/* NOTE: no need to free serio when unregister */
	serio = kzalloc(sizeof(struct serio), GFP_KERNEL);
	if (!serio)
		return -ENOMEM;

	serio->id.type      = ps2->is_kbd ? SERIO_8042_XL : SERIO_PS_PSTHRU;
	serio->write        = io373b_ps2_write;
	serio->dev.parent   = &ps2->pdev->dev;
	strlcpy(serio->name, dev_name(&ps2->pdev->dev), sizeof(serio->name));
	strlcpy(serio->phys, dev_name(&ps2->pdev->dev), sizeof(serio->phys));
	ps2->serio = serio;
	serio->port_data = ps2;
	serio_register_port(serio);

	return 0;
}

static int io373b_ps2_read_rsp(struct io373b_ps2 *ps2, struct response *rsp)
{
	int ret;

	rsp->cnt = 0;

	ret = io373b_subdev_read(&ps2->pdev->dev, CMD_READ_RSP, (unsigned char *) rsp, sizeof(*rsp));
	if (ret < 0)
		return ret;

	if (rsp->cnt != sizeof(*rsp) - 1) { /* data cnt */
		dev_err(&ps2->pdev->dev, "response.cnt (%d) != %d!\n", rsp->cnt, sizeof(*rsp) - 1);
		return -EIO;
	}

	return 0;
}

static int io373b_ps2_enter_ps2_mode(struct io373b_ps2 *ps2)
{
	unsigned char dummy = 0;
	struct response rsp;
	int ret;

	ret = io373b_subdev_write(&ps2->pdev->dev, CMD_PS2_MODE, &dummy, 1);
	if (ret < 0)
		return ret;

	ret = io373b_ps2_read_rsp(ps2, &rsp);
	if (ret < 0)
		return ret;

	if (!rsp.is_ps2_mode) {
		dev_err(&ps2->pdev->dev, "response.is_ps2_mode is false!\n");
		return -EIO;
	}

	return 0;
}

static int io373b_ps2_probe(struct platform_device *pdev)
{
	struct io373b_ps2 *ps2 = 0;
	bool notifier_registered = false, serio_registered = false;
	int err = 0;

	dev_dbg(&pdev->dev, "%s\n", __func__);

	if (!(ps2 = kzalloc(sizeof(struct io373b_ps2), GFP_KERNEL)))
		return -ENOMEM;

	platform_set_drvdata(pdev, ps2);
	ps2->pdev = pdev;
	ps2->is_kbd = (platform_get_device_id(pdev)->driver_data == KBD);
	ps2->nb.notifier_call = io373b_ps2_notify;

	if ((err = io373b_ps2_enter_ps2_mode(ps2)))
		goto error_exit;

	if ((err = io373b_register_notifier(pdev->dev.parent, &ps2->nb)))
		goto error_exit;
	else
		notifier_registered = true;

	if ((err = io373b_ps2_register_port(ps2)))
		goto error_exit;
	else
		serio_registered = true;

	return 0;

error_exit:

	if (serio_registered)
		serio_unregister_port(ps2->serio);
	if (notifier_registered)
		io373b_unregister_notifier(pdev->dev.parent, &ps2->nb);
	kfree(ps2);

	return err;
}

static int io373b_ps2_remove(struct platform_device *pdev)
{
	struct io373b_ps2 *ps2 = platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "%s\n", __func__);

	serio_unregister_port(ps2->serio);
	io373b_unregister_notifier(ps2->pdev->dev.parent, &ps2->nb);
	kfree(ps2);

	return 0;
}

static const struct platform_device_id io373b_ps2_id_table[] = {
	/* name           , driver_data */
	{ "io373b-ps2-kbd", KBD },
	{ "io373b-ps2-aux", AUX },
	{},
};
MODULE_DEVICE_TABLE(platform, io373b_ps2_id_table);

static struct platform_driver io373b_ps2_driver = {
	.driver = {
		.name  = "io373b-ps2",
		.owner = THIS_MODULE,
	},
	.probe    = io373b_ps2_probe,
	.remove   = io373b_ps2_remove,
	.id_table = io373b_ps2_id_table,
};

static int __init io373b_ps2_init(void)
{
	printk(KERN_INFO "%s\n", __func__);

	return platform_driver_register(&io373b_ps2_driver);
}

static void __exit io373b_ps2_exit(void)
{
	printk(KERN_INFO "%s\n", __func__);

	platform_driver_unregister(&io373b_ps2_driver);
}

module_init(io373b_ps2_init);
module_exit(io373b_ps2_exit);
MODULE_AUTHOR("flychen");
MODULE_DESCRIPTION("IO373X-B PS/2 driver");
MODULE_LICENSE("GPL");
