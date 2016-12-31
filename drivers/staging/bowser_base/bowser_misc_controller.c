/*
 * Copyright (C) 2012 Insyde Software Corp.
 *
 *	work with io373b
 *
 * GPIO mapping:
 *	GPIO Name	PIN Name in schematic	Status
 *	GPIO3_PJ.03	SOC_DOCK_DET#		1 = undock, 0 = docked
 *	GPIO3_PS.00	HALL_INT* (lid switch)	1 = opened, 0 = closed
 *	GPIO3_PT.05	GEN2_I2C_SCL_1V8
 *	GPIO3_PT.06	GEN2_I2C_SDA_1V8
 *	GPIO3_PC.07	EC_INT_SW#		Low-active.
 */

/* mark following line to disable dev_dbg */
#define DEBUG

#define BOWSER_MISC_TOUCHPAD
// #define INSTALL_FW_IF_NOT_MATCH


#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/major.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/firmware.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/bowser_misc.h>

#include "bowser_misc_controller.h"
#include "io373x-b/io373b.h"
#include <linux/ektf3k.h>

/* TP */
#ifdef BOWSER_MISC_TOUCHPAD
#include <linux/i2c/i2c-hid.h>
#endif

/* I2C
 * @BOWSER_EC_I2C_BUS			GEN2_I2C bus number in dalmore
 * @BOWSER_EC_I2C_BOARD_TYPE		driver type of EC i2c device
 * @BOWSER_EC_I2C_BOARD_ADDR		EC i2c address with firmware installed
 * @BOWSER_EC_NO_FW_I2C_BOARD_ADDR	EC i2c address without firmware installed
 */
#define BOWSER_EC_I2C_BUS		1
#define BOWSER_EC_I2C_BOARD_TYPE	"io373b-smbd-base"
#define BOWSER_EC_I2C_BOARD_ADDR	0x62
#define BOWSER_EC_NO_FW_I2C_BOARD_ADDR	0x60

/* ec firmware
 * @EC_FW_VERSION_LEN			the length of EC firmware version
 * @EC_FW_VERSION_OFFSET		the offset of EC firmware version
 * @EC_FW_NAME				the name of EC firmware
 */
#define EC_FW_VERSION_LEN		6
#define EC_FW_VERSION_OFFSET		0x500
#define EC_FW_NAME			"bowser_ec.bin"

/* GPIO */
#define TEGRA_GPIO_PC7			23		/* GPIO3_PC.07 */
//#define TEGRA_GPIO_PI5			69		/* GMI_IORDY */
//#define TEGRA_GPIO_PJ3			75		/* GPIO3_PJ.03 */
#define TEGRA_GPIO_PO5			117		/* GPIO3_PO.05 */
//#define TEGRA_GPIO_PO6			118		/* GPIO3_PO.06 */
#define TEGRA_GPIO_PO7			119		/* ULPI_DATA6 */
//#define TEGRA_GPIO_PQ5			133		/* KB_COL5 */
#define TEGRA_GPIO_PS0			144		/* GPIO3_PS.00 */
#define TEGRA_GPIO_PV0			168		/* GPIO_PV0 */
/*
 * @BOWSER_PV_ID_GPIO		[Input] High = SI build, Low = PV build
 * @BOWSER_PV_SOC_I2C_DOCK_EN_GPIO [Output] Dock enable pin in PV build
 * @BOWSER_PV_SOC_DOCK_DET_GPIO	[Input] Dock Detection pin in PV build
 * @BOWSER_SOC_DOCK_DET_GPIO	[Input] SOC_DOCK_DET# pin,
					 High = undock, low = docked
 * @BOWSER_HALL_INT_GPIO		[Input] LID Switch pin,
					 high = opened, low = closed
 * @BOWSER_EC_INT_SW_GPIO		[Input] EC_INT_SW# pin, low-active,
 * @BOWSER_EC_RST_GPIO		[OUTPUT] EC reset/write protect pin
 *					 high active
 */
#define BOWSER_PV_ID_GPIO			TEGRA_GPIO_PO7
#define BOWSER_HALL_INT_GPIO		TEGRA_GPIO_PS0
#define BOWSER_EC_INT_SW_GPIO		TEGRA_GPIO_PC7
#define BOWSER_EC_RST_GPIO		TEGRA_GPIO_PV0
#define BOWSER_LID_SW_IRQ_NAME		"lid_switch"
#define BOWSER_KB_IRQ_GPIO_NAME		"kb_irq"
#define BOWSER_EC_RST_NAME		"EC_reset"

/* TP */
#ifdef BOWSER_MISC_TOUCHPAD
#define BOWSER_TP_IRQ_GPIO		TEGRA_GPIO_PO5
#define BOWSER_TP_IRQ_GPIO_NAME		"tp_irq"
#define BOWSER_TM2735_ADDR		0x2c
/* using LID_Switch pin for test
 *#define SYNAPTICS_RESET_GPIO		TEGRA_GPIO_PS0
 */
#endif	/* BOWSER_MISC_TOUCHPAD */

/* char device */
#define BOWSER_MISC_FIRST_MINOR		0
#define BOWSER_MISC_MAX_DEVICES		1
#define BOWSER_MISC_DEV_NAME		"bowser_misc"

/* ioctl
 * BOWSERBASE_R_READECVERSION	read EC version
 * BOWSERBASE_W_SETMUTELED	set mute LED
 * BOWSERBASE_W_SETSYSTEMSTATE	set system state
 * BOWSERBASE_W_UPDATEECFW	update firmware
 * BOWSERBASE_W_SETSCREENSTATE	set screen state
 * BOWSERBASE_R_READCONNSTATE	read base connection state
 * BOWSERBASE_R_READFWVERSION	read firmware version
 * BOWSERBASE_W_SETLIDSTATE	set lid open/close
 * BOWSERBASE_W_FWUPDATENOTICE	send notice for ec app
 * BOWSERBASE_W_BLINKMUTELED	set Mute LED blinking
 * BOWSERBASE_W_BLINKCAPSLOCKLED	set CapsLock LED blinking
 * BOWSERBASE_W_FWUPDATEFINISHED	read EC update flag
 */
#define BOWSER_IOC_MAGIC				'M'
#define BOWSERBASE_R_READECVERSION	_IOR(BOWSER_IOC_MAGIC, 1, int)
#define BOWSERBASE_W_SETMUTELED		_IOW(BOWSER_IOC_MAGIC, 3, int)
#define BOWSERBASE_W_SETSYSTEMSTATE	_IOW(BOWSER_IOC_MAGIC, 4, int)
#define BOWSERBASE_W_UPDATEECFW		_IOW(BOWSER_IOC_MAGIC, 5, int)
#define BOWSERBASE_W_SETSCREENSTATE	_IOW(BOWSER_IOC_MAGIC, 6, int)
#define BOWSERBASE_R_READCONNSTATE	_IOR(BOWSER_IOC_MAGIC, 8, int)
#define BOWSERBASE_R_READFWVERSION	_IOR(BOWSER_IOC_MAGIC, 9, int)
#define BOWSERBASE_W_SETLIDSTATE		_IOW(BOWSER_IOC_MAGIC, 11, int)
#define BOWSERBASE_W_FWUPDATENOTICE	_IOW(BOWSER_IOC_MAGIC, 13, int)
#define BOWSERBASE_W_FWUPDATENOTICEEN	_IOW(BOWSER_IOC_MAGIC, 14, int)
#define BOWSERBASE_W_BLINKMUTELED		_IOW(BOWSER_IOC_MAGIC, 15, int)
#define BOWSERBASE_W_BLINKCAPSLOCKLED	_IOW(BOWSER_IOC_MAGIC, 16, int)
#define BOWSERBASE_W_FWUPDATEFINISHED	_IOW(BOWSER_IOC_MAGIC, 17, int)
#define BOWSERBASE_W_SETKBMATRIXTYPE	_IOW(BOWSER_IOC_MAGIC, 18, int)
#define BOWSERBASE_W_SETMESSAGELED	_IOW(BOWSER_IOC_MAGIC, 19, int)

/* read/write */
#define BUF_LEN			100		/* length of Message */
#define EC_UPDATE_NOT_FINISHED_STRING	"0"
#define EC_UPDATE_FINISHED_STRING	"1"

#define MISC_BUSY_BIT_POS	1
#define MB_EC_VER_NOT_FOUND_VERSION	"000000"
#define MB_EC_VER_NOT_READ_VERSION	"000001"
#define MB_EC_VER_INVALID_VERSION	"000002"
#define MB_EC_VER_ERR_VERSION		"000003"


// misc info
enum mute_state {
	MB_MUTE = 1,
	MB_UNMUTE,
};

enum lid_switch {
	MB_LID_OPENED = 0,	/* LID open */
	MB_LID_CLOSED,		/* LID close */
};

enum system_state {
	MB_SYS_RUNNING = 1,
	MB_SYS_SHUT_DOWN,
	MB_SYS_IDLE,
	MB_SYS_SUSPEND,
	MB_SYS_DEEP_SLEEP,
};

enum screen_state {
	MB_SCREEN_ON = 1,
	MB_SCREEN_OFF,
};

/* enum ec_version_valid
 * @MB_EC_VER_VALID:	EC version is valid. (addr=0x62)
 * @MB_EC_VER_INVALID:	EC version is invalid. (using MaskROM, addr=0x62)
 * @MB_EC_VER_NOT_FOUND:EC version can not be read (no firmware) (addr=0x60)
 * @MB_EC_VER_NOT_READ	EC version is not read.
 */
enum ec_version_valid {
	MB_EC_VER_VALID = 1,
	MB_EC_VER_INVALID,
	MB_EC_VER_NOT_FOUND,
	MB_EC_VER_NOT_READ,
};

enum fw_file_state {
	MB_FW_READY = 1,	/* fw is read and valid. */
	MB_FW_INVALID,		/* fw is read and invalid. */
	MB_FW_NOT_READ,		/* fw is not read. */
};

enum ec_driver_status {
	LOADED = 0,
	UNLOADED,
};

struct bowser_misc_dev {
	struct platform_device *pdev;
	unsigned int lid_irq;
	unsigned int kb_irq;
	struct work_struct work;
	struct mutex lock;
	struct mutex lock_lid;
	struct mutex lock_sync;
	struct input_dev *input_dev;
	unsigned long flags;
	long mute_led_blinking_freq;
	long caps_lock_led_blinking_freq;

	struct device *io373_acpi_dev;

	int update_success;     /* 1 = success, 0 = failure */
	int do_update_fw;	/* 1 = update FW , 0 = don't update FW */
	int lid_change;		/* 1 = changed, 0 = no changed. */
	enum mute_state	mute;
	enum lid_switch	lid_sw;
	enum system_state system_state;
	enum screen_state screen_state;
	enum ec_driver_status ec_driver_status;
/* char device */
	int major;
	dev_t dev_id;
	struct cdev cdev;	/* associated character device */
	struct class *class;
	struct device *char_dev;
/* i2c */
	struct i2c_adapter *adapter;
	struct i2c_client *ec_i2c_client;
/* firmware */
	struct {
		char *name;	/* the name of EC firmware file */
		const struct firmware *fw;
		char version[EC_FW_VERSION_LEN + 1];
		enum fw_file_state state;
        } fw;
	char ec_fw_version[EC_FW_VERSION_LEN + 1];
	enum ec_version_valid ec_version_valid;
	int need_notify_os;
	int message_led_status;
	unsigned char charge_state;
	bool do_ec_ram_dump;
#ifdef BOWSER_MISC_TOUCHPAD
	unsigned int tp_irq;
	bool tp_loaded;
	struct i2c_client *touchpad_i2c_client;
#endif	/* BOWSER_MISC_TOUCHPAD */
};

static char Message[BUF_LEN];	/* message from/to user space */

//* function */
static int load_ec_driver(struct bowser_misc_dev *mb_dev, bool on);
static int read_ec_ver(struct bowser_misc_dev *mb_dev);
static unsigned char kb_matrix_type;
struct bowser_misc_dev *g_mb_dev;

static struct i2c_board_info io373b_virtual_i2c_devs[] = {
	/*               name              i2c-addr(7 bits) */
	{ I2C_BOARD_INFO("io373b-ps2-kbd", 0x01), },
	{ I2C_BOARD_INFO("io373b-acpi", 0x07), },
	{ },
};

extern int ac_status;

#ifdef BOWSER_MISC_TOUCHPAD
static struct i2c_hid_platform_data tm2735_platformdata = {
	.hid_descriptor_address = 0x20,
};
#endif	/* BOWSER_MISC_TOUCHPAD */

static void kbc_irq_free(struct bowser_misc_dev *mb_dev)
{
	gpio_free(BOWSER_EC_INT_SW_GPIO);
#ifdef BOWSER_MISC_TOUCHPAD
	gpio_free(BOWSER_TP_IRQ_GPIO);
#endif
}

static int bowser_irq_request(struct bowser_misc_dev *mb_dev)
{
	int ret = 0;

	/* get irq for EC(io373b) */
	ret = gpio_request(BOWSER_EC_INT_SW_GPIO, BOWSER_KB_IRQ_GPIO_NAME);
	if (ret < 0) {
		dev_err(&mb_dev->pdev->dev,
		 "%s: %s gpio_request failed (err=%d)\n", __func__,
					 BOWSER_KB_IRQ_GPIO_NAME, ret);
		return ret;
	}

	ret = gpio_direction_input(BOWSER_EC_INT_SW_GPIO);
	if (ret < 0) {
		dev_err(&mb_dev->pdev->dev,
		 "%s: KB_IRQ gpio_direction_output failed (err=%d)\n",
							 __func__, ret);
		gpio_free(BOWSER_EC_INT_SW_GPIO);
		return ret;
	}

	ret = mb_dev->kb_irq = gpio_to_irq(BOWSER_EC_INT_SW_GPIO);
	if (ret < 0) {
		dev_err(&mb_dev->pdev->dev,
		 "%s: gpio_to_irq() failed (err=%d)\n", __func__, ret);
		return ret;
	}

#ifdef BOWSER_MISC_TOUCHPAD
	/* get irq for TP */
	ret = gpio_request(BOWSER_TP_IRQ_GPIO,
					 BOWSER_TP_IRQ_GPIO_NAME);
	if (ret < 0) {
		dev_err(&mb_dev->pdev->dev,
		 "%s: %s gpio_request failed (err=%d)\n", __func__,
					 BOWSER_TP_IRQ_GPIO_NAME, ret);
		return ret;
	}

	ret = gpio_direction_input(BOWSER_TP_IRQ_GPIO);
	if (ret < 0) {
		dev_err(&mb_dev->pdev->dev,
		 "%s: TP IRQ gpio_direction_output failed (err=%d)\n",
							 __func__, ret);
		gpio_free(BOWSER_TP_IRQ_GPIO);
		return ret;
	}

	ret = mb_dev->tp_irq = gpio_to_irq(BOWSER_TP_IRQ_GPIO);
	if (ret < 0) {
		dev_err(&mb_dev->pdev->dev,
		 "%s: gpio_to_irq() TP_IRQ failed (err=%d)\n", __func__, ret);
		return ret;
	}
#endif
	return ret;
}

static void _dump_ec_ram(struct bowser_misc_dev *mb_dev, unsigned short addr, int len)
{
	int i;
	unsigned char data = 0x00;
	unsigned short start_reg = addr;

	printk("=================== 0x%X ===================\n", start_reg);
	for (i = 1; i <= len; i++) {
		io373b_read_regs(&mb_dev->ec_i2c_client->dev, start_reg, &data, sizeof(data));
		printk("%02x ", data);
		if ((i % 0x10) == 0)
			printk("\n");
		if ((i % 0x100) == 0)
			printk("==================== 0x%X ==================\n", start_reg + 1);
		start_reg++;
		data = 0x00;
	}
}

static void dump_ec_ram(struct bowser_misc_dev *mb_dev)
{
	msleep(3000);
	printk("#################### XRAM ####################\n");
        _dump_ec_ram(mb_dev, 0x8000, 0x800);
        printk("\n\n\n\n\n");
        printk("#################### GPIO ####################\n");
        _dump_ec_ram(mb_dev, 0xFC00, 0x80);
        printk("\n\n\n\n\n");
        printk("#################### IKB  ####################\n");
	_dump_ec_ram(mb_dev, 0xFCA0, 0x10);
}

void bowser_ec_ram_dump(void)
{
	g_mb_dev->do_ec_ram_dump = true;
	schedule_work(&g_mb_dev->work);
}
EXPORT_SYMBOL(bowser_ec_ram_dump);

static int install_ec_fw(struct bowser_misc_dev *mb_dev,
				 const struct firmware *fw)
{
	int ret = 0;
	int num_of_retry = 0, ec_driver_ret = 0;
	struct i2c_board_info ec_i2c_board_info =
			 {.type = BOWSER_EC_I2C_BOARD_TYPE,};

	mb_dev->update_success = 0;

	dev_dbg(&mb_dev->pdev->dev, "%s start !!\n", __func__);

	if (!fw) {
		dev_err(&mb_dev->pdev->dev,
		 "%s (err) no firmware , action aborted!!\n", __func__);
		return -ENOENT;
	}

	/* To reduce occurance of bugs,
	 * we use a pure i2c device without platform data. */
	ret = load_ec_driver(mb_dev, false);
	if (ret < 0) {
		dev_err(&mb_dev->pdev->dev,
			 "%s: failed, cannot unload EC driver\n", __func__);
		/* continue anyway since EC may not have firmware */
	}

	msleep(10);

update:
	/* Disable EC write protection */
	gpio_set_value(BOWSER_EC_RST_GPIO, 1);
	msleep(1200);
	gpio_set_value(BOWSER_EC_RST_GPIO, 0);
	msleep(1000);	/* wait a while for EC reset */

	/* After EC reset, we read EC version to confirm its address*/
	ec_i2c_board_info.addr = BOWSER_EC_I2C_BOARD_ADDR;
	ec_i2c_board_info.irq  = mb_dev->kb_irq;

	mb_dev->ec_i2c_client =
			 i2c_new_device(mb_dev->adapter, &ec_i2c_board_info);
	if (!mb_dev->ec_i2c_client) {
		dev_err(&mb_dev->pdev->dev,
		 "can't create new device(%s)\n", ec_i2c_board_info.type);
		ret = -EBUSY;
		goto retry;
	}

	read_ec_ver(mb_dev);

	if (mb_dev->ec_i2c_client)
		i2c_unregister_device(mb_dev->ec_i2c_client);


	/* EC without FW(version not found): addr=0x60, EC with FW: addr=0x62 */
	if (mb_dev->ec_version_valid == MB_EC_VER_NOT_FOUND) {
		dev_dbg(&mb_dev->pdev->dev,
		 "%s: EC version not found, using addr=0x60\n", __func__);
		ec_i2c_board_info.addr = BOWSER_EC_NO_FW_I2C_BOARD_ADDR;
	} else {
		ec_i2c_board_info.irq  = mb_dev->kb_irq;
		ec_i2c_board_info.addr = BOWSER_EC_I2C_BOARD_ADDR;
	}

	mb_dev->ec_i2c_client =
			 i2c_new_device(mb_dev->adapter, &ec_i2c_board_info);
	if (!mb_dev->ec_i2c_client) {
		dev_err(&mb_dev->pdev->dev,
		 "can't create new device(%s)\n", ec_i2c_board_info.type);
		ret = -EBUSY;
		goto exit;
	}
	dev_dbg(&mb_dev->pdev->dev,
				 "%s i2c_new_device() finished.\n", __func__);

	ret = io373b_update_flash(&mb_dev->ec_i2c_client->dev,
							 fw->data, fw->size);
	if (ret < 0) {
		dev_err(&mb_dev->pdev->dev,
		 "%s failed to install EC firmware!!(err=%d)\n", __func__, ret);
		/* keep going */
		ec_driver_ret = -1;
	}

	if (mb_dev->ec_i2c_client)
		i2c_unregister_device(mb_dev->ec_i2c_client);

	if (ec_driver_ret == -1 && num_of_retry < 3)
		goto retry;

	msleep(600);	// wait a while for EC reset.
	/* Before doing load_ec_driver(), we have to make sure EC
	 * firmware is ready after firmware update. This prevent infinite loop
	 * since load_ec_driver() will execute install_ec_fw()
	 * again if it failed to do read_ec_ver().
	 */
	ec_i2c_board_info.addr = BOWSER_EC_I2C_BOARD_ADDR;
	ec_i2c_board_info.irq  = mb_dev->kb_irq;

	mb_dev->ec_i2c_client =
			 i2c_new_device(mb_dev->adapter, &ec_i2c_board_info);
	if (!mb_dev->ec_i2c_client) {
		dev_err(&mb_dev->pdev->dev,
		 "can't create new device(%s)\n", ec_i2c_board_info.type);
		ret = -EBUSY;
		goto retry;
	}

	ret = read_ec_ver(mb_dev);
	if (ret < 0) {
		dev_err(&mb_dev->pdev->dev,
		 "%s: read_ec_ver() failed! err=%d, num_of_retry=%d\n"
					, __func__, ret, num_of_retry);
		if (mb_dev->ec_i2c_client)
			i2c_unregister_device(mb_dev->ec_i2c_client);
		goto retry;
	}

	if (mb_dev->ec_i2c_client)
		i2c_unregister_device(mb_dev->ec_i2c_client);

	ret = load_ec_driver(mb_dev, true);
	if (ret < 0) {
		dev_err(&mb_dev->pdev->dev,
			 "%s: failed, cannot load EC driver\n", __func__);
		goto exit;
	}

	mb_dev->update_success = 1;
	dev_dbg(&mb_dev->pdev->dev, "%s finished !!\n", __func__);

	return ret;
retry:
	if (num_of_retry < 3) { // retry if failed.
		dev_err(&mb_dev->pdev->dev,
			"%s: try to update EC again. (num_of_retry = %d)\n",
						 __func__, num_of_retry);
		ec_driver_ret = 0;
		num_of_retry++;
		msleep(200);
		goto update;
	}
exit:
	dev_err(&mb_dev->pdev->dev,
		 "%s failed! err=%d\n", __func__, ret);
	return ret;
}

static void bowser_misc_release_firmware(const struct firmware *fw)
{
	if (fw) {
		release_firmware(fw);
		memset(&fw, 0, sizeof(fw));
	}
}

/* This function will be executed after request_firmware_nowait() is finished */
static void ec_firmware_cont(const struct firmware *fw, void *context)
{
	struct bowser_misc_dev *mb_dev = context;
	int i;

	dev_dbg(&mb_dev->pdev->dev, "%s start !!\n", __func__);

	if (fw) {
		mb_dev->fw.fw = fw;
	} else {
		dev_err(&mb_dev->pdev->dev,
					 "%s firmware not found!!\n", __func__);
		mb_dev->fw.version[EC_FW_VERSION_LEN] = '\0';
		mb_dev->fw.state = MB_FW_NOT_READ;
		return;
	}

	for (i = 0; i < EC_FW_VERSION_LEN; i++) {
		mb_dev->fw.version[i] =
			 *(mb_dev->fw.fw->data + EC_FW_VERSION_OFFSET + i);
	}
	mb_dev->fw.version[EC_FW_VERSION_LEN] = '\0';
	mb_dev->fw.state = MB_FW_READY;

	dev_dbg(&mb_dev->pdev->dev,
	 "%s EC firmware file version = %s\n", __func__, mb_dev->fw.version);
}

/* get EC firmware file and its version*/
static int get_ec_fw_file_version(struct bowser_misc_dev *mb_dev)
{
	int err = 0;
	char *fw_name = NULL;

	dev_dbg(&mb_dev->pdev->dev, "%s start !!\n", __func__);

	/* using request_firmware_nowait()
	 * (asynchronous version of request_firmware()),
	 * ec_firmware_cont() will be called asynchronously
	 *  when firmware request is over.
	 */
	err = request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
		 mb_dev->fw.name, &mb_dev->pdev->dev, GFP_KERNEL, mb_dev,
							 ec_firmware_cont);
	if (err) {
		dev_err(&mb_dev->pdev->dev,
		 "%s Failed to load %s err = %d\n", __func__, fw_name, err);
		return err;
	}

	return err;
}

#ifdef BOWSER_MISC_TOUCHPAD
/* Only for LID */
static int load_tp_driver(struct bowser_misc_dev *mb_dev, bool on)
{
	struct i2c_board_info tp_i2c_board_info = {
			I2C_BOARD_INFO("hid", BOWSER_TM2735_ADDR),
			.platform_data = &tm2735_platformdata,
			.irq = mb_dev->tp_irq,
	};

	if (on) {
		dev_err(&mb_dev->pdev->dev, "%s() load tp driver\n", __func__);

		if (mb_dev->tp_loaded == true) {
			dev_err(&mb_dev->pdev->dev, "WARN: TP driver was loaded.\n");
			return -1;
		}

		/* new TP device */
		if(!mb_dev->touchpad_i2c_client){
			mb_dev->touchpad_i2c_client =
				i2c_new_device(mb_dev->adapter, &tp_i2c_board_info);
			if (!mb_dev->touchpad_i2c_client) {
				dev_err(&mb_dev->pdev->dev,
				"can't create new device(%s)\n", tp_i2c_board_info.type);
				return -1;
			}
		}
		mb_dev->tp_loaded = true;
	} else {
		dev_err(&mb_dev->pdev->dev, "%s() unload tp driver\n", __func__);

		if (mb_dev->tp_loaded == false) {
			dev_err(&mb_dev->pdev->dev, "WARN: TP driver was unloaded.\n");
			return 0;
		}

		if (mb_dev->touchpad_i2c_client) {
			disable_irq_nosync(mb_dev->tp_irq);
			i2c_unregister_device(mb_dev->touchpad_i2c_client);
			mb_dev->touchpad_i2c_client = NULL;
			enable_irq(mb_dev->tp_irq);
		} else {
			dev_err(&mb_dev->pdev->dev,
			"can't remove TP driver(%s), no TP client\n", tp_i2c_board_info.type);
		}

		mb_dev->tp_loaded = false;
	}
	return 0;
}
#endif	// BOWSER_MISC_TOUCHPAD

static void sync_ec_status_task(struct work_struct *w)
{
	struct bowser_misc_dev *mb_dev = container_of(w,
				 struct bowser_misc_dev, work);
	unsigned char event1 = 0x00;
	unsigned char state;

	mutex_lock(&mb_dev->lock_sync);
	dev_dbg(&mb_dev->pdev->dev, "%s start !!\n", __func__);

	if (mb_dev->ec_version_valid != MB_EC_VER_VALID) {
		dev_err(&mb_dev->pdev->dev,
		 "%s failed to sync status(invalid EC version)\n", __func__);
		goto out;
	} else if (!mb_dev->ec_i2c_client) {
		dev_err(&mb_dev->pdev->dev,
		 "%s failed to sync status(no i2c device)\n", __func__);
		goto out;
	} else if (!mb_dev->io373_acpi_dev) {
		dev_err(&mb_dev->pdev->dev,
		 "%s failed to sync status(no io373 acpi device)\n", __func__);
		goto out;
	} else if (mb_dev->ec_driver_status == UNLOADED) {
		dev_err(&mb_dev->pdev->dev,
		 "%s failed to sync status(Failed to load EC driver)\n", __func__);
		goto out;
	}

	/* set screen state */
	if (mb_dev->screen_state == MB_SCREEN_ON) {
		event1 = 0x01;
	} else {
		event1 = 0x00;
	}
	io373b_subdev_write(mb_dev->io373_acpi_dev,
		 EC_REG_SCREEN_STATE, &event1, sizeof(event1));
	event1 = 0x00;

	/* Mute led blinking */
	if (mb_dev->mute_led_blinking_freq > 0x00 &&
			mb_dev->mute_led_blinking_freq <= 0xff) {
		event1 = mb_dev->mute_led_blinking_freq;
		io373b_subdev_write(mb_dev->io373_acpi_dev,
			EC_REG_MUTE_LED_BLINKING, &event1, sizeof(char));
		mb_dev->mute_led_blinking_freq = 0x00;
		event1 = 0x00;
	}

	/* CapsLock led blinking */
	if (mb_dev->caps_lock_led_blinking_freq > 0x00 &&
			mb_dev->caps_lock_led_blinking_freq <= 0xff) {
		event1 = mb_dev->caps_lock_led_blinking_freq;
		io373b_subdev_write(mb_dev->io373_acpi_dev,
			EC_REG_CAPSLOCK_LED_BLINKING, &event1, sizeof(char));
		mb_dev->caps_lock_led_blinking_freq = 0x00;
		event1 = 0x00;
	}

	io373b_subdev_read(mb_dev->io373_acpi_dev,
		 EC_REG_ECRAM_EVENT1, &event1, sizeof(event1));

	/* lid switch */
	if (mb_dev->lid_sw == MB_LID_OPENED) {
		event1 = event1 & ~BOWSER_MISC_LID_CLOSE;
	} else if (mb_dev->lid_sw == MB_LID_CLOSED) {
		event1 = event1 | BOWSER_MISC_LID_CLOSE;
	}

	/* mute led */
	if (mb_dev->mute == MB_MUTE) {
		event1 = event1 | BOWSER_MISC_MUTE_LED_ON;
	} else if (mb_dev->mute == MB_UNMUTE) {
		event1 = event1 & ~BOWSER_MISC_MUTE_LED_ON;
	}

	if (ac_status)
		event1 = event1 | BOWSER_MISC_S3_USB_VBUS;
	else
		event1 = event1 & ~BOWSER_MISC_S3_USB_VBUS;

	dev_dbg(&mb_dev->pdev->dev, "%s runs. set ecram event1=%u\n", __func__, event1);
	io373b_subdev_write(mb_dev->io373_acpi_dev,
		 EC_REG_ECRAM_EVENT1, &event1, sizeof(event1));

	/* Store Keyboard Matrix type */
	event1 = kb_matrix_type;
	io373b_subdev_write(mb_dev->io373_acpi_dev,
		EC_REG_KB_MATRIX_TYPE, &event1, sizeof(event1));
	dev_dbg(&mb_dev->pdev->dev, "%s runs. kb_matrix_type=%d\n", __func__, (int)kb_matrix_type);

	/* Enable EC write protection */
	event1 = BOWSER_BASE_WRITE_LOCK;
	io373b_subdev_write(mb_dev->io373_acpi_dev,
		 EC_REG_WRITE_PROTECT, &event1, sizeof(event1));

	/* Set EC message Led status */
	event1 = mb_dev->message_led_status;
	io373b_subdev_write(mb_dev->io373_acpi_dev,
		 EC_REG_MESSAGE_LED, &event1, sizeof(event1));

	/* Set battery charge/discharge */
	io373b_subdev_write(mb_dev->io373_acpi_dev,
		 EC_REG_FORCE_DISCHARGE ,&mb_dev->charge_state, sizeof(mb_dev->charge_state));

	/* system state */
	if (mb_dev->system_state == MB_SYS_RUNNING ||
		 mb_dev->system_state == MB_SYS_IDLE) {
		state = BOWSER_EC_S0;
	} else if (mb_dev->system_state == MB_SYS_SUSPEND) {
		state = BOWSER_EC_S3;
	} else if (mb_dev->system_state == MB_SYS_DEEP_SLEEP) {
		state = BOWSER_EC_S4;
	} else if (mb_dev->system_state == MB_SYS_SHUT_DOWN) {
		state = BOWSER_EC_S5;
	}
	io373b_subdev_write(mb_dev->io373_acpi_dev,
			 EC_REG_SYSTEM_STATE, &state, sizeof(state));

out:
	mutex_unlock(&mb_dev->lock_sync);

	if (mb_dev->do_update_fw) {
		mb_dev->do_update_fw = 0;
		install_ec_fw(mb_dev, mb_dev->fw.fw);
	}

	if (mb_dev->do_ec_ram_dump) {
		mb_dev->do_ec_ram_dump = false;
		dump_ec_ram(mb_dev);
	}

	return;
}

/* read EC version */
static int read_ec_ver(struct bowser_misc_dev *mb_dev)
{
	int err = 0;
	char ec_ver[EC_FW_VERSION_LEN];
	unsigned char mask_rom_check;

	dev_dbg(&mb_dev->pdev->dev, "%s start !!\n", __func__);


	if (!mb_dev->ec_i2c_client) {
		dev_err(&mb_dev->pdev->dev, "%s() no i2c device.\n", __func__);
		return -ENODEV;
	}

	/* check EC MaskROM */
	err = io373b_read_regs(&mb_dev->ec_i2c_client->dev,
		 EC_REG_MASKROM, &mask_rom_check, sizeof(mask_rom_check));
	if (err < 0) {
		/* cannot get EC version(no firmware) */
		dev_err(&mb_dev->pdev->dev,
			 "ec_ver is invalid(read MaskROM failed). \n");
		goto fail_not_found;
	}

	if (!(mask_rom_check & BOWSER_EC_MASK_ROM_BIT)) {
		dev_err(&mb_dev->pdev->dev, "MaskROM check failed. \n");
		err = -1;
		goto fail_invalid;
	}
	dev_dbg(&mb_dev->pdev->dev, "MaskROM check passed. \n");

	err = io373b_read_regs(&mb_dev->ec_i2c_client->dev,
			 EC_REG_KBC_VERSION, ec_ver, EC_FW_VERSION_LEN);

	if (err < 0) {
		/* cannot get EC version(no firmware) */
		dev_err(&mb_dev->pdev->dev,
				 "ec_ver is invalid(no EC firmware). \n");
		goto fail_not_found;
	}

	strncpy(mb_dev->ec_fw_version, ec_ver, EC_FW_VERSION_LEN);
	mb_dev->ec_version_valid = MB_EC_VER_VALID;
	dev_dbg(&mb_dev->pdev->dev, "%s end !! (EC version = %s)\n",
				 __func__, mb_dev->ec_fw_version);

	return err;

fail_not_found:
	dev_err(&mb_dev->pdev->dev, "%s EC version not found.\n", __func__);
	mb_dev->ec_version_valid = MB_EC_VER_NOT_FOUND;
	strncpy(mb_dev->ec_fw_version, MB_EC_VER_NOT_FOUND_VERSION,
						 EC_FW_VERSION_LEN);
	return err;

fail_invalid:
	dev_err(&mb_dev->pdev->dev, "%s EC version is invalid.\n", __func__);
	mb_dev->ec_version_valid = MB_EC_VER_INVALID;
	strncpy(mb_dev->ec_fw_version, MB_EC_VER_INVALID_VERSION,
						 EC_FW_VERSION_LEN);
	return err;
}

static int load_ec_driver(struct bowser_misc_dev *mb_dev, bool on)
{
	int err = 0;
	struct i2c_board_info ec_board_info =
				 {.type = BOWSER_EC_I2C_BOARD_TYPE,};
#ifdef BOWSER_MISC_TOUCHPAD
	struct i2c_board_info tp_i2c_board_info = {
			I2C_BOARD_INFO("hid", BOWSER_TM2735_ADDR),
			.platform_data = &tm2735_platformdata,
	};
#endif

	dev_dbg(&mb_dev->pdev->dev, "%s start !!\n", __func__);

	mutex_lock(&mb_dev->lock);

	if (on == true) {
		if (mb_dev->ec_driver_status == LOADED) {
			dev_err(&mb_dev->pdev->dev,
			 "EC driver was loaded, and won't be load again.\n");
			err = -1;
			goto out;
		}
		bowser_irq_request(mb_dev);

#ifdef BOWSER_MISC_TOUCHPAD
		tp_i2c_board_info.irq = mb_dev->tp_irq;
/* rmi over i2c
		struct i2c_board_info TP_board_info = {
				I2C_BOARD_INFO("rmi_i2c", BOWSER_TM2735_ADDR),
				.irq = mb_dev->tp_irq,
				.platform_data = &tm2735_platformdata,
		};
*/
		if (mb_dev->tp_loaded == true)
			dev_err(&mb_dev->pdev->dev, "WARN: TP driver was loaded.\n");
		else {
			/* new TP device */
			mb_dev->touchpad_i2c_client =
				i2c_new_device(mb_dev->adapter, &tp_i2c_board_info);
			if (!mb_dev->touchpad_i2c_client) {
				dev_err(&mb_dev->pdev->dev,
				"can't create new device(%s)\n", tp_i2c_board_info.type);
				err = -EBUSY;
			}
		}

		if(err != -EBUSY)
			mb_dev->tp_loaded = true;
		else
			mb_dev->tp_loaded = false;

		err = 0;
#endif	/* BOWSER_MISC_TOUCHPAD */

		ec_board_info.addr = BOWSER_EC_I2C_BOARD_ADDR;
		ec_board_info.irq  = mb_dev->kb_irq;

		mb_dev->ec_i2c_client =
			 i2c_new_device(mb_dev->adapter, &ec_board_info);

		if (!mb_dev->ec_i2c_client) {
			dev_err(&mb_dev->pdev->dev,
			 "can't create new device(%s)\n", ec_board_info.type);
			err = -EBUSY;
		}else{
			mb_dev->ec_driver_status = LOADED;

			/* check EC version (retrun err if no version is read.) */
			err = read_ec_ver(mb_dev);
			if (err < 0) {
				dev_err(&mb_dev->pdev->dev,
				 "%s: read_ec_ver() failed! err=%d\n", __func__, err);
				goto out;
			}

			err = io373b_add_subdevs(&mb_dev->ec_i2c_client->dev,
						 io373b_virtual_i2c_devs);
			if (err < 0) {
				dev_err(&mb_dev->pdev->dev,
					 "%s: io373b_add_subdevs() failed.\n", __func__);
				goto out;
			}

			mb_dev->io373_acpi_dev =
			 io373b_get_subdev(&mb_dev->ec_i2c_client->dev, "io373b-acpi");

			if (!mb_dev->io373_acpi_dev) {
				dev_err(&mb_dev->pdev->dev,
					 "%s: io373b_get_subdev() failed.\n", __func__);
				err = -ENODEV;
			}

			schedule_work(&mb_dev->work);
		}
	} else {
		if (mb_dev->ec_driver_status == UNLOADED) {
			dev_err(&mb_dev->pdev->dev,
			 "EC driver was unloaded, and won't be unloaded again.\n");
			err = -1;
			goto out;
		}

#ifdef BOWSER_MISC_TOUCHPAD
		if (mb_dev->touchpad_i2c_client) {
			i2c_unregister_device(mb_dev->touchpad_i2c_client);
			mb_dev->touchpad_i2c_client = NULL;
		}
		mb_dev->tp_loaded = false;
#endif	// BOWSER_MISC_TOUCHPAD

		if (mb_dev->ec_i2c_client) {
			i2c_unregister_device(mb_dev->ec_i2c_client);
			mb_dev->ec_i2c_client = NULL;
		}
		mb_dev->io373_acpi_dev = NULL;

		kbc_irq_free(mb_dev);

		mb_dev->ec_driver_status = UNLOADED;
	}
out:
	mutex_unlock(&mb_dev->lock);

	return err;
}

static ssize_t user_set_mute_led(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct bowser_misc_dev *mb_dev = dev_get_drvdata(dev);

	if (!strcmp(buf, "mute")) {
		mb_dev->mute = MB_MUTE;
	} else if (!strcmp(buf, "unmute")) {
		mb_dev->mute = MB_UNMUTE;
	} else {
		dev_err(&mb_dev->pdev->dev, "failed to set Mute LED. \n");
		return -EINVAL;
	}

	schedule_work(&mb_dev->work);

	return count;
}
static DEVICE_ATTR(set_mute_led, S_IWUSR | S_IWGRP, NULL, user_set_mute_led);

static ssize_t user_read_ec_version(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bowser_misc_dev *mb_dev = dev_get_drvdata(dev);
	ssize_t ret = 0;

	if (mb_dev->ec_version_valid == MB_EC_VER_VALID) {
		ret = sprintf(buf, "%s\n", mb_dev->ec_fw_version);
	} else if (mb_dev->ec_version_valid == MB_EC_VER_NOT_READ){
		ret = sprintf(buf, "%s\n", MB_EC_VER_NOT_READ_VERSION);
	} else if (mb_dev->ec_version_valid == MB_EC_VER_NOT_FOUND){
		ret = sprintf(buf, "%s\n", MB_EC_VER_NOT_FOUND_VERSION);
	} else if (mb_dev->ec_version_valid == MB_EC_VER_INVALID) {
		ret = sprintf(buf, "%s\n", MB_EC_VER_INVALID_VERSION);
	} else {
		ret = sprintf(buf, "%s\n", MB_EC_VER_ERR_VERSION);
		dev_dbg(&mb_dev->pdev->dev, "%s wrong version state\n", __func__);
	}

	return ret;
}
static DEVICE_ATTR(ec_version, S_IRUSR | S_IRGRP, user_read_ec_version, NULL);

static ssize_t user_read_fw_version(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bowser_misc_dev *mb_dev = dev_get_drvdata(dev);
	ssize_t ret = 0;

	if (mb_dev->fw.state == MB_FW_READY) {
		ret = sprintf(buf, "%s\n", mb_dev->fw.version);
	} else if (mb_dev->fw.state == MB_FW_NOT_READ) {
		ret = sprintf(buf, "not read\n");
	} else {
		ret = sprintf(buf, "invalid\n");
	}

	return ret;
}
static DEVICE_ATTR(fw_version, S_IRUSR | S_IRGRP, user_read_fw_version, NULL);

static ssize_t user_set_bat_charge(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct bowser_misc_dev *mb_dev = dev_get_drvdata(dev);

	if (!memcmp(buf, "charge", 6)) {
		mb_dev->charge_state = 0x00;
	} else if (!memcmp(buf, "discharge", 9)) {
		mb_dev->charge_state = 0x01;
	} else {
		dev_err(&mb_dev->pdev->dev, "failed to set battery charge. \n");
		return -EINVAL;
	}

	schedule_work(&mb_dev->work);

	return count;
}
static DEVICE_ATTR(set_bat_charge, S_IWUSR | S_IWGRP, NULL, user_set_bat_charge);

static ssize_t user_read_ac_online(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bowser_misc_dev *mb_dev = dev_get_drvdata(dev);
	ssize_t ret = 0;
	unsigned char ac_online = 0x00;

	if (mb_dev->ec_version_valid != MB_EC_VER_VALID) {
		dev_err(&mb_dev->pdev->dev, "%s failed (invalid EC version)\n", __func__);
		goto out;
	} else if (!mb_dev->ec_i2c_client) {
		dev_err(&mb_dev->pdev->dev, "%s failed (no i2c device)\n", __func__);
		goto out;
	} else if (!mb_dev->io373_acpi_dev) {
		dev_err(&mb_dev->pdev->dev,
		 "%s failed (no io373 acpi device)\n", __func__);
		goto out;
	} else if (mb_dev->ec_driver_status == UNLOADED) {
		dev_err(&mb_dev->pdev->dev,
		 "%s failed (Failed to load EC driver)\n", __func__);
		goto out;
	}

	ret = io373b_read_regs(&mb_dev->ec_i2c_client->dev,
		 EC_REG_GET_AC_STATUS, &ac_online, sizeof(ac_online));
	if (ret < 0) {
		dev_err(&mb_dev->pdev->dev,
			 "failed to get AC status from EC. \n");
		return ret;
	}

	if (ac_online & 0x02) {
		ret = sprintf(buf, "1\n");
	} else {
		ret = sprintf(buf, "0\n");
	}

	return ret;
out:
	return -1;
}
static DEVICE_ATTR(get_ac_online, S_IRUGO, user_read_ac_online, NULL);

static struct attribute *bowser_misc_attrs[] = {
	&dev_attr_set_mute_led.attr,
	&dev_attr_ec_version.attr,
	&dev_attr_fw_version.attr,
	&dev_attr_set_bat_charge.attr,
	&dev_attr_get_ac_online.attr,
	NULL
};

struct attribute_group bowser_misc_attr_group = {
	.attrs = bowser_misc_attrs,
};

static ssize_t
bowser_misc_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	struct bowser_misc_dev *mb_dev = file->private_data;
	int i = 0;

	dev_dbg(&mb_dev->pdev->dev,
			 "%s start !! Message = %s\n", __func__, Message);
	while (count && Message[i]) {
		put_user(Message[i], buf + i);
		count--;
		i++;
	}
	put_user('\0', buf + i);

	return i;
}

static ssize_t bowser_misc_write(struct file *file,
			 const char __user *buf, size_t count, loff_t *ppos)
{
	struct bowser_misc_dev *mb_dev = file->private_data;
	int i;

	dev_dbg(&mb_dev->pdev->dev,
		 "%s start !! count = %d\n", __func__, count);

	for (i = 0; i < count && BUF_LEN; i++)
		get_user(Message[i], buf + i);
	Message[i] = '\0';

	dev_dbg(&mb_dev->pdev->dev,
			 "%s Message = %s, length = %d\n", __func__, Message, i);

	return i;
}

static int bowser_misc_open(struct inode *inode, struct file *file)
{
	struct bowser_misc_dev *mb_dev = container_of(inode->i_cdev,
					struct bowser_misc_dev, cdev);
	dev_dbg(&mb_dev->pdev->dev, "%s start !!\n", __func__);

	/* for only one process at a time */
	if (test_and_set_bit(MISC_BUSY_BIT_POS, &mb_dev->flags)) {
		dev_err(&mb_dev->pdev->dev, "%s device is busy.\n", __func__);
		return -EBUSY;
	}

	file->private_data = mb_dev;
	dev_dbg(&mb_dev->pdev->dev, "%s finished \n", __func__);

	return 0;
}

static int bowser_misc_release(struct inode * inode, struct file *file)
{
	struct bowser_misc_dev *mb_dev =
			container_of(inode->i_cdev, struct bowser_misc_dev, cdev);
	dev_dbg(&mb_dev->pdev->dev, "%s start !!\n", __func__);

	clear_bit(MISC_BUSY_BIT_POS, &mb_dev->flags);

	return 0;
}

static long bowser_misc_ioctl(struct file *file,
					 unsigned int cmd, unsigned long arg)
{
	struct bowser_misc_dev *mb_dev = file->private_data;
	long err = 0;
	int len, need_sync = 0;

	dev_dbg(&mb_dev->pdev->dev, "%s start !!\n", __func__);
	switch (cmd) {
		case BOWSERBASE_R_READECVERSION:	/* Get EC Version. */
			if (mb_dev->ec_version_valid == MB_EC_VER_VALID) {
				strcpy(Message, mb_dev->ec_fw_version);
			} else if (mb_dev->ec_version_valid ==
							 MB_EC_VER_NOT_READ){
				strcpy(Message, MB_EC_VER_NOT_READ_VERSION);
			} else if (mb_dev->ec_version_valid ==
							 MB_EC_VER_NOT_FOUND){
				strcpy(Message, MB_EC_VER_NOT_FOUND_VERSION);
			} else if (mb_dev->ec_version_valid ==
							MB_EC_VER_INVALID) {
				strcpy(Message, MB_EC_VER_INVALID_VERSION);
			} else {
				strcpy(Message, MB_EC_VER_ERR_VERSION);
				dev_dbg(&mb_dev->pdev->dev,
					"%s wrong version state\n", __func__);
			}
			bowser_misc_read(file, (char*)arg, BUF_LEN, 0);
			break;
		case BOWSERBASE_W_SETMUTELED:	/* set mute led */
			len = strlen((char*)arg);
			bowser_misc_write(file, (char*)arg, len, 0);

			if (!strcmp(Message, "mute")) {
				mb_dev->mute = MB_MUTE;
			} else if (!strcmp(Message, "unmute")) {
				mb_dev->mute = MB_UNMUTE;
			}
			need_sync = 1;
			break;
		case BOWSERBASE_W_SETSYSTEMSTATE:	/* set system state */
			len = strlen((char*)arg);
			bowser_misc_write(file, (char*)arg, len, 0);

			if (!strcmp(Message, "shutdown")) {
				mb_dev->system_state = MB_SYS_SHUT_DOWN;
			} else if (!strcmp(Message, "running")) {
				mb_dev->system_state = MB_SYS_RUNNING;
			} else if (!strcmp(Message, "lp0")) {
				mb_dev->system_state = MB_SYS_IDLE;
			} else if (!strcmp(Message, "lp1")) {
				mb_dev->system_state = MB_SYS_SUSPEND;
			} else if (!strcmp(Message, "lp2")) {
				mb_dev->system_state = MB_SYS_DEEP_SLEEP;
			} else {
				err = -EINVAL;
			}
			need_sync = 1;
			break;
		case BOWSERBASE_W_UPDATEECFW:	/* update EC firmware */
			len = strlen((char*)arg);
			bowser_misc_write(file, (char*)arg, len, 0);

			mb_dev->do_update_fw = 1;
			need_sync = 1;
			break;
		case BOWSERBASE_W_SETSCREENSTATE:	/* set screen state */
			len = strlen((char*)arg);
			bowser_misc_write(file, (char*)arg, len, 0);

			if (!strcmp(Message, "on")) {
				mb_dev->screen_state = MB_SCREEN_ON;
			} else if (!strcmp(Message, "off")) {
				mb_dev->screen_state = MB_SCREEN_OFF;
			} else {
				err = -EINVAL;
			}
			need_sync = 1;
			break;
		case BOWSERBASE_R_READCONNSTATE: /* read base connection state */
			strcpy(Message, "connected");

			bowser_misc_read(file, (char*)arg, BUF_LEN, 0);
			break;
		case BOWSERBASE_R_READFWVERSION: /* read firmware file version */
			if (mb_dev->fw.state == MB_FW_READY) {
				strcpy(Message, mb_dev->fw.version);
			} else if (mb_dev->fw.state == MB_FW_NOT_READ) {
				strcpy(Message, "not read");
			} else {
				strcpy(Message, "invalid");
			}
			bowser_misc_read(file, (char*)arg, BUF_LEN, 0);
			break;
		case BOWSERBASE_W_SETLIDSTATE:
			len = strlen((char*)arg);
			bowser_misc_write(file, (char*)arg, len, 0);

			if (!strcmp(Message, "opened")) {
				mb_dev->lid_sw = MB_LID_OPENED;
			} else if (!strcmp(Message, "closed")) {
				mb_dev->lid_sw = MB_LID_CLOSED;
			}
			input_report_switch(mb_dev->input_dev,
						 SW_LID, !!mb_dev->lid_sw);
			input_sync(mb_dev->input_dev);
			break;
		case BOWSERBASE_W_FWUPDATENOTICE:
			len = strlen((char*)arg);
			bowser_misc_write(file, (char*)arg, len, 0);

			read_ec_ver(mb_dev);
			if (!strcmp(Message, "pressed")) {
				input_report_key(mb_dev->input_dev,
						 KEY_EC_UPDATE_NOTICE, (int)1);
				input_sync(mb_dev->input_dev);
				// msleep(300);
				input_report_key(mb_dev->input_dev,
						 KEY_EC_UPDATE_NOTICE, (int)0);
				input_sync(mb_dev->input_dev);
			}
			break;
		case BOWSERBASE_W_FWUPDATENOTICEEN:
			len = strlen((char*)arg);
			bowser_misc_write(file, (char*)arg, len, 0);

			if (!strcmp(Message, "disable")) {
				mb_dev->need_notify_os = 0;
			} else if (!strcmp(Message, "enable")) {
				mb_dev->need_notify_os = 1;
			}
			break;

		case BOWSERBASE_W_BLINKMUTELED:
			len = strlen((char*)arg);
			bowser_misc_write(file, (char*)arg, len, 0);

			if (kstrtol(Message, 10, &mb_dev->mute_led_blinking_freq)) {
				err = -EINVAL;
				break;
			}
			dev_dbg(&mb_dev->pdev->dev,
			 "%s ioctl set mute_led_blinking_freq = %ld\n",
					 __func__, mb_dev->mute_led_blinking_freq);
			need_sync = 1;
			break;
		case BOWSERBASE_W_BLINKCAPSLOCKLED:
			len = strlen((char*)arg);
			bowser_misc_write(file, (char*)arg, len, 0);

			if (kstrtol(Message, 10, &mb_dev->caps_lock_led_blinking_freq)) {
				err = -EINVAL;
				break;
			}
			dev_dbg(&mb_dev->pdev->dev,
			 "%s ioctl set caps_lock_led_blinking_freq = %ld\n",
					 __func__, mb_dev->caps_lock_led_blinking_freq);
			need_sync = 1;
			break;
		case BOWSERBASE_W_FWUPDATEFINISHED:
			if (mb_dev->update_success == 0) {
				strcpy(Message, EC_UPDATE_NOT_FINISHED_STRING);
			} else {
				strcpy(Message, EC_UPDATE_FINISHED_STRING);
			}
			bowser_misc_read(file, (char*)arg, BUF_LEN, 0);
			break;
		case BOWSERBASE_W_SETMESSAGELED:
			mb_dev->message_led_status= arg;
			need_sync = 1;
			break;
		case BOWSERBASE_W_SETKBMATRIXTYPE:
			len = strlen((char*)arg);
			bowser_misc_write(file, (char*)arg, len, 0);

			if (!strcmp(Message, "2")) {
				kb_matrix_type = 0x02;
			} else if (!strcmp(Message, "3")) {
				kb_matrix_type = 0x03;
			} else
				kb_matrix_type = 0x00;
			need_sync = 1;
			break;
		default:
			dev_err(&mb_dev->pdev->dev,
			 "%s: ioctl: not support command[%x]\n", __FILE__, cmd);
			err = -EFAULT;
			goto out;
	}

	if (need_sync) {
		schedule_work(&mb_dev->work);
	}
out:
	return err;
}

static const struct file_operations bowser_misc_ops = {
	.owner		= THIS_MODULE,
	.read		= bowser_misc_read,
	.write		= bowser_misc_write,
	.open		= bowser_misc_open,
	.release	= bowser_misc_release,
	.unlocked_ioctl	= bowser_misc_ioctl,
};

/* add a char device to create a ioctl entry for userspace control */
static int bowser_char_dev_init(struct bowser_misc_dev *mb_dev)
{
	int retval;

	dev_dbg(&mb_dev->pdev->dev, "%s start !!\n", __func__);

	mb_dev->major = 0;
	mb_dev->class = NULL;
	mb_dev->char_dev = NULL;

	retval = alloc_chrdev_region(&mb_dev->dev_id, BOWSER_MISC_FIRST_MINOR,
			BOWSER_MISC_MAX_DEVICES, "bowser_misc");

	mb_dev->major = MAJOR(mb_dev->dev_id);

	if (retval < 0) {
		dev_err(&mb_dev->pdev->dev,
				 "%s() can't get major number\n", __FILE__);
		goto done;
	}

	cdev_init(&mb_dev->cdev, &bowser_misc_ops);
	cdev_add(&mb_dev->cdev, mb_dev->dev_id, BOWSER_MISC_MAX_DEVICES);

	if (!mb_dev->class) {
		mb_dev->class = class_create(THIS_MODULE, "bowser_misc");
		if (IS_ERR(mb_dev->class)) {
			dev_err(&mb_dev->pdev->dev,
				 "[%s] failed in creating class\n", __FILE__);
			return -EINVAL;
		}
	}

	mb_dev->char_dev = device_create(mb_dev->class, NULL,
		MKDEV(mb_dev->major, 0), NULL, "%s", BOWSER_MISC_DEV_NAME);

	if (IS_ERR(mb_dev->char_dev)) {
		dev_err(&mb_dev->pdev->dev,
			"[%s] failed in creating %s device in sysfs\n",
						 __FILE__, BOWSER_MISC_DEV_NAME);
		return -EINVAL;
	}

	clear_bit(MISC_BUSY_BIT_POS, &mb_dev->flags);

	retval = sysfs_create_group(&mb_dev->pdev->dev.kobj, &bowser_misc_attr_group);
	if (retval) {
		dev_err(&mb_dev->pdev->dev, "error creating sysfs entries\n");
		return retval;
	}

	dev_dbg(&mb_dev->pdev->dev, "%s finished\n", __func__);
done:
	return retval;
}

/* reverse effect of bowser_char_dev_init()*/
static void bowser_char_dev_exit(struct bowser_misc_dev *mb_dev)
{
	dev_dbg(&mb_dev->pdev->dev, "%s start !!\n", __func__);

	cdev_del(&mb_dev->cdev);

	device_destroy(mb_dev->class, mb_dev->dev_id);
	class_destroy(mb_dev->class);
	mb_dev->class = NULL;

	unregister_chrdev_region(mb_dev->dev_id, BOWSER_MISC_MAX_DEVICES);
}

/* true = lid opened, false = lid closed */
bool bowser_is_lid_opened(void)
{
	if (!g_mb_dev)
		return true;

	return (g_mb_dev->lid_sw == MB_LID_OPENED) ? true : false;
}
EXPORT_SYMBOL(bowser_is_lid_opened);

void bowser_set_screen_state(int state)
{
	dev_dbg(&g_mb_dev->pdev->dev, "%s start !!\n", __func__);

	if (state == 1)
		g_mb_dev->screen_state = MB_SCREEN_ON;
	else
		g_mb_dev->screen_state = MB_SCREEN_OFF;
}
EXPORT_SYMBOL(bowser_set_screen_state);

/* lid switch interrupt handler */
static irqreturn_t lid_sw_irq_handler(int irq, void *dev_id)
{
	struct bowser_misc_dev *mb_dev = dev_id;
	enum lid_switch current_lid_state;

	disable_irq_nosync(mb_dev->lid_irq);

	/* Debounce */
	msleep(200);

	enable_irq(mb_dev->lid_irq);

	mutex_lock(&mb_dev->lock_lid);

	if(gpio_get_value(BOWSER_HALL_INT_GPIO)){
		current_lid_state = MB_LID_OPENED;
	}else{
		current_lid_state = MB_LID_CLOSED;
	}

	if (mb_dev->lid_sw == current_lid_state){
		mutex_unlock(&mb_dev->lock_lid);
		return IRQ_HANDLED;
	}

	mb_dev->lid_sw = current_lid_state;

	dev_dbg(&mb_dev->pdev->dev,
		 "%s send SW_LID value=%d\n", __func__, mb_dev->lid_sw);

	input_report_switch(mb_dev->input_dev, SW_LID, !!mb_dev->lid_sw);
	input_sync(mb_dev->input_dev);

	if (mb_dev->lid_sw == MB_LID_CLOSED) {
		elan_ktf3k_ts_pre_suspend();
#ifdef BOWSER_MISC_TOUCHPAD
		load_tp_driver(mb_dev, false);
#endif	/* BOWSER_MISC_TOUCHPAD */
	} else {
#ifdef BOWSER_MISC_TOUCHPAD
		load_tp_driver(mb_dev, true);
#endif	/* BOWSER_MISC_TOUCHPAD */
		elan_ktf3k_ts_pre_resume();
	}
	mutex_unlock(&mb_dev->lock_lid);

	schedule_work(&mb_dev->work);

	return IRQ_HANDLED;
}

static int bowser_misc_irq_init(struct bowser_misc_dev *mb_dev)
{
	int ret = 0;
	unsigned long req_flags =
		 IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT;

	dev_dbg(&mb_dev->pdev->dev, "%s start !!\n", __func__);
	/* In dalmore, controlling this gpio pin may casue system unable to
	 * suspend/resume issue. initialize HALL_INT# pin
	 */
	ret = gpio_request(BOWSER_HALL_INT_GPIO, BOWSER_LID_SW_IRQ_NAME);
	if (ret < 0) {
		dev_err(&mb_dev->pdev->dev,
		 "%s: %s gpio_request failed (err=%d)\n", __func__,
					 BOWSER_LID_SW_IRQ_NAME, ret);
		return ret;
	}

	ret = gpio_direction_input(BOWSER_HALL_INT_GPIO);
	if (ret < 0) {
		dev_err(&mb_dev->pdev->dev,
		 "%s: HALL_INT gpio_direction_output failed (err=%d)\n",
							 __func__, ret);
		gpio_free(BOWSER_HALL_INT_GPIO);
		return ret;
	}

	ret = mb_dev->lid_irq = gpio_to_irq(BOWSER_HALL_INT_GPIO);
	if (ret < 0) {
		dev_err(&mb_dev->pdev->dev,
		 "%s: gpio_to_irq() failed (err=%d)\n", __func__, ret);
		return ret;
	}

	ret = request_threaded_irq(mb_dev->lid_irq, NULL, lid_sw_irq_handler,
			req_flags, BOWSER_LID_SW_IRQ_NAME, mb_dev);
	if (ret < 0) {
		dev_err(&mb_dev->pdev->dev,
		 "%s: request_irq() failed (err=%d)\n", __func__, ret);
		return ret;
	}

	/* initialize EC reset pin */
	ret = gpio_request(BOWSER_EC_RST_GPIO, BOWSER_EC_RST_NAME);
	if (ret < 0) {
		dev_err(&mb_dev->pdev->dev, "%s: %s gpio_request failed %d\n",
				 __func__, BOWSER_EC_RST_NAME, ret);
		return ret;
	}

	ret = gpio_direction_output(BOWSER_EC_RST_GPIO, 0);
        if (ret < 0) {
		dev_err(&mb_dev->pdev->dev,
		 "%s: EC_reset gpio_direction_output failed %d\n", __func__, ret);
		gpio_free(BOWSER_EC_RST_GPIO);
		return ret;
	}

	return ret;
}

static int bowser_misc_input_dev_init(struct bowser_misc_dev *mb_dev)
{
	int err = 0;
	dev_dbg(&mb_dev->pdev->dev, "%s start !!\n", __func__);

	mb_dev->input_dev = input_allocate_device();
	if (!mb_dev->input_dev) {
		dev_err(&mb_dev->pdev->dev,
		 "%s: input_allocate_device() failed.\n", __func__);
		return -ENOMEM;
	}

	mb_dev->input_dev->name = "Bowser Lid Switch";
	mb_dev->input_dev->phys = "bowser_misc/gpio/input0";
	mb_dev->input_dev->id.bustype = BUS_HOST;
	mb_dev->input_dev->id.vendor = 0x0001;
	mb_dev->input_dev->id.product = 0x0001;
	mb_dev->input_dev->dev.parent = &mb_dev->pdev->dev;

	set_bit(EV_SW, mb_dev->input_dev->evbit);
	set_bit(EV_KEY, mb_dev->input_dev->evbit);
	set_bit(SW_LID, mb_dev->input_dev->swbit);

	err = input_register_device(mb_dev->input_dev);
	if (err) {
		dev_err(&mb_dev->pdev->dev,
			 "%s: Failed to register input device\n", __func__);
		input_free_device(mb_dev->input_dev);
		return err;
	}

	return 0;
}

static int bowser_misc_driver_probe(struct platform_device *pdev)
{
	int ret =  0;
	struct bowser_misc_dev *mb_dev;

	dev_dbg(&pdev->dev, "%s start !!\n", __func__);

	mb_dev = kzalloc(sizeof(struct bowser_misc_dev), GFP_KERNEL);
	if (mb_dev == NULL) {
		dev_err(&pdev->dev,
			 "%s kzalloc failed (Out of memory)\n", __func__);
		ret = -ENOMEM;
		goto exit;
	}

	g_mb_dev = mb_dev;
	dev_set_drvdata(&pdev->dev, mb_dev);
	mutex_init(&mb_dev->lock);
	mutex_init(&mb_dev->lock_lid);
	mutex_init(&mb_dev->lock_sync);
	INIT_WORK(&mb_dev->work, sync_ec_status_task);
	mb_dev->pdev = pdev;
	mb_dev->system_state = MB_SYS_RUNNING;
	mb_dev->ec_version_valid = MB_EC_VER_NOT_READ;
	mb_dev->fw.state = MB_FW_NOT_READ;
	mb_dev->lid_sw = MB_LID_OPENED;
	mb_dev->fw.name = EC_FW_NAME;
	mb_dev->need_notify_os = 1;
	mb_dev->lid_change = 0;
	mb_dev->mute_led_blinking_freq = 0x00;
	mb_dev->caps_lock_led_blinking_freq = 0x00;
	mb_dev->ec_driver_status = UNLOADED;
	mb_dev->do_update_fw = 0;
	mb_dev->update_success = 0;
	mb_dev->io373_acpi_dev = NULL;
	mb_dev->ec_i2c_client = NULL;
#ifdef BOWSER_MISC_TOUCHPAD
	mb_dev->touchpad_i2c_client = NULL;
	mb_dev->tp_loaded = false;
#endif	/* BOWSER_MISC_TOUCHPAD */
	mb_dev->message_led_status = 0;
	mb_dev->screen_state = MB_SCREEN_ON;
	mb_dev->charge_state = 0x00;
	mb_dev->mute = MB_UNMUTE;
	mb_dev->do_ec_ram_dump = false;
	strcpy(mb_dev->ec_fw_version, "NONE");

	device_init_wakeup(&mb_dev->pdev->dev, true);

	ret = bowser_misc_input_dev_init(mb_dev);
	if (ret) {
		dev_err(&pdev->dev,
			 "%s bowser_misc_input_dev_init() failed !!\n", __func__);
		goto exit;
	}

	ret = bowser_misc_irq_init(mb_dev);
	if (ret < 0) {
		dev_err(&pdev->dev,
			 "%s bowser_misc_irq_init() failed !!\n", __func__);
		goto exit;
	}

	ret = bowser_char_dev_init(mb_dev);
	if (ret < 0) {
		dev_err(&pdev->dev,
			 "%s bowser_char_dev_init() failed !!\n", __func__);
		goto exit;
	}

	/* get a i2c adapter for new i2c usage */
	mb_dev->adapter = i2c_get_adapter(BOWSER_EC_I2C_BUS);
	if (!mb_dev->adapter) {
		dev_err(&pdev->dev,
		 "can't get EC adpater (bus = %d)\n", BOWSER_EC_I2C_BUS);
                ret = -EBUSY;
                goto exit;
        }

	ret = get_ec_fw_file_version(mb_dev);
	if (ret < 0) {
		dev_err(&pdev->dev,
			 "%s get_ec_fw_file_version() failed !!\n", __func__);
		return ret;
	}

	ret = load_ec_driver(mb_dev, true);
	if (ret < 0) {
		dev_err(&pdev->dev, "%s failed to load EC driver\n", __func__);
		return ret;
	}


	dev_dbg(&pdev->dev, "%s finished\n", __func__);
exit:
	return ret;
}

static int bowser_misc_driver_remove(struct platform_device *pdev)
{
	struct bowser_misc_dev *mb_dev = dev_get_drvdata(&pdev->dev);
	dev_dbg(&pdev->dev, "%s start !!\n", __func__);

	bowser_misc_release_firmware(mb_dev->fw.fw);

	bowser_char_dev_exit(mb_dev);
	free_irq(mb_dev->lid_irq, mb_dev);

	gpio_free(BOWSER_HALL_INT_GPIO);

	input_unregister_device(mb_dev->input_dev);
//	kfree(mb_dev); // freed by free_irq()
	return 0;
}

static void bowser_misc_driver_shutdown(struct platform_device *pdev)
{
	struct bowser_misc_dev *mb_dev = dev_get_drvdata(&pdev->dev);
	dev_dbg(&pdev->dev, "%s start !!\n", __func__);

	mb_dev->system_state = MB_SYS_SHUT_DOWN;
	sync_ec_status_task(&mb_dev->work);
	return;
}

#ifdef CONFIG_PM
static int bowser_misc_driver_suspend(struct device *dev)
{
	struct bowser_misc_dev *mb_dev = dev_get_drvdata(dev);
	int ret;

	dev_dbg(dev, "%s start !!!\n", __func__);

	/* set irq can wake up kernel */
	if (device_may_wakeup(&mb_dev->pdev->dev)) {
		ret = enable_irq_wake(mb_dev->lid_irq);
		dev_dbg(dev, "%s enable_irq_wake(lid_irq) ret=%d", __func__, ret);
	}

	mb_dev->screen_state = MB_SCREEN_OFF;
	mb_dev->system_state = MB_SYS_SUSPEND;
	sync_ec_status_task(&mb_dev->work);
	return 0;
}

static int bowser_misc_driver_resume(struct device *dev)
{
	struct bowser_misc_dev *mb_dev = dev_get_drvdata(dev);

	dev_dbg(dev, "%s start !!\n", __func__);

	mb_dev->system_state = MB_SYS_RUNNING;
	sync_ec_status_task(&mb_dev->work);

	if (device_may_wakeup(&mb_dev->pdev->dev)) {
		disable_irq_wake(mb_dev->lid_irq);
	}

	return 0;
}

static const struct dev_pm_ops bowser_misc_dev_pm_ops = {
	.suspend = bowser_misc_driver_suspend,
	.resume = bowser_misc_driver_resume,
};
#endif

static struct platform_driver bowser_misc_driver = {
	.probe = bowser_misc_driver_probe,
	.remove = bowser_misc_driver_remove,
	.shutdown = bowser_misc_driver_shutdown,
	.driver = {
		.name = "bowser_misc",
#ifdef CONFIG_PM
		.pm   = &bowser_misc_dev_pm_ops,
#endif
	},
};

static int __init bowser_misc_init(void)
{
	int ret =  0;

	printk(KERN_INFO "%s start !!\n", __func__);

	ret = platform_driver_register(&bowser_misc_driver);
	if (ret < 0) {
		printk(KERN_ERR
			 "%s: platform_driver_register() failed (err=%d)\n",
							 __func__, ret);
	}

	return ret;
}

static void __exit bowser_misc_exit(void)
{
	printk(KERN_INFO "%s start !!\n", __func__);

	platform_driver_unregister(&bowser_misc_driver);
}

module_init(bowser_misc_init);
module_exit(bowser_misc_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jonathan Lin <jonathan.lin@insyde.com>");
MODULE_DESCRIPTION("Bowser misc Controller driver");
