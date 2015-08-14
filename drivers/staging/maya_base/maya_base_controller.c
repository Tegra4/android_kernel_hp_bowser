/*
 * Copyright (C) 2012 Insyde Software Corp.
 *
 *	work with io373b
 *
 * GPIO mapping:
 *	GPIO Name	PIN Name in schematic	Status
 *	GPIO3_PJ.03	SOC_DOCK_DET#		1 = undock, 0 = docked
 *	GPIO3_PO.06	SOC_I2C_DOCK_EN#	1 = disable, 0 = enable
 *	GPIO3_PS.00	HALL_INT* (lid switch)	1 = opened, 0 = closed
 *	GPIO3_PT.05	GEN2_I2C_SCL_1V8
 *	GPIO3_PT.06	GEN2_I2C_SDA_1V8
 *	GPIO3_PC.07	EC_INT_SW#		Low-active.
 */

/* mark following line to disable dev_dbg */
#define DEBUG

#define MAYA_BASE_TOUCHPAD
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

#include "maya_base_controller.h"
#include "io373x-b/io373b.h"

/* TP */
#ifdef MAYA_BASE_TOUCHPAD
#include <linux/i2c/i2c-hid.h>
#endif

#include <mach/dc.h>
#include "../../video/tegra/dc/dc_priv_defs.h"


/* I2C
 * @MAYA_BASE_I2C_BUS			GEN2_I2C bus number in dalmore
 * @MAYA_BASE_I2C_BOARD_TYPE		driver type of EC i2c device
 * @MAYA_BASE_I2C_BOARD_ADDR		EC i2c address with firmware installed
 * @MAYA_BASE_NO_FW_I2C_BOARD_ADDR	EC i2c address without firmware installed
 */
#define MAYA_BASE_I2C_BUS		1
#define MAYA_BASE_I2C_BOARD_TYPE	"io373b-smbd-base"
#define MAYA_BASE_I2C_BOARD_ADDR	0x62
#define MAYA_BASE_NO_FW_I2C_BOARD_ADDR	0x60

/* ec firmware
 * @EC_FW_VERSION_LEN			the length of EC firmware version
 * @EC_FW_VERSION_OFFSET		the offset of EC firmware version
 * @EC_FW_NAME				the name of EC firmware
 */
#define EC_FW_VERSION_LEN		6
#define EC_FW_VERSION_OFFSET		0x500
#define EC_FW_NAME			"maya_ec.bin"

/* GPIO */
#define TEGRA_GPIO_PC7			23		/* GPIO3_PC.07 */
#define TEGRA_GPIO_PI5			69		/* GMI_IORDY */
#define TEGRA_GPIO_PJ3			75		/* GPIO3_PJ.03 */
#define TEGRA_GPIO_PO5			117		/* GPIO3_PO.05 */
#define TEGRA_GPIO_PO6			118		/* GPIO3_PO.06 */
#define TEGRA_GPIO_PO7			119		/* ULPI_DATA6 */
#define TEGRA_GPIO_PQ5			133		/* KB_COL5 */
#define TEGRA_GPIO_PS0			144		/* GPIO3_PS.00 */
#define TEGRA_GPIO_PV0			168		/* GPIO_PV0 */
/*
 * @MAYA_PV_ID_GPIO		[Input] High = SI build, Low = PV build
 * @MAYA_PV_SOC_I2C_DOCK_EN_GPIO [Output] Dock enable pin in PV build
 * @MAYA_PV_SOC_DOCK_DET_GPIO	[Input] Dock Detection pin in PV build
 * @MAYA_SOC_I2C_DOCK_EN_GPIO	[Output] SOC_I2C_DOCK_EN# pin,
					 High = disable, low = enable
 * @MAYA_SOC_DOCK_DET_GPIO	[Input] SOC_DOCK_DET# pin,
					 High = undock, low = docked
 * @MAYA_HALL_INT_GPIO		[Input] LID Switch pin,
					 high = opened, low = closed
 * @MAYA_EC_INT_SW_GPIO		[Input] EC_INT_SW# pin, low-active,
 *			 SOC_I2C_DOCK_EN# must be enabled after base attached.
 * @MAYA_TP_IRQ_GPIO		[Input] TP_IRQ* pin
 * @MAYA_EC_RST_GPIO		[OUTPUT] EC reset/write protect pin
 *					 high active
 */
#define MAYA_PV_ID_GPIO			TEGRA_GPIO_PO7
#define MAYA_PV_SOC_I2C_DOCK_EN_GPIO	TEGRA_GPIO_PQ5
#define MAYA_PV_SOC_DOCK_DET_GPIO	TEGRA_GPIO_PI5
#define MAYA_SOC_I2C_DOCK_EN_GPIO	TEGRA_GPIO_PO6
#define MAYA_SOC_DOCK_DET_GPIO		TEGRA_GPIO_PJ3
#define MAYA_HALL_INT_GPIO		TEGRA_GPIO_PS0
#define MAYA_EC_INT_SW_GPIO		TEGRA_GPIO_PC7
#define MAYA_TP_IRQ_GPIO		TEGRA_GPIO_PO5
#define MAYA_EC_RST_GPIO		TEGRA_GPIO_PV0
#define MAYA_BASE_BOARD_ID_NAME		"boardID"
#define MAYA_BASE_DET_GPIO_NAME		"socDockDet"
#define MAYA_BASE_I2C_EN_GPIO_NAME	"socI2cDockEN"
#define MAYA_BASE_LID_SW_IRQ_NAME	"lid_switch"
#define MAYA_BASE_KB_IRQ_GPIO_NAME	"kb_irq"
#define MAYA_BASE_TP_IRQ_GPIO_NAME	"tp_irq"
#define MAYA_BASE_EC_RST_NAME		"EC_reset"

/* TP */
#define MAYA_BASE_TM2735_ADDR		0x2c
/* using LID_Switch pin for test
 *#define SYNAPTICS_RESET_GPIO		TEGRA_GPIO_PS0
 */

/* char device */
#define MAYA_BASE_FIRST_MINOR		0
#define MAYA_BASE_MAX_DEVICES		1
#define MAYA_BASE_DEV_NAME		"maya_base"

/* ioctl
 * MAYABASE_R_READECVERSION	read EC version
 * MAYABASE_W_SETBASESTATUS	(test only) set base status
 * MAYABASE_W_SETMUTELED	set mute LED
 * MAYABASE_W_SETSYSTEMSTATE	set system state
 * MAYABASE_W_UPDATEECFW	update firmware
 * MAYABASE_W_SETSCREENSTATE	set screen state
 * MAYABASE_R_READCONNSTATE	read base connection state
 * MAYABASE_R_READFWVERSION	read firmware version
 * MAYABASE_W_SETLIDSTATE	set lid open/close
 * MAYABASE_W_SETRFKILL		set rfkill key
 * MAYABASE_W_FWUPDATENOTICE	send notice for ec app
 * MAYABASE_W_BLINKMUTELED	set Mute LED blinking
 * MAYABASE_W_BLINKCAPSLOCKLED	set CapsLock LED blinking
 * MAYABASE_W_FWUPDATEFINISHED	read EC update flag
 */
#define MAYA_IOC_MAGIC				'M'
#define MAYABASE_R_READECVERSION	_IOR(MAYA_IOC_MAGIC, 1, int)
#define MAYABASE_W_SETBASESTATUS	_IOW(MAYA_IOC_MAGIC, 2, int)
#define MAYABASE_W_SETMUTELED		_IOW(MAYA_IOC_MAGIC, 3, int)
#define MAYABASE_W_SETSYSTEMSTATE	_IOW(MAYA_IOC_MAGIC, 4, int)
#define MAYABASE_W_UPDATEECFW		_IOW(MAYA_IOC_MAGIC, 5, int)
#define MAYABASE_W_SETSCREENSTATE	_IOW(MAYA_IOC_MAGIC, 6, int)
#define MAYABASE_R_READCONNSTATE	_IOR(MAYA_IOC_MAGIC, 8, int)
#define MAYABASE_R_READFWVERSION	_IOR(MAYA_IOC_MAGIC, 9, int)
#define MAYABASE_W_SETLIDSTATE		_IOW(MAYA_IOC_MAGIC, 11, int)
#define MAYABASE_W_SETRFKILL		_IOW(MAYA_IOC_MAGIC, 12, int)
#define MAYABASE_W_FWUPDATENOTICE	_IOW(MAYA_IOC_MAGIC, 13, int)
#define MAYABASE_W_FWUPDATENOTICEEN	_IOW(MAYA_IOC_MAGIC, 14, int)
#define MAYABASE_W_BLINKMUTELED		_IOW(MAYA_IOC_MAGIC, 15, int)
#define MAYABASE_W_BLINKCAPSLOCKLED	_IOW(MAYA_IOC_MAGIC, 16, int)
#define MAYABASE_W_FWUPDATEFINISHED	_IOW(MAYA_IOC_MAGIC, 17, int)

/* read/write */
#define BUF_LEN			100		/* length of Message */
#define BASECONNSTRING		"connected"	/* the connected string */
#define BASEDISCONNSTRING	"disconnected"	/* the disconnected string */
#define EC_UPDATE_NOT_FINISHED_STRING	"0"
#define EC_UPDATE_FINISHED_STRING	"1"

#define BASE_BUSY_BIT_POS	1
#define MB_EC_VER_NOT_FOUND_VERSION	"000000"
#define MB_EC_VER_NOT_READ_VERSION	"000001"
#define MB_EC_VER_INVALID_VERSION	"000002"
#define MB_EC_VER_ERR_VERSION		"000003"


// base info
enum maya_base_connection_state {
        MAYA_BASE_CONNECTED = 1,
        MAYA_BASE_DISCONNECTED,
};

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

enum attach_status {
	ATTACH_STATUS_SUCCESS = 0,
	ATTACH_STATUS_FAILURE,
};

struct maya_base_dev {
	struct platform_device *pdev;
	unsigned int lid_irq;
	unsigned int det_irq;
	unsigned int kb_irq;
	unsigned int tp_irq;
	struct work_struct work;
	struct mutex lock;
	struct mutex lock_det;
	struct mutex lock_lid;
	struct mutex lock_sync;
	struct input_dev *input_dev;
	unsigned long flags;
	long mute_led_blinking_freq;
	long caps_lock_led_blinking_freq;

	struct device *io373_acpi_dev;

	bool tp_loaded;
	int update_success;     /* 1 = success, 0 = failure */
	int do_update_fw;	/* 1 = update FW , 0 = don't update FW */
	int board_id;
	int det_gpio;
	int dock_en_gpio;
	int lid_change;		/* 1 = changed, 0 = no changed. */
	enum mute_state	mute;
	enum lid_switch	lid_sw;
	enum system_state system_state;
	enum maya_base_connection_state connection_state;
	enum screen_state screen_state;
	enum attach_status attach_status;
/* char device */
	int major;
	dev_t dev_id;
	struct cdev cdev;	/* associated character device */
	struct class *class;
	struct device *char_dev;
/* i2c */
	struct i2c_adapter *adapter;
	struct i2c_client *base_i2c_client;
	struct i2c_client *touchpad_i2c_client;
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
#ifdef CONFIG_SWITCH
	struct switch_dev dock_sw;
#endif //CONFIG_SWITCH
};

static char Message[BUF_LEN];	/* message from/to user space */

extern struct tegra_dc *g_maya_dc;

//* function */
static int maya_base_attach(struct maya_base_dev *);
static int maya_base_detach(struct maya_base_dev *);
static int maya_base_conn_change_handler(struct maya_base_dev *mb_dev,
				 enum maya_base_connection_state state);
static int read_ec_ver(struct maya_base_dev *mb_dev);


/* we use only 1 acpi device, which reports batteries/chargers
   info no matter they are on slate or base. */
union io373b_subdev_board_info acpi_info = {
        .acpi.num_bat = 1, /* number of batteries and ac-chargers */
	.acpi.bat_name[0]     = "battery-base",
	.acpi.charger_name[0] = "ac-base",
};

#ifdef SLATE_TOO /* then all batteries/chargers info go to slate acpi */
static struct i2c_board_info io373b_slate_virtual_i2c_devs[] = {
        { .type = "io373b-acpi",       /* battery/charger acpi info device */
          .addr = 0x07,                /* i2c-addr(7 bits) */
          .platform_data = &acpi_info, /* points to info required */
        },
        { },
};
#endif

static struct i2c_board_info io373b_base_virtual_i2c_devs[] = {
	/*               name              i2c-addr(7 bits) */
	{ I2C_BOARD_INFO("io373b-ps2-kbd", 0x01), },
#ifndef SLATE_TOO /* no slate io373b, then both acpi and kbd are on base */
	{ .type = "io373b-acpi",       /* battery/charger acpi info device */
	  .addr = 0x07,                /* i2c-addr(7 bits) */
	  .platform_data = &acpi_info, /* points to info required */
	},
#endif /* SLATE_TOO */
	{ },
};

#ifdef MAYA_BASE_TOUCHPAD
static struct i2c_hid_platform_data tm2735_platformdata = {
	.hid_descriptor_address = 0x20,
};
#endif	/* MAYA_BASE_TOUCHPAD */

static void kbc_tp_irq_free(struct maya_base_dev *mb_dev)
{
	gpio_free(MAYA_EC_INT_SW_GPIO);
#ifdef MAYA_BASE_TOUCHPAD
	gpio_free(MAYA_TP_IRQ_GPIO);
#endif
}

static int kbc_tp_irq_request(struct maya_base_dev *mb_dev)
{
	int ret = 0;

	/* get irq for EC(io373b) */
	ret = gpio_request(MAYA_EC_INT_SW_GPIO, MAYA_BASE_KB_IRQ_GPIO_NAME);
	if (ret < 0) {
		dev_err(&mb_dev->pdev->dev,
		 "%s: %s gpio_request failed (err=%d)\n", __func__,
					 MAYA_BASE_KB_IRQ_GPIO_NAME, ret);
		return ret;
	}

	ret = gpio_direction_input(MAYA_EC_INT_SW_GPIO);
	if (ret < 0) {
		dev_err(&mb_dev->pdev->dev,
		 "%s: KB_IRQ gpio_direction_output failed (err=%d)\n",
							 __func__, ret);
		gpio_free(MAYA_EC_INT_SW_GPIO);
		return ret;
	}

	ret = mb_dev->kb_irq = gpio_to_irq(MAYA_EC_INT_SW_GPIO);
	if (ret < 0) {
		dev_err(&mb_dev->pdev->dev,
		 "%s: gpio_to_irq() failed (err=%d)\n", __func__, ret);
		return ret;
	}

#ifdef MAYA_BASE_TOUCHPAD
	/* get irq for TP */
	ret = gpio_request(MAYA_TP_IRQ_GPIO,
					 MAYA_BASE_TP_IRQ_GPIO_NAME);
	if (ret < 0) {
		dev_err(&mb_dev->pdev->dev,
		 "%s: %s gpio_request failed (err=%d)\n", __func__,
					 MAYA_BASE_TP_IRQ_GPIO_NAME, ret);
		return ret;
	}

	ret = gpio_direction_input(MAYA_TP_IRQ_GPIO);
	if (ret < 0) {
		dev_err(&mb_dev->pdev->dev,
		 "%s: TP IRQ gpio_direction_output failed (err=%d)\n",
							 __func__, ret);
		gpio_free(MAYA_TP_IRQ_GPIO);
		return ret;
	}

	ret = mb_dev->tp_irq = gpio_to_irq(MAYA_TP_IRQ_GPIO);
	if (ret < 0) {
		dev_err(&mb_dev->pdev->dev,
		 "%s: gpio_to_irq() TP_IRQ failed (err=%d)\n", __func__, ret);
		return ret;
	}
#endif
	return ret;
}

static void send_update_key_event(struct maya_base_dev *mb_dev)
{
	dev_dbg(&mb_dev->pdev->dev, "%s start !!\n", __func__);

	if (gpio_get_value(mb_dev->det_gpio)) {
		dev_err(&mb_dev->pdev->dev,
			"%s: base is not connected. update_key_event is aborted.\n", __func__);
		return;
	}

	input_report_key(mb_dev->input_dev, KEY_EC_UPDATE_NOTICE, (int)1);
	input_sync(mb_dev->input_dev);
	msleep(200);
	input_report_key(mb_dev->input_dev, KEY_EC_UPDATE_NOTICE, (int)0);
	input_sync(mb_dev->input_dev);
}

static int install_ec_fw(struct maya_base_dev *mb_dev,
				 const struct firmware *fw)
{
	int ret = 0;
	int num_of_retry = 0, ec_driver_ret = 0;
	struct i2c_board_info ec_i2c_board_info =
			 {.type = MAYA_BASE_I2C_BOARD_TYPE,};

	mb_dev->update_success = 0;

	dev_dbg(&mb_dev->pdev->dev, "%s start !!\n", __func__);

	/* stop fw installation if base is disconnected */
	if (mb_dev->connection_state == MAYA_BASE_DISCONNECTED) {
		dev_err(&mb_dev->pdev->dev,
		 "%s (warning) no base , action aborted!!\n", __func__);
		return -ENODEV;
	} else if (!fw) {
		dev_err(&mb_dev->pdev->dev,
		 "%s (err) no firmware , action aborted!!\n", __func__);
		return -ENOENT;
	}

	/* To reduce occurance of bugs,
	 * we use a pure i2c device without platform data. */
	ret = maya_base_conn_change_handler(mb_dev, MAYA_BASE_DISCONNECTED);
	if (ret < 0) {
		dev_err(&mb_dev->pdev->dev,
			 "%s: failed, cannot disconnect base\n", __func__);
		/* continue anyway since EC may not have firmware */
	}

	gpio_set_value(mb_dev->dock_en_gpio, 0); /* enable i2c bus */
	msleep(10);

update:
	/* Disable EC write protection */
	gpio_set_value(MAYA_EC_RST_GPIO, 1);
	msleep(1200);
	gpio_set_value(MAYA_EC_RST_GPIO, 0);
	msleep(1000);	/* wait a while for EC reset */

	/* After EC reset, we read EC version to confirm its address*/
	ec_i2c_board_info.addr = MAYA_BASE_I2C_BOARD_ADDR;
	ec_i2c_board_info.irq  = mb_dev->kb_irq;

	mb_dev->base_i2c_client =
			 i2c_new_device(mb_dev->adapter, &ec_i2c_board_info);
	if (!mb_dev->base_i2c_client) {
		dev_err(&mb_dev->pdev->dev,
		 "can't create new device(%s)\n", ec_i2c_board_info.type);
		ret = -EBUSY;
		goto retry;
	}

	read_ec_ver(mb_dev);

	if (mb_dev->base_i2c_client)
		i2c_unregister_device(mb_dev->base_i2c_client);


	/* EC without FW(version not found): addr=0x60, EC with FW: addr=0x62 */
	if (mb_dev->ec_version_valid == MB_EC_VER_NOT_FOUND) {
		dev_dbg(&mb_dev->pdev->dev,
		 "%s: EC version not found, using addr=0x60\n", __func__);
		ec_i2c_board_info.addr = MAYA_BASE_NO_FW_I2C_BOARD_ADDR;
	} else {
		ec_i2c_board_info.irq  = mb_dev->kb_irq;
		ec_i2c_board_info.addr = MAYA_BASE_I2C_BOARD_ADDR;
	}

	mb_dev->base_i2c_client =
			 i2c_new_device(mb_dev->adapter, &ec_i2c_board_info);
	if (!mb_dev->base_i2c_client) {
		dev_err(&mb_dev->pdev->dev,
		 "can't create new device(%s)\n", ec_i2c_board_info.type);
		ret = -EBUSY;
		goto exit;
	}
	dev_dbg(&mb_dev->pdev->dev,
				 "%s i2c_new_device() finished.\n", __func__);

	ret = io373b_update_flash(&mb_dev->base_i2c_client->dev,
							 fw->data, fw->size);
	if (ret < 0) {
		dev_err(&mb_dev->pdev->dev,
		 "%s failed to install EC firmware!!(err=%d)\n", __func__, ret);
		/* keep going */
		ec_driver_ret = -1;
	}

	if (mb_dev->base_i2c_client)
		i2c_unregister_device(mb_dev->base_i2c_client);

	if (ec_driver_ret == -1 && num_of_retry < 3)
		goto retry;

	msleep(600);	// wait a while for EC reset.
	/* Before doing maya_base_conn_change_handler(), we have to make sure EC
	 * firmware is ready after firmware update. This prevent infinite loop
	 * since maya_base_conn_change_handler() will execute install_ec_fw()
	 * again if it failed to do read_ec_ver().
	 */
	ec_i2c_board_info.addr = MAYA_BASE_I2C_BOARD_ADDR;
	ec_i2c_board_info.irq  = mb_dev->kb_irq;

	mb_dev->base_i2c_client =
			 i2c_new_device(mb_dev->adapter, &ec_i2c_board_info);
	if (!mb_dev->base_i2c_client) {
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
		if (mb_dev->base_i2c_client)
			i2c_unregister_device(mb_dev->base_i2c_client);
		goto retry;
	}

	if (mb_dev->base_i2c_client)
		i2c_unregister_device(mb_dev->base_i2c_client);

	ret = maya_base_conn_change_handler(mb_dev, MAYA_BASE_CONNECTED);
	if (ret < 0) {
		dev_err(&mb_dev->pdev->dev,
			 "%s: failed, cannot connect base\n", __func__);
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

static void maya_base_release_firmware(const struct firmware *fw)
{
	if (fw) {
		release_firmware(fw);
		memset(&fw, 0, sizeof(fw));
	}
}

/* This function will be executed after request_firmware_nowait() is finished */
static void ec_firmware_cont(const struct firmware *fw, void *context)
{
	struct maya_base_dev *mb_dev = context;
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
static int get_ec_fw_file_version(struct maya_base_dev *mb_dev)
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

/* Only for LID */
static int load_tp_driver(struct maya_base_dev *mb_dev, bool on)
{
#ifdef MAYA_BASE_TOUCHPAD
	struct i2c_board_info tp_i2c_board_info = {
			I2C_BOARD_INFO("hid", MAYA_BASE_TM2735_ADDR),
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
#endif	// MAYA_BASE_TOUCHPAD
	return 0;
}

static void sync_base_status_task(struct work_struct *w)
{
	struct maya_base_dev *mb_dev = container_of(w,
				 struct maya_base_dev, work);
	unsigned char event1 = 0x00;
	unsigned char state;

	mutex_lock(&mb_dev->lock_sync);
	dev_dbg(&mb_dev->pdev->dev, "%s start !!\n", __func__);

	if (mb_dev->connection_state == MAYA_BASE_DISCONNECTED) {
		dev_err(&mb_dev->pdev->dev,
		 "%s failed to sync status(no connection)\n", __func__);
		goto out;
	} else if (mb_dev->ec_version_valid != MB_EC_VER_VALID) {
		dev_err(&mb_dev->pdev->dev,
		 "%s failed to sync status(invalid EC version)\n", __func__);
		goto out;
	} else if (!mb_dev->base_i2c_client) {
		dev_err(&mb_dev->pdev->dev,
		 "%s failed to sync status(no i2c device)\n", __func__);
		goto out;
	} else if (!mb_dev->io373_acpi_dev) {
		dev_err(&mb_dev->pdev->dev,
		 "%s failed to sync status(no io373 acpi device)\n", __func__);
		goto out;
	} else if (mb_dev->attach_status == ATTACH_STATUS_FAILURE) {
		dev_err(&mb_dev->pdev->dev,
		 "%s failed to sync status(Failed to attach base)\n", __func__);
		goto out;
	} else if (gpio_get_value(mb_dev->det_gpio)) {
		dev_err(&mb_dev->pdev->dev,
		 "%s failed to sync status(det_gpio=1)\n", __func__);
		goto out;
	}

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

	/* lid switch */
	if (mb_dev->lid_sw == MB_LID_OPENED) {
		event1 = event1 & ~MAYA_BASE_LID_CLOSE;
	} else if (mb_dev->lid_sw == MB_LID_CLOSED) {
		event1 = event1 | MAYA_BASE_LID_CLOSE;
	}

	/* mute led */
	if (mb_dev->mute == MB_MUTE) {
		event1 = event1 | MAYA_BASE_MUTE_LED_ON;
	} else if (mb_dev->mute == MB_UNMUTE) {
		event1 = event1 & ~MAYA_BASE_MUTE_LED_ON;
	}
	io373b_subdev_write(mb_dev->io373_acpi_dev,
		 EC_REG_ECRAM_EVENT1, &event1, sizeof(event1));

	/* Enable EC write protection */
	event1 = MAYA_BASE_WRITE_LOCK;
	io373b_subdev_write(mb_dev->io373_acpi_dev,
		 EC_REG_WRITE_PROTECT, &event1, sizeof(event1));

	/* system state */
	if (mb_dev->system_state == MB_SYS_RUNNING ||
		 mb_dev->system_state == MB_SYS_IDLE) {
		state = MAYA_BASE_S0;
	} else if (mb_dev->system_state == MB_SYS_SUSPEND) {
		state = MAYA_BASE_S3;
	} else if (mb_dev->system_state == MB_SYS_DEEP_SLEEP) {
		state = MAYA_BASE_S4;
	} else if (mb_dev->system_state == MB_SYS_SHUT_DOWN) {
		state = MAYA_BASE_S5;
	}
	io373b_subdev_write(mb_dev->io373_acpi_dev,
			 EC_REG_SYSTEM_STATE, &state, sizeof(state));

out:
	mutex_unlock(&mb_dev->lock_sync);

	if (mb_dev->do_update_fw) {
		mb_dev->do_update_fw = 0;
		install_ec_fw(mb_dev, mb_dev->fw.fw);
	}

	return;
}

/* return 0 if the version matched, return -1 if not*/
static int check_ec_fw_version(struct maya_base_dev *mb_dev)
{
	dev_dbg(&mb_dev->pdev->dev, "%s start !!\n", __func__);

	if (mb_dev->fw.state != MB_FW_READY) {
		dev_err(&mb_dev->pdev->dev,
			 "%s fw version is not found\n", __func__);
		return 1; // do not send update key.
	}

	if (strcmp(mb_dev->ec_fw_version, mb_dev->fw.version)) {
		dev_err(&mb_dev->pdev->dev,
			 "%s fw version not match EC = %s, FW = %s\n",
			 __func__, mb_dev->ec_fw_version, mb_dev->fw.version);
		return -1;
	}
	return 0;
}

/* read EC version */
static int read_ec_ver(struct maya_base_dev *mb_dev)
{
	int err = 0;
	char ec_ver[EC_FW_VERSION_LEN];
	unsigned char mask_rom_check;

	dev_dbg(&mb_dev->pdev->dev, "%s start !!\n", __func__);

	if (gpio_get_value(mb_dev->det_gpio)) {
		dev_err(&mb_dev->pdev->dev,
			 "%s: base is not connected. \n", __func__);
		return -ENODEV;
	}

	if (!mb_dev->base_i2c_client) {
		dev_err(&mb_dev->pdev->dev, "%s() no i2c device.\n", __func__);
		return -ENODEV;
	}

	/* check EC MaskROM */
	err = io373b_read_regs(&mb_dev->base_i2c_client->dev,
		 EC_REG_MASKROM, &mask_rom_check, sizeof(mask_rom_check));
	if (err < 0) {
		/* cannot get EC version(no firmware) */
		dev_err(&mb_dev->pdev->dev,
			 "ec_ver is invalid(read MaskROM failed). \n");
		goto fail_not_found;
	}

	if (!(mask_rom_check & MAYA_BASE_MASK_ROM_BIT)) {
		dev_err(&mb_dev->pdev->dev, "MaskROM check failed. \n");
		err = -1;
		goto fail_invalid;
	}
	dev_dbg(&mb_dev->pdev->dev, "MaskROM check passed. \n");

	err = io373b_read_regs(&mb_dev->base_i2c_client->dev,
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

/* Do not call this function directly, plz use maya_base_conn_change_handler()*/
static int maya_base_attach(struct maya_base_dev *mb_dev)
{
	int err = 0;
	struct i2c_board_info base_board_info =
				 {.type = MAYA_BASE_I2C_BOARD_TYPE,};
#ifdef MAYA_BASE_TOUCHPAD
	struct i2c_board_info tp_i2c_board_info = {
			I2C_BOARD_INFO("hid", MAYA_BASE_TM2735_ADDR),
			.platform_data = &tm2735_platformdata,
	};
#endif

	kbc_tp_irq_request(mb_dev);

#ifdef MAYA_BASE_TOUCHPAD
	tp_i2c_board_info.irq = mb_dev->tp_irq;
/* rmi over i2c
	struct i2c_board_info TP_board_info = {
			I2C_BOARD_INFO("rmi_i2c", MAYA_BASE_TM2735_ADDR),
			.irq = mb_dev->tp_irq,
			.platform_data = &tm2735_platformdata,
	};
*/
#endif	/* MAYA_BASE_TOUCHPAD */

	dev_dbg(&mb_dev->pdev->dev, "%s start !!\n", __func__);

	/*	Pull down SOC_I2C_DOCK_EN# pin to enable I2C interface
	 * when base attached.
	 *	mb_dev->dock_en_gpio:
	 * SOC_I2C_DOCK_EN# pin, High = disable, low = enable
	 */
	gpio_set_value(mb_dev->dock_en_gpio, 0);

	msleep(50);	// time for stabilization of i2c bus.

	if (mb_dev->tp_loaded == true)
		dev_err(&mb_dev->pdev->dev, "WARN: TP driver was loaded.\n");
#ifdef MAYA_BASE_TOUCHPAD
	/* new TP device */
	else{
		mb_dev->touchpad_i2c_client =
			 i2c_new_device(mb_dev->adapter, &tp_i2c_board_info);
		if (!mb_dev->touchpad_i2c_client) {
			dev_err(&mb_dev->pdev->dev,
			 "can't create new device(%s)\n", tp_i2c_board_info.type);
			err = -EBUSY;
		}
	}
#endif	/* MAYA_BASE_TOUCHPAD */
	if(err != -EBUSY)
		mb_dev->tp_loaded = true;
	else
		mb_dev->tp_loaded = false;
	err = 0;

	base_board_info.addr = MAYA_BASE_I2C_BOARD_ADDR;
	base_board_info.irq  = mb_dev->kb_irq;
	// base_board_info.platform_data = &io373b_base_virtual_i2c_devs;

	mb_dev->base_i2c_client =
			 i2c_new_device(mb_dev->adapter, &base_board_info);
	if (!mb_dev->base_i2c_client) {
		dev_err(&mb_dev->pdev->dev,
		 "can't create new device(%s)\n", base_board_info.type);
		err = -EBUSY;
	}else{
	// msleep(10ms);  /* might need some time for driver to new devices. */

	/* check EC version (retrun err if no version is read.) */
		err = read_ec_ver(mb_dev);
		if (err < 0) {
			dev_err(&mb_dev->pdev->dev,
			 "%s: read_ec_ver() failed! err=%d\n", __func__, err);
			return err;
		}

		err = io373b_add_subdevs(&mb_dev->base_i2c_client->dev,
					 io373b_base_virtual_i2c_devs);
		if (err < 0) {
			dev_err(&mb_dev->pdev->dev,
				 "%s: io373b_add_subdevs() failed.\n", __func__);
				return err;
		}

		mb_dev->io373_acpi_dev =
			 io373b_get_subdev(&mb_dev->base_i2c_client->dev, "io373b-acpi");

		if (!mb_dev->io373_acpi_dev) {
			dev_err(&mb_dev->pdev->dev,
				 "%s: io373b_get_subdev() failed.\n", __func__);
			return -ENODEV;
		}
	}
	return err;
}

/* Do not call this function directly, plz use maya_base_conn_change_handler()*/
static int maya_base_detach(struct maya_base_dev *mb_dev)
{
	dev_dbg(&mb_dev->pdev->dev, "%s start !!\n", __func__);

	/*	pull up SOC_I2C_DOCK_EN# pin to disable I2C interface
	 * when base detached.
	 *
	 * mb_dev->dock_en_gpio:
	 *	 (Output) SOC_I2C_DOCK_EN# pin, High = disable, low = enable
	 */
	gpio_set_value(mb_dev->dock_en_gpio, 1);

#ifdef MAYA_BASE_TOUCHPAD
	if (mb_dev->touchpad_i2c_client) {
		i2c_unregister_device(mb_dev->touchpad_i2c_client);
		mb_dev->touchpad_i2c_client = NULL;
	}
	mb_dev->tp_loaded = false;
#endif	// MAYA_BASE_TOUCHPAD

	if (mb_dev->base_i2c_client) {
		i2c_unregister_device(mb_dev->base_i2c_client);
		mb_dev->base_i2c_client = NULL;
	}
	mb_dev->io373_acpi_dev = NULL;

	kbc_tp_irq_free(mb_dev);

	return 0;
}

static int wait_for_fw(struct maya_base_dev *mb_dev, int time)
{
	int i;

	for (i = 0; i < time; i++) {
		msleep(500);
		dev_dbg(&mb_dev->pdev->dev, "%s wait for FW (%d)\n", __func__, i);
		if (mb_dev->fw.state == MB_FW_READY)
			return 0;
	}
	dev_dbg(&mb_dev->pdev->dev, "%s timeout waiting for FW\n", __func__);

	return -1;
}

/* handle base connection and disconnection operation*/
static int maya_base_conn_change_handler(struct maya_base_dev *mb_dev,
				 enum maya_base_connection_state state)
{
	int ret = 0;
	int i;

	dev_dbg(&mb_dev->pdev->dev, "%s start !!\n", __func__);

	if (mb_dev->connection_state == state) {
		dev_err(&mb_dev->pdev->dev,
		 "%s failed to change connection state (state is the same)\n",
								 __func__);
		return -EINVAL;
	}

	mutex_lock(&mb_dev->lock);
	if (state == MAYA_BASE_CONNECTED) {
		mb_dev->connection_state = MAYA_BASE_CONNECTED;
		for (i = 0; i < 3; i++) { // retry if failed
			ret = maya_base_attach(mb_dev);
			if (ret >= 0 || i >= 2) // exit if success or 4 times retry
				break;
			maya_base_detach(mb_dev);
			msleep(200);
			dev_err(&mb_dev->pdev->dev,
			 "%s retry... attach base again...%d \n", __func__, i);
		}

		if (ret < 0) {
			mb_dev->attach_status = ATTACH_STATUS_FAILURE;
			dev_err(&mb_dev->pdev->dev,
			 "%s failed to attach base. \n", __func__);
		} else {
			mb_dev->attach_status = ATTACH_STATUS_SUCCESS;
		}

#ifdef CONFIG_SWITCH
		switch_set_state(&mb_dev->dock_sw, 1);
#endif //CONFIG_SWITCH
	} else {
		mb_dev->connection_state = MAYA_BASE_DISCONNECTED;
		ret = maya_base_detach(mb_dev);
		if (ret < 0) {
			dev_err(&mb_dev->pdev->dev,
			 "%s failed to detach base. \n", __func__);
		}
		mb_dev->attach_status = ATTACH_STATUS_FAILURE;

#ifdef CONFIG_SWITCH
		switch_set_state(&mb_dev->dock_sw, 0);
#endif //CONFIG_SWITCH
	}
	mutex_unlock(&mb_dev->lock);

	if (mb_dev->connection_state == MAYA_BASE_CONNECTED) {
		/* Things to do after base connected. */

		if (mb_dev->ec_version_valid == MB_EC_VER_NOT_FOUND ||
			mb_dev->ec_version_valid == MB_EC_VER_INVALID) {
			if (mb_dev->fw.state != MB_FW_READY) {
				dev_err(&mb_dev->pdev->dev,
					"%s fw version is not found\n", __func__);
				if (wait_for_fw(mb_dev, 8)) // wait 4 seconds
					return -1; // do not send update key.
			}
			ret = install_ec_fw(mb_dev, mb_dev->fw.fw);
			if (ret < 0) {
				dev_err(&mb_dev->pdev->dev,
				 "install EC firmware failed. \n");
				 /* EC cannot be updated, lost EC connection.*/
				return -1;
			}
		}
		/* check EC firmware version,
		 *  and send key code if the version does not matched
		 */
		ret = check_ec_fw_version(mb_dev);
		if (ret < 0) { // EC firmware does not match.
			dev_err(&mb_dev->pdev->dev,
			 "%s EC FW version does not matched."
			 " Send firmware update key code \n",  __func__);
			if (mb_dev->need_notify_os)
				send_update_key_event(mb_dev);
		}

#ifdef INSTALL_FW_IF_NOT_MATCH
		if ((ret < 0) || (mb_dev->ec_version_valid != MB_EC_VER_VALID)) {
			/* (for test) install EC firmware
			 *  if the version does not match.
			 */
			ret = install_ec_fw(mb_dev, mb_dev->fw.fw);
			if (ret < 0)
				dev_err(&mb_dev->pdev->dev,
				 "install EC firmware failed. \n");
		}
#endif	//INSTALL_IF_NOT_MATCH

		/* synchronize slate info with base
		 * (ex: Lid, mute LED, Wifi state, System state, etc.)
		 */
		schedule_work(&mb_dev->work);
	}
	return ret;
//out:
//	mutex_unlock(&mb_dev->lock);
//	return ret;
}

static ssize_t user_set_mute_led(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct maya_base_dev *mb_dev = dev_get_drvdata(dev);

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
	struct maya_base_dev *mb_dev = dev_get_drvdata(dev);
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
	struct maya_base_dev *mb_dev = dev_get_drvdata(dev);
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

static int maya_create_dev_attrs(struct maya_base_dev *mb_dev)
{
	int err = 0;

	if ((err = device_create_file(&mb_dev->pdev->dev, &dev_attr_set_mute_led)))
	{
		dev_err(&mb_dev->pdev->dev, "Failed to create attr set_mute_led errno=%d\n", err);
                return err;
	}

	if ((err = device_create_file(&mb_dev->pdev->dev, &dev_attr_ec_version)))
	{
		dev_err(&mb_dev->pdev->dev, "Failed to create attr ec_version errno=%d\n", err);
		device_remove_file(&mb_dev->pdev->dev, &dev_attr_set_mute_led);
                return err;
	}

	if ((err = device_create_file(&mb_dev->pdev->dev, &dev_attr_fw_version)))
	{
		dev_err(&mb_dev->pdev->dev, "Failed to create attr fw_version errno=%d\n", err);
		device_remove_file(&mb_dev->pdev->dev, &dev_attr_set_mute_led);
		device_remove_file(&mb_dev->pdev->dev, &dev_attr_ec_version);
                return err;
	}
	return err;
}

static ssize_t
maya_base_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	struct maya_base_dev *mb_dev = file->private_data;
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

static ssize_t maya_base_write(struct file *file,
			 const char __user *buf, size_t count, loff_t *ppos)
{
	struct maya_base_dev *mb_dev = file->private_data;
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

static int maya_base_open(struct inode *inode, struct file *file)
{
	struct maya_base_dev *mb_dev = container_of(inode->i_cdev,
					struct maya_base_dev, cdev);
	dev_dbg(&mb_dev->pdev->dev, "%s start !!\n", __func__);

	/* for only one process at a time */
	if (test_and_set_bit(BASE_BUSY_BIT_POS, &mb_dev->flags)) {
		dev_err(&mb_dev->pdev->dev, "%s device is busy.\n", __func__);
		return -EBUSY;
	}

	file->private_data = mb_dev;
	dev_dbg(&mb_dev->pdev->dev, "%s finished \n", __func__);

	return 0;
}

static int maya_base_release(struct inode * inode, struct file *file)
{
	struct maya_base_dev *mb_dev =
			container_of(inode->i_cdev, struct maya_base_dev, cdev);
	dev_dbg(&mb_dev->pdev->dev, "%s start !!\n", __func__);

	clear_bit(BASE_BUSY_BIT_POS, &mb_dev->flags);

	return 0;
}

static long maya_base_ioctl(struct file *file,
					 unsigned int cmd, unsigned long arg)
{
	struct maya_base_dev *mb_dev = file->private_data;
	long err = 0;
	int len, need_sync = 0;

	dev_dbg(&mb_dev->pdev->dev, "%s start !!\n", __func__);
	switch (cmd) {
		case MAYABASE_R_READECVERSION:	/* Get EC Version. */
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
			maya_base_read(file, (char*)arg, BUF_LEN, 0);
			break;
		case MAYABASE_W_SETBASESTATUS:	/* (test only) */
			len = strlen((char*)arg);
			maya_base_write(file, (char*)arg, len, 0);

			if (!strcmp(Message, "connect")) {
				maya_base_conn_change_handler(mb_dev,
							 MAYA_BASE_CONNECTED);
			} else if (!strcmp(Message, "disconnect")) {
				maya_base_conn_change_handler(mb_dev,
							 MAYA_BASE_DISCONNECTED);
			}
			break;
		case MAYABASE_W_SETMUTELED:	/* set mute led */
			len = strlen((char*)arg);
			maya_base_write(file, (char*)arg, len, 0);

			if (!strcmp(Message, "mute")) {
				mb_dev->mute = MB_MUTE;
			} else if (!strcmp(Message, "unmute")) {
				mb_dev->mute = MB_UNMUTE;
			}
			need_sync = 1;
			break;
		case MAYABASE_W_SETSYSTEMSTATE:	/* set system state */
			len = strlen((char*)arg);
			maya_base_write(file, (char*)arg, len, 0);

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
		case MAYABASE_W_UPDATEECFW:	/* update EC firmware */
			len = strlen((char*)arg);
			maya_base_write(file, (char*)arg, len, 0);

			mb_dev->do_update_fw = 1;
			need_sync = 1;
			break;
		case MAYABASE_W_SETSCREENSTATE:	/* set screen state */
			len = strlen((char*)arg);
			maya_base_write(file, (char*)arg, len, 0);

			if (!strcmp(Message, "screen_on")) {
				mb_dev->screen_state = MB_SCREEN_ON;
			} else if (!strcmp(Message, "lp2")) {
				mb_dev->screen_state = MB_SCREEN_OFF;
			} else {
				err = -EINVAL;
			}
			break;
		case MAYABASE_R_READCONNSTATE: /* read base connection state */
			if (mb_dev->connection_state == MAYA_BASE_CONNECTED) {
				strcpy(Message, BASECONNSTRING);
			} else {
				strcpy(Message, BASEDISCONNSTRING);
			}

			maya_base_read(file, (char*)arg, BUF_LEN, 0);
			break;
		case MAYABASE_R_READFWVERSION: /* read firmware file version */
			if (mb_dev->fw.state == MB_FW_READY) {
				strcpy(Message, mb_dev->fw.version);
			} else if (mb_dev->fw.state == MB_FW_NOT_READ) {
				strcpy(Message, "not read");
			} else {
				strcpy(Message, "invalid");
			}
			maya_base_read(file, (char*)arg, BUF_LEN, 0);
			break;
		case MAYABASE_W_SETLIDSTATE:
			len = strlen((char*)arg);
			maya_base_write(file, (char*)arg, len, 0);

			if (!strcmp(Message, "opened")) {
				mb_dev->lid_sw = MB_LID_OPENED;
			} else if (!strcmp(Message, "closed")) {
				mb_dev->lid_sw = MB_LID_CLOSED;
			}
			input_report_switch(mb_dev->input_dev,
						 SW_LID, !!mb_dev->lid_sw);
			input_sync(mb_dev->input_dev);
			break;
		case MAYABASE_W_SETRFKILL:
			len = strlen((char*)arg);
			maya_base_write(file, (char*)arg, len, 0);

			if (!strcmp(Message, "pressed")) {
				input_report_key(mb_dev->input_dev,
							 KEY_RFKILL, (int)1);
				input_sync(mb_dev->input_dev);
			} else if (!strcmp(Message, "released")) {
				input_report_key(mb_dev->input_dev,
							 KEY_RFKILL, (int)0);
				input_sync(mb_dev->input_dev);
			}
			break;
		case MAYABASE_W_FWUPDATENOTICE:
			len = strlen((char*)arg);
			maya_base_write(file, (char*)arg, len, 0);

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
		case MAYABASE_W_FWUPDATENOTICEEN:
			len = strlen((char*)arg);
			maya_base_write(file, (char*)arg, len, 0);

			if (!strcmp(Message, "disable")) {
				mb_dev->need_notify_os = 0;
			} else if (!strcmp(Message, "enable")) {
				mb_dev->need_notify_os = 1;
			}
			break;

		case MAYABASE_W_BLINKMUTELED:
			len = strlen((char*)arg);
			maya_base_write(file, (char*)arg, len, 0);

			if (kstrtol(Message, 10, &mb_dev->mute_led_blinking_freq)) {
				err = -EINVAL;
				break;
			}
			dev_dbg(&mb_dev->pdev->dev,
			 "%s ioctl set mute_led_blinking_freq = %ld\n",
					 __func__, mb_dev->mute_led_blinking_freq);
			need_sync = 1;
			break;
		case MAYABASE_W_BLINKCAPSLOCKLED:
			len = strlen((char*)arg);
			maya_base_write(file, (char*)arg, len, 0);

			if (kstrtol(Message, 10, &mb_dev->caps_lock_led_blinking_freq)) {
				err = -EINVAL;
				break;
			}
			dev_dbg(&mb_dev->pdev->dev,
			 "%s ioctl set caps_lock_led_blinking_freq = %ld\n",
					 __func__, mb_dev->caps_lock_led_blinking_freq);
			need_sync = 1;
			break;
		case MAYABASE_W_FWUPDATEFINISHED:
			if (mb_dev->update_success == 0) {
				strcpy(Message, EC_UPDATE_NOT_FINISHED_STRING);
			} else {
				strcpy(Message, EC_UPDATE_FINISHED_STRING);
			}
			maya_base_read(file, (char*)arg, BUF_LEN, 0);
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

static const struct file_operations maya_base_ops = {
	.owner		= THIS_MODULE,
	.read		= maya_base_read,
	.write		= maya_base_write,
	.open		= maya_base_open,
	.release	= maya_base_release,
	.unlocked_ioctl	= maya_base_ioctl,
};

/* add a char device to create a ioctl entry for userspace control */
static int maya_char_dev_init(struct maya_base_dev *mb_dev)
{
	int retval;

	dev_dbg(&mb_dev->pdev->dev, "%s start !!\n", __func__);

	mb_dev->major = 0;
	mb_dev->class = NULL;
	mb_dev->char_dev = NULL;

	retval = alloc_chrdev_region(&mb_dev->dev_id, MAYA_BASE_FIRST_MINOR,
			MAYA_BASE_MAX_DEVICES, "maya_base");

	mb_dev->major = MAJOR(mb_dev->dev_id);

	if (retval < 0) {
		dev_err(&mb_dev->pdev->dev,
				 "%s() can't get major number\n", __FILE__);
		goto done;
	}

	cdev_init(&mb_dev->cdev, &maya_base_ops);
	cdev_add(&mb_dev->cdev, mb_dev->dev_id, MAYA_BASE_MAX_DEVICES);

	if (!mb_dev->class) {
		mb_dev->class = class_create(THIS_MODULE, "maya_base");
		if (IS_ERR(mb_dev->class)) {
			dev_err(&mb_dev->pdev->dev,
				 "[%s] failed in creating class\n", __FILE__);
			return -EINVAL;
		}
	}

	mb_dev->char_dev = device_create(mb_dev->class, NULL,
		MKDEV(mb_dev->major, 0), NULL, "%s", MAYA_BASE_DEV_NAME);

	if (IS_ERR(mb_dev->char_dev)) {
		dev_err(&mb_dev->pdev->dev,
			"[%s] failed in creating %s device in sysfs\n",
						 __FILE__, MAYA_BASE_DEV_NAME);
		return -EINVAL;
	}

	clear_bit(BASE_BUSY_BIT_POS, &mb_dev->flags);

	dev_dbg(&mb_dev->pdev->dev, "%s finished\n", __func__);
done:
	return retval;
}

/* reverse effect of maya_char_dev_init()*/
static void maya_char_dev_exit(struct maya_base_dev *mb_dev)
{
	dev_dbg(&mb_dev->pdev->dev, "%s start !!\n", __func__);

	cdev_del(&mb_dev->cdev);

	device_destroy(mb_dev->class, mb_dev->dev_id);
	class_destroy(mb_dev->class);
	mb_dev->class = NULL;

	unregister_chrdev_region(mb_dev->dev_id, MAYA_BASE_MAX_DEVICES);
}

static void maya_base_set_touchpad_power_mode(struct maya_base_dev *mb_dev)
{
	dev_dbg(&mb_dev->pdev->dev, "%s start !!\n", __func__);
}

#ifdef CONFIG_MACH_MAYA
extern int g_maya_base_state;
#endif
static irqreturn_t base_detect_handler(int irq, void *dev_id)
{
	struct maya_base_dev *mb_dev = dev_id;
	int i, high = 0, low = 0;

	enum maya_base_connection_state current_maya_base_conn_state;

	dev_dbg(&mb_dev->pdev->dev, "%s start !\n", __func__);
	disable_irq_nosync(mb_dev->det_irq);

	/* Debounce */
	for (i = 0; i < 100; i++) {
		msleep(5);
		if (gpio_get_value(mb_dev->det_gpio)) {
			high++;
			low = 0;
			if (high == 20) {
				current_maya_base_conn_state = MAYA_BASE_DISCONNECTED;
				if (mb_dev->connection_state != current_maya_base_conn_state)
					break;
				high = 0; // try again if connection state is the same
				dev_err(&mb_dev->pdev->dev, "%s: Debounce(same state) high\n", __func__);
			}
		} else {
			low++;
			high = 0;
			if (low == 20) {
				current_maya_base_conn_state = MAYA_BASE_CONNECTED;
				if (mb_dev->connection_state != current_maya_base_conn_state)
					break;
				low = 0; // try again if connection state is the same
				dev_err(&mb_dev->pdev->dev, "%s: Debounce(same state) low\n", __func__);
			}
		}
	}

	if (i >= 100) {
		dev_err(&mb_dev->pdev->dev, "%s: debounce time out.\n", __func__);
		enable_irq(mb_dev->det_irq);
		return IRQ_HANDLED;
	}
#ifdef CONFIG_MACH_MAYA
	if ( current_maya_base_conn_state == MAYA_BASE_CONNECTED)
		g_maya_base_state = 1;
	else
		g_maya_base_state = 0;
#endif

	enable_irq(mb_dev->det_irq);

	mutex_lock(&mb_dev->lock_det);
	dev_dbg(&mb_dev->pdev->dev, "%s start !! (l=%d, h=%d)\n", __func__, low, high);

	if (mb_dev->connection_state != current_maya_base_conn_state) {
		/* If undocked, we disable DOCK_EN immediately to prevent conflict with LED
		 * (Output) SOC_I2C_DOCK_EN# pin, High = disable, low = enable
		 */
		if (current_maya_base_conn_state == MAYA_BASE_DISCONNECTED)
			gpio_set_value(mb_dev->dock_en_gpio, 1);

		dev_dbg(&mb_dev->pdev->dev,
			 "%s change state !! low=%d, high=%d\n", __func__, low, high);
		/* Send wakeup key */
		dev_dbg(&mb_dev->pdev->dev,
			"%s send WAKEUP DROPPED key\n", __func__);
		input_report_key(mb_dev->input_dev, KEY_WAKEUP_DROPPED, (int)1);
		input_sync(mb_dev->input_dev);
		msleep(100);
		input_report_key(mb_dev->input_dev, KEY_WAKEUP_DROPPED, (int)0);
		input_sync(mb_dev->input_dev);

		maya_base_conn_change_handler(mb_dev,
						 current_maya_base_conn_state);
	} else {
		dev_err(&mb_dev->pdev->dev,
			"%s: fail to change the connection state.(same state)"
                        "connection_state=%d, current_maya_base_conn_state=%d\n"
			, __func__, mb_dev->connection_state, current_maya_base_conn_state);
	}

	dev_dbg(&mb_dev->pdev->dev, "%s finish !!\n", __func__);
	mutex_unlock(&mb_dev->lock_det);

	return IRQ_HANDLED;
}

/* lid switch interrupt handler */
static irqreturn_t lid_sw_irq_handler(int irq, void *dev_id)
{
	struct maya_base_dev *mb_dev = dev_id;
	enum lid_switch current_lid_state;
	int i;

	disable_irq_nosync(mb_dev->lid_irq);

	/* Debounce */
	for (i = 0; i < 100; i++) {
		msleep(3);
	}
	enable_irq(mb_dev->lid_irq);

	mutex_lock(&mb_dev->lock_lid);

	if(gpio_get_value(MAYA_HALL_INT_GPIO)){
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

	if (mb_dev->lid_sw == MB_LID_CLOSED) { /* deep sleep mode */
//		mb_dev->system_state = MB_SYS_DEEP_SLEEP;

		/* Remove TP driver to avoid palm event */
		if(!gpio_get_value(mb_dev->det_gpio)) {
			load_tp_driver(mb_dev, false);
		}

		/* set touchpad as deep sleep mode here when lid closed*/
		maya_base_set_touchpad_power_mode(mb_dev);
	} else {
//		mb_dev->system_state = MB_SYS_RUNNING;

		if(!gpio_get_value(mb_dev->det_gpio)) {
			load_tp_driver(mb_dev, true);
		}
	}
	mutex_unlock(&mb_dev->lock_lid);

	schedule_work(&mb_dev->work);

	return IRQ_HANDLED;
}

static int maya_base_irq_init(struct maya_base_dev *mb_dev)
{
	int ret = 0;
	unsigned long req_flags =
		 IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT;

	dev_dbg(&mb_dev->pdev->dev, "%s start !!\n", __func__);
	/* In dalmore, controlling this gpio pin may casue system unable to
	 * suspend/resume issue. initialize HALL_INT# pin
	 */
	ret = gpio_request(MAYA_HALL_INT_GPIO, MAYA_BASE_LID_SW_IRQ_NAME);
	if (ret < 0) {
		dev_err(&mb_dev->pdev->dev,
		 "%s: %s gpio_request failed (err=%d)\n", __func__,
					 MAYA_BASE_LID_SW_IRQ_NAME, ret);
		return ret;
	}

	ret = gpio_direction_input(MAYA_HALL_INT_GPIO);
	if (ret < 0) {
		dev_err(&mb_dev->pdev->dev,
		 "%s: HALL_INT gpio_direction_output failed (err=%d)\n",
							 __func__, ret);
		gpio_free(MAYA_HALL_INT_GPIO);
		return ret;
	}

	ret = mb_dev->lid_irq = gpio_to_irq(MAYA_HALL_INT_GPIO);
	if (ret < 0) {
		dev_err(&mb_dev->pdev->dev,
		 "%s: gpio_to_irq() failed (err=%d)\n", __func__, ret);
		return ret;
	}

	ret = request_threaded_irq(mb_dev->lid_irq, NULL, lid_sw_irq_handler,
			req_flags, MAYA_BASE_LID_SW_IRQ_NAME, mb_dev);
	if (ret < 0) {
		dev_err(&mb_dev->pdev->dev,
		 "%s: request_irq() failed (err=%d)\n", __func__, ret);
		return ret;
	}

	/* get irq for dock detection */
	ret = gpio_request(mb_dev->det_gpio, MAYA_BASE_DET_GPIO_NAME);
	if (ret < 0) {
		dev_err(&mb_dev->pdev->dev,
		 "%s: %s gpio_request failed (err=%d)\n", __func__,
					 MAYA_BASE_DET_GPIO_NAME, ret);
		return ret;
	}

	ret = gpio_direction_input(mb_dev->det_gpio);
	if (ret < 0) {
		dev_err(&mb_dev->pdev->dev,
		 "%s: Dock_det gpio_direction_output failed (err=%d)\n",
							 __func__, ret);
		gpio_free(mb_dev->det_gpio);
		return ret;
	}

	ret = mb_dev->det_irq = gpio_to_irq(mb_dev->det_gpio);
	if (ret < 0) {
		dev_err(&mb_dev->pdev->dev,
		 "%s: gpio_to_irq() failed (err=%d)\n", __func__, ret);
		return ret;
	}

	ret = request_threaded_irq(mb_dev->det_irq, NULL, base_detect_handler,
			req_flags, MAYA_BASE_DET_GPIO_NAME, mb_dev);
	if (ret < 0) {
		dev_err(&mb_dev->pdev->dev,
		 "%s: request_irq() failed (err=%d)\n", __func__, ret);
		return ret;
	}

	/* initialize SOC_I2C_DOCK_EN# pin */
	ret = gpio_request(mb_dev->dock_en_gpio, MAYA_BASE_I2C_EN_GPIO_NAME);
	if (ret < 0) {
		dev_err(&mb_dev->pdev->dev, "%s: %s gpio_request failed %d\n",
				 __func__, MAYA_BASE_I2C_EN_GPIO_NAME, ret);
		return ret;
	}

	ret = gpio_direction_output(mb_dev->dock_en_gpio, 1);
        if (ret < 0) {
		dev_err(&mb_dev->pdev->dev,
		 "%s: DOCK_EN gpio_direction_output failed %d\n", __func__, ret);
		gpio_free(mb_dev->dock_en_gpio);
		return ret;
	}

	/* initialize EC reset pin */
	ret = gpio_request(MAYA_EC_RST_GPIO, MAYA_BASE_EC_RST_NAME);
	if (ret < 0) {
		dev_err(&mb_dev->pdev->dev, "%s: %s gpio_request failed %d\n",
				 __func__, MAYA_BASE_EC_RST_NAME, ret);
		return ret;
	}

	ret = gpio_direction_output(MAYA_EC_RST_GPIO, 0);
        if (ret < 0) {
		dev_err(&mb_dev->pdev->dev,
		 "%s: EC_reset gpio_direction_output failed %d\n", __func__, ret);
		gpio_free(MAYA_EC_RST_GPIO);
		return ret;
	}

	return ret;
}

static int maya_base_input_dev_init(struct maya_base_dev *mb_dev)
{
	int err = 0;
	dev_dbg(&mb_dev->pdev->dev, "%s start !!\n", __func__);

	mb_dev->input_dev = input_allocate_device();
	if (!mb_dev->input_dev) {
		dev_err(&mb_dev->pdev->dev,
		 "%s: input_allocate_device() failed.\n", __func__);
		return -ENOMEM;
	}

	mb_dev->input_dev->name = "Maya Base Lid Switch";
	mb_dev->input_dev->phys = "maya_base/gpio/input0";
	mb_dev->input_dev->id.bustype = BUS_HOST;
	mb_dev->input_dev->id.vendor = 0x0001;
	mb_dev->input_dev->id.product = 0x0001;
	mb_dev->input_dev->dev.parent = &mb_dev->pdev->dev;

	set_bit(EV_SW, mb_dev->input_dev->evbit);
	set_bit(EV_KEY, mb_dev->input_dev->evbit);
	set_bit(SW_LID, mb_dev->input_dev->swbit);
	set_bit(KEY_RFKILL, mb_dev->input_dev->keybit);
	set_bit(KEY_EC_UPDATE_NOTICE, mb_dev->input_dev->keybit);
	set_bit(KEY_WAKEUP_DROPPED, mb_dev->input_dev->keybit);

	err = input_register_device(mb_dev->input_dev);
	if (err) {
		dev_err(&mb_dev->pdev->dev,
			 "%s: Failed to register input device\n", __func__);
		input_free_device(mb_dev->input_dev);
		return err;
	}

	return 0;
}

static int board_id_init(struct maya_base_dev *mb_dev)
{
	int ret;
	dev_dbg(&mb_dev->pdev->dev, "%s start !!\n", __func__);

	ret = gpio_request(MAYA_PV_ID_GPIO, MAYA_BASE_BOARD_ID_NAME);
	if (ret < 0) {
		dev_err(&mb_dev->pdev->dev, "%s: %s gpio_request failed %d\n",
				 __func__, MAYA_BASE_BOARD_ID_NAME, ret);
		return ret;
	}

	ret = gpio_direction_input(MAYA_PV_ID_GPIO);
	if (ret < 0) {
		dev_err(&mb_dev->pdev->dev,
		 "%s: board_id gpio_direction_intput failed (err=%d)\n",
							 __func__, ret);
		gpio_free(MAYA_PV_ID_GPIO);
		return ret;
	}

	mb_dev->board_id = gpio_get_value(MAYA_PV_ID_GPIO);

	if (mb_dev->board_id == 0) {
		dev_dbg(&mb_dev->pdev->dev, "%s this is a PV build.\n", __func__);
		mb_dev->det_gpio = MAYA_PV_SOC_DOCK_DET_GPIO;
	} else {
		dev_dbg(&mb_dev->pdev->dev, "%s this is a SI build.\n", __func__);
		mb_dev->det_gpio = MAYA_SOC_DOCK_DET_GPIO;
	}
	mb_dev->dock_en_gpio = MAYA_SOC_I2C_DOCK_EN_GPIO;

	return 0;
}

static int maya_base_driver_probe(struct platform_device *pdev)
{
	int ret =  0;
	struct maya_base_dev *mb_dev;

	dev_dbg(&pdev->dev, "%s start !!\n", __func__);

	mb_dev = kzalloc(sizeof(struct maya_base_dev), GFP_KERNEL);
	if (mb_dev == NULL) {
		dev_err(&pdev->dev,
			 "%s kzalloc failed (Out of memory)\n", __func__);
		ret = -ENOMEM;
		goto exit;
	}

	dev_set_drvdata(&pdev->dev, mb_dev);
	mutex_init(&mb_dev->lock);
	mutex_init(&mb_dev->lock_det);
	mutex_init(&mb_dev->lock_lid);
	mutex_init(&mb_dev->lock_sync);
	INIT_WORK(&mb_dev->work, sync_base_status_task);
	mb_dev->pdev = pdev;
	mb_dev->system_state = MB_SYS_RUNNING;
	mb_dev->connection_state = MAYA_BASE_DISCONNECTED;
	mb_dev->ec_version_valid = MB_EC_VER_NOT_READ;
	mb_dev->fw.state = MB_FW_NOT_READ;
	mb_dev->lid_sw = MB_LID_OPENED;
	mb_dev->fw.name = EC_FW_NAME;
	mb_dev->need_notify_os = 1;
	mb_dev->lid_change = 0;
	mb_dev->mute_led_blinking_freq = 0x00;
	mb_dev->caps_lock_led_blinking_freq = 0x00;
	mb_dev->attach_status = ATTACH_STATUS_FAILURE;
	mb_dev->do_update_fw = 0;
	mb_dev->update_success = 0;
	mb_dev->io373_acpi_dev = NULL;
	mb_dev->base_i2c_client = NULL;
	mb_dev->touchpad_i2c_client = NULL;
	mb_dev->tp_loaded = false;
	strcpy(mb_dev->ec_fw_version, "NONE");

	board_id_init(mb_dev);

	device_init_wakeup(&mb_dev->pdev->dev, true);

	ret = maya_base_input_dev_init(mb_dev);
	if (ret) {
		dev_err(&pdev->dev,
			 "%s maya_base_input_dev_init() failed !!\n", __func__);
		goto exit;
	}

	ret = maya_base_irq_init(mb_dev);
	if (ret < 0) {
		dev_err(&pdev->dev,
			 "%s maya_base_irq_init() failed !!\n", __func__);
		goto exit;
	}

	ret = maya_char_dev_init(mb_dev);
	if (ret < 0) {
		dev_err(&pdev->dev,
			 "%s maya_char_dev_init() failed !!\n", __func__);
		goto exit;
	}

	/* get a i2c adapter for new i2c usage */
	mb_dev->adapter = i2c_get_adapter(MAYA_BASE_I2C_BUS);
	if (!mb_dev->adapter) {
		dev_err(&pdev->dev,
		 "can't get base adpater (bus = %d)\n", MAYA_BASE_I2C_BUS);
                ret = -EBUSY;
                goto exit;
        }

	ret = maya_create_dev_attrs(mb_dev);
	if (ret < 0) {
		goto exit;
	}

	ret = get_ec_fw_file_version(mb_dev);
	if (ret < 0) {
		dev_err(&pdev->dev,
			 "%s get_ec_fw_file_version() failed !!\n", __func__);
		return ret;
	}

#ifdef CONFIG_SWITCH
	mb_dev->dock_sw.name = "dock";
	ret = switch_dev_register(&mb_dev->dock_sw);
	if (ret != 0) {
		dev_err(&pdev->dev, "%s switch_dev_register() failed !!\n", __func__);
		goto exit;
	}
#endif //CONFIG_SWITCH

	/* check if base is connected */
	if (!gpio_get_value(mb_dev->det_gpio)) {
		g_maya_base_state = 1;
		maya_base_conn_change_handler(mb_dev, MAYA_BASE_CONNECTED);
	} else {
		g_maya_base_state = 0;
	}

	dev_dbg(&pdev->dev, "%s finished\n", __func__);
exit:
	return ret;
}

static int maya_base_driver_remove(struct platform_device *pdev)
{
	struct maya_base_dev *mb_dev = dev_get_drvdata(&pdev->dev);
	dev_dbg(&pdev->dev, "%s start !!\n", __func__);

	maya_base_release_firmware(mb_dev->fw.fw);

	if (mb_dev->connection_state == MAYA_BASE_CONNECTED)
		maya_base_conn_change_handler(mb_dev, MAYA_BASE_DISCONNECTED);

	maya_char_dev_exit(mb_dev);
	free_irq(mb_dev->lid_irq, mb_dev);
	free_irq(mb_dev->det_irq, mb_dev);

	gpio_free(mb_dev->det_gpio);
	gpio_free(mb_dev->dock_en_gpio);
	gpio_free(MAYA_HALL_INT_GPIO);

#ifdef CONFIG_SWITCH
	switch_dev_unregister(&mb_dev->dock_sw);
#endif //CONFIG_SWITCH
	input_unregister_device(mb_dev->input_dev);
//	kfree(mb_dev); // freed by free_irq()
	return 0;
}

static void maya_base_driver_shutdown(struct platform_device *pdev)
{
	struct maya_base_dev *mb_dev = dev_get_drvdata(&pdev->dev);
	dev_dbg(&pdev->dev, "%s start !!\n", __func__);

	mb_dev->system_state = MB_SYS_SHUT_DOWN;
	schedule_work(&mb_dev->work);
	return;
}

#ifdef CONFIG_PM
static int maya_base_driver_suspend(struct device *dev)
{
	struct maya_base_dev *mb_dev = dev_get_drvdata(dev);
	int ret;

	dev_dbg(dev, "%s start !!!\n", __func__);

	if (mb_dev->board_id == 1)
		base_detect_handler(mb_dev->det_irq, mb_dev);

	/* set irq can wake up kernel */
	if (device_may_wakeup(&mb_dev->pdev->dev)) {
		ret = enable_irq_wake(mb_dev->lid_irq);
		dev_dbg(dev, "%s enable_irq_wake(lid_irq) ret=%d", __func__, ret);

		if (mb_dev->board_id == 0) {
			ret = enable_irq_wake(mb_dev->det_irq);
			dev_dbg(dev, "%s enable_irq_wake(det_irq) ret=%d", __func__, ret);
		}
	}

	mb_dev->system_state = MB_SYS_SUSPEND;
	sync_base_status_task(&mb_dev->work);
	return 0;
}

static int maya_base_driver_resume(struct device *dev)
{
	struct maya_base_dev *mb_dev = dev_get_drvdata(dev);

	dev_dbg(dev, "%s start !!\n", __func__);

	mb_dev->system_state = MB_SYS_RUNNING;
	sync_base_status_task(&mb_dev->work);

	if (mb_dev->board_id == 1)
		base_detect_handler(mb_dev->det_irq, mb_dev);

	if (device_may_wakeup(&mb_dev->pdev->dev)) {
		disable_irq_wake(mb_dev->lid_irq);
		if (mb_dev->board_id == 0)
			disable_irq_wake(mb_dev->det_irq);
	}

	msleep(10);
	if (tegra_dc_hpd(g_maya_dc)) {
		dev_dbg(dev, "%s hdmi is connected. delay 1100ms\n", __func__);
		msleep(1100); // wait for EC to turn on MAINON power
	}
	return 0;
}

static const struct dev_pm_ops maya_base_dev_pm_ops = {
	.suspend = maya_base_driver_suspend,
	.resume = maya_base_driver_resume,
};
#endif

static struct platform_driver maya_base_driver = {
	.probe = maya_base_driver_probe,
	.remove = maya_base_driver_remove,
	.shutdown = maya_base_driver_shutdown,
	.driver = {
		.name = "maya_base",
#ifdef CONFIG_PM
		.pm   = &maya_base_dev_pm_ops,
#endif
	},
};

static int __init maya_base_init(void)
{
	int ret =  0;

	printk(KERN_INFO "%s start !!\n", __func__);

	ret = platform_driver_register(&maya_base_driver);
	if (ret < 0) {
		printk(KERN_ERR
			 "%s: platform_driver_register() failed (err=%d)\n",
							 __func__, ret);
	}

	return ret;
}

static void __exit maya_base_exit(void)
{
	printk(KERN_INFO "%s start !!\n", __func__);

	platform_driver_unregister(&maya_base_driver);
}

module_init(maya_base_init);
module_exit(maya_base_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jonathan Lin <jonathan.lin@insyde.com>");
MODULE_DESCRIPTION("Maya base Controller driver");
