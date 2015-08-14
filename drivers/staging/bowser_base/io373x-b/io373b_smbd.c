/*
 * IO373X-B SMBD (I2C) driver.
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>
#include "io373b.h"
#include "Ec.h"
#include "EcFlash.h"

#define CMD_SET_ADR             0x00
#define CMD_READ_BYTE           0x81
#define CMD_READ_WORD           0x82
#define CMD_READ_BLOCK          0x80
#define CMD_WRITE_BYTE          0x01
#define CMD_WRITE_WORD          0x02
#define CMD_WRITE_BLOCK         0x03
#define CMD_READ_IRQ_PF         0xD1

#define MAX_SUBDEV_COUNT        4

enum XFER { XFER_READ, XFER_WRITE };

struct conn_io373b_smbd
{
	struct Conn _conn; /* the "base" class obj holding the "virtual functions" */
	struct io373b *io373b;
};

struct io373b {
	struct i2c_client *client;
	struct mutex lock;
	struct blocking_notifier_head notifier_list;
	int num_subdevs;
	struct i2c_client *dummies[MAX_SUBDEV_COUNT];
	struct platform_device *pdevs[MAX_SUBDEV_COUNT];
	bool dev_attrs_created;
	bool irq_requested;
	bool irq_wake_enabled;
	struct conn_io373b_smbd conn;
	struct reg_ua /* user access */
	{
		unsigned short idx;
		int len;
	} reg_ua;
	struct vdev_ua /* user access */
	{
		unsigned short addr; /* i2c addr */
		unsigned char cmd;
		int len;
	} vdev_ua;
};

int io373b_register_notifier(struct device *io373b_dev, struct notifier_block *nb)
{
	struct io373b *io373b = dev_get_drvdata(io373b_dev);
	return blocking_notifier_chain_register(&io373b->notifier_list, nb);
}
EXPORT_SYMBOL(io373b_register_notifier);

int io373b_unregister_notifier(struct device *io373b_dev, struct notifier_block *nb)
{
	struct io373b *io373b = dev_get_drvdata(io373b_dev);
	return blocking_notifier_chain_unregister(&io373b->notifier_list, nb);
}
EXPORT_SYMBOL(io373b_unregister_notifier);

static int io373b_i2c_xfer(struct io373b *io373b, unsigned short addr, unsigned char cmd, unsigned char *buf, int byte_cnt, enum XFER x)
{
	struct i2c_msg msg[2] = {};
	int n_msg = 1, ret;
	unsigned char tmp_buf[1 + 256]; /* cmd + max-256-data */

	if (byte_cnt > 256)
		return -EINVAL;

	msg[0].addr = addr;

	if (x == XFER_WRITE) {
		tmp_buf[0] = cmd;
		memcpy(tmp_buf + 1, buf, byte_cnt);
		msg[0].buf = tmp_buf;
		msg[0].len = 1 + byte_cnt;
	} else {
		msg[0].buf = &cmd;
		msg[0].len = 1;

		msg[1].addr = addr;
		msg[1].buf = buf;
		msg[1].len = byte_cnt;
		msg[1].flags = I2C_M_RD;

		n_msg = 2;
	}

	ret = i2c_transfer(io373b->client->adapter, msg, n_msg);

	if (ret == n_msg)
		return 0;
	else {
		if (ret >= 0) /* xfer ok but not all msg done */
			ret = -EIO;
		dev_err(&io373b->client->dev, "%s %s failed(%d), addr %04x, cmd %02x, len %d\n",
			__func__, (x == XFER_WRITE) ? "write" : "read", ret, addr, cmd, byte_cnt);
		return ret;
	}
}

static int io373b_set_reg_index(struct io373b *io373b, unsigned short reg)
{
	unsigned char buf[2];

	buf[0] = reg >> 8; /* reg addr hi first */
	buf[1] = (unsigned char) reg;
	return io373b_i2c_xfer(io373b, io373b->client->addr, CMD_SET_ADR, buf, 2, XFER_WRITE);
}

static int io373b_read_reg_1byte(struct io373b *io373b, unsigned char *p1byte)
{
	return 	io373b_i2c_xfer(io373b, io373b->client->addr, CMD_READ_BYTE, p1byte, 1, XFER_READ);
}

static int io373b_read_reg_2byte(struct io373b *io373b, unsigned char *p2bytes)
{
	return 	io373b_i2c_xfer(io373b, io373b->client->addr, CMD_READ_WORD, p2bytes, 2, XFER_READ);
}

int io373b_read_regs(struct device *io373b_dev, unsigned short start_reg, unsigned char *buf, int byte_cnt)
{
	struct io373b *io373b = dev_get_drvdata(io373b_dev);
	unsigned char cmd;
	int data_this_read;
	int data_left = byte_cnt;
	unsigned char tmp_buf[1 + 32]; /* CNT+max_32_bytes_data */
	int ret = 0;

	mutex_lock(&io373b->lock);

	ret = io373b_set_reg_index(io373b, start_reg);
	if (ret < 0)
		goto out;

	while ((ret >= 0) && data_left) {
		if (data_left == 1) {
			ret = io373b_read_reg_1byte(io373b, buf);
			data_left = 0;
		} else if (data_left == 2) {
			ret = io373b_read_reg_2byte(io373b, buf);
			data_left = 0;
		} else {
			data_this_read = min(32, data_left);
			/* cmd=0x80 | CNT, where CNT=0: 32 byte DATA, CNT=3~31: 3~31 byte DATA */
			cmd = CMD_READ_BLOCK | (data_this_read & 0x1F);

			/* +1 for CNT */
			ret = io373b_i2c_xfer(io373b, io373b->client->addr, cmd, tmp_buf, data_this_read + 1, XFER_READ);
			if (ret >= 0) {
				memcpy(buf, tmp_buf + 1, data_this_read);
				data_left -= data_this_read;
				buf += data_this_read;
			}
		}
	}

out:
	mutex_unlock(&io373b->lock);

	return ret;
}
EXPORT_SYMBOL(io373b_read_regs);

int io373b_write_regs(struct device *io373b_dev, unsigned short start_reg, unsigned char *buf, int byte_cnt)
{
	struct io373b *io373b = dev_get_drvdata(io373b_dev);
	int data_this_write;
	int data_left = byte_cnt;
	int ret;
	unsigned char tmp_buf[1 + 32]; /* CNT+max_32_bytes_data */

	mutex_lock(&io373b->lock);

	ret = io373b_set_reg_index(io373b, start_reg);
	if (ret < 0)
		goto out;

	while (data_left) {
		data_this_write = min(32, data_left); /* we can do block write for max 32 bytes */
		tmp_buf[0] = data_this_write;
		memcpy(tmp_buf + 1, buf, data_this_write);

		ret = io373b_i2c_xfer(io373b, io373b->client->addr, CMD_WRITE_BLOCK, tmp_buf, data_this_write + 1, XFER_WRITE);
		if (ret < 0)
			goto out;

		data_left -= data_this_write;
		buf += data_this_write;
	}

out:
	mutex_unlock(&io373b->lock);

	return ret;
}
EXPORT_SYMBOL(io373b_write_regs);

struct device *io373b_get_subdev(struct device *io373b_dev, char *name)
{
	struct io373b *io373b = dev_get_drvdata(io373b_dev);
	int i = 0;

	for (i = 0; i < io373b->num_subdevs; i++) {
		if (strcmp(io373b->pdevs[i]->name, name) == 0)
			return &io373b->pdevs[i]->dev;
	}
	return NULL;
}
EXPORT_SYMBOL(io373b_get_subdev);

int io373b_read_subdev(struct device *io373b_dev, unsigned short addr, unsigned char cmd, unsigned char *buf, int byte_cnt)
{
	struct io373b *io373b = dev_get_drvdata(io373b_dev);
	return io373b_i2c_xfer(io373b, addr, cmd, buf, byte_cnt, XFER_READ);
}
EXPORT_SYMBOL(io373b_read_subdev);

int io373b_write_subdev(struct device *io373b_dev, unsigned short addr, unsigned char cmd, unsigned char *buf, int byte_cnt)
{
	struct io373b *io373b = dev_get_drvdata(io373b_dev);
	return io373b_i2c_xfer(io373b, addr, cmd, buf, byte_cnt, XFER_WRITE);
}
EXPORT_SYMBOL(io373b_write_subdev);

int io373b_subdev_read(struct device *subdev, unsigned char cmd, unsigned char *buf, int byte_cnt)
{
	struct io373b *io373b = dev_get_drvdata(subdev->parent);
	struct i2c_client *dummy = io373b->dummies[to_platform_device(subdev)->id];

	return io373b_i2c_xfer(io373b, dummy->addr, cmd, buf, byte_cnt, XFER_READ);
}
EXPORT_SYMBOL(io373b_subdev_read);

int io373b_subdev_write(struct device *subdev, unsigned char cmd, unsigned char *buf, int byte_cnt)
{
	struct io373b *io373b = dev_get_drvdata(subdev->parent);
	struct i2c_client *dummy = io373b->dummies[to_platform_device(subdev)->id];

	return io373b_i2c_xfer(io373b, dummy->addr, cmd, buf, byte_cnt, XFER_WRITE);
}
EXPORT_SYMBOL(io373b_subdev_write);

static bool conn_read_regs(struct conn_io373b_smbd *conn, unsigned short start_reg, unsigned char *pbytes, int n_bytes)
			{ return io373b_read_regs(&conn->io373b->client->dev, start_reg, pbytes, n_bytes) >= 0; }
static bool conn_write_regs(struct conn_io373b_smbd *conn, unsigned short start_reg, unsigned char *pbytes, int n_bytes)
			{ return io373b_write_regs(&conn->io373b->client->dev, start_reg, pbytes, n_bytes) >= 0; }
static bool conn_enter_code_in_ram(struct conn_io373b_smbd *conn) { return true; }
static bool conn_exit_code_in_ram(struct conn_io373b_smbd *conn) { return true; }
static void conn_dtor(struct conn_io373b_smbd *conn) { _Conn_Dtor((Conn *) conn); /* call base dtor */ }

/* exported for Conn.c to call */
struct conn_io373b_smbd *conn_io373b_smbd_ctor(struct io373b *io373b)
{
	struct conn_io373b_smbd *conn = &io373b->conn;

	/* call base ctor */
	if(_Conn_Ctor((Conn *) conn) == NULL)
		goto error_exit;

	conn->io373b = io373b;
	conn->_conn.ReadRegs        = (PFN_CONN_READ_REGS)         conn_read_regs;
	conn->_conn.WriteRegs       = (PFN_CONN_WRITE_REGS)        conn_write_regs;
	conn->_conn.Dtor            = (PFN_CONN_DTOR)              conn_dtor;
	conn->_conn.EnterCodeInRam  = (PFN_CONN_ENTER_CODE_IN_RAM) conn_enter_code_in_ram;
	conn->_conn.ExitCodeInRam   = (PFN_CONN_EXIT_CODE_IN_RAM)  conn_exit_code_in_ram;

	return conn;

error_exit:
	if(conn)
		conn_dtor(conn);

	return NULL;
}

static int enable_wdt_led(struct device *io373b_dev)
{
	unsigned char data;

	// Enable 15 second WDT
	if (io373b_write_reg(io373b_dev, 0xFE80, 0x81) || // disable WDT
	    io373b_write_reg(io373b_dev, 0xFE83, 0x50) || // WDT low 6-bit counter
	    io373b_write_reg(io373b_dev, 0xFE82, 0x07) || // WDT high 8-bit counter
	    io373b_write_reg(io373b_dev, 0xFE80, 0x83))   // enable WDT 15 second
		return -EIO;

	// Turn on MAINON power
	if (io373b_read_reg(io373b_dev, 0xFC23, &data)        ||
	    io373b_write_reg(io373b_dev, 0xFC23, data | 0x08) ||
	    io373b_read_reg(io373b_dev, 0xFC13, &data)        ||
	    io373b_write_reg(io373b_dev, 0xFC13, data | 0x08))
		return -EIO;

	// Enable WDT LED
	if (io373b_read_reg(io373b_dev, 0xF028, &data)        ||
	    io373b_write_reg(io373b_dev, 0xF028, data | 0x08) ||
	    io373b_write_reg(io373b_dev, 0xFE89, 0x09)        ||
	    io373b_read_reg(io373b_dev, 0xFC00, &data)        ||
	    io373b_write_reg(io373b_dev, 0xFC00, data | 0x80))
		return -EIO;

	return 0;
}

int io373b_update_flash(struct device *io373b_dev, const u8 *data, size_t size)
{
	struct io373b *io373b = dev_get_drvdata(io373b_dev);
	int err = 0;
	struct ConnParam param;
	struct Ec *ec = NULL;
	struct EcFlash *ec_flash = NULL;
	//unsigned char code_sel = 0;
	//unsigned long timeout;
	bool reenable_irq = false;

	if (io373b->irq_requested) {
		disable_irq(io373b->client->irq);
		reenable_irq = true;
	}

	param.res.p = io373b;
	if (!(ec = EcInit(CONN_IO373B_SMBD, param))) {
		err = -EIO;
		goto out;
	}

	if (size > EcGetEbdFlashSize(ec)) {
		dev_err(&io373b->client->dev, "bin file size %d > ebd flash size %ld !\n", size, EcGetEbdFlashSize(ec));
		err = -EINVAL;
		goto out;
	}

	if (!(ec_flash = EcFlashInit(ec, NULL))) {
		dev_err(&io373b->client->dev, "Failed to init ebd flash\n");
		err = -EIO;
		goto out;
	}

	// Do this after EcFlashInit(), which disabled wdt.
	if ((err = enable_wdt_led(io373b_dev)) != 0) {
		dev_err(&io373b->client->dev, "Failed to enable wdt led\n");
		goto out;
	}

	if (!EcFlashChipErase(ec_flash)) {
		dev_err(&io373b->client->dev, "Failed to erase flash\n");
		err = -EIO;
		goto out;
	}

	if (!EcFlashWrite(ec_flash, 0, (unsigned char *) data, size)) {
		dev_err(&io373b->client->dev, "Failed to program flash\n");
		err = -EIO;
		goto out;
	}
/* For projects that slave addr 60 --> 62 after flash-update, the checking below gets NACK. So exits without checking.
	EcFlashExit(ec_flash); // back to flash if every thing fine
	ec_flash = NULL;

	timeout = jiffies + msecs_to_jiffies(3000);
	while (1) {
		err = io373b_read_regs(&io373b->client->dev, 0xF011, &code_sel, 1);
		if (err < 0)
			goto out;
		if (code_sel & 1)
			break;
		if (time_after(jiffies, timeout)) {
			dev_err(&io373b->client->dev, "Failed to verify flash\n");
			err = -EIO;
			goto out;
		}
		msleep(100);
	}
*/
	err = 0;
out:
	if (ec_flash)
		EcFlashExit(ec_flash);
	if (ec)
		EcExit(ec);
	if (reenable_irq)
		enable_irq(io373b->client->irq);

	return err;
}
EXPORT_SYMBOL(io373b_update_flash);

static unsigned short name_to_addr(struct io373b *io373b, char *name)
{
	int i = 0;
	char *endptr;
	unsigned long addr;

	addr = simple_strtoul(name, &endptr, 16);
	if (*endptr == '\0') { /* a value */
		if (addr == io373b->client->addr) /* myself */
			return addr;
		for (i = 0; i < io373b->num_subdevs; i++) {
			if (addr == io373b->dummies[i]->addr) /* one of subdevs */
				return addr;
		}
		dev_err(&io373b->client->dev, "No subdevs with addr 0x%lx\n", addr);
		return 0; /* fail it */
	}

	/* A string */

	if (strlen(name) < 3) { /* "smbd", "acpi", "kbd" */
		dev_err(&io373b->client->dev, "Bad subdev name \"%s\"\n", name);
		return 0;
	}

	if (strcmp(name, "smbd") == 0)
		return io373b->client->addr;

	for (i = 0; i < io373b->num_subdevs; i++) {
		if (strstr(io373b->pdevs[i]->name, name))
			return io373b->dummies[i]->addr;
	}

	dev_err(&io373b->client->dev, "No subdev names contain \"%s\"\n", name);
	return 0;
}

static int user_update_flash(struct device *dev, struct device_attribute *attr,
                            const char *buf, size_t count)
{
	struct io373b *io373b = dev_get_drvdata(dev);
	int err = 0;
	const struct firmware *fw = NULL;
	char *fw_name = NULL;

	fw_name = kzalloc(count + 1, GFP_KERNEL);
	if (!fw_name) {
		err = -ENOMEM;
		goto out;
	}

	if (sscanf(buf, "%s", fw_name) != 1) {
		err = -EINVAL;
		goto out;
	}

	err = request_firmware(&fw, fw_name, dev);
	if (err) {
		dev_err(dev, "Failed to load %s\n", fw_name);
		goto out;
	}

	err = io373b_update_flash(&io373b->client->dev, fw->data, fw->size);
	if (err)
		goto out;

	err = count;
out:
	if (fw)
		release_firmware(fw);
	if (fw_name)
		kfree(fw_name);

	return err;
}
static DEVICE_ATTR(update_flash, S_IWUSR | S_IRUGO, NULL, user_update_flash);

/*
 Under /sys/devices/i2c-x/x-0060, where 60 is i2c addr for smbd,
 echo 8000 3 > reg, cat reg (read 3 bytes from index 0x8000)
 echo 8000 3 byte0 byte1 byte2 > reg (write 3 bytes to index 0x8000)
*/
static ssize_t user_reg_write(struct device *dev, struct device_attribute *attr,
                              const char *buf, size_t count)
{
	struct io373b *io373b = dev_get_drvdata(dev);
	unsigned long idx;
	int len, n_field, ret, pos, write_len = 0;
	unsigned char data[256];

	io373b->reg_ua.len = 0; /* invalidate len */

	n_field = sscanf(buf, "%lx %d%n", &idx, &len, &pos);
	if (n_field != 2)
		return -EINVAL;
	if (idx > 0xFFFF)
		return -EINVAL;
	if (len <= 0 || len > 256)
		return -EINVAL;

	io373b->reg_ua.idx = idx;
	io373b->reg_ua.len = len;

	while (pos < count)
	{
		unsigned long val;
		int n_scanned;
		int n_field;

		n_field = sscanf(buf + pos, "%lx%n", &val, &n_scanned);
		if (n_field != 1)
			break;
		if (val > 0xFF)
			return -EINVAL;

		pos += n_scanned;
		data[write_len++] = (unsigned char) val;

		if (write_len > len)
			return -EINVAL;
	}

	if (write_len) {
		if (write_len != len)
			return -EINVAL;
		ret = io373b_write_regs(dev, idx, data, write_len);
		if (ret < 0)
			return ret;
	}

	return count;
}

static ssize_t user_reg_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct io373b *io373b = dev_get_drvdata(dev);
	int n_char = 0, i, ret;
	unsigned char data[256];

	if (io373b->reg_ua.idx > 0xFFFF || io373b->reg_ua.len <= 0 || io373b->reg_ua.len > 256) {
		return sprintf(buf,
				"Usage:\n"
				"  echo 8000 3 > reg, cat reg (read 3 bytes from 0x8000)\n"
				"  echo 8000 3 byte0 byte1 byte2 > reg (write 3 bytes to 0x8000)\n");
	}

	ret = io373b_read_regs(dev, io373b->reg_ua.idx, data, io373b->reg_ua.len);
	if (ret < 0)
		return ret;

	for (i = 0; i < io373b->reg_ua.len; i++) {
		/*if (i % 8 == 0 && i != 0) {
			if (i % 16 == 0)
				n_char += sprintf(buf + n_char, "\n");
			else
				n_char += sprintf(buf + n_char, "- ");
		}*/
		n_char += sprintf(buf + n_char, "%02x ", data[i]);
	}

	n_char--; /* discard the final blank */
	n_char += sprintf(buf + n_char, "\n");

	return n_char;
}
static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO, user_reg_read, user_reg_write);

/*
 Under /sys/devices/i2c-x/x-0060, where 60 is i2c addr for smbd,
 to read from virtual device:
   echo name_or_addr cmd len > vdev
   cat vdev
 to write to virtual device:
   echo name_or_addr cmd len data[0] data[1] ... data[len-1] > vdev
 where name_or_addr = smbd, acpi, kbd, (part of dev name), or
   7 bit slave addr in hex, e.g, 60, b, etc.
   len is in decimal.
*/
static ssize_t user_vdev_write(struct device *dev, struct device_attribute *attr,
                               const char *buf, size_t count)
{
	struct io373b *io373b = dev_get_drvdata(dev);
	unsigned long cmd;
	int len, n_field, ret, pos, write_len = 0;
	unsigned char data[256];
	char vdev_name[10];
	unsigned short addr;

	io373b->vdev_ua.len = 0; /* invalidate len */

	n_field = sscanf(buf, "%s %lx %d%n", vdev_name, &cmd, &len, &pos);
	if (n_field != 3)
		return -EINVAL;
	if (!(addr = name_to_addr(io373b, vdev_name)))
		return -EINVAL;
	if (cmd > 0xFF)
		return -EINVAL;
	if (len <= 0 || len > sizeof(data))
		return -EINVAL;

	io373b->vdev_ua.addr = addr;
	io373b->vdev_ua.cmd = cmd;
	io373b->vdev_ua.len = len;

	while (pos < count)
	{
		unsigned long val;
		int n_scanned;
		int n_field;

		n_field = sscanf(buf + pos, "%lx%n", &val, &n_scanned);
		if (n_field != 1)
			break;
		if (val > 0xFF)
			return -EINVAL;

		pos += n_scanned;
		data[write_len++] = (unsigned char) val;

		if (write_len > len)
			return -EINVAL;
	}

	if (write_len) {
		if (write_len != len)
			return -EINVAL;
		ret = io373b_i2c_xfer(io373b, addr, cmd, data, write_len, XFER_WRITE);
		if (ret < 0)
			return ret;
	}

	return count;
}

static ssize_t user_vdev_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct io373b *io373b = dev_get_drvdata(dev);
	int n_char = 0, i, ret;
	unsigned char data[256];

	if (!io373b->vdev_ua.addr || io373b->vdev_ua.cmd > 0xFF || io373b->vdev_ua.len <= 0 || io373b->vdev_ua.len > sizeof(data)) {
		return sprintf(buf,
				"Usage:\n"
				"  echo smbd|acpi|kbd|b d1 2 > vdev, cat vdev\n"
				"    write cmd d1, and read 2 bytes from smbd or acpi... or addr b\n"
				"  echo smbd|acpi|kbd|b d1 2 byte0 byte1 > vdev\n"
				"    write cmd d1 and 2 bytes to smbd or acpi... or addr b\n");
	}

	ret = io373b_i2c_xfer(io373b, io373b->vdev_ua.addr, io373b->vdev_ua.cmd, data, io373b->vdev_ua.len, XFER_READ);
	if (ret < 0)
		return ret;

	for (i = 0; i < io373b->vdev_ua.len; i++) {
		/*if (i % 8 == 0 && i != 0) {
			if (i % 16 == 0)
				n_char += sprintf(buf + n_char, "\n");
			else
				n_char += sprintf(buf + n_char, "- ");
		}*/
		n_char += sprintf(buf + n_char, "%02x ", data[i]);
	}

	n_char--; /* discard the final blank */
	n_char += sprintf(buf + n_char, "\n");

	return n_char;
}
static DEVICE_ATTR(vdev, S_IWUSR | S_IRUGO, user_vdev_read, user_vdev_write);

static int io373b_create_dev_attrs(struct io373b *io373b)
{
	int err;

	if ((err = device_create_file(&io373b->client->dev, &dev_attr_reg))) {
		dev_warn(&io373b->client->dev, "Failed to create attr reg - %d\n", err);
		return err;
	}
	if ((err = device_create_file(&io373b->client->dev, &dev_attr_vdev)))
	{
		dev_warn(&io373b->client->dev, "Failed to create attr vdev - %d\n", err);
		device_remove_file(&io373b->client->dev, &dev_attr_reg);
		return err;
	}
	if ((err = device_create_file(&io373b->client->dev, &dev_attr_update_flash)))
	{
		dev_warn(&io373b->client->dev, "Failed to create attr update_flash - %d\n", err);
		device_remove_file(&io373b->client->dev, &dev_attr_reg);
		device_remove_file(&io373b->client->dev, &dev_attr_vdev);
		return err;
	}

	io373b->dev_attrs_created = true;
	return 0;
}

static void io373b_remove_dev_attrs(struct io373b *io373b)
{
	if (io373b->dev_attrs_created) {
		device_remove_file(&io373b->client->dev, &dev_attr_reg);
		device_remove_file(&io373b->client->dev, &dev_attr_vdev);
		device_remove_file(&io373b->client->dev, &dev_attr_update_flash);
		io373b->dev_attrs_created = false;
	}
}

static irqreturn_t io373b_irq_thread(int irq, void *dev_id)
{
	struct io373b *io373b = dev_id;
	int ret;
	unsigned short pf;

	ret = io373b_i2c_xfer(io373b, io373b->client->addr, CMD_READ_IRQ_PF, (unsigned char *) &pf, 2, XFER_READ);
	if (ret >= 0) {
		pf = le16_to_cpu(pf);
		//dev_dbg(&io373b->client->dev, "IRQ_PF = %04X\n", pf);
		blocking_notifier_call_chain(&io373b->notifier_list, pf, 0);
	}

	return IRQ_HANDLED;
}

static struct platform_device *io373b_add_pdev(struct io373b *io373b, struct i2c_board_info *info, int id)
{
	struct platform_device *pdev = NULL;
	int err = 0;
	union io373b_subdev_board_info *pdata = (union io373b_subdev_board_info *) info->platform_data;
	int pdata_size = pdata ? sizeof(*pdata) : 0;

	pdev = platform_device_alloc(info->type, id);
	if (!pdev) {
		dev_err(&io373b->client->dev, "Failed to platform_device_alloc() for %s, addr 0x%x.\n", info->type, info->addr);
		goto err_out;
	}

	err = platform_device_add_data(pdev, pdata, pdata_size);
	if (err) {
		dev_err(&io373b->client->dev, "Failed to platform_device_add_data() for %s, addr 0x%x.\n", info->type, info->addr);
		goto err_out;
	}

	pdev->dev.parent = &io373b->client->dev;
	io373b->pdevs[id] = pdev;

	err = platform_device_add(pdev);
	if (err) {
		io373b->pdevs[id] = NULL;
		dev_err(&io373b->client->dev, "Failed to platform_device_add() for %s, addr 0x%x.\n", info->type, info->addr);
		goto err_out;
	}

	return pdev;

err_out:
	if (pdev)
		platform_device_put(pdev); /* free */
	return NULL;
}

static void io373b_remove_subdevs(struct io373b *io373b)
{
	int id;

	/* Don't count on io373b->num_subdevs; If io373b_add_subdevs() failed,
	   number of pdevs and dummies might be different.*/

	for (id = 0; id < MAX_SUBDEV_COUNT; id++) {
		if (io373b->pdevs[id]) {
			platform_device_unregister(io373b->pdevs[id]);
			io373b->pdevs[id] = NULL;
		}
	}

	for (id = 0; id < MAX_SUBDEV_COUNT; id++) {
		if (io373b->dummies[id]) {
			i2c_unregister_device(io373b->dummies[id]);
			io373b->dummies[id] = NULL;
		}
	}

	io373b->num_subdevs = 0;

	if (io373b->irq_requested) {
		if (io373b->irq_wake_enabled) { /* if base removed when suspended, resume() which enables wakeup is not called; */
			disable_irq_wake(io373b->client->irq); /* so undo wakeup here, hope it eliminates unexpected wakeup */
			io373b->irq_wake_enabled = false;
		}
		free_irq(io373b->client->irq, io373b);
		io373b->irq_requested = false;
	}
}

int io373b_add_subdevs(struct device *io373b_dev, struct i2c_board_info *info)
{
	struct io373b *io373b = dev_get_drvdata(io373b_dev);
	int id = 0;
	int err = 0;

	if (!info) /* allow no subdev board info */
		return 0;

	if (io373b->num_subdevs) {
		dev_err(&io373b->client->dev, "Subdevs already added\n");
		return -EINVAL;
	}

	if (io373b->client->irq) { /* allow no irq for smbd 0xc0 */
#ifdef DEVELOPING /* setting MINI6410 GPIO, For EINT9, Inturrupt */
		s3c_gpio_cfgpin(S3C64XX_GPN(9), 0x02 << 18);
#endif
		if ((err = request_threaded_irq(io373b->client->irq, NULL, io373b_irq_thread,
						IRQF_TRIGGER_LOW | IRQF_ONESHOT, io373b->client->name, io373b))) {
			dev_err(&io373b->client->dev, "Failed to request IRQ %d -- %d\n", io373b->client->irq, err);
			goto err_out;
		}
		else
			io373b->irq_requested = true;
	}

	while (info->addr && (id < MAX_SUBDEV_COUNT)) {
		struct i2c_client *dummy;
		struct platform_device *pdev;

		dummy = i2c_new_dummy(io373b->client->adapter, info->addr);
		if (!dummy) {
			dev_err(&io373b->client->dev, "Failed to i2c_new_dummy() for %s, addr 0x%x.\n", info->type, info->addr);
			err = -ENODEV;
			goto err_out;
		}
		io373b->dummies[id] = dummy; /* must do this early! */

		pdev = io373b_add_pdev(io373b, info, id);
		if (!pdev)
			goto err_out;

		info++;
		id++;
	}

	if (info->addr) { /* still has sub devs, too many */
		err = -EINVAL;
		dev_err(&io373b->client->dev, "Too many virtual i2c devices specified, at most %d\n", MAX_SUBDEV_COUNT);
		goto err_out;
	}

	io373b->num_subdevs = id;

	return 0;

err_out:

	io373b_remove_subdevs(io373b);

	return err;
}
EXPORT_SYMBOL(io373b_add_subdevs);

static int io373b_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct io373b *io373b = 0;

	dev_dbg(&client->dev, ".probe() with name = %s, addr = 0x%x, irq = %d\n", client->name, client->addr, client->irq);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) { /* to use i2c_transfer() */
		dev_err(&client->dev, "I2C adapter has no I2C_FUNC_I2C functionality.\n");
		return -ENODEV;
	}

	if (!(io373b = kzalloc(sizeof(*io373b), GFP_KERNEL)))
		return -ENOMEM;

	io373b->client = client;
	i2c_set_clientdata(client, io373b);

	mutex_init(&io373b->lock);
	BLOCKING_INIT_NOTIFIER_HEAD(&io373b->notifier_list);

	/* Now don't error exit to keep providing reg/subdev read/write
	   and flash-update functions. */

	io373b_create_dev_attrs(io373b);

	io373b_add_subdevs(&client->dev, dev_get_platdata(&client->dev));

	device_init_wakeup(&client->dev, true);

	return 0;
}

static int io373b_remove(struct i2c_client *client)
{
	struct io373b *io373b = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "%s\n", __func__);

	io373b_remove_subdevs(io373b);
	io373b_remove_dev_attrs(io373b);
	device_init_wakeup(&client->dev, false);
	kfree(io373b);

	return 0;
}

#ifdef CONFIG_PM
static int io373b_suspend(struct device *dev)
{
	struct io373b *io373b = dev_get_drvdata(dev);
	struct i2c_client *client = io373b->client;

	if (io373b->irq_requested) {
		disable_irq(client->irq);

		if (device_may_wakeup(&client->dev)) {
			enable_irq_wake(client->irq);
			io373b->irq_wake_enabled = true;
		}
	}

	return 0;
}

static int io373b_resume(struct device *dev)
{
	struct io373b *io373b = dev_get_drvdata(dev);
	struct i2c_client *client = io373b->client;

	if (io373b->irq_requested) {
		if (io373b->irq_wake_enabled)
		{
			disable_irq_wake(client->irq);
			io373b->irq_wake_enabled = false;
		}

		enable_irq(client->irq);
	}

	return 0;
}

static const struct dev_pm_ops io373b_dev_pm_ops = {
	.suspend = io373b_suspend,
	.resume  = io373b_resume,
};
#endif

static const struct i2c_device_id io373b_idtable[] = {
	{ "io373b-smbd", 0 },
	{ "io373b-smbd-slate", 0 },
	{ "io373b-smbd-base", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, io373b_idtable);

static struct i2c_driver io373b_driver = {
	.driver = {
		.name   = "io373b-smbd",
		.owner  = THIS_MODULE,
#ifdef CONFIG_PM
		.pm   = &io373b_dev_pm_ops,
#endif
	},
	.probe      = io373b_probe,
	.remove     = io373b_remove,
	.id_table   = io373b_idtable,
};

static int __init io373b_init(void)
{
	printk(KERN_INFO "%s\n", __func__);

	return i2c_add_driver(&io373b_driver);
}

static void __exit io373b_exit(void)
{
	printk(KERN_INFO "%s\n", __func__);

	i2c_del_driver(&io373b_driver);
}

module_init(io373b_init);
module_exit(io373b_exit);
MODULE_AUTHOR("flychen");
MODULE_DESCRIPTION("IO373X-B SMBD (I2C) driver");
MODULE_LICENSE("GPL");
