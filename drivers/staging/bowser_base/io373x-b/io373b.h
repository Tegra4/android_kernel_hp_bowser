/*
 * Interface between IO373X-B SMBD (I2C) and its sub devices.
 */

#ifndef _IO373B_H_
#define _IO373B_H_

struct i2c_board_info;

extern int io373b_register_notifier(struct device *io373b_dev, struct notifier_block *nb);
extern int io373b_unregister_notifier(struct device *io373b_dev, struct notifier_block *nb);

extern int io373b_subdev_read(struct device *subdev, unsigned char cmd, unsigned char *buf, int byte_cnt);
extern int io373b_subdev_write(struct device *subdev, unsigned char cmd, unsigned char *buf, int byte_cnt);

extern int io373b_read_regs(struct device *io373b_dev, unsigned short start_reg, unsigned char *buf, int byte_cnt);
extern int io373b_write_regs(struct device *io373b_dev, unsigned short start_reg, unsigned char *buf, int byte_cnt);

// Read/write only 1 byte.
static inline int io373b_read_reg(struct device *io373b_dev, unsigned short reg, unsigned char *p_1byte) { return io373b_read_regs(io373b_dev, reg, p_1byte, 1); }
static inline int io373b_write_reg(struct device *io373b_dev, unsigned short reg, unsigned char _1byte) { return io373b_write_regs(io373b_dev, reg, &_1byte, 1); }

extern int io373b_add_subdevs(struct device *io373b_dev, struct i2c_board_info *info);
extern int io373b_update_flash(struct device *io373b_dev, const u8 *data, size_t size);
extern struct device *io373b_get_subdev(struct device *io373b_dev, char *name);

#define IRQ_PF_CHARGER		(1 << 5)
#define IRQ_PF_BATTERY		(1 << 6)
#define IRQ_PF_PS2_KBD		(1 << 1)
#define IRQ_PF_PS2_AUX		(1 << 2)

union io373b_subdev_board_info {
	struct acpi {
		int num_bat;
		char bat_name[2][20];
		char charger_name[2][20];
	} acpi;
	struct ps2_kbd {
	} ps2_kbd;
	struct ps2_aux {
	} ps2_aux;
};

#endif // _IO373B_H_
