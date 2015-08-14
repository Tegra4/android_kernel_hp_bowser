#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/i2c.h>

#include "edid.h"

/* I2C relation */
#define DEVICE_NAME "AUO101"

#define AUO_REG_NUM  0x80

static int AUO101_probe(struct i2c_client *adapter, const struct i2c_device_id *id);
static int AUO101_detach(struct i2c_client *client);

static const struct i2c_device_id AUO101_id[] = {
       {DEVICE_NAME, 0},
       {},
};

MODULE_DEVICE_TABLE(i2c, AUO101_id);

static struct i2c_driver AUO101_i2c_driver = {
       .driver = {
                  .owner = THIS_MODULE,
                  .name = DEVICE_NAME,
                  },
       .probe = AUO101_probe,
       .remove = AUO101_detach,
       .id_table = AUO101_id,
};

struct i2c_client *i2c_client;


// i2c_read_reg
static inline int i2c_read_reg(char reg, char* ret)
{
       char buf[3];

    ///printk(KERN_ALERT"[EDID] i2c_read_reg+++\n");

       if (i2c_client == NULL) {
        printk(KERN_ALERT"[EDID] Fail! i2c_client is NULL. \n");
        return -1;
    }

    buf[0] = reg & 0xFF;

    if (i2c_master_send(i2c_client, buf, 1) != 1) {
        printk(KERN_ALERT"[EDID] Fail to i2c_master_send()! \n");
        return -1;
    }

    if (i2c_master_recv(i2c_client, buf, 1) != 1) {
        printk(KERN_ALERT"[EDID] Fail to i2c_master_send()! \n");
        return -1;
    }

    *ret = buf[0];

    return 0;
}


// AUO101_probe
static int AUO101_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    printk(KERN_ALERT"[EDID] AUO101_probe+++\n");

    i2c_client = client;

       return 0;
}


// AUO101_detach
static int AUO101_detach(struct i2c_client *client)
{
    printk(KERN_ALERT"[EDID] AUO101_detach+++\n");
       return 0;
}


// panel_proc_read
static ssize_t panel_proc_read(char* page, char** start, off_t off, int count, int* eof, void* data)
{
    char i = 0;
    char ret;
    char buff[AUO_REG_NUM];
    int cnt = 0;

    printk(KERN_ALERT"[EDID] panel_proc_read+++\n");

    printk("off= %d \n", off);
    if (off > 0) {
        *eof = 1;
        return 0;
    }


    for (i=0;i<AUO_REG_NUM;i++) {
        if (i2c_read_reg(i, buff+i) == 0) {
            cnt++;

        } else {
            printk("Fail to read register addr= 0x%02X! \n", i);
        }

    }

    //for (i=0;i<AUO_REG_NUM;i++)
    //    printk("[0x%02X] 0x%02X \n", i, buff[i]);

    memcpy(page, buff, AUO_REG_NUM);

    return cnt;
}


// panel_create_proc
static void panel_create_proc(void)
{
       struct proc_dir_entry* entry;

    printk(KERN_ALERT"[EDID] panel_create_proc+++\n");

       entry = create_proc_entry(EDID_PROC_NAME, 0, NULL);
       if(entry) {
               entry->read_proc = panel_proc_read;

       }
}


// panel_init
static int __init panel_init(void)
{
    int err = 0;

    printk(KERN_ALERT"[EDID] panel_init+++\n");

    i2c_client = NULL;

       err = i2c_add_driver(&AUO101_i2c_driver);
       if (err != 0) {
        printk(KERN_ALERT"[EDID] Fail to add i2c driver! \n");
        return err;
    }

       panel_create_proc();

    return err;
}


// panel_exit
static void __exit panel_exit(void)
{
    printk(KERN_ALERT"[EDID] panel_exit+++\n");

    i2c_del_driver(&AUO101_i2c_driver);

    remove_proc_entry(EDID_PROC_NAME, NULL);
}


MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Panel EDID Driver");

module_init(panel_init);
module_exit(panel_exit);
