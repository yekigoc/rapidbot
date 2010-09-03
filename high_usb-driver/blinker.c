#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/usb.h>
#include <linux/major.h>

MODULE_LICENSE("GPL");

#define DEVICE_NAME "blinker"
#define PRODUCT_ID 0x0920
#define VENDOR_ID 0xeb03

#define SET_LED 0x04
#define GET_COUNT 0x03

#define CTRL_IN (0x01 | 0x80)
#define CTRL_OUT (0x01 | 0x00)

#define USB_RQ_START 0x00

static struct usb_device_id blinker_devices [] __devinitdata = {
    { USB_DEVICE(VENDOR_ID, PRODUCT_ID) },
    { },
};

MODULE_DEVICE_TABLE(usb, blinker_devices);

int blinker_probe(struct usb_interface *intf, const struct usb_device_id *id);
void blinker_disconnect(struct usb_interface *intf);

struct blinker_dev {
    struct usb_device   *udev;      /* our blinker */
    int                 open_count; /* count the number openers */
    int                 is_read;    /* 0 if counter wasn't read */
    int                 is_written;
    char                leds;
};

static struct usb_driver blinker_driver = {
    .name       =   DEVICE_NAME,
    .probe      =   blinker_probe,
    .disconnect =   blinker_disconnect,
    .id_table   =   blinker_devices,
};

static int blinker_open(struct inode *inode, struct file *file) {
    struct blinker_dev *dev;
    struct usb_interface *intf;
    int subminor;
    int ret = 0;

    subminor = iminor(inode);

    intf = usb_find_interface(&blinker_driver, subminor);
    if(!intf) {
        err("%s - error, can't find device for minor %d", __func__, subminor);
        ret = -ENODEV;
        goto exit;
    }

    dev = usb_get_intfdata(intf);
    if(!dev) {
        ret = -ENODEV;
        goto exit;
    }

    if(dev->open_count) {
        ret = -EBUSY;
    } else {
        dev->open_count++;
        dev->is_read = 0;
    }

    file->private_data = dev;

 exit:
    return ret;
}

static int blinker_release(struct inode *inode, struct file *file) {
    struct blinker_dev *dev;
    int ret = 0;

    dev = (struct blinker_dev *) file->private_data;
    if(!dev) {
        ret = -ENODEV;
        goto exit;
    }

    dev->open_count--;
    dev->is_read = 0;
    dev->is_written = 0;

 exit:
    return ret;
}
static int blinker_get_counter(struct usb_device *udev, unsigned int *counter) {
    int ret;
    ret = usb_control_msg(udev,
            usb_rcvctrlpipe(udev, 0),
            USB_RQ_START,
            CTRL_IN,
            GET_COUNT,
            0,
            counter,
            4,
            0
    );

    return ret;
}

static ssize_t blinker_read(struct file *file, char *buffer, size_t count, loff_t *ppos) {
    int ret;
    unsigned int counter;

    struct blinker_dev *dev;

    char *ansbuf;
    int answer_size;
    const int ansbuf_size = 12; // max unsigned int's length + 2 sumbols for \n\0
    const char answer_template[] = "%u\n";

    dev = (struct blinker_dev *) file->private_data;

    if(dev->is_read) {
        ret = 0;
        goto exit;
    }

    ret = blinker_get_counter(dev->udev, &counter);
    if(ret == 4) { // 4 is counter length

        printk(KERN_NOTICE "in if (ret > 0)");
        printk(KERN_NOTICE "ret is %d counter is %u", ret, counter);

        ansbuf = kzalloc(ansbuf_size, GFP_KERNEL);
        if(!ansbuf) {
            err("out of memory");
            ret = -ENOMEM;
            goto exit;
        }

        sprintf(ansbuf, answer_template, counter);
        answer_size = strlen(ansbuf);
        if(copy_to_user(buffer, ansbuf, answer_size)) {
            ret = -EFAULT;
        } else {
            ret = answer_size;
        }

        kfree(ansbuf);
        dev->is_read = 1;
    } else {
        err("Something wrong");
    }

 exit:
    return ret;
}

static int blinker_set_led(struct usb_device *udev, char *leds) {
    int ret;
    ret = usb_control_msg(udev, //device
            usb_sndctrlpipe(udev, 0), //pipe
            USB_RQ_START, //request
            CTRL_OUT, //type
            SET_LED, //value
            0, //index
            leds, //data
            1, //size
            0 //timeout
    );

    if(ret == 1) {
        printk(KERN_NOTICE "led %s", *leds ? "off" : "on");
    }

    return ret;
}

static ssize_t blinker_write(struct file *file, const char *ubuff, size_t count, loff_t *ppos) {
    struct blinker_dev *dev;
    int ret;
    
    dev = (struct blinker_dev *) file->private_data;
    if(dev->is_written) {
        ret = count;
        goto exit;
    }

    ret = blinker_set_led(dev->udev, &dev->leds);
    if(ret < 0) {
        goto exit;
    }

    dev->leds = !dev->leds;
    dev->is_written = 1;
    ret = count;

 exit:
    return ret;
}

static const struct file_operations blinker_fops = {
    .owner      =   THIS_MODULE,
    .open       =   blinker_open,
    .read       =   blinker_read,
    .write      =   blinker_write,
    .release    =   blinker_release,
};

static struct usb_class_driver blinker_class_driver = {
    .name       =  "blink%d",
    .fops       = &blinker_fops,
    .minor_base = 0,
};

int blinker_probe(struct usb_interface *intf, const struct usb_device_id *id) {
    int ret;
    struct blinker_dev *dev;

    printk(KERN_NOTICE "call probe function");
    
    /* allocation and inicialization for our blinker device data */
    dev = kzalloc(sizeof(*dev), GFP_KERNEL);
    if(!dev) {
        err("Out of memory");
        ret = -ENOMEM;
        goto error;
    }

    dev->open_count = 0;
    dev->is_read = 0;
    dev->is_written = 0;
    dev->leds = 1;
    dev->udev = usb_get_dev(interface_to_usbdev(intf));

    /* push blinker's data pointer to interface */
    usb_set_intfdata(intf, dev);

    /* register blinker's device */
    ret = usb_register_dev(intf, &blinker_class_driver);
    if(ret < 0) {
        err("Not able to get a minor for this device.");
    }
    
    printk(KERN_NOTICE DEVICE_NAME " now attached to /dev/blink%d", intf->minor);
    return 0;

 error:
    if(dev) {
        kfree(dev);
    }

    return ret;
}

void blinker_disconnect(struct usb_interface *intf) {
    int minor;
    struct blinker_dev *dev;

    printk(KERN_NOTICE "call disconnect function");
    minor = intf->minor;

    dev = usb_get_intfdata(intf);
    kfree(dev);

    usb_deregister_dev(intf, &blinker_class_driver);
    printk(KERN_NOTICE DEVICE_NAME " /dev/blink%d now disconnected", minor);
}

static int __init blinker_init(void) {
    int ret;

    printk(KERN_NOTICE "loading " DEVICE_NAME " driver");
    ret = usb_register(&blinker_driver);
    if(ret) {
        err("register " DEVICE_NAME " driver was failed");
    }

    return ret;
}

static void __exit blinker_cleanup(void) {
    printk(KERN_NOTICE "cleanup " DEVICE_NAME " driver");

    usb_deregister(&blinker_driver);
}

module_init(blinker_init);
module_exit(blinker_cleanup);

