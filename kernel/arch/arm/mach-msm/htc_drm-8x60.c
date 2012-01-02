/* arch/arm/mach-msm/htc_drm-8x60.c
 *
 * Copyright (C) 2011 HTC Corporation.
 * Author: Eddic Hsien <eddic_hsien@htc.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include <mach/scm.h>

#define DEVICE_NAME "htcdrm"

#define HTCDRM_IOCTL_WIDEVINE	0x2563

#define DEVICE_ID_LEN			32
#define WIDEVINE_KEYBOX_LEN		128

#undef PDEBUG
#define PDEBUG(fmt, args...) printk(KERN_INFO "%s(%i, %s): " fmt "\n", \
		__func__, current->pid, current->comm, ## args)

#undef PERR
#define PERR(fmt, args...) printk(KERN_ERR "%s(%i, %s): " fmt "\n", \
		__func__, current->pid, current->comm, ## args)

static int htcdrm_major;
static struct class *htcdrm_class;
static const struct file_operations htcdrm_fops;

typedef struct _htc_drm_msg_s {
	int func;
	int offset;
	unsigned char *req_buf;
	int req_len;
	unsigned char *resp_buf;
	int resp_len;
} htc_drm_msg_s;

enum {
		HTC_OEMCRYPTO_STORE_KEYBOX = 1,
		HTC_OEMCRYPTO_GET_KEYBOX,
		HTC_OEMCRYPTO_IDENTIFY_DEVICE,
		HTC_OEMCRYPTO_GET_RANDOM,
};

static unsigned char *htc_device_id;
static unsigned char *htc_keybox;

static int htcdrm_ioctl(struct inode *inode, struct file *file,
						unsigned int command, unsigned long arg)
{
	htc_drm_msg_s hmsg;
	int ret = 0;
	unsigned char *ptr;

	PDEBUG("command = %x\n", command);
	switch (command) {
	case HTCDRM_IOCTL_WIDEVINE:
		if (copy_from_user(&hmsg, (void __user *)arg, sizeof(hmsg))) {
			PERR("copy_from_user error (msg)");
			return -EFAULT;
		}

		PDEBUG("func = %x\n", hmsg.func);
		switch (hmsg.func) {
		case HTC_OEMCRYPTO_STORE_KEYBOX:
			if ((hmsg.req_buf == NULL) || (hmsg.req_len != WIDEVINE_KEYBOX_LEN)) {
				PERR("invalid arguments");
				return -EFAULT;
			}
			if (copy_from_user(htc_keybox, (void __user *)hmsg.req_buf, hmsg.req_len)) {
				PERR("copy_from_user error (keybox)");
				return -EFAULT;
			}

			ret = secure_access_item(1, ITEM_KEYBOX_PROVISION, hmsg.req_len,
					htc_keybox);
			if (ret)
				PERR("provision keybox failed (%d)\n", ret);
			break;
		case HTC_OEMCRYPTO_GET_KEYBOX:
			if ((hmsg.resp_buf == NULL) || !hmsg.resp_len ||
					((hmsg.offset + hmsg.resp_len) > WIDEVINE_KEYBOX_LEN)) {
				PERR("invalid arguments");
				return -EFAULT;
			}

			ret = secure_access_item(0, ITEM_KEYBOX_DATA, WIDEVINE_KEYBOX_LEN,
					htc_keybox);
			if (ret)
				PERR("get keybox failed (%d)\n", ret);
			else {
				if (copy_to_user((void __user *)hmsg.resp_buf, htc_keybox + hmsg.offset, hmsg.resp_len)) {
					PERR("copy_to_user error (keybox)");
					return -EFAULT;
				}
			}
			break;
		case HTC_OEMCRYPTO_IDENTIFY_DEVICE:
			if ((hmsg.resp_buf == NULL) || (hmsg.resp_len != DEVICE_ID_LEN)) {
				PERR("invalid arguments");
				return -EFAULT;
			}

			ret = secure_access_item(0, ITEM_DEVICE_ID, DEVICE_ID_LEN,
					htc_device_id);
			if (ret)
				PERR("get device ID failed (%d)\n", ret);
			else {
				if (copy_to_user((void __user *)hmsg.resp_buf, htc_device_id, DEVICE_ID_LEN)) {
					PERR("copy_to_user error (device ID)");
					return -EFAULT;
				}
			}
			break;
		case HTC_OEMCRYPTO_GET_RANDOM:
			if ((hmsg.resp_buf == NULL) || !hmsg.resp_len) {
				PERR("invalid arguments");
				return -EFAULT;
			}
			ptr = kzalloc(hmsg.resp_len, GFP_KERNEL);
			if (ptr == NULL) {
				PERR("allocate the space for random data failed\n");
				return -1;
			}

			ret = secure_access_item(0, ITEM_RAND_DATA, hmsg.resp_len, ptr);
			if (ret)
				PERR("get random data failed (%d)\n", ret);
			else {
				if (copy_to_user((void __user *)hmsg.resp_buf, ptr, hmsg.resp_len)) {
					PERR("copy_to_user error (random data)");
					kfree(ptr);
					return -EFAULT;
				}
			}
			kfree(ptr);
			break;
		default:
			PERR("func error\n");
			return -EFAULT;
		}

		break;

	default:
		PERR("command error\n");
		return -EFAULT;
	}
	return ret;
}

static const struct file_operations htcdrm_fops = {
	.ioctl  =   htcdrm_ioctl,
	.owner  =   THIS_MODULE,
};


static int __init htcdrm_init(void)
{
	int ret;

	htc_device_id = kzalloc(DEVICE_ID_LEN, GFP_KERNEL);
	if (htc_device_id == NULL) {
		PERR("allocate the space for device ID failed\n");
		return -1;
	}
	htc_keybox = kzalloc(WIDEVINE_KEYBOX_LEN, GFP_KERNEL);
	if (htc_keybox == NULL) {
		PERR("allocate the space for keybox failed\n");
		kfree(htc_device_id);
		return -1;
	}

	ret = register_chrdev(0, DEVICE_NAME, &htcdrm_fops);
	if (ret < 0) {
		PERR("register module fail\n");
		kfree(htc_device_id);
		kfree(htc_keybox);
		return ret;
	}
	htcdrm_major = ret;

	htcdrm_class = class_create(THIS_MODULE, "htcdrm");
	device_create(htcdrm_class, NULL, MKDEV(htcdrm_major, 0), NULL, DEVICE_NAME);

	PDEBUG("register module ok\n");
	return 0;
}


static void  __exit htcdrm_exit(void)
{
	device_destroy(htcdrm_class, MKDEV(htcdrm_major, 0));
	class_unregister(htcdrm_class);
	class_destroy(htcdrm_class);
	unregister_chrdev(htcdrm_major, DEVICE_NAME);
	kfree(htc_device_id);
	kfree(htc_keybox);
	PDEBUG("un-registered module ok\n");
}

module_init(htcdrm_init);
module_exit(htcdrm_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Eddic Hsien");

