/* tui/main.c
 *
 * Samsung TUI HW Handler driver.
 *
 * Copyright (c) 2015 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/uaccess.h>

#include "stui_core.h"
#include "stui_hal.h"
#include "stui_inf.h"
#include "stui_ioctl.h"

struct device *stui_device;
static struct cdev stui_cdev;
static struct class *tui_class;
static DEFINE_MUTEX(stui_mode_mutex);

static int stui_open(struct inode *inode, struct file *filp)
{
	int ret = 0;
	mutex_lock(&stui_mode_mutex);
	filp->private_data = NULL;
	if (stui_get_mode() & STUI_MODE_ALL) {
		ret = -EBUSY;
		pr_err("[STUI] Device is busy\n");
		mutex_unlock(&stui_mode_mutex);
	}
	mutex_unlock(&stui_mode_mutex);
	return ret;
}

static int stui_release(struct inode *inode, struct file *filp)
{
	mutex_lock(&stui_mode_mutex);
	if ((stui_get_mode() & STUI_MODE_ALL) && filp->private_data) {
		pr_err("[STUI] Device close while TUI session is active\n");
		msleep(4000);
		stui_process_cmd(filp, STUI_HW_IOCTL_FINISH_TUI, 0);
	}
	mutex_unlock(&stui_mode_mutex);
	return 0;
}


static long stui_handler_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	long ret;
	mutex_lock(&stui_mode_mutex);
	ret = stui_process_cmd(f, cmd, arg);
	f->private_data = (void *)((stui_get_mode() & STUI_MODE_ALL)?1l:0l);
	mutex_unlock(&stui_mode_mutex);
	return ret;
}

static const struct file_operations tui_fops = {
	.owner = THIS_MODULE,
	.open = stui_open,
	.release = stui_release,
	.unlocked_ioctl = stui_handler_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = stui_handler_ioctl,
#endif /* CONFIG_COMPAT */
};

static int stui_handler_init(void)
{
	int err = 0;
	dev_t devno;

	pr_debug("[STUI] stui_handler_init\n");
	err = alloc_chrdev_region(&devno, 0, 1, STUI_DEV_NAME);
	if (err) {
		pr_err("[STUI] Unable to allocate TUI device number(%d)\n", err);
		return err;
	}
	tui_class = class_create(THIS_MODULE, STUI_DEV_NAME);
	if (IS_ERR(tui_class)) {
		pr_err("[STUI] Failed to create TUI class\n");
		err = PTR_ERR(tui_class);
		goto err_class_create;
	}
	cdev_init(&stui_cdev, &tui_fops);
	err = cdev_add(&stui_cdev, devno, 1);
	if (err) {
		pr_err("[STUI] Unable to add TUI char device(%d)\n", err);
		goto err_cdev_add;
	}
	wake_lock_init(&tui_wakelock, WAKE_LOCK_SUSPEND, "TUI_WAKELOCK");
	stui_device = device_create(tui_class, NULL, devno, NULL, STUI_DEV_NAME);
	if (!IS_ERR(stui_device))
		return 0;

	err = PTR_ERR(stui_device);
	wake_lock_destroy(&tui_wakelock);

err_cdev_add:
		class_destroy(tui_class);

err_class_create:
		unregister_chrdev_region(devno, 1);
		return err;
	}

static void stui_handler_exit(void)
{
	pr_debug("[STUI] stui_handler_exit\n");
	wake_lock_destroy(&tui_wakelock);
	unregister_chrdev_region(stui_cdev.dev, 1);
	cdev_del(&stui_cdev);
	device_destroy(tui_class, stui_cdev.dev);
	class_destroy(tui_class);
}

module_init(stui_handler_init);
module_exit(stui_handler_exit);
