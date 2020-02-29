/*
 * door_lock.c
 *
 * Door lock is a simple device, which handle a lock (open/close
 * using relay) and a button.
 * Copyright (C) 2019 Artsiom Asadchy <artyomka111@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

#define DOOR_LOCK_NUM_MINORS	1
#define DOOR_LOCK_DEFAULT_DELAY	5000

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static DECLARE_BITMAP(minors, DOOR_LOCK_NUM_MINORS);

struct door_lock {
	struct platform_device *pdev;
	struct gpio_desc *gpiod_lock;
	struct delayed_work dwork;
	struct list_head device_entry;
	struct mutex mutex_lock;
	struct mutex button_mutex_lock;
	struct completion button_completion;
	int button_pressing_counter;
	dev_t devt;
	int irq;
	bool const_open;
	unsigned int delay;
};

static inline void door_lock_open(struct door_lock *ctx)
{
	gpiod_set_value(ctx->gpiod_lock, 0);
}

static inline void door_lock_close(struct door_lock *ctx)
{
	gpiod_set_value(ctx->gpiod_lock, 1);
}

static int door_lock_fops_open(struct inode *inode, struct file *filp)
{
	struct door_lock *ctx;
	int ret = -ENXIO;

	mutex_lock(&device_list_lock);
	list_for_each_entry(ctx, &device_list, device_entry) {
		if (ctx->devt == inode->i_rdev) {
			ret = 0;
			filp->private_data = ctx;
			nonseekable_open(inode, filp);
			break;
		}
	}
	mutex_unlock(&device_list_lock);

	return ret;
}

static ssize_t door_lock_fops_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	signed char delay_value;
	struct door_lock *ctx = filp->private_data;

	if(!count)
		return -EINVAL;

	if(copy_from_user(&delay_value, buf, sizeof(delay_value)) != 0)
		return -EAGAIN;

	if(delay_value < -1)
		return -EINVAL;

	if(mutex_lock_interruptible(&ctx->mutex_lock) < 0)
		return -EINTR;

	if(delay_value == 1) {
		if(!delayed_work_pending(&ctx->dwork) && !ctx->const_open) {
			door_lock_open(ctx);
			schedule_delayed_work(&ctx->dwork, msecs_to_jiffies(ctx->delay));
		}
	} else if(delay_value == -1) {
		ctx->const_open = true;
		door_lock_open(ctx);
	} else if(delay_value == 0) {
		ctx->const_open = false;
		door_lock_close(ctx);
	}

	mutex_unlock(&ctx->mutex_lock);

	return count;
}

static ssize_t door_lock_fops_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	ssize_t ret;
	unsigned long status;
	struct door_lock *ctx = filp->private_data;

	if(count < sizeof(ctx->button_pressing_counter))
		return -EINVAL;

	if(mutex_lock_interruptible(&ctx->button_mutex_lock) < 0)
		return -EINTR;

	reinit_completion(&ctx->button_completion);

	if(wait_for_completion_interruptible(&ctx->button_completion) < 0) {
		mutex_unlock(&ctx->button_mutex_lock);
		return -EINTR;
	}

	status = copy_to_user(buf, &ctx->button_pressing_counter, sizeof(ctx->button_pressing_counter));
	mutex_unlock(&ctx->button_mutex_lock);

	if(status == 0) {
		ret = sizeof(ctx->button_pressing_counter);
	} else {
		ret = -EAGAIN;
	}

	return ret;
}

static const struct file_operations door_lock_fops = {
	.owner = THIS_MODULE,
	.open = door_lock_fops_open,
	.write = door_lock_fops_write,
	.read = door_lock_fops_read,
};

static irqreturn_t door_lock_threaded_irq(int irq, void *dev_id)
{
	struct door_lock *ctx = dev_id;

	mutex_lock(&ctx->mutex_lock);

	if(!delayed_work_pending(&ctx->dwork) && !ctx->const_open) {
		door_lock_open(ctx);
		ctx->button_pressing_counter++;
		complete(&ctx->button_completion);
		schedule_delayed_work(&ctx->dwork, msecs_to_jiffies(ctx->delay));
	}

	mutex_unlock(&ctx->mutex_lock);

	return IRQ_HANDLED;
}

static void door_lock_work(struct work_struct *work)
{
	struct door_lock *ctx = container_of(work, struct door_lock, dwork.work);

	mutex_lock(&ctx->mutex_lock);

	if(!ctx->const_open) {
		door_lock_close(ctx);
	}

	mutex_unlock(&ctx->mutex_lock);
}

static struct class *door_lock_class;
static unsigned int major_num;

static int door_lock_probe(struct platform_device *pdev)
{
	struct door_lock *ctx;
	int ret;
	struct device *dev = &pdev->dev;
	unsigned long minor;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->pdev = pdev;
	ctx->const_open = false;

	if(of_property_read_u32(pdev->dev.of_node, "open-time", &ctx->delay) < 0)
		ctx->delay = DOOR_LOCK_DEFAULT_DELAY;

	ctx->gpiod_lock = devm_gpiod_get_optional(dev, "lock", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->gpiod_lock)) {
		ret = PTR_ERR(ctx->gpiod_lock);
		dev_err(dev, "cannot get lock GPIO: %d\n", ret);
		return ret;
	}
	if(ctx->gpiod_lock == NULL)
		return -ENODEV;

	platform_set_drvdata(pdev, ctx);

	ctx->devt = MKDEV(major_num, 0);
	INIT_LIST_HEAD(&ctx->device_entry);
	INIT_DELAYED_WORK(&ctx->dwork, door_lock_work);
	mutex_init(&ctx->mutex_lock);
	mutex_init(&ctx->button_mutex_lock);
	init_completion(&ctx->button_completion);

	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, DOOR_LOCK_NUM_MINORS);
	if(minor < DOOR_LOCK_NUM_MINORS) {
		dev = device_create(door_lock_class, dev, ctx->devt, ctx, "doorlock%ld", minor);
		ret = PTR_ERR_OR_ZERO(dev);
		if(!ret) {
			set_bit(minor, minors);
			list_add(&ctx->device_entry, &device_list);
		}
	} else {
		ret = -ENODEV;
	}
	mutex_unlock(&device_list_lock);

	if(ret)
		return ret;

	ctx->irq = platform_get_irq(pdev, 0);
	if(ctx->irq >= 0) {
		ret = devm_request_threaded_irq(&pdev->dev, ctx->irq, NULL, door_lock_threaded_irq,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT, dev_name(&pdev->dev), ctx);
		if(ret)
			goto dev_destroy;
	}

	return ret;

dev_destroy:
	mutex_lock(&device_list_lock);
	list_del(&ctx->device_entry);
	device_destroy(door_lock_class, ctx->devt);
	clear_bit(MINOR(ctx->devt), minors);
	mutex_unlock(&device_list_lock);
	mutex_destroy(&ctx->mutex_lock);
	mutex_destroy(&ctx->button_mutex_lock);
	complete_release(&ctx->button_completion);

	return ret;
}

static int door_lock_remove(struct platform_device *pdev)
{
	struct door_lock *ctx = platform_get_drvdata(pdev);

	mutex_lock(&device_list_lock);
	list_del(&ctx->device_entry);
	device_destroy(door_lock_class, ctx->devt);
	clear_bit(MINOR(ctx->devt), minors);
	mutex_unlock(&device_list_lock);

	cancel_delayed_work_sync(&ctx->dwork);
	mutex_destroy(&ctx->mutex_lock);
	mutex_destroy(&ctx->button_mutex_lock);
	complete_release(&ctx->button_completion);

	return 0;
}

static const struct of_device_id door_lock_of_match[] = {
	{ .compatible = "door_lock" },
	{ }
};

MODULE_DEVICE_TABLE(of, door_lock_of_match);

static struct platform_driver door_lock_driver = {
	.driver = {
		.name	= "door_lock",
		.of_match_table	= of_match_ptr(door_lock_of_match),
	},
	.probe	= door_lock_probe,
	.remove = door_lock_remove,
};

static int __init door_lock_driver_init(void)
{
	int ret;

	major_num = register_chrdev(0, door_lock_driver.driver.name, &door_lock_fops);
	if (major_num < 0)
		return major_num;

	door_lock_class = class_create(THIS_MODULE, door_lock_driver.driver.name);
	if (IS_ERR(door_lock_class)) {
		ret = PTR_ERR(door_lock_class);
		goto chdev_destroy;
	}

	ret = platform_driver_register(&door_lock_driver);
	if(ret < 0)
		goto cl_destroy;

	return 0;

cl_destroy:
	class_destroy(door_lock_class);
chdev_destroy:
	unregister_chrdev(major_num, door_lock_driver.driver.name);

	return ret;
}

static void __exit door_lock_driver_exit(void)
{
	platform_driver_unregister(&door_lock_driver);
	class_destroy(door_lock_class);
	unregister_chrdev(major_num, door_lock_driver.driver.name);
}

module_init(door_lock_driver_init);
module_exit(door_lock_driver_exit);

MODULE_AUTHOR("Artsiom Asadchy");
MODULE_DESCRIPTION("Door lock driver");
MODULE_LICENSE("GPL");
