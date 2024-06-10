/*
 * Raspberry Pi IN-PI556 PWM driver
 *
 * Written by: Artsiom Asadchy
 * Based on WS2812 driver from Gordon Hollingworth <gordon@fiveninjas.com>
 * Based on DMA PWM driver from Jonathan Bell <jonathan@raspberrypi.org>
 *
 * Copyright (C) 2014 Raspberry Pi Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * To use this driver you need to make sure that the PWM clock is set to 3MHz
 * and the correct PWM output is connected.  The best way to do this is to
 * create a dt-blob.bin on your RaspberryPi, start by downloading the default
 * dt-blob.dts from
 *
 * Note, this uses the same PWM hardware as the standard audio output on the Pi
 * so you cannot use both simultaneously.
 *
 * http://www.raspberrypi.org/documentation/configuration/pin-configuration.md
 *
 * (Copy the bit from /dts-v1/; through to the end...  This will contain the pin
 * configuration for all the Raspberry Pi versions (since they are different.
 * You can get rid of the ones you don't care about.  Next alter the PWM0 output
 * you want to use.
 *
 * http://www.raspberrypi.org/documentation/hardware/raspberrypi/bcm2835/BCM2835-ARM-Peripherals.pdf
 *
 * The link above will help understand what the GPIOs can do, check out page 102
 *
 * pin@p40 {
 *  function = "pwm";
 *  termination = "no_pulling";
 * };
 *
 * And at the bottom of the dts file, although still in the 'videocore' block we
 * have:
 *
 * clock_setup {
 *  clock@PWM { freq = <3000000>; };
 * };
 *
 * To check whether the changes are correct you can use 'vcgencmd measure_clock 25'
 * This should return the value 3000000
 *
 * Also if you use wiringPi then you can do 'gpio readall' to check that the pin
 * alternate setting is set correctly.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fcntl.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/wait.h>
#include <linux/io.h>
#include <linux/ioctl.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/gpio/consumer.h>
#include <linux/pwm.h>
#include <asm-generic/ioctl.h>

#define DRIVER_NAME "in-pi556"

struct in_pi556_state {
	struct device          *dev;
    struct cdev            cdev;
    struct class           *cl;
    struct dma_chan        *dma_chan;
	dma_addr_t             dma_addr;

    struct pwm_device      *pwm;
    void __iomem           *ioaddr;
    phys_addr_t            phys_addr;

    uint8_t                *buffer;
    uint32_t               *pixbuf;

    unsigned char          brightness;
    u32                    num_leds;
};

/* Each LED is controlled with a 24 bit RGB value
 * each bit is created from a nibble of data either
 * 1000 or 1100 so to create 24 bits you need 12 bytes
 * of PWM output
 */
#define BYTES_PER_LED 12

// Number of 3MHz bits in 80us to create a reset condition
#define RESET_BYTES 32

#define ADDR_LED_MODE 0x80000000

#define PWM_FIFO 0x18

#define PWM_DMA_DREQ 5

static dev_t devid = MKDEV(1337, 0);

static int pwm_init(struct in_pi556_state * state)
{
	pwm_apply_args(state->pwm);
	state->pwm->flags = ADDR_LED_MODE;
	pwm_enable(state->pwm);

	return 0;
}

/*
 * DMA callback function, release the mapping and the calling function
 */
void in_pi556_callback(void * param)
{
	struct in_pi556_state * state = (struct in_pi556_state *) param;

	dma_unmap_single(state->dev, state->dma_addr, state->num_leds * BYTES_PER_LED,
		DMA_TO_DEVICE);
}

/*
 * Issue a DMA to the PWM peripheral from the assigned buffer
 * buffer must be unmapped again before being used
 */
int issue_dma(struct in_pi556_state * state, uint8_t *buffer, int length)
{
	struct dma_async_tx_descriptor *desc;

	state->dma_addr = dma_map_single(state->dev,
		buffer, length,
		DMA_TO_DEVICE);

	if(state->dma_addr == 0) {
		dev_err(state->dev, "Failed to map buffer for DMA\n");
		return -1;
	}

	desc = dmaengine_prep_slave_single(state->dma_chan, state->dma_addr,
		length, DMA_MEM_TO_DEV, DMA_PREP_INTERRUPT);

	if(desc == NULL) {
		dev_err(state->dev, "Failed to prep the DMA transfer\n");
		return -1;
	}

	desc->callback = in_pi556_callback;
	desc->callback_param = state;
	dmaengine_submit(desc);
	dma_async_issue_pending(state->dma_chan);

	return 0;
}


int clear_leds(struct in_pi556_state * state)
{
    int i;

    for(i = 0; i < state->num_leds * BYTES_PER_LED; i++) {
        state->buffer[i] = 0x88;
    }
    for(i = 0; i < RESET_BYTES; i++) {
        state->buffer[state->num_leds * BYTES_PER_LED + i] = 0;
    }

    issue_dma(state, state->buffer, state->num_leds * BYTES_PER_LED + RESET_BYTES);

	return 0;
}

static int in_pi556_open(struct inode *inode, struct file *file)
{
    struct in_pi556_state * state;
    state  = container_of(inode->i_cdev, struct in_pi556_state, cdev);

    file->private_data = state;

	return 0;
}

/* gamma correction
GammaE=255*(res/255).^(1/.45)
From: http://rgb-123.com/ws2812-color-output/
*/
unsigned char gamma_(unsigned char brightness, unsigned char val)
{
	int bright = val;
	unsigned char GammaE[] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2,
	2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5,
	6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 11, 11,
	11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18,
	19, 19, 20, 21, 21, 22, 22, 23, 23, 24, 25, 25, 26, 27, 27, 28,
	29, 29, 30, 31, 31, 32, 33, 34, 34, 35, 36, 37, 37, 38, 39, 40,
	40, 41, 42, 43, 44, 45, 46, 46, 47, 48, 49, 50, 51, 52, 53, 54,
	55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70,
	71, 72, 73, 74, 76, 77, 78, 79, 80, 81, 83, 84, 85, 86, 88, 89,
	90, 91, 93, 94, 95, 96, 98, 99,100,102,103,104,106,107,109,110,
	111,113,114,116,117,119,120,121,123,124,126,128,129,131,132,134,
	135,137,138,140,142,143,145,146,148,150,151,153,155,157,158,160,
	162,163,165,167,169,170,172,174,176,178,179,181,183,185,187,189,
	191,193,194,196,198,200,202,204,206,208,210,212,214,216,218,220,
	222,224,227,229,231,233,235,237,239,241,244,246,248,250,252,255};
	bright = (bright * brightness) / 255;
	return GammaE[bright];
}

// LED serial output
// 4 bits make up a single bit of the output
// 1 1 0 0  -- 1
// 1 0 0 0  -- 0
//
// Plus require a space of 80+ microseconds for reset
// 24 bits per led
//
// (24 * 4) / 8 = 12 bytes per led
unsigned char * led_encode(struct in_pi556_state * state, int rgb, unsigned char *buf)
{
	int i;
	unsigned char red = gamma_(state->brightness, rgb >> 16);
	unsigned char grn = gamma_(state->brightness, rgb >> 8);
	unsigned char blu = gamma_(state->brightness, rgb);
	int rearrange =  grn +
			(red << 8) +
			(blu << 16);
	for(i = 11; i >= 0; i--) {
		switch(rearrange & 3)
		{
			case 0: *buf++ = 0x88; break;
			case 1: *buf++ = 0x8c; break;
			case 2: *buf++ = 0xc8; break;
			case 3: *buf++ = 0xcc; break;
		}
		rearrange >>= 2;
	}

	return buf;
}

/* Write to the PWM through DMA
 * Function to write the RGB buffer to the IN-PI556 leds, the input buffer
 * contains a sequence of up to num_leds RGB32 integers, these are then
 * converted into the nibble per bit sequence required to drive the PWM
 */
ssize_t in_pi556_write(struct file *filp, const char __user *buf, size_t count, loff_t *pos)
{
	int32_t *p_rgb;
	int8_t * p_buffer;
	int i, length, num_leds;
    struct in_pi556_state * state = (struct in_pi556_state *) filp->private_data;

    num_leds = min(count / 4, state->num_leds);

	if(copy_from_user(state->pixbuf, buf, num_leds * 4))
		return -EFAULT;

	p_rgb = state->pixbuf;
	p_buffer = state->buffer;
	for(i = 0; i < num_leds; i++)
		p_buffer = led_encode(state, *p_rgb++, p_buffer);

    /* Fill rest with '0' */
	memset(p_buffer, 0x00, RESET_BYTES);

	length = (int) p_buffer - (int) state->buffer + RESET_BYTES;

	/* Setup DMA engine */
    issue_dma(state, state->buffer, length);

	return count;
}


struct file_operations in_pi556_ops = {
    .owner = THIS_MODULE,
    .llseek = NULL,
    .read = NULL,
    .write = in_pi556_write,
    .open = in_pi556_open,
    .release = NULL,
};

/*
 * Probe function
 */
static int in_pi556_probe(struct platform_device *pdev)
{
    int ret;
    struct device *dev = &pdev->dev;
    struct device_node *node = dev->of_node;
    struct in_pi556_state * state;
    struct dma_slave_config cfg = {
        .src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
        .dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
        .direction = DMA_MEM_TO_DEV,
        .src_addr = 0,
    };
    struct resource *iomem;

    if(node == NULL) {
        pr_err("Require device tree entry\n");
        goto fail;
    }

    state = kmalloc(sizeof(struct in_pi556_state), GFP_KERNEL);
    if (!state) {
    	dev_err(dev, "Can't allocate state\n");
        goto fail;
    }

    state->pwm = devm_pwm_get(&pdev->dev, NULL);
    if (IS_ERR(state->pwm)) {
    	ret = PTR_ERR(state->pwm);
    	if(ret != -EPROBE_DEFER) {
    		dev_err(dev, "Failed to request PWM channel");
    	}
    	kfree(state);
    	return ret;
    }

    state->dev = dev;
    state->brightness = 255;

    // Create character device interface /dev/in-pi556
    if(alloc_chrdev_region(&devid, 0, 1, "in-pi556") < 0) {
    	dev_err(dev, "Unable to create chrdev region");
        goto fail_malloc;
    }
    if((state->cl = class_create("in-pi556")) == NULL) {
        unregister_chrdev_region(devid, 1);
        dev_err(dev, "Unable to create class in-pi556");
        goto fail_chrdev;
    }

    if(device_create(state->cl, NULL, devid, NULL, "in-pi556") == NULL) {
        class_destroy(state->cl);
        unregister_chrdev_region(devid, 1);
        dev_err(dev, "Unable to create device in-pi556");
        goto fail_class;
    }

    state->cdev.owner = THIS_MODULE;
    cdev_init(&state->cdev, &in_pi556_ops);

    if(cdev_add(&state->cdev, devid, 1)) {
    	dev_err(dev, "CDEV failed\n");
        goto fail_device;
    }

    platform_set_drvdata(pdev, state);

    /* get parameters from device tree */
    of_property_read_u32(node,
                 "rpi,num_leds",
                 &state->num_leds);

    state->pixbuf = kmalloc(state->num_leds * sizeof(int), GFP_KERNEL);
    if(state->pixbuf == NULL) {
    	dev_err(dev, "Failed to allocate internal buffer\n");
		goto fail_cdev;
	}

    iomem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    state->phys_addr = iomem->start;

    state->buffer = kmalloc(state->num_leds * BYTES_PER_LED + RESET_BYTES, GFP_KERNEL);
    if(state->buffer == NULL) {
    	dev_err(dev, "Failed to allocate internal buffer\n");
		goto fail_pixbuf;
	}

    state->dma_chan = dma_request_chan(dev, "pwm_dma");
    if(state->dma_chan == NULL) {
    	dev_err(dev, "Failed to request DMA channel");
        goto fail_buffer;
    }
    /* request a DMA channel */
    cfg.dst_addr = state->phys_addr + PWM_FIFO;
    ret = dmaengine_slave_config(state->dma_chan, &cfg);
    if (ret < 0) {
    	dev_err(dev, "Can't allocate DMA channel\n");
        goto fail_dma_init;
    }

	pwm_init(state);

	clear_leds(state);

    return 0;

fail_dma_init:
    dma_release_channel(state->dma_chan);
fail_buffer:
	kfree(state->buffer);
fail_pixbuf:
    kfree(state->pixbuf);
fail_cdev:
    cdev_del(&state->cdev);
fail_device:
    device_destroy(state->cl, devid);
fail_class:
    class_destroy(state->cl);
fail_chrdev:
    unregister_chrdev_region(devid, 1);
fail_malloc:
    kfree(state);
fail:

    return -1;
}


static int in_pi556_remove(struct platform_device *pdev)
{
    struct in_pi556_state *state = platform_get_drvdata(pdev);

    platform_set_drvdata(pdev, NULL);

    pwm_disable(state->pwm);
    dma_release_channel(state->dma_chan);
    kfree(state->buffer);
    kfree(state->pixbuf);
    cdev_del(&state->cdev);
    device_destroy(state->cl, devid);
    class_destroy(state->cl);
    unregister_chrdev_region(devid, 1);
    kfree(state);

    return 0;
}

static const struct of_device_id in_pi556_match[] = {
    { .compatible = "rpi,in-pi556" },
    { }
};
MODULE_DEVICE_TABLE(of, in_pi556_match);

static struct platform_driver in_pi556 = {
    .probe      = in_pi556_probe,
    .remove     = in_pi556_remove,
    .driver     = {
        .name       = DRIVER_NAME,
        .owner      = THIS_MODULE,
        .of_match_table = in_pi556_match,
    },
};
module_platform_driver(in_pi556);

MODULE_DESCRIPTION("IN-PI556 PWM driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Gordon Hollingworth");
MODULE_AUTHOR("Artsiom Asadchy");
