/*
 * htrc110.c
 *
 * The HTRC110 is a HITAG Reader Chip from NXP.
 * Copyright (C) 2019 Artsiom Asadchy <artyomka111@gmail.com>
 *
 * The HTRC110 communicates with a host processor via a serial
 * interface (be careful, it's not a SPI interface, since it uses
 * 3 bits commands and such commands can't be send by SPI bus, so
 * we use bit banging). The complete datasheet is available at NXP's
 * website here:
 * https://www.nxp.com/docs/en/data-sheet/037031.pdf
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
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/bitops.h>

#define HTRC110_NUM_MINORS				1

#define HTRC110_DEFAULT_GAIN			0
#define HTRC110_ANTENNA_FAIL_MASK		0x10
#define HTRC110_OFFSET_COMPENSATION		0x3F

#define HTRC110_DEAD_TIME_BITS			2
#define HTRC110_BITS_NUM_TO_GET_HEADER	21
#define HTRC110_PSK_EDGE_PERIOD_US		76
#define HTRC110_OUT_OF_RANGE_PERIOD_US	720
#define HTRC110_HEADER_BITS_NUM			8
#define HTRC110_HEADER_BITS_LIMIT		128

#define HTRC110_NIBBLES_NUM				10

#define HTRC110_BIT_DELAY				50

#define HTRC110_TAG_ID_SIZE				5

#define HTRC110_TAG_READ_TIME_MS		100
#define HTRC110_TAG_POLL_TIME_MS		250

enum htrc110_modulation_type {
	HTRC110_MODULATION_MANCHESTER,
	HTRC110_MODULATION_BIPHASE
};

struct htrc110_modulation {
	enum htrc110_modulation_type type;
	int manchester_bit_counter;
	int biphase_bit_counter;
};

enum htrc110_decoder_state {
	HTRC110_DECODER_CAPTURE_INITIALIZE,
	HTRC110_DECODER_GET_BITRATE,
	HTRC110_DECODER_GET_HEADER,
	HTRC110_DECODER_GET_DATA,
	HTRC110_DECODER_CAPRUTE_FINISHED,
	HTRC110_DECODER_CAPTURE_SUCCESS,
	HTRC110_DECODER_CAPRUTE_FAILED
};

enum htrc110_bitrate {
	HTRC110_BITRATE_LOW,
	HTRC110_BITRATE_MIDDLE,
	HTRC110_BITRATE_HIGH
};

struct htrc110_decoder {
	int bit_counter;
	int edge_period;
	ktime_t prev_time;
	enum htrc110_decoder_state state;
	enum htrc110_bitrate bitrate;
	struct htrc110_modulation modulation;
};

enum htrc110_data_state {
	HTRC110_DATA_MORE_BITS_TO_GET,
	HTRC110_DATA_VALID,
	HTRC110_DATA_INVALID
};

struct htrc110_data {
	u8 nibble_idx;
	u8 bit_idx;
	u8 carry;
	u8 last_bit_state;
	u8 data_nibbles[HTRC110_NIBBLES_NUM + 1];
	u8 tag_id[HTRC110_TAG_ID_SIZE];
	enum htrc110_data_state state;
};

struct htrc110 {
	struct platform_device *pdev;
	struct gpio_desc *gpiod_dout;
	struct gpio_desc *gpiod_din;
	struct gpio_desc *gpiod_sclk;
	struct gpio_desc *gpiod_cs;
	int irq;
	struct list_head device_entry;
	dev_t devt;

	struct htrc110_decoder decoder;
	struct htrc110_data data;

	struct delayed_work dwork;
	struct mutex tag_data_lock;
	struct completion request_completion;
};

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static DECLARE_BITMAP(minors, HTRC110_NUM_MINORS);

#define htrc110_dout_high(ctx)	{gpiod_set_value(ctx->gpiod_dout, 1); udelay(HTRC110_BIT_DELAY);}
#define htrc110_dout_low(ctx)	{gpiod_set_value(ctx->gpiod_dout, 0); udelay(HTRC110_BIT_DELAY);}

#define htrc110_sclk_high(ctx)	{gpiod_set_value(ctx->gpiod_sclk, 1); udelay(2 * HTRC110_BIT_DELAY);}
#define htrc110_sclk_low(ctx)	{gpiod_set_value(ctx->gpiod_sclk, 0); udelay(HTRC110_BIT_DELAY);}

#define htrc110_cs_high(ctx)	{gpiod_set_value(ctx->gpiod_cs, 1); udelay(HTRC110_BIT_DELAY);}
#define htrc110_cs_low(ctx)		{gpiod_set_value(ctx->gpiod_cs, 0); udelay(HTRC110_BIT_DELAY);}

#define htrc110_din_get(ctx)	gpiod_get_value_cansleep(ctx->gpiod_din);

static inline void htrc110_init_transaction(struct htrc110 *ctx)
{
	htrc110_sclk_low(ctx);
	htrc110_dout_low(ctx);
	htrc110_sclk_high(ctx);
	htrc110_dout_high(ctx);
}

static void htrc110_transmit_byte(struct htrc110 *ctx, u8 data)
{
	int i;

	htrc110_init_transaction(ctx);

	for(i = 0; i < 8; i++) {
		htrc110_sclk_low(ctx);
		if(data & 0x80) {
			htrc110_dout_high(ctx);
		} else {
			htrc110_dout_low(ctx);
		}
		htrc110_sclk_high(ctx);
		data <<= 1;
	}

	htrc110_sclk_low(ctx);
}

static u8 htrc110_receive_byte(struct htrc110 *ctx)
{
	int i;
	u8 data = 0;

	for(i = 0; i < 8; i++) {
		data <<= 1;
		htrc110_sclk_low(ctx);
		udelay(HTRC110_BIT_DELAY);
		htrc110_sclk_high(ctx);
		data |= htrc110_din_get(ctx);
	}

	htrc110_sclk_low(ctx);

	return data;
}

static void htrc110_start_read_tag(struct htrc110 *ctx)
{
	int i;

	htrc110_init_transaction(ctx);

	for(i = 0; i < 3; i++) {
		htrc110_sclk_low(ctx);
		htrc110_sclk_high(ctx);
	}

	htrc110_sclk_low(ctx);
	htrc110_dout_low(ctx);
}

static void htrc110_end_read_tag(struct htrc110 *ctx)
{
	htrc110_sclk_high(ctx);
}

static void htrc110_power_down(struct htrc110 *ctx)
{
	/* SET_CONFIG_PAGE page=0x1 data=0x5 (PD_MODE=1[power down], PD=1[power down], HYSTERESIS=0[off], TXDIS=1[coil driver off]) */
	htrc110_transmit_byte(ctx, 0x5D);
}

static int htrc110_read_tag(struct htrc110 *ctx)
{
	u8 ans;
	u8 samp_time;

	/* SET_CONFIG_PAGE page=0x3 data=0x0 (DISLP1=0, DISSMARTCOMP=0, FSEL=10[12MHz osc]) */
	htrc110_transmit_byte(ctx, 0x72);

	/* SET_CONFIG_PAGE page=0x0 data=0xB (GAIN=##, FILTERH=1, FILTERL=1) */
	htrc110_transmit_byte(ctx, 0x43 | (HTRC110_DEFAULT_GAIN << 2));

	/* SET_CONFIG_PAGE page=0x2 data=0xF (THRESET=1, ACQAMP=1, FREEZE=11) */
	htrc110_transmit_byte(ctx, 0x6F);

	/* SET_CONFIG_PAGE page=0x1 data=0x5 (PD_MODE=0[active], PD=1[power down], HYSTERESIS=0[off], TXDIS=1[coil driver off]) */
	htrc110_transmit_byte(ctx, 0x55);

	/* SET_CONFIG_PAGE page=0x3 data=0x0 (DISLP1=0, DISSMARTCOMP=0, FSEL=10[12MHz osc]) */
	htrc110_transmit_byte(ctx, 0x72);

	/* SET_CONFIG_PAGE page=0x0 data=0xB (GAIN=##, FILTERH=1, FILTERL=1) */
	htrc110_transmit_byte(ctx, 0x43 | (HTRC110_DEFAULT_GAIN << 2));

	/* SET_CONFIG_PAGE page=0x1 data=0x0 (PD_MODE=0[active], PD=0[active], HYSTERESIS=0[off], TXDIS=0[coil driver on]) */
	htrc110_transmit_byte(ctx, 0x50);

	msleep(20);

	/* SET_CONFIG_PAGE page=0x2 data=0xB (THRESET=1, ACQAMP=0, FREEZE=11) */
	htrc110_transmit_byte(ctx, 0x6B);

	msleep(6);

	/* SET_CONFIG_PAGE page=0x2 data=0x8 (THRESET=1, ACQAMP=0, FREEZE=00[normal operation]) */
	htrc110_transmit_byte(ctx, 0x68);

	msleep(2);

	/* SET_CONFIG_PAGE page=0x2 data=0x0 (THRESET=0, ACQAMP=0, FREEZE=00) */
	htrc110_transmit_byte(ctx, 0x60);

	msleep(200);
	/* Check antenna status */
	htrc110_transmit_byte(ctx, 0x06);
	ans = htrc110_receive_byte(ctx);
	if(ans & HTRC110_ANTENNA_FAIL_MASK)
		return -EIO;

	/* Read phase */
	htrc110_transmit_byte(ctx, 0x08);
	mdelay(1);
	ans = htrc110_receive_byte(ctx);
	samp_time = ans << 1;
	samp_time += HTRC110_OFFSET_COMPENSATION;
	samp_time &= HTRC110_OFFSET_COMPENSATION;

	/* Set sampling time */
	htrc110_transmit_byte(ctx, 0x80 | samp_time);
	mdelay(1);
	htrc110_transmit_byte(ctx, 0x02);
	ans = htrc110_receive_byte(ctx);
	if(ans != samp_time)
		return -EIO;

	return 0;
}

static int htrc110_fops_open(struct inode *inode, struct file *filp)
{
	struct htrc110 *ctx;
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

static ssize_t htrc110_fops_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	ssize_t ret;
	unsigned long status;
	struct htrc110 *ctx = filp->private_data;

	if(count < HTRC110_TAG_ID_SIZE)
		return -EINVAL;

	if(mutex_lock_interruptible(&ctx->tag_data_lock) < 0)
		return -EINTR;

	reinit_completion(&ctx->request_completion);
	schedule_delayed_work(&ctx->dwork, 0);

	if(wait_for_completion_interruptible(&ctx->request_completion) < 0) {
		cancel_delayed_work_sync(&ctx->dwork);
		mutex_unlock(&ctx->tag_data_lock);
		return -EINTR;
	}

	status = copy_to_user(buf, ctx->data.tag_id, HTRC110_TAG_ID_SIZE);
	mutex_unlock(&ctx->tag_data_lock);

	if(status == 0) {
		ret = HTRC110_TAG_ID_SIZE;
	} else {
		ret = -EAGAIN;
	}

	return ret;
}

static const struct file_operations htrc110_fops = {
	.owner = THIS_MODULE,
	.open = htrc110_fops_open,
	.read = htrc110_fops_read,
};

static inline void htrc110_init_decoder(struct htrc110_decoder *decoder)
{
	decoder->bit_counter = 0;
	decoder->edge_period = INT_MAX;
	decoder->prev_time = 0;
	decoder->bitrate = HTRC110_BITRATE_LOW;
	decoder->state = HTRC110_DECODER_CAPTURE_INITIALIZE;
	decoder->modulation.manchester_bit_counter = 0;
	decoder->modulation.biphase_bit_counter = 0;
}

static inline void htrc110_reset_data(struct htrc110_data *data)
{
	memset(data, 0x00, sizeof(*data));
	data->last_bit_state = 1;
}

static inline enum htrc110_bitrate htrc110_get_bitrate(int edge_time, int time_difference)
{
	enum htrc110_bitrate ret;
	int quater_edge_time = edge_time >> 2;

	if(time_difference < (edge_time + quater_edge_time))
	{
		ret = HTRC110_BITRATE_LOW;
	} else if(time_difference < (edge_time + 3 * quater_edge_time)) {
		ret = HTRC110_BITRATE_MIDDLE;
	} else {
		ret = HTRC110_BITRATE_HIGH;
	}

	return ret;
}

static enum htrc110_data_state htrc110_handle_bit(struct htrc110_data *data)
{
	bool odd;
	enum htrc110_data_state ret = HTRC110_DATA_MORE_BITS_TO_GET;

	if(data->nibble_idx > HTRC110_NIBBLES_NUM)
		return HTRC110_DATA_VALID;

	if(data->bit_idx < 4) {
		/* Write bits to the nibble */
		if(data->last_bit_state)
			data->data_nibbles[data->nibble_idx] |= (0x08 >> data->bit_idx);

		data->bit_idx++;
	} else {
		if(data->nibble_idx < HTRC110_NIBBLES_NUM) {
			/* Check parity */
			odd = hweight8(data->data_nibbles[data->nibble_idx]) % 2;
			if((odd && !data->last_bit_state) || (!odd && data->last_bit_state)) {
				ret = HTRC110_DATA_INVALID;
			}
		} else {
			/* Handle the last bit */
			if(data->last_bit_state) {
				/* Error, stop bit should be low */
				ret = HTRC110_DATA_INVALID;
			}
		}
		data->bit_idx = 0;
		data->nibble_idx++;
	}

	return ret;
}

static irqreturn_t htrc110_irq(int irq, void *dev_id)
{
	struct htrc110 *ctx = dev_id;
	struct htrc110_decoder *decoder = &ctx->decoder;
	struct htrc110_data *data = &ctx->data;
	ktime_t curr_time = ktime_get();
	int time_difference = ((int)(curr_time - decoder->prev_time)) / 1000;

	decoder->prev_time = curr_time;

	/* Determine the bit type */
	if(decoder->state > HTRC110_DECODER_GET_BITRATE)
	{
		decoder->bitrate = htrc110_get_bitrate(decoder->edge_period, time_difference);
	}

	switch(decoder->state)
	{
		case HTRC110_DECODER_CAPTURE_INITIALIZE:
		{
			decoder->prev_time = ktime_get();
			decoder->state = HTRC110_DECODER_GET_BITRATE;
			htrc110_reset_data(data);
			break;
		}

		case HTRC110_DECODER_GET_BITRATE:
		{
			/* Avoid garbage from HTRC110 after start */
			if(decoder->bit_counter > HTRC110_DEAD_TIME_BITS) {
				/* Determine the shortest time between to edges to get bitrate */
				if(time_difference < decoder->edge_period) {
					decoder->edge_period = time_difference;
				}
			}

			if(decoder->bit_counter++ > HTRC110_BITS_NUM_TO_GET_HEADER) {
				/* Check whether modulation is PSK */
				if(decoder->edge_period < HTRC110_PSK_EDGE_PERIOD_US) {
					/* PSK is not used by typical tags. It's also too difficult to decode,
					 * so just fallthrough to the fail */
					decoder->state = HTRC110_DECODER_CAPRUTE_FAILED;
					break;
				}

				/* Check whether bit rate out of range */
				if(decoder->edge_period > HTRC110_OUT_OF_RANGE_PERIOD_US) {
					decoder->state = HTRC110_DECODER_CAPRUTE_FAILED;
					htrc110_end_read_tag(ctx);
					break;
				}

				decoder->state = HTRC110_DECODER_GET_HEADER;
				decoder->bit_counter = 0;
				break;
			}

			decoder->bit_counter++;
			break;
		}

		case HTRC110_DECODER_GET_HEADER:
		{
			/* Wait for the 9 high bits header */

			if(decoder->bitrate == HTRC110_BITRATE_LOW) {
				decoder->modulation.manchester_bit_counter++;
			} else {
				decoder->modulation.manchester_bit_counter = 0;
			}

			if(decoder->bitrate == HTRC110_BITRATE_HIGH) {
				decoder->modulation.biphase_bit_counter++;
			} else {
				decoder->modulation.biphase_bit_counter = 0;
			}

			if(HTRC110_HEADER_BITS_NUM == decoder->modulation.manchester_bit_counter) {
				decoder->modulation.type = HTRC110_MODULATION_MANCHESTER;
				decoder->bit_counter = 0;
				decoder->state = HTRC110_DECODER_GET_DATA;
				break;
			}

			if(HTRC110_HEADER_BITS_NUM == decoder->modulation.biphase_bit_counter) {
				decoder->modulation.type = HTRC110_MODULATION_BIPHASE;
				decoder->bit_counter = 0;
				decoder->state = HTRC110_DECODER_GET_DATA;
				break;
			}

			if(decoder->bit_counter > HTRC110_HEADER_BITS_LIMIT) {
				/* Something went wrong and we didn't find a header */
				decoder->state = HTRC110_DECODER_CAPRUTE_FAILED;
				break;
			}

			break;
		}

		case HTRC110_DECODER_GET_DATA:
		{
			if(decoder->modulation.type == HTRC110_MODULATION_MANCHESTER) {
				if(decoder->bitrate == HTRC110_BITRATE_LOW) {
					data->state = htrc110_handle_bit(data);
					if(data->state != HTRC110_DATA_MORE_BITS_TO_GET) {
						decoder->state = HTRC110_DECODER_CAPRUTE_FINISHED;
						break;
					}
				} else if(decoder->bitrate == HTRC110_BITRATE_MIDDLE) {
					if(data->carry) {
						data->state = htrc110_handle_bit(data);
						if(data->state != HTRC110_DATA_MORE_BITS_TO_GET) {
							decoder->state = HTRC110_DECODER_CAPRUTE_FINISHED;
							break;
						}
						data->last_bit_state ^= 1;
						data->state = htrc110_handle_bit(data);
						if(data->state != HTRC110_DATA_MORE_BITS_TO_GET) {
							decoder->state = HTRC110_DECODER_CAPRUTE_FINISHED;
							break;
						}
						data->carry = 0;
					} else {
						data->last_bit_state ^= 1;
						data->state = htrc110_handle_bit(data);
						if(data->state != HTRC110_DATA_MORE_BITS_TO_GET) {
							decoder->state = HTRC110_DECODER_CAPRUTE_FINISHED;
							break;
						}
						data->carry = 1;
					}
				} else if(decoder->bitrate == HTRC110_BITRATE_HIGH) {
					data->last_bit_state ^= 1;
					data->state = htrc110_handle_bit(data);
					if(data->state != HTRC110_DATA_MORE_BITS_TO_GET) {
						decoder->state = HTRC110_DECODER_CAPRUTE_FINISHED;
						break;
					}
					data->last_bit_state ^= 1;
					data->state = htrc110_handle_bit(data);
					if(data->state != HTRC110_DATA_MORE_BITS_TO_GET) {
						decoder->state = HTRC110_DECODER_CAPRUTE_FINISHED;
						break;
					}
				}
			} else {
				if(data->carry) {
					if(decoder->bitrate == HTRC110_BITRATE_LOW) {
						data->last_bit_state = 0;
						data->state = htrc110_handle_bit(data);
						if(data->state != HTRC110_DATA_MORE_BITS_TO_GET) {
							decoder->state = HTRC110_DECODER_CAPRUTE_FINISHED;
							break;
						}
					} else if(decoder->bitrate == HTRC110_BITRATE_MIDDLE) {
						data->last_bit_state = 0;
						data->state = htrc110_handle_bit(data);
						if(data->state != HTRC110_DATA_MORE_BITS_TO_GET) {
							decoder->state = HTRC110_DECODER_CAPRUTE_FINISHED;
							break;
						}
						data->last_bit_state = 1;
						data->state = htrc110_handle_bit(data);
						if(data->state != HTRC110_DATA_MORE_BITS_TO_GET) {
							decoder->state = HTRC110_DECODER_CAPRUTE_FINISHED;
							break;
						}
						data->carry = 0;
					} else if(decoder->bitrate == HTRC110_BITRATE_HIGH) {
						/* This case is not possible */
						decoder->state = HTRC110_DECODER_CAPRUTE_FINISHED;
						break;
					}
				} else {
					if(decoder->bitrate == HTRC110_BITRATE_LOW) {
						data->last_bit_state = 0;
						data->state = htrc110_handle_bit(data);
						if(data->state != HTRC110_DATA_MORE_BITS_TO_GET) {
							decoder->state = HTRC110_DECODER_CAPRUTE_FINISHED;
							break;
						}
					} else if(decoder->bitrate == HTRC110_BITRATE_MIDDLE) {
						data->last_bit_state = 1;
						data->state = htrc110_handle_bit(data);
						if(data->state != HTRC110_DATA_MORE_BITS_TO_GET) {
							decoder->state = HTRC110_DECODER_CAPRUTE_FINISHED;
							break;
						}
						data->carry = 1;
					} else if(decoder->bitrate == HTRC110_BITRATE_HIGH) {
						data->last_bit_state = 1;
						data->state = htrc110_handle_bit(data);
						if(data->state != HTRC110_DATA_MORE_BITS_TO_GET) {
							decoder->state = HTRC110_DECODER_CAPRUTE_FINISHED;
							break;
						}
						data->last_bit_state = 1;
						data->state = htrc110_handle_bit(data);
						if(data->state != HTRC110_DATA_MORE_BITS_TO_GET) {
							decoder->state = HTRC110_DECODER_CAPRUTE_FINISHED;
							break;
						}
					}
				}
			}
			break;
		}

		case HTRC110_DECODER_CAPRUTE_FINISHED:
		{
			int i;

			htrc110_end_read_tag(ctx);
			if(data->state == HTRC110_DATA_VALID) {
				decoder->state = HTRC110_DECODER_CAPTURE_SUCCESS;
				for(i = 0; i < HTRC110_TAG_ID_SIZE; i++) {
					data->tag_id[i] = (data->data_nibbles[i * 2] << 4) | data->data_nibbles[i * 2 + 1];
				}
			} else {
				decoder->state = HTRC110_DECODER_CAPRUTE_FAILED;
			}
			break;
		}

		case HTRC110_DECODER_CAPRUTE_FAILED:
		{
			htrc110_end_read_tag(ctx);
			break;
		}

		default:
		{
			break;
		}
	}

	return IRQ_HANDLED;
}

static void htrc110_work(struct work_struct *work)
{
	struct htrc110 *ctx = container_of(work, struct htrc110, dwork.work);

	htrc110_init_decoder(&ctx->decoder);
	htrc110_read_tag(ctx);
	enable_irq(ctx->irq);
	htrc110_start_read_tag(ctx);

	mdelay(HTRC110_TAG_READ_TIME_MS);

	disable_irq(ctx->irq);
	htrc110_end_read_tag(ctx);
	htrc110_power_down(ctx);

	if(ctx->decoder.state == HTRC110_DECODER_CAPTURE_SUCCESS) {
		complete(&ctx->request_completion);
	} else {
		/* Read operation was failed, continue after some time */
		schedule_delayed_work(&ctx->dwork, msecs_to_jiffies(HTRC110_TAG_POLL_TIME_MS));
	}
}

static int htrc110_init_pins(struct device *dev, struct htrc110 *ctx)
{
	int ret;

	ctx->gpiod_dout = devm_gpiod_get_optional(dev, "dout", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->gpiod_dout)) {
		ret = PTR_ERR(ctx->gpiod_dout);
		dev_err(dev, "cannot get dout GPIO: %d\n", ret);
		return ret;
	}
	if(ctx->gpiod_dout == NULL)
		return -ENODEV;

	ctx->gpiod_din = devm_gpiod_get_optional(dev, "din", GPIOD_IN);
	if (IS_ERR(ctx->gpiod_din)) {
		ret = PTR_ERR(ctx->gpiod_din);
		dev_err(dev, "cannot get din GPIO: %d\n", ret);
		return ret;
	}
	if(ctx->gpiod_din == NULL)
		return -ENODEV;

	ctx->gpiod_sclk = devm_gpiod_get_optional(dev, "sclk", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->gpiod_sclk)) {
		ret = PTR_ERR(ctx->gpiod_sclk);
		dev_err(dev, "cannot get sclk GPIO: %d\n", ret);
		return ret;
	}
	if(ctx->gpiod_sclk == NULL)
		return -ENODEV;

	ctx->gpiod_cs = devm_gpiod_get_optional(dev, "cs", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->gpiod_cs)) {
		ret = PTR_ERR(ctx->gpiod_cs);
		dev_err(dev, "cannot get cs GPIO: %d\n", ret);
		return ret;
	}

	return 0;
}

static struct class *htrc110_class;
static unsigned int major_num;

static int htrc110_probe(struct platform_device *pdev)
{
	struct htrc110 *ctx;
	int ret;
	struct device *dev = &pdev->dev;
	unsigned long minor;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->pdev = pdev;

	ret = htrc110_init_pins(dev, ctx);
	if(ret < 0)
		return ret;

	ctx->irq = platform_get_irq(pdev, 0);
	if(ctx->irq >= 0) {
		ret = devm_request_threaded_irq(&pdev->dev, ctx->irq, htrc110_irq, NULL,
				IRQF_TRIGGER_RISING | IRQF_ONESHOT, dev_name(&pdev->dev), ctx);
		if(ret)
			return ret;
	} else {
		return ret;
	}

	disable_irq(ctx->irq);

	platform_set_drvdata(pdev, ctx);

	ctx->devt = MKDEV(major_num, 0);
	INIT_LIST_HEAD(&ctx->device_entry);

	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, HTRC110_NUM_MINORS);
	if(minor < HTRC110_NUM_MINORS) {
		dev = device_create(htrc110_class, dev, ctx->devt, ctx, "htrc110_dev");
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

	mutex_init(&ctx->tag_data_lock);
	init_completion(&ctx->request_completion);
	INIT_DELAYED_WORK(&ctx->dwork, htrc110_work);

	if(ctx->gpiod_cs)
		htrc110_cs_low(ctx);

	return 0;
}

static int htrc110_remove(struct platform_device *pdev)
{
	struct htrc110 *ctx = platform_get_drvdata(pdev);

	htrc110_end_read_tag(ctx);

	htrc110_power_down(ctx);

	if(ctx->gpiod_cs)
		htrc110_cs_high(ctx);

	mutex_lock(&device_list_lock);
	list_del(&ctx->device_entry);
	device_destroy(htrc110_class, ctx->devt);
	clear_bit(MINOR(ctx->devt), minors);
	mutex_unlock(&device_list_lock);

	complete_release(&ctx->request_completion);
	mutex_destroy(&ctx->tag_data_lock);

	return 0;
}


static const struct of_device_id htrc110_of_match[] = {
	{ .compatible = "nxp,htrc110" },
	{ }
};

MODULE_DEVICE_TABLE(of, htrc110_of_match);

static struct platform_driver htrc110_driver = {
	.driver = {
		.name	= "htrc110",
		.of_match_table	= of_match_ptr(htrc110_of_match),
	},
	.probe	= htrc110_probe,
	.remove = htrc110_remove,
};

static int __init htrc110_driver_init(void)
{
	int ret;

	major_num = register_chrdev(0, htrc110_driver.driver.name, &htrc110_fops);
	if (major_num < 0)
		return major_num;

	htrc110_class = class_create(THIS_MODULE, htrc110_driver.driver.name);
	if (IS_ERR(htrc110_class)) {
		ret = PTR_ERR(htrc110_class);
		goto chdev_destroy;
	}

	ret = platform_driver_register(&htrc110_driver);
	if(ret < 0)
		goto cl_destroy;

	return 0;

cl_destroy:
	class_destroy(htrc110_class);
chdev_destroy:
	unregister_chrdev(major_num, htrc110_driver.driver.name);

	return ret;
}

static void __exit htrc110_driver_exit(void)
{
	platform_driver_unregister(&htrc110_driver);
	class_destroy(htrc110_class);
	unregister_chrdev(major_num, htrc110_driver.driver.name);
}

module_init(htrc110_driver_init);
module_exit(htrc110_driver_exit);

MODULE_AUTHOR("Artsiom Asadchy");
MODULE_DESCRIPTION("HTRC110 driver");
MODULE_LICENSE("GPL");
