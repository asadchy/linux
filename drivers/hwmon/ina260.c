/*
 * ina260.c
 *
 * The INA260 is a precision digital current and power monitor with low-drift,
 * precision integrated shunt from TI.
 * Copyright (C) 2019 Artsiom Asadchy <artyomka111@gmail.com>
 *
 * The INA260 communicates with a host processor via an I2C interface.
 * The complete datasheet is available at NXP's website here:
 * http://www.ti.com/lit/ds/symlink/ina260.pdf
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/jiffies.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/util_macros.h>
#include <linux/regmap.h>

/* common register definitions */
#define INA260_CONFIG_REG			0x00
#define INA260_CURRENT_REG			0x01
#define INA260_BUS_VOLTAGE_REG		0x02
#define INA260_POWER_REG			0x03

#define INA260_RESET				0xF000

#define INA260_FULLCFG_MASK			0x8FFF

#define INA260_AVG_MASK				0x0E00
#define INA260_AVG_OFFSET			9
/* number of averages = 128 */
#define INA260_AVG_DEFAULT_VAL		4

/* bus voltage conversion time mask */
#define INA260_VBUSCT_MASK			0x01C0
#define INA260_VBUSCT_OFFSET		6

/* current conversion time mask */
#define INA260_ISHCT_MASK			0x0038
#define INA260_ISHCT_OFFSET			3

/* Do not use shift operation, since converted values are signed */
#define INA260_UNITS_TO_mVOLTS(x)	((x) + (x / 4))
#define INA260_UNITS_TO_mAMPS(x)	((x) + (x / 4))
#define INA260_UNITS_TO_mWATTS(x)	((x) * 10)

#define INA260_MAX_CFG_VAL			7

struct ina260 {
	struct regmap *regmap;
	const struct attribute_group *group;
};

static const struct regmap_config ina260_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = 8,
};

static int ina260_read_reg(struct ina260 *ctx, int reg, unsigned int *regval)
{
	return regmap_read(ctx->regmap, reg, regval);
}

static int ina260_write_reg(struct ina260 *ctx, int reg, unsigned int regval)
{
	return regmap_write(ctx->regmap, reg, regval);
}

static int ina260_update_reg(struct ina260 *ctx, int reg, unsigned int mask, unsigned int val)
{
	return regmap_update_bits(ctx->regmap, reg, mask, val);
}

static int ina260_reset(struct ina260 *ctx)
{
	return ina260_write_reg(ctx, INA260_CONFIG_REG, INA260_RESET);
}

static int ina260_get_value(int reg, unsigned int regval)
{
	int ret;

	switch(reg) {
	case INA260_CURRENT_REG:
		ret = INA260_UNITS_TO_mAMPS((s16)regval);
		break;

	case INA260_BUS_VOLTAGE_REG:
		ret = INA260_UNITS_TO_mVOLTS((s16)regval);
		break;

	case INA260_POWER_REG:
		ret = INA260_UNITS_TO_mWATTS((s16)regval);
		break;

	default:
		break;
	}

	return ret;
}

static ssize_t ina260_show_value(struct device *dev,
				 struct device_attribute *da, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	struct ina260 *ctx = dev_get_drvdata(dev);
	unsigned int regval;

	int ret = ina260_read_reg(ctx, attr->index, &regval);
	if (ret < 0)
		return ret;

	return snprintf(buf, PAGE_SIZE, "%d\n",
			ina260_get_value(attr->index, regval));
}

static int ina260_get_config(unsigned int mask, unsigned int regval)
{
	int ret;

	switch(mask) {
	case INA260_AVG_MASK:
		ret = (regval & mask) >> INA260_AVG_OFFSET;
		break;

	case INA260_VBUSCT_MASK:
		ret = (regval & mask) >> INA260_VBUSCT_OFFSET;
		break;

	case INA260_ISHCT_MASK:
		ret = (regval & mask) >> INA260_ISHCT_OFFSET;
		break;

	case INA260_FULLCFG_MASK:
		ret = (regval & mask);
		break;

	default:
		ret = -1;
		break;
	}

	return ret;
}

static ssize_t ina260_show_config(struct device *dev,
		 struct device_attribute *da, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	struct ina260 *ctx = dev_get_drvdata(dev);
	unsigned int regval;

	int ret = ina260_read_reg(ctx, INA260_CONFIG_REG, &regval);
	if(ret < 0)
		return ret;

	return snprintf(buf, PAGE_SIZE, "%d\n",
			ina260_get_config(attr->index, regval));
}

static int ina260_set_config(struct ina260 *ctx, unsigned int mask, unsigned int val)
{
	unsigned int regval;

	switch(mask) {
	case INA260_AVG_MASK:
		regval = val << INA260_AVG_OFFSET;
		break;

	case INA260_VBUSCT_MASK:
		regval = val << INA260_VBUSCT_OFFSET;
		break;

	case INA260_ISHCT_MASK:
		regval = val << INA260_ISHCT_OFFSET;
		break;

	default:
		return -EINVAL;
	}

	return ina260_update_reg(ctx, INA260_CONFIG_REG, mask, regval);
}

/* Averaging Mode
 * attr = 0, number of averages = 1
 * attr = 1, number of averages = 4
 * attr = 2, number of averages = 16
 * attr = 3, number of averages = 64
 * attr = 4, number of averages = 128
 * attr = 5, number of averages = 256
 * attr = 6, number of averages = 512
 * attr = 7, number of averages = 1024
 *
 * Bus voltage and shunt current conversion time
 * attr = 0, conversion time = 140 µs
 * attr = 1, conversion time = 204 µs
 * attr = 2, conversion time = 332 µs
 * attr = 3, conversion time = 588 µs
 * attr = 4, conversion time = 1.1 ms
 * attr = 5, conversion time = 2.116 ms
 * attr = 6, conversion time = 4.156 ms
 * attr = 7, conversion time = 8.244 ms
 */
static ssize_t ina260_store_config(struct device *dev, struct device_attribute *da,
				  const char *buf, size_t count)
{
	unsigned long val;
	int ret;
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	struct ina260 *ctx = dev_get_drvdata(dev);

	ret = kstrtoul(buf, 10, &val);
	if (ret < 0)
		return ret;

	if(val > INA260_MAX_CFG_VAL)
		return -EINVAL;

	ret = ina260_set_config(ctx, attr->index, val);
	if(ret)
		return ret;

	return count;
}

static SENSOR_DEVICE_ATTR(current_monitor, S_IRUGO, ina260_show_value, NULL, INA260_CURRENT_REG);

static SENSOR_DEVICE_ATTR(voltage_monitor, S_IRUGO, ina260_show_value, NULL, INA260_BUS_VOLTAGE_REG);

static SENSOR_DEVICE_ATTR(power_monitor, S_IRUGO, ina260_show_value, NULL, INA260_POWER_REG);

static SENSOR_DEVICE_ATTR(averaging_mode, S_IRUGO | S_IWUSR, ina260_show_config, ina260_store_config,
		INA260_AVG_MASK);

static SENSOR_DEVICE_ATTR(voltage_conv_time, S_IRUGO | S_IWUSR, ina260_show_config, ina260_store_config,
		INA260_VBUSCT_MASK);

static SENSOR_DEVICE_ATTR(current_conv_time, S_IRUGO | S_IWUSR, ina260_show_config, ina260_store_config,
		INA260_ISHCT_MASK);

static SENSOR_DEVICE_ATTR(configuration_register, S_IRUGO, ina260_show_config, NULL, INA260_FULLCFG_MASK);

/* pointers to created device attributes */
static struct attribute *ina260_attrs[] = {
	&sensor_dev_attr_current_monitor.dev_attr.attr,
	&sensor_dev_attr_voltage_monitor.dev_attr.attr,
	&sensor_dev_attr_power_monitor.dev_attr.attr,
	&sensor_dev_attr_averaging_mode.dev_attr.attr,
	&sensor_dev_attr_voltage_conv_time.dev_attr.attr,
	&sensor_dev_attr_current_conv_time.dev_attr.attr,
	&sensor_dev_attr_configuration_register.dev_attr.attr,
	NULL,
};

static const struct attribute_group ina260_group = {
	.attrs = ina260_attrs,
};

static int ina260_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct device *hwmon_dev;
	struct ina260 *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->group = &ina260_group;

	ctx->regmap = devm_regmap_init_i2c(client, &ina260_regmap_config);
	if (IS_ERR(ctx->regmap)) {
		dev_err(dev, "failed to allocate register map\n");
		return PTR_ERR(ctx->regmap);
	}

	hwmon_dev = devm_hwmon_device_register_with_groups(dev, client->name,
							   ctx, &ctx->group);

	ret = ina260_reset(ctx);
	if(ret)
		return ret;

	msleep(100);

	ret = ina260_update_reg(ctx, INA260_CONFIG_REG, INA260_AVG_MASK,
			INA260_AVG_DEFAULT_VAL << INA260_AVG_OFFSET);
	if(ret)
		return ret;

	if (IS_ERR(hwmon_dev))
		return PTR_ERR(hwmon_dev);

	return 0;
}

static const struct of_device_id ina260_of_match[] = {
	{ .compatible = "ti,ina260" },
	{ }
};

MODULE_DEVICE_TABLE(of, ina260_of_match);

static struct i2c_driver ina260_driver = {
	.driver = {
		.name	= "ina260",
		.of_match_table = of_match_ptr(ina260_of_match),
	},
	.probe		= ina260_probe,
};

module_i2c_driver(ina260_driver);

MODULE_AUTHOR("Artsiom Asadchy");
MODULE_DESCRIPTION("INA260 driver");
MODULE_LICENSE("GPL");
