/*
 * mfrc522.c
 *
 * The MFRC522 is a contactless reader/writer from NXP.
 * Copyright (C) 2019 Artsiom Asadchy <artyomka111@gmail.com>
 *
 * The MFRC522 communicates with a host processor via an SPI Bus
 * interface. The complete datasheet is available at NXP's website
 * here:
 * https://www.nxp.com/docs/en/data-sheet/MFRC522.pdf
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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/mod_devicetable.h>
#include <linux/spi/spi.h>
#include <linux/of_device.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>

#define MFRC522_READ_MASK			0x80
#define MFRC522_FIFO_DEPTH		    64
#define MFRC522_RX_DATA_MAX_LEN     18
#define MFRC522_BLOCK_SIZE			16
#define MFRC522_KEY_SIZE			6
#define MFRC522_CARD_TYPE_SIZE		2
#define MFRC522_SERIAL_SIZE			8
#define MFRC522_KEYS_NUM			2
#define MFRC522_4BYTEUID			4
#define MFRC522_7BYTEUID			7

#define MFRC522_KEY_A				0
#define MFRC522_KEY_B				1

#define MFRC522_PCD_IDLE            0x00    /* Cancle the current command */
#define MFRC522_PCD_AUTHENT         0x0E    /* Authentication key */
#define MFRC522_PCD_RECEIVE         0x08    /* Receive data */
#define MFRC522_PCD_TRANSMIT        0x04    /* To send data */
#define MFRC522_PCD_TRANSCEIVE      0x0C    /* To send and receive data */
#define MFRC522_PCD_RESETPHASE      0x0F    /* Reset */
#define MFRC522_PCD_CALCCRC         0x03    /* CRC calculation */

#define MFRC522_PICC_REQIDL         0x26    /* Looking for a IC card in the area of antenna, that
                                       	   	   didn't enter a dormant state the area of IC card */
#define MFRC522_PICC_REQALL         0x52    /* Looking for all IC card in the area of antena */
#define MFRC522_PICC_ANTICOLL1      0x93    /* Anticollision */
#define MFRC522_PICC_ANTICOLL2      0x95    /* Anticollision */
#define MFRC522_PICC_AUTHENT1A      0x60    /* Authentication A key */
#define MFRC522_PICC_AUTHENT1B      0x61    /* Authentication B key */
#define MFRC522_PICC_READ           0x30    /* Read block */
#define MFRC522_PICC_WRITE          0xA0    /* Write block */
#define MFRC522_PICC_DECREMENT      0xC0    /* Deductions */
#define MFRC522_PICC_INCREMENT      0xC1    /* Rechange */
#define MFRC522_PICC_RESTORE        0xC2    /* The block of data transferrend to the buffer */
#define MFRC522_PICC_TRANSFER       0xB0    /* Save a data in a buffer */
#define MFRC522_PICC_HALT           0x50    /* dormancy */

/* Registers map */
/* PAGE 0 */
#define MFRC522_RFU00				0x00
#define MFRC522_COMMAND_REG         0x01
#define MFRC522_COM_IEN_REG         0x02
#define MFRC522_DIVL_EN_REG         0x03
#define MFRC522_COM_IRQ_REG         0x04
#define MFRC522_DIV_IRQ_REG         0x05
#define MFRC522_ERROR_REG           0x06
#define MFRC522_STATUS1_REG         0x07
#define MFRC522_STATUS2_REG         0x08
#define MFRC522_FIFO_DATA_REG       0x09
#define MFRC522_FIFO_LEVEL_REG      0x0A
#define MFRC522_WATER_LEVEL_REG     0x0B
#define MFRC522_CONTROL_REG         0x0C
#define MFRC522_BIT_FRAMING_REG     0x0D
#define MFRC522_COLL_REG            0x0E
#define MFRC522_RFU0F               0x0F

/* PAGE 1 */
#define MFRC522_RFU10               0x10
#define MFRC522_MODE_REG            0x11
#define MFRC522_TX_MODE_REG         0x12
#define MFRC522_RX_MODE_REG         0x13
#define MFRC522_TX_CONTROL_REG      0x14
#define MFRC522_TX_AUTO_REG         0x15
#define MFRC522_TX_SEL_REG          0x16
#define MFRC522_RX_SEL_REG          0x17
#define MFRC522_RX_THRESHOLD_REG    0x18
#define MFRC522_DEMOD_REG           0x19
#define MFRC522_RFU1A               0x1A
#define MFRC522_RFU1B               0x1B
#define MFRC522_MIFARE_REG          0x1C
#define MFRC522_RFU1D               0x1D
#define MFRC522_RFU1E               0x1E
#define MFRC522_SERIAL_SPEED_REG    0x1F

/* PAGE 2 */
#define MFRC522_RFU20               0x20
#define MFRC522_CRC_RESULT_REG_H    0x21
#define MFRC522_CRC_RESULT_REG_L    0x22
#define MFRC522_RFU23               0x23
#define MFRC522_MOD_WIDTH_REG       0x24
#define MFRC522_RFU25               0x25
#define MFRC522_RF_CFG_REG          0x26
#define MFRC522_GS_NREG             0x27
#define MFRC522_CWGS_CFG_REG        0x28
#define MFRC522_MOD_GS_CFG_REG      0x29
#define MFRC522_T_MODE_REG          0x2A
#define MFRC522_T_PRESCALER_REG     0x2B
#define MFRC522_T_RELOAD_REG_H      0x2C
#define MFRC522_T_RELOAD_REG_L      0x2D
#define MFRC522_T_COUN_VALUE_REGH   0x2E
#define MFRC522_T_COUN_VALUE_REGL   0x2F

/* PAGE 3 */
#define MFRC522_RFU30               0x30
#define MFRC522_TEST_SEL1_REG       0x31
#define MFRC522_TEST_SEL2_REG       0x32
#define MFRC522_TEST_PIN_EN_REG     0x33
#define MFRC522_TEST_PIN_VALUE_REG  0x34
#define MFRC522_TEST_BUS_REG        0x35
#define MFRC522_AUTO_TEST_REG       0x36
#define MFRC522_VERSION_REG         0x37
#define MFRC522_ANALOG_TEST_REG     0x38
#define MFRC522_TEST_DAC1_REG       0x39
#define MFRC522_TEST_DAC2_REG       0x3A
#define MFRC522_TEST_ADC_REG        0x3B
#define MFRC522_RFU3C               0x3C
#define MFRC522_RFU3D               0x3D
#define MFRC522_RFU3E               0x3E
#define MFRC522_RFU3F               0x3F

#define MFRC522_NUM_MINORS			1

#define MIFARE_UID_SIZE_MASK		0x40

#define MFRC522_KBITS_TO_BLOCKS(x)	((x) * 8)

#define MFRC522_IOC_MAGIC			'a'

#define MFRC522_IOC_SET_BL_ADDR		1
#define MFRC522_IOC_GET_BL_ADDR		2
#define MFRC522_IOC_SET_KEYA		3
#define MFRC522_IOC_SET_KEYB		4
#define MFRC522_IOC_GET_KEYA		5
#define MFRC522_IOC_GET_KEYB		6
#define MFRC522_IOC_SET_KEY			7
#define MFRC522_IOC_GET_KEY			8
#define MFRC522_IOC_SET_RO_ID		9
#define MFRC522_IOC_GET_RO_ID		10

#define MFRC522_IOCSBLADDR			_IOW(MFRC522_IOC_MAGIC, MFRC522_IOC_SET_BL_ADDR, int)
#define MFRC522_IOCGBLADDR			_IOR(MFRC522_IOC_MAGIC, MFRC522_IOC_GET_BL_ADDR, int)

#define MFRC522_IOCSKEYA			_IOW(MFRC522_IOC_MAGIC, MFRC522_IOC_SET_KEYA, char*)
#define MFRC522_IOCSKEYB			_IOW(MFRC522_IOC_MAGIC, MFRC522_IOC_SET_KEYB, char*)
#define MFRC522_IOCGKEYA			_IOR(MFRC522_IOC_MAGIC, MFRC522_IOC_GET_KEYA, char*)
#define MFRC522_IOCGKEYB			_IOR(MFRC522_IOC_MAGIC, MFRC522_IOC_GET_KEYB, char*)

#define MFRC522_IOCSKEY				_IOW(MFRC522_IOC_MAGIC, MFRC522_IOC_SET_KEY, int)
#define MFRC522_IOCGKEY				_IOR(MFRC522_IOC_MAGIC, MFRC522_IOC_GET_KEY, int)

#define MFRC522_IOCSROID			_IOW(MFRC522_IOC_MAGIC, MFRC522_IOC_SET_RO_ID, int)
#define MFRC522_IOCGROID			_IOR(MFRC522_IOC_MAGIC, MFRC522_IOC_GET_RO_ID, int)

#define MFRC522_WRITE_BUFFER        512

struct mfrc522 {
	struct spi_device *spi;
	struct gpio_desc *gpiod_rst;
	struct delayed_work dwork;
	struct mutex tag_data_lock;
	struct completion request_completion;

	struct list_head device_entry;
	dev_t devt;

	u8 block_data[MFRC522_BLOCK_SIZE];
	int blocks_num;
	u8 key[MFRC522_KEYS_NUM][MFRC522_KEY_SIZE];
	int key_idx;
	int block_addr;
	int read_only_id;
};

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static DECLARE_BITMAP(minors, MFRC522_NUM_MINORS);

static const u8 mfrc522_auth_cmd[MFRC522_KEYS_NUM] = {MFRC522_PICC_AUTHENT1A, MFRC522_PICC_AUTHENT1B};

static int mfrc522_write_reg_seq(struct mfrc522 *ctx, u8 reg, u8 *data, size_t len)
{
	u8 wr_data[MFRC522_WRITE_BUFFER];

	reg <<= 1;

	memcpy(&wr_data, &reg, 1);
	memcpy(&wr_data[1], data, len);

	return spi_write(ctx->spi, wr_data, len + 1);
}

static int mfrc522_write_reg(struct mfrc522 *ctx, u8 reg, u8 data)
{
	return mfrc522_write_reg_seq(ctx, reg, &data, 1);
}

static int mfrc522_read_reg_seq(struct mfrc522 *ctx, u8 reg, u8 *data, size_t len)
{
	size_t index;
	int ret;

	reg <<= 1;
	reg |= MFRC522_READ_MASK;

	for(index = 0; index < len; index++) {
		ret = spi_write_then_read(ctx->spi, &reg, 1, &data[index], 1);
		if(ret < 0)
			return ret;
	}

	return len;
}

static int mfrc522_read_reg(struct mfrc522 *ctx, u8 reg, u8 *data)
{
	return mfrc522_read_reg_seq(ctx, reg, data, 1);
}

static int mfrc522_set_register_bitmask(struct mfrc522 *ctx, u8 reg, u8 mask)
{
	u8 data;
	int ret;

	ret = mfrc522_read_reg(ctx, reg, &data);
	if(ret < 0)
		return ret;

	data |= mask;

	return mfrc522_write_reg(ctx, reg, data);
}

static int mfrc522_clear_register_bitmask(struct mfrc522 *ctx, u8 reg, u8 mask)
{
	u8 data;
	int ret;

	ret = mfrc522_read_reg(ctx, reg, &data);
	if(ret < 0)
		return ret;

	data &= (~mask);

	return mfrc522_write_reg(ctx, reg, data);
}

static int mfrc522_calculate_crc(struct mfrc522 *ctx, u8 *input, size_t len, u16 *crc)
{
	int ret;
	u8 read_data;

	ret = mfrc522_write_reg(ctx, MFRC522_COMMAND_REG, MFRC522_PCD_IDLE);
	if(ret < 0)
		return ret;

	ret = mfrc522_write_reg(ctx, MFRC522_DIV_IRQ_REG, 0x04);
	if(ret < 0)
		return ret;

	ret = mfrc522_write_reg(ctx, MFRC522_FIFO_LEVEL_REG, 0x80);
	if(ret < 0)
		return ret;

	ret = mfrc522_write_reg_seq(ctx, MFRC522_FIFO_DATA_REG, input, len);
	if(ret < 0)
		return ret;

	ret = mfrc522_write_reg(ctx, MFRC522_COMMAND_REG, MFRC522_PCD_CALCCRC);
	if(ret < 0)
		return ret;

	msleep(30);

	ret = mfrc522_read_reg(ctx, MFRC522_DIV_IRQ_REG, &read_data);
	if(ret < 0)
		return ret;

	if(read_data & 0x04) {
		ret = mfrc522_write_reg(ctx, MFRC522_COMMAND_REG, MFRC522_PCD_IDLE);
		if(ret < 0)
			return ret;

		ret = mfrc522_read_reg(ctx, MFRC522_CRC_RESULT_REG_H, &read_data);
		if(ret < 0)
			return ret;

		*crc = read_data;
		*crc <<= 8;

		ret = mfrc522_read_reg(ctx, MFRC522_CRC_RESULT_REG_L, &read_data);
		if(ret < 0)
			return ret;

		*crc |= read_data;

		return 0;
	} else {
		return -1;
	}
}

static int mfrc522_rfid_com(struct mfrc522 *ctx, u8 cmd, u8 *idata, size_t ilen, u8 *odata, size_t *olen)
{
    u8 irq_en;
    u8 wait_for;
    u8 last_bits;
    u8 reg_val;
    u8 com_irq_reg;
    int ret;

    switch(cmd)
    {
        case MFRC522_PCD_AUTHENT:
            irq_en      = 0x12;
            wait_for    = 0x10;
            break;
        case MFRC522_PCD_TRANSCEIVE:
            irq_en      = 0x77;
            wait_for    = 0x30;
            break;
        default:
            return -1;
    }

    ret = mfrc522_write_reg(ctx, MFRC522_COM_IEN_REG, irq_en | 80);
	if(ret < 0)
		return ret;

    ret = mfrc522_clear_register_bitmask(ctx, MFRC522_COM_IRQ_REG, 0x80);
	if(ret < 0)
		return ret;

    ret = mfrc522_write_reg(ctx, MFRC522_COMMAND_REG, MFRC522_PCD_IDLE);
	if(ret < 0)
		return ret;

    ret = mfrc522_set_register_bitmask(ctx, MFRC522_FIFO_LEVEL_REG, 0x80);
	if(ret < 0)
		return ret;

    ret = mfrc522_write_reg_seq(ctx, MFRC522_FIFO_DATA_REG, idata, ilen);
	if(ret < 0)
		return ret;

    ret = mfrc522_write_reg(ctx, MFRC522_COMMAND_REG, cmd);
	if(ret < 0)
		return ret;

    if(MFRC522_PCD_TRANSCEIVE == cmd) {
    	ret = mfrc522_set_register_bitmask(ctx, MFRC522_BIT_FRAMING_REG, 0x80);
    	if(ret < 0)
    		return ret;
    }

    msleep(35);

    mfrc522_read_reg(ctx, MFRC522_COM_IRQ_REG, &com_irq_reg);
    if(!(com_irq_reg & wait_for))
    	return -EAGAIN;

    mfrc522_clear_register_bitmask(ctx, MFRC522_BIT_FRAMING_REG, 0x80);

    mfrc522_read_reg(ctx, MFRC522_ERROR_REG, &reg_val);
	if(!(reg_val & 0x1B)) {
		if(MFRC522_PCD_TRANSCEIVE == cmd) {
			ret = mfrc522_read_reg(ctx, MFRC522_FIFO_LEVEL_REG, &reg_val);
	    	if(ret < 0) {
	    		return ret;
    		}

			ret = mfrc522_read_reg(ctx, MFRC522_CONTROL_REG, &last_bits);
	    	if(ret < 0) {
	    		return ret;
	    	}

			last_bits = last_bits & 0x07;

			if(last_bits) {
				*olen = (reg_val - 1) * 8 + last_bits;
			} else {
				*olen = reg_val * 8;
			}

			if(0 == reg_val){
				reg_val = 1;
			}

			if(reg_val > MFRC522_RX_DATA_MAX_LEN){
				reg_val = MFRC522_RX_DATA_MAX_LEN;
			}

			ret = mfrc522_read_reg_seq(ctx, MFRC522_FIFO_DATA_REG, odata, reg_val);
	    	if(ret < 0)
	    		return ret;
		}
	} else {
		return -EAGAIN;
	}

	ret = mfrc522_set_register_bitmask(ctx, MFRC522_CONTROL_REG, 0x80);
	if(ret < 0)
		return ret;

	return mfrc522_write_reg(ctx, MFRC522_COMMAND_REG, MFRC522_PCD_IDLE);
}

static int mfrc522_rfid_request (struct mfrc522 *ctx, u8 req_code, u8 *card_type)
{
	int ret;
    size_t len;
    u8 com_buf[MFRC522_RX_DATA_MAX_LEN] = {0, 0};

    ret = mfrc522_clear_register_bitmask(ctx, MFRC522_STATUS2_REG, 0x08);
	if(ret < 0)
		return ret;

    ret = mfrc522_write_reg(ctx, MFRC522_BIT_FRAMING_REG, 0x07);
	if(ret < 0)
		return ret;

    ret = mfrc522_set_register_bitmask(ctx, MFRC522_TX_CONTROL_REG, 0x03);
	if(ret < 0)
		return ret;

    com_buf[0] = req_code;

    ret = mfrc522_rfid_com(ctx, MFRC522_PCD_TRANSCEIVE, com_buf, 1, com_buf, &len);
	if(ret < 0)
		return ret;

	*card_type = com_buf[0];
	*(card_type + 1) = com_buf[1];

    return ret;
}

static int mfrc522_rfid_anticoll(struct mfrc522 *ctx, u8 select, u8 *serial_num)
{
    int ret;
    u8 i;
    u8 sernum_check = 0;
    size_t len;
    u8 com_buf[MFRC522_RX_DATA_MAX_LEN];

    ret = mfrc522_clear_register_bitmask(ctx, MFRC522_STATUS2_REG, 0x08);
	if(ret < 0)
		return ret;

    ret = mfrc522_write_reg(ctx, MFRC522_BIT_FRAMING_REG, 0x00);
	if(ret < 0)
		return ret;

    ret = mfrc522_clear_register_bitmask(ctx, MFRC522_COLL_REG, 0x80);
	if(ret < 0)
		return ret;

    com_buf[0] = select;
    com_buf[1] = 0x20;

    ret = mfrc522_rfid_com(ctx, MFRC522_PCD_TRANSCEIVE, com_buf, 2, com_buf, &len);
	if(ret < 0)
		return ret;

	for(i = 0; i < 4; i++) {
		serial_num[i] = com_buf[i];
		sernum_check ^= com_buf[i];
	}

    return mfrc522_set_register_bitmask(ctx, MFRC522_COLL_REG, 0x80);
}

static int mfrc522_rfid_select(struct mfrc522 *ctx, u8 select, u8 *serial_num)
{
    int ret;
    int i;
    size_t len;
    u8 com_buf[MFRC522_RX_DATA_MAX_LEN];
    u16 crc;

    com_buf[0] = select;
    com_buf[1] = 0x70;
    com_buf[6] = 0;

    for(i = 0; i < 4; i++) {
        com_buf[i + 2]  = serial_num[i];
        com_buf[6] ^= serial_num[i];
    }

    ret = mfrc522_calculate_crc(ctx, com_buf, 7, &crc);
    if(ret < 0)
    	return ret;

    com_buf[7] = crc;
    com_buf[8] = crc >> 8;

    ret = mfrc522_clear_register_bitmask(ctx, MFRC522_STATUS2_REG, 0x08);
    if(ret < 0)
    	return ret;

    ret = mfrc522_rfid_com(ctx, MFRC522_PCD_TRANSCEIVE, com_buf, 9, com_buf, &len);
	if(ret == 0) {
		ret = com_buf[0];
	} else {
		ret = -EAGAIN;
	}

    return ret;
}

static int mfrc522_rfid_read(struct mfrc522 *ctx, u8 block_addr, u8 *data)
{
    int ret;
    size_t len;
    u8 com_buf[MFRC522_RX_DATA_MAX_LEN];
    u16 crc;

    com_buf[0] = MFRC522_PICC_READ;
    com_buf[1] = block_addr;

    ret = mfrc522_calculate_crc(ctx, com_buf, 2, &crc);
    if(ret < 0)
    	return ret;

    com_buf[2] = crc;
    com_buf[3] = crc >> 8;

    ret = mfrc522_rfid_com(ctx, MFRC522_PCD_TRANSCEIVE, com_buf, 4, com_buf, &len);
    if((ret == 0) && (len == 0x90)) {
        memcpy(data, com_buf, MFRC522_BLOCK_SIZE);
    }
    else {
        ret = -EAGAIN;
    }

    return ret;
}

static int mfrc522_rfid_write(struct mfrc522 *ctx, u8 block_addr, u8 *data)
{
    int ret;
    size_t len;
    u8 com_buf[MFRC522_RX_DATA_MAX_LEN];
    u16 crc;

    com_buf[0] = MFRC522_PICC_WRITE;
    com_buf[1] = block_addr;

    ret = mfrc522_calculate_crc(ctx, com_buf, 2, &crc);
    if(ret < 0)
    	return ret;

    com_buf[2] = crc;
    com_buf[3] = crc >> 8;

    ret = mfrc522_rfid_com(ctx, MFRC522_PCD_TRANSCEIVE, com_buf, 4, com_buf, &len);
    if((ret == 0) && (len == 0x04) && ((com_buf[0] & 0x0F) == 0x0A)) {
        memcpy(com_buf, data, MFRC522_BLOCK_SIZE);

        ret = mfrc522_calculate_crc(ctx, com_buf, MFRC522_BLOCK_SIZE, &crc);
        if(ret < 0)
        	return ret;

        com_buf[16] = crc;
        com_buf[17] = crc >> 8;

        ret = mfrc522_rfid_com(ctx, MFRC522_PCD_TRANSCEIVE, com_buf, MFRC522_RX_DATA_MAX_LEN, com_buf, &len);
        if(ret < 0) {
        	ret = -EAGAIN;
        }
    }
    else {
        ret = -EAGAIN;
    }

    return ret;
}

static int mfrc522_rfid_setup_ssuid(struct mfrc522 *ctx, u8 *card_rev, u8 *card_serial)
{
	int ret;

	ret = mfrc522_rfid_anticoll(ctx, MFRC522_PICC_ANTICOLL1, card_serial);
	if(ret < 0)
		return ret;

	return mfrc522_rfid_select(ctx, MFRC522_PICC_ANTICOLL1, card_serial);
}

static int mfrc522_rfid_setup_dsuid(struct mfrc522 *ctx, u8 *card_rev, u8 *card_serial)
{
	int ret;

	ret = mfrc522_rfid_anticoll(ctx, MFRC522_PICC_ANTICOLL1, card_serial);
	if(ret < 0)
		return ret;

	ret = mfrc522_rfid_select(ctx, MFRC522_PICC_ANTICOLL1, card_serial);
	if(ret < 0)
		return ret;

	ret = mfrc522_rfid_anticoll(ctx, MFRC522_PICC_ANTICOLL2, &card_serial[4]);
	if(ret < 0)
		return ret;

	return ret = mfrc522_rfid_select(ctx, MFRC522_PICC_ANTICOLL2, &card_serial[4]);;
}

static int mfrc522_rfid_auth_state(struct mfrc522 *ctx, u8 auth_mode, u8 addr, u8 *key, u8 *serial_num)
{
    int ret;
    size_t len;
    u8 reg_val;
    u8 com_buf[MFRC522_RX_DATA_MAX_LEN];

    com_buf[0] = auth_mode;
    com_buf[1] = addr;

    memcpy(&com_buf[2], key, 6);
    memcpy(&com_buf[8], serial_num, 4);

    ret = mfrc522_rfid_com(ctx, MFRC522_PCD_AUTHENT, com_buf, 12, com_buf, &len);
	if(ret < 0)
		return ret;

    ret = mfrc522_read_reg(ctx, MFRC522_STATUS2_REG, &reg_val);
    if(ret < 0)
    	return ret;

    if(!(reg_val & 0x08))
    	ret = -EAGAIN;

    return ret;
}

static int mfrc522_rfid_halt(struct mfrc522 *ctx)
{
	int ret;
    size_t len;
    u8 com_buf[MFRC522_RX_DATA_MAX_LEN];
    u16 crc;

    com_buf[0] = MFRC522_PICC_HALT;
    com_buf[1] = 0;

    ret = mfrc522_calculate_crc(ctx, com_buf, 2, &crc);
    if(ret < 0)
    	return ret;

    com_buf[2] = crc;
    com_buf[3] = crc >> 8;

    return mfrc522_rfid_com(ctx, MFRC522_PCD_TRANSCEIVE, com_buf, 4, com_buf, &len);
}

static int mfrc522_rfid_auth(struct mfrc522 *ctx, u8 block_addr)
{
	int ret;
	u8 card_type[MFRC522_CARD_TYPE_SIZE];
	u8 card_serial[MFRC522_SERIAL_SIZE];
	u8 *card_serial_ptr;

	ret = mfrc522_rfid_request(ctx, MFRC522_PICC_REQIDL, card_type);
	if(ret >= 0) {
		if(card_type[0] & MIFARE_UID_SIZE_MASK) {
			ret = mfrc522_rfid_setup_dsuid(ctx, card_type, card_serial);
			card_serial_ptr = &card_serial[4];
		} else {
			ret = mfrc522_rfid_setup_ssuid(ctx, card_type, card_serial);
			card_serial_ptr = card_serial;
		}
		if(ret >= 0) {
			ctx->blocks_num = MFRC522_KBITS_TO_BLOCKS(ret);
			ret = mfrc522_rfid_auth_state(ctx, mfrc522_auth_cmd[ctx->key_idx], block_addr,
					ctx->key[ctx->key_idx], card_serial_ptr);
		}
	}

	return ret;
}

static int mfrc522_rfid_read_block(struct mfrc522 *ctx, u8 block_addr, u8 *data)
{
	int ret = mfrc522_rfid_auth(ctx, block_addr);
	if(ret >= 0) {
		if(block_addr < ctx->blocks_num) {
			ret = mfrc522_rfid_read(ctx, block_addr, data);
		} else {
			ret = -EINVAL;
		}
	}
	mfrc522_rfid_halt(ctx);

	return ret;
}

static int mfrc522_rfid_read_uid(struct mfrc522 *ctx)
{
	int ret;
	u8 card_type[MFRC522_CARD_TYPE_SIZE];
	u8 card_serial[MFRC522_SERIAL_SIZE];

	ret = mfrc522_rfid_request(ctx, MFRC522_PICC_REQIDL, card_type);
	if(ret >= 0) {
		if(card_type[0] & MIFARE_UID_SIZE_MASK) {
			ret = mfrc522_rfid_setup_dsuid(ctx, card_type, card_serial);
			if(ret >= 0) {
				memset(ctx->block_data, 0x00, MFRC522_BLOCK_SIZE);
				memcpy(ctx->block_data, &card_serial[1], MFRC522_7BYTEUID);
				ret = 0;
			}
		} else {
			ret = mfrc522_rfid_setup_ssuid(ctx, card_type, card_serial);
			if(ret >= 0) {
				memset(ctx->block_data, 0x00, MFRC522_BLOCK_SIZE);
				memcpy(ctx->block_data, card_serial, MFRC522_4BYTEUID);
				ret = 0;
			}
		}
	}
	mfrc522_rfid_halt(ctx);

	return ret;
}

static int mfrc522_rfid_write_block(struct mfrc522 *ctx, u8 block_addr, u8 *data)
{
	int ret = mfrc522_rfid_auth(ctx, block_addr);
	if(ret >= 0) {
		if(block_addr < ctx->blocks_num) {
			ret = mfrc522_rfid_write(ctx, block_addr, data);
		} else {
			ret = -EINVAL;
		}
	}
	mfrc522_rfid_halt(ctx);

	return ret;
}

static int mfrc522_antenna_on(struct mfrc522 *ctx)
{
	return mfrc522_set_register_bitmask(ctx, MFRC522_TX_CONTROL_REG, 0x03);
}

static int mfrc522_antenna_off(struct mfrc522 *ctx)
{
	return mfrc522_clear_register_bitmask(ctx, MFRC522_TX_CONTROL_REG, 0x03);
}

static void mfrc522_set_default_parameters(struct mfrc522 *ctx)
{
	ctx->block_addr = 0;
	ctx->key_idx = MFRC522_KEY_A;
	ctx->read_only_id = 1;
	memset(ctx->key, 0xFF, sizeof(ctx->key));
}

static int mfrc522_init(struct mfrc522 *ctx)
{
	int ret;

	/* perform hard reset */
	if(ctx->gpiod_rst) {
		gpiod_set_value_cansleep(ctx->gpiod_rst, 0);
		msleep(1);
		gpiod_set_value_cansleep(ctx->gpiod_rst, 1);
		msleep(50);
	}

	ret = mfrc522_write_reg(ctx, MFRC522_COMMAND_REG, MFRC522_PCD_RESETPHASE);
	if(ret < 0)
		return ret;
    msleep(10);

    ret = mfrc522_write_reg(ctx, MFRC522_MODE_REG, 0x3D);
	if(ret < 0)
		return ret;

    ret = mfrc522_write_reg(ctx, MFRC522_T_RELOAD_REG_L, 30);
	if(ret < 0)
		return ret;

    ret = mfrc522_write_reg(ctx, MFRC522_T_RELOAD_REG_H, 0);
	if(ret < 0)
		return ret;


    ret = mfrc522_write_reg(ctx, MFRC522_T_MODE_REG, 0x8D);
	if(ret < 0)
		return ret;

    ret = mfrc522_write_reg(ctx, MFRC522_T_PRESCALER_REG, 0x3E);
	if(ret < 0)
		return ret;

    ret = mfrc522_write_reg(ctx, MFRC522_TX_AUTO_REG, 0x40);
	if(ret < 0)
		return ret;

	ret = mfrc522_antenna_off(ctx);
	if(ret < 0)
		return ret;

    ret = mfrc522_clear_register_bitmask(ctx, MFRC522_STATUS2_REG, 0X08);
	if(ret < 0)
		return ret;

    ret = mfrc522_write_reg(ctx, MFRC522_MODE_REG, 0x3D);
	if(ret < 0)
		return ret;

    ret = mfrc522_write_reg(ctx, MFRC522_RX_SEL_REG, 0x86);
	if(ret < 0)
		return ret;

    ret = mfrc522_write_reg(ctx, MFRC522_RF_CFG_REG, 0x5f);
	if(ret < 0)
		return ret;

    ret = mfrc522_write_reg(ctx, MFRC522_T_RELOAD_REG_L, 30);
	if(ret < 0)
		return ret;

    ret = mfrc522_write_reg(ctx, MFRC522_T_RELOAD_REG_H, 0);
	if(ret < 0)
		return ret;

    ret = mfrc522_write_reg(ctx, MFRC522_T_MODE_REG, 0x8D);
	if(ret < 0)
		return ret;

    ret = mfrc522_write_reg(ctx, MFRC522_T_PRESCALER_REG, 0x3E);
	if(ret < 0)
		return ret;

    msleep(10);

    return mfrc522_antenna_on(ctx);
}

static void mfrc522_work(struct work_struct *work)
{
	int ret;
	unsigned long delay;

	struct mfrc522 *ctx = container_of(work, struct mfrc522, dwork.work);

	if(ctx->read_only_id) {
		ret = mfrc522_rfid_read_uid(ctx);
	} else {
		ret = mfrc522_rfid_read_block(ctx, ctx->block_addr, ctx->block_data);
	}

	if(ret == 0) {
		complete(&ctx->request_completion);
	} else {
		delay = 250; /* 250 msec */
		schedule_delayed_work(&ctx->dwork, msecs_to_jiffies(delay));
	}
}

static int mfrc522_fops_open(struct inode *inode, struct file *filp)
{
	struct mfrc522 *ctx;
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

static long mfrc522_fops_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	struct mfrc522 *ctx = filp->private_data;

	if(mutex_lock_interruptible(&ctx->tag_data_lock) < 0)
		return -EINTR;

	switch(cmd)
	{
		case MFRC522_IOCSBLADDR:
		{
			ret = copy_from_user(&ctx->block_addr, (const void*)arg, sizeof(ctx->block_addr));
			break;
		}

		case MFRC522_IOCGBLADDR:
		{
			ret = copy_to_user((void*)arg, &ctx->block_addr, sizeof(ctx->block_addr));
			break;
		}

		case MFRC522_IOCSKEYA:
		{
			ret = copy_from_user(&ctx->key[MFRC522_KEY_A], (const void*)arg, MFRC522_KEY_SIZE);
			break;
		}

		case MFRC522_IOCSKEYB:
		{
			ret = copy_from_user(&ctx->key[MFRC522_KEY_B], (const void*)arg, MFRC522_KEY_SIZE);
			break;
		}

		case MFRC522_IOCGKEYA:
		{
			ret = copy_to_user((void*)arg, &ctx->key[MFRC522_KEY_A], MFRC522_KEY_SIZE);
			break;
		}

		case MFRC522_IOCGKEYB:
		{
			ret = copy_to_user((void*)arg, &ctx->key[MFRC522_KEY_B], MFRC522_KEY_SIZE);
			break;
		}

		case MFRC522_IOCSKEY:
		{
			ret = copy_from_user(&ctx->key_idx, (const void*)arg, sizeof(ctx->key_idx));
			break;
		}

		case MFRC522_IOCGKEY:
		{
			ret = copy_to_user((void*)arg, &ctx->key_idx, sizeof(ctx->key_idx));
			break;
		}

		case MFRC522_IOCSROID:
		{
			ret = copy_from_user(&ctx->read_only_id, (const void*)arg, sizeof(ctx->read_only_id));
			break;
		}

		case MFRC522_IOCGROID:
		{
			ret = copy_to_user((void*)arg, &ctx->read_only_id, sizeof(&ctx->read_only_id));
			break;
		}

		default:
		{
			ret = -ENOTTY;
		}
	}
	mutex_unlock(&ctx->tag_data_lock);

	return ret;
}

static ssize_t mfrc522_fops_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	ssize_t ret;
	unsigned long status;
	struct mfrc522 *ctx = filp->private_data;

	if(count < sizeof(ctx->block_data))
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

	status = copy_to_user(buf, ctx->block_data, sizeof(ctx->block_data));
	mutex_unlock(&ctx->tag_data_lock);

	if(status == 0) {
		ret = sizeof(ctx->block_data);
	} else {
		ret = -EAGAIN;
	}

	return ret;
}

static ssize_t mfrc522_fops_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	ssize_t ret;
	int status;
	struct mfrc522 *ctx = filp->private_data;
	u8 mfrc522_data[MFRC522_BLOCK_SIZE];

	if(count < MFRC522_BLOCK_SIZE)
		return -EINVAL;

	if(copy_from_user(mfrc522_data, buf, sizeof(mfrc522_data)) != 0) {
		return -EAGAIN;
	}

	if(mutex_lock_interruptible(&ctx->tag_data_lock) < 0)
		return -EINTR;

	status = mfrc522_rfid_write_block(ctx, ctx->block_addr, mfrc522_data);
	if(status < 0) {
		ret = status;
	} else {
		ret = count;
	}

	mutex_unlock(&ctx->tag_data_lock);

	return ret;
}

static const struct file_operations mfrc522_fops = {
	.owner = THIS_MODULE,
	.open = mfrc522_fops_open,
	.read = mfrc522_fops_read,
	.write = mfrc522_fops_write,
	.unlocked_ioctl = mfrc522_fops_ioctl,
};

static struct class *mfrc522_class;
static unsigned int major_num;

static int mfrc522_probe(struct spi_device *spi)
{
	struct mfrc522 *ctx;
	int ret;
	struct device *dev = &spi->dev;
	unsigned long minor;

	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0;
	ret = spi_setup(spi);
	if (ret < 0)
		return ret;

	ctx = devm_kzalloc(&spi->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->spi = spi;

	mfrc522_set_default_parameters(ctx);

	ctx->gpiod_rst = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->gpiod_rst)) {
		ret = PTR_ERR(ctx->gpiod_rst);
		dev_err(dev, "cannot get reset GPIO: %d\n", ret);
		return ret;
	}

	spi_set_drvdata(spi, ctx);

	ret = mfrc522_init(ctx);
	if(ret < 0)
		goto disable_mfrc522;

	ctx->devt = MKDEV(major_num, 0);

	mutex_init(&ctx->tag_data_lock);
	init_completion(&ctx->request_completion);

	INIT_DELAYED_WORK(&ctx->dwork, mfrc522_work);

	INIT_LIST_HEAD(&ctx->device_entry);

	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, MFRC522_NUM_MINORS);
	if(minor < MFRC522_NUM_MINORS) {
		dev = device_create(mfrc522_class, dev, ctx->devt, ctx, "mfrc522_dev");
		ret = PTR_ERR_OR_ZERO(dev);
	} else {
		ret = -ENODEV;
	}
	if(!ret) {
		set_bit(minor, minors);
		list_add(&ctx->device_entry, &device_list);
	}
	mutex_unlock(&device_list_lock);

	if(ret)
		goto tag_sync_release;

	return 0;

tag_sync_release:
	complete_release(&ctx->request_completion);
	mutex_destroy(&ctx->tag_data_lock);
disable_mfrc522:
	if(ctx->gpiod_rst)
		gpiod_set_value_cansleep(ctx->gpiod_rst, 0);
	return ret;
}

static void mfrc522_remove(struct spi_device *spi)
{
	struct mfrc522 *ctx = spi_get_drvdata(spi);

	if(ctx->gpiod_rst) {
		gpiod_set_value_cansleep(ctx->gpiod_rst, 0);
	}

	mutex_lock(&device_list_lock);
	list_del(&ctx->device_entry);
	device_destroy(mfrc522_class, ctx->devt);
	clear_bit(MINOR(ctx->devt), minors);
	mutex_unlock(&device_list_lock);

	complete_release(&ctx->request_completion);
	mutex_destroy(&ctx->tag_data_lock);

	mfrc522_antenna_off(ctx);
}

static const struct of_device_id mfrc522_of_match[] = {
	{ .compatible = "nxp,mfrc522" },
	{ }
};

MODULE_DEVICE_TABLE(of, mfrc522_of_match);

static struct spi_driver mfrc522_driver = {
	.driver = {
		.name	= "mfrc522",
		.of_match_table	= of_match_ptr(mfrc522_of_match),
	},
	.probe	= mfrc522_probe,
	.remove = mfrc522_remove,
};

static int __init mfrc522_driver_init(void)
{
	int ret;

	major_num = register_chrdev(0, mfrc522_driver.driver.name, &mfrc522_fops);
	if (major_num < 0)
		return major_num;

	mfrc522_class = class_create(mfrc522_driver.driver.name);
	if (IS_ERR(mfrc522_class)) {
		ret = PTR_ERR(mfrc522_class);
		goto chdev_destroy;
	}

	ret = spi_register_driver(&mfrc522_driver);
	if(ret < 0)
		goto cl_destroy;

	return 0;

cl_destroy:
	class_destroy(mfrc522_class);
chdev_destroy:
	unregister_chrdev(major_num, mfrc522_driver.driver.name);

	return ret;
}

static void __exit mfrc522_driver_exit(void)
{
	spi_unregister_driver(&mfrc522_driver);
	class_destroy(mfrc522_class);
	unregister_chrdev(major_num, mfrc522_driver.driver.name);
}

module_init(mfrc522_driver_init);
module_exit(mfrc522_driver_exit);

MODULE_AUTHOR("Artsiom Asadchy");
MODULE_DESCRIPTION("MFRC522 driver");
MODULE_LICENSE("GPL");
