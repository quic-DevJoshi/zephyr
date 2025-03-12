// Copyright, Qualcomm Invocation Center.

/*
 * Copyright (c) 2023, ithinx GmbH
 * Copyright (c) 2023, Tonies GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Emulator for bq40z50 fuel gauge
 */

#include <string.h>
#define DT_DRV_COMPAT ti_bq40z50

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(EMUL_BQ40Z50);

#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c_emul.h>
#include <zephyr/sys/byteorder.h>

#include "bq40z50.h"

struct bq40z50_emul_data {
	uint16_t mac_cmd;
	uint16_t mac_blk_cmd;
};

/** Static configuration for the emulator */
struct bq40z50_emul_cfg {
	/** I2C address of emulator */
	uint16_t addr;
};

static int emul_bq40z70_buffer_read(int reg, uint8_t *buf, size_t len)
{
	const char manufacturer_name[] = "Texas Instruments";
	const char device_name[] = "BQ40Z50";
	const char device_chemistry[] = "LION";

	switch (reg) {
	case BQ40Z50_MANUFACTURERNAME:
		if (len < sizeof(manufacturer_name)) {
			return -EIO;
		}
		buf[0] = strlen(manufacturer_name);
		memcpy(&buf[1], manufacturer_name, strlen(manufacturer_name));
		break;

	case BQ40Z50_DEVICENAME:
		if (len < sizeof(device_name)) {
			return -EIO;
		}
		buf[0] = strlen(device_name);
		memcpy(&buf[1], device_name, strlen(device_name));
		break;

	case BQ40Z50_DEVICECHEMISTRY:
		if (len < sizeof(device_chemistry)) {
			return -EIO;
		}
		buf[0] = strlen(device_chemistry);
		memcpy(&buf[1], device_chemistry, strlen(device_chemistry));
		break;

	default:
		LOG_ERR("Buffer Read for reg 0x%x is not supported", reg);
		return -EIO;
	}

	return 0;
}

static int emul_bq27z746_write(const struct emul *target, uint8_t *buf, size_t len)
{
	struct bq40z50_emul_data *data = target->data;
	const uint8_t reg = buf[0];
	const uint8_t size = buf[1];
	switch (reg) {
	case BQ40Z50_MANUFACTURERACCESS:
		data->mac_cmd = sys_get_le16(&buf[1]);
		break;
	case BQ40Z50_MANUFACTURERBLOCKACCESS:
		// with block access, 1 byte will be 0x44, 2nd byte will be size (discard?), then
		// two bytes cmd and rest data(we don't support that).
		data->mac_blk_cmd = sys_get_le16(&buf[2]);
		break;
	default:
		LOG_ERR("Writing is only supported to ALTMAC currently");
		return -EIO;
	}

	return 0;
}

static int emul_bq27z746_reg_read(const struct emul *target, int reg, int *val)
{
	switch (reg) {
	case BQ40Z50_MANUFACTURERACCESS:
		*val = 1;
		break;
	case BQ40Z50_ATRATE:
		*val = 0;
		break;
	case BQ40Z50_ATRATETIMETOEMPTY:
		*val = 0xFFFF;
		break;
	case BQ40Z50_TEMPERATURE:
		*val = 2980;
		break;
	case BQ40Z50_VOLTAGE:
		*val = 1;
		break;
	case BQ40Z50_BATTERYSTATUS:
		*val = 1;
		break;
	case BQ40Z50_CURRENT:
		*val = 1;
		break;
	case BQ40Z50_REMAININGCAPACITY:
		*val = 1;
		break;
	case BQ40Z50_FULLCHARGECAPACITY:
		*val = 1;
		break;
	case BQ40Z50_AVERAGECURRENT:
		*val = 1;
		break;
	case BQ40Z50_AVERAGETIMETOEMPTY:
		*val = 0xFFFF;
		break;
	case BQ40Z50_ATRATETIMETOFULL:
		*val = 0xFFFF;
		break;
	case BQ40Z50_BTPDISCHARGE:
		*val = 150;
		break;
	case BQ40Z50_BTPCHARGE:
		*val = 175;
		break;
	case BQ40Z50_CYCLECOUNT:
		*val = 1;
		break;
	case BQ40Z50_RELATIVESTATEOFCHARGE:
		*val = 100;
		break;
	case BQ40Z50_CHARGINGVOLTAGE:
		*val = 1;
		break;
	case BQ40Z50_CHARGINGCURRENT:
		*val = 1;
		break;
	case BQ40Z50_DESIGNCAPACITY:
		*val = 1;
		break;

	case BQ40Z50_RUNTIMETOEMPTY:
		*val = 0xFFFF;
		break;

	default:
		LOG_ERR("Unknown register 0x%x read", reg);
		return -EIO;
	}
	LOG_INF("read 0x%x = 0x%x", reg, *val);

	return 0;
}

static int emul_bq27z746_read(const struct emul *target, int reg, uint8_t *buf, size_t len)
{
	struct bq40z50_emul_data *data = target->data;
	if (len <= 2) {
		unsigned int val;
		int rc = emul_bq27z746_reg_read(target, reg, &val);

		if (rc) {
			return rc;
		}

		sys_put_le16(val, buf);
	} else {
		switch (reg) {
		case BQ40Z50_MANUFACTURERACCESS:
			LOG_DBG("Reading %u byte from ALTMAC", len);
			sys_put_le16(data->mac_cmd, buf);
			break;
		case BQ40Z50_MANUFACTURERNAME:
		case BQ40Z50_DEVICECHEMISTRY:
		case BQ40Z50_DEVICENAME:
			return emul_bq40z70_buffer_read(reg, buf, len);
		default:
			LOG_ERR("Reading is only supported from ALTMAC currently");
			return -EIO;
		}
	}

	return 0;
}

static int bq40z50_emul_transfer_i2c(const struct emul *target, struct i2c_msg *msgs, int num_msgs,
				     int addr)
{
	int reg;
	int rc;

	__ASSERT_NO_MSG(msgs && num_msgs);

	i2c_dump_msgs_rw(target->dev, msgs, num_msgs, addr, false);
	switch (num_msgs) {
	case 1:
		if (msgs->flags & I2C_MSG_READ) {
			LOG_ERR("Unexpected read");
			return -EIO;
		}

		return emul_bq27z746_write(target, msgs->buf, msgs->len);
	case 2:
		if (msgs->flags & I2C_MSG_READ) {
			LOG_ERR("Unexpected read");
			return -EIO;
		}
		if (msgs->len != 1) {
			LOG_ERR("Unexpected msg0 length %d", msgs->len);
			return -EIO;
		}
		reg = msgs->buf[0];

		/* Now process the 'read' part of the message */
		msgs++;
		if (msgs->flags & I2C_MSG_READ) {
			rc = emul_bq27z746_read(target, reg, msgs->buf, msgs->len);
			if (rc) {
				return rc;
			}
		} else {
			LOG_ERR("Second message must be an I2C write");
			return -EIO;
		}
		return rc;
	default:
		LOG_ERR("Invalid number of messages: %d", num_msgs);
		return -EIO;
	}

	return 0;
}

static const struct i2c_emul_api bq40z50_emul_api_i2c = {
	.transfer = bq40z50_emul_transfer_i2c,
};

/**
 * Set up a new emulator (I2C)
 *
 * @param emul Emulation information
 * @param parent Device to emulate
 * @return 0 indicating success (always)
 */
static int emul_bq40z50_init(const struct emul *target, const struct device *parent)
{
	ARG_UNUSED(target);
	ARG_UNUSED(parent);

	return 0;
}

/*
 * Main instantiation macro.
 */
#define BQ40Z50_EMUL(n)                                                                            \
	static struct bq40z50_emul_data bq40z50_emul_data_##n;                                     \
	static const struct bq40z50_emul_cfg bq40z50_emul_cfg_##n = {                              \
		.addr = DT_INST_REG_ADDR(n),                                                       \
	};                                                                                         \
	EMUL_DT_INST_DEFINE(n, emul_bq40z50_init, &bq40z50_emul_data_##n, &bq40z50_emul_cfg_##n,   \
			    &bq40z50_emul_api_i2c, NULL)

DT_INST_FOREACH_STATUS_OKAY(BQ40Z50_EMUL)
