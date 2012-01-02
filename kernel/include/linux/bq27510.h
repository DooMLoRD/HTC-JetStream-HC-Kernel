/*
 *
 * Copyright (C) 2009 HTC Corporation
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
#ifndef BQ27510_H
#define BQ27510_H

#include <linux/ubattery_alg.h>

#define BQ27510_REG_TEMP		0x06
#define BQ27510_REG_VOLT		0x08
#define BQ27510_REG_FLAGS		0x0A
#define BQ27510_REG_REMAINING_CAP	0x10
#define BQ27510_REG_FULL_CHARGE_CAP	0x12
#define BQ27510_REG_AI			0x14
#define BQ27510_REG_SOC			0x2C
#define BQ27510_REG_CONTROL		0x00


int bq27510_battery_param_update(struct battery_type *battery,	struct protect_flags_type *flags);

enum {
	HTC_BATT_DEBUG_UEVT = 1U << 1,
	HTC_BATT_DEBUG_USER_QUERY = 1U << 2,
	HTC_BATT_DEBUG_USB_NOTIFY = 1U << 3,
	HTC_BATT_DEBUG_FULL_LOG = 1U << 4,
};

enum {
	GUAGE_NONE,
	GUAGE_MODEM,
	GUAGE_DS2784,
	GUAGE_DS2746,
	GUAGE_BQ27510,
};

enum {
	LINEAR_CHARGER,
	SWITCH_CHARGER_TPS65200,
};

struct bq27510_device_info;
struct bq27510_access_methods {
	int (*read)(u8 reg, int *rt_value, int b_single,
		struct bq27510_device_info *di);
	int (*write)(u8 reg, u8 value,
		struct bq27510_device_info *di);
};

struct bq27510_device_info {
	struct device                   *dev;
	struct bq27510_access_methods   *bus;
	struct i2c_client               *client;
	int rup_dis;
};

struct bq27510_battery_platform_data {
	int guage_driver;
	int gpio_mbat_in;
	int charger;
	int (*func_battery_gpio_init)(void);
	int (*func_battery_charging_ctrl)(int ctl);
};

#endif
