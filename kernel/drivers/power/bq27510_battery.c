/*
 * BQ27510 battery driver
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

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/bq27510.h>
#include <mach/board.h>
#include <linux/rtc.h>
#include <linux/notifier.h>
#include <linux/wakelock.h>
#include <linux/gpio.h>
#include <linux/ubattery_alg.h>
#include <mach/htc_battery_core.h>

#define BQ27510_RETRY_COUNT 10

static int htc_batt_debug_mask = HTC_BATT_DEBUG_UEVT |
				HTC_BATT_DEBUG_USER_QUERY;
module_param_named(debug_mask, htc_batt_debug_mask,
			int, S_IRUGO | S_IWUSR | S_IWGRP);

static struct bq27510_device_info *di;
struct bq27510_battery_info {
	struct mutex info_lock;
	int gpio_mbat_in;
};

static struct bq27510_battery_info bq27510_batt_info;

static int bq27510_write_i2c(u8 reg, u8 value,
			    struct bq27510_device_info *di)
{
	struct i2c_client *client = di->client;
	u8 buf[2];
	struct i2c_msg xfer_msg;
	int err;

	if (!client->adapter)
		return -ENODEV;

	/* [MSG1] fill the register address data and fill the data Tx buffer */
	xfer_msg.addr = client->addr;
	xfer_msg.len = 2;
	xfer_msg.flags = 0;
	xfer_msg.buf = buf;

	buf[0] = reg;
	buf[1] = value;

	err = i2c_transfer(client->adapter, &xfer_msg, 1);
	if (err < 0){
		printk(DRIVER_ZONE "[%s] fail.\n", __func__);
		return err;
	}
	return 0;
}

static int bq27x00_write(u8 reg, u8 value, struct bq27510_device_info *di)
{
	int loop_i;
	int ret=0;

	for (loop_i = 0; loop_i < BQ27510_RETRY_COUNT; loop_i++) {
		ret = di->bus->write(reg, value, di);
		if (ret == 0)
			break;
		mdelay(10);
	}


	return ret;

}


static int bq27510_read_i2c_not_bsingle(u8 reg, int *rt_value,  struct bq27510_device_info *di)
{
	unsigned char data[2];
	int err;
	struct i2c_client *client = di->client;
	struct i2c_msg msgs[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = 1,
		 .buf = data,
		 },
		{
		 .addr = client->addr,
		 .flags = I2C_M_RD,
		 .len = 2,
		 .buf = data,
		 },
	};

	if (!client->adapter)
		return -ENODEV;

		data[0] = reg;

	err = i2c_transfer(client->adapter, msgs, 2);

		if (err >= 0) {
				*rt_value = get_unaligned_le16(data);

			return 0;
		}
	return err;
}


static int bq27510_read_i2c_bsingle(u8 reg, int *rt_value,
			    struct bq27510_device_info *di)
{
	unsigned char data[2];
	int err;
	struct i2c_client *client = di->client;
	struct i2c_msg msgs[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = 1,
		 .buf = data,
		 },
		{
		 .addr = client->addr,
		 .flags = I2C_M_RD,
		 .len = 1,
		 .buf = data,
		 },
	};

	if (!client->adapter)
		return -ENODEV;

		data[0] = reg;

	err = i2c_transfer(client->adapter, msgs, 2);

		if (err >= 0) {
				*rt_value = data[0];

			return 0;
		}
	return err;
}

#if 1  // re-write separate I2C transaction into one
static int bq27510_read_i2c(u8 reg, int *rt_value, int b_single,
			    struct bq27510_device_info *di)
{

		if (!b_single)
			return bq27510_read_i2c_not_bsingle(reg, rt_value, di);
		else
			return bq27510_read_i2c_bsingle(reg, rt_value, di);
}

#else
static int bq27510_read_i2c(u8 reg, int *rt_value, int b_single,
			    struct bq27510_device_info *di)
{
	struct i2c_client *client = di->client;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int err;

	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 1;
	msg->buf = data;

	data[0] = reg;
	err = i2c_transfer(client->adapter, msg, 1);

	if (err >= 0) {
		if (!b_single)
			msg->len = 2;
		else
			msg->len = 1;

		msg->flags = I2C_M_RD;
		err = i2c_transfer(client->adapter, msg, 1);
		if (err >= 0) {
			if (!b_single)
				*rt_value = get_unaligned_le16(data);
			else
				*rt_value = data[0];

			return 0;
		}
	}
	return err;
}

#endif

//sff
static int bq27x00_read(u8 reg, int *rt_value, int b_single,
			struct bq27510_device_info *di)
{
	int loop_i;
	int ret=0;

	for (loop_i = 0; loop_i < BQ27510_RETRY_COUNT; loop_i++) {
		ret = di->bus->read(reg, rt_value, b_single, di);
		if (ret == 0)
			break;
		mdelay(10);
	}


	return ret;
}

int bq27510_battery_it_enable(void)
{
	int ret;
	printk("[BATT-BQ27510] set IT_enable\n");
	/* write 0x21 to 0x00 */
	ret = bq27x00_write(BQ27510_REG_CONTROL, 0x21, di);
	if (ret)
		dev_err(di->dev, "error write 0x%x to 0x%x\n",
			BQ27510_REG_CONTROL, 0x21);
	/* write 0x00 to 0x01 */
	ret = bq27x00_write(BQ27510_REG_CONTROL + 1, 0x00, di);
	if (ret)
		dev_err(di->dev, "error write 0x%x to 0x%x\n",
			BQ27510_REG_CONTROL + 1, 0x00);
	return ret;
}

/*
 * Return the battery temperature in tenths of degree Celsius
 * Or < 0 if something fails.
 */
static int bq27510_battery_temperature(struct bq27510_device_info *di)
{
	int ret;
	int temp = 0;

	ret = bq27x00_read(BQ27510_REG_TEMP, &temp, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading temperature\n");
		return ret;
	}
	return temp - 2731;
}

/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */
static int bq27510_battery_voltage(struct bq27510_device_info *di)
{
	int ret;
	int volt = 0;

	ret = bq27x00_read(BQ27510_REG_VOLT, &volt, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading voltage\n");
		return ret;
	}
	return volt;
}

/*
 * Return the battery full charge capacity in mAh
 * Or < 0 if something fails.
 */
static int bq27510_battery_full_charge_cap(struct bq27510_device_info *di)
{
	int ret;
	int cap = 0;

	ret = bq27x00_read(BQ27510_REG_FULL_CHARGE_CAP, &cap, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading full charge capacity\n");
		return ret;
	}
	return cap;
}

/*
 * Return the battery remaining capacity in mAh
 * Or < 0 if something fails.
 */
static int bq27510_battery_remaining_cap(struct bq27510_device_info *di)
{
	int ret;
	int cap = 0;

	ret = bq27x00_read(BQ27510_REG_REMAINING_CAP, &cap, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading remaining capacity\n");
		return ret;
	}
	return cap;
}
/*
 * Return the battery average current
 * Note that current can be negative signed as well
 * Or 0 if something fails.
*/
static int bq27510_battery_current(struct bq27510_device_info *di)
{
	int ret;
	int curr = 0;
	//int flags = 0;

	ret = bq27x00_read(BQ27510_REG_AI, &curr, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading current\n");
		return 0;
	}
	curr = (int)(s16)curr;
	return curr;
}

/*
 * Return the battery State-of-Charge
 * Or < 0 if something fails.
 */
static int bq27510_battery_soc(struct bq27510_device_info *di)
{
	int ret;
	int soc = 0;

	ret = bq27x00_read(BQ27510_REG_SOC, &soc, 0, di);

	if (ret) {
		dev_err(di->dev, "error reading relative State-of-Charge\n");
		return ret;
	}

	return soc;
}

/*
 * Return the battery Flags
 * Or < 0 if something fails.
 */
static int bq27510_battery_flag(struct bq27510_device_info *di)
{
	int ret;
	int flags = 0;

	ret = bq27x00_read(BQ27510_REG_FLAGS, &flags, 0, di);

	if (ret) {
		dev_err(di->dev, "error reading relative State-of-Charge\n");
		return ret;
	}
	return flags;
}

static int bq27510_battery_control_status(struct bq27510_device_info *di)
{
	int ret;
	int control_status;

	ret = bq27x00_write(BQ27510_REG_CONTROL, 0x00, di);
	if (ret)
		dev_err(di->dev, "error writing REG_CONTROL\n");
	ret = bq27x00_write(BQ27510_REG_CONTROL + 1, 0x00, di);
	if (ret)
		dev_err(di->dev, "error writing REG_CONTROL\n");
	ret = bq27x00_read(BQ27510_REG_CONTROL, &control_status, 0, di);
	if (ret)
		dev_err(di->dev, "error reading CONTROL_STATUS\n");
	di->rup_dis = ((control_status & 0x0004) >> 2); /* bit2 RUP_DIS*/
	return control_status;
}

int bq27510_battery_param_update(struct battery_type *battery,	struct protect_flags_type *flags)
{
	int flag = 0;
	int cs = 0;
	// STEP.1: update battery param
	battery->voltage_mV = bq27510_battery_voltage(di);
	battery->current_mA = bq27510_battery_current(di);
	battery->temp_01c = bq27510_battery_temperature(di);
	battery->charge_full_real_mAh = bq27510_battery_full_charge_cap(di);
	battery->charge_counter_mAh = bq27510_battery_remaining_cap(di);
	battery->soc = bq27510_battery_soc(di) * 10;
	flag = bq27510_battery_flag(di);
	if (flag & 0x0200) /* bit9 FC */
		battery->fc = 1;
	else
		battery->fc = 0;
	cs = bq27510_battery_control_status(di);
#if defined(CONFIG_MACH_VERDI_LTE) //show the control status in pd_m column
	battery->pd_m = cs;
#endif
	printk("[BATT-BQ27510] v=%d i=%d t=%d full=%d remain=%d soc=%d fc=%d flag=0x%x, cs=0x%x\n",
		battery->voltage_mV, battery->current_mA,
		battery->temp_01c, battery->charge_full_real_mAh,
		battery->charge_counter_mAh, battery->soc, battery->fc, flag, cs);
	return TRUE;
}

static int bq27510_battery_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct bq27510_access_methods *bus;
	struct bq27510_battery_platform_data *pdata =
					client->dev.platform_data;
	int retval = 0;

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		retval = -ENOMEM;
		return retval;
	}

	i2c_set_clientdata(client, di);

	bus = kzalloc(sizeof(*bus), GFP_KERNEL);
	if (!bus) {
		dev_err(&client->dev, "failed to allocate access "
			"method data\n");
		retval = -ENOMEM;
		kfree(di);
		return retval;
	}

	di->dev = &client->dev;
	bus->read = &bq27510_read_i2c;
	bus->write = &bq27510_write_i2c;
	di->bus = bus;
	di->client = client;
	di->rup_dis = -1; /* init state = -1 */

	bq27510_batt_info.gpio_mbat_in = pdata->gpio_mbat_in;
	if (pdata->func_battery_gpio_init != NULL) {
		pdata->func_battery_gpio_init();
	}

	return 0;
}

static int bq27510_battery_remove(struct i2c_client *client)
{
	struct bq27510_device_info *di = i2c_get_clientdata(client);

	kfree(di);

	return 0;
}


static const struct i2c_device_id bq27510_id[] = {
	{ "bq27510-battery", 0 },
	{},
};

static int bq27510_battery_suspend(struct device *dev)
{
	/*if (1 == di->rup_dis)*/
	if (0)
		bq27510_battery_it_enable();
	return 0;
}

static int bq27510_battery_resume(struct device *dev)
{
	return 0;
}

static struct dev_pm_ops bq27510_pm_ops = {
	.suspend = bq27510_battery_suspend,
	.resume = bq27510_battery_resume,
};

static struct i2c_driver bq27510_battery_driver = {
	.probe = bq27510_battery_probe,
	.remove = bq27510_battery_remove,
	.id_table = bq27510_id,
	.driver = {
		.name = "bq27510-battery",
		.pm = &bq27510_pm_ops,
	},
};

static int __init bq27510_battery_init(void)
{
	int ret;

	mutex_init(&bq27510_batt_info.info_lock);
	ret = i2c_add_driver(&bq27510_battery_driver);
	if (ret)
		BATT_ERR("Unable to register BQ27510 driver\n");

	return ret;
}
module_init(bq27510_battery_init);

static void __exit bq27510_battery_exit(void)
{
	i2c_del_driver(&bq27510_battery_driver);
}
module_exit(bq27510_battery_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("bq27510 battery driver");
