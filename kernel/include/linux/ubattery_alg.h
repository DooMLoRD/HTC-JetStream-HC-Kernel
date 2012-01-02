/*
 * Copyright (C) 2007 HTC Incorporated
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __UBATTERY_ALG_H__
#define __UBATTERY_ALG_H__
#include <linux/wrapper_types.h>

#define UBATTERY "[BATT-UA]"

#define UBATT_LOG(x...) do { \
struct timespec ts; \
struct rtc_time tm; \
getnstimeofday(&ts); \
rtc_time_to_tm(ts.tv_sec, &tm); \
printk(KERN_INFO "[BATT-UA] " x); \
printk(" at %lld (%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n", \
ktime_to_ns(ktime_get()), tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, \
tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec); \
} while (0)

enum batt_ctl_t {
	DISABLE = 0,
	ENABLE_SLOW_CHG,
	ENABLE_FAST_CHG,
	ENABLE_SUPER_CHG,
	CHARGER_CHK,
	TOGGLE_CHARGER,
	ENABLE_MIN_TAPER,
	DISABLE_MIN_TAPER
};

#define BATTERY_ID_UNKNOWN	0

/* battery charging state*/

enum {
	CHARGE_STATE_UNKNOWN,   		/* before anything is ready, we are in this state. shall default low current charge and show charging LED*/
	CHARGE_STATE_PREDICTION,		/* in normal case, we need to enter prediction for 10 seconds for 1st KADC*/
	CHARGE_STATE_DISCHARGE, 		/* cable out state*/
	CHARGE_STATE_CHARGING,  		/* charging state*/
	CHARGE_STATE_PENDING,   		/* charging state but no good*/
	CHARGE_STATE_FULL_WAIT_STABLE,  /* charging state but going full*/
	CHARGE_STATE_FULL_CHARGING, 	/* charging full, keep charging*/
	CHARGE_STATE_FULL_PENDING,  	/* charging full, stop charging*/
	CHARGE_STATE_FULL_RECHARGING,  	/* charging full, recharging*/
};

/* power algorithm data structure and config data structure*/
struct battery_type{

		BOOL is_power_on_reset;

		INT32 voltage_mV;
		INT32 current_mA;
		INT32 discharge_mA;
		INT32 charge_counter_mAh;
		INT32 temp_01c;
		INT32 last_temp_01c;
		INT32 id_ohm;
		INT32 vref_mv;

		INT32 voltage_adc;
		INT32 current_adc;
		INT32 discharge_adc;
		INT32 charge_counter_adc;
		INT32 temp_adc;
		INT32 last_temp_adc;
		INT32 id_adc;
		INT32 vref_adc;

		INT32 id_index;
		INT32 charge_full_design_mAh;
		INT32 charge_full_real_mAh;

		INT32 temp_index;
		INT32 temp_check_index;

		INT32 KADC_01p;
		INT32 RARC_01p;
		INT32 pd_m;

		INT32 software_charge_counter_mAms;
		INT32 thermal_id;

		INT32 soc;	/* for smart gauge */
		INT32 fc;
};

struct protect_flags_type{

		BOOL is_charging_enable_available;
		BOOL is_charging_high_current_avaialble;
		BOOL is_low_current_charging_enable;
		BOOL is_charging_indicator_available;
		BOOL is_temperature_fault;
		BOOL is_battery_dead;
		BOOL is_disable_temp_protect; /* MATT: considering remove*/
};

typedef struct _ubattery_platform_data ubattery_platform_data;

struct poweralg_type
{
	int batt_state; /* 0: initial, 1: can get valid battery info */
	int charge_state;
	int capacity_01p;
	int last_capacity_01p;
	int fst_discharge_capacity_01p;
	int fst_discharge_acr_mAh;
	int charging_source;
	int last_charging_source;
	int charging_enable;
	int low_current_charging_enable;
	BOOL is_need_calibrate_at_49p;
	BOOL is_need_calibrate_at_14p;
	BOOL is_charge_over_load;
	struct battery_type battery;
	struct protect_flags_type protect_flags;
	BOOL is_china_ac_in;
	BOOL is_super_ac;
	BOOL is_cable_in;
	BOOL is_unknown_ac;
	BOOL is_voltage_stable;
	BOOL is_software_charger_timeout;
	BOOL is_superchg_software_charger_timeout;
	UINT32 state_start_time_ms;
	UINT32 last_charger_enable_toggled_time_ms;
	BOOL is_need_toggle_charger;
	ubattery_platform_data* pdata;
};

struct poweralg_config_type
{
	INT32 full_charging_mv;
	INT32 full_charging_ma;
	INT32 full_pending_ma;			/* 0 to disable*/
	INT32 full_charging_timeout_sec;	/* 0 to disable*/
	INT32 voltage_recharge_mv;		/* 0 to disable*/
	INT32 capacity_recharge_p;		/* <0 to disable*/
	INT32 voltage_exit_full_mv;		/* 0 to disable*/
	INT32 exit_full_p;			/* <0 to disable*/
	INT32 min_taper_current_mv;		/* 0 to disable*/
	INT32 min_taper_current_ma;		/* 0 to disable*/
	INT32 exit_min_taper_current_ma;	/* 0 to disable*/
	INT32 wait_votlage_statble_sec;
	INT32 predict_timeout_sec;
	INT32 polling_time_normal_sec;
	INT32 polling_time_extension1_sec;

	BOOL enable_full_calibration;
	BOOL enable_weight_percentage;
	INT32 software_charger_timeout_sec;  /* 0 to disable*/ /* for china AC */
	INT32 superchg_software_charger_timeout_sec;  /* 0 to disable*/ /* for superchg */
	INT32 charger_hw_safety_timer_watchdog_sec;  /* 0 to disable*/

	BOOL debug_disable_shutdown;
	BOOL debug_disable_temp_protect;
	BOOL debug_disable_hw_timer;
	BOOL debug_always_predict;
	INT32 full_level;                  /* 0 to disable*/
};

struct _ubattery_platform_data {
	int (*func_get_thermal_id)(void);
	int (*func_get_battery_id)(void);
	int (*func_gpio_init)(int ctl);
	void (*func_poweralg_config_init)(struct poweralg_config_type*);
	int (*func_param_update)(struct battery_type*,	struct protect_flags_type*);
	int (*func_capacity_update)(struct poweralg_type*);
	int (*func_capacity_param_update)(struct battery_type*, int capacity_01p);
	int (*func_update_charging_protect_flag)(struct poweralg_type*);
	int (*func_charger_ctrl)(int ctl);
	int (*func_get_batt_id_ohm)(void);
	int r2_kohm;
	UINT32* id_tbl;
};

/* battery behavior constant*/
#define BATTERY_PERCENTAGE_UNKNOWN  0xFF
#define BATTERY_LOW_PERCENTAGE	    10  	/* in 1%*/
#define BATTERY_CRITICAL_PERCENTAGE 5   	/* in 1%*/
#define BATTERY_EMPTY_PERCENTAGE    0   	/* in 1%*/

#endif
