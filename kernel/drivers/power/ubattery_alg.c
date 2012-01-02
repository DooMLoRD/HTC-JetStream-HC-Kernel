/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

Copyright (c) 2010 High Tech Computer Corporation

Module Name:

		ubattery_alg.c

Abstract:

		This module implements the power algorithm, including below concepts:
		1. Charging function control.
		2. Charging full condition.
		3. Recharge control.
		4. Battery capacity maintainance.
		5. Battery full capacity calibration.

Original Auther:

		Matt.SY Yeh  January-24-2011
---------------------------------------------------------------------------------*/
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/android_alarm.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/wakelock.h>
#include <asm/gpio.h>
#include <linux/delay.h>
#include <linux/ubattery_alg.h>
#include <linux/wrapper_types.h>
#include <mach/htc_battery_core.h>
#include <asm/mach-types.h>
#include "../../arch/arm/mach-msm/proc_comm.h"
#include <linux/i2c.h>  					/* for i2c_adapter, i2c_client define*/
#include <linux/time.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <mach/board.h>
#include <mach/board_htc.h>

extern int usb_get_connect_type(void);
extern int usb_is_connect_type_ready(void);
extern int show_meminfo(void);
extern int show_vmallocinfo(struct seq_file * m,void * p);

struct ubattery_device_info {

		struct device *dev;
		struct workqueue_struct *monitor_wqueue;
		struct work_struct monitor_work;
		/* lock to protect the battery info */
		struct mutex lock;
		unsigned long update_time;	/* jiffies when data read */
		struct alarm alarm;
		struct wake_lock work_wake_lock;
		u8 slow_poll;
		ktime_t last_poll;
};
static struct wake_lock vbus_wake_lock;
static struct mutex poweralg_lock;

/*========================================================================================

HTC power algorithm helper member and functions

========================================================================================*/

static struct poweralg_type poweralg = {0};
static struct poweralg_config_type config = {0};
static struct poweralg_config_type debug_config = {0};
BOOL is_need_battery_id_detection = TRUE;
static struct ubattery_device_info *g_di_ptr = NULL;

static int g_first_update_charger_ctl = 1;

/* overrdie charger control from outside:
 * -1: not override 0: disable charger, (not support yet)...*/
static int charger_control_override = -1;

/*#define FAST_POLL	(1 * 30)*/
/*#define SLOW_POLL	(1 * 60)*/
#define PREDIC_POLL	(1 * 10)

#define HTC_UBATTERY_DEBUG_ENABLE 	1

/* safety timer */
static UINT32 delta_time_sec = 0;
static UINT32 chg_en_time_sec = 0;
static UINT32 super_chg_on_time_sec = 0;


/* function declare */
static void ubattery_charger_control( int type);
static void ubattery_cable_status_handler_func(enum usb_connect_type online);


/* time function */
DWORD ubattery_gettime_msec(void)
{
	struct timespec now;
	getnstimeofday(&now);
	/*struct timespec t;
	t.tv_sec = t.tv_nsec = 0;
	clock_gettime(CLOCK_MONOTONIC, &t);*/
	return now.tv_sec * 1000 + now.tv_nsec / 1000000;
}

/*========================================================================================

	HTC supporting MFG testing member and functions

=========================================================================================*/

static BOOL b_is_charge_off_by_bounding = FALSE;
static void bounding_fullly_charged_level(int upperbd)
{
	static int pingpong = 1;
	int lowerbd;
	int current_level;
	b_is_charge_off_by_bounding = FALSE;
	if (upperbd <= 0)
		return; /* doesn't activated this function */
	lowerbd = upperbd - 5; /* 5% range */

	if (lowerbd < 0)
		lowerbd = 0;
	current_level = CEILING(poweralg.capacity_01p, 10);

	if (pingpong == 1 && upperbd <= current_level) {
		printk(UBATTERY "MFG: lowerbd=%d, upperbd=%d, current=%d, pingpong:1->0 turn off\n", lowerbd, upperbd, current_level);
		b_is_charge_off_by_bounding = TRUE;
		pingpong = 0;
	} else if (pingpong == 0 && lowerbd < current_level) {
		printk(UBATTERY "MFG: lowerbd=%d, upperbd=%d, current=%d, toward 0, turn off\n", lowerbd, upperbd, current_level);
		b_is_charge_off_by_bounding = TRUE;
	} else if (pingpong == 0 && current_level <= lowerbd) {
		printk(UBATTERY "MFG: lowerbd=%d, upperbd=%d, current=%d, pingpong:0->1 turn on\n", lowerbd, upperbd, current_level);
		pingpong = 1;
	} else {
		printk(UBATTERY "MFG: lowerbd=%d, upperbd=%d, current=%d, toward %d, turn on\n", lowerbd, upperbd, current_level, pingpong);
	}

}

static BOOL is_charge_off_by_bounding_condition(void)
{
	return b_is_charge_off_by_bounding;
}

static BOOL is_charging_avaiable(void)
{
	if (poweralg.is_superchg_software_charger_timeout) return FALSE;
	if (poweralg.is_software_charger_timeout) return FALSE;
	/* ignore charging control in temp protection disable mode */
	if (!poweralg.protect_flags.is_charging_enable_available &&
		!poweralg.protect_flags.is_disable_temp_protect) return FALSE;
	if (!poweralg.is_cable_in) return FALSE;
	if (poweralg.charge_state == CHARGE_STATE_PENDING) return FALSE;
	if (poweralg.charge_state == CHARGE_STATE_FULL_PENDING)	return FALSE;
	if (poweralg.charge_state == CHARGE_STATE_PREDICTION) return FALSE;
	if (is_charge_off_by_bounding_condition()) return FALSE;
	if (0 == charger_control_override) return FALSE;
	if (poweralg.is_unknown_ac) return FALSE;
	return TRUE; /* CHARGE_STATE_UNKNOWN, SET_LED_BATTERY_CHARGING is available to be charged by default*/
}

static BOOL is_high_current_charging_avaialable(void)
{
	/* ignore charging control in temp protection disable mode */
	if (!poweralg.protect_flags.is_charging_high_current_avaialble &&
		!poweralg.protect_flags.is_disable_temp_protect) return FALSE;
	if (!poweralg.is_china_ac_in) return FALSE;
	if (poweralg.charge_state == CHARGE_STATE_UNKNOWN) return FALSE;
	return TRUE;
}

static BOOL is_super_current_charging_avaialable(void)
{
	if (!poweralg.is_super_ac) return FALSE;
	return TRUE;
}

/* battery id related function */
static INT32 ubattery_get_batt_id_index(INT32 batt_id_ohm, UINT32* map_tbl)
{
	if (map_tbl) {
		int i;
		for (i = 0; map_tbl[2*i] != -1; i++) {
			/* minus 1, unknown battery is not in ID_RANGE
			[min, max)*/
			INT32 resister_min = map_tbl[i*2];
			INT32 resister_max = map_tbl[i*2 + 1];

			if (resister_min <= batt_id_ohm && ((resister_max > batt_id_ohm)||(resister_max==-1)) ) {
				return i + 1;
			}
		}
	}
	return BATTERY_ID_UNKNOWN; /* unknown battery id */
}

static void ubattery_batt_id_detection(void)
{

	if (poweralg.pdata->func_get_batt_id_ohm) {
		poweralg.battery.id_ohm = poweralg.pdata->func_get_batt_id_ohm();
	} else
		printk(UBATTERY "No func_get_batt_id_ohm() hooked.\n");

	if (poweralg.pdata->id_tbl) {
		poweralg.battery.id_index = ubattery_get_batt_id_index(poweralg.battery.id_ohm, poweralg.pdata->id_tbl);
	} else
		printk(UBATTERY "No id_tbl hooked.\n");
	/* TODO: do we need batt_id stable checking mechanism? */
	/* printk(UBATTERY "batt_id = %d.\n", poweralg.battery.id_index); */
}

#define HTC_ENABLE_DUMMY_BATTERY	0
BOOL ubattery_param_update(void)
{
	/* STEP 1: update battery parameter */
	if (poweralg.pdata->func_param_update)
		if (!poweralg.pdata->func_param_update(&poweralg.battery, &poweralg.protect_flags))
			return FALSE;

	/* STEP 2: update protect flags and is_low_current_charging_enable */
	/* only when charger exists we need to do this step2 */
	if ((0 <= poweralg.charging_source) &&	/* charging_source state valid */
		(CONNECT_TYPE_NONE != poweralg.charging_source)) {
		if( NULL != poweralg.pdata->func_update_charging_protect_flag)
		{
			int pstate;
/* the following code impl is moved into func_update_charging_protect_flag. */
#if 0
			/* we need to reset protect state to detecting state
				when every time charger just plugged in. */
			BOOL reset =
				(CONNECT_TYPE_NONE == poweralg.last_charging_source)?TRUE:FALSE;
			pstate = poweralg.pdata->func_update_charging_protect_flag(
				poweralg.battery.current_mA, poweralg.battery.voltage_mV,
				poweralg.battery.temp_01c,
				&(poweralg.protect_flags.is_charging_enable_available),
				&(poweralg.protect_flags.is_charging_high_current_avaialble),
				&(poweralg.protect_flags.is_low_current_charging_enable),
				reset);
#endif
			pstate = poweralg.pdata->func_update_charging_protect_flag(&poweralg);
			/* printk(UBATTERY "PState = %d,(allowchg,allowhchg,lchgen) = (%d,%d,%d)\n",
				pstate,
				poweralg.protect_flags.is_charging_enable_available,
				poweralg.protect_flags.is_charging_high_current_avaialble,
				poweralg.protect_flags.is_low_current_charging_enable); */
		}
	}

/* MATT: is_diable_temp_protect member is redundant. should be removed.
	the following code segment is removed */
#if 0
	/* when disable_temp_protect == TRE, driver needs to:
		1. ignore charging contorl caused by temperature.
		2. prevent shutdown by framework, if temp > 68. */
	if (poweralg.protect_flags.is_disable_temp_protect) {
		if (680 < poweralg.battery.temp_01c) {
			printk(DRIVER_ZONE "set fake temp = 680 (real temp = %d)\n",
			poweralg.battery.temp_01c);
			poweralg.battery.temp_01c = 680;
		}
		poweralg.protect_flags.is_charging_enable_available = TRUE;
		poweralg.protect_flags.is_charging_high_current_avaialble = TRUE;
	}
#endif

#if ! HTC_ENABLE_DUMMY_BATTERY
	if (poweralg.battery.id_index == BATTERY_ID_UNKNOWN) {
		poweralg.protect_flags.is_charging_enable_available = FALSE;
	}
#else /* HTC_ENABLE_DUMMY_BATTERY*/
	/* do not disable charging for debug stage*/
	poweralg.protect_flags.is_charging_enable_available = TRUE;
#endif /* HTC_ENABLE_DUMMY_BATTERY*/

	return TRUE;
}

void ubattery_capacity_param_update(struct battery_type* battery, int cap_01p)
{
	if (poweralg.pdata->func_capacity_param_update)
		poweralg.pdata->func_capacity_param_update(&poweralg.battery, poweralg.capacity_01p);
	poweralg.battery.is_power_on_reset = FALSE;
}

static void ubattery_update_next_charge_state(void)
{
	static UINT32 count_charging_full_condition;
	static UINT32 count_charge_over_load;
	int next_charge_state;
	int i;

	/*  unknown -> prediction -> unknown -> discharge/charging/pending
	charging -> full-wait-stable -> full-charging -> full-pending
	full-pending -> full-charging -> charging
	*(cable in group) -> discharge, charge-pending, dead
	*(cable out group), full-wait-stable, charge-pending, dead -> charging*/

	for (i = 0; i < 25; i++) /* maximun 25 times state transition to prevent from busy loop; ideally the transition time shall be less than 5 times.*/
	{
		next_charge_state = poweralg.charge_state;

		/* 0. enter prediction state or not*/
		if (poweralg.charge_state == CHARGE_STATE_UNKNOWN){
			if (poweralg.battery.is_power_on_reset || config.debug_always_predict){
				if (poweralg.protect_flags.is_battery_dead){
					/* keep poweralg.charge_state unchanged, set capacity to 0% directly*/
					printk(UBATTERY "dead battery, assign p=0%%\n");
					poweralg.capacity_01p = 0;
					ubattery_capacity_param_update(&poweralg.battery, poweralg.capacity_01p);

					poweralg.fst_discharge_capacity_01p = poweralg.capacity_01p;
					poweralg.fst_discharge_acr_mAh = poweralg.battery.charge_counter_mAh;
				}
				else{
					/* battery replaced, recalculate capacity based on battery voltage*/
					printk(UBATTERY "start predict discharge...\n");
					next_charge_state = CHARGE_STATE_PREDICTION;
				}

				config.debug_always_predict = FALSE;
			}
		}

		if (next_charge_state == poweralg.charge_state){
			/*---------------------------------------------------*/
			/* 1. cable in group*/
			if (poweralg.charge_state == CHARGE_STATE_UNKNOWN ||
				poweralg.charge_state == CHARGE_STATE_CHARGING ||
				poweralg.charge_state == CHARGE_STATE_PENDING ||
				poweralg.charge_state == CHARGE_STATE_FULL_WAIT_STABLE ||
				poweralg.charge_state == CHARGE_STATE_FULL_CHARGING ||
				poweralg.charge_state == CHARGE_STATE_FULL_RECHARGING ||
				poweralg.charge_state == CHARGE_STATE_FULL_PENDING) {
				if (!poweralg.is_cable_in){
					next_charge_state = CHARGE_STATE_DISCHARGE;
				}
				else if (!poweralg.protect_flags.is_charging_enable_available){
					/* if is_disable_temp_protect is set,
					ignore is_charging_enable_available set by over
					temperature. */
					if ((!poweralg.protect_flags.is_disable_temp_protect) ||
						(poweralg.battery.id_index == BATTERY_ID_UNKNOWN))
					next_charge_state = CHARGE_STATE_PENDING;
				}
			}
			/*---------------------------------------------------*/
			/* 2. cable out group*/
			if (poweralg.charge_state == CHARGE_STATE_UNKNOWN ||
				poweralg.charge_state == CHARGE_STATE_DISCHARGE){
				if (poweralg.is_cable_in){
					next_charge_state = CHARGE_STATE_CHARGING;
				}
			}
			/*---------------------------------------------------*/
			/* 3. check if cable is unknown type (12V cable)*/
			if (poweralg.is_unknown_ac){
				printk(UBATTERY "found unsupport cable! disable charging\n");
				next_charge_state = CHARGE_STATE_DISCHARGE;
			}
		}

		/*-----------------------------------------------------------*/
		/* 3. state handler/transition,
		if the charge state is not changed due to cable/protect flags*/
		if (next_charge_state == poweralg.charge_state){
			switch (poweralg.charge_state){
				case CHARGE_STATE_PREDICTION:
					{
						UINT32 end_time_ms = ubattery_gettime_msec();

						if (end_time_ms - poweralg.state_start_time_ms >=
							config.predict_timeout_sec * 1000){

							printk(UBATTERY "predict done [%u->%u]\n", poweralg.state_start_time_ms,
								end_time_ms);
							next_charge_state = CHARGE_STATE_UNKNOWN;
						}
					}
					break;
				case CHARGE_STATE_CHARGING:
					if (!poweralg.battery.is_power_on_reset){
						/* -> full-charging, pending, dead*/
						if (poweralg.battery.fc) {
							next_charge_state = CHARGE_STATE_FULL_CHARGING;
						}
/* TODO:check this condition */
#if 0
						else if (poweralg.capacity_01p > 990){
							/* only ever charge-full, the capacity can be larger than 99.0%*/
							next_charge_state = CHARGE_STATE_FULL_CHARGING;
						}
#endif
						else if (poweralg.battery.voltage_mV >= config.full_charging_mv &&
							poweralg.battery.current_mA >= 0 &&
							poweralg.battery.current_mA <= config.full_charging_ma){
							/* meet charge full terminate condition, check again*/
							next_charge_state = CHARGE_STATE_FULL_WAIT_STABLE;
						}
					}

					if (poweralg.battery.current_mA <= 0){
						/* count_charge_over_load is 5 as max*/
						if (count_charge_over_load < 5)
							count_charge_over_load++;
						else
							poweralg.is_charge_over_load = TRUE;
					}
					else{
						count_charge_over_load = 0;
						poweralg.is_charge_over_load = FALSE;
					}

					/* is_software_charger_timeout: only triggered when AC adapter in*/
					if (config.software_charger_timeout_sec && poweralg.is_china_ac_in){
						/* software charger timer is enabled; for AC charge only*/
						UINT32 end_time_ms = ubattery_gettime_msec();

						if (end_time_ms - poweralg.state_start_time_ms >=
							config.software_charger_timeout_sec * 1000){

							printk(UBATTERY "software charger timer timeout [%u->%u]\n",
								poweralg.state_start_time_ms,
								end_time_ms);
							poweralg.is_software_charger_timeout = TRUE;
						}
					}
					break;
				case CHARGE_STATE_FULL_WAIT_STABLE:
					{
						/* -> full-charging, pending, dead*/
						if (poweralg.battery.voltage_mV >= config.full_charging_mv &&
							poweralg.battery.current_mA >= 0 &&
							poweralg.battery.current_mA <= config.full_charging_ma){

							count_charging_full_condition++;
						}
						else{
							count_charging_full_condition = 0;
							next_charge_state = CHARGE_STATE_CHARGING;
						}

						if (count_charging_full_condition >= 3){

							poweralg.capacity_01p = 1000;
							ubattery_capacity_param_update(&poweralg.battery, poweralg.capacity_01p);

							next_charge_state = CHARGE_STATE_FULL_CHARGING;
						}
					}
					break;
				case CHARGE_STATE_FULL_CHARGING:
					{
						/* -> full-pending, charging*/
						UINT32 end_time_ms = ubattery_gettime_msec();

						if (poweralg.battery.voltage_mV < config.voltage_exit_full_mv){
							if (poweralg.capacity_01p > 990)
								poweralg.capacity_01p = 990;
							next_charge_state = CHARGE_STATE_CHARGING;
						}
						else if (config.full_pending_ma != 0 &&
							poweralg.battery.current_mA >= 0 &&
							poweralg.battery.current_mA <= config.full_pending_ma){

							printk(UBATTERY " charge-full pending(%dmA)(%u:%u)\n",
								poweralg.battery.current_mA,
								poweralg.state_start_time_ms,
								end_time_ms);

							next_charge_state = CHARGE_STATE_FULL_PENDING;
						}
						else if (end_time_ms - poweralg.state_start_time_ms >=
							config.full_charging_timeout_sec * 1000){

							printk(UBATTERY " charge-full (expect:%dsec)(%u:%u)\n",
								config.full_charging_timeout_sec,
								poweralg.state_start_time_ms,
								end_time_ms);
							next_charge_state = CHARGE_STATE_FULL_PENDING;
						}
					}
					break;
				case CHARGE_STATE_FULL_PENDING:
					if ((poweralg.battery.voltage_mV >= 0 &&
						poweralg.battery.voltage_mV < config.voltage_recharge_mv) ||
						(0 <= config.capacity_recharge_p &&
						(((0 <= poweralg.battery.RARC_01p) &&
						(poweralg.battery.RARC_01p <= config.capacity_recharge_p * 10)) ||
						((0 <= poweralg.battery.soc) &&
						(poweralg.battery.soc <= config.capacity_recharge_p * 10))))){
						/* -> full-recharging*/
						next_charge_state = CHARGE_STATE_FULL_RECHARGING;
					}
					break;
				case CHARGE_STATE_FULL_RECHARGING:
					{
						if ((poweralg.battery.voltage_mV < config.voltage_exit_full_mv) ||
							((0 < config.exit_full_p) && (0 < poweralg.battery.soc) &&
							(poweralg.battery.soc < (config.exit_full_p * 10)))){
							if (poweralg.capacity_01p > 990)
								poweralg.capacity_01p = 990;
							next_charge_state = CHARGE_STATE_CHARGING;
						}
						else if (poweralg.battery.fc) {
							next_charge_state = CHARGE_STATE_FULL_CHARGING;
						}
						else if (poweralg.battery.voltage_mV >= config.full_charging_mv &&
							poweralg.battery.current_mA >= 0 &&
							poweralg.battery.current_mA <= config.full_charging_ma){
							/* meet charge full terminate condition, check again*/
							next_charge_state = CHARGE_STATE_FULL_CHARGING;
						}
					}
					break;
				case CHARGE_STATE_PENDING:
				case CHARGE_STATE_DISCHARGE:
					{
						UINT32 end_time_ms = ubattery_gettime_msec();

						if (!poweralg.is_voltage_stable){
							if (end_time_ms - poweralg.state_start_time_ms >=
								config.wait_votlage_statble_sec * 1000){

								printk(UBATTERY " voltage stable\n");
								poweralg.is_voltage_stable = TRUE;
							}
						}
					}

					if (poweralg.is_cable_in &&
						poweralg.protect_flags.is_charging_enable_available && (!poweralg.is_unknown_ac)){
						/* -> charging*/
						next_charge_state = CHARGE_STATE_CHARGING;
					}
					break;
			}
		}
		/*---------------------------------------------------------------------------------------------------*/
		/* 4. state transition*/
		if (next_charge_state != poweralg.charge_state){
			/* state exit*/
			switch (poweralg.charge_state){
				case CHARGE_STATE_UNKNOWN:
/* TODO: only ds2746 kinda gauge need it */
#if 0
					poweralg.capacity_01p = poweralg.battery.RARC_01p;
					if (poweralg.capacity_01p > 990)
						poweralg.capacity_01p = 990;
					if (poweralg.capacity_01p < 0)
						poweralg.capacity_01p = 0;

					poweralg.fst_discharge_capacity_01p = poweralg.capacity_01p;
					poweralg.fst_discharge_acr_mAh = poweralg.battery.charge_counter_mAh;
#endif
					break;
				case CHARGE_STATE_PREDICTION:
					ubattery_param_update(); /*TODO: why need to do it here?*/

					poweralg.capacity_01p = poweralg.battery.KADC_01p;
					if (poweralg.capacity_01p > 990)
						poweralg.capacity_01p = 990;
					if (poweralg.capacity_01p < 0)
						poweralg.capacity_01p = 0;
					ubattery_capacity_param_update(&poweralg.battery,
						poweralg.capacity_01p);

					poweralg.fst_discharge_capacity_01p = poweralg.capacity_01p;
					poweralg.fst_discharge_acr_mAh = poweralg.battery.charge_counter_mAh;
					break;
			}

			/* state init*/
			poweralg.state_start_time_ms = ubattery_gettime_msec();

			switch (next_charge_state){
				case CHARGE_STATE_DISCHARGE:
				case CHARGE_STATE_PENDING:
					/*! star_lee 20100426 - always set ACR=FULL when discharge starts and ACR>FULL*/
					if (poweralg.battery.RARC_01p > 1000)
						ubattery_capacity_param_update(&poweralg.battery, 1000);

					poweralg.is_need_calibrate_at_49p = TRUE;
					poweralg.is_need_calibrate_at_14p = TRUE;
					poweralg.fst_discharge_capacity_01p = poweralg.capacity_01p;
					poweralg.fst_discharge_acr_mAh = poweralg.battery.charge_counter_mAh;
					poweralg.is_voltage_stable = FALSE;

					break;
				case CHARGE_STATE_CHARGING:
					poweralg.is_need_toggle_charger = FALSE;
					poweralg.last_charger_enable_toggled_time_ms = ubattery_gettime_msec();
					poweralg.is_software_charger_timeout = FALSE;   /* reset software charger timer every time when charging re-starts*/
					poweralg.is_charge_over_load = FALSE;
					count_charge_over_load = 0;
					/* TODO: check why need this: poweralg.battery.charge_full_real_mAh = poweralg.battery.charge_full_design_mAh; */
					ubattery_capacity_param_update(&poweralg.battery, poweralg.capacity_01p);
					break;
				case CHARGE_STATE_FULL_WAIT_STABLE:
					/* set to 0 first; the cournter will be add to 1 soon in CHARGE_STATE_FULL_WAIT_STABLE state handler*/
					count_charging_full_condition = 0;
					break;
			}

			printk(UBATTERY "state change(%d->%d), full count=%d, over load count=%d [%u]\n",
				poweralg.charge_state,
				next_charge_state,
				count_charging_full_condition,
				count_charge_over_load,
				poweralg.state_start_time_ms);

			poweralg.charge_state = next_charge_state;
			continue;
		}

		break;
	}
}

void ubattery_capacity_update(void)
{
	static int last_capacity_01p = -1;

	if (poweralg.pdata->func_capacity_update)
		poweralg.pdata->func_capacity_update(&poweralg);
	else {
		if (poweralg.charge_state == CHARGE_STATE_PREDICTION ||
			poweralg.charge_state == CHARGE_STATE_UNKNOWN){

			/*! star_lee 20100429 - return 99%~25% when in prediction mode*/
			poweralg.capacity_01p = max(min(990, poweralg.battery.soc), 250);
			printk(UBATTERY "fake percentage (%d) during prediction.\n",
				poweralg.capacity_01p);
		}
		else if (poweralg.charge_state == CHARGE_STATE_FULL_CHARGING ||
			poweralg.charge_state == CHARGE_STATE_FULL_RECHARGING ||
			poweralg.charge_state == CHARGE_STATE_FULL_PENDING){

			if(last_capacity_01p > -1)
			{

				if(last_capacity_01p < 1000)
				{
					poweralg.capacity_01p += 10; // 10 means 1% for soc
					last_capacity_01p = poweralg.capacity_01p;
				}
				else
				{
					poweralg.capacity_01p = 1000;
					last_capacity_01p = poweralg.capacity_01p;
				}
			}
			else
			{
				poweralg.capacity_01p = 1000;
				last_capacity_01p = poweralg.capacity_01p;
			}
		}
		else if (!is_charging_avaiable() && poweralg.is_voltage_stable){
			/* DISCHARGE ALG: capacity is based on KADC/RARC; only do this after cable in 3 minutes later*/
			if(last_capacity_01p > -1)
			{
				if(poweralg.battery.voltage_mV < 3250)
				{
					poweralg.capacity_01p -= 30; // 30 means 3% for soc
					if(poweralg.capacity_01p < 0)
						poweralg.capacity_01p = 0;
					last_capacity_01p = poweralg.capacity_01p;
				}
				else if((poweralg.battery.soc - last_capacity_01p) >= 0)
					poweralg.capacity_01p = last_capacity_01p;
				else
				{
					poweralg.capacity_01p = last_capacity_01p - 10;
					last_capacity_01p = poweralg.capacity_01p;
				}
			}
			else
			{
				poweralg.capacity_01p = poweralg.battery.soc;
				last_capacity_01p = poweralg.capacity_01p;
			}
		}
		else{
			/* CHARGE ALG: capacity is always based on ACR
			1. plus 1% as max when charge, if the orignal capacity is <= 99%, the result is no more than 99%
			2. minus 1% as max when discharge, not less than 0%*/
			if(last_capacity_01p > -1)
			{
				if(poweralg.battery.soc > last_capacity_01p)
				{
					poweralg.capacity_01p += 10; // 10 means 1% for soc
					last_capacity_01p = poweralg.capacity_01p;
				}
				else
				{
					poweralg.capacity_01p = poweralg.battery.soc;
					last_capacity_01p = poweralg.capacity_01p;
				}

			}
			else
			{
				poweralg.capacity_01p = poweralg.battery.soc;
				last_capacity_01p = poweralg.capacity_01p;
			}
		}
	}
}

/*========================================================================================

HTC power algorithm implemetation

========================================================================================*/

static BOOL ubattery_do_power_alg(BOOL is_event_triggered)
{
	/* is_event_triggered - TRUE: handle event only, do not update capacity; FALSE; always update capacity*/
	static BOOL s_bFirstEntry = TRUE;
	static UINT32 s_pre_time_ms;
	/*static INT32 s_level;*/

	UINT32 now_time_ms = ubattery_gettime_msec();
#if !HTC_UBATTERY_DEBUG_ENABLE
	BOOL show_debug_message = FALSE;
#endif

	printk(UBATTERY "%s, %d\n", __func__, is_event_triggered);

	/*------------------------------------------------------
	1 get low level batt row data and trans to meaningful form
	  a. get battery id if need.*/
	if (is_need_battery_id_detection)
		ubattery_batt_id_detection();
	/*
	  b. updating poweralg.battery and poweralg.protect_flags */
	if (!ubattery_param_update()){
		printk(UBATTERY "battery_param_update fail, please retry next time.\n");
		return FALSE;
		/* TODO: goto chg_ctl; */
	}

	/*------------------------------------------------------
	2 updating algorithm state by parameters.*/
	ubattery_update_next_charge_state();

	/*-----------------------------------------------------
	2 calculate battery capacity (predict if necessary)*/
	if (s_bFirstEntry || now_time_ms - s_pre_time_ms > 10000 || !is_event_triggered){
		/* TODO check:DO not update capacity when plug/unplug cable less than 10 seconds*/
		ubattery_capacity_update();

		s_bFirstEntry = FALSE;
		s_pre_time_ms = now_time_ms;
	}

	if (config.debug_disable_shutdown){
		if (poweralg.capacity_01p <= 0){
			poweralg.capacity_01p = 1;
		}
	}

	bounding_fullly_charged_level(config.full_level);

	/* is_superchg_software_charger_timeout: only triggered when superAC adapter in*/
	if (config.superchg_software_charger_timeout_sec && poweralg.is_super_ac
		&& FALSE==poweralg.is_superchg_software_charger_timeout){
		super_chg_on_time_sec += delta_time_sec;
		if (config.superchg_software_charger_timeout_sec <= super_chg_on_time_sec){
			printk(UBATTERY "superchg charger on timer timeout: %u sec\n",
				super_chg_on_time_sec);
			poweralg.is_superchg_software_charger_timeout = TRUE;
		}
	}
/* chg_ctl: TODO: use this as update fail default flow */
	/*------------------------------------------------------
	3 charging function change*/
	if (is_charging_avaiable()){
		/* STEP 1: for toggle charger */
		/* Software should also toggle MCHG_EN within 4 hrs
			to prevent charger HW safety timer expired. */
		if (config.charger_hw_safety_timer_watchdog_sec){
			chg_en_time_sec += delta_time_sec;
			if (config.charger_hw_safety_timer_watchdog_sec <= chg_en_time_sec) {
				printk(UBATTERY "need software toggle charger: lasts %d sec\n", chg_en_time_sec);
				chg_en_time_sec = 0;
				ubattery_charger_control(DISABLE);
				udelay(200);
			}
		}

		/* STEP 2: enable/disable low current charging if need */
		/* EXPRESS/VERDI: control charger IC minimum taper current */
		if (poweralg.protect_flags.is_low_current_charging_enable)
			ubattery_charger_control(ENABLE_MIN_TAPER);
		else
			ubattery_charger_control(DISABLE_MIN_TAPER);

		/* STEP 3: enable/disable charger */
		if (is_high_current_charging_avaialable()){
			if (is_super_current_charging_avaialable()) {
				ubattery_charger_control(ENABLE_SUPER_CHG);
			} else {
				ubattery_charger_control(ENABLE_FAST_CHG);
			}
		} else {
			ubattery_charger_control(ENABLE_SLOW_CHG);
		}

	} else {
		ubattery_charger_control(DISABLE);
		/* reset charger safety timer */
		chg_en_time_sec = 0;
		super_chg_on_time_sec = 0;
		poweralg.is_need_toggle_charger = FALSE;
	}
	poweralg.batt_state = 1; /* batt_state is valid after first update */

#if 0
	if (config.debug_disable_hw_timer && poweralg.is_charge_over_load){
		ubattery_charger_control(DISABLE);
		printk(UBATTERY "Toggle charger due to HW disable charger.\n");
	}
#endif

	/*------------------------------------------------------
	4 notify userspace if needed. */
/* MATT: last_capacity_01p and last_temp_01c are not used */
#if 0
	s_level = CEILING(poweralg.capacity_01p, 10);
	if (CEILING(poweralg.last_capacity_01p, 10) != s_level ||
		poweralg.battery.last_temp_01c != poweralg.battery.temp_01c) {
		/* notify level/temp change to userspace. */
		/* printk(UBATTERY "Send UEvent(level/temp)\n"); */
		poweralg.battery.last_temp_01c = poweralg.battery.temp_01c;
		poweralg.last_capacity_01p = poweralg.capacity_01p;
		htc_battery_core_update(BATTERY_SUPPLY);

#if !HTC_UBATTERY_DEBUG_ENABLE
		show_debug_message = TRUE;
#endif
	}
#endif

/* MATT: last_charging_source is used in func_update_charging_protect_flag. */
	if (poweralg.last_charging_source != poweralg.charging_source) {
		mutex_lock(&poweralg_lock);
		poweralg.last_charging_source = poweralg.charging_source;
		mutex_unlock(&poweralg_lock);
	}

	htc_battery_core_update_changed();

	/*------------------------------------------------------
	 5 debug messages and update os battery status*/
#if HTC_UBATTERY_DEBUG_ENABLE
	UBATT_LOG("S=%d P=%d chg=%d%d cable=%d%d%d flg=%d%d%d%d id=%d dbg=%d%d%d%d fst_dischg=%d/%d [%u]",
		poweralg.charge_state,
		poweralg.capacity_01p,
		poweralg.charging_enable,
		poweralg.low_current_charging_enable,
		poweralg.is_cable_in,
		poweralg.is_china_ac_in,
		poweralg.is_super_ac,
		poweralg.protect_flags.is_charging_enable_available,
		poweralg.protect_flags.is_charging_high_current_avaialble,
		poweralg.protect_flags.is_battery_dead,
		poweralg.protect_flags.is_temperature_fault,
		poweralg.battery.id_index,
		config.debug_disable_shutdown,
		config.debug_disable_temp_protect,
		config.debug_disable_hw_timer,
		config.debug_always_predict,
		poweralg.fst_discharge_capacity_01p,
		poweralg.fst_discharge_acr_mAh,
		ubattery_gettime_msec());
#else
	if (show_debug_message == TRUE)
		printk(UBATTERY "P=%d V=%d T=%d I=%d ACR=%d/%d KADC=%d charger=%d%d%d\n",
			poweralg.capacity_01p,
			poweralg.battery.voltage_mV,
			poweralg.battery.temp_01c,
			poweralg.battery.current_mA,
			poweralg.battery.charge_counter_mAh,
			poweralg.battery.charge_full_real_mAh,
			poweralg.battery.KADC_01p,
			poweralg.charging_source,
			poweralg.charging_enable,
			poweralg.low_current_charging_enable);
#endif
	return TRUE;
}

void ubattery_param_default_init(struct battery_type *batt_param)
{
	batt_param->id_index = 1;
	batt_param->temp_01c = 250;
	batt_param->last_temp_01c = batt_param->temp_01c;
	batt_param->temp_check_index = 0;
	batt_param->last_temp_adc = 0;
	batt_param->voltage_mV = 3550;
	batt_param->charge_full_real_mAh = 1200;
	batt_param->fc = 0;
	batt_param->RARC_01p = -1;
	batt_param->KADC_01p = -1;
}

void ubattery_poweralg_config_default_init(struct poweralg_config_type *config)
{
	printk(UBATTERY
		"[WARNING] default %s() is used.(hook your own in board\n",
		__func__);
	config->full_charging_mv = 5000;	/* disabled */
	config->full_charging_ma = 0;		/* disabled */
	config->full_pending_ma = 0;		/* disabled*/
	config->full_charging_timeout_sec = 0;	/* disabled */
	config->voltage_recharge_mv = 0;	/* disabled */
	config->capacity_recharge_p = 94;
	config->voltage_exit_full_mv = 0;	/* disabled */
	config->exit_full_p = 94;
	config->min_taper_current_mv = 4100;
	config->min_taper_current_ma = 500;
	config->exit_min_taper_current_ma = 600;
	config->wait_votlage_statble_sec = 1 * 60;
	config->predict_timeout_sec = 10;
	config->polling_time_normal_sec = 30;
	config->polling_time_extension1_sec = 60;

	config->enable_full_calibration = TRUE;
	config->enable_weight_percentage = TRUE;
	config->software_charger_timeout_sec = 0;   	 /* disabled*/
	config->superchg_software_charger_timeout_sec = 16 * 60 * 60;	/* 16 hrs */
	config->charger_hw_safety_timer_watchdog_sec = 4 * 60 * 60;	/*  4 hrs */

	config->debug_disable_shutdown = FALSE;
	config->debug_disable_temp_protect = FALSE;
	config->debug_disable_hw_timer = FALSE;
	config->debug_always_predict = FALSE;
	config->full_level = 0;
}

static void ubattery_power_alg_init(struct poweralg_config_type *debug_config)
{
	printk(UBATTERY "%s()\n",__func__);
	/*-------------------------------------------------------------
	1. setup default poweralg data*/
	mutex_lock(&poweralg_lock);
	poweralg.batt_state = 0;
	poweralg.last_charging_source = -1;
	poweralg.charging_source = 0;
	mutex_unlock(&poweralg_lock);
	poweralg.charge_state = CHARGE_STATE_UNKNOWN;
	poweralg.capacity_01p = 660;
	poweralg.last_capacity_01p = poweralg.capacity_01p;
	poweralg.fst_discharge_capacity_01p = 0;
	poweralg.fst_discharge_acr_mAh = 0;
	poweralg.is_need_calibrate_at_49p = TRUE;
	poweralg.is_need_calibrate_at_14p = TRUE;
	poweralg.is_charge_over_load = FALSE;
	poweralg.is_cable_in = FALSE;
	poweralg.is_china_ac_in = FALSE;
	poweralg.is_super_ac = FALSE;
	poweralg.is_voltage_stable = FALSE;
	poweralg.is_software_charger_timeout = FALSE;
	poweralg.is_superchg_software_charger_timeout = FALSE;
	poweralg.is_need_toggle_charger = FALSE;
	poweralg.last_charger_enable_toggled_time_ms = 0;
	poweralg.state_start_time_ms = 0;
	poweralg.low_current_charging_enable = FALSE;

	/* get cable type from usb driver dierectly for init state. */
	if (usb_is_connect_type_ready())
	{
#ifdef CONFIG_MACH_VERDI_LTE
		/* for D+/D- not shorted cable, before usb driver recognize it, we need to treat it as USB first
		   because usb drive won't trigger cable in even later, thus battery won't be informed   */
		if(usb_get_connect_type() == CONNECT_TYPE_UNKNOWN)
			ubattery_cable_status_handler_func(CONNECT_TYPE_USB);
		else
#endif
			ubattery_cable_status_handler_func(usb_get_connect_type());
	}
	else
		printk(UBATTERY "connect_type is not ready.\n");

	/*-------------------------------------------------------------
	2. setup default config flags (board dependent)*/
	if (poweralg.pdata && poweralg.pdata->func_poweralg_config_init)
		poweralg.pdata->func_poweralg_config_init(&config);
	else
		ubattery_poweralg_config_default_init(&config);

	if (debug_config){
		config.debug_disable_shutdown = debug_config->debug_disable_shutdown;
		config.debug_disable_temp_protect = debug_config->debug_disable_temp_protect;
		config.debug_disable_hw_timer = debug_config->debug_disable_hw_timer;
		config.debug_always_predict = debug_config->debug_always_predict;
	}

	/* if ( BAHW_IsTestMode() )
	 {
		config.debug_disable_shutdown = TRUE;
		config.debug_disable_temp_protect = TRUE;
		config.debug_disable_hw_timer = TRUE;
	 }*/

	/*-------------------------------------------------------------
	3. setup default protect flags*/
	poweralg.protect_flags.is_charging_enable_available = TRUE;
	poweralg.protect_flags.is_battery_dead = FALSE;
	poweralg.protect_flags.is_charging_high_current_avaialble = TRUE;
	poweralg.protect_flags.is_low_current_charging_enable = FALSE;
	poweralg.protect_flags.is_disable_temp_protect = config.debug_disable_temp_protect;

	/*-------------------------------------------------------------
	4. setup default battery structure*/
	/* this default_init is coded for smart gauge. other gauge like ds2746
	needs to init other parameters. */
	ubattery_param_default_init(&poweralg.battery);

	/*pr_info("power alg inited with board name <%s>\n", HTC_BATT_BOARD_NAME);*/
}

static void ubattery_program_alarm(struct ubattery_device_info *di, int seconds)
{
	ktime_t low_interval = ktime_set(seconds, 0);
	ktime_t slack = ktime_set(1, 0);
	ktime_t next;

	next = ktime_add(di->last_poll, low_interval);

	/* cancel last alarm */
	alarm_try_to_cancel(&di->alarm);
	printk(UBATTERY "%s(%d)\n",__func__,seconds);
	delta_time_sec = seconds;
	alarm_start_range(&di->alarm, next, ktime_add(next, slack));
}

static void ubattery_cable_status_handler_func(enum usb_connect_type online)
{
	u32 cable_type = (u32) online;
	if (cable_type == poweralg.charging_source) {
		printk(UBATTERY "%s(%u) same return\n",__func__, cable_type);
		return;
	}
	printk(UBATTERY "%s(%u)\n",__func__, cable_type);

	delta_time_sec = 0;
	mutex_lock(&poweralg_lock);
	poweralg.last_charging_source = poweralg.charging_source;
	poweralg.charging_source = cable_type;
	mutex_unlock(&poweralg_lock);
	/* STEP 1: update cable type relating variable
	only update these variables in this function. */
	if (cable_type == CONNECT_TYPE_NONE) {
		poweralg.is_cable_in = 0;
		poweralg.is_china_ac_in = 0;
		poweralg.is_super_ac = 0;
		poweralg.is_unknown_ac = 0;
		if (TRUE == poweralg.is_superchg_software_charger_timeout) {
			poweralg.is_superchg_software_charger_timeout = FALSE;	/* reset */
			printk(UBATTERY "reset superchg software timer\n");
		}
		wake_unlock(&vbus_wake_lock);
	} else if (cable_type == CONNECT_TYPE_USB) {
		wake_lock(&vbus_wake_lock);
		poweralg.is_cable_in = 1;
		poweralg.is_china_ac_in = 0;
		poweralg.is_super_ac = 0;
	} else if (cable_type == CONNECT_TYPE_AC) {
		poweralg.is_cable_in = 1;
		poweralg.is_china_ac_in = 1;
		poweralg.is_super_ac = 0;
	} else if (cable_type == CONNECT_TYPE_9V_AC) {
		poweralg.is_cable_in = 1;
		poweralg.is_china_ac_in = 1;
		poweralg.is_super_ac = 1;
	} else if (cable_type == CONNECT_TYPE_UNSUPPORTED) {
		poweralg.is_cable_in = 1;
		poweralg.is_china_ac_in = 0;
                poweralg.is_super_ac = 0;
		poweralg.is_unknown_ac = 1;
	}
	else {
		printk(UBATTERY "error: Unknown cable type(%d).\n", cable_type);
	}

	/* STEP2: trigger battery algorithm and uevent for cable change */
	if (g_di_ptr)
		ubattery_program_alarm(g_di_ptr, 0);
	else
		printk(UBATTERY "probe not completed yet.\n");


	return;
}

static struct t_cable_status_notifier cable_status_notifier = {
	.name = "ubattery_alg",
	.func = ubattery_cable_status_handler_func,
};

void ubattery_charger_control(int chg_ctl)
{

/*        reset superchg safety timer if charger disabled*/
	if (chg_ctl == DISABLE)
		super_chg_on_time_sec = 0;

	/* printk(UBATTERY "%s(%d)\n",__func__, chg_ctl); */
	if (TOGGLE_CHARGER == chg_ctl || ENABLE_MIN_TAPER == chg_ctl || DISABLE_MIN_TAPER == chg_ctl) {
		/* only set control at changes */
		if (poweralg.low_current_charging_enable == poweralg.protect_flags.is_low_current_charging_enable)
			return;
		poweralg.low_current_charging_enable = poweralg.protect_flags.is_low_current_charging_enable;
	} else if (poweralg.charge_state != CHARGE_STATE_PREDICTION) {
		/* only set control at changes or the first time since boot. */
		if (g_first_update_charger_ctl == 1) {
			/* because charger may be turned on by default
			since boot up. for syncing up real state, we
			force to do set charger contorl at first time. */
			printk(UBATTERY "first update charger control forcely.\n");
			g_first_update_charger_ctl = 0;
			poweralg.charging_enable = chg_ctl;
			if (poweralg.pdata->func_gpio_init) {
				poweralg.pdata->func_gpio_init(chg_ctl);
				return;
			}
		} else if (poweralg.charging_enable == chg_ctl)
			return;
		else
			poweralg.charging_enable = chg_ctl;
	} else {
		poweralg.charging_enable = DISABLE;
		chg_ctl = DISABLE;
		printk(UBATTERY "Charging disable due to PREDICTION state\n");
	}
	poweralg.pdata->func_charger_ctrl(chg_ctl);
	return;
}


static void ubattery_work(struct work_struct *work)
{
	struct ubattery_device_info *di = container_of(work,
				struct ubattery_device_info, monitor_work);
	unsigned long flags;
	static unsigned int meminfo_count = 0;

	if (get_kernel_flag() & BIT4) {
		meminfo_count++;
		if (meminfo_count == 20) {
			meminfo_count = 0;
			#if 0
			if (show_meminfo() == 0)
				show_vmallocinfo();
			#else
			show_meminfo();
			#endif
		}
	}
	ubattery_do_power_alg(0);
	di->last_poll = alarm_get_elapsed_realtime();
	/* prevent suspend before starting the alarm */
	local_irq_save(flags);

	wake_unlock(&di->work_wake_lock);
	if (poweralg.battery.is_power_on_reset)
		ubattery_program_alarm(di, PREDIC_POLL);
	else
		ubattery_program_alarm(di, config.polling_time_normal_sec);

	local_irq_restore(flags);
}

static void ubattery_do_trigger(struct work_struct *work)
{
	if (g_di_ptr) {
		printk(UBATTERY "%s\n",__func__);
		ubattery_program_alarm(g_di_ptr, 0);
	}
}

/* delay trigger set alarm */
static DECLARE_DELAYED_WORK(work_ubatt_trigger, ubattery_do_trigger);
static void ubattery_alarm(struct alarm *alarm)
{
	struct ubattery_device_info *di = container_of(alarm, struct ubattery_device_info, alarm);
	wake_lock(&di->work_wake_lock);
	queue_work(di->monitor_wqueue, &di->monitor_work);
}

static int ubattery_show_batt_attr(struct device_attribute *attr, char *buf)
{
	int len = 0;
	/*printk(UBATTERY "%s()\n",__func__);*/
	if (!strcmp(attr->attr.name, "batt_attr_text")){
		len += scnprintf(buf +
				len,
				PAGE_SIZE -
				len,
				"Percentage(%%): %d;\n"
				"KADC(%%): %d;\n"
				"RARC(%%): %d;\n"
				"SOC(%%): %d;\n"
				"FC: %d;\n"
				"V_MBAT(mV): %d;\n"
				"Battery_ID: %d;\n"
				"pd_M: %d;\n"
				"Current(mA): %d;\n"
				"Temp: %d;\n"
				"Charging_source: %d;\n"
				"ACR(mAh): %d;\n"
				"FULL(mAh): %d;\n"
				"1st_dis_percentage(%%): %d;\n"
				"1st_dis_ACR: %d;\n",
				CEILING(poweralg.capacity_01p, 10),
				CEILING(poweralg.battery.KADC_01p, 10),
				CEILING(poweralg.battery.RARC_01p, 10),
				CEILING(poweralg.battery.soc, 10),
				poweralg.battery.fc,
				poweralg.battery.voltage_mV,
				poweralg.battery.id_index,
				poweralg.battery.pd_m,
				poweralg.battery.current_mA,
				CEILING(poweralg.battery.temp_01c, 10),
				poweralg.charging_source,
				poweralg.battery.charge_counter_mAh,
				poweralg.battery.charge_full_real_mAh,
				CEILING(poweralg.fst_discharge_capacity_01p, 10),
				poweralg.fst_discharge_acr_mAh
		);
	}
	return len;
}

static int ubattery_get_battery_info(struct battery_info_reply *htc_batt_update)
{
	/* printk(UBATTERY "%s()\n",__func__);*/
	htc_batt_update->batt_id = poweralg.battery.id_index; /*Mbat ID*/
	htc_batt_update->batt_vol = poweralg.battery.voltage_mV; /*VMbat*/
	/* prevent framework shutdown device while temp > 68
		in temp protection disable mode */
	if (config.debug_disable_temp_protect &&
		(680 < poweralg.battery.temp_01c)) {
		htc_batt_update->batt_temp = 680; /* fake Temperature*/
	} else
		htc_batt_update->batt_temp = poweralg.battery.temp_01c; /*Temperature*/
	htc_batt_update->batt_current = poweralg.battery.current_mA; /*Current*/
	htc_batt_update->level = CEILING(poweralg.capacity_01p, 10); /*last_show%*/
	htc_batt_update->charging_source = poweralg.charging_source;
	htc_batt_update->charging_enabled = poweralg.charging_enable;
	htc_batt_update->full_bat = poweralg.battery.charge_full_real_mAh;
	htc_batt_update->temp_fault = poweralg.protect_flags.is_temperature_fault;
	htc_batt_update->batt_state = poweralg.batt_state;

	if(poweralg.is_unknown_ac)
		htc_batt_update->over_vchg = 1;
	else
		htc_batt_update->over_vchg = 0;


	return 0;
}

static int ubattery_charger_control_override(enum charger_control_flag ctl)
{
	/* override only suppport disable charging */
	printk(UBATTERY "%s(%d)\n", __func__, ctl);

	if (ctl == STOP_CHARGER) {
		/* set charger_override as disable */
		charger_control_override = 0;
		/* ubattery_charger_control(DISABLE);
		directly call may have race condition.
		SOL: use alarm trigger alg instead. */
		chg_en_time_sec = 0;
		super_chg_on_time_sec = 0;
		poweralg.is_need_toggle_charger = FALSE;
	} else {
		/* cancel override and set charger again*/
		charger_control_override = -1;
		/* ubattery_check_charging_function();
		directly call may have race condition.
		SOL: use alarm trigger alg instead. */
	}
	if (g_di_ptr)
		ubattery_program_alarm(g_di_ptr, 0);
	return 0;
}

static void ubattery_set_full_level(int full_level)
{
	/* in htc_battery_core.c, it restricts input
	 0 < full_level < 100 */
	if (full_level < 0)
		config.full_level = 0;
	else if (100 < full_level)
		config.full_level = 100;
	else
		config.full_level = full_level;
}

static struct htc_battery_core htc_battery_core = {
	.func_show_batt_attr = ubattery_show_batt_attr,
	.func_get_battery_info = ubattery_get_battery_info,
	.func_charger_control = ubattery_charger_control_override,
	.func_set_full_level = ubattery_set_full_level,
};

static int ubattery_probe(struct platform_device *pdev)
{
	int rc;
	struct ubattery_device_info *di;

	printk(UBATTERY "%s()\n",__func__);
	poweralg.pdata = pdev->dev.platform_data;

	/* TODO: MATT check if still need it */
	if (poweralg.pdata->func_get_thermal_id)
		poweralg.battery.thermal_id = poweralg.pdata->func_get_thermal_id();
	if (poweralg.pdata->func_get_battery_id != NULL) {
		poweralg.battery.id_index = poweralg.pdata->func_get_battery_id();
		is_need_battery_id_detection = FALSE;
	}
	else {
		poweralg.battery.id_index = BATTERY_ID_UNKNOWN;
		is_need_battery_id_detection = TRUE;
	}
#if 0
	if (poweralg.pdata->func_gpio_init)
		poweralg.pdata->func_gpio_init();
#endif

	ubattery_power_alg_init(&debug_config);

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di){
		rc = -ENOMEM;
		goto fail_register;
	}

	di->update_time = jiffies;
	platform_set_drvdata(pdev, di);

	di->dev = &pdev->dev;

	INIT_WORK(&di->monitor_work, ubattery_work);
	di->monitor_wqueue = create_singlethread_workqueue(dev_name(&pdev->dev));

	/* init to something sane */
	di->last_poll = alarm_get_elapsed_realtime();

	if (!di->monitor_wqueue){
		rc = -ESRCH;
		goto fail_workqueue;
	}
	wake_lock_init(&di->work_wake_lock, WAKE_LOCK_SUSPEND, "ubattery");
	printk(UBATTERY "%s: alarm_init\n",__func__);
	alarm_init(&di->alarm,
		ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
		ubattery_alarm);

	/* register htc_battery_core: after this, we takes resiposibility
		to report batt info and batt view to userspace. */
	htc_battery_core_register(&pdev->dev, &htc_battery_core);

	printk(UBATTERY "delay 3 sec trigger batt update\n");
	/* queue_work(di->monitor_wqueue, &di->monitor_work);*/
	schedule_delayed_work(&work_ubatt_trigger, 3*(HZ));

	g_di_ptr = di; /* save di to global */

	return 0;

	fail_workqueue : fail_register : kfree(di);
	return rc;
}


static int ubattery_remove(struct platform_device *pdev)
{
	struct ubattery_device_info *di = platform_get_drvdata(pdev);

	cancel_work_sync(&di->monitor_work);
	destroy_workqueue(di->monitor_wqueue);

	return 0;
}

/* FIXME: power down DQ master when not in use. */
static int ubattery_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct ubattery_device_info *di = platform_get_drvdata(pdev);
	unsigned long flags;
	/* If we are on battery, reduce our update rate until
	 * we next resume.*/
	if (poweralg.charging_source == CONNECT_TYPE_NONE) {
		local_irq_save(flags);
		ubattery_program_alarm(di, config.polling_time_extension1_sec);
		di->slow_poll = 1;
		local_irq_restore(flags);
	}
	return 0;
}
static void ubattery_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct ubattery_device_info *di = platform_get_drvdata(pdev);
	unsigned long flags;

	/* We might be on a slow sample cycle.  If we're
	 * resuming we should resample the battery state
	 * if it's been over a minute since we last did
	 * so, and move back to sampling every minute until
	 * we suspend again.*/
	if (di->slow_poll){
		local_irq_save(flags);
		ubattery_program_alarm(di, config.polling_time_normal_sec);
		di->slow_poll = 0;
		local_irq_restore(flags);
	}
}

static struct dev_pm_ops ubattery_pm_ops = {
       .prepare = ubattery_suspend,
       .complete  = ubattery_resume,
};

MODULE_ALIAS("platform:ubattery_alg");
static struct platform_driver ubattery_driver =
{
	.driver = {
	.name = "ubattery_alg",
	.pm = &ubattery_pm_ops,
	},
	.probe = ubattery_probe,
	.remove = ubattery_remove,
};

static int __init ubattery_fake_temp_setup(char *str)
{
	if(!strcmp(str,"true"))
		debug_config.debug_disable_temp_protect = TRUE;
	else
		debug_config.debug_disable_temp_protect = FALSE;
	return 1;
}
__setup("battery_fake_temp=", ubattery_fake_temp_setup);

static int __init ubattery_init(void)
{
	printk(UBATTERY "%s\n",__func__);
	wake_lock_init(&vbus_wake_lock, WAKE_LOCK_SUSPEND, "vbus_present");
	mutex_init(&poweralg_lock);
	cable_detect_register_notifier(&cable_status_notifier);
	/*mutex_init(&htc_batt_info.lock);*/
	return platform_driver_register(&ubattery_driver);
}

static void __exit ubattery_exit(void)
{
	platform_driver_unregister(&ubattery_driver);
}

module_init(ubattery_init);
module_exit(ubattery_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("MATT.SY YEH  <matt_yeh@htc.com>");
MODULE_DESCRIPTION("ubattery alg driver");

