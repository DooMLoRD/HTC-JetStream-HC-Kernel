
/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
#define HASTIMPANI 1
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/mfd/pmic8058.h>
#include <linux/mfd/bahama.h>
#include <linux/input/pmic8058-keypad.h>
#include <linux/pmic8058-pwrkey.h>
#include <linux/leds.h>
#include <linux/pmic8058-othc.h>
#include <linux/mfd/pmic8901.h>
#include <linux/regulator/pmic8058-regulator.h>
#include <linux/regulator/pmic8901-regulator.h>
#include <linux/bootmem.h>
#include <linux/pwm.h>
#include <linux/pmic8058-pwm.h>
#include <linux/leds-pm8058.h>
#include <linux/pmic8058-xoadc.h>
#include <linux/m_adc.h>
#include <linux/m_adcproc.h>
#include <linux/mfd/marimba.h>
#include <linux/proc_fs.h>
#include <linux/akm8975.h>
#include <linux/bma250.h>
#include <linux/ubattery_alg.h>
#include <mach/msm_serial_hs.h>
#include <mach/msm_serial_hs_lite.h>
#ifdef CONFIG_BT
#include <mach/htc_bdaddress.h>
#include <mach/htc_sleep_clk.h>
#endif
#include <linux/lightsensor.h>
#include <linux/cm3628.h>
#include <linux/mpu.h>

#include <linux/msm-charger.h>
#include <linux/i2c.h>
#include <linux/i2c/sx150x.h>
#include <linux/smsc911x.h>
#include <linux/spi/spi.h>
#include <linux/input/tdisc_shinetsu.h>

#ifdef CONFIG_ANDROID_PMEM
#include <linux/android_pmem.h>
#endif

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>

#include <mach/board.h>
#include <mach/mpp.h>
#include <mach/irqs.h>
#include <mach/irqs-8x60.h>
#include <mach/msm_spi.h>
#include <mach/msm_serial_hs.h>
#include <mach/msm_iomap.h>
#include <asm/mach/mmc.h>
#include <mach/htc_battery_core.h>
#include <linux/bq27510.h>
#include <mach/msm_hsusb.h>
#ifdef CONFIG_MSM_DSPS
#include <mach/msm_dsps.h>
#endif
#include <mach/msm_xo.h>
#include <mach/msm_bus_board.h>
#include <mach/msm_flashlight.h>
#include <mach/htc_headset_mgr.h>
#include <mach/htc_headset_gpio.h>
#include <mach/htc_headset_pmic.h>
#include <mach/htc_headset_misc.h>
#include <mach/htc_headset_8x60.h>
#include <linux/i2c/isl9519.h>
#include <mach/tpa2026.h>
#include <mach/tpa2028.h>
#include <linux/a1028.h>
#ifdef CONFIG_USB_ANDROID
#include <linux/usb/android_composite.h>
#endif
#include <mach/usb_gadget_fserial.h>

#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/delay.h>
#include <linux/rmt_storage_client-8x60.h>
#include <mach/sdio_al.h>
#include <mach/socinfo.h>

#include "board-verdi-lte.h"
#include "devices.h"
#include "devices-msm8x60.h"
#include "cpuidle.h"
#include "pm.h"
#include "rpm.h"
#include "spm.h"
#include "rpm_log.h"
#include "timer.h"
#include "saw-regulator.h"
#ifdef CONFIG_FB_MSM_HDMI_MHL
#include <mach/mhl.h>
#endif

#include "gpiomux.h"
#include "gpiomux-8x60.h"
#include "mpm.h"
#include "rpm-regulator.h"
#include "sysinfo-8x60.h"

#include <mach/htc_usb.h>
#include <mach/rpc_hsusb.h>
#include <mach/cable_detect.h>
#include <mach/panel_id.h>
#include "../../../drivers/staging/android/timed_gpio.h"
#include <linux/ntrig.h>
#include <mach/mdm.h>
#include <mach/htc_util.h>
#include <mach/board_htc.h>

#include "clock-8x60.h"
#include "rpm_stats.h"
#include <linux/cap_sense.h>

extern int panel_type;
#define MSM_SHARED_RAM_PHYS 0x40000000

#ifdef CONFIG_PERFLOCK
#include <mach/perflock.h>
#endif
#include "mach/msm_fb.h"
#include "mach/debug_display.h"
#include <mach/msm_panel.h>
/*
 * The UI_INTx_N lines are pmic gpio lines which connect i2c
 * gpio expanders to the pm8058.
 */
#define UI_INT1_N 25
#define UI_INT2_N 34
#define UI_INT3_N 14

#define PM8058_MPP_BASE			(PM8058_GPIO_BASE + PM8058_GPIOS)
#define PM8058_MPP_PM_TO_SYS(pm_gpio)		(pm_gpio + PM8058_MPP_BASE)

// -----------------------------------------------------------------------------
//                         External routine declaration
// -----------------------------------------------------------------------------
static void capsense_reset(void);

#ifdef CONFIG_FB_MSM_HDMI_MHL
extern void sii9234_change_usb_owner(bool bMHL);
#endif //CONFIG_FB_MSM_HDMI_MHL


#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
static void (*sdc2_status_notify_cb)(int card_present, void *dev_id);
static void *sdc2_status_notify_cb_devid;
#endif

#ifdef CONFIG_MMC_MSM_SDC5_SUPPORT
static void (*sdc5_status_notify_cb)(int card_present, void *dev_id);
static void *sdc5_status_notify_cb_devid;
#endif
#define _GET_REGULATOR(var, name) do {				\
	var = regulator_get(NULL, name);			\
	if (IS_ERR(var)) {					\
		pr_err("'%s' regulator not found, rc=%ld\n",	\
			name, IS_ERR(var));			\
		var = NULL;					\
		return -ENODEV;					\
	}							\
} while (0)

static struct msm_spm_platform_data msm_spm_data_v1[] __initdata = {
	[0] = {
		.reg_base_addr = MSM_SAW0_BASE,

#ifdef CONFIG_MSM_AVS_HW
		.reg_init_values[MSM_SPM_REG_SAW_AVS_CTL] = 0x586020FF,
#endif
		.reg_init_values[MSM_SPM_REG_SAW_CFG] = 0x0F,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_CTL] = 0x68,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_SLP_TMR_DLY] = 0xFFFFFFFF,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_WAKE_TMR_DLY] = 0xFFFFFFFF,

		.reg_init_values[MSM_SPM_REG_SAW_SLP_CLK_EN] = 0x11,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_PRECLMP_EN] = 0x07,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_POSTCLMP_EN] = 0x00,

		.reg_init_values[MSM_SPM_REG_SAW_SLP_CLMP_EN] = 0x01,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_RST_EN] = 0x00,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_MPM_CFG] = 0x00,

		.awake_vlevel = 0x94,
		.retention_vlevel = 0x81,
		.collapse_vlevel = 0x20,
		.retention_mid_vlevel = 0x94,
		.collapse_mid_vlevel = 0x8C,

		.vctl_timeout_us = 50,
	},

	[1] = {
		.reg_base_addr = MSM_SAW1_BASE,

#ifdef CONFIG_MSM_AVS_HW
		.reg_init_values[MSM_SPM_REG_SAW_AVS_CTL] = 0x586020FF,
#endif
		.reg_init_values[MSM_SPM_REG_SAW_CFG] = 0x0F,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_CTL] = 0x68,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_SLP_TMR_DLY] = 0xFFFFFFFF,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_WAKE_TMR_DLY] = 0xFFFFFFFF,

		.reg_init_values[MSM_SPM_REG_SAW_SLP_CLK_EN] = 0x13,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_PRECLMP_EN] = 0x07,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_POSTCLMP_EN] = 0x00,

		.reg_init_values[MSM_SPM_REG_SAW_SLP_CLMP_EN] = 0x01,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_RST_EN] = 0x00,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_MPM_CFG] = 0x00,

		.awake_vlevel = 0x94,
		.retention_vlevel = 0x81,
		.collapse_vlevel = 0x20,
		.retention_mid_vlevel = 0x94,
		.collapse_mid_vlevel = 0x8C,

		.vctl_timeout_us = 50,
	},
};

static struct msm_spm_platform_data msm_spm_data[] __initdata = {
	[0] = {
		.reg_base_addr = MSM_SAW0_BASE,

#ifdef CONFIG_MSM_AVS_HW
		.reg_init_values[MSM_SPM_REG_SAW_AVS_CTL] = 0x586020FF,
#endif
		.reg_init_values[MSM_SPM_REG_SAW_CFG] = 0x1C,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_CTL] = 0x68,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_SLP_TMR_DLY] = 0x0C0CFFFF,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_WAKE_TMR_DLY] = 0x78780FFF,

		.reg_init_values[MSM_SPM_REG_SAW_SLP_CLK_EN] = 0x11,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_PRECLMP_EN] = 0x07,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_POSTCLMP_EN] = 0x00,

		.reg_init_values[MSM_SPM_REG_SAW_SLP_CLMP_EN] = 0x01,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_RST_EN] = 0x00,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_MPM_CFG] = 0x00,

		.awake_vlevel = 0xA0,
		.retention_vlevel = 0x89,
		.collapse_vlevel = 0x20,
		.retention_mid_vlevel = 0x89,
		.collapse_mid_vlevel = 0x89,

		.vctl_timeout_us = 50,
	},

	[1] = {
		.reg_base_addr = MSM_SAW1_BASE,

#ifdef CONFIG_MSM_AVS_HW
		.reg_init_values[MSM_SPM_REG_SAW_AVS_CTL] = 0x586020FF,
#endif
		.reg_init_values[MSM_SPM_REG_SAW_CFG] = 0x1C,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_CTL] = 0x68,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_SLP_TMR_DLY] = 0x0C0CFFFF,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_WAKE_TMR_DLY] = 0x78780FFF,

		.reg_init_values[MSM_SPM_REG_SAW_SLP_CLK_EN] = 0x13,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_PRECLMP_EN] = 0x07,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_POSTCLMP_EN] = 0x00,

		.reg_init_values[MSM_SPM_REG_SAW_SLP_CLMP_EN] = 0x01,
		.reg_init_values[MSM_SPM_REG_SAW_SLP_RST_EN] = 0x00,
		.reg_init_values[MSM_SPM_REG_SAW_SPM_MPM_CFG] = 0x00,

		.awake_vlevel = 0xA0,
		.retention_vlevel = 0x89,
		.collapse_vlevel = 0x20,
		.retention_mid_vlevel = 0x89,
		.collapse_mid_vlevel = 0x89,

		.vctl_timeout_us = 50,
	},
};

static struct msm_acpu_clock_platform_data msm8x60_acpu_clock_data = {
};

#ifdef CONFIG_PERFLOCK
static unsigned puccini_perf_acpu_table[] = {
        540000000,
        1026000000,
        1512000000,
};

static struct perflock_platform_data puccini_perflock_data = {
        .perf_acpu_table = puccini_perf_acpu_table,
        .table_size = ARRAY_SIZE(puccini_perf_acpu_table),
};
#endif

static struct regulator_consumer_supply saw_s0_supply =
	REGULATOR_SUPPLY("8901_s0", NULL);
static struct regulator_consumer_supply saw_s1_supply =
	REGULATOR_SUPPLY("8901_s1", NULL);

static struct regulator_init_data saw_s0_init_data = {
		.constraints = {
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.min_uV = 840000,
			.max_uV = 1400000,
		},
		.num_consumer_supplies = 1,
		.consumer_supplies = &saw_s0_supply,
};

static struct regulator_init_data saw_s1_init_data = {
		.constraints = {
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			.min_uV = 840000,
			.max_uV = 1400000,
		},
		.num_consumer_supplies = 1,
		.consumer_supplies = &saw_s1_supply,
};

static struct platform_device msm_device_saw_s0 = {
	.name          = "saw-regulator",
	.id            = SAW_VREG_ID_S0,
	.dev           = {
		.platform_data = &saw_s0_init_data,
	},
};

static struct platform_device msm_device_saw_s1 = {
	.name          = "saw-regulator",
	.id            = SAW_VREG_ID_S1,
	.dev           = {
		.platform_data = &saw_s1_init_data,
	},
};

static struct akm8975_platform_data compass_platform_data = {
	.layouts = VERDI_LTE_LAYOUTS,
};

static struct i2c_board_info msm_i2c_gsbi10_akm8975_info[] = {
	{
		I2C_BOARD_INFO(AKM8975_I2C_NAME, 0x18 >> 1),
		.platform_data = &compass_platform_data,
		.irq = MSM_GPIO_TO_INT(VERDI_LTE_ECOMPASS_INT),
	},
};

static struct bma250_platform_data gsensor_platform_data = {
	.intr = VERDI_LTE_GSENSOR_INT,
	.chip_layout = 1,
};

static struct i2c_board_info msm_i2c_gsbi10_bma250_info[] = {
	{
		I2C_BOARD_INFO(BMA250_I2C_NAME, 0x30 >> 1),
		.platform_data = &gsensor_platform_data,
		.irq = MSM_GPIO_TO_INT(VERDI_LTE_GSENSOR_INT),
	},
};

static struct regulator *reg_8058_l12;	     	/* L-sensorI */
static struct pwm_device* pwm_backlight= NULL;

static int __capella_cm3602_power(int on)
{
	int rc;

	printk(KERN_INFO "%s: : %s\n",
		__func__, (on) ? "on" : "off");
	reg_8058_l12 = regulator_get(NULL, "8058_l12");
	if (IS_ERR(reg_8058_l12)) {
		pr_err("%s: Unable to get 8058_l12\n", __func__);
		rc = PTR_ERR(reg_8058_l12);
		return rc;
	}
	if (on) {
		rc = regulator_set_voltage(reg_8058_l12, 2850000, 2850000);
		if (!rc)
			rc = regulator_enable(reg_8058_l12);
		if (rc) {
			pr_err("'%s' regulator enable failed, rc=%d\n",
				"reg_8058_l12", rc);
			return rc;
		}
		pr_info("%s(on): success\n", __func__);
	} else {
		rc = regulator_disable(reg_8058_l12);
		if (rc) {
			pr_err("'%s' regulator disable failed, rc=%d\n",
				"reg_8058_l12", rc);
			return rc;
		}
		pr_info("%s(off): success\n", __func__);
	}
	return 0;
}

static DEFINE_MUTEX(capella_cm3602_lock);
static int als_power_control;

static int capella_cm3602_power(int pwr_device, uint8_t enable)
{

	unsigned int old_status = 0;
	int ret = 0, on = 0;
	static int power_on = 0;

	if (enable && power_on == 0) {
		power_on = 1;
		pr_info("%s(on): success\n", __func__);
	} else {
	/*not to disable power*/
		return 0;
	}

	mutex_lock(&capella_cm3602_lock);

	old_status = als_power_control;
	if (enable)
		als_power_control |= pwr_device;
	else
		als_power_control &= ~pwr_device;

	on = als_power_control ? 1 : 0;
	if (old_status == 0 && on)
		ret = __capella_cm3602_power(1);
	else if (!on)
		ret = __capella_cm3602_power(0);

	mutex_unlock(&capella_cm3602_lock);
	return ret;
}

static struct lightsensor_smd_platform_data lightsensor_data = {
	.levels = { 1, 3, 5, 2016, 2987, 21383, 37471, 51503,
			61739, 65535 },
	.golden_adc = 0x6DB0,
	.m_voltage = 1800,
	.ls_power = capella_cm3602_power,
	.mpp_data = {
               .lightsensor_mpp = XOADC_MPP_5,
               .lightsensor_amux = PM_MPP_AIN_AMUX_CH6
      },

};

static struct platform_device lightsensor_pdev = {
	.name = "lightsensor_pmic",
	.id = -1,
	.dev = {
		.platform_data = &lightsensor_data
	}
};

static struct msm_pm_platform_data msm_pm_data[MSM_PM_SLEEP_MODE_NR * 2] = {
	[MSM_PM_MODE(0, MSM_PM_SLEEP_MODE_POWER_COLLAPSE)] = {
		.supported = 1,
		.suspend_enabled = 1,
		.idle_enabled = 1,
		.latency = 4000,
		.residency = 13000,
	},

	[MSM_PM_MODE(0, MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE)] = {
		.supported = 1,
		.suspend_enabled = 1,
		.idle_enabled = 1,
		.latency = 500,
		.residency = 6000,
	},

	[MSM_PM_MODE(0, MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT)] = {
		.supported = 1,
		.suspend_enabled = 1,
		.idle_enabled = 1,
		.latency = 2,
		.residency = 0,
	},

	[MSM_PM_MODE(1, MSM_PM_SLEEP_MODE_POWER_COLLAPSE)] = {
		.supported = 1,
		.suspend_enabled = 1,
		.idle_enabled = 1,
		.latency = 600,
		.residency = 7200,
	},

	[MSM_PM_MODE(1, MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE)] = {
		.supported = 1,
		.suspend_enabled = 1,
		.idle_enabled = 1,
		.latency = 500,
		.residency = 6000,
	},

	[MSM_PM_MODE(1, MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT)] = {
		.supported = 1,
		.suspend_enabled = 1,
		.idle_enabled = 1,
		.latency = 2,
		.residency = 0,
	},
};

static struct msm_cpuidle_state msm_cstates[] __initdata = {
	{0, 0, "C0", "WFI",
		MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT},

	{0, 1, "C1", "STANDALONE_POWER_COLLAPSE",
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE},

	{0, 2, "C2", "POWER_COLLAPSE",
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE},

	{1, 0, "C0", "WFI",
		MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT},

	{1, 1, "C1", "STANDALONE_POWER_COLLAPSE",
		MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE},
};

static void config_gpio_table(uint32_t *table, int len);

static int verdi_lte_phy_init_seq[] = { 0x06, 0x36, 0x0C, 0x31, 0x31, 0x32, 0x1, 0x0E, 0x1, 0x11, -1 };
static struct regulator *ldo6_3p3;
static struct regulator *ldo7_1p8;

#define USB_PHY_3P3_VOL_MIN	3500000 /* uV */
#define USB_PHY_3P3_VOL_MAX	3500000 /* uV */
#define USB_PHY_3P3_HPM_LOAD	50000	/* uA */
#define USB_PHY_3P3_LPM_LOAD	4000	/* uA */

#define USB_PHY_1P8_VOL_MIN	1800000 /* uV */
#define USB_PHY_1P8_VOL_MAX	1800000 /* uV */
#define USB_PHY_1P8_HPM_LOAD	50000	/* uA */
#define USB_PHY_1P8_LPM_LOAD	4000	/* uA */
static int msm_hsusb_ldo_init(int init)
{
	int rc = 0;

	if (init) {
		ldo6_3p3 = regulator_get(NULL, "8058_l6");
		if (IS_ERR(ldo6_3p3))
			return PTR_ERR(ldo6_3p3);

		ldo7_1p8 = regulator_get(NULL, "8058_l7");
		if (IS_ERR(ldo7_1p8)) {
			rc = PTR_ERR(ldo7_1p8);
			goto put_3p3;
		}

		rc = regulator_set_voltage(ldo6_3p3, USB_PHY_3P3_VOL_MIN,
				USB_PHY_3P3_VOL_MAX);
		if (rc) {
			pr_err("%s: Unable to set voltage level for"
				"ldo6_3p3 regulator\n", __func__);
			goto put_1p8;
		}
		rc = regulator_enable(ldo6_3p3);
		if (rc) {
			pr_err("%s: Unable to enable the regulator:"
				"ldo6_3p3\n", __func__);
			goto put_1p8;
		}
		rc = regulator_set_voltage(ldo7_1p8, USB_PHY_1P8_VOL_MIN,
				USB_PHY_1P8_VOL_MAX);
		if (rc) {
			pr_err("%s: Unable to set voltage level for"
				"ldo7_1p8 regulator\n", __func__);
			goto disable_3p3;
		}
		rc = regulator_enable(ldo7_1p8);
		if (rc) {
			pr_err("%s: Unable to enable the regulator:"
				"ldo7_1p8\n", __func__);
			goto disable_3p3;
		}

		return 0;
	}

	regulator_disable(ldo7_1p8);
disable_3p3:
	regulator_disable(ldo6_3p3);
put_1p8:
	regulator_put(ldo7_1p8);
put_3p3:
	regulator_put(ldo6_3p3);
	return rc;
}

static int msm_hsusb_ldo_enable(int on)
{
	int ret = 0;

	if (!ldo7_1p8 || IS_ERR(ldo7_1p8)) {
		pr_err("%s: ldo7_1p8 is not initialized\n", __func__);
		return -ENODEV;
	}

	if (!ldo6_3p3 || IS_ERR(ldo6_3p3)) {
		pr_err("%s: ldo6_3p3 is not initialized\n", __func__);
		return -ENODEV;
	}

	if (on) {
		ret = regulator_set_optimum_mode(ldo7_1p8,
				USB_PHY_1P8_HPM_LOAD);
		if (ret < 0) {
			pr_err("%s: Unable to set HPM of the regulator:"
				"ldo7_1p8\n", __func__);
			return ret;
		}
		ret = regulator_set_optimum_mode(ldo6_3p3,
				USB_PHY_3P3_HPM_LOAD);
		if (ret < 0) {
			pr_err("%s: Unable to set HPM of the regulator:"
				"ldo6_3p3\n", __func__);
			regulator_set_optimum_mode(ldo7_1p8,
				USB_PHY_1P8_LPM_LOAD);
			return ret;
		}
	} else {
		ret = regulator_set_optimum_mode(ldo7_1p8,
				USB_PHY_1P8_LPM_LOAD);
		if (ret < 0)
			pr_err("%s: Unable to set LPM of the regulator:"
				"ldo7_1p8\n", __func__);
		ret = regulator_set_optimum_mode(ldo6_3p3,
				USB_PHY_3P3_LPM_LOAD);
		if (ret < 0)
			pr_err("%s: Unable to set LPM of the regulator:"
				"ldo6_3p3\n", __func__);
	}

	pr_debug("reg (%s)\n", on ? "HPM" : "LPM");
	return ret < 0 ? ret : 0;
 }

static void verdi_lte_usb_host_power_gpio_config(int on)
{
	struct pm8058_gpio h2w_pwr_en = {
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = 0,
		.output_value   = on,
		.pull           = PM_GPIO_PULL_NO,
		.vin_sel        = PM_GPIO_VIN_S3,
		.out_strength   = 0,
		.function       = PM_GPIO_FUNC_NORMAL,
	};

	static uint32_t cable_in1_gpio[] = {
		GPIO_CFG(VERDI_LTE_GPIO_CABLE_IN1_XD,
			0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
	};

	pm8058_gpio_config(VERDI_LTE_H2W_PWR_EN, &h2w_pwr_en);

	printk(KERN_INFO "%s USB Host support over-current \
				in XD (rev: %d)\n", "[USBH]", system_rev);
	if (system_rev > 2) {
		/* over-current detection in XD */
		config_gpio_table(cable_in1_gpio, ARRAY_SIZE(cable_in1_gpio));
	}
}

static void verdi_lte_h2wpower_switch(int enable)
{
	printk(KERN_INFO "%s: %d (rev:%d)\n", __func__, enable, system_rev);

	if (enable) {
		verdi_lte_usb_host_power_gpio_config(1);
#if 0
		pm8058_mpp_config_digital_out(VERDI_LTE_H2W_PWR_EN,
				PM8058_MPP_DIG_LEVEL_S3, PM_MPP_DOUT_CTL_HIGH);
#endif
		hr_msleep(10);
	} else {
		verdi_lte_usb_host_power_gpio_config(0);
#if 0
		pm8058_mpp_config_digital_out(VERDI_LTE_H2W_PWR_EN,
				PM8058_MPP_DIG_LEVEL_S3, PM_MPP_DOUT_CTL_LOW);
#endif
	}
}

static void verdi_lte_config_9v_gpio(int input)
{
	struct pm8058_gpio ac_9v_gpio_in = {
		.direction      = PM_GPIO_DIR_IN,
		.pull           = PM_GPIO_PULL_UP_30,
		.vin_sel        = PM_GPIO_VIN_L2,
		.function       = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol    = 0,
	};

	struct pm8058_gpio chg_stat_gpio_in = {
		.direction      = PM_GPIO_DIR_IN,
		.pull           = PM_GPIO_PULL_UP_30,
		.vin_sel        = PM_GPIO_VIN_L2,
		.function       = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol    = 0,
	};

	struct pm8058_gpio ac_9v_gpio_out = {
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
		.pull           = PM_GPIO_PULL_DN,
		.vin_sel        = PM_GPIO_VIN_L2,
		.function       = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol    = 0,
	};

	printk(KERN_INFO "%s: %d\n", __func__, input);
	pm8058_gpio_config(VERDI_LTE_CHG_STAT, &chg_stat_gpio_in);

	if (input)
		pm8058_gpio_config(VERDI_LTE_9V_AC_DETECT, &ac_9v_gpio_in);
	else
		pm8058_gpio_config(VERDI_LTE_9V_AC_DETECT, &ac_9v_gpio_out);
}

static struct msm_hsusb_platform_data msm_hsusb_pdata = {
	.phy_init_seq		= verdi_lte_phy_init_seq,
	.ldo_init		 = msm_hsusb_ldo_init,
	.ldo_enable		 = msm_hsusb_ldo_enable,
	.pclk_src_name	= "dfab_usb_hs_clk",
	.usb_id_pin_gpio	= VERDI_LTE_GPIO_USB_ID,
	.usb_host_switch	= verdi_lte_h2wpower_switch,
	.ac_9v_gpio		= PM8058_GPIO_PM_TO_SYS(VERDI_LTE_9V_AC_DETECT),
	.configure_ac_9v_gpio	= verdi_lte_config_9v_gpio,
	.chg_stat_irq		= PM8058_GPIO_IRQ(PM8058_IRQ_BASE,
				VERDI_LTE_CHG_STAT),
};

#ifdef CONFIG_USB_EHCI_MSM
static void msm_hsusb_vbus_power(unsigned phy_info, int on)
{
	static int vbus_is_on;

	/* If VBUS is already on (or off), do nothing. */
	if (unlikely(on == vbus_is_on))
		return;

	vbus_is_on = on;
}

static struct msm_usb_host_platform_data msm_usb_host_pdata = {
	.phy_info	= (USB_PHY_INTEGRATED | USB_PHY_MODEL_45NM),
	.power_budget	= 200, /* FIXME: 390, */
};
#endif


#if defined(CONFIG_BATTERY_MSM8X60) && !defined(CONFIG_USB_EHCI_MSM)
static int msm_hsusb_pmic_notif_init(void (*callback)(int online), int init)
{
	if (init)
		msm_charger_register_vbus_sn(callback);
	else
		msm_charger_unregister_vbus_sn(callback);
	return 0;
}
#endif
#if defined(CONFIG_USB_GADGET_MSM_72K) || defined(CONFIG_USB_EHCI_MSM)
static struct msm_otg_platform_data msm_otg_pdata = {
	/* if usb link is in sps there is no need for
	 * usb pclk as dayatona fabric clock will be
	 * used instead
	 */
	.usb_in_sps = 1,
	.pclk_src_name		 = "dfab_usb_hs_clk",
	.pemp_level		 = PRE_EMPHASIS_WITH_20_PERCENT,
	.cdr_autoreset		 = CDR_AUTO_RESET_DISABLE,
	.se1_gating		 = SE1_GATING_DISABLE,
#ifdef CONFIG_USB_EHCI_MSM
	.vbus_power = msm_hsusb_vbus_power,
#endif
#if defined(CONFIG_BATTERY_MSM8X60) && !defined(CONFIG_USB_EHCI_MSM)
	.pmic_notif_init         = msm_hsusb_pmic_notif_init,
#endif
	.ldo_init		 = msm_hsusb_ldo_init,
	.ldo_enable		 = msm_hsusb_ldo_enable,
#ifdef CONFIG_BATTERY_MSM8X60
	.chg_vbus_draw = msm_charger_vbus_draw,
#endif
	.drv_ampl		= HS_DRV_AMPLITUDE_DEFAULT,

};

static void verdi_lte_otg_overcurrent_gpio(void)
{
	if (system_rev > 2) {
		/* over-current detection in XD */
		msm_otg_pdata.usb_oc_pin = VERDI_LTE_GPIO_CABLE_IN1_XD;
		msm_otg_pdata.usb_oc_irq =
			MSM_GPIO_TO_INT(VERDI_LTE_GPIO_CABLE_IN1_XD);
	}
}
#endif
#ifdef CONFIG_FB_MSM_HDMI_MHL
static void mhl_sii9234_1v2_power(bool enable);
#endif
static uint32_t usb_ID_PIN_input_table[] = {
	GPIO_CFG(VERDI_LTE_GPIO_USB_ID, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static uint32_t usb_ID_PIN_ouput_table[] = {
	GPIO_CFG(VERDI_LTE_GPIO_USB_ID, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

void config_verdi_lte_usb_id_gpios(bool output)
{
	if (output) {
		config_gpio_table(usb_ID_PIN_ouput_table, ARRAY_SIZE(usb_ID_PIN_ouput_table));
		gpio_set_value(VERDI_LTE_GPIO_USB_ID, 1);
		printk(KERN_INFO "%s %d output high\n",  __func__, VERDI_LTE_GPIO_USB_ID);
	} else {
		config_gpio_table(usb_ID_PIN_input_table, ARRAY_SIZE(usb_ID_PIN_input_table));
		printk(KERN_INFO "%s %d input none pull\n",  __func__, VERDI_LTE_GPIO_USB_ID);
	}
}

static void pm8058_usb_config(void)
{
	int ret;

	ret = pm8058_mpp_config_digital_in(10, PM8058_MPP_DIG_LEVEL_S3, PM_MPP_DIN_TO_INT);
	if(ret != 0)
		printk(KERN_INFO "%s: MPP 10 fail\n", __func__);

	ret = pm8058_mpp_config_bi_dir(11, PM8058_MPP_DIG_LEVEL_S3, PM_MPP_BI_PULLUP_10KOHM);
	if(ret != 0)
		printk(KERN_INFO "%s: MPP 11 fail\n", __func__);
}

static uint32_t mhl_reset_pin_ouput_table[] = {
	GPIO_CFG(VERDI_LTE_GPIO_MHL_RESET, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
};

static uint32_t mhl_usb_switch_ouput_table[] = {
	GPIO_CFG(VERDI_LTE_GPIO_MHL_USB_SW, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
};

void config_verdi_lte_mhl_gpios(void)
{
	config_gpio_table(mhl_usb_switch_ouput_table,
			ARRAY_SIZE(mhl_usb_switch_ouput_table));
	config_gpio_table(mhl_reset_pin_ouput_table,
			ARRAY_SIZE(mhl_reset_pin_ouput_table));
}

static void verdi_lte_usb_dpdn_switch(int path)
{
	int mhl = 0, polarity = 0; /* low = mhl */

	switch (path) {
	case PATH_MHL:
		mhl = 1;
		/* go through */
	case PATH_USB:
		config_gpio_table(mhl_usb_switch_ouput_table,
				ARRAY_SIZE(mhl_usb_switch_ouput_table));
		verdi_lte_h2wpower_switch(mhl);

		pr_info("[CABLE] %s: Set %s path\n", __func__, mhl ? "MHL" : "USB");
		gpio_set_value(VERDI_LTE_GPIO_MHL_USB_SW, (mhl ^ !polarity) ? 1 : 0);
		break;
	}

	#ifdef CONFIG_FB_MSM_HDMI_MHL
	sii9234_change_usb_owner((path==PATH_MHL)?1:0);
	#endif //CONFIG_FB_MSM_HDMI_MHL
}

static struct cable_detect_platform_data cable_detect_pdata = {
	.vbus_mpp_gpio	= PM8058_MPP_PM_TO_SYS(10),
	.vbus_mpp_config = pm8058_usb_config,
	.vbus_mpp_irq  = PM8058_CBLPWR_IRQ(PM8058_IRQ_BASE),
	.detect_type = CABLE_TYPE_PMIC_ADC,
	.usb_id_pin_gpio = VERDI_LTE_GPIO_USB_ID,
	.usb_dpdn_switch = verdi_lte_usb_dpdn_switch,
	.mhl_reset_gpio = VERDI_LTE_GPIO_MHL_RESET,
	.mpp_data = {
		.usbid_mpp	=  XOADC_MPP_4,
		.usbid_amux	= PM_MPP_AIN_AMUX_CH5,
	},
	.config_usb_id_gpios = config_verdi_lte_usb_id_gpios,
	.ac_9v_gpio = PM8058_GPIO_PM_TO_SYS(VERDI_LTE_9V_AC_DETECT),
	.configure_ac_9v_gpio = verdi_lte_config_9v_gpio,
	.mhl_internal_3v3 = true,
#ifdef CONFIG_FB_MSM_HDMI_MHL
	.mhl_1v2_power = mhl_sii9234_1v2_power,
#endif
};

static struct platform_device cable_detect_device = {
	.name	= "cable_detect",
	.id	= -1,
	.dev	= {
		.platform_data = &cable_detect_pdata,
	},
};

#ifdef CONFIG_USB_ANDROID
static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= 2,
	.vendor		= "HTC",
	.product	= "Android Phone",
};

static struct platform_device usb_mass_storage_device = {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
		.platform_data = &mass_storage_pdata,
	},
};

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= 0x0BB4,
	.product_id	= 0x0cc8,
	.version	= 0x0100,
	.product_name		= "Android Phone",
	.manufacturer_name	= "HTC",
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
	.enable_fast_charge=NULL,
};
static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};

static int __init board_serialno_setup(char *serialno)
{
	android_usb_pdata.serial_number = serialno;
	return 1;
}
__setup("androidboot.serialno=", board_serialno_setup);
#endif

#ifdef CONFIG_MSM_VPE
static struct resource msm_vpe_resources[] = {
	{
		.start	= 0x05300000,
		.end	= 0x05300000 + SZ_1M - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_VPE,
		.end	= INT_VPE,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device msm_vpe_device = {
	.name = "msm_vpe",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_vpe_resources),
	.resource = msm_vpe_resources,
};
#endif

#if 1
static int flashlight_control(int mode)
{
#if CONFIG_ARCH_MSM_FLASHLIGHT
	return aat1277_flashlight_control(mode);
#else
	return 0;
#endif
}
#endif
#if CONFIG_ARCH_MSM_FLASHLIGHT
static void config_flashlight_gpios(void)
{
	static uint32_t flashlight_gpio_table[] = {
		GPIO_CFG(VERDI_LTE_TORCH_EN, 0, GPIO_CFG_OUTPUT,
			GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		GPIO_CFG(VERDI_LTE_FLASH_EN, 0, GPIO_CFG_OUTPUT,
			GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	};

	static struct pm8058_gpio flashlight_cfg = {
		.direction	= PM_GPIO_DIR_OUT,
		.output_value	= 0,
		.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
		.pull		= PM_GPIO_PULL_NO,
		.out_strength	= PM_GPIO_STRENGTH_HIGH,
		.function	= PM_GPIO_FUNC_NORMAL,
		.vin_sel	= PM_GPIO_VIN_L5,
		.inv_int_pol	= 0,
	};

	config_gpio_table(flashlight_gpio_table,
		ARRAY_SIZE(flashlight_gpio_table));

	if (system_rev == 0)
		pm8058_gpio_config(VERDI_LTE_TORCH_SET1, &flashlight_cfg);
	else
		pm8058_gpio_config(VERDI_LTE_TORCH_SET1_XB, &flashlight_cfg);

	pm8058_gpio_config(VERDI_LTE_TORCH_SET2, &flashlight_cfg);
}

static struct flashlight_platform_data flashlight_data = {
	.gpio_init 		= config_flashlight_gpios,
	.torch			= VERDI_LTE_TORCH_EN,
	.flash			= VERDI_LTE_FLASH_EN,
	.torch_set1 		= PM8058_GPIO_PM_TO_SYS(VERDI_LTE_TORCH_SET1),
	.torch_set2 		= PM8058_GPIO_PM_TO_SYS(VERDI_LTE_TORCH_SET2),
	.flash_duration_ms 	= 600,
	.chip_model 		= AAT1277,
};

static struct platform_device flashlight_device = {
	.name = FLASHLIGHT_NAME,
	.dev = {
		.platform_data	= &flashlight_data,
	},
};
#endif

static struct timed_gpio verdi_lte_timed_gpios_str[] = {
       {
		.name = "vibrator",
		.gpio = VERDI_LTE_GPIO_VIBRATOR_ON,
		.max_timeout = 15000,
       },
};

static struct timed_gpio_platform_data verdi_lte_timed_gpio_data = {
       .num_gpios      = ARRAY_SIZE(verdi_lte_timed_gpios_str),
       .gpios          = verdi_lte_timed_gpios_str,
};

static struct platform_device verdi_lte_timed_gpios = {
       .name           = TIMED_GPIO_NAME,
       .id             = -1,
       .dev            = {
		.platform_data  = &verdi_lte_timed_gpio_data,
       },
};

#undef CONFIG_SP3D /* disable SP3D */
#ifdef CONFIG_SP3D

static uint32_t camera_off_gpio_table_sp3d[] = {
	GPIO_CFG(VERDI_LTE_CAM_I2C_SDA, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
	GPIO_CFG(VERDI_LTE_CAM_I2C_SCL, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
	GPIO_CFG(VERDI_LTE_MCLK, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),/*MCLK*/
	GPIO_CFG(106, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_SP3D_SPI_DO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_SP3D_SPI_DI, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_SP3D_SPI_CS, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_SP3D_SPI_CLK, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_SP3D_GATE, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_SP3D_CORE_GATE, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_SP3D_SYS_RST, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_SP3D_PDX, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_WEBCAM_RST, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),/*camera reset*/
	GPIO_CFG(VERDI_LTE_WEBCAM_STB, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),/*camera standby*/
	GPIO_CFG(VERDI_LTE_CLK_SWITCH, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),/*camera switch*/
};

static uint32_t camera_on_gpio_table_sp3d[] = {
	GPIO_CFG(VERDI_LTE_CAM_I2C_SDA, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	GPIO_CFG(VERDI_LTE_CAM_I2C_SCL, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	GPIO_CFG(VERDI_LTE_MCLK, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),/*MCLK*/
	GPIO_CFG(106, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_SP3D_GATE, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	GPIO_CFG(VERDI_LTE_SP3D_CORE_GATE, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
	GPIO_CFG(VERDI_LTE_SP3D_SYS_RST, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
	GPIO_CFG(VERDI_LTE_SP3D_PDX, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
	GPIO_CFG(VERDI_LTE_WEBCAM_RST, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
	GPIO_CFG(VERDI_LTE_WEBCAM_STB, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
	GPIO_CFG(VERDI_LTE_CLK_SWITCH, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
	GPIO_CFG(VERDI_LTE_SP3D_SPI_DO,  1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
	GPIO_CFG(VERDI_LTE_SP3D_SPI_DI,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
	GPIO_CFG(VERDI_LTE_SP3D_SPI_CS,  1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	GPIO_CFG(VERDI_LTE_SP3D_SPI_CLK, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
};

static uint32_t sp3d_spi_gpio[] = {
#if 1 /* or this? the i/o direction and up/down are much more correct */
	GPIO_CFG(VERDI_LTE_SP3D_SPI_DO,  1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
	GPIO_CFG(VERDI_LTE_SP3D_SPI_DI,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
	GPIO_CFG(VERDI_LTE_SP3D_SPI_CS,  1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	GPIO_CFG(VERDI_LTE_SP3D_SPI_CLK, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
#endif
};
#endif/*CONFIG_SP3D*/

#ifdef CONFIG_S5K3H2YX

static uint32_t camera_off_gpio_table[] = {
	GPIO_CFG(VERDI_LTE_CAM_I2C_SDA, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_16MA),
	GPIO_CFG(VERDI_LTE_CAM_I2C_SCL, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_16MA),

	GPIO_CFG(VERDI_LTE_MCLK, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),/*MCLK*/
	GPIO_CFG(VERDI_LTE_CAM_INT, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_CAM_CORE_GATE /*58*/, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_CAM_RST/*137*/, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_CLK_SWITCH /*124*/, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_WEBCAM_RST, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_WEBCAM_STB, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),

	// by hardware comment, these are not real used, only for power concern
	GPIO_CFG(VERDI_LTE_CAM_GATE, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_CAM_MIX_RST, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_CAM_SPI_DO,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_CAM_SPI_DI, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_CAM_SPI_CS,  0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_CAM_SPI_CLK, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	// by hardware comment, these are not real used, only for power concern
};

static uint32_t camera_on_gpio_table[] = {
	GPIO_CFG(VERDI_LTE_CAM_I2C_SDA, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
	GPIO_CFG(VERDI_LTE_CAM_I2C_SCL, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),

	GPIO_CFG(VERDI_LTE_MCLK, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),/*MCLK*/
	GPIO_CFG(VERDI_LTE_CAM_INT, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_CAM_CORE_GATE /*58*/, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_CAM_RST /*137*/, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_CLK_SWITCH /*124*/, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_WEBCAM_RST, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_WEBCAM_STB, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),

	// by hardware comment, these are not real used, only for power concern
	GPIO_CFG(VERDI_LTE_CAM_GATE, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_CAM_MIX_RST, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_CAM_SPI_DO,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_CAM_SPI_DI, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_CAM_SPI_CS,  0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_CAM_SPI_CLK, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	// by hardware comment, these are not real used, only for power concern
};

#endif/*CONFIG_S5K3H2YX*/

static int camera_sensor_power_enable(char *power, unsigned volt)
{
	struct regulator *sensor_power;
	int rc;
	if (power == NULL)
		return -ENODEV;

	sensor_power = regulator_get(NULL, power);
	if (IS_ERR(sensor_power)) {
		pr_err("[Camera]%s: Unable to get %s\n", __func__, power);
		return -ENODEV;
	}
	rc = regulator_set_voltage(sensor_power, volt, volt);
	if (rc) {
		pr_err("[Camera]%s: unable to set %s voltage to %d rc:%d\n",
			__func__, power, volt, rc);
	}
	rc = regulator_enable(sensor_power);
	if (rc) {
		pr_err("[Camera]%s: Enable regulator %s failed\n", __func__, power);
	}
	regulator_put(sensor_power);
	return rc;
}


static int camera_sensor_power_disable(char *power)
{
	struct regulator *sensor_power;
	int rc;
	if (power == NULL)
		return -ENODEV;

	sensor_power = regulator_get(NULL, power);
	if (IS_ERR(sensor_power)) {
		pr_err("[Camera]%s: Unable to get %s\n", __func__, power);
		return -ENODEV;
	}
	rc = regulator_disable(sensor_power);
	if (rc) {
		regulator_put(sensor_power);
		pr_err("[Camera]%s: Enable regulator %s failed\n", __func__, power);
	}
	regulator_put(sensor_power);
	return rc;
}

#ifdef CONFIG_S5K3H2YX
static int verdi_lte_set_gpio(int gpio, int state){
	int rc;
	rc = gpio_request(gpio, "s5k3h2yx");
	if (!rc) {
		gpio_direction_output(gpio, state);
	} else {
		printk("gpio reset fail");
	}
	gpio_free(gpio);
	return rc;
}

static int Verdi_lte_s5k3h2yx_vreg_on(void)
{
	int rc = 0;
	pr_err("[CAM] %s\n", __func__);

	/* VCM */
	//PM8058_GPIO_07
	gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_SP3D_PMIC_AVDD), 1);

	if (rc < 0)
		goto init_fail;

	udelay(10);

	/* digital */
	rc = camera_sensor_power_enable("8058_l8", 1800000);
	pr_err("[CAM] sensor_power_enable(\"8058_l8\", 1.8V) == %d\n", rc);

	udelay(10);

	//CPU GPIO106 pull high
	{
		rc = gpio_request(VERDI_LTE_CAM_INT, "s5k3h2yx");
		if (!rc) {
			gpio_direction_output(VERDI_LTE_CAM_INT, 1);
		} else {
			pr_err("[Camera]GPIO (%d) request failed\n", VERDI_LTE_CAM_INT);
			goto init_fail;
		}
		gpio_free(VERDI_LTE_CAM_INT);
	}

	if (rc < 0)
		goto init_fail;

	/* IO */
	rc = camera_sensor_power_enable("8058_l9", 1800000);
	pr_err("[CAM] sensor_power_enable(\"8058_l9\", 1.8V) == %d\n", rc);

	if (rc < 0)
		goto init_fail;

	/* analog */
	rc = camera_sensor_power_enable("8901_l6", 2800000);
	pr_err("[CAM] sensor_power_enable(\"8901_l6\", 2.8V) == %d\n", rc);

init_fail:
	return rc;
}

static int Verdi_lte_s5k3h2yx_vreg_off(void)
{
	int rc = 0;
	pr_err("[CAM] %s\n", __func__);

	/* analog */
	rc = camera_sensor_power_disable("8901_l6");
	pr_err("[CAM] sensor_power_disable(\"8901_l6\") == %d\n", rc);

	/* VCM */
	//PM8058_GPIO_07
	gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_SP3D_PMIC_AVDD), 0);

	if (rc < 0)
		goto init_fail;

	udelay(10);

	//CPU GPIO106 pull low
	{
		rc = gpio_request(VERDI_LTE_CAM_INT, "s5k3h2yx");
		if (!rc) {
			gpio_direction_output(VERDI_LTE_CAM_INT, 0);
		} else {
			pr_err("[Camera]GPIO (%d) request failed\n", VERDI_LTE_CAM_INT);
			goto init_fail;
		}
		gpio_free(VERDI_LTE_CAM_INT);
	}

	udelay(10);

	/* digital */
	rc = camera_sensor_power_disable("8058_l8");
	pr_err("[CAM] sensor_power_disable(\"8058_l8\") == %d\n", rc);

	if (rc < 0)
		goto init_fail;

	/* IO */
	rc = camera_sensor_power_disable("8058_l9");
	pr_err("[CAM] sensor_power_disable(\"8058_l9\") == %d\n", rc);

init_fail:
	return rc;
}

#ifdef CONFIG_OV8830
static int Verdi_lte_ov8830_vreg_on(void)
{
	int rc = 0;
	//struct regulator * votg_2_8v_switch;
	pr_info("[CAM] %s\n", __func__);

	/* analog */
	//PM8058_GPIO_07
	gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_SP3D_PMIC_AVDD), 1);
	pr_info("[CAM] sensor_power_enable(AVDD, 2.8V) == %d\n", rc);

	msleep(1);

	//digital
	rc = camera_sensor_power_enable("8058_l8", 1800000);
	pr_info("[CAM] sensor_power_enable(\"8058_l8\", 1.8V) == %d\n", rc);

	if (rc < 0)
		goto init_fail;

	msleep(1);

	/* IO */
	rc = camera_sensor_power_enable("8058_l9", 1800000);
	pr_err("[CAM] sensor_power_enable(\"8058_l9\", 1.8V) == %d\n", rc);

	if (rc < 0)
		goto init_fail;

	msleep(1);

	/* VCM */
	rc = camera_sensor_power_enable("8901_l6", 2800000);
	pr_err("[CAM] sensor_power_enable(\"8901_l6\", 2.8V) == %d\n", rc);

	if (rc < 0)
		goto init_fail;


	pr_info("Verdi_lte_ov8830_vreg_on,end");

init_fail:
	return rc;
}

static int Verdi_lte_ov8830_vreg_off(void)
{
	int rc = 0;
	//struct regulator * votg_2_8v_switch;
	pr_info("[CAM] %s\n", __func__);

	/* VCM */
	rc = camera_sensor_power_disable("8901_l6");
	pr_err("[CAM] sensor_power_disable(\"8901_l6\") == %d\n", rc);

	if (rc < 0)
		goto init_fail;

	msleep(1);

	/* IO */
	rc = camera_sensor_power_disable("8058_l9");
	pr_err("[CAM] sensor_power_disable(\"8058_l9\") == %d\n", rc);

	if (rc < 0)
		goto init_fail;

	msleep(1);

	/* digital */
	rc = camera_sensor_power_disable("8058_l8");
	pr_err("[CAM] sensor_power_disable(\"8058_l8\") == %d\n", rc);

	if (rc < 0)
		goto init_fail;

	msleep(1);

	/* analog */
	//PM8058_GPIO_07
	gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_SP3D_PMIC_AVDD), 0);
	pr_info("[CAM] sensor_power_disable(AVDD, 2.8V) == %d\n", rc);

	msleep(1);

init_fail:
	return rc;
}
#endif


static void Verdi_lte_maincam_clk_switch(void)
{
	int rc = 0;
	pr_info("[Camera]Doing clk switch (Main Cam s5k3h2yx)\n");
	rc = gpio_request(VERDI_LTE_CLK_SWITCH, "s5k3h2yx");
	if (rc < 0)
		pr_err("[Camera]GPIO (%d) request fail\n", VERDI_LTE_CLK_SWITCH);
	else
		gpio_direction_output(VERDI_LTE_CLK_SWITCH, 0);
	gpio_free(VERDI_LTE_CLK_SWITCH);
	return;
}
#endif

#ifdef CONFIG_SP3D

static int verdi_set_gpio(int gpio, int state){
	int rc;
	rc = gpio_request(gpio, "sp3d");
	if (!rc) {
		gpio_direction_output(gpio, state);
	} else {
		printk("gpio reset fail");
	}
	gpio_free(gpio);
	return rc;
}

static int Verdi_sp3d_vreg_on(void)
{
	int rc;
	pr_info("[Camera]%s\n", __func__);
	/* main camera VDDIO */
	rc = camera_sensor_power_enable("8058_l9", 1800000);
	pr_info("[Camera]sensor_power_enable(\"8058_l9\", 1800) == %d\n", rc);
	udelay(26);
	/* main camera DVDD18 */
	pr_info("[Camera]sensor_power_enable(\"8058_l8\", 1800) == %d\n", rc);
	rc = camera_sensor_power_enable("8058_l8", 1800000);
	verdi_set_gpio(VERDI_LTE_SP3D_PDX, 1); // PDX
	verdi_set_gpio(VERDI_LTE_SP3D_GATE, 1); // GATE
	verdi_set_gpio(VERDI_LTE_SP3D_CORE_GATE, 1); // CORE GATE
	verdi_set_gpio(VERDI_LTE_SP3D_SYS_RST, 1); // RST
	/* main camera AVDD */
	gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_SP3D_PMIC_AVDD), 1);
	/* main camera MVDD */
	verdi_set_gpio(VERDI_LTE_SP3D_VCM, 1);
	gpio_tlmm_config(sp3d_spi_gpio[0], GPIO_CFG_ENABLE);
	gpio_tlmm_config(sp3d_spi_gpio[1], GPIO_CFG_ENABLE);
	gpio_tlmm_config(sp3d_spi_gpio[2], GPIO_CFG_ENABLE);
	gpio_tlmm_config(sp3d_spi_gpio[3], GPIO_CFG_ENABLE);
	return rc;
}

static int Verdi_sp3d_vreg_off(void)
{
	int rc;
	verdi_set_gpio(VERDI_LTE_SP3D_PDX, 0); // PDX
	/* main camera MVDD */
	verdi_set_gpio(VERDI_LTE_SP3D_VCM, 0);
	udelay(10);
	/* main camera AVDD */
	gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_SP3D_PMIC_AVDD), 0);
	udelay(10);
	/* main camera DVDD18 */
	rc = camera_sensor_power_disable("8058_l8");
	verdi_set_gpio(VERDI_LTE_SP3D_SYS_RST, 0); // RST
	verdi_set_gpio(VERDI_LTE_SP3D_CORE_GATE, 0); // CORE GATE
	verdi_set_gpio(VERDI_LTE_SP3D_GATE, 0); // GATE
	/* main camera VDDIO */
	rc = camera_sensor_power_disable("8058_l9");
	return rc;
}

static void Verdi_maincam_clk_switch(void)
{
	int rc = 0;
	pr_info("[Camera]Doing clk switch (Main Cam sp3d)\n");
	rc = gpio_request(VERDI_LTE_CLK_SWITCH, "sp3d");
	if (rc < 0)
		pr_err("[Camera]GPIO (%d) request fail\n", VERDI_LTE_CLK_SWITCH);
	else
		gpio_direction_output(VERDI_LTE_CLK_SWITCH, 0);
	gpio_free(VERDI_LTE_CLK_SWITCH);
	return;
}

#endif/*CONFIG_SP3D*/

static int Verdi_s5k6aafx_vreg_on(void)
{
	int rc;
	pr_info("[Camera]%s\n", __func__);

    /* main / 2nd camera analog power */
	rc = camera_sensor_power_enable("8058_l15", 2800000);
	pr_info("[Camera]sensor_power_enable(\"8058_l15\", 2800) == %d\n", rc);

    mdelay(5);

    /* main / 2nd camera digital power */
	rc = camera_sensor_power_enable("8058_l8", 1800000);
	pr_info("[Camera]sensor_power_enable(\"8058_l8\", 1800) == %d\n", rc);

    mdelay(5);

    /*IO*/
	rc = camera_sensor_power_enable("8058_l9", 1800000);
	pr_info("[Camera]sensor_power_enable(\"8058_l9\", 1800) == %d\n", rc);

    mdelay(1);

	/* VCM */
	rc = camera_sensor_power_enable("8901_l6", 2800000);
	pr_err("[CAM] sensor_power_enable(\"8901_l6\", 2.8V) == %d\n", rc);

	return rc;
}

static int Verdi_s5k6aafx_vreg_off(void)
{
	int rc;
	pr_info("[Camera]%s\n", __func__);

	/* VCM */
	rc = camera_sensor_power_disable("8901_l6");
	pr_err("[CAM] sensor_power_disable(\"8901_l6\") == %d\n", rc);

	/* IO power off */
	rc = camera_sensor_power_disable("8058_l9");
	pr_info("[Camera]sensor_power_disable(\"8058_l9\") == %d\n", rc);

    mdelay(1);

    /* main / 2nd camera digital power */
	rc = camera_sensor_power_disable("8058_l8");
	pr_info("[Camera]sensor_power_disable(\"8058_l8\") == %d\n", rc);

    mdelay(1);

	/* main / 2nd camera analog power */
	rc = camera_sensor_power_disable("8058_l15");
	pr_info("[Camera]sensor_power_disable(\"8058_l15\") == %d\n", rc);

	return rc;
}

static void Verdi_seccam_clk_switch(void)
{
	int rc = 0;
	pr_info("Doing clk switch (2nd Cam)\n");
	rc = gpio_request(VERDI_LTE_CLK_SWITCH, "s5k6aafx");

	if (rc < 0)
		pr_err("GPIO (%d) request fail\n", VERDI_LTE_CLK_SWITCH);
	else
		gpio_direction_output(VERDI_LTE_CLK_SWITCH, 1);

	gpio_free(VERDI_LTE_CLK_SWITCH);
	return;
}

#ifdef CONFIG_BATTERY_BQ27510
static int verdi_lte_battery_charging_ctrl(int ctl)
{
	int result = 0;

	switch (ctl) {
	case POWER_SUPPLY_DISABLE_CHARGE:
		BATT_LOG("charger OFF");
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_CHG_EN), 1);
		break;
	case POWER_SUPPLY_ENABLE_SLOW_CHARGE:
		BATT_LOG("charger ON (SLOW)");
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_CHG_USB), 0);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_CHG_5VAC), 0);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_CHG_EN), 0);
		break;
	case POWER_SUPPLY_ENABLE_FAST_CHARGE:
		BATT_LOG("charger ON (FAST)");
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_CHG_USB), 1);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_CHG_5VAC), 1);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_CHG_EN), 0);
		break;
	case POWER_SUPPLY_ENABLE_9VAC_CHARGE:
		BATT_LOG("charger ON (9V AC)");
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_CHG_USB), 1);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_CHG_5VAC), 0);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_CHG_EN), 0);
		break;
	case ENABLE_MIN_TAPER:
		BATT_LOG("Verdi-lte enable low battery charging current\n");
			gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_CHG_BATT), 1);
		break;
	case DISABLE_MIN_TAPER:
		BATT_LOG("Verdi-lte disable low battery charging current\n");
			gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_CHG_BATT), 0);
		break;
	default:
		BATT_LOG("Not supported battery ctr called.!");
		result = -EINVAL;
		break;
	}

	return result;
}

static int verdi_lte_battery_gpio_init(int ctl)
{
	struct pm8058_gpio battery_gpio_config_en = {
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 0,
		.pull           = PM_GPIO_PULL_NO,
		.vin_sel        = PM_GPIO_VIN_L2,
		.out_strength   = PM_GPIO_STRENGTH_HIGH,
		.function       = PM_GPIO_FUNC_NORMAL,
	};
	struct pm8058_gpio battery_gpio_config_usb = {
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 0,
		.pull           = PM_GPIO_PULL_NO,
		.vin_sel        = PM_GPIO_VIN_L2,
		.out_strength   = PM_GPIO_STRENGTH_HIGH,
		.function       = PM_GPIO_FUNC_NORMAL,
	};
	struct pm8058_gpio battery_gpio_config_5vac = {
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 0,
		.pull           = PM_GPIO_PULL_NO,
		.vin_sel        = PM_GPIO_VIN_L2,
		.out_strength   = PM_GPIO_STRENGTH_HIGH,
		.function       = PM_GPIO_FUNC_NORMAL,
	};
	struct pm8058_gpio battery_gpio_config_batt = {
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 0,
		.pull           = PM_GPIO_PULL_NO,
		.vin_sel        = PM_GPIO_VIN_L2,
		.out_strength   = PM_GPIO_STRENGTH_HIGH,
		.function       = PM_GPIO_FUNC_NORMAL,
	};

	switch (ctl) {
	case POWER_SUPPLY_DISABLE_CHARGE:
		BATT_LOG("init charger OFF");
		battery_gpio_config_en.output_value = 1;
		break;
	case POWER_SUPPLY_ENABLE_SLOW_CHARGE:
		BATT_LOG("init charger ON (SLOW)");
		battery_gpio_config_usb.output_value = 0;
		battery_gpio_config_5vac.output_value = 0;
		battery_gpio_config_en.output_value = 0;
		break;
	case POWER_SUPPLY_ENABLE_FAST_CHARGE:
		BATT_LOG("init charger ON (FAST)");
		battery_gpio_config_usb.output_value = 1;
		battery_gpio_config_5vac.output_value = 1;
		battery_gpio_config_en.output_value = 0;
		break;
	case POWER_SUPPLY_ENABLE_9VAC_CHARGE:
		BATT_LOG("init charger ON (9V AC)");
		battery_gpio_config_usb.output_value = 1;
		battery_gpio_config_5vac.output_value = 0;
		battery_gpio_config_en.output_value = 0;
		break;
	case ENABLE_MIN_TAPER:
	case DISABLE_MIN_TAPER:
	default:
		BATT_LOG("Not supported battery ctr called.!");
		break;
	}

	pm8058_gpio_config(VERDI_LTE_CHG_USB, &battery_gpio_config_usb);
	pm8058_gpio_config(VERDI_LTE_CHG_BATT, &battery_gpio_config_batt);
	pm8058_gpio_config(VERDI_LTE_CHG_5VAC, &battery_gpio_config_5vac);
	pm8058_gpio_config(VERDI_LTE_CHG_EN, &battery_gpio_config_en);
	return 0;
}

static struct bq27510_battery_platform_data bq27510_battery_data = {
	.guage_driver = GUAGE_BQ27510,
	.gpio_mbat_in = VERDI_LTE_GPIO_MBAT_IN,
	.charger = LINEAR_CHARGER,
};
#endif

static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("[Camera]%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}
#define GPIO_CAM_EN (GPIO_EXPANDER_GPIO_BASE + 13)
static void verdi_lte_config_camera_on_gpios(void)
{
#ifdef CONFIG_SP3D
	config_gpio_table(camera_on_gpio_table_sp3d,
		ARRAY_SIZE(camera_on_gpio_table_sp3d))
#endif

#ifdef CONFIG_S5K3H2YX
	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));
#endif
}

static void verdi_lte_config_camera_off_gpios(void)
{
#ifdef CONFIG_SP3D
	config_gpio_table(camera_off_gpio_table_sp3d,
		ARRAY_SIZE(camera_off_gpio_table_sp3d));
	verdi_set_gpio(VERDI_LTE_SP3D_SPI_DO, 0);
	verdi_set_gpio(VERDI_LTE_SP3D_SPI_DI, 0);
	verdi_set_gpio(VERDI_LTE_SP3D_SPI_CS, 0);
	verdi_set_gpio(VERDI_LTE_SP3D_SPI_CLK, 0);
	verdi_set_gpio(VERDI_LTE_WEBCAM_RST, 0);
	verdi_set_gpio(VERDI_LTE_CLK_SWITCH, 0);
	verdi_set_gpio(VERDI_LTE_MCLK, 0);
#endif

#ifdef CONFIG_S5K3H2YX
	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));
	verdi_lte_set_gpio(VERDI_LTE_MCLK, 0);
#endif
}

static struct msm_camera_device_platform_data msm_camera_device_data = {
	.camera_gpio_on  = verdi_lte_config_camera_on_gpios,
	.camera_gpio_off = verdi_lte_config_camera_off_gpios,
	.ioext.csiphy = 0x04800000,
	.ioext.csisz  = 0x00000400,
	.ioext.csiirq = CSI_0_IRQ,
	.ioclk.mclk_clk_rate = 24000000,
	.ioclk.vfe_clk_rate  = 228570000,
};

static struct msm_camera_device_platform_data msm_camera_device_data_web_cam = {
	.camera_gpio_on  = verdi_lte_config_camera_on_gpios,
	.camera_gpio_off = verdi_lte_config_camera_off_gpios,
	.ioext.csiphy = 0x04900000,
	.ioext.csisz  = 0x00000400,
	.ioext.csiirq = CSI_1_IRQ,
	.ioclk.mclk_clk_rate = 24000000,
	.ioclk.vfe_clk_rate  = 228570000,
};

static struct resource msm_camera_resources[] = {
	{
		.start	= 0x04500000,
		.end	= 0x04500000 + SZ_1M - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= VFE_IRQ,
		.end	= VFE_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};
static struct msm_camera_sensor_flash_src msm_flash_src = {
	.flash_sr_type				= MSM_CAMERA_FLASH_SRC_CURRENT_DRIVER,
	.camera_flash				= flashlight_control,
};

static struct camera_flash_cfg msm_camera_sensor_flash_cfg = {
	.low_temp_limit		= 5,
	.low_cap_limit		= 15,
};

#ifdef CONFIG_S5K3H2YX
static struct msm_camera_sensor_flash_data flash_s5k3h2yx = {
	.flash_type		= MSM_CAMERA_FLASH_LED,
	.flash_src		= &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_s5k3h2yx_data = {
	.sensor_name	= "s5k3h2yx",
	.sensor_reset 	= 0,
	.sensor_pwd 	= VERDI_LTE_CAM1_PWD,
	.vcm_pwd 		= VERDI_LTE_CAM1_VCM_PWD,
	.vcm_enable 	= 1,
	.camera_power_on = Verdi_lte_s5k3h2yx_vreg_on,
	.camera_power_off = Verdi_lte_s5k3h2yx_vreg_off,
	.camera_clk_switch = Verdi_lte_maincam_clk_switch,
	.pdata 			= &msm_camera_device_data,
	.resource 		= msm_camera_resources,
	.num_resources 	= ARRAY_SIZE(msm_camera_resources),
	.flash_data 	= &flash_s5k3h2yx,
	.flash_cfg = &msm_camera_sensor_flash_cfg,
	.mirror_mode 	= 0,
	.csi_if			= 1,
	.dev_node		= 0
};

static struct platform_device msm_camera_sensor_s5k3h2yx = {
	.name	= "msm_camera_s5k3h2yx",
	.dev	= {
		.platform_data = &msm_camera_sensor_s5k3h2yx_data,
	},
};
#endif

#ifdef CONFIG_OV8830
static struct msm_camera_sensor_flash_data flash_ov8830 = {
	.flash_type		= MSM_CAMERA_FLASH_LED,
	.flash_src		= &msm_flash_src
};


static struct msm_camera_sensor_info msm_camera_sensor_ov8830_data = {
	.sensor_name	= "ov8830",
	.sensor_reset	=	0,
	.sensor_pwd	= VERDI_LTE_CAM1_PWD,
	.vcm_pwd = VERDI_LTE_CAM1_VCM_PWD,
	.vcm_enable = 1,
	.camera_power_on = Verdi_lte_ov8830_vreg_on,
	.camera_power_off = Verdi_lte_ov8830_vreg_off,
	.camera_clk_switch	= Verdi_lte_maincam_clk_switch,
	.pdata = &msm_camera_device_data,
	.resource = msm_camera_resources,
	.num_resources = ARRAY_SIZE(msm_camera_resources),
	.flash_data = &flash_ov8830,
	.flash_cfg	= &msm_camera_sensor_flash_cfg,
	.mirror_mode = 0,
	.csi_if			= 1,
	.dev_node		= 0
};

static struct platform_device msm_camera_sensor_ov8830 = {
    .name	= "msm_camera_ov8830",
    .dev	= {
			.platform_data = &msm_camera_sensor_ov8830_data,
    },
};
#endif


#ifdef CONFIG_SP3D

static struct msm_camera_sensor_flash_data flash_sp3d = {
	.flash_type		= MSM_CAMERA_FLASH_LED,
	.flash_src		= &msm_flash_src
};

static struct spi_board_info sp3d_spi_board_info[] __initdata = {
	{
		.modalias	= "sp3d_spi",
		.mode		= SPI_MODE_3,
		.bus_num	= 3,
		.chip_select	= 0,
		.max_speed_hz	= 15060000,
	}
};

static struct msm_camera_sensor_info msm_camera_sensor_sp3d_data = {
	.sensor_name	= "sp3d",
	.sensor_reset	= VERDI_LTE_SP3D_SYS_RST, /*sys reset*/
	.sensor_pwd	= VERDI_LTE_SP3D_PDX, /*PDX*/
	.sp3d_gate	= VERDI_LTE_SP3D_GATE, /*GATE*/
	.sp3d_core_gate	= VERDI_LTE_SP3D_CORE_GATE, /*CORE GATE*/
	.vcm_pwd		= VERDI_LTE_SP3D_VCM,/*VCM_PD*/
	.vcm_enable		= 0,
	.camera_power_on = Verdi_sp3d_vreg_on,
	.camera_power_off = Verdi_sp3d_vreg_off,
	.camera_clk_switch = Verdi_maincam_clk_switch,
	.pdata			= &msm_camera_device_data,
	.resource		= msm_camera_resources,
	.num_resources	= ARRAY_SIZE(msm_camera_resources),
	.flash_data		= &flash_sp3d,
	.csi_if			= 1,
	.dev_node		= 0
};
struct platform_device msm_camera_sensor_sp3d = {
	.name	= "msm_camera_sp3d",
	.dev	= {
		.platform_data = &msm_camera_sensor_sp3d_data,
	},
};

#endif/*CONFIG_SP3D*/

static struct msm_camera_sensor_flash_data flash_s5k6aafx = {
	.flash_type		= MSM_CAMERA_FLASH_NONE,
};

static struct msm_camera_sensor_info msm_camera_sensor_s5k6aafx_data = {
	.sensor_name	= "s5k6aafx",
	.sensor_reset	= VERDI_LTE_WEBCAM_RST,/*2nd Cam RST*/
	.sensor_pwd		= VERDI_LTE_WEBCAM_STB,/*2nd Cam PWD*/
	.vcm_enable		= 0,
	.camera_power_on = Verdi_s5k6aafx_vreg_on,
	.camera_power_off = Verdi_s5k6aafx_vreg_off,
	.camera_clk_switch = Verdi_seccam_clk_switch,
	.pdata			= &msm_camera_device_data_web_cam,
	.resource		= msm_camera_resources,
	.num_resources	= ARRAY_SIZE(msm_camera_resources),
	.flash_data             = &flash_s5k6aafx,
	.mirror_mode        = 1,
	.csi_if			= 1,
	.dev_node		= 1,
};

static void __init msm8x60_init_camera(void)
{
	msm_camera_sensor_webcam.name = "msm_camera_webcam";
	msm_camera_sensor_webcam.dev.platform_data = &msm_camera_sensor_s5k6aafx_data;
}

static struct i2c_board_info msm_camera_boardinfo[] __initdata = {
#ifdef CONFIG_S5K3H2YX
	{
		I2C_BOARD_INFO("s5k3h2yx", 0x20 >> 1),
	},
#endif
	{
		I2C_BOARD_INFO("s5k6aafx", 0x78 >> 1),
	},
	{
		I2C_BOARD_INFO("s5k6aafx", 0x5a >> 1),
	},
};

static struct i2c_board_info msm_camera_boardinfo_OV8830[] __initdata = {
#ifdef CONFIG_OV8830
	{
		I2C_BOARD_INFO("ov8830", 0x20 >> 1),
	},
#endif
#ifdef CONFIG_S5K6AAFX
	{
		I2C_BOARD_INFO("s5k6aafx", 0x78 >> 1),
	},
#endif
	{
		I2C_BOARD_INFO("s5k6aafx", 0x5a >> 1),
	},
};


#ifdef CONFIG_MSM_GEMINI
static struct resource msm_gemini_resources[] = {
	{
		.start  = 0x04600000,
		.end    = 0x04600000 + SZ_1M - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = INT_JPEG,
		.end    = INT_JPEG,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device msm_gemini_device = {
	.name           = "msm_gemini",
	.resource       = msm_gemini_resources,
	.num_resources  = ARRAY_SIZE(msm_gemini_resources),
};
#endif

#ifdef CONFIG_BATTERY_BQ27510
static struct i2c_board_info msm_bq27510_boardinfo[] __initdata = {
	{
		I2C_BOARD_INFO("bq27510-battery", 0xAA >> 1),
		.platform_data = &bq27510_battery_data,
	},
};
#endif

extern void mdp_color_enhancement(const struct mdp_reg *reg_seq, int size);

struct mdp_reg mdp_img_stick_on[] = {
	{0x94800, 0x050505, 0x0},
	{0x94804, 0x050505, 0x0},
	{0x94808, 0x050505, 0x0},
	{0x9480C, 0x050505, 0x0},
	{0x94810, 0x050505, 0x0},
	{0x94814, 0x050505, 0x0},
	{0x94818, 0x060606, 0x0},
	{0x9481C, 0x070707, 0x0},
	{0x94820, 0x080808, 0x0},
	{0x94824, 0x090909, 0x0},
	{0x94828, 0x0A0A0A, 0x0},
	{0x9482C, 0x0B0B0B, 0x0},
	{0x94830, 0x0C0C0C, 0x0},
	{0x94834, 0x0D0D0D, 0x0},
	{0x94838, 0x0E0E0E, 0x0},
	{0x9483C, 0x0F0F0F, 0x0},
	{0x94840, 0x101010, 0x0},
	{0x94844, 0x111111, 0x0},
	{0x94848, 0x121212, 0x0},
	{0x9484C, 0x131313, 0x0},
	{0x94850, 0x141414, 0x0},
	{0x94854, 0x151515, 0x0},
	{0x94858, 0x161616, 0x0},
	{0x9485C, 0x171717, 0x0},
	{0x94860, 0x181818, 0x0},
	{0x94864, 0x191919, 0x0},
	{0x94868, 0x1A1A1A, 0x0},
	{0x9486C, 0x1B1B1B, 0x0},
	{0x94870, 0x1C1C1C, 0x0},
	{0x94874, 0x1D1D1D, 0x0},
	{0x94878, 0x1E1E1E, 0x0},
	{0x9487C, 0x1F1F1F, 0x0},
	{0x94880, 0x202020, 0x0},
	{0x94884, 0x212121, 0x0},
	{0x94888, 0x222222, 0x0},
	{0x9488C, 0x232323, 0x0},
	{0x94890, 0x242424, 0x0},
	{0x94894, 0x252525, 0x0},
	{0x94898, 0x262626, 0x0},
	{0x9489C, 0x272727, 0x0},
	{0x948A0, 0x282828, 0x0},
	{0x948A4, 0x292929, 0x0},
	{0x948A8, 0x2A2A2A, 0x0},
	{0x948AC, 0x2B2B2B, 0x0},
	{0x948B0, 0x2C2C2C, 0x0},
	{0x948B4, 0x2D2D2D, 0x0},
	{0x948B8, 0x2E2E2E, 0x0},
	{0x948BC, 0x2F2F2F, 0x0},
	{0x948C0, 0x303030, 0x0},
	{0x948C4, 0x313131, 0x0},
	{0x948C8, 0x323232, 0x0},
	{0x948CC, 0x333333, 0x0},
	{0x948D0, 0x343434, 0x0},
	{0x948D4, 0x353535, 0x0},
	{0x948D8, 0x363636, 0x0},
	{0x948DC, 0x373737, 0x0},
	{0x948E0, 0x383838, 0x0},
	{0x948E4, 0x393939, 0x0},
	{0x948E8, 0x3A3A3A, 0x0},
	{0x948EC, 0x3B3B3B, 0x0},
	{0x948F0, 0x3C3C3C, 0x0},
	{0x948F4, 0x3D3D3D, 0x0},
	{0x948F8, 0x3E3E3E, 0x0},
	{0x948FC, 0x3F3F3F, 0x0},
	{0x94900, 0x404040, 0x0},
	{0x94904, 0x414141, 0x0},
	{0x94908, 0x424242, 0x0},
	{0x9490C, 0x434343, 0x0},
	{0x94910, 0x444444, 0x0},
	{0x94914, 0x454545, 0x0},
	{0x94918, 0x464646, 0x0},
	{0x9491C, 0x474747, 0x0},
	{0x94920, 0x484848, 0x0},
	{0x94924, 0x494949, 0x0},
	{0x94928, 0x4A4A4A, 0x0},
	{0x9492C, 0x4B4B4B, 0x0},
	{0x94930, 0x4C4C4C, 0x0},
	{0x94934, 0x4D4D4D, 0x0},
	{0x94938, 0x4E4E4E, 0x0},
	{0x9493C, 0x4F4F4F, 0x0},
	{0x94940, 0x505050, 0x0},
	{0x94944, 0x515151, 0x0},
	{0x94948, 0x525252, 0x0},
	{0x9494C, 0x535353, 0x0},
	{0x94950, 0x545454, 0x0},
	{0x94954, 0x555555, 0x0},
	{0x94958, 0x565656, 0x0},
	{0x9495C, 0x575757, 0x0},
	{0x94960, 0x585858, 0x0},
	{0x94964, 0x595959, 0x0},
	{0x94968, 0x5A5A5A, 0x0},
	{0x9496C, 0x5B5B5B, 0x0},
	{0x94970, 0x5C5C5C, 0x0},
	{0x94974, 0x5D5D5D, 0x0},
	{0x94978, 0x5E5E5E, 0x0},
	{0x9497C, 0x5F5F5F, 0x0},
	{0x94980, 0x606060, 0x0},
	{0x94984, 0x616161, 0x0},
	{0x94988, 0x626262, 0x0},
	{0x9498C, 0x636363, 0x0},
	{0x94990, 0x646464, 0x0},
	{0x94994, 0x656565, 0x0},
	{0x94998, 0x666666, 0x0},
	{0x9499C, 0x676767, 0x0},
	{0x949A0, 0x686868, 0x0},
	{0x949A4, 0x696969, 0x0},
	{0x949A8, 0x6A6A6A, 0x0},
	{0x949AC, 0x6B6B6B, 0x0},
	{0x949B0, 0x6C6C6C, 0x0},
	{0x949B4, 0x6D6D6D, 0x0},
	{0x949B8, 0x6E6E6E, 0x0},
	{0x949BC, 0x6F6F6F, 0x0},
	{0x949C0, 0x707070, 0x0},
	{0x949C4, 0x717171, 0x0},
	{0x949C8, 0x727272, 0x0},
	{0x949CC, 0x737373, 0x0},
	{0x949D0, 0x747474, 0x0},
	{0x949D4, 0x757575, 0x0},
	{0x949D8, 0x767676, 0x0},
	{0x949DC, 0x777777, 0x0},
	{0x949E0, 0x787878, 0x0},
	{0x949E4, 0x797979, 0x0},
	{0x949E8, 0x7A7A7A, 0x0},
	{0x949EC, 0x7B7B7B, 0x0},
	{0x949F0, 0x7C7C7C, 0x0},
	{0x949F4, 0x7D7D7D, 0x0},
	{0x949F8, 0x7E7E7E, 0x0},
	{0x949FC, 0x7F7F7F, 0x0},
	{0x94A00, 0x808080, 0x0},
	{0x94A04, 0x818181, 0x0},
	{0x94A08, 0x828282, 0x0},
	{0x94A0C, 0x838383, 0x0},
	{0x94A10, 0x848484, 0x0},
	{0x94A14, 0x858585, 0x0},
	{0x94A18, 0x868686, 0x0},
	{0x94A1C, 0x878787, 0x0},
	{0x94A20, 0x888888, 0x0},
	{0x94A24, 0x898989, 0x0},
	{0x94A28, 0x8A8A8A, 0x0},
	{0x94A2C, 0x8B8B8B, 0x0},
	{0x94A30, 0x8C8C8C, 0x0},
	{0x94A34, 0x8D8D8D, 0x0},
	{0x94A38, 0x8E8E8E, 0x0},
	{0x94A3C, 0x8F8F8F, 0x0},
	{0x94A40, 0x909090, 0x0},
	{0x94A44, 0x919191, 0x0},
	{0x94A48, 0x929292, 0x0},
	{0x94A4C, 0x939393, 0x0},
	{0x94A50, 0x949494, 0x0},
	{0x94A54, 0x959595, 0x0},
	{0x94A58, 0x969696, 0x0},
	{0x94A5C, 0x979797, 0x0},
	{0x94A60, 0x989898, 0x0},
	{0x94A64, 0x999999, 0x0},
	{0x94A68, 0x9A9A9A, 0x0},
	{0x94A6C, 0x9B9B9B, 0x0},
	{0x94A70, 0x9C9C9C, 0x0},
	{0x94A74, 0x9D9D9D, 0x0},
	{0x94A78, 0x9E9E9E, 0x0},
	{0x94A7C, 0x9F9F9F, 0x0},
	{0x94A80, 0xA0A0A0, 0x0},
	{0x94A84, 0xA1A1A1, 0x0},
	{0x94A88, 0xA2A2A2, 0x0},
	{0x94A8C, 0xA3A3A3, 0x0},
	{0x94A90, 0xA4A4A4, 0x0},
	{0x94A94, 0xA5A5A5, 0x0},
	{0x94A98, 0xA6A6A6, 0x0},
	{0x94A9C, 0xA7A7A7, 0x0},
	{0x94AA0, 0xA8A8A8, 0x0},
	{0x94AA4, 0xA9A9A9, 0x0},
	{0x94AA8, 0xAAAAAA, 0x0},
	{0x94AAC, 0xABABAB, 0x0},
	{0x94AB0, 0xACACAC, 0x0},
	{0x94AB4, 0xADADAD, 0x0},
	{0x94AB8, 0xAEAEAE, 0x0},
	{0x94ABC, 0xAFAFAF, 0x0},
	{0x94AC0, 0xB0B0B0, 0x0},
	{0x94AC4, 0xB1B1B1, 0x0},
	{0x94AC8, 0xB2B2B2, 0x0},
	{0x94ACC, 0xB3B3B3, 0x0},
	{0x94AD0, 0xB4B4B4, 0x0},
	{0x94AD4, 0xB5B5B5, 0x0},
	{0x94AD8, 0xB6B6B6, 0x0},
	{0x94ADC, 0xB7B7B7, 0x0},
	{0x94AE0, 0xB8B8B8, 0x0},
	{0x94AE4, 0xB9B9B9, 0x0},
	{0x94AE8, 0xBABABA, 0x0},
	{0x94AEC, 0xBBBBBB, 0x0},
	{0x94AF0, 0xBCBCBC, 0x0},
	{0x94AF4, 0xBDBDBD, 0x0},
	{0x94AF8, 0xBEBEBE, 0x0},
	{0x94AFC, 0xBFBFBF, 0x0},
	{0x94B00, 0xC0C0C0, 0x0},
	{0x94B04, 0xC1C1C1, 0x0},
	{0x94B08, 0xC2C2C2, 0x0},
	{0x94B0C, 0xC3C3C3, 0x0},
	{0x94B10, 0xC4C4C4, 0x0},
	{0x94B14, 0xC5C5C5, 0x0},
	{0x94B18, 0xC6C6C6, 0x0},
	{0x94B1C, 0xC7C7C7, 0x0},
	{0x94B20, 0xC8C8C8, 0x0},
	{0x94B24, 0xC9C9C9, 0x0},
	{0x94B28, 0xCACACA, 0x0},
	{0x94B2C, 0xCBCBCB, 0x0},
	{0x94B30, 0xCCCCCC, 0x0},
	{0x94B34, 0xCDCDCD, 0x0},
	{0x94B38, 0xCECECE, 0x0},
	{0x94B3C, 0xCFCFCF, 0x0},
	{0x94B40, 0xD0D0D0, 0x0},
	{0x94B44, 0xD1D1D1, 0x0},
	{0x94B48, 0xD2D2D2, 0x0},
	{0x94B4C, 0xD3D3D3, 0x0},
	{0x94B50, 0xD4D4D4, 0x0},
	{0x94B54, 0xD5D5D5, 0x0},
	{0x94B58, 0xD6D6D6, 0x0},
	{0x94B5C, 0xD7D7D7, 0x0},
	{0x94B60, 0xD8D8D8, 0x0},
	{0x94B64, 0xD9D9D9, 0x0},
	{0x94B68, 0xDADADA, 0x0},
	{0x94B6C, 0xDBDBDB, 0x0},
	{0x94B70, 0xDCDCDC, 0x0},
	{0x94B74, 0xDDDDDD, 0x0},
	{0x94B78, 0xDEDEDE, 0x0},
	{0x94B7C, 0xDFDFDF, 0x0},
	{0x94B80, 0xE0E0E0, 0x0},
	{0x94B84, 0xE1E1E1, 0x0},
	{0x94B88, 0xE2E2E2, 0x0},
	{0x94B8C, 0xE3E3E3, 0x0},
	{0x94B90, 0xE4E4E4, 0x0},
	{0x94B94, 0xE5E5E5, 0x0},
	{0x94B98, 0xE6E6E6, 0x0},
	{0x94B9C, 0xE7E7E7, 0x0},
	{0x94BA0, 0xE8E8E8, 0x0},
	{0x94BA4, 0xE9E9E9, 0x0},
	{0x94BA8, 0xEAEAEA, 0x0},
	{0x94BAC, 0xEBEBEB, 0x0},
	{0x94BB0, 0xECECEC, 0x0},
	{0x94BB4, 0xEDEDED, 0x0},
	{0x94BB8, 0xEEEEEE, 0x0},
	{0x94BBC, 0xEFEFEF, 0x0},
	{0x94BC0, 0xF0F0F0, 0x0},
	{0x94BC4, 0xF1F1F1, 0x0},
	{0x94BC8, 0xF2F2F2, 0x0},
	{0x94BCC, 0xF3F3F3, 0x0},
	{0x94BD0, 0xF4F4F4, 0x0},
	{0x94BD4, 0xF5F5F5, 0x0},
	{0x94BD8, 0xF6F6F6, 0x0},
	{0x94BDC, 0xF7F7F7, 0x0},
	{0x94BE0, 0xF8F8F8, 0x0},
	{0x94BE4, 0xF9F9F9, 0x0},
	{0x94BE8, 0xFAFAFA, 0x0},
	{0x94BEC, 0xFAFAFA, 0x0},
	{0x94BF0, 0xFAFAFA, 0x0},
	{0x94BF4, 0xFAFAFA, 0x0},
	{0x94BF8, 0xFAFAFA, 0x0},
	{0x94BFC, 0xFAFAFA, 0x0},
	{0x90070, 0x17	  , 0x17},

};

struct mdp_reg mdp_img_stick_off[] = {
	{0x94800, 0x000000, 0x0},
	{0x94804, 0x010101, 0x0},
	{0x94808, 0x020202, 0x0},
	{0x9480C, 0x030303, 0x0},
	{0x94810, 0x040404, 0x0},
	{0x94814, 0x050505, 0x0},
	{0x94818, 0x060606, 0x0},
	{0x9481C, 0x070707, 0x0},
	{0x94820, 0x080808, 0x0},
	{0x94824, 0x090909, 0x0},
	{0x94828, 0x0A0A0A, 0x0},
	{0x9482C, 0x0B0B0B, 0x0},
	{0x94830, 0x0C0C0C, 0x0},
	{0x94834, 0x0D0D0D, 0x0},
	{0x94838, 0x0E0E0E, 0x0},
	{0x9483C, 0x0F0F0F, 0x0},
	{0x94840, 0x101010, 0x0},
	{0x94844, 0x111111, 0x0},
	{0x94848, 0x121212, 0x0},
	{0x9484C, 0x131313, 0x0},
	{0x94850, 0x141414, 0x0},
	{0x94854, 0x151515, 0x0},
	{0x94858, 0x161616, 0x0},
	{0x9485C, 0x171717, 0x0},
	{0x94860, 0x181818, 0x0},
	{0x94864, 0x191919, 0x0},
	{0x94868, 0x1A1A1A, 0x0},
	{0x9486C, 0x1B1B1B, 0x0},
	{0x94870, 0x1C1C1C, 0x0},
	{0x94874, 0x1D1D1D, 0x0},
	{0x94878, 0x1E1E1E, 0x0},
	{0x9487C, 0x1F1F1F, 0x0},
	{0x94880, 0x202020, 0x0},
	{0x94884, 0x212121, 0x0},
	{0x94888, 0x222222, 0x0},
	{0x9488C, 0x232323, 0x0},
	{0x94890, 0x242424, 0x0},
	{0x94894, 0x252525, 0x0},
	{0x94898, 0x262626, 0x0},
	{0x9489C, 0x272727, 0x0},
	{0x948A0, 0x282828, 0x0},
	{0x948A4, 0x292929, 0x0},
	{0x948A8, 0x2A2A2A, 0x0},
	{0x948AC, 0x2B2B2B, 0x0},
	{0x948B0, 0x2C2C2C, 0x0},
	{0x948B4, 0x2D2D2D, 0x0},
	{0x948B8, 0x2E2E2E, 0x0},
	{0x948BC, 0x2F2F2F, 0x0},
	{0x948C0, 0x303030, 0x0},
	{0x948C4, 0x313131, 0x0},
	{0x948C8, 0x323232, 0x0},
	{0x948CC, 0x333333, 0x0},
	{0x948D0, 0x343434, 0x0},
	{0x948D4, 0x353535, 0x0},
	{0x948D8, 0x363636, 0x0},
	{0x948DC, 0x373737, 0x0},
	{0x948E0, 0x383838, 0x0},
	{0x948E4, 0x393939, 0x0},
	{0x948E8, 0x3A3A3A, 0x0},
	{0x948EC, 0x3B3B3B, 0x0},
	{0x948F0, 0x3C3C3C, 0x0},
	{0x948F4, 0x3D3D3D, 0x0},
	{0x948F8, 0x3E3E3E, 0x0},
	{0x948FC, 0x3F3F3F, 0x0},
	{0x94900, 0x404040, 0x0},
	{0x94904, 0x414141, 0x0},
	{0x94908, 0x424242, 0x0},
	{0x9490C, 0x434343, 0x0},
	{0x94910, 0x444444, 0x0},
	{0x94914, 0x454545, 0x0},
	{0x94918, 0x464646, 0x0},
	{0x9491C, 0x474747, 0x0},
	{0x94920, 0x484848, 0x0},
	{0x94924, 0x494949, 0x0},
	{0x94928, 0x4A4A4A, 0x0},
	{0x9492C, 0x4B4B4B, 0x0},
	{0x94930, 0x4C4C4C, 0x0},
	{0x94934, 0x4D4D4D, 0x0},
	{0x94938, 0x4E4E4E, 0x0},
	{0x9493C, 0x4F4F4F, 0x0},
	{0x94940, 0x505050, 0x0},
	{0x94944, 0x515151, 0x0},
	{0x94948, 0x525252, 0x0},
	{0x9494C, 0x535353, 0x0},
	{0x94950, 0x545454, 0x0},
	{0x94954, 0x555555, 0x0},
	{0x94958, 0x565656, 0x0},
	{0x9495C, 0x575757, 0x0},
	{0x94960, 0x585858, 0x0},
	{0x94964, 0x595959, 0x0},
	{0x94968, 0x5A5A5A, 0x0},
	{0x9496C, 0x5B5B5B, 0x0},
	{0x94970, 0x5C5C5C, 0x0},
	{0x94974, 0x5D5D5D, 0x0},
	{0x94978, 0x5E5E5E, 0x0},
	{0x9497C, 0x5F5F5F, 0x0},
	{0x94980, 0x606060, 0x0},
	{0x94984, 0x616161, 0x0},
	{0x94988, 0x626262, 0x0},
	{0x9498C, 0x636363, 0x0},
	{0x94990, 0x646464, 0x0},
	{0x94994, 0x656565, 0x0},
	{0x94998, 0x666666, 0x0},
	{0x9499C, 0x676767, 0x0},
	{0x949A0, 0x686868, 0x0},
	{0x949A4, 0x696969, 0x0},
	{0x949A8, 0x6A6A6A, 0x0},
	{0x949AC, 0x6B6B6B, 0x0},
	{0x949B0, 0x6C6C6C, 0x0},
	{0x949B4, 0x6D6D6D, 0x0},
	{0x949B8, 0x6E6E6E, 0x0},
	{0x949BC, 0x6F6F6F, 0x0},
	{0x949C0, 0x707070, 0x0},
	{0x949C4, 0x717171, 0x0},
	{0x949C8, 0x727272, 0x0},
	{0x949CC, 0x737373, 0x0},
	{0x949D0, 0x747474, 0x0},
	{0x949D4, 0x757575, 0x0},
	{0x949D8, 0x767676, 0x0},
	{0x949DC, 0x777777, 0x0},
	{0x949E0, 0x787878, 0x0},
	{0x949E4, 0x797979, 0x0},
	{0x949E8, 0x7A7A7A, 0x0},
	{0x949EC, 0x7B7B7B, 0x0},
	{0x949F0, 0x7C7C7C, 0x0},
	{0x949F4, 0x7D7D7D, 0x0},
	{0x949F8, 0x7E7E7E, 0x0},
	{0x949FC, 0x7F7F7F, 0x0},
	{0x94A00, 0x808080, 0x0},
	{0x94A04, 0x818181, 0x0},
	{0x94A08, 0x828282, 0x0},
	{0x94A0C, 0x838383, 0x0},
	{0x94A10, 0x848484, 0x0},
	{0x94A14, 0x858585, 0x0},
	{0x94A18, 0x868686, 0x0},
	{0x94A1C, 0x878787, 0x0},
	{0x94A20, 0x888888, 0x0},
	{0x94A24, 0x898989, 0x0},
	{0x94A28, 0x8A8A8A, 0x0},
	{0x94A2C, 0x8B8B8B, 0x0},
	{0x94A30, 0x8C8C8C, 0x0},
	{0x94A34, 0x8D8D8D, 0x0},
	{0x94A38, 0x8E8E8E, 0x0},
	{0x94A3C, 0x8F8F8F, 0x0},
	{0x94A40, 0x909090, 0x0},
	{0x94A44, 0x919191, 0x0},
	{0x94A48, 0x929292, 0x0},
	{0x94A4C, 0x939393, 0x0},
	{0x94A50, 0x949494, 0x0},
	{0x94A54, 0x959595, 0x0},
	{0x94A58, 0x969696, 0x0},
	{0x94A5C, 0x979797, 0x0},
	{0x94A60, 0x989898, 0x0},
	{0x94A64, 0x999999, 0x0},
	{0x94A68, 0x9A9A9A, 0x0},
	{0x94A6C, 0x9B9B9B, 0x0},
	{0x94A70, 0x9C9C9C, 0x0},
	{0x94A74, 0x9D9D9D, 0x0},
	{0x94A78, 0x9E9E9E, 0x0},
	{0x94A7C, 0x9F9F9F, 0x0},
	{0x94A80, 0xA0A0A0, 0x0},
	{0x94A84, 0xA1A1A1, 0x0},
	{0x94A88, 0xA2A2A2, 0x0},
	{0x94A8C, 0xA3A3A3, 0x0},
	{0x94A90, 0xA4A4A4, 0x0},
	{0x94A94, 0xA5A5A5, 0x0},
	{0x94A98, 0xA6A6A6, 0x0},
	{0x94A9C, 0xA7A7A7, 0x0},
	{0x94AA0, 0xA8A8A8, 0x0},
	{0x94AA4, 0xA9A9A9, 0x0},
	{0x94AA8, 0xAAAAAA, 0x0},
	{0x94AAC, 0xABABAB, 0x0},
	{0x94AB0, 0xACACAC, 0x0},
	{0x94AB4, 0xADADAD, 0x0},
	{0x94AB8, 0xAEAEAE, 0x0},
	{0x94ABC, 0xAFAFAF, 0x0},
	{0x94AC0, 0xB0B0B0, 0x0},
	{0x94AC4, 0xB1B1B1, 0x0},
	{0x94AC8, 0xB2B2B2, 0x0},
	{0x94ACC, 0xB3B3B3, 0x0},
	{0x94AD0, 0xB4B4B4, 0x0},
	{0x94AD4, 0xB5B5B5, 0x0},
	{0x94AD8, 0xB6B6B6, 0x0},
	{0x94ADC, 0xB7B7B7, 0x0},
	{0x94AE0, 0xB8B8B8, 0x0},
	{0x94AE4, 0xB9B9B9, 0x0},
	{0x94AE8, 0xBABABA, 0x0},
	{0x94AEC, 0xBBBBBB, 0x0},
	{0x94AF0, 0xBCBCBC, 0x0},
	{0x94AF4, 0xBDBDBD, 0x0},
	{0x94AF8, 0xBEBEBE, 0x0},
	{0x94AFC, 0xBFBFBF, 0x0},
	{0x94B00, 0xC0C0C0, 0x0},
	{0x94B04, 0xC1C1C1, 0x0},
	{0x94B08, 0xC2C2C2, 0x0},
	{0x94B0C, 0xC3C3C3, 0x0},
	{0x94B10, 0xC4C4C4, 0x0},
	{0x94B14, 0xC5C5C5, 0x0},
	{0x94B18, 0xC6C6C6, 0x0},
	{0x94B1C, 0xC7C7C7, 0x0},
	{0x94B20, 0xC8C8C8, 0x0},
	{0x94B24, 0xC9C9C9, 0x0},
	{0x94B28, 0xCACACA, 0x0},
	{0x94B2C, 0xCBCBCB, 0x0},
	{0x94B30, 0xCCCCCC, 0x0},
	{0x94B34, 0xCDCDCD, 0x0},
	{0x94B38, 0xCECECE, 0x0},
	{0x94B3C, 0xCFCFCF, 0x0},
	{0x94B40, 0xD0D0D0, 0x0},
	{0x94B44, 0xD1D1D1, 0x0},
	{0x94B48, 0xD2D2D2, 0x0},
	{0x94B4C, 0xD3D3D3, 0x0},
	{0x94B50, 0xD4D4D4, 0x0},
	{0x94B54, 0xD5D5D5, 0x0},
	{0x94B58, 0xD6D6D6, 0x0},
	{0x94B5C, 0xD7D7D7, 0x0},
	{0x94B60, 0xD8D8D8, 0x0},
	{0x94B64, 0xD9D9D9, 0x0},
	{0x94B68, 0xDADADA, 0x0},
	{0x94B6C, 0xDBDBDB, 0x0},
	{0x94B70, 0xDCDCDC, 0x0},
	{0x94B74, 0xDDDDDD, 0x0},
	{0x94B78, 0xDEDEDE, 0x0},
	{0x94B7C, 0xDFDFDF, 0x0},
	{0x94B80, 0xE0E0E0, 0x0},
	{0x94B84, 0xE1E1E1, 0x0},
	{0x94B88, 0xE2E2E2, 0x0},
	{0x94B8C, 0xE3E3E3, 0x0},
	{0x94B90, 0xE4E4E4, 0x0},
	{0x94B94, 0xE5E5E5, 0x0},
	{0x94B98, 0xE6E6E6, 0x0},
	{0x94B9C, 0xE7E7E7, 0x0},
	{0x94BA0, 0xE8E8E8, 0x0},
	{0x94BA4, 0xE9E9E9, 0x0},
	{0x94BA8, 0xEAEAEA, 0x0},
	{0x94BAC, 0xEBEBEB, 0x0},
	{0x94BB0, 0xECECEC, 0x0},
	{0x94BB4, 0xEDEDED, 0x0},
	{0x94BB8, 0xEEEEEE, 0x0},
	{0x94BBC, 0xEFEFEF, 0x0},
	{0x94BC0, 0xF0F0F0, 0x0},
	{0x94BC4, 0xF1F1F1, 0x0},
	{0x94BC8, 0xF2F2F2, 0x0},
	{0x94BCC, 0xF3F3F3, 0x0},
	{0x94BD0, 0xF4F4F4, 0x0},
	{0x94BD4, 0xF5F5F5, 0x0},
	{0x94BD8, 0xF6F6F6, 0x0},
	{0x94BDC, 0xF7F7F7, 0x0},
	{0x94BE0, 0xF8F8F8, 0x0},
	{0x94BE4, 0xF9F9F9, 0x0},
	{0x94BE8, 0xFAFAFA, 0x0},
	{0x94BEC, 0xFBFBFB, 0x0},
	{0x94BF0, 0xFCFCFC, 0x0},
	{0x94BF4, 0xFDFDFD, 0x0},
	{0x94BF8, 0xFEFEFE, 0x0},
	{0x94BFC, 0xFFFFFF, 0x0},
	{0x90070, 0x17	  , 0x17},
};

void img_stick_wa(bool on) {
	if(on) {
		mdp_color_enhancement(mdp_img_stick_on, ARRAY_SIZE(mdp_img_stick_on));
	} else {
		mdp_color_enhancement(mdp_img_stick_off, ARRAY_SIZE(mdp_img_stick_off));
	}
	PR_DISP_INFO("%s: on(%d)\n", __func__, on);
}

#ifdef CONFIG_FB_MSM_HDMI_MHL
#define GPIO_MHL_RST_N   70
#define GPIO_MHL_INTR_N  71
static int pm8901_mpp_init(void);
static struct regulator *reg_8901_l0;
static struct regulator *reg_8058_l19;
static struct regulator *reg_8901_l1;
static struct regulator *reg_8901_usb_otg;

static uint32_t msm_hdmi_off_gpio[] = {
	GPIO_CFG(170,  0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(171,  0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(172,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
};

static uint32_t msm_hdmi_on_gpio[] = {
	GPIO_CFG(170,  1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
	GPIO_CFG(171,  1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
	GPIO_CFG(172,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
};

void hdmi_hpd_feature(int enable);

static void mhl_sii9234_1v2_power(bool enable)
{
	static bool prev_on;

	if (enable == prev_on)
		return;

	if (enable) {
		config_gpio_table(msm_hdmi_on_gpio, ARRAY_SIZE(msm_hdmi_on_gpio));
		hdmi_hpd_feature(1);
		pr_info("%s(on): success\n", __func__);
	} else {
		config_gpio_table(msm_hdmi_off_gpio, ARRAY_SIZE(msm_hdmi_off_gpio));
		hdmi_hpd_feature(0);
		pr_info("%s(off): success\n", __func__);
	}

	prev_on = enable;
}

static int mhl_sii9234_all_power(bool enable)
{
	static bool prev_on;
	int rc;

	if (enable == prev_on)
		return 0;

	if (!reg_8058_l19)
		_GET_REGULATOR(reg_8058_l19, "8058_l19");

	if (system_rev == 0) {
		if (!reg_8901_usb_otg)
			_GET_REGULATOR(reg_8901_usb_otg, "8901_usb_otg");
	} else {
		if (!reg_8901_l1)
			_GET_REGULATOR(reg_8901_l1, "8901_l1");
	}

	if (!reg_8901_l0)
		_GET_REGULATOR(reg_8901_l0, "8901_l0");

	if (enable) {
		rc = regulator_set_voltage(reg_8058_l19, 1800000, 1800000);
		if (rc) {
			pr_err("%s: regulator_set_voltage reg_8058_l19 failed rc=%d\n",
				__func__, rc);
			return rc;
		}
		if (system_rev > 0) {
			rc = regulator_set_voltage(reg_8901_l1, 3300000, 3300000);
			if (rc) {
				pr_err("%s: regulator_set_voltage reg_8901_l1 failed rc=%d\n",
					__func__, rc);
				return rc;
			}
		}

		rc = regulator_set_voltage(reg_8901_l0, 1200000, 1200000);
		if (rc) {
			pr_err("%s: regulator_set_voltage reg_8901_l0 failed rc=%d\n",
				__func__, rc);
			return rc;
		}	rc = regulator_enable(reg_8058_l19);

		if (rc) {
			pr_err("'%s' regulator enable failed, rc=%d\n",
				"reg_8058_l19", rc);
			return rc;
		}
		if (system_rev == 0) {
			rc = regulator_enable(reg_8901_usb_otg);
			if (rc) {
				pr_err("'%s' regulator enable failed, rc=%d\n",
					"reg_8901_usb_otg", rc);
				return rc;
			}
		} else {
			rc = regulator_enable(reg_8901_l1);
			if (rc) {
				pr_err("'%s' regulator enable failed, rc=%d\n",
					"reg_8901_l1", rc);
				return rc;
			}
		}

		rc = regulator_enable(reg_8901_l0);
		if (rc) {
			pr_err("'%s' regulator enable failed, rc=%d\n",
				"reg_8901_l0", rc);
			return rc;
		}
		pr_info("%s(on): success\n", __func__);
	} else {
		rc = regulator_disable(reg_8058_l19);
		if (rc)
			pr_warning("'%s' regulator disable failed, rc=%d\n",
				"reg_8058_l19", rc);
		if (system_rev == 0) {
			rc = regulator_disable(reg_8901_usb_otg);
			if (rc)
				pr_warning("'%s' regulator disable failed, rc=%d\n",
					"reg_8901_usb_otg", rc);
		} else {
			rc = regulator_disable(reg_8901_l1);
			if (rc)
				pr_warning("'%s' regulator disable failed, rc=%d\n",
					"reg_8901_l1", rc);
		}

		rc = regulator_disable(reg_8901_l0);
		if (rc)
			pr_warning("'%s' regulator disable failed, rc=%d\n",
				"reg_8901_l0", rc);
		pr_info("%s(off): success\n", __func__);
	}

	prev_on = enable;

	return 0;
}

#ifdef CONFIG_FB_MSM_HDMI_MHL_SII9234
static uint32_t mhl_gpio_table[] = {
	GPIO_CFG(GPIO_MHL_RST_N, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_MHL_INTR_N, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
};
static int mhl_sii9234_power(int on)
{
	int rc = 0;

	switch (on) {
		case 0:
			mhl_sii9234_1v2_power(false);
			break;
		case 1:
			mhl_sii9234_all_power(true);
			pm8901_mpp_init();
			config_gpio_table(mhl_gpio_table, ARRAY_SIZE(mhl_gpio_table));
			break;
		default:
			break;
	}
	return rc;
}

static T_MHL_PLATFORM_DATA mhl_sii9234_device_data = {
	.gpio_intr = GPIO_MHL_INTR_N,
	.gpio_reset = GPIO_MHL_RST_N,
	.ci2ca = 0,
	.power = mhl_sii9234_power,
	#ifdef CONFIG_FB_MSM_HDMI_MHL
	.mhl_usb_switch		= verdi_lte_usb_dpdn_switch,
	.mhl_1v2_power = mhl_sii9234_1v2_power,
	#endif
};

static struct i2c_board_info msm_i2c_gsbi10_mhl_sii9234_info[] =
{
	{
		I2C_BOARD_INFO(MHL_SII9234_I2C_NAME, 0x72 >> 1),
		.platform_data = &mhl_sii9234_device_data,
		.irq = MSM_GPIO_TO_INT(GPIO_MHL_INTR_N)
	},
};
#endif
#endif

#ifdef CONFIG_I2C_QUP

static uint32_t gsbi4_gpio_table[] = {
	GPIO_CFG(VERDI_LTE_CAM_I2C_SDA, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(VERDI_LTE_CAM_I2C_SCL, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
};

static uint32_t gsbi7_gpio_table[] = {
	GPIO_CFG(VERDI_LTE_GENERAL_I2C_SDA, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(VERDI_LTE_GENERAL_I2C_SCL, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
};

static uint32_t gsbi10_gpio_table[] = {
	GPIO_CFG(VERDI_LTE_SENSOR_I2C_SDA, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(VERDI_LTE_SENSOR_I2C_SCL, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
};

static void gsbi_qup_i2c_gpio_config(int adap_id, int config_type)
{
	printk(KERN_INFO "%s(): adap_id = %d, config_type = %d \n", __func__,adap_id,config_type);

	if ((adap_id == MSM_GSBI4_QUP_I2C_BUS_ID) && (config_type == 1)) {
		gpio_tlmm_config(gsbi4_gpio_table[0], GPIO_CFG_ENABLE);
		gpio_tlmm_config(gsbi4_gpio_table[1], GPIO_CFG_ENABLE);
	}

	if ((adap_id == MSM_GSBI4_QUP_I2C_BUS_ID) && (config_type == 0)) {
		gpio_tlmm_config(gsbi4_gpio_table[0], GPIO_CFG_DISABLE);
		gpio_tlmm_config(gsbi4_gpio_table[1], GPIO_CFG_DISABLE);
	}

	if ((adap_id == MSM_GSBI7_QUP_I2C_BUS_ID) && (config_type == 1)) {
		gpio_tlmm_config(gsbi7_gpio_table[0], GPIO_CFG_ENABLE);
		gpio_tlmm_config(gsbi7_gpio_table[1], GPIO_CFG_ENABLE);
	}

	if ((adap_id == MSM_GSBI7_QUP_I2C_BUS_ID) && (config_type == 0)) {
		gpio_tlmm_config(gsbi7_gpio_table[0], GPIO_CFG_DISABLE);
		gpio_tlmm_config(gsbi7_gpio_table[1], GPIO_CFG_DISABLE);
	}

	if ((adap_id == MSM_GSBI10_QUP_I2C_BUS_ID) && (config_type == 1)) {
		gpio_tlmm_config(gsbi10_gpio_table[0], GPIO_CFG_ENABLE);
		gpio_tlmm_config(gsbi10_gpio_table[1], GPIO_CFG_ENABLE);
	}

	if ((adap_id == MSM_GSBI10_QUP_I2C_BUS_ID) && (config_type == 0)) {
		gpio_tlmm_config(gsbi10_gpio_table[0], GPIO_CFG_DISABLE);
		gpio_tlmm_config(gsbi10_gpio_table[1], GPIO_CFG_DISABLE);
	}

}

static struct msm_i2c_platform_data msm_gsbi4_qup_i2c_pdata = {
	.clk_freq = 384000,
	.src_clk_rate = 24000000,
	.clk = "gsbi_qup_clk",
	.pclk = "gsbi_pclk",
	.msm_i2c_config_gpio = gsbi_qup_i2c_gpio_config,
};

static struct msm_i2c_platform_data msm_gsbi7_qup_i2c_pdata = {
	.clk_freq = 100000,
	.src_clk_rate = 24000000,
	.clk = "gsbi_qup_clk",
	.pclk = "gsbi_pclk",
	.msm_i2c_config_gpio = gsbi_qup_i2c_gpio_config,
#ifdef CONFIG_HTC_PUI_I2C_SAR_RECOVERY
	.chip_reset = capsense_reset,
#endif
};

static struct msm_i2c_platform_data msm_gsbi10_qup_i2c_pdata = {
	.clk_freq = 100000,
	.src_clk_rate = 24000000,
	.clk = "gsbi_qup_clk",
	.pclk = "gsbi_pclk",
	.msm_i2c_config_gpio = gsbi_qup_i2c_gpio_config,
};

#endif

#if defined(CONFIG_SPI_QUP) || defined(CONFIG_SPI_QUP_MODULE)
static struct msm_spi_platform_data msm_gsbi1_qup_spi_pdata = {
	.max_clock_speed = 10800000,
	.clk_name = "gsbi_qup_clk",
	.pclk_name = "gsbi_pclk",
};
static struct msm_spi_platform_data msm_gsbi8_qup_spi_pdata = {
	.max_clock_speed = 15060000,
	.clk_name = "gsbi_qup_clk",
	.pclk_name = "gsbi_pclk",
};
#endif

#if defined(CONFIG_I2C_SSBI) || defined(CONFIG_MSM8X60_SSBI)
/* PMIC SSBI */
static struct msm_ssbi_platform_data msm_ssbi1_pdata = {
	.controller_type = MSM_SBI_CTRL_PMIC_ARBITER,
};

/* PMIC SSBI */
static struct msm_ssbi_platform_data msm_ssbi2_pdata = {
	.controller_type = MSM_SBI_CTRL_PMIC_ARBITER,
};

/* CODEC/TSSC SSBI */
static struct msm_ssbi_platform_data msm_ssbi3_pdata = {
	.controller_type = MSM_SBI_CTRL_SSBI,
};
#endif /* CONFIG_MSM8X60_SSBI */

#ifdef CONFIG_BATTERY_MSM
/* Use basic value for fake MSM battery */
static struct msm_psy_batt_pdata msm_psy_batt_data = {
	.avail_chg_sources = AC_CHG,
};

static struct platform_device msm_batt_device = {
	.name              = "msm-battery",
	.id                = -1,
	.dev.platform_data = &msm_psy_batt_data,
};
#endif

#ifdef CONFIG_FB_MSM_LCDC_DSUB
/* VGA = 1440 x 900 x 4(bpp) x 2(pages)
   prim = 1024 x 600 x 4(bpp) x 2(pages)
   This is the difference. */
#define MSM_FB_DSUB_PMEM_ADDER (0x9E3400-0x4B0000)
#else
#define MSM_FB_DSUB_PMEM_ADDER (0)
#endif

/* Sensors DSPS platform data */
#ifdef CONFIG_MSM_DSPS
static struct dsps_gpio_info dsps_gpios[] = {
};

static void __init msm8x60_init_dsps(void)
{
	struct msm_dsps_platform_data *pdata =
		msm_dsps_device.dev.platform_data;

	pdata->gpios = dsps_gpios;
	pdata->gpios_num = ARRAY_SIZE(dsps_gpios);
}
#endif /* CONFIG_MSM_DSPS */

#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
/* prim = 1280 x 800 x 4(bpp) x 3(pages) = 0xBB8000
 * hdmi = 1920 x 1080 x 2(bpp) x 1(page)
 * Note: must be multiple of 4096 */
#define MSM_FB_SIZE roundup(0xBB8000 + 0x3F4800 + MSM_FB_DSUB_PMEM_ADDER, 4096)
#elif defined(CONFIG_FB_MSM_TVOUT)
/* prim = 1024 x 600 x 4(bpp) x 2(pages)
 * tvout = 720 x 576 x 2(bpp) x 2(pages)
 * Note: must be multiple of 4096 */
#define MSM_FB_SIZE roundup(0x7D0000 + 0x195000 + MSM_FB_DSUB_PMEM_ADDER, 4096)
#else /* CONFIG_FB_MSM_HDMI_MSM_PANEL */
#define MSM_FB_SIZE roundup(0x7D0000 + MSM_FB_DSUB_PMEM_ADDER, 4096)
#endif /* CONFIG_FB_MSM_HDMI_MSM_PANEL */
#define MSM_PMEM_SF_SIZE 0x1000000 /* 16 Mbytes */
#define MSM_PMEM_RMT_STORAGE_SIZE 0x100000 /* 1 Mbytes */

#define MSM_PMEM_KERNEL_EBI1_SIZE  0x600000
#define MSM_PMEM_ADSP_SIZE         0x4000000
#define MSM_PMEM_AUDIO_SIZE        0x239000

#define MSM_PMEM_ADSP_BASE      0x7C000000
#define MSM_PMEM_SF_BASE        0x42F00000
#define MSM_PMEM_AUDIO_BASE	(MSM_PMEM_SF_BASE + MSM_PMEM_SF_SIZE)
#define MSM_PMEM_FB_BASE	0x40400000

#define MSM_SMI_BASE          0x38000000
/* Kernel SMI PMEM Region for video core, used for Firmware */
/* and encoder,decoder scratch buffers */
/* Kernel SMI PMEM Region Should always precede the user space */
/* SMI PMEM Region, as the video core will use offset address */
/* from the Firmware base */
#define PMEM_KERNEL_SMI_BASE  (MSM_SMI_BASE)
#define PMEM_KERNEL_SMI_SIZE  0x300000
/* User space SMI PMEM Region for video core*/
/* used for encoder, decoder input & output buffers  */
#define MSM_PMEM_SMIPOOL_BASE (PMEM_KERNEL_SMI_BASE + PMEM_KERNEL_SMI_SIZE)
#define MSM_PMEM_SMIPOOL_SIZE 0x3D00000

static unsigned fb_size = MSM_FB_SIZE;
static int __init fb_size_setup(char *p)
{
	fb_size = memparse(p, NULL);
	return 0;
}
early_param("fb_size", fb_size_setup);

#ifdef CONFIG_ANDROID_PMEM
static unsigned pmem_sf_size = MSM_PMEM_SF_SIZE;
static int __init pmem_sf_size_setup(char *p)
{
	pmem_sf_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_sf_size", pmem_sf_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;

static int __init pmem_adsp_size_setup(char *p)
{
	pmem_adsp_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_adsp_size", pmem_adsp_size_setup);

static unsigned pmem_audio_size = MSM_PMEM_AUDIO_SIZE;

static int __init pmem_audio_size_setup(char *p)
{
	pmem_audio_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_audio_size", pmem_audio_size_setup);
#endif

static struct resource msm_fb_resources[] = {
	{
		.flags  = IORESOURCE_DMA,
	}
};

static int msm_fb_detect_panel(const char *name)
{
	if(panel_type == PANEL_ID_VERDI_AUO || panel_type == PANEL_ID_VERDI_AUO_RGB888) {
		if (!strcmp(name, "lcdc_auo_wxga"))
			return 0;
        } else if (panel_type == PANEL_ID_VERDI_SAMSUNG) {
		if (!strcmp(name, "lcdc_samsung_wsvga"))
			return 0;
	}
#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
	else if (!strcmp(name, "hdmi_msm"))
		return 0;
#endif /* CONFIG_FB_MSM_HDMI_MSM_PANEL */

	pr_warning("%s: not supported '%s'", __func__, name);
	return -ENODEV;
}

static struct msm_fb_platform_data msm_fb_pdata = {
	.detect_client = msm_fb_detect_panel,
	.width = 216,
	.height = 136,
};

static struct platform_device msm_fb_device = {
	.name   = "msm_fb",
	.id     = 0,
	.num_resources     = ARRAY_SIZE(msm_fb_resources),
	.resource          = msm_fb_resources,
	.dev.platform_data = &msm_fb_pdata,
};

#ifdef CONFIG_BT
static struct platform_device verdi_lte_rfkill = {
	.name = "verdi_lte_rfkill",
	.id = -1,
};

static struct htc_sleep_clk_platform_data htc_slp_clk_data = {
	.sleep_clk_pin = VERDI_LTE_WIFI_BT_SLEEP_CLK,

};

static struct platform_device wifi_bt_slp_clk = {
	.name = "htc_slp_clk",
	.id = -1,
	.dev = {
		.platform_data = &htc_slp_clk_data,
	},
};
#endif


#ifdef CONFIG_KERNEL_PMEM_SMI_REGION
static struct android_pmem_platform_data android_pmem_kernel_smi_pdata = {
	.name = PMEM_KERNEL_SMI_DATA_NAME,
	/* defaults to bitmap don't edit */
	.cached = 0,
};

static struct platform_device android_pmem_kernel_smi_device = {
	.name = "android_pmem",
	.id = 6,
	.dev = { .platform_data = &android_pmem_kernel_smi_pdata },
};
#endif

#ifdef CONFIG_ANDROID_PMEM
static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_ALLORNOTHING,
	.cached = 1,
};

static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = {.platform_data = &android_pmem_pdata},
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
};

static struct platform_device android_pmem_adsp_device = {
	.name = "android_pmem",
	.id = 2,
	.dev = { .platform_data = &android_pmem_adsp_pdata },
};

static struct android_pmem_platform_data android_pmem_audio_pdata = {
	.name = "pmem_audio",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
};

static struct platform_device android_pmem_audio_device = {
	.name = "android_pmem",
	.id = 4,
	.dev = { .platform_data = &android_pmem_audio_pdata },
};

#define PMEM_BUS_WIDTH(_bw) \
	{ \
		.vectors = &(struct msm_bus_vectors){ \
			.src = MSM_BUS_MASTER_AMPSS_M0, \
			.dst = MSM_BUS_SLAVE_SMI, \
			.ib = (_bw), \
			.ab = 0, \
		}, \
	.num_paths = 1, \
	}
static struct msm_bus_paths pmem_smi_table[] = {
	[0] = PMEM_BUS_WIDTH(0), /* Off */
	[1] = PMEM_BUS_WIDTH(1), /* On */
};

static struct msm_bus_scale_pdata smi_client_pdata = {
	.usecase = pmem_smi_table,
	.num_usecases = ARRAY_SIZE(pmem_smi_table),
	.name = "pmem_smi",
};

void pmem_request_smi_region(void *data)
{
	int bus_id = (int) data;

	msm_bus_scale_client_update_request(bus_id, 1);
}

void pmem_release_smi_region(void *data)
{
	int bus_id = (int) data;

	msm_bus_scale_client_update_request(bus_id, 0);
}

void *pmem_setup_smi_region(void)
{
	return (void *)msm_bus_scale_register_client(&smi_client_pdata);
}
static struct android_pmem_platform_data android_pmem_smipool_pdata = {
	.name = "pmem_smipool",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
	.request_region = pmem_request_smi_region,
	.release_region = pmem_release_smi_region,
	.setup_region = pmem_setup_smi_region,
	.map_on_demand = 1,
};
static struct platform_device android_pmem_smipool_device = {
	.name = "android_pmem",
	.id = 7,
	.dev = { .platform_data = &android_pmem_smipool_pdata },
};

#endif
#if 0
#define GPIO_BACKLIGHT_PWM0 0
#define GPIO_BACKLIGHT_PWM1 1

static int pmic_backlight_gpio[2]
	= { GPIO_BACKLIGHT_PWM0, GPIO_BACKLIGHT_PWM1 };
static struct msm_panel_common_pdata lcdc_samsung_panel_data = {
	.gpio_num = pmic_backlight_gpio, /* two LPG CHANNELS for backlight */
};
#endif

static int lcdc_samsung_pmic_bl(int level)
{
	if (level == 0) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_LCM_BL_PWM), 0);
		mdelay(10);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_V_LED_EN), 0);
	} else {
		mdelay(200);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_V_LED_EN), 1);
		mdelay(10);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_LCM_BL_PWM), 1);
	}
	return 0;
}

static struct msm_panel_common_pdata lcdc_samsung_panel_data = {
	.pmic_backlight = lcdc_samsung_pmic_bl,
};

static struct platform_device lcdc_samsung_panel_device = {
	.name = "lcdc_samsung_wsvga",
	.id = 0,
	.dev = {
		.platform_data = &lcdc_samsung_panel_data,
	}
};

void bkl_smooth(bool on);

#define BRI_SETTING_MIN		30
#define BRI_SETTING_DEF		143
#define BRI_SETTING_MAX		255

#define AUO_PWM_MIN			8
#define AUO_PWM_DEFAULT		158
#define AUO_PWM_MAX			255

static unsigned char lcdc_auo_shrink_pwm(int br)
{
	unsigned char shrink_br=AUO_PWM_MAX;

	if (br <= 0) {
		shrink_br = 0;
	} else if (br > 0 && br <= BRI_SETTING_MIN ) {
		shrink_br = AUO_PWM_MIN;
	} else if (br > BRI_SETTING_MIN && br <= BRI_SETTING_DEF ) {
		shrink_br = (AUO_PWM_MIN + ( br - BRI_SETTING_MIN ) *
				(AUO_PWM_DEFAULT - AUO_PWM_MIN) /
				(BRI_SETTING_DEF - BRI_SETTING_MIN));
	} else if (br > BRI_SETTING_DEF && br <= BRI_SETTING_MAX) {
		shrink_br = (AUO_PWM_DEFAULT + (br - BRI_SETTING_DEF) *
				(AUO_PWM_MAX - AUO_PWM_DEFAULT) /
				(BRI_SETTING_MAX - BRI_SETTING_DEF));
	} else if (br > BRI_SETTING_MAX)
		shrink_br = AUO_PWM_MAX;

	PR_DISP_INFO("brightness orig=%d, transformed=%d\n", br, shrink_br);
	return shrink_br;
}

static int lcdc_auo_pmic_bl(int level)
{
	struct pw8058_pwm_config pwm_conf;
	static int init=0;

	if(!init)
	{
		if(pwm_backlight == NULL)
			pwm_backlight = pwm_request(2, "PWM_BACKLIGHT");
		init=1;
	}

	if (level == 0) {
		bkl_smooth(0);
		pwm_disable(pwm_backlight);
	} else {
		if(level == AUO_PWM_MIN)
			bkl_smooth(0);
		pwm_conf.pwm_size = 9;
		pwm_conf.clk = PM_PWM_CLK_19P2MHZ;
		pwm_conf.pre_div = PM_PWM_PREDIVIDE_5;
		pwm_conf.pre_div_exp = 3;
		pwm_conf.pwm_value = level*2;
		pwm_conf.bypass_lut = 1;
		pwm_configure(pwm_backlight, &pwm_conf);
		pwm_enable(pwm_backlight);
	}

	return 0;
}

int lcdc_auo_rgb_format(void)
{
	if(panel_type==PANEL_ID_VERDI_AUO_RGB888)
		return 24;
	else if(panel_type==PANEL_ID_VERDI_AUO)
		return 18;
        else
                return 0;
}

static struct msm_panel_common_pdata mdp_pdata;

long dcr_i2c_read_1byte(long dev, long addr);
bool b5v_lc = 0;

static int bkl_enable_after_display_on(int bl_level) {
	static bool bfirst = 1;
	long lcversion = 0;

	gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_V_LED_EN), 1);
	hr_msleep(10);
	lcdc_auo_pmic_bl(lcdc_auo_shrink_pwm(bl_level));
	hr_msleep(10);
	gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_LCM_BL_EN), 1);
	if(mdp_pdata.dcr_panel_pinfo && (mdp_pdata.dcr_panel_pinfo)->auto_bkl_stat != 0)
		bkl_smooth(1);
	if(bfirst) {
		lcversion = dcr_i2c_read_1byte(0xA0, 0x7F);
		if ( lcversion == 0xc2) // 5v lc, lut_table is built-in
			b5v_lc = 1;
		else //if ( lcversion == 0xaa || lcversion == 0x22) // 4v lc, lut_table is not built-in
			b5v_lc = 0;
		bfirst = 0;
	}
	return 1;
}

static struct msm_panel_common_pdata lcdc_auo_panel_data = {
	.shrink_pwm = lcdc_auo_shrink_pwm,
	.pmic_backlight = lcdc_auo_pmic_bl,
	.rgb_format = lcdc_auo_rgb_format,
	.bkl_enable	=	bkl_enable_after_display_on,
};

static struct platform_device lcdc_auo_panel_device = {
       .name = "lcdc_auo_wxga",
       .id = 0,
       .dev = {
               .platform_data = &lcdc_auo_panel_data,
       }
};

#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
static struct resource hdmi_msm_resources[] = {
	{
		.name  = "hdmi_msm_qfprom_addr",
		.start = 0x00700000,
		.end   = 0x007060FF,
		.flags = IORESOURCE_MEM,
	},
	{
		.name  = "hdmi_msm_hdmi_addr",
		.start = 0x04A00000,
		.end   = 0x04A00FFF,
		.flags = IORESOURCE_MEM,
	},
	{
		.name  = "hdmi_msm_irq",
		.start = HDMI_IRQ,
		.end   = HDMI_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static int hdmi_enable_5v(int on);
static int hdmi_core_power(int on, int show);
static int hdmi_cec_power(int on);

static struct msm_hdmi_platform_data hdmi_msm_data = {
	.irq = HDMI_IRQ,
	.enable_5v = hdmi_enable_5v,
	.core_power = hdmi_core_power,
	.cec_power = hdmi_cec_power,
};

static struct platform_device hdmi_msm_device = {
	.name = "hdmi_msm",
	.id = 0,
	.num_resources = ARRAY_SIZE(hdmi_msm_resources),
	.resource = hdmi_msm_resources,
	.dev.platform_data = &hdmi_msm_data,
};
#endif /* CONFIG_FB_MSM_HDMI_MSM_PANEL */

static void __init msm8x60_allocate_memory_regions(void)
{
	void *addr;
	unsigned long size;

	size = MSM_FB_SIZE;
	msm_fb_resources[0].start = MSM_PMEM_FB_BASE;
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at 0x%p (0x%x physical) for fb\n",
	        size, __va(MSM_PMEM_FB_BASE), MSM_PMEM_FB_BASE);


	addr = alloc_bootmem(MSM_PMEM_RMT_STORAGE_SIZE);
	pr_info("allocating %d bytes at 0x%p (0x%lx physical) for "
		"mdm rmt_storage\n", MSM_PMEM_RMT_STORAGE_SIZE,
		addr, __pa(addr));
	rmt_storage_add_mem(__pa(addr), MSM_PMEM_RMT_STORAGE_SIZE);

#ifdef CONFIG_KERNEL_PMEM_SMI_REGION
	size = PMEM_KERNEL_SMI_SIZE;
	if (size) {
		android_pmem_kernel_smi_pdata.start = PMEM_KERNEL_SMI_BASE;
		android_pmem_kernel_smi_pdata.size = size;
		pr_info("allocating %lu bytes at 0x%p (0x%lx physical) for kernel"
			" smi pmem arena\n", size, __va(PMEM_KERNEL_SMI_BASE),
			(unsigned long) PMEM_KERNEL_SMI_BASE);
	}
#endif

#ifdef CONFIG_ANDROID_PMEM
	size = MSM_PMEM_ADSP_SIZE;
	if (size) {
		android_pmem_adsp_pdata.start = MSM_PMEM_ADSP_BASE;
		android_pmem_adsp_pdata.size = size;
		pr_info("allocating %lu bytes at 0x%p (0x%lx physical) for user"
			" adsp pmem arena\n", size, __va(MSM_PMEM_ADSP_BASE),
			(unsigned long) MSM_PMEM_ADSP_BASE);
	}

	size = MSM_PMEM_SMIPOOL_SIZE;
	if (size) {
		android_pmem_smipool_pdata.start = MSM_PMEM_SMIPOOL_BASE;
		android_pmem_smipool_pdata.size = size;
		pr_info("allocating %lu bytes at 0x%p (0x%lx physical) for user"
			" smi  pmem arena\n", size, __va(MSM_PMEM_SMIPOOL_BASE),
			(unsigned long) MSM_PMEM_SMIPOOL_BASE);
	}

	size = MSM_PMEM_AUDIO_SIZE;
	if (size) {
		android_pmem_audio_pdata.start = MSM_PMEM_AUDIO_BASE;
		android_pmem_audio_pdata.size = size;
		pr_info("allocating %lu bytes at 0x%p (0x%x physical) for audio "
			"pmem arena\n", size, __va(MSM_PMEM_AUDIO_BASE), MSM_PMEM_AUDIO_BASE);
	}

	size = MSM_PMEM_SF_SIZE;
	if (size) {
		android_pmem_pdata.start = MSM_PMEM_SF_BASE;
		android_pmem_pdata.size = size;
		pr_info("alocating %lu bytes at 0x%p (0x%lx physical) for user"
			" sf pmem arena\n", size, __va(MSM_PMEM_SF_BASE),
			(unsigned long) MSM_PMEM_SF_BASE);
	}
#endif
}

#ifdef CONFIG_SERIAL_MSM_HS
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	.rx_wakeup_irq = MSM_GPIO_TO_INT(VERDI_LTE_GPIO_BT_HOST_WAKE),
	.inject_rx_on_wakeup = 0,
	.cpu_lock_supported = 1,

	/* for bcm */
	.bt_wakeup_pin_supported = 1,
	.bt_wakeup_pin = VERDI_LTE_GPIO_BT_CHIP_WAKE,
	.host_wakeup_pin = VERDI_LTE_GPIO_BT_HOST_WAKE,
};
#endif

#if defined(CONFIG_MSM_RPM_LOG) || defined(CONFIG_MSM_RPM_LOG_MODULE)

static struct msm_rpm_log_platform_data msm_rpm_log_pdata = {
	.phys_addr_base = 0x00106000,
	.reg_offsets = {
		[MSM_RPM_LOG_PAGE_INDICES] = 0x00000C80,
		[MSM_RPM_LOG_PAGE_BUFFER]  = 0x00000CA0,
	},
	.phys_size = SZ_8K,
	.log_len = 4096,		  /* log's buffer length in bytes */
	.log_len_mask = (4096 >> 2) - 1,  /* length mask in units of u32 */
};

static struct platform_device msm_rpm_log_device = {
	.name	= "msm_rpm_log",
	.id	= -1,
	.dev	= {
		.platform_data = &msm_rpm_log_pdata,
	},
};
#endif

#ifdef CONFIG_BATTERY_MSM8X60
static struct msm_charger_platform_data msm_charger_data = {
	.safety_time = 180,
	.update_time = 1,
	.max_voltage = 4200,
	.min_voltage = 3200,
	.resume_voltage = 4100,
};

static struct platform_device msm_charger_device = {
	.name = "msm-charger",
	.id = -1,
	.dev = {
		.platform_data = &msm_charger_data,
	}
};
#endif

static struct regulator_consumer_supply rpm_vreg_supply[RPM_VREG_ID_MAX] = {
	[RPM_VREG_ID_PM8058_L0]  = REGULATOR_SUPPLY("8058_l0", NULL),
	[RPM_VREG_ID_PM8058_L1]  = REGULATOR_SUPPLY("8058_l1", NULL),
	[RPM_VREG_ID_PM8058_L2]  = REGULATOR_SUPPLY("8058_l2", NULL),
	[RPM_VREG_ID_PM8058_L3]  = REGULATOR_SUPPLY("8058_l3", NULL),
	[RPM_VREG_ID_PM8058_L4]  = REGULATOR_SUPPLY("8058_l4", NULL),
	[RPM_VREG_ID_PM8058_L5]  = REGULATOR_SUPPLY("8058_l5", NULL),
	[RPM_VREG_ID_PM8058_L6]  = REGULATOR_SUPPLY("8058_l6", NULL),
	[RPM_VREG_ID_PM8058_L7]  = REGULATOR_SUPPLY("8058_l7", NULL),
	[RPM_VREG_ID_PM8058_L8]  = REGULATOR_SUPPLY("8058_l8", NULL),
	[RPM_VREG_ID_PM8058_L9]  = REGULATOR_SUPPLY("8058_l9", NULL),
	[RPM_VREG_ID_PM8058_L10] = REGULATOR_SUPPLY("8058_l10", NULL),
	[RPM_VREG_ID_PM8058_L11] = REGULATOR_SUPPLY("8058_l11", NULL),
	[RPM_VREG_ID_PM8058_L12] = REGULATOR_SUPPLY("8058_l12", NULL),
	[RPM_VREG_ID_PM8058_L13] = REGULATOR_SUPPLY("8058_l13", NULL),
	[RPM_VREG_ID_PM8058_L14] = REGULATOR_SUPPLY("8058_l14", NULL),
	[RPM_VREG_ID_PM8058_L15] = REGULATOR_SUPPLY("8058_l15", NULL),
	[RPM_VREG_ID_PM8058_L16] = REGULATOR_SUPPLY("8058_l16", NULL),
	[RPM_VREG_ID_PM8058_L17] = REGULATOR_SUPPLY("8058_l17", NULL),
	[RPM_VREG_ID_PM8058_L18] = REGULATOR_SUPPLY("8058_l18", NULL),
	[RPM_VREG_ID_PM8058_L19] = REGULATOR_SUPPLY("8058_l19", NULL),
	[RPM_VREG_ID_PM8058_L20] = REGULATOR_SUPPLY("8058_l20", NULL),
	[RPM_VREG_ID_PM8058_L21] = REGULATOR_SUPPLY("8058_l21", NULL),
	[RPM_VREG_ID_PM8058_L22] = REGULATOR_SUPPLY("8058_l22", NULL),
	[RPM_VREG_ID_PM8058_L23] = REGULATOR_SUPPLY("8058_l23", NULL),
	[RPM_VREG_ID_PM8058_L24] = REGULATOR_SUPPLY("8058_l24", NULL),
	[RPM_VREG_ID_PM8058_L25] = REGULATOR_SUPPLY("8058_l25", NULL),

	[RPM_VREG_ID_PM8058_S0] = REGULATOR_SUPPLY("8058_s0", NULL),
	[RPM_VREG_ID_PM8058_S1] = REGULATOR_SUPPLY("8058_s1", NULL),
	[RPM_VREG_ID_PM8058_S2] = REGULATOR_SUPPLY("8058_s2", NULL),
	[RPM_VREG_ID_PM8058_S3] = REGULATOR_SUPPLY("8058_s3", NULL),
	[RPM_VREG_ID_PM8058_S4] = REGULATOR_SUPPLY("8058_s4", NULL),

	[RPM_VREG_ID_PM8058_LVS0] = REGULATOR_SUPPLY("8058_lvs0", NULL),
	[RPM_VREG_ID_PM8058_LVS1] = REGULATOR_SUPPLY("8058_lvs1", NULL),

	[RPM_VREG_ID_PM8058_NCP] = REGULATOR_SUPPLY("8058_ncp", NULL),

	[RPM_VREG_ID_PM8901_L0]  = REGULATOR_SUPPLY("8901_l0",  NULL),
	[RPM_VREG_ID_PM8901_L1]  = REGULATOR_SUPPLY("8901_l1",  NULL),
	[RPM_VREG_ID_PM8901_L2]  = REGULATOR_SUPPLY("8901_l2",  NULL),
	[RPM_VREG_ID_PM8901_L3]  = REGULATOR_SUPPLY("8901_l3",  NULL),
	[RPM_VREG_ID_PM8901_L4]  = REGULATOR_SUPPLY("8901_l4",  NULL),
	[RPM_VREG_ID_PM8901_L5]  = REGULATOR_SUPPLY("8901_l5",  NULL),
	[RPM_VREG_ID_PM8901_L6]  = REGULATOR_SUPPLY("8901_l6",  NULL),

	[RPM_VREG_ID_PM8901_S2] = REGULATOR_SUPPLY("8901_s2", NULL),
	[RPM_VREG_ID_PM8901_S3] = REGULATOR_SUPPLY("8901_s3", NULL),
	[RPM_VREG_ID_PM8901_S4] = REGULATOR_SUPPLY("8901_s4", NULL),

	[RPM_VREG_ID_PM8901_LVS0] = REGULATOR_SUPPLY("8901_lvs0", NULL),
	[RPM_VREG_ID_PM8901_LVS1] = REGULATOR_SUPPLY("8901_lvs1", NULL),
	[RPM_VREG_ID_PM8901_LVS2] = REGULATOR_SUPPLY("8901_lvs2", NULL),
	[RPM_VREG_ID_PM8901_LVS3] = REGULATOR_SUPPLY("8901_lvs3", NULL),
	[RPM_VREG_ID_PM8901_MVS0] = REGULATOR_SUPPLY("8901_mvs0", NULL),
};

#ifdef CONFIG_MSM_GSBI9_UART
static struct msm_serial_hslite_platform_data msm_uart_gsbi9_pdata = {
	.config_gpio	= 1,
	.uart_tx_gpio	= 67,
	.uart_rx_gpio	= 66,
};
#endif

#define RPM_VREG_INIT(_id, _min_uV, _max_uV, _modes, _ops, _apply_uV, \
		      _default_uV, _peak_uA, _avg_uA, _pull_down, _pin_ctrl, \
		      _freq, _pin_fn, _rpm_mode, _state, _sleep_selectable, \
		      _always_on) \
	[RPM_VREG_ID_##_id] = { \
		.init_data = { \
			.constraints = { \
				.valid_modes_mask = _modes, \
				.valid_ops_mask = _ops, \
				.min_uV = _min_uV, \
				.max_uV = _max_uV, \
				.input_uV = _min_uV, \
				.apply_uV = _apply_uV, \
				.always_on = _always_on, \
			}, \
			.num_consumer_supplies = 1, \
			.consumer_supplies = \
				&rpm_vreg_supply[RPM_VREG_ID_##_id], \
		}, \
		.default_uV = _default_uV, \
		.peak_uA = _peak_uA, \
		.avg_uA = _avg_uA, \
		.pull_down_enable = _pull_down, \
		.pin_ctrl = _pin_ctrl, \
		.freq = _freq, \
		.pin_fn = _pin_fn, \
		.mode = _rpm_mode, \
		.state = _state, \
		.sleep_selectable = _sleep_selectable, \
	}

/*
 * The default LPM/HPM state of an RPM controlled regulator can be controlled
 * via the peak_uA value specified in the table below.  If the value is less
 * than the high power min threshold for the regulator, then the regulator will
 * be set to LPM.  Otherwise, it will be set to HPM.
 *
 * This value can be further overridden by specifying an initial mode via
 * .init_data.constraints.initial_mode.
 */

#define RPM_VREG_INIT_LDO(_id, _always_on, _pd, _sleep_selectable, _min_uV, _max_uV, _init_peak_uA, _pin_ctrl) \
	RPM_VREG_INIT(_id, _min_uV, _max_uV, REGULATOR_MODE_FAST | \
		      REGULATOR_MODE_NORMAL | REGULATOR_MODE_IDLE | \
		      REGULATOR_MODE_STANDBY, REGULATOR_CHANGE_VOLTAGE | \
		      REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE | \
		      REGULATOR_CHANGE_DRMS, 0, _min_uV, _init_peak_uA, \
		      _init_peak_uA, _pd, _pin_ctrl, RPM_VREG_FREQ_NONE, \
		      RPM_VREG_PIN_FN_ENABLE, RPM_VREG_MODE_NONE, \
		      RPM_VREG_STATE_OFF, _sleep_selectable, _always_on)

#define RPM_VREG_INIT_LDO_PF(_id, _always_on, _pd, _sleep_selectable, _min_uV, _max_uV, _init_peak_uA, _pin_ctrl, _pin_fn) \
	RPM_VREG_INIT(_id, _min_uV, _max_uV, REGULATOR_MODE_FAST | \
			REGULATOR_MODE_NORMAL | REGULATOR_MODE_IDLE | \
			REGULATOR_MODE_STANDBY, REGULATOR_CHANGE_VOLTAGE | \
			REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE | \
			REGULATOR_CHANGE_DRMS, 0, _min_uV, _init_peak_uA, \
			_init_peak_uA, _pd, _pin_ctrl, RPM_VREG_FREQ_NONE, \
			_pin_fn, RPM_VREG_MODE_NONE, RPM_VREG_STATE_OFF, \
			_sleep_selectable, _always_on)

#define RPM_VREG_INIT_SMPS(_id, _always_on, _pd, _sleep_selectable, _min_uV, _max_uV, _init_peak_uA, _pin_ctrl, _freq) \
	RPM_VREG_INIT(_id, _min_uV, _max_uV, REGULATOR_MODE_FAST | \
		      REGULATOR_MODE_NORMAL | REGULATOR_MODE_IDLE | \
		      REGULATOR_MODE_STANDBY, REGULATOR_CHANGE_VOLTAGE | \
		      REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE | \
		      REGULATOR_CHANGE_DRMS, 0, _min_uV, _init_peak_uA, \
		      _init_peak_uA, _pd, _pin_ctrl, _freq, \
		      RPM_VREG_PIN_FN_ENABLE, RPM_VREG_MODE_NONE, \
		      RPM_VREG_STATE_OFF, _sleep_selectable, _always_on)

#define RPM_VREG_INIT_VS(_id, _always_on, _pd, _sleep_selectable, _pin_ctrl) \
	RPM_VREG_INIT(_id, 0, 0, REGULATOR_MODE_NORMAL | REGULATOR_MODE_IDLE, \
		      REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE, 0, 0, \
		      1000, 1000, _pd, _pin_ctrl, RPM_VREG_FREQ_NONE, \
		      RPM_VREG_PIN_FN_ENABLE, RPM_VREG_MODE_NONE, \
		      RPM_VREG_STATE_OFF, _sleep_selectable, _always_on)

#define RPM_VREG_INIT_NCP(_id, _always_on, _pd, _sleep_selectable, _min_uV, \
			  _max_uV, _pin_ctrl) \
	RPM_VREG_INIT(_id, _min_uV, _max_uV, REGULATOR_MODE_NORMAL, \
		      REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS, 0, \
		      _min_uV, 1000, 1000, _pd, _pin_ctrl, RPM_VREG_FREQ_NONE, \
		      RPM_VREG_PIN_FN_ENABLE, RPM_VREG_MODE_NONE, \
		      RPM_VREG_STATE_OFF, _sleep_selectable, _always_on)

#define LDO50HMIN	RPM_VREG_LDO_50_HPM_MIN_LOAD
#define LDO150HMIN	RPM_VREG_LDO_150_HPM_MIN_LOAD
#define LDO300HMIN	RPM_VREG_LDO_300_HPM_MIN_LOAD
#define SMPS_HMIN	RPM_VREG_SMPS_HPM_MIN_LOAD
#define FTS_HMIN	RPM_VREG_FTSMPS_HPM_MIN_LOAD

static struct rpm_vreg_pdata rpm_vreg_init_pdata[RPM_VREG_ID_MAX] = {
	RPM_VREG_INIT_LDO(PM8058_L0,  0, 1, 0, 1200000, 1200000, LDO150HMIN, 0 ),
	RPM_VREG_INIT_LDO(PM8058_L1,  0, 1, 0, 1350000, 1350000, LDO300HMIN, 0 ),
	RPM_VREG_INIT_LDO(PM8058_L2,  0, 1, 0, 1800000, 2600000, LDO300HMIN, 0 ),
	RPM_VREG_INIT_LDO(PM8058_L3,  0, 1, 0, 1800000, 3000000, LDO150HMIN, 0 ), /* RUIM, 1G:3.0v, 3G:1.8v */
	RPM_VREG_INIT_LDO(PM8058_L4,  0, 1, 0, 2850000, 2850000, LDO50HMIN, 0 ),
	RPM_VREG_INIT_LDO(PM8058_L5,  0, 1, 0, 2850000, 2850000, LDO300HMIN, 0 ),
	RPM_VREG_INIT_LDO(PM8058_L6,  0, 1, 0, 3000000, 3600000, LDO50HMIN, 0 ),
	RPM_VREG_INIT_LDO(PM8058_L7,  0, 1, 0, 1800000, 1800000, LDO50HMIN, 0 ),
	RPM_VREG_INIT_LDO(PM8058_L8,  0, 1, 0, 1800000, 1800000, LDO300HMIN, 0),
	RPM_VREG_INIT_LDO(PM8058_L9,  0, 1, 0, 1800000, 1800000, LDO300HMIN, 0 ),
	RPM_VREG_INIT_LDO(PM8058_L10, 0, 1, 0, 1200000, 1200000, LDO300HMIN, 0 ),
	RPM_VREG_INIT_LDO(PM8058_L11, 0, 1, 0, 2850000, 2850000, LDO150HMIN, 0 ),
	RPM_VREG_INIT_LDO(PM8058_L12, 0, 1, 0, 2850000, 2850000, LDO150HMIN, 0 ),
	RPM_VREG_INIT_LDO(PM8058_L13, 0, 1, 0, 2050000, 2050000, LDO300HMIN, 0 ),
	RPM_VREG_INIT_LDO(PM8058_L14, 0, 1, 0, 2850000, 2850000, LDO300HMIN, 0),
	RPM_VREG_INIT_LDO(PM8058_L15, 0, 1, 0, 2800000, 2800000, LDO300HMIN, 0 ),
	RPM_VREG_INIT_LDO(PM8058_L16, 1, 1, 1, 1800000, 1800000, LDO300HMIN, 0 ),
	RPM_VREG_INIT_LDO(PM8058_L17, 0, 1, 0, 2600000, 2600000, LDO150HMIN, 0 ),
	RPM_VREG_INIT_LDO(PM8058_L18, 0, 1, 1, 2200000, 2200000, LDO150HMIN, 0 ),
	RPM_VREG_INIT_LDO(PM8058_L19, 0, 1, 0, 1800000, 1800000, LDO150HMIN, 0 ),
	RPM_VREG_INIT_LDO(PM8058_L20, 0, 1, 0, 1800000, 1800000, LDO150HMIN, 0 ),
	RPM_VREG_INIT_LDO(PM8058_L21, 1, 1, 0, 1200000, 1200000, LDO150HMIN, 0 ),
	RPM_VREG_INIT_LDO(PM8058_L22, 0, 1, 0, 1200000, 1200000, LDO300HMIN, 0 ), /* On till SB3 on than off */
	RPM_VREG_INIT_LDO(PM8058_L23, 0, 1, 0, 1800000, 1800000, LDO300HMIN, 0 ), /* N/A */
	RPM_VREG_INIT_LDO(PM8058_L24, 0, 1, 0, 1200000, 1200000, LDO150HMIN, 0 ), /* N/A */
	RPM_VREG_INIT_LDO(PM8058_L25, 0, 1, 0, 1200000, 1200000, LDO150HMIN, 0 ), /* N/A */

	RPM_VREG_INIT_SMPS(PM8058_S0, 0, 1, 1,  500000, 1400000, SMPS_HMIN, 0,
		RPM_VREG_FREQ_1p92),
	RPM_VREG_INIT_SMPS(PM8058_S1, 0, 1, 1,  500000, 1400000, SMPS_HMIN, 0,
		RPM_VREG_FREQ_1p92),
	RPM_VREG_INIT_SMPS(PM8058_S2, 0, 1, 0, 1200000, 1400000, SMPS_HMIN,
		RPM_VREG_PIN_CTRL_A0, RPM_VREG_FREQ_1p92),
	RPM_VREG_INIT_SMPS(PM8058_S3, 1, 1, 0, 1800000, 1800000, SMPS_HMIN, 0,
		RPM_VREG_FREQ_1p92),
	RPM_VREG_INIT_SMPS(PM8058_S4, 1, 1, 0, 2200000, 2200000, SMPS_HMIN, 0,
		RPM_VREG_FREQ_1p92),

	RPM_VREG_INIT_VS(PM8058_LVS0, 0, 1,	0,		  0),
	RPM_VREG_INIT_VS(PM8058_LVS1, 0, 1, 0,			  0),

	RPM_VREG_INIT_NCP(PM8058_NCP, 0, 1, 0, 1800000, 1800000, 0),

	RPM_VREG_INIT_LDO(PM8901_L0,  0, 1, 0, 1200000, 1200000, LDO300HMIN, 0 ),
	RPM_VREG_INIT_LDO(PM8901_L1,  0, 1, 0, 3300000, 3300000, LDO300HMIN, 0 ),
	RPM_VREG_INIT_LDO(PM8901_L2,  0, 1, 0, 2850000, 3300000, LDO300HMIN, 0 ),
	RPM_VREG_INIT_LDO(PM8901_L3,  0, 1, 0, 3300000, 3300000, LDO300HMIN, 0),
	RPM_VREG_INIT_LDO(PM8901_L4,  0, 1, 0, 2850000, 2850000, LDO300HMIN, 0 ),
	RPM_VREG_INIT_LDO(PM8901_L5,  0, 1, 0, 2850000, 2850000, LDO300HMIN, 0),
	RPM_VREG_INIT_LDO(PM8901_L6,  0, 1, 0, 2800000, 2800000, LDO300HMIN, 0 ),

	RPM_VREG_INIT_SMPS(PM8901_S2, 0, 1, 0, 1200000, 1200000, FTS_HMIN, 0,
		RPM_VREG_FREQ_1p60),
	RPM_VREG_INIT_SMPS(PM8901_S3, 0, 1, 0, 1100000, 1100000, FTS_HMIN, 0,
		RPM_VREG_FREQ_1p60),
	RPM_VREG_INIT_SMPS(PM8901_S4, 0, 1, 0, 1200000, 1200000, FTS_HMIN,
		RPM_VREG_PIN_CTRL_A0, RPM_VREG_FREQ_1p60),

	RPM_VREG_INIT_VS(PM8901_LVS0, 0, 1, 0,		     0),
	RPM_VREG_INIT_VS(PM8901_LVS1, 0, 1, 0,		     0),
	RPM_VREG_INIT_VS(PM8901_LVS2, 0, 1, 0,		     0),
	RPM_VREG_INIT_VS(PM8901_LVS3, 0, 1, 0,		     0),
	RPM_VREG_INIT_VS(PM8901_MVS0, 0, 1, 0,		     0),
};

#define RPM_VREG(_id) \
	[_id] = { \
		.name = "rpm-regulator", \
		.id = _id, \
		.dev = { \
			.platform_data = &rpm_vreg_init_pdata[_id], \
		}, \
	}

static struct platform_device rpm_vreg_device[RPM_VREG_ID_MAX] = {
	RPM_VREG(RPM_VREG_ID_PM8058_L0),
	RPM_VREG(RPM_VREG_ID_PM8058_L1),
	RPM_VREG(RPM_VREG_ID_PM8058_L2),
	RPM_VREG(RPM_VREG_ID_PM8058_L3),
	RPM_VREG(RPM_VREG_ID_PM8058_L4),
	RPM_VREG(RPM_VREG_ID_PM8058_L5),
	RPM_VREG(RPM_VREG_ID_PM8058_L6),
	RPM_VREG(RPM_VREG_ID_PM8058_L7),
	RPM_VREG(RPM_VREG_ID_PM8058_L8),
	RPM_VREG(RPM_VREG_ID_PM8058_L9),
	RPM_VREG(RPM_VREG_ID_PM8058_L10),
	RPM_VREG(RPM_VREG_ID_PM8058_L11),
	RPM_VREG(RPM_VREG_ID_PM8058_L12),
	RPM_VREG(RPM_VREG_ID_PM8058_L13),
	RPM_VREG(RPM_VREG_ID_PM8058_L14),
	RPM_VREG(RPM_VREG_ID_PM8058_L15),
	RPM_VREG(RPM_VREG_ID_PM8058_L16),
	RPM_VREG(RPM_VREG_ID_PM8058_L17),
	RPM_VREG(RPM_VREG_ID_PM8058_L18),
	RPM_VREG(RPM_VREG_ID_PM8058_L19),
	RPM_VREG(RPM_VREG_ID_PM8058_L20),
	RPM_VREG(RPM_VREG_ID_PM8058_L21),
	RPM_VREG(RPM_VREG_ID_PM8058_L22),
	RPM_VREG(RPM_VREG_ID_PM8058_L23),
	RPM_VREG(RPM_VREG_ID_PM8058_L24),
	RPM_VREG(RPM_VREG_ID_PM8058_L25),
	RPM_VREG(RPM_VREG_ID_PM8058_S0),
	RPM_VREG(RPM_VREG_ID_PM8058_S1),
	RPM_VREG(RPM_VREG_ID_PM8058_S2),
	RPM_VREG(RPM_VREG_ID_PM8058_S3),
	RPM_VREG(RPM_VREG_ID_PM8058_S4),
	RPM_VREG(RPM_VREG_ID_PM8058_LVS0),
	RPM_VREG(RPM_VREG_ID_PM8058_LVS1),
	RPM_VREG(RPM_VREG_ID_PM8058_NCP),
	RPM_VREG(RPM_VREG_ID_PM8901_L0),
	RPM_VREG(RPM_VREG_ID_PM8901_L1),
	RPM_VREG(RPM_VREG_ID_PM8901_L2),
	RPM_VREG(RPM_VREG_ID_PM8901_L3),
	RPM_VREG(RPM_VREG_ID_PM8901_L4),
	RPM_VREG(RPM_VREG_ID_PM8901_L5),
	RPM_VREG(RPM_VREG_ID_PM8901_L6),
	RPM_VREG(RPM_VREG_ID_PM8901_S2),
	RPM_VREG(RPM_VREG_ID_PM8901_S3),
	RPM_VREG(RPM_VREG_ID_PM8901_S4),
	RPM_VREG(RPM_VREG_ID_PM8901_LVS0),
	RPM_VREG(RPM_VREG_ID_PM8901_LVS1),
	RPM_VREG(RPM_VREG_ID_PM8901_LVS2),
	RPM_VREG(RPM_VREG_ID_PM8901_LVS3),
	RPM_VREG(RPM_VREG_ID_PM8901_MVS0),
};

static struct platform_device *early_regulators[] __initdata = {
	&msm_device_saw_s0,
	&msm_device_saw_s1,
#ifdef CONFIG_PMIC8058
	&rpm_vreg_device[RPM_VREG_ID_PM8058_S0],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_S1],
#endif
	&lightsensor_pdev
};

static struct platform_device *early_devices[] __initdata = {
#ifdef CONFIG_MSM_BUS_SCALING
	&msm_bus_apps_fabric,
	&msm_bus_sys_fabric,
	&msm_bus_mm_fabric,
	&msm_bus_sys_fpb,
	&msm_bus_cpss_fpb,
#endif
#ifdef CONFIG_ARCH_MSM_FLASHLIGHT
	&flashlight_device,
#endif
	&verdi_lte_timed_gpios,
	&msm_device_dmov_adm0,
	&msm_device_dmov_adm1,
};

static struct resource msm_aux_pcm_resources[] = {

	{
		.name   = "aux_pcm_dout",
		.start  = 111,
		.end    = 111,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_din",
		.start  = 112,
		.end    = 112,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_syncout",
		.start  = 113,
		.end    = 113,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_clkin_a",
		.start  = 114,
		.end    = 114,
		.flags  = IORESOURCE_IO,
	},
};

static struct platform_device msm_aux_pcm_device = {
	.name   = "msm_aux_pcm",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_aux_pcm_resources),
	.resource       = msm_aux_pcm_resources,
};

#if 0
static uint32_t mi2s_config_gpio[] = {
	GPIO_CFG(107, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(101, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(102, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(103, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static void fm_mi2s_enable(void)
{
	gpio_tlmm_config(mi2s_config_gpio[0], GPIO_CFG_ENABLE);
	gpio_tlmm_config(mi2s_config_gpio[1], GPIO_CFG_ENABLE);
	gpio_tlmm_config(mi2s_config_gpio[2], GPIO_CFG_ENABLE);
	gpio_tlmm_config(mi2s_config_gpio[3], GPIO_CFG_ENABLE);
}

static void fm_mi2s_disable(void)
{
	gpio_tlmm_config(mi2s_config_gpio[0], GPIO_CFG_DISABLE);
	gpio_tlmm_config(mi2s_config_gpio[1], GPIO_CFG_DISABLE);
	gpio_tlmm_config(mi2s_config_gpio[2], GPIO_CFG_DISABLE);
	gpio_tlmm_config(mi2s_config_gpio[3], GPIO_CFG_DISABLE);
}

static struct resource msm_mi2s_gpio_resources[] = {

	{
		.name   = "mi2s_ws",
		.start  = 101,
		.end    = 101,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "mi2s_sclk",
		.start  = 102,
		.end    = 102,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "mi2s_mclk",
		.start  = 103,
		.end    = 103,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "fm_i2s_sd",
		.start  = 107,
		.end    = 107,
		.flags  = IORESOURCE_IO,
	},
};

static struct msm_mi2s_gpio_data gpio_data = {

	.enable		 = fm_mi2s_enable,
	.disable	 = fm_mi2s_disable,
};

static struct platform_device msm_mi2s_device = {
	.name		= "msm_mi2s",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(msm_mi2s_gpio_resources),
	.resource	= msm_mi2s_gpio_resources,
	.dev		= { .platform_data = &gpio_data },
};
#endif

#ifdef CONFIG_SENSORS_M_ADC
static struct adc_access_fn xoadc_fn = {
	pm8058_xoadc_select_chan_and_start_conv,
	pm8058_xoadc_read_adc_code,
	pm8058_xoadc_get_properties,
	pm8058_xoadc_slot_request,
	pm8058_xoadc_restore_slot,
	pm8058_xoadc_calibrate,
};

static struct msm_adc_channels msm_adc_channels_data[] = {
	{"vbatt", CHANNEL_ADC_VBATT, 0, &xoadc_fn, CHAN_PATH_TYPE2,
		ADC_CONFIG_TYPE2, ADC_CALIB_CONFIG_TYPE3, scale_default},
	{"vcoin", CHANNEL_ADC_VCOIN, 0, &xoadc_fn, CHAN_PATH_TYPE1,
		ADC_CONFIG_TYPE2, ADC_CALIB_CONFIG_TYPE2, scale_default},
	{"vcharger_channel", CHANNEL_ADC_VCHG, 0, &xoadc_fn, CHAN_PATH_TYPE13,
		ADC_CONFIG_TYPE2, ADC_CALIB_CONFIG_TYPE4, scale_default},
	{"charger_current_monitor", CHANNEL_ADC_CHG_MONITOR, 0, &xoadc_fn,
		CHAN_PATH_TYPE4,
		ADC_CONFIG_TYPE2, ADC_CALIB_CONFIG_TYPE1, scale_default},
	{"vph_pwr", CHANNEL_ADC_VPH_PWR, 0, &xoadc_fn, CHAN_PATH_TYPE5,
		ADC_CONFIG_TYPE2, ADC_CALIB_CONFIG_TYPE3, scale_default},
	{"usb_vbus", CHANNEL_ADC_USB_VBUS, 0, &xoadc_fn, CHAN_PATH_TYPE11,
		ADC_CONFIG_TYPE2, ADC_CALIB_CONFIG_TYPE3, scale_default},
	{"pmic_therm", CHANNEL_ADC_DIE_TEMP, 0, &xoadc_fn, CHAN_PATH_TYPE12,
		ADC_CONFIG_TYPE2, ADC_CALIB_CONFIG_TYPE1, scale_pmic_therm},
	{"pmic_therm_4K", CHANNEL_ADC_DIE_TEMP_4K, 0, &xoadc_fn,
		CHAN_PATH_TYPE12,
		ADC_CONFIG_TYPE1, ADC_CALIB_CONFIG_TYPE7, scale_pmic_therm},
	{"xo_therm", CHANNEL_ADC_XOTHERM, 0, &xoadc_fn, CHAN_PATH_TYPE_NONE,
		ADC_CONFIG_TYPE2, ADC_CALIB_CONFIG_TYPE5, tdkntcgtherm},
	{"xo_therm_4K", CHANNEL_ADC_XOTHERM_4K, 0, &xoadc_fn,
		CHAN_PATH_TYPE_NONE,
		ADC_CONFIG_TYPE1, ADC_CALIB_CONFIG_TYPE6, tdkntcgtherm},
	{"hdset_detect", CHANNEL_ADC_HDSET, 0, &xoadc_fn, CHAN_PATH_TYPE6,
		ADC_CONFIG_TYPE2, ADC_CALIB_CONFIG_TYPE1, scale_default},
	{"chg_batt_amon", CHANNEL_ADC_BATT_AMON, 0, &xoadc_fn, CHAN_PATH_TYPE7,
		ADC_CONFIG_TYPE2, ADC_CALIB_CONFIG_TYPE1,
		scale_default},
	{"batt_therm", CHANNEL_ADC_BATT_THERM, 0, &xoadc_fn, CHAN_PATH_TYPE6,
		ADC_CONFIG_TYPE2, ADC_CALIB_CONFIG_TYPE2, scale_default},
	{"batt_id", CHANNEL_ADC_BATT_ID, 0, &xoadc_fn, CHAN_PATH_TYPE9,
		ADC_CONFIG_TYPE2, ADC_CALIB_CONFIG_TYPE2, scale_default},
};

static struct msm_adc_platform_data msm_adc_pdata = {
	.channel = msm_adc_channels_data,
	.num_chan_supported = ARRAY_SIZE(msm_adc_channels_data),
};

static struct platform_device msm_adc_device = {
	.name   = "msm_adc",
	.id = -1,
	.dev = {
		.platform_data = &msm_adc_pdata,
	},
};
#endif

static uint32_t headset_h2w_gpio_table[] = {
	GPIO_CFG(VERDI_H2W_IO1_CLK, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL,
		 GPIO_CFG_2MA),
	GPIO_CFG(VERDI_H2W_IO2_DAT, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL,
		 GPIO_CFG_2MA),
#if 0 /* Same GPIO as VERDI_LTE_GPIO_CABLE_IN1_XD */
	GPIO_CFG(VERDI_H2W_CABLE_IN1, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL,
		 GPIO_CFG_2MA),
#endif
	GPIO_CFG(VERDI_H2W_CABLE_IN2, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL,
		 GPIO_CFG_2MA),
};

/* HTC_HEADSET_GPIO Driver */
static struct htc_headset_gpio_platform_data htc_headset_gpio_data = {
	.hpin_gpio		= VERDI_LTE_GPIO_AUD_HP_DET,
	.key_enable_gpio	= 0,
	.mic_select_gpio	= 0,
};

static struct platform_device htc_headset_gpio = {
	.name	= "HTC_HEADSET_GPIO",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_gpio_data,
	},
};

/* HTC_HEADSET_PMIC Driver */
static struct htc_headset_pmic_platform_data htc_headset_pmic_data = {
	.driver_flag		= 0,
	.hpin_gpio		= 0,
	.hpin_irq		= 0,
	.key_gpio		= 0,
	.key_irq		= 0,
	.key_enable_gpio	= 0,
	.adc_mic_bias		= {0, 0},
	.adc_remote		= {0, 0, 0, 0, 0, 0},
};

static struct platform_device htc_headset_pmic = {
	.name	= "HTC_HEADSET_PMIC",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_pmic_data,
	},
};

/* HTC_HEADSET_8X60 Driver */
static struct htc_headset_8x60_platform_data htc_headset_8x60_data = {
	.adc_mpp	= XOADC_MPP_10,
	.adc_amux	= PM_MPP_AIN_AMUX_CH5,
	.adc_mic_bias	= {HS_DEF_MIC_ADC_15_BIT_MIN,
			   HS_DEF_MIC_ADC_15_BIT_MAX},
	.adc_remote	= {0, 1251, 1430, 3411, 4543, 6807},
};

static struct htc_headset_8x60_platform_data htc_headset_8x60_data_xb = {
	.adc_mpp	= XOADC_MPP_10,
	.adc_amux	= PM_MPP_AIN_AMUX_CH5,
	.adc_mic_bias	= {14375, 26643},
	.adc_remote	= {0, 1219, 1440, 3862, 4231, 6783},
};

static struct platform_device htc_headset_8x60 = {
	.name	= "HTC_HEADSET_8X60",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_8x60_data,
	},
};

/* HTC_HEADSET_MISC Driver */
static struct htc_headset_misc_platform_data htc_headset_misc_data = {
	.driver_flag		= DRIVER_HS_MISC_EXT_HP_DET,
	.ext_hpin_gpio		= VERDI_H2W_CABLE_IN1,
	.ext_accessory_type	= USB_AUDIO_OUT,
};

static struct platform_device htc_headset_misc = {
	.name	= "HTC_HEADSET_MISC",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_misc_data,
	},
};

/* HTC_HEADSET_MGR Driver */
static struct platform_device *headset_devices[] = {
	&htc_headset_pmic,
	&htc_headset_misc,
	&htc_headset_8x60,
	&htc_headset_gpio,
	/* Please put the headset detection driver on the last */
};

static struct headset_adc_config htc_headset_mgr_config[] = {
	{
		.type = HEADSET_MIC,
		.adc_max = 27404,
		.adc_min = 22294,
	},
	{
		.type = HEADSET_BEATS,
		.adc_max = 22293,
		.adc_min = 7976,
	},
	{
		.type = HEADSET_MIC,
		.adc_max = 7975,
		.adc_min = 666,
	},
	{
		.type = HEADSET_NO_MIC,
		.adc_max = 665,
		.adc_min = 0,
	},
};

static struct htc_headset_mgr_platform_data htc_headset_mgr_data = {
	.driver_flag		= 0,
	.headset_devices_num	= ARRAY_SIZE(headset_devices),
	.headset_devices	= headset_devices,
	.headset_config_num	= 0,
	.headset_config		= 0,
};

static struct platform_device htc_headset_mgr = {
	.name	= "HTC_HEADSET_MGR",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_mgr_data,
	},
};

static struct pm8058_led_config pm_led_config[] = {
	{
		.name = "amber-landscape",
		.type = PM8058_LED_DRVX,
		.bank = 4,
		.flags = PM8058_LED_LTU_EN,
		.period_us = USEC_PER_SEC / 1000,
		.start_index = 0,
		.duites_size = 8,
		.duty_time_ms = 32,
		.lut_flag = PM_PWM_LUT_RAMP_UP | PM_PWM_LUT_PAUSE_HI_EN,
		.out_current = 20,
	},
	{
		.name = "green-landscape",
		.type = PM8058_LED_CURRENT,
		.bank = 3,
		.flags = PM8058_LED_LTU_EN,
		.period_us = USEC_PER_SEC / 1000,
		.start_index = 0,
		.duites_size = 8,
		.duty_time_ms = 32,
		.lut_flag = PM_PWM_LUT_RAMP_UP | PM_PWM_LUT_PAUSE_HI_EN,
		.out_current = 200,
	},
	{
		.name = "blue-landscape",
		.type = PM8058_LED_DRVX,
		.bank = 5,
		.flags = PM8058_LED_LTU_EN,
		.period_us = USEC_PER_SEC / 1000,
		.start_index = 0,
		.duites_size = 8,
		.duty_time_ms = 32,
		.lut_flag = PM_PWM_LUT_RAMP_UP | PM_PWM_LUT_PAUSE_HI_EN,
		.out_current = 20,
	},
	{
		.name = "button-backlight",
		.type = PM8058_LED_DRVX,
		.bank = 6,
		.flags = PM8058_LED_LTU_EN,
		.period_us = USEC_PER_SEC / 1000,
		.start_index = 0,
		.duites_size = 8,
		.duty_time_ms = 32,
		.lut_flag = PM_PWM_LUT_RAMP_UP | PM_PWM_LUT_PAUSE_HI_EN,
		.out_current = 20,
	},

};

static struct pm8058_led_config pm_led_config_XB[] = {
	{
		.name = "green",
		.type = PM8058_LED_RGB,
		.bank = 0,
		.pwm_size = 9,
		.clk = PM_PWM_CLK_32KHZ,
		.pre_div = PM_PWM_PREDIVIDE_2,
		.pre_div_exp = 1,
		.pwm_value = 511,
	},
	{
		.name = "amber",
		.type = PM8058_LED_RGB,
		.bank = 1,
		.pwm_size = 9,
		.clk = PM_PWM_CLK_32KHZ,
		.pre_div = PM_PWM_PREDIVIDE_2,
		.pre_div_exp = 1,
		.pwm_value = 511,
	},
	{
		.name = "amber-landscape",
		.type = PM8058_LED_DRVX,
		.bank = 4,
		.flags = PM8058_LED_LTU_EN,
		.period_us = USEC_PER_SEC / 1000,
		.start_index = 0,
		.duites_size = 8,
		.duty_time_ms = 32,
		.lut_flag = PM_PWM_LUT_RAMP_UP | PM_PWM_LUT_PAUSE_HI_EN,
		.out_current = 20,
	},
	{
		.name = "green-landscape",
		.type = PM8058_LED_CURRENT,
		.bank = 3,
		.flags = PM8058_LED_LTU_EN,
		.period_us = USEC_PER_SEC / 1000,
		.start_index = 0,
		.duites_size = 8,
		.duty_time_ms = 32,
		.lut_flag = PM_PWM_LUT_RAMP_UP | PM_PWM_LUT_PAUSE_HI_EN,
		.out_current = 200,
	},
	{
		.name = "blue-landscape",
		.type = PM8058_LED_DRVX,
		.bank = 5,
		.flags = PM8058_LED_LTU_EN,
		.period_us = USEC_PER_SEC / 1000,
		.start_index = 0,
		.duites_size = 8,
		.duty_time_ms = 32,
		.lut_flag = PM_PWM_LUT_RAMP_UP | PM_PWM_LUT_PAUSE_HI_EN,
		.out_current = 20,
	},
	{
		.name = "button-backlight",
		.type = PM8058_LED_DRVX,
		.bank = 6,
		.flags = PM8058_LED_LTU_EN,
		.period_us = USEC_PER_SEC / 1000,
		.start_index = 0,
		.duites_size = 8,
		.duty_time_ms = 32,
		.lut_flag = PM_PWM_LUT_RAMP_UP | PM_PWM_LUT_PAUSE_HI_EN,
		.out_current = 20,
	},

};

static struct pm8058_led_platform_data pm8058_leds_data = {
	.led_config = pm_led_config,
	.num_leds = ARRAY_SIZE(pm_led_config),
	.duties = {0, 15, 30, 45, 60, 75, 90, 100,
		   100, 90, 75, 60, 45, 30, 15, 0,
		   0, 0, 0, 0, 0, 0, 0, 0,
		   0, 0, 0, 0, 0, 0, 0, 0,
		   0, 0, 0, 0, 0, 0, 0, 0,
		   0, 0, 0, 0, 0, 0, 0, 0,
		   0, 0, 0, 0, 0, 0, 0, 0,
		   0, 0, 0, 0, 0, 0, 0, 0},
};

static struct platform_device pm8058_leds = {
	.name	= "leds-pm8058",
	.id	= -1,
	.dev	= {
		.platform_data	= &pm8058_leds_data,
	},
};

static struct gpio_led gpio_led_config[] = {
	{
		.name = "green-camera",
		.gpio = PM8058_GPIO_PM_TO_SYS(VERDI_LTE_CAMERA_LED),
		.active_low = 1,
	},
};

static struct gpio_led_platform_data gpio_leds_data = {
	.num_leds = ARRAY_SIZE(gpio_led_config),
	.leds = gpio_led_config,
};

static struct platform_device gpio_leds = {
	.name = "leds-gpio",
	.id = -1,
	.dev = {
		.platform_data = &gpio_leds_data,
	},
};

#ifdef CONFIG_MSM_SDIO_AL
static uint32_t mdm2ap_gpio_table[] = {
	GPIO_CFG(VERDI_LTE_MDM2AP_STATUS, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_MDM2AP_VDDMIN, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_AP2MDM_WAKEUP, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};
static int configure_mdm2ap_status(int on)
{
	return 0;
#if 0
	int ret = 0;
	if (on)
		ret = msm_gpiomux_get(VERDI_LTE_MDM2AP_STATUS);
	else
		ret = msm_gpiomux_put(VERDI_LTE_MDM2AP_STATUS);

	if (ret)
		pr_err("%s: mdm2ap_status config failed, on = %d\n", __func__,
		       on);

	return ret;
#endif
}

static int get_mdm2ap_status(void)
{
	return gpio_get_value(VERDI_LTE_MDM2AP_VDDMIN);
}

static void trigger_mdm_fatal(void)
{
	gpio_set_value(VERDI_LTE_AP2MDM_ERRFATAL, 1);
}

static struct sdio_al_platform_data sdio_al_pdata = {
	.config_mdm2ap_status = configure_mdm2ap_status,
	.get_mdm2ap_status = get_mdm2ap_status,
	.trigger_mdm_fatal = trigger_mdm_fatal,
	.allow_sdioc_version_major_2 = 0,
	.peer_sdioc_version_minor = 0x0001,
	.peer_sdioc_version_major = 0x0003,
	.peer_sdioc_boot_version_minor = 0x0001,
	.peer_sdioc_boot_version_major = 0x0003,
	.mdm2ap_errfatal_gpio = VERDI_LTE_MDM2AP_ERRFATAL,
};

static struct platform_device msm_device_sdio_al = {
	.name = "msm_sdio_al",
	.id = -1,
	.dev		= {
		.platform_data	= &sdio_al_pdata,
	},
};
#endif /* CONFIG_MSM_SDIO_AL */

static struct resource ram_console_resources[] = {
        {
                .start  = MSM_RAM_CONSOLE_BASE,
                .end    = MSM_RAM_CONSOLE_BASE + MSM_RAM_CONSOLE_SIZE - 1,
                .flags  = IORESOURCE_MEM,
        },
};

static struct platform_device ram_console_device = {
	.name		= "ram_console",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(ram_console_resources),
	.resource	= ram_console_resources,
};

static int mdm9k_status;

static int mdm_loaded_status_proc(char *page, char **start, off_t off,
			   int count, int *eof, void *data)
{
	int ret;
	char *p = page;

	if (off > 0) {
		ret = 0;
	} else {
		p += sprintf(p, "%d\n", mdm9k_status);
		ret = p - page;
	}

	return ret;
}

static void mdm_loaded_info(void)
{
	struct proc_dir_entry *entry = NULL;

	mdm9k_status = 0;
	entry = create_proc_read_entry("mdm9k_status", 0, NULL, mdm_loaded_status_proc, NULL);
}

static void charm_ap2mdm_kpdpwr_on(void)
{
	static int first_flag = 1;

	if (first_flag && (system_rev <= 2) && (system_rev >= 1)) { /* only for  XC and XB */
		printk("Trigger ap2mdm_pmic_reset for HW issue...(%d ms)\n", VERDI_LTE_AP2MDM_PMIC_RESET_TIME_MS);
		gpio_set_value(VERDI_LTE_AP2MDM_PMIC_RESET_N, 1);
		msleep(VERDI_LTE_AP2MDM_PMIC_RESET_TIME_MS);
		printk("Release ap2mdm_pmic_reset for HW issue...\n");
		gpio_set_value(VERDI_LTE_AP2MDM_PMIC_RESET_N, 0);
		msleep(100);
		first_flag = 0;
	}

	pr_info("Trigger ap2mdm_kpdpwr...\n");
	gpio_set_value(VERDI_LTE_AP2MDM_KPDPWR_N, 1);
}

static void charm_ap2mdm_kpdpwr_off(void)
{
	pr_info("Release ap2mdm_kpdpwr...\n");
	gpio_set_value(VERDI_LTE_AP2MDM_KPDPWR_N, 0);
	mdm9k_status = 1;
}

static void charm_ap2mdm_pmic_reset(void)
{
	pr_info("Trigger ap2mdm_pmic_reset...(%d ms)\n", VERDI_LTE_AP2MDM_PMIC_RESET_TIME_MS);
	if (system_rev >= 1) {
		gpio_set_value(VERDI_LTE_AP2MDM_PMIC_RESET_N, 1);
		msleep(VERDI_LTE_AP2MDM_PMIC_RESET_TIME_MS);
		pr_info("Release ap2mdm_pmic_reset...\n");
		gpio_set_value(VERDI_LTE_AP2MDM_PMIC_RESET_N, 0);
	} else {
		gpio_set_value(VERDI_LTE_AP2MDM_PMIC_RESET_N, 0);
		msleep(VERDI_LTE_AP2MDM_PMIC_RESET_TIME_MS);
		pr_info("Release ap2mdm_pmic_reset...\n");
		gpio_set_value(VERDI_LTE_AP2MDM_PMIC_RESET_N, 1);
	}
}

static void charm_ap_suspend(void)
{
	uint32_t charm_ap_gpio_table_suspend[] = {
		GPIO_CFG(VERDI_LTE_MDM2AP_STATUS, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	};
	int n;

	pr_info("charm_ap suspending...\n");

	for (n = 0; n < ARRAY_SIZE(charm_ap_gpio_table_suspend); ++n)
		gpio_tlmm_config(charm_ap_gpio_table_suspend[n], GPIO_CFG_ENABLE);
}

static void charm_ap_resume(void)
{
	uint32_t charm_ap_gpio_table_resume[] = {
		GPIO_CFG(VERDI_LTE_MDM2AP_STATUS, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	};
	int n;

	pr_info("charm_ap resuming...\n");

	for (n = 0; n < ARRAY_SIZE(charm_ap_gpio_table_resume); ++n)
		gpio_tlmm_config(charm_ap_gpio_table_resume[n], GPIO_CFG_ENABLE);
}

static struct resource charm_resources[] = {
	{
		.start	= MSM_GPIO_TO_INT(VERDI_LTE_MDM2AP_ERRFATAL),
		.end	= MSM_GPIO_TO_INT(VERDI_LTE_MDM2AP_ERRFATAL),
		.flags = IORESOURCE_IRQ,
	},
	/* MDM2AP_STATUS */
	{
		.start	= MSM_GPIO_TO_INT(VERDI_LTE_MDM2AP_STATUS),
		.end	= MSM_GPIO_TO_INT(VERDI_LTE_MDM2AP_STATUS),
		.flags = IORESOURCE_IRQ,
	}
};

static struct charm_platform_data mdm_platform_data = {
	.charm_modem_on	 = charm_ap2mdm_kpdpwr_on,
	.charm_modem_off = charm_ap2mdm_kpdpwr_off,
	.charm_modem_reset = charm_ap2mdm_pmic_reset,
	.charm_modem_suspend = charm_ap_suspend,
	.charm_modem_resume = charm_ap_resume,

	.gpio_ap2mdm_status = VERDI_LTE_AP2MDM_STATUS,
	.gpio_ap2mdm_wakeup = VERDI_LTE_AP2MDM_WAKEUP,
	.gpio_ap2mdm_errfatal = VERDI_LTE_AP2MDM_ERRFATAL,
	/*.gpio_ap2mdm_sync = VERDI_LTE_AP2MDM_SYNC,*/
	.gpio_ap2mdm_pmic_reset_n = VERDI_LTE_AP2MDM_PMIC_RESET_N,
	.gpio_ap2mdm_kpdpwr_n = VERDI_LTE_AP2MDM_KPDPWR_N,
	.gpio_ap2pmic_tmpni_cken = VERDI_LTE_AP2PMIC_TMPNI_CKEN,

	.gpio_mdm2ap_status = VERDI_LTE_MDM2AP_STATUS,
	.gpio_mdm2ap_wakeup = VERDI_LTE_MDM2AP_WAKEUP,
	.gpio_mdm2ap_errfatal = VERDI_LTE_MDM2AP_ERRFATAL,
	.gpio_mdm2ap_sync = VERDI_LTE_MDM2AP_SYNC,
	.gpio_mdm2ap_vfr = VERDI_LTE_MDM2AP_VFR,
};

static struct platform_device msm_charm_modem = {
	.name		= "charm_modem",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(charm_resources),
	.resource	= charm_resources,
	.dev		= {
		.platform_data = &mdm_platform_data,
	},
};

static struct platform_device *charm_devices[] __initdata = {
	&msm_charm_modem,
};

static struct platform_device msm_tsens_device = {
	.name   = "tsens-tm",
	.id = -1,
};

static struct platform_device simhotswap_device = {
	.name	= "htc_simhotswap",
	.id = -1,
};

static struct platform_device scm_log_device = {
	.name	= "scm-log",
	.id = -1,
};

/* ubattery section bin*/
UINT32 verdi_lte_battery_id_tbl[] =
{
	/* id resister range = [min, max)*/
	1500, 5001,	/* id=1 ATL      10k */
	5001, 10000,	/* id=2 Formosa  22k */
	-1,		/* end of table */
};

int verdi_lte_get_batt_id_ohm(void)
{
	int ret = 0;
	int32_t id_adc[5];

	/* Read battery id adc data. */
	ret = pm8058_htc_config_mpp_and_adc_read(
			id_adc,
			5,
			CHANNEL_ADC_BATT_AMON,
			XOADC_MPP_8,
			PM_MPP_AIN_AMUX_CH6);
	if (ret) {
		goto get_adc_failed;
	}
	ret = (id_adc[0]+id_adc[1]+id_adc[2]+id_adc[3]+id_adc[4]) / 5;
	BATT_LOG("id_adc=%d\n",ret);
get_adc_failed:
	return ret;
}

static void verdi_lte_poweralg_config_init(struct poweralg_config_type *config)
{
	BATT_LOG("%s() is used\n",__func__);
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
	config->polling_time_extension1_sec = 60 * 60;

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

static int verdi_lte_update_charging_protect_flag(struct poweralg_type* poweralg)
{
	static int pState = 0;
	int old_pState = pState;
	const int ibat_ma = poweralg->battery.current_mA;
	const int vbat_mv = poweralg->battery.voltage_mV;
	const int temp_01c = poweralg->battery.temp_01c;
	const BOOL lchg_en = poweralg->protect_flags.is_low_current_charging_enable;
	const BOOL reset = (CONNECT_TYPE_NONE == poweralg->last_charging_source)?TRUE:FALSE;

	/* pStates:
		0: initial (temp detection)
		1: temp < 0 degree c				(CASE A)
		2: 0 <= temp <= 58 degree c			(NORMAL)
		3: 48 < temp <= 58 degree c && 3950 < vbat mV	(CASE C)
		4: 58 < temp					(CASE B)
	*/
	enum {
		PSTAT_DETECT=0,
		PSTAT_LOW_STOP,
		PSTAT_NORMAL,
		PSTAT_LIMITED,
		PSTAT_HIGH_STOP
	};
	/* generally we assumed that pState implies last temp.
		it won't hold if temp changes faster than sample rate */

	/* reset pState: when everytime charger just plugged in */
	if (TRUE == reset) {
		BATT_LOG("Protect pState reset from %d to %d\n", old_pState, PSTAT_DETECT);
		pState = old_pState = PSTAT_DETECT;
	}

	/* step 1. check if change state condition is hit. */
	/*BATT_LOG("%s(i=%d, v=%d, t=%d, en=%d, h-en=%d, l-en%d)\n",
		__func__, ibat_ma, vbat_mv, temp_01c,
		poweralg->protect_flags.is_charging_enable_available,
		poweralg->protect_flags.is_charging_high_current_avaialble,
		poweralg->protect_flags.is_low_current_charging_enable); */
	switch(pState) {
		default:
			BATT_LOG("error: unexpected pState\n");
		case PSTAT_DETECT:
			/* conditions below may not mutual exclusion
			pState may be overwrite by later conditions. */
			if (temp_01c < 0)
				pState = PSTAT_LOW_STOP;
			if ((0 <= temp_01c) && (temp_01c <= 580))
				pState = PSTAT_NORMAL;
			if ((480 < temp_01c) && (temp_01c <= 580) && (3950 < vbat_mv))
				pState = PSTAT_LIMITED;
			if (550 < temp_01c)
				pState = PSTAT_HIGH_STOP;
			break;
		case PSTAT_LOW_STOP:
			if (0 <= temp_01c)
				pState = PSTAT_NORMAL;
			/* suppose never jump to LIMITED/HIGH_STOP from here */
			break;
		case PSTAT_NORMAL:
			if (temp_01c < 0)
				pState = PSTAT_LOW_STOP;
			else if (580 < temp_01c)
				pState = PSTAT_HIGH_STOP;
			else if ((480 < temp_01c) && (3950 < vbat_mv)) /* also implies t <= 580 */
				pState = PSTAT_LIMITED;
			break;
		case PSTAT_LIMITED:
			if ((temp_01c < 450) || (vbat_mv < 3700))
				pState = PSTAT_NORMAL;
			else if (580 < temp_01c)
				pState = PSTAT_HIGH_STOP;
			/* suppose never jump to LOW_STOP from here */
			break;
		case PSTAT_HIGH_STOP:
			if (((temp_01c < 550) && (vbat_mv < 3700)) || (temp_01c < 450))
				pState = PSTAT_NORMAL;
			/* suppose never jump to LOW_STOP from here */
			break;
	}
	if (old_pState != pState)
		BATT_LOG("Protect pState changed from %d to %d\n", old_pState, pState);

	/* step 2. check state protect condition */
	/* "is_charging_enable_available = TRUE" only means it's allowed
		no matter it has charger.
		same as is_charging_high_current_avaialble. */
	switch(pState) {
		default:
		case PSTAT_DETECT:
			BATT_LOG("error: unexpected pState\n");
			break;
		case PSTAT_LOW_STOP:
			poweralg->protect_flags.is_charging_enable_available = FALSE;
			poweralg->protect_flags.is_charging_high_current_avaialble = FALSE;
			poweralg->protect_flags.is_low_current_charging_enable = FALSE;
			poweralg->protect_flags.is_temperature_fault = TRUE;
			break;
		case PSTAT_NORMAL:
			poweralg->protect_flags.is_charging_enable_available = TRUE;
			poweralg->protect_flags.is_charging_high_current_avaialble = TRUE;
			poweralg->protect_flags.is_temperature_fault = FALSE;
			if (PSTAT_NORMAL != old_pState)
				poweralg->protect_flags.is_low_current_charging_enable = FALSE;
			if ((FALSE == lchg_en) && (0 < ibat_ma) && (ibat_ma < 500) && (4100 < vbat_mv))
				poweralg->protect_flags.is_low_current_charging_enable = TRUE;
			else if ((TRUE == lchg_en) && (600 < ibat_ma))
				poweralg->protect_flags.is_low_current_charging_enable = FALSE;
			break;
		case PSTAT_LIMITED:
			poweralg->protect_flags.is_temperature_fault = TRUE;
			if ((PSTAT_LIMITED != old_pState) && (FALSE == lchg_en)) {
				/* first time detect this state */
				poweralg->protect_flags.is_low_current_charging_enable = TRUE;
			}
			else if ((TRUE == lchg_en)) {
				if ((480 < temp_01c) && (3950 < vbat_mv)) {
					/* second time hits this state or
					just first time but lchg_en is already enabled.*/
					poweralg->protect_flags.is_charging_enable_available = FALSE;
				}
			}
			break;
		case PSTAT_HIGH_STOP:
			poweralg->protect_flags.is_temperature_fault = TRUE;
			poweralg->protect_flags.is_charging_enable_available = FALSE;
			poweralg->protect_flags.is_charging_high_current_avaialble = FALSE;
			poweralg->protect_flags.is_low_current_charging_enable = FALSE;
			break;
	}

	/* workaround: override update is_low_current_charging_enable flag
			if is_disable_temp_protect is set and pState!= PSTAT_NORMAL. */
	if (poweralg->protect_flags.is_disable_temp_protect && pState != PSTAT_NORMAL) {
		if ((0 < ibat_ma) && (ibat_ma < 500) && (4100 < vbat_mv))
			poweralg->protect_flags.is_low_current_charging_enable = TRUE;
		else if ((600 < ibat_ma))
			poweralg->protect_flags.is_low_current_charging_enable = FALSE;
	}

	return pState;
}

static int verdi_lte_battery_param_update(struct battery_type *battery, struct protect_flags_type *flags)
{
	if(!bq27510_battery_param_update(battery, flags))
		return FALSE;
	/* calibrate raw data read from gauge */
	/* Tbatt = Tbatt_raw + (Ibatt * 0.80 / 500) [unit: Celcius] */
	battery->temp_01c = battery->temp_01c +
		(battery->current_mA * 80 / 5000);
	return TRUE;
}

static ubattery_platform_data ubattery_device_data = {
	.func_get_thermal_id = NULL,
	.func_get_battery_id = NULL,
	.func_gpio_init = verdi_lte_battery_gpio_init,
	.func_poweralg_config_init = verdi_lte_poweralg_config_init,
	.func_param_update = verdi_lte_battery_param_update,
	.func_capacity_update = NULL,
	.func_capacity_param_update = NULL,
	.func_update_charging_protect_flag = verdi_lte_update_charging_protect_flag,
	.func_charger_ctrl = verdi_lte_battery_charging_ctrl,
	.func_get_batt_id_ohm = verdi_lte_get_batt_id_ohm,
	.r2_kohm = 0,
	.id_tbl = verdi_lte_battery_id_tbl,
};

static struct platform_device ubattery_device = {
	.name = "ubattery_alg",
	.id = -1,
	.dev = {
		.platform_data = &ubattery_device_data,
	},
};
/* ubattery section end*/

static struct platform_device *surf_devices[] __initdata = {
        &ram_console_device,
	&msm_device_smd,
	&msm_device_uart_dm12,
#ifdef CONFIG_I2C_QUP
	&msm_gsbi4_qup_i2c_device,
	&msm_gsbi7_qup_i2c_device,
	&msm_gsbi10_qup_i2c_device,
#endif
#if defined(CONFIG_SPI_QUP) || defined(CONFIG_SPI_QUP_MODULE)
	&msm_gsbi1_qup_spi_device,
	&msm_gsbi8_qup_spi_device,
#endif
#ifdef CONFIG_BT
	&wifi_bt_slp_clk,
	&verdi_lte_rfkill,
#endif
#ifdef CONFIG_SERIAL_MSM_HS
	&msm_device_uart_dm1,
#endif
#ifdef CONFIG_MSM8X60_SSBI
	&msm_device_ssbi1,
	&msm_device_ssbi2,
	&msm_device_ssbi3,
	&msm_device_pm8058,
	&msm_device_pm8901,
#else
#ifdef CONFIG_I2C_SSBI
	&msm_device_ssbi1,
	&msm_device_ssbi2,
	&msm_device_ssbi3,
#endif
#endif /* CONFIG_MSM8X60_SSBI */
#ifdef CONFIG_MSM_DSPS
	&msm_dsps_device,
#endif
#ifdef CONFIG_USB_ANDROID_QCT_DIAG
       &usb_diag_device,
#endif
#ifdef CONFIG_BATTERY_MSM
	&msm_batt_device,
#endif
#ifdef CONFIG_KERNEL_PMEM_SMI_REGION
	&android_pmem_kernel_smi_device,
#endif
#ifdef CONFIG_ANDROID_PMEM
	&android_pmem_device,
	&android_pmem_adsp_device,
	&android_pmem_audio_device,
	&android_pmem_smipool_device,
#endif
#ifdef CONFIG_MSM_ROTATOR
	&msm_rotator_device,
#endif
	&msm_fb_device,
	&msm_kgsl_3d0,
#ifdef CONFIG_MSM_KGSL_2D
	&msm_kgsl_2d0,
	&msm_kgsl_2d1,
#endif
	&lcdc_samsung_panel_device,
	&lcdc_auo_panel_device,
#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
	&hdmi_msm_device,
#endif /* CONFIG_FB_MSM_HDMI_MSM_PANEL */
#ifdef CONFIG_S5K3H2YX
	&msm_camera_sensor_s5k3h2yx,
#endif
#ifdef CONFIG_OV8830
	&msm_camera_sensor_ov8830,
#endif
#ifdef CONFIG_SP3D
	&msm_camera_sensor_sp3d,
#endif
	&msm_camera_sensor_webcam,

#ifdef CONFIG_MSM_GEMINI
	&msm_gemini_device,
#endif
#ifdef CONFIG_MSM_VPE
	&msm_vpe_device,
#endif
#if defined(CONFIG_MSM_RPM_LOG) || defined(CONFIG_MSM_RPM_LOG_MODULE)
	&msm_rpm_log_device,
#endif
#if defined(CONFIG_MSM_RPM_STATS_LOG)
	&msm_rpm_stat_device,
#endif
#ifdef CONFIG_BATTERY_MSM8X60
	&msm_charger_device,
#endif
	&msm_device_vidc,
	&msm_aux_pcm_device,
#ifdef CONFIG_SENSORS_M_ADC
	&msm_adc_device,
#endif
	&htc_headset_mgr,
	&pm8058_leds,
	& gpio_leds,
	&ubattery_device,
#ifdef CONFIG_PMIC8058
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L0],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L1],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L2],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L3],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L4],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L5],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L6],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L7],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L8],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L9],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L10],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L11],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L12],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L13],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L14],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L15],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L16],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L17],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L18],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L19],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L20],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L21],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L22],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L23],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L24],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_L25],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_S2],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_S3],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_S4],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_LVS0],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_LVS1],
	&rpm_vreg_device[RPM_VREG_ID_PM8058_NCP],
#endif
#ifdef CONFIG_PMIC8901
	&rpm_vreg_device[RPM_VREG_ID_PM8901_L0],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_L1],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_L2],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_L3],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_L4],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_L5],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_L6],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_S2],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_S3],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_S4],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_LVS0],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_LVS1],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_LVS2],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_LVS3],
	&rpm_vreg_device[RPM_VREG_ID_PM8901_MVS0],
#endif
	&cable_detect_device,
#ifdef CONFIG_MSM_SDIO_AL
	&msm_device_sdio_al,
#endif

#ifdef CONFIG_HW_RANDOM_MSM
	&msm_device_rng,
#endif
	&msm_tsens_device,
	&scm_log_device,
	&simhotswap_device,
};

#ifdef CONFIG_PMIC8058
#define PMIC_GPIO_SDC3_DET 34

static int pm8058_gpios_init(void)
{
	int i;
	int rc;
	struct pm8058_gpio_cfg {
		int                gpio;
		struct pm8058_gpio cfg;
	};

	struct pm8058_gpio_cfg gpio_cfgs[] = {
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
		{
			PMIC_GPIO_SDC3_DET - 1,
			{
				.direction      = PM_GPIO_DIR_IN,
				.pull           = PM_GPIO_PULL_UP_30,
				.vin_sel        = 2,
				.function       = PM_GPIO_FUNC_NORMAL,
				.inv_int_pol    = 0,
			},
		},
#endif
		{ /* Camera SP3D AVDD */
			VERDI_LTE_SP3D_PMIC_AVDD,	/* 7 */
			{
				.direction	= PM_GPIO_DIR_OUT,
				.output_value	= 0,
				.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
				.pull		= PM_GPIO_PULL_NO,
				.out_strength	= PM_GPIO_STRENGTH_HIGH,
				.function	= PM_GPIO_FUNC_NORMAL,
				.vin_sel	= 6,	/* LDO5 2.85 V */
				.inv_int_pol	= 0,
			}
		},
		{
			VERDI_LTE_AUD_STEREO_REC,
			{
				.direction	= PM_GPIO_DIR_OUT,
				.output_value	= 0,
				.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
				.pull		= PM_GPIO_PULL_NO,
				.out_strength	= PM_GPIO_STRENGTH_HIGH,
				.function	= PM_GPIO_FUNC_NORMAL,
				.vin_sel	= 6,
				.inv_int_pol	= 0,
			}
		},
		{
			VERDI_LTE_9V_AC_DETECT,
			{
				.direction      = PM_GPIO_DIR_IN,
				.pull           = PM_GPIO_PULL_UP_30,
				.vin_sel        = PM_GPIO_VIN_L2,
				.function       = PM_GPIO_FUNC_NORMAL,
				.inv_int_pol    = 0,
			}
		},
		{ /* Audio Microphone Selector */
			VERDI_LTE_AUD_MIC_SEL,
			{
				.direction	= PM_GPIO_DIR_OUT,
				.output_value	= 0,
				.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
				.pull		= PM_GPIO_PULL_NO,
				.out_strength	= PM_GPIO_STRENGTH_HIGH,
				.function	= PM_GPIO_FUNC_NORMAL,
				.vin_sel	= 6,	/* LDO5 2.85 V */
				.inv_int_pol	= 0,
			}
		},
		{
			VERDI_LTE_AUD_HP_EN,
			{
				.direction	= PM_GPIO_DIR_OUT,
				.output_value	= 0,
				.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
				.pull		= PM_GPIO_PULL_NO,
				.out_strength	= PM_GPIO_STRENGTH_HIGH,
				.function	= PM_GPIO_FUNC_NORMAL,
				.vin_sel	= PM_GPIO_VIN_L7,
				.inv_int_pol	= 0,
			}
		},
		{
			VERDI_LTE_AUD_REMO_PRES,
			{
				.direction		= PM_GPIO_DIR_IN,
				.pull			= PM_GPIO_PULL_NO,
				.vin_sel		= PM_GPIO_VIN_L2,
				.function		= PM_GPIO_FUNC_NORMAL,
				.inv_int_pol	= 0,
			}
		},
		{
			VERDI_LTE_AUD_SPK1_EN,
			{
				.direction	= PM_GPIO_DIR_OUT,
				.output_value	= 0,
				.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
				.pull		= PM_GPIO_PULL_NO,
				.out_strength	= PM_GPIO_STRENGTH_HIGH,
				.function	= PM_GPIO_FUNC_NORMAL,
				.vin_sel	= PM_GPIO_VIN_L7,
				.inv_int_pol	= 0,
			}
		},
		{
			VERDI_LTE_AUD_WSPK_EN,
			{
				.direction	= PM_GPIO_DIR_OUT,
				.output_value	= 0,
				.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
				.pull		= PM_GPIO_PULL_NO,
				.out_strength	= PM_GPIO_STRENGTH_HIGH,
				.function	= PM_GPIO_FUNC_NORMAL,
				.vin_sel	= PM_GPIO_VIN_L7,
				.inv_int_pol	= 0,
			}
		},
		{ /* Timpani Reset */
			VERDI_LTE_AUD_CODEC_RST,
			{
				.direction	= PM_GPIO_DIR_OUT,
				.output_value	= 1,
				.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
				.pull		= PM_GPIO_PULL_DN,
				.out_strength	= PM_GPIO_STRENGTH_HIGH,
				.function	= PM_GPIO_FUNC_NORMAL,
				.vin_sel	= PM_GPIO_VIN_S3,
				.inv_int_pol	= 0,
			}
		},
		{
			VERDI_LTE_AUD_REMO_EN_PM,
			{
				.direction	= PM_GPIO_DIR_OUT,
				.output_value	= 0,
				.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
				.pull		= PM_GPIO_PULL_NO,
				.out_strength	= PM_GPIO_STRENGTH_HIGH,
				.function	= PM_GPIO_FUNC_NORMAL,
				.vin_sel	= PM_GPIO_VIN_L2,
				.inv_int_pol	= 0,
			}
		},
		{
			VERDI_LTE_CAMERA_LED,
			{
				.direction	= PM_GPIO_DIR_OUT,
				.output_value	= 1,
				.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
				.pull		= PM_GPIO_PULL_NO,
				.out_strength	= PM_GPIO_STRENGTH_HIGH,
				.function	= PM_GPIO_FUNC_NORMAL,
				.vin_sel	= PM_GPIO_VIN_L6,	/*LDO6 3.075 V*/
				.inv_int_pol	= 0,
			}
		},
		{ /* LCM_EN */
			VERDI_LTE_LCM_EN,
			{
				.direction	= PM_GPIO_DIR_OUT,
				.output_value	= 0,
				.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
				.pull		= PM_GPIO_PULL_NO,
				.out_strength	= PM_GPIO_STRENGTH_NO,
				.function	= PM_GPIO_FUNC_NORMAL,
				.vin_sel	= PM_GPIO_VIN_VPH,	/*  VPH_PWR  */
				.inv_int_pol	= 0,
			}
		},
		{
			VERDI_LTE_V_LCM_3V3_EN,
			{
				.direction	= PM_GPIO_DIR_OUT,
				.output_value	= 1,
				.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
				.pull		= PM_GPIO_PULL_NO,
				.out_strength	= PM_GPIO_STRENGTH_NO,
				.function	= PM_GPIO_FUNC_NORMAL,
				.vin_sel	= PM_GPIO_VIN_VPH,	/*  VPH_PWR  */
				.inv_int_pol	= 0,
			}
		},
                {
                        VERDI_LTE_V_LCM_3V3_EN_1_XC,
                        {
                                .direction      = PM_GPIO_DIR_OUT,
                                .output_value   = 0,
                                .output_buffer  = PM_GPIO_OUT_BUF_CMOS,
                                .pull           = PM_GPIO_PULL_NO,
                                .out_strength   = PM_GPIO_STRENGTH_NO,
                                .function       = PM_GPIO_FUNC_NORMAL,
                                .vin_sel        = PM_GPIO_VIN_L7,
                                .inv_int_pol    = 0,
                        }
                },
		{
			VERDI_LTE_V_LCM_3V3_SYNC,
			{
				.direction	= PM_GPIO_DIR_OUT,
				.output_value	= 0,
				.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
				.pull		= PM_GPIO_PULL_NO,
				.out_strength	= PM_GPIO_STRENGTH_NO,
				.function	= PM_GPIO_FUNC_NORMAL,
				.vin_sel	= PM_GPIO_VIN_VPH,	/*  VPH_PWR  */
				.inv_int_pol	= 0,
			}
		},
		{
			VERDI_LTE_V_LED_EN,
			{
				.direction	= PM_GPIO_DIR_OUT,
				.output_value	= 0,
				.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
				.pull		= PM_GPIO_PULL_NO,
				.out_strength	= PM_GPIO_STRENGTH_NO,
				.function	= PM_GPIO_FUNC_NORMAL,
				.vin_sel	= PM_GPIO_VIN_VPH,	/*  VPH_PWR  */
				.inv_int_pol	= 0,
			}
		},
		{
			VERDI_LTE_LCM_BL_PWM,
			{
				.direction	= PM_GPIO_DIR_OUT,
				.output_value	= 0,
				.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
				.pull		= PM_GPIO_PULL_NO,
				.out_strength	= PM_GPIO_STRENGTH_NO,
				.function	= PM_GPIO_FUNC_2,
				.vin_sel	= (system_rev >= 1)?PM_GPIO_VIN_L6:PM_GPIO_VIN_L5,
				.inv_int_pol	= 0,
			}
		},
		{
			VERDI_LTE_LCM_BL_EN,
			{
				.direction      = PM_GPIO_DIR_OUT,
				.output_value   = 0,
				.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
				.pull           = PM_GPIO_PULL_NO,
				.out_strength   = PM_GPIO_STRENGTH_NO,
				.function       = PM_GPIO_FUNC_NORMAL,
				.vin_sel        = (system_rev >= 1)?PM_GPIO_VIN_L6:PM_GPIO_VIN_L5,
				.inv_int_pol    = 0,
			}
		},
		{
			VERDI_LTE_LCM_DCR_EN_XB,
			{
				.direction	= PM_GPIO_DIR_OUT,
				.output_value	= 0,
				.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
				.pull		= PM_GPIO_PULL_DN,
				.out_strength	= PM_GPIO_STRENGTH_NO,
				.function	= PM_GPIO_FUNC_NORMAL,
				.vin_sel	= PM_GPIO_VIN_BB,
				.inv_int_pol	= 0,
			}
		},
		#if 0
		{
			VERDI_LTE_PLS_INT,
			{
				.direction		= PM_GPIO_DIR_IN,
				.pull			= PM_GPIO_PULL_UP_1P5,
				.vin_sel		= PM_GPIO_VIN_L5,
				.function		= PM_GPIO_FUNC_NORMAL,
				.inv_int_pol	= 0,
			},
		},
		#endif
		{ /* Green LED */
			VERDI_LTE_GREEN_LED,
			{
				.direction	= PM_GPIO_DIR_OUT,
				.output_value	= 1,
				.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
				.pull		= PM_GPIO_PULL_NO,
				.out_strength	= PM_GPIO_STRENGTH_HIGH,
				.function	= PM_GPIO_FUNC_2,
				.vin_sel	= PM_GPIO_VIN_L5,
				.inv_int_pol	= 0,
			}
		},
		{ /* AMBER */
			VERDI_LTE_AMBER_LED,
			{
				.direction	= PM_GPIO_DIR_OUT,
				.output_value	= 1,
				.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
				.pull		= PM_GPIO_PULL_NO,
				.out_strength	= PM_GPIO_STRENGTH_HIGH,
				.function	= PM_GPIO_FUNC_2,
				.vin_sel	= PM_GPIO_VIN_L5,
				.inv_int_pol	= 0,
			}
		},
		#if 0
		{ /* Volume Up Key */
			VERDI_LTE_VOL_UP,
			{
				.direction      = PM_GPIO_DIR_IN,
				.pull           = PM_GPIO_PULL_UP_31P5,
				.vin_sel        = PM_GPIO_VIN_S3,
				.function       = PM_GPIO_FUNC_NORMAL,
				.inv_int_pol    = 0,
			}
		},
		{ /* Volume Down key */
			VERDI_LTE_VOL_DN,
			{
				.direction      = PM_GPIO_DIR_IN,
				.pull           = PM_GPIO_PULL_UP_1P5,
				.vin_sel        = 2,
				.function       = PM_GPIO_FUNC_NORMAL,
				.inv_int_pol    = 0,
			}
		},
		#endif
	};

	for (i = 0; i < ARRAY_SIZE(gpio_cfgs); ++i) {
		if(gpio_cfgs[i].gpio == VERDI_LTE_V_LCM_3V3_SYNC && system_rev != 0)
			continue;
		else if(gpio_cfgs[i].gpio == VERDI_LTE_LCM_DCR_EN_XB && system_rev < 1)
			continue;

		rc = pm8058_gpio_config(gpio_cfgs[i].gpio,
				&gpio_cfgs[i].cfg);
		if (rc < 0) {
			pr_err("%s pmic gpio config failed\n",
				__func__);
			return rc;
		}
	}

	return 0;
}

static struct resource resources_pwrkey[] = {
	{
		.start	= PM8058_PWRKEY_REL_IRQ(PM8058_IRQ_BASE),
		.end	= PM8058_PWRKEY_REL_IRQ(PM8058_IRQ_BASE),
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= PM8058_PWRKEY_PRESS_IRQ(PM8058_IRQ_BASE),
		.end	= PM8058_PWRKEY_PRESS_IRQ(PM8058_IRQ_BASE),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct pmic8058_pwrkey_pdata pwrkey_pdata = {
	.pull_up		= 1,
	.kpd_trigger_delay_us   = 970,
	.wakeup			= 1,
	.pwrkey_time_ms		= 500,
};

#if defined(CONFIG_PMIC8058_OTHC) || defined(CONFIG_PMIC8058_OTHC_MODULE)
#define PM8058_OTHC_CNTR_BASE0	0xA0
#define PM8058_OTHC_CNTR_BASE1	0x134
#define PM8058_OTHC_CNTR_BASE2	0x137

/* HTC OTHC Driver - BEGIN */
#if 0
static struct othc_switch_info htc_headset_othc_switch_info[] = {
	{
		.min_adc_threshold = 0,
		.max_adc_threshold = 50,
		.key_code = HS_MGR_KEY_PLAY,
	},
	{
		.min_adc_threshold = 50,
		.max_adc_threshold = 200,
		.key_code = HS_MGR_KEY_BACKWARD,
	},
	{
		.min_adc_threshold = 200,
		.max_adc_threshold = 500,
		.key_code = HS_MGR_KEY_FORWARD,
	},
};

static struct othc_n_switch_config htc_headset_othc_switch_config = {
	.voltage_settling_time_ms = 0,
	.num_adc_samples = 1,
	.adc_channel = CHANNEL_ADC_HDSET,
	.switch_info = htc_headset_othc_switch_info,
	.num_keys = ARRAY_SIZE(htc_headset_othc_switch_info),
};

static struct hsed_bias_config htc_headset_othc_bias_config = {
	.othc_headset = OTHC_HEADSET_NO,
	.othc_lowcurr_thresh_uA = 100,
	.othc_highcurr_thresh_uA = 500,
	.othc_hyst_prediv_us = 3000,
	.othc_period_clkdiv_us = 3000,
	.othc_hyst_clk_us = 45000,
	.othc_period_clk_us = 6000,
	.othc_wakeup = 1,
};

static struct othc_hsed_config htc_headset_othc_config = {
	.hsed_bias_config = &htc_headset_othc_bias_config,
	.detection_delay_ms = 200,
	/* Switch info */
	.switch_debounce_ms = 1000,
	.othc_support_n_switch = true,
	.switch_config = &htc_headset_othc_switch_config,
};

static struct pmic8058_othc_config_pdata htc_headset_othc_pdata = {
	.micbias_select = OTHC_MICBIAS_2,
	.micbias_capability = OTHC_MICBIAS_HSED,
	.micbias_enable = OTHC_SIGNAL_PWM_TCXO,
	.hsed_config = &htc_headset_othc_config,
	.hsed_name = "htc_headset",
};

static struct resource htc_headset_othc_resources[] = {
	{
		.start = PM8058_SW_2_IRQ(PM8058_IRQ_BASE),
		.end   = PM8058_SW_2_IRQ(PM8058_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = PM8058_IR_2_IRQ(PM8058_IRQ_BASE),
		.end   = PM8058_IR_2_IRQ(PM8058_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "othc_base",
		.start = PM8058_OTHC_CNTR_BASE2,
		.end   = PM8058_OTHC_CNTR_BASE2,
		.flags = IORESOURCE_IO,
	},
};
#endif
/* HTC OTHC Driver - END */

/* MIC_BIAS0 is configured as normal MIC BIAS */
static struct pmic8058_othc_config_pdata othc_config_pdata_0 = {
	.micbias_select = OTHC_MICBIAS_0,
	.micbias_capability = OTHC_MICBIAS,
	.micbias_enable = OTHC_SIGNAL_OFF,
};

/* MIC_BIAS1 is configured as normal for OTHC */
static struct pmic8058_othc_config_pdata othc_config_pdata_1 = {
	.micbias_select = OTHC_MICBIAS_1,
	.micbias_capability = OTHC_MICBIAS,
	.micbias_enable = OTHC_SIGNAL_OFF,
};

static struct resource resources_othc_0[] = {
	{
		.name = "othc_base",
		.start = PM8058_OTHC_CNTR_BASE0,
		.end   = PM8058_OTHC_CNTR_BASE0,
		.flags = IORESOURCE_IO,
	},
};

static struct resource resources_othc_1[] = {
	{
		.start = PM8058_SW_1_IRQ(PM8058_IRQ_BASE),
		.end   = PM8058_SW_1_IRQ(PM8058_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = PM8058_IR_1_IRQ(PM8058_IRQ_BASE),
		.end   = PM8058_IR_1_IRQ(PM8058_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "othc_base",
		.start = PM8058_OTHC_CNTR_BASE1,
		.end   = PM8058_OTHC_CNTR_BASE1,
		.flags = IORESOURCE_IO,
	},
};

#endif /* CONFIG_PMIC8058_OTHC || CONFIG_PMIC8058_OTHC_MODULE */

static int pm8058_pwm_config(struct pwm_device *pwm, int ch, int on)
{
	struct pm8058_gpio pwm_gpio_config = {
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 0,
		.pull           = PM_GPIO_PULL_NO,
		.vin_sel        = PM_GPIO_VIN_VPH,
		.out_strength   = PM_GPIO_STRENGTH_HIGH,
		.function       = PM_GPIO_FUNC_2,
	};

	int rc = -EINVAL;
	int id, mode, max_mA;

	id = mode = max_mA = 0;
	switch (ch) {
	case 0:
	case 1:
	case 2:
		if (on) {
			id = 24 + ch;
			rc = pm8058_gpio_config(id - 1, &pwm_gpio_config);
			if (rc)
				pr_err("%s: pm8058_gpio_config(%d): rc=%d\n",
					__func__, id, rc);
		}
		break;

	case 6:
		id = PM_PWM_LED_FLASH;
		mode = PM_PWM_CONF_PWM1;
		max_mA = 300;
		break;

	case 7:
		id = PM_PWM_LED_FLASH1;
		mode = PM_PWM_CONF_PWM1;
		max_mA = 300;
		break;

	default:
		break;
	}

	if (ch >= 6 && ch <= 7) {
		if (!on) {
			mode = PM_PWM_CONF_NONE;
			max_mA = 0;
		}
		rc = pm8058_pwm_config_led(pwm, id, mode, max_mA);
		if (rc)
			pr_err("%s: pm8058_pwm_config_led(ch=%d): rc=%d\n",
			       __func__, ch, rc);
	}
	return rc;

}

static struct pm8058_pwm_pdata pm8058_pwm_data = {
	.config		= pm8058_pwm_config,
};

#define PM8058_GPIO_INT           88

static struct pm8058_gpio_platform_data pm8058_gpio_data = {
	.gpio_base	= PM8058_GPIO_PM_TO_SYS(0),
	.irq_base	= PM8058_GPIO_IRQ(PM8058_IRQ_BASE, 0),
	.init		= pm8058_gpios_init,
};

static struct pm8058_gpio_platform_data pm8058_mpp_data = {
	.gpio_base	= PM8058_GPIO_PM_TO_SYS(PM8058_GPIOS),
	.irq_base	= PM8058_MPP_IRQ(PM8058_IRQ_BASE, 0),
};

#define PM8058_VREG(_id) { \
	.name = "pm8058-regulator", \
	.id = _id, \
	.platform_data = &pm8058_vreg_init[_id], \
	.data_size = sizeof(pm8058_vreg_init[_id]), \
}

static struct resource resources_rtc[] = {
       {
		.start  = PM8058_RTC_IRQ(PM8058_IRQ_BASE),
		.end    = PM8058_RTC_IRQ(PM8058_IRQ_BASE),
		.flags  = IORESOURCE_IRQ,
       },
       {
		.start  = PM8058_RTC_ALARM_IRQ(PM8058_IRQ_BASE),
		.end    = PM8058_RTC_ALARM_IRQ(PM8058_IRQ_BASE),
		.flags  = IORESOURCE_IRQ,
       },
};

static struct pmic8058_led pmic8058_flash_leds[] = {
	[0] = {
		.name		= "camera:flash0",
		.max_brightness = 15,
		.id		= PMIC8058_ID_FLASH_LED_0,
	},
	[1] = {
		.name		= "camera:flash1",
		.max_brightness = 15,
		.id		= PMIC8058_ID_FLASH_LED_1,
	},
};

static struct pmic8058_leds_platform_data pm8058_flash_leds_data = {
	.num_leds = ARRAY_SIZE(pmic8058_flash_leds),
	.leds	= pmic8058_flash_leds,
};

static struct mfd_cell pm8058_subdevs[] = {
	{	.name = "pm8058-gpio",
		.id		= -1,
		.platform_data	= &pm8058_gpio_data,
		.data_size	= sizeof(pm8058_gpio_data),
	},
	{	.name = "pm8058-mpp",
		.id		= -1,
		.platform_data	= &pm8058_mpp_data,
		.data_size	= sizeof(pm8058_mpp_data),
	},
	{	.name = "pm8058-pwrkey",
		.id	= -1,
		.resources = resources_pwrkey,
		.num_resources = ARRAY_SIZE(resources_pwrkey),
		.platform_data = &pwrkey_pdata,
		.data_size = sizeof(pwrkey_pdata),
	},
	{
		.name = "pm8058-pwm",
		.id = -1,
		.platform_data = &pm8058_pwm_data,
		.data_size = sizeof(pm8058_pwm_data),
	},
#ifdef CONFIG_SENSORS_M_ADC
	{
		.name = "pm8058-xoadc",
		.id = -1,
		.num_resources = 1,
		.resources = &resources_adc,
		.platform_data = &xoadc_pdata,
		.data_size = sizeof(xoadc_pdata),
	},
#endif
#if defined(CONFIG_PMIC8058_OTHC) || defined(CONFIG_PMIC8058_OTHC_MODULE)
	{
		.name = "pm8058-othc",
		.id = 0,
		.platform_data = &othc_config_pdata_0,
		.data_size = sizeof(othc_config_pdata_0),
		.num_resources = ARRAY_SIZE(resources_othc_0),
		.resources = resources_othc_0,
	},
	{
		/* OTHC1 module has headset/switch dection */
		.name = "pm8058-othc",
		.id = 1,
		.num_resources = ARRAY_SIZE(resources_othc_1),
		.resources = resources_othc_1,
		.platform_data = &othc_config_pdata_1,
		.data_size = sizeof(othc_config_pdata_1),
	},
#if 0
	{
		.name = "pm8058-othc",
		.id = 2,
		.platform_data = &htc_headset_othc_pdata,
		.data_size = sizeof(htc_headset_othc_pdata),
		.num_resources = ARRAY_SIZE(htc_headset_othc_resources),
		.resources = htc_headset_othc_resources,
	},
#endif
#endif /* CONFIG_PMIC8058_OTHC || CONFIG_PMIC8058_OTHC_MODULE */
	{
		.name = "pm8058-rtc",
		.id = -1,
		.num_resources  = ARRAY_SIZE(resources_rtc),
		.resources      = resources_rtc,
	},
	{	.name = "pm8058-led",
		.id = -1,
		.platform_data = &pm8058_flash_leds_data,
		.data_size = sizeof(pm8058_flash_leds_data),
	},
	{	.name = "pm8058-upl",
		.id = -1,
	},
#ifdef CONFIG_PMIC8058_BATTALARM
	{
		.name = "pm8058-batt-alarm",
		.id = -1,
		.num_resources = 1,
		.resources = &resources_batt_alarm,
	},
#endif
};

#ifdef CONFIG_MSM8X60_SSBI
static struct pm8058_platform_data pm8058_platform_data = {
	.irq_base = PM8058_IRQ_BASE,
	.irq = MSM_GPIO_TO_INT(PM8058_GPIO_INT),
	.num_subdevs = ARRAY_SIZE(pm8058_subdevs),
	.sub_devices = pm8058_subdevs,
	.irq_trigger_flags = IRQF_TRIGGER_LOW,
};
#else
static struct pm8058_platform_data pm8058_platform_data = {
	.irq_base = PM8058_IRQ_BASE,

	.num_subdevs = ARRAY_SIZE(pm8058_subdevs),
	.sub_devices = pm8058_subdevs,
	.irq_trigger_flags = IRQF_TRIGGER_LOW,
};

static struct i2c_board_info pm8058_boardinfo[] __initdata = {
	{
		I2C_BOARD_INFO("pm8058-core", 0x55),
		.irq = MSM_GPIO_TO_INT(PM8058_GPIO_INT),
		.platform_data = &pm8058_platform_data,
	},
};
#endif /*CONFIG_MSM8X60_SSBI*/
#endif /* CONFIG_PMIC8058 */

#if HASTIMPANI
static struct regulator *vreg_timpani_1;
static struct regulator *vreg_timpani_2;

static unsigned int msm_timpani_setup_power(void)
{
	int rc;

	pr_info("%s", __func__);

	vreg_timpani_1 = regulator_get(NULL, "8058_l0");
	if (IS_ERR(vreg_timpani_1)) {
		pr_err("%s: Unable to get 8058_l0\n", __func__);
		return -ENODEV;
	}

	vreg_timpani_2 = regulator_get(NULL, "8058_s3");
	if (IS_ERR(vreg_timpani_2)) {
		pr_err("%s: Unable to get 8058_s3\n", __func__);
		regulator_put(vreg_timpani_1);
		return -ENODEV;
	}

	rc = regulator_set_voltage(vreg_timpani_1, 1200000, 1200000);
	if (rc) {
		pr_err("%s: unable to set L0 voltage to 1.2V\n", __func__);
		goto fail;
	}

	rc = regulator_set_voltage(vreg_timpani_2, 1800000, 1800000);
	if (rc) {
		pr_err("%s: unable to set S3 voltage to 1.8V\n", __func__);
		goto fail;
	}

	rc = regulator_enable(vreg_timpani_1);
	if (rc) {
		pr_err("%s: Enable regulator 8058_l0 failed\n", __func__);
		goto fail;
	}

	rc = regulator_enable(vreg_timpani_2);
	if (rc) {
		pr_err("%s: Enable regulator 8058_s3 failed\n", __func__);
		regulator_disable(vreg_timpani_1);
		goto fail;
	}

	return rc;

fail:
	regulator_put(vreg_timpani_1);
	regulator_put(vreg_timpani_2);
	return rc;
}

static void msm_timpani_shutdown_power(void)
{
	int rc;

	pr_info("%s", __func__);

	rc = regulator_disable(vreg_timpani_1);
	if (rc)
		pr_err("%s: Disable regulator 8058_l0 failed\n", __func__);

	regulator_put(vreg_timpani_1);

	rc = regulator_disable(vreg_timpani_2);
	if (rc)
		pr_err("%s: Disable regulator 8058_s3 failed\n", __func__);

	regulator_put(vreg_timpani_2);
}

/* Power analog function of codec */
static struct regulator *vreg_timpani_cdc_apwr;
static struct regulator *vreg_timpani_ncp;
static int msm_timpani_codec_power(int vreg_on)
{
	int rc = 0;

	pr_info("%s: %d", __func__, vreg_on);

	if (vreg_on) {
		vreg_timpani_cdc_apwr = regulator_get(NULL, "8058_s4");
		if (IS_ERR(vreg_timpani_cdc_apwr)) {
			pr_err("%s: vreg_get failed (%ld)\n",
			__func__, PTR_ERR(vreg_timpani_cdc_apwr));
			rc = PTR_ERR(vreg_timpani_cdc_apwr);
			return rc;
		}

		vreg_timpani_ncp = regulator_get(NULL, "8058_ncp");
		if (IS_ERR(vreg_timpani_ncp)) {
			pr_err("%s: vreg_get failed (%ld)\n",
			__func__, PTR_ERR(vreg_timpani_ncp));
			rc = PTR_ERR(vreg_timpani_ncp);
			return rc;
		}

		rc = regulator_set_voltage(vreg_timpani_cdc_apwr,
				2200000, 2200000);
		if (rc) {
			pr_err("%s: unable to set 8058_s4 voltage to 2.2 V\n",
					__func__);
			goto vreg_cdc_fail;
		}

		rc = regulator_enable(vreg_timpani_cdc_apwr);
		if (rc) {
			pr_err("%s: vreg_enable failed %d\n", __func__, rc);
			goto vreg_cdc_fail;
		}

		rc = regulator_set_voltage(vreg_timpani_ncp, 1800000, 1800000);
		if (rc < 0) {
			pr_aud_err("%s: set_voltage(ncp) failed (%d)\n", \
				__func__, rc);
			goto vreg_ncp_fail;
		}
		rc = regulator_enable(vreg_timpani_ncp);
		if (rc < 0) {
			pr_aud_err("%s: regulator_enable(ncp) failed (%d)\n",\
				__func__, rc);
			goto vreg_ncp_fail;
		}
	} else {
		rc = regulator_disable(vreg_timpani_cdc_apwr);
		if (rc) {
			pr_err("%s: vreg_disable failed %d\n",
			__func__, rc);
			goto vreg_cdc_fail;
		}
		rc = regulator_disable(vreg_timpani_ncp);
		if (rc < 0)
			pr_aud_err("%s: regulator_disable(ncp) failed (%d)\n",
					__func__, rc);
		regulator_put(vreg_timpani_ncp);
	}

	return 0;

vreg_cdc_fail:
	regulator_put(vreg_timpani_cdc_apwr);
	vreg_timpani_cdc_apwr = NULL;
	return rc;
vreg_ncp_fail:
	regulator_put(vreg_timpani_ncp);
	vreg_timpani_ncp = NULL;
	return rc;
}

static struct marimba_codec_platform_data timpani_codec_pdata = {
	.marimba_codec_power =  msm_timpani_codec_power,
};

#define TIMPANI_SLAVE_ID_CDC_ADDR		0X77
#define TIMPANI_SLAVE_ID_QMEMBIST_ADDR		0X66

static struct marimba_platform_data timpani_pdata = {
	.slave_id[MARIMBA_SLAVE_ID_CDC]	= TIMPANI_SLAVE_ID_CDC_ADDR,
	.slave_id[MARIMBA_SLAVE_ID_QMEMBIST] = TIMPANI_SLAVE_ID_QMEMBIST_ADDR,
	.marimba_setup = msm_timpani_setup_power,
	.marimba_shutdown = msm_timpani_shutdown_power,
	.codec = &timpani_codec_pdata,
};

#define TIMPANI_I2C_SLAVE_ADDR	0xD

static struct i2c_board_info msm_i2c_gsbi7_timpani_info[] = {
	{
		I2C_BOARD_INFO("timpani", TIMPANI_I2C_SLAVE_ADDR),
		.platform_data = &timpani_pdata,
	},
};
#endif

static struct tpa2026_platform_data tpa2026_pdata = {
	.gpio_tpa2026_spk_en = VERDI_LTE_AUD_SPK1_EN,
	.spkr_cmd = {0xC2, 0x05, 0x01, 0x00, 0x16, 0x9A, 0xC0},
};

#define TPA2026_I2C_SLAVE_ADDR	(0xB0 >> 1)

static struct i2c_board_info msm_i2c_gsbi7_tpa2026_info[] = {
	{
		I2C_BOARD_INFO(TPA2026_I2C_NAME, TPA2026_I2C_SLAVE_ADDR),
		.platform_data = &tpa2026_pdata,
	},
};

static struct tpa2028_platform_data tpa2028_pdata = {
	.gpio_tpa2028_spk_en = VERDI_LTE_AUD_WSPK_EN,
	.spkr_cmd = {0xC2, 0x05, 0x01, 0x00, 0x16, 0x9A, 0xC0},
	.off_cmd = {0xE2, 0x05, 0x01, 0x00, 0x30, 0x9A, 0xC0},
};

static struct tpa2028_platform_data tpa2028_pdata_pvt = {
	.gpio_tpa2028_spk_en = VERDI_LTE_AUD_WSPK_EN,
	.spkr_cmd = {0xC2, 0x05, 0x01, 0x00, 0x18, 0x9A, 0xC0},
	.off_cmd = {0xE2, 0x05, 0x01, 0x00, 0x0F, 0x9A, 0xC0},
};

#define TPA2028_I2C_SLAVE_ADDR	(0xB0 >> 1)

static struct i2c_board_info msm_i2c_gsbi10_tpa2028_info[] = {
	{
		I2C_BOARD_INFO(TPA2028_I2C_NAME, TPA2028_I2C_SLAVE_ADDR),
		.platform_data = &tpa2028_pdata,
	},
};

static struct a1028_platform_data a1028_data = {
	.gpio_a1028_wakeup = VERDI_LTE_AUD_A1028_WAKEUP,
	.gpio_a1028_reset = VERDI_LTE_AUD_A1028_RESET,
	.gpio_a1028_micswitch = VERDI_LTE_AUD_STEREO_REC,
};

#define A1028_I2C_SLAVE_ADDR	(0x3E)

static struct i2c_board_info msm_i2c_gsbi10_a1028_info[] = {
	{
		I2C_BOARD_INFO(A1028_I2C_NAME, A1028_I2C_SLAVE_ADDR),
		.platform_data = &a1028_data,
	},
};

static int verdi_lte_ts_ntrig_power(int on)
{
	pr_info("[ts]%s(%d)", __func__, on);

	if (on) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_TP_RST), 1);
	} else {
		if (system_rev == 0)
			gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_SPI_ENABLE), 0);
		else
			gpio_set_value(VERDI_LTE_SPI_ENABLE_XB, 0);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_TP_RST), 0);
	}

	return 0;
}

static int verdi_lte_lcm_ts_power(int on)
{
	pr_info("[ts]%s(%d)", __func__, on);
	if (on) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_V_LCM_3V3_EN), 1);
		msleep(10);
	} else {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_V_LCM_3V3_EN), 0);
	}
	return 0;
}

static int verdi_lte_ts_ntrig_gpio_config(void)
{
	struct pm8058_gpio twen_gpio_config = {
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 1,
		.pull           = PM_GPIO_PULL_NO,
		.vin_sel        = PM_GPIO_VIN_L6,
		.out_strength   = PM_GPIO_STRENGTH_HIGH,
		.function       = PM_GPIO_FUNC_NORMAL,
	};

	struct pm8058_gpio spien_gpio_config = {
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 1,
		.pull           = PM_GPIO_PULL_NO,
		.vin_sel        = PM_GPIO_VIN_L7,
		.out_strength   = PM_GPIO_STRENGTH_HIGH,
		.function       = PM_GPIO_FUNC_NORMAL,
	};

	pm8058_gpio_config(VERDI_LTE_TP_RST, &twen_gpio_config);

	if (system_rev == 0)
		pm8058_gpio_config(VERDI_LTE_SPI_ENABLE, &spien_gpio_config);

	pr_info("[ts]%s", __func__);

	return 0;
}

static struct ntrig_spi_platform_data verdi_lte_ts_ntrig_data[] = {
	{
		.fw_version = 0x1633,
		.abs_x_min = 0,
		.abs_x_max = 9599,
		.abs_y_min = 0,
		.abs_y_max = 7199,
		.fwtwozero = 0x1620,
		.abs_width_min = 0,
		.abs_width_max = 100,
		.orientate = 0x0,
		.abs_pressure_min = 0,
		.abs_pressure_max = 255,
		.spi_enable = VERDI_LTE_SPI_ENABLE_XB,
		.irq_gpio = VERDI_LTE_TP_ATT_N_XB,
		.power = verdi_lte_ts_ntrig_power,
		.pmicgpio_config = verdi_lte_ts_ntrig_gpio_config,
		.lcm_ts_power = verdi_lte_lcm_ts_power,
		.esdFlag = true,
		.palm_rejection_attr = true,
	},
	{
		.fw_version = 0x0,
		.abs_x_min = 0,
		.abs_x_max = 9600,
		.abs_y_min = 0,
		.abs_y_max = 6790,
		.fwtwozero = 0x1620,
		.abs_width_min = 0,
		.abs_width_max = 100,
		.orientate = 0x0,
		.abs_pressure_min = 0,
		.abs_pressure_max = 255,
		.spi_enable = VERDI_LTE_SPI_ENABLE_XB,
		.irq_gpio = VERDI_LTE_TP_ATT_N_XB,
		.power = verdi_lte_ts_ntrig_power,
		.pmicgpio_config = verdi_lte_ts_ntrig_gpio_config,
		.lcm_ts_power = verdi_lte_lcm_ts_power,
		.esdFlag = true,
		.palm_rejection_attr = true,
	},

};

static struct spi_board_info msm_spi_board_info[] __initdata = {
#ifdef CONFIG_MSM8X60_AUDIO_LTE
	{
		.modalias	= "spi_aic3254",
		.mode           = SPI_MODE_1,
		.bus_num        = 0,
		.chip_select    = 1,
		.ext_gpio_cs    = VERDI_LTE_AUD_SPI_CS,
		.max_speed_hz   = 10800000,
	},
#endif
	{
		.modalias	= NTRIG_NAME,
		.mode           = SPI_MODE_0,
		.bus_num        = 0,
		.chip_select    = 0,
		.max_speed_hz   = 10800000,
		.platform_data  = &verdi_lte_ts_ntrig_data,
		.irq	= MSM_GPIO_TO_INT(VERDI_LTE_TP_ATT_N),
	},
};

static struct spi_board_info msm_spi_board_info_xb[] __initdata = {
#ifdef CONFIG_MSM8X60_AUDIO_LTE
	{
		.modalias	= "spi_aic3254",
		.mode           = SPI_MODE_1,
		.bus_num        = 0,
		.chip_select    = 1,
		.ext_gpio_cs    = VERDI_LTE_AUD_SPI_CS_XB,
		.max_speed_hz   = 10800000,
	},
#endif
	{
		.modalias	= NTRIG_NAME,
		.mode           = SPI_MODE_0,
		.bus_num        = 0,
		.chip_select    = 0,
		.max_speed_hz   = 10800000,
		.platform_data  = &verdi_lte_ts_ntrig_data,
		.irq	= MSM_GPIO_TO_INT(VERDI_LTE_TP_ATT_N_XB),
	},
};

#ifdef CONFIG_PMIC8901

#define PM8901_GPIO_INT           91

#ifdef CONFIG_FB_MSM_HDMI_MHL
static int pm8901_mpp_init(void)
{
	int rc;
	pr_err("%s\n", __func__);

	rc = pm8901_mpp_config(0, PM_MPP_TYPE_D_BI_DIR,
			PM8901_MPP_DIG_LEVEL_MSMIO,
			PM_MPP_BI_PULLUP_10KOHM);
	if (rc)
		pr_err("%s: pm8901_mpp_config failed with %d\n", __func__, rc);


	rc = pm8901_mpp_config(1, PM_MPP_TYPE_D_BI_DIR,
			PM8901_MPP_DIG_LEVEL_L5,
			PM_MPP_BI_PULLUP_10KOHM);
	if (rc)
		pr_err("%s: pm8901_mpp_config failed with %d\n", __func__, rc);


	rc = pm8901_mpp_config(2, PM_MPP_TYPE_D_BI_DIR,
			PM8901_MPP_DIG_LEVEL_MSMIO,
			PM_MPP_BI_PULLUP_10KOHM);
	if (rc)
		pr_err("%s: pm8901_mpp_config failed with %d\n", __func__, rc);


	rc = pm8901_mpp_config(3, PM_MPP_TYPE_D_BI_DIR,
			PM8901_MPP_DIG_LEVEL_L5,
			PM_MPP_BI_PULLUP_10KOHM);
	if (rc)
		pr_err("%s: pm8901_mpp_config failed with %d\n", __func__, rc);

	return rc;
}
#endif

static struct pm8901_gpio_platform_data pm8901_mpp_data = {
	.gpio_base	= PM8901_GPIO_PM_TO_SYS(0),
	.irq_base	= PM8901_MPP_IRQ(PM8901_IRQ_BASE, 0),
};
#if 0
static struct resource pm8901_temp_alarm[] = {
	{
		.start = PM8901_TEMP_ALARM_IRQ(PM8901_IRQ_BASE),
		.end = PM8901_TEMP_ALARM_IRQ(PM8901_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = PM8901_TEMP_HI_ALARM_IRQ(PM8901_IRQ_BASE),
		.end = PM8901_TEMP_HI_ALARM_IRQ(PM8901_IRQ_BASE),
		.flags = IORESOURCE_IRQ,
	},
};
#endif
static struct regulator_consumer_supply pm8901_vreg_supply[PM8901_VREG_MAX] = {
	[PM8901_VREG_ID_L0]  = REGULATOR_SUPPLY("8901_l0",  NULL),
	[PM8901_VREG_ID_L1]  = REGULATOR_SUPPLY("8901_l1",  NULL),
	[PM8901_VREG_ID_L2]  = REGULATOR_SUPPLY("8901_l2",  NULL),
	[PM8901_VREG_ID_L3]  = REGULATOR_SUPPLY("8901_l3",  NULL),
	[PM8901_VREG_ID_L4]  = REGULATOR_SUPPLY("8901_l4",  NULL),
	[PM8901_VREG_ID_L5]  = REGULATOR_SUPPLY("8901_l5",  NULL),
	[PM8901_VREG_ID_L6]  = REGULATOR_SUPPLY("8901_l6",  NULL),

	[PM8901_VREG_ID_S3] = REGULATOR_SUPPLY("8901_s3", NULL),
	[PM8901_VREG_ID_S4] = REGULATOR_SUPPLY("8901_s4", NULL),

	[PM8901_VREG_ID_LVS0]     = REGULATOR_SUPPLY("8901_lvs0",     NULL),
	[PM8901_VREG_ID_LVS1]     = REGULATOR_SUPPLY("8901_lvs1",     NULL),
	[PM8901_VREG_ID_LVS2]     = REGULATOR_SUPPLY("8901_lvs2",     NULL),
	[PM8901_VREG_ID_LVS3]     = REGULATOR_SUPPLY("8901_lvs3",     NULL),
	[PM8901_VREG_ID_MVS0]     = REGULATOR_SUPPLY("8901_mvs0",     NULL),
	[PM8901_VREG_ID_USB_OTG]  = REGULATOR_SUPPLY("8901_usb_otg",  NULL),
	[PM8901_VREG_ID_HDMI_MVS] = REGULATOR_SUPPLY("8901_hdmi_mvs", NULL),
};

#define PM8901_VREG_INIT(_id, _min_uV, _max_uV, \
		_modes, _ops, _apply_uV, _init) \
	[_id] = { \
		.constraints = { \
			.valid_modes_mask = _modes, \
			.valid_ops_mask = _ops, \
			.min_uV = _min_uV, \
			.max_uV = _max_uV, \
			.apply_uV = _apply_uV, \
		}, \
		.num_consumer_supplies = 1, \
		.consumer_supplies = &pm8901_vreg_supply[_id], \
		.regulator_init = _init, \
	}

#define PM8901_VREG_INIT_LDO(_id, _min_uV, _max_uV) \
	PM8901_VREG_INIT(_id, _min_uV, _max_uV, REGULATOR_MODE_NORMAL | \
			REGULATOR_MODE_IDLE | REGULATOR_MODE_STANDBY, \
			REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS | \
			REGULATOR_CHANGE_MODE, 1, NULL)

#define PM8901_VREG_INIT_SMPS(_id, _min_uV, _max_uV) \
	PM8901_VREG_INIT(_id, _min_uV, _max_uV, REGULATOR_MODE_NORMAL | \
			REGULATOR_MODE_IDLE | REGULATOR_MODE_STANDBY, \
			REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS | \
			REGULATOR_CHANGE_MODE, 1, NULL)

#define PM8901_VREG_INIT_VS(_id, _init) \
	PM8901_VREG_INIT(_id, 0, 0, REGULATOR_MODE_NORMAL, \
			REGULATOR_CHANGE_STATUS, 0, _init)

static struct regulator_init_data pm8901_vreg_init[PM8901_VREG_MAX] = {
	PM8901_VREG_INIT_VS(PM8901_VREG_ID_USB_OTG,  NULL),
	PM8901_VREG_INIT_VS(PM8901_VREG_ID_HDMI_MVS, NULL),
};

#define PM8901_VREG(_id) { \
	.name = "pm8901-regulator", \
	.id = _id, \
	.platform_data = &pm8901_vreg_init[_id], \
	.data_size = sizeof(pm8901_vreg_init[_id]), \
}

static struct mfd_cell pm8901_subdevs[] = {
	{	.name = "pm8901-mpp",
		.id		= -1,
		.platform_data	= &pm8901_mpp_data,
		.data_size	= sizeof(pm8901_mpp_data),
	},
#if 0
	{	.name = "pm8901-tm",
		.id		= -1,
		.num_resources  = ARRAY_SIZE(pm8901_temp_alarm),
		.resources      = pm8901_temp_alarm,
	},
#endif
	PM8901_VREG(PM8901_VREG_ID_USB_OTG),
	PM8901_VREG(PM8901_VREG_ID_HDMI_MVS),
};

#ifdef CONFIG_MSM8X60_SSBI
static struct pm8901_platform_data pm8901_platform_data = {
	.irq_base = PM8901_IRQ_BASE,
	.irq = MSM_GPIO_TO_INT(PM8901_GPIO_INT),
	.num_subdevs = ARRAY_SIZE(pm8901_subdevs),
	.sub_devices = pm8901_subdevs,
	.irq_trigger_flags = IRQF_TRIGGER_LOW,
};
#else
static struct pm8901_platform_data pm8901_platform_data = {
	.irq_base = PM8901_IRQ_BASE,
	.num_subdevs = ARRAY_SIZE(pm8901_subdevs),
	.sub_devices = pm8901_subdevs,
	.irq_trigger_flags = IRQF_TRIGGER_LOW,
};

static struct i2c_board_info pm8901_boardinfo[] __initdata = {
	{
		I2C_BOARD_INFO("pm8901-core", 0x55),
		.irq = MSM_GPIO_TO_INT(PM8901_GPIO_INT),
		.platform_data = &pm8901_platform_data,
	},
};
#endif /*CONFIG_MSM8X60_SSBI*/
#endif /* CONFIG_PMIC8901 */

#if (0)
static struct mpu3050_platform_data mpu3050_data = {
	.int_config = 0x10,
	.orientation = { -1, 0, 0,
					0, 1, 0,
					0, 0, -1 },
	.level_shifter = 0,

	.accel = {
		.get_slave_descr = get_accel_slave_descr,
		.adapt_num = 5, /* The i2c bus to which the mpu device is connected */
		.bus = EXT_SLAVE_BUS_SECONDARY,
		.address = 0x70 >> 1,
			.orientation = { -1, 0, 0,
							0, -1, 0,
							0, 0, 1 },

	},

	.compass = {
		.get_slave_descr = get_compass_slave_descr,
		.adapt_num = 5, /* The i2c bus to which the mpu device is connected */
		.bus = EXT_SLAVE_BUS_PRIMARY,
		.address = 0x18 >> 1,
			.orientation = { -1, 0, 0,
							0, 1, 0,
							0, 0, -1 },
	},
};

static struct mpu3050_platform_data mpu3050_data_XB = {
	.int_config = 0x10,
	.orientation = { -1, 0, 0,
					0, 1, 0,
					0, 0, -1 },
	.level_shifter = 0,

	.accel = {
		.get_slave_descr = get_accel_slave_descr,
		.adapt_num = 5, /* The i2c bus to which the mpu device is connected */
		.bus = EXT_SLAVE_BUS_SECONDARY,
		.address = 0x70 >> 1,
			.orientation = { -1, 0, 0,
							0, -1, 0,
							0, 0, 1 },

	},

	.compass = {
		.get_slave_descr = get_compass_slave_descr,
		.adapt_num = 5, /* The i2c bus to which the mpu device is connected */
		.bus = EXT_SLAVE_BUS_PRIMARY,
		.address = 0x1A >> 1,
			.orientation = { -1, 0, 0,
							0, 1, 0,
							0, 0, -1 },
	},
};


static struct i2c_board_info __initdata mpu3050_GSBI10_boardinfo[] = {
	{
		I2C_BOARD_INFO("mpu3050", 0xD0 >> 1),
		/*.irq = MSM_GPIO_TO_INT(VERDI_LTE_GYRO_INT),*/
		.platform_data = &mpu3050_data,
	},
};

static struct i2c_board_info __initdata mpu3050_GSBI10_boardinfo_XB[] = {
	{
		I2C_BOARD_INFO("mpu3050", 0xD0 >> 1),
		/*.irq = MSM_GPIO_TO_INT(VERDI_LTE_GYRO_INT),*/
		.platform_data = &mpu3050_data_XB,
	},
};
#endif

#ifdef CONFIG_SENSORS_CAPSENSE
static void capsense_reset(void)
{
	pm8058_mpp_config_digital_out(VERDI_LTE_SAR_RST, PM8058_MPP_DIG_LEVEL_S3, PM_MPP_DOUT_CTL_HIGH);
	udelay(20);
	pm8058_mpp_config_digital_out(VERDI_LTE_SAR_RST, PM8058_MPP_DIG_LEVEL_S3, PM_MPP_DOUT_CTL_LOW);
#ifdef CONFIG_HTC_PUI_I2C_SAR_RECOVERY
	capsense_reconfigure_th();
#else
	mdelay(200);
	csa_set_kadc();
#endif
	printk(KERN_INFO "capsense_reset\n");
}

static void capsense_gpio_init(int enable)
{
	struct pm8058_gpio capsense_gpio_config = {
		.direction      = PM_GPIO_DIR_IN,
		.pull           = PM_GPIO_PULL_UP_31P5,
		.vin_sel        = PM_GPIO_VIN_L5,
		.function       = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol    = 0,
	};

	if (enable) {
		pm8058_mpp_config_digital_out(VERDI_LTE_SAR_RST, PM8058_MPP_DIG_LEVEL_S3, PM_MPP_DOUT_CTL_LOW);
	} else {
		capsense_gpio_config.pull = PM_GPIO_PULL_DN;
		pm8058_mpp_config_digital_out(VERDI_LTE_SAR_RST, PM8058_MPP_DIG_LEVEL_S3, PM_MPP_DOUT_CTL_HIGH);
	}
	pm8058_gpio_config(VERDI_LTE_CAPSENSE_INT_XB, &capsense_gpio_config);
}
/* CapSense platform data */
static struct capsense_platform_data capsense_data = {
	.intr = PM8058_GPIO_PM_TO_SYS(VERDI_LTE_CAPSENSE_INT_XB),
	.gpio_init = capsense_gpio_init,
	.chip_reset = capsense_reset,
};

static struct i2c_board_info verdi_lte_capsense_XB[] = {
	{
		I2C_BOARD_INFO(CAPSENSE_NAME, 0x40 >> 1),
		.platform_data = &capsense_data,
		.irq = MSM_GPIO_TO_INT(
			PM8058_GPIO_PM_TO_SYS(VERDI_LTE_CAPSENSE_INT_XB))
	},
};
#endif

#ifdef CONFIG_I2C
#define I2C_SURF 1
#define I2C_FFA  (1 << 1)
#define I2C_RUMI (1 << 2)
#define I2C_SIM  (1 << 3)

struct i2c_registry {
	u8                     machs;
	int                    bus;
	struct i2c_board_info *info;
	int                    len;
};

static struct i2c_registry msm8x60_i2c_devices[] __initdata = {
#ifndef CONFIG_MSM8X60_SSBI
#ifdef CONFIG_PMIC8058
	{
		I2C_SURF | I2C_FFA,
		MSM_SSBI1_I2C_BUS_ID,
		pm8058_boardinfo,
		ARRAY_SIZE(pm8058_boardinfo),
	},
#endif
#ifdef CONFIG_PMIC8901
	{
		I2C_SURF | I2C_FFA,
		MSM_SSBI2_I2C_BUS_ID,
		pm8901_boardinfo,
		ARRAY_SIZE(pm8901_boardinfo),
	},
#endif
#endif /*CONFIG_MSM8X60_SSBI*/
#ifdef CONFIG_BATTERY_BQ27510
	{
		I2C_SURF | I2C_FFA,
		MSM_GSBI7_QUP_I2C_BUS_ID,
		msm_bq27510_boardinfo,
		ARRAY_SIZE(msm_bq27510_boardinfo),
	},
#endif
#ifdef CONFIG_FB_MSM_HDMI_MHL
#ifdef CONFIG_FB_MSM_HDMI_MHL_SII9234
	{
		I2C_SURF | I2C_FFA,
		MSM_GSBI10_QUP_I2C_BUS_ID,
		msm_i2c_gsbi10_mhl_sii9234_info,
		ARRAY_SIZE(msm_i2c_gsbi10_mhl_sii9234_info),
	},
#endif
#endif
#if HASTIMPANI
	{
		I2C_SURF | I2C_FFA,
		MSM_GSBI7_QUP_I2C_BUS_ID,
		msm_i2c_gsbi7_timpani_info,
		ARRAY_SIZE(msm_i2c_gsbi7_timpani_info),
	},
#endif
	{
		I2C_SURF | I2C_FFA,
		MSM_GSBI7_QUP_I2C_BUS_ID,
		msm_i2c_gsbi7_tpa2026_info,
		ARRAY_SIZE(msm_i2c_gsbi7_tpa2026_info),
	},
	{
		I2C_SURF | I2C_FFA,
		MSM_GSBI10_QUP_I2C_BUS_ID,
		msm_i2c_gsbi10_akm8975_info,
		ARRAY_SIZE(msm_i2c_gsbi10_akm8975_info),
	},
	{
		I2C_SURF | I2C_FFA,
		MSM_GSBI10_QUP_I2C_BUS_ID,
		msm_i2c_gsbi10_bma250_info,
		ARRAY_SIZE(msm_i2c_gsbi10_bma250_info),
	},
	{
		I2C_SURF | I2C_FFA,
		MSM_GSBI10_QUP_I2C_BUS_ID,
		msm_i2c_gsbi10_tpa2028_info,
		ARRAY_SIZE(msm_i2c_gsbi10_tpa2028_info),
	},
	{
		I2C_SURF | I2C_FFA,
		MSM_GSBI10_QUP_I2C_BUS_ID,
		msm_i2c_gsbi10_a1028_info,
		ARRAY_SIZE(msm_i2c_gsbi10_a1028_info),
	},
};
#endif /* CONFIG_I2C */

static struct i2c_registry msm8x60_i2c_devices_cam_s5k3h2yx[] __initdata = {
#ifdef CONFIG_MSM_CAMERA
	{
		I2C_SURF | I2C_FFA,
		MSM_GSBI4_QUP_I2C_BUS_ID,
		msm_camera_boardinfo,
		ARRAY_SIZE(msm_camera_boardinfo),
	},
#endif
};

static struct i2c_registry msm8x60_i2c_devices_cam_ov8830[] __initdata = {
#ifdef CONFIG_MSM_CAMERA
	{
		I2C_SURF | I2C_FFA,
		MSM_GSBI4_QUP_I2C_BUS_ID,
		msm_camera_boardinfo_OV8830,
		ARRAY_SIZE(msm_camera_boardinfo_OV8830),
	},
#endif
};

static void register_i2c_devices(void)
{
#ifdef CONFIG_I2C
	u8 mach_mask = 0;
	int i;
	int rc = 0;

	/* Build the matching 'supported_machs' bitmask */
	if (machine_is_verdi_lte())
		mach_mask = I2C_SURF;
	else
		pr_err("unmatched machine ID in register_i2c_devices\n");

	if (system_rev == 0x80)
		msm_i2c_gsbi10_tpa2028_info[0].platform_data = \
		    &tpa2028_pdata_pvt;

	/* Run the array and install devices as appropriate */
	for (i = 0; i < ARRAY_SIZE(msm8x60_i2c_devices); ++i) {
		if (msm8x60_i2c_devices[i].machs & mach_mask)
			i2c_register_board_info(msm8x60_i2c_devices[i].bus,
						msm8x60_i2c_devices[i].info,
						msm8x60_i2c_devices[i].len);
	}

	rc = gpio_request(VERDI_LTE_CAM_ID, "ov8830");
	if (!rc)
		gpio_direction_output(VERDI_LTE_CAM_ID, 1);
	else
		pr_err("[CAM]VERDI_LTE_CAM_ID GPIO request faile\n");
	gpio_free(VERDI_LTE_CAM_ID);

	if (gpio_get_value(VERDI_LTE_CAM_ID) == 0)
	{
		i2c_register_board_info(msm8x60_i2c_devices_cam_s5k3h2yx[0].bus,
		msm8x60_i2c_devices_cam_s5k3h2yx[0].info,
		msm8x60_i2c_devices_cam_s5k3h2yx[0].len);
	}
	else
	{
		i2c_register_board_info(msm8x60_i2c_devices_cam_ov8830[0].bus,
		msm8x60_i2c_devices_cam_ov8830[0].info,
		msm8x60_i2c_devices_cam_ov8830[0].len);
	}

	rc = gpio_request(VERDI_LTE_CAM_ID, "ov8830");
	if (!rc)
		gpio_direction_output(VERDI_LTE_CAM_ID, 0);
	else
		pr_err("[CAM]VERDI_LTE_CAM_ID GPIO request faile\n");
	gpio_free(VERDI_LTE_CAM_ID);

#ifdef CONFIG_SENSORS_CAPSENSE
	if (system_rev >= 1)
		i2c_register_board_info(MSM_GSBI7_QUP_I2C_BUS_ID, verdi_lte_capsense_XB, ARRAY_SIZE(verdi_lte_capsense_XB));
#endif

	/*if (system_rev >= 1) {
		i2c_register_board_info(MSM_GSBI10_QUP_I2C_BUS_ID,
				i2c_CM3628_devices, ARRAY_SIZE(i2c_CM3628_devices));

		i2c_register_board_info(MSM_GSBI10_QUP_I2C_BUS_ID,
				mpu3050_GSBI10_boardinfo_XB, ARRAY_SIZE(mpu3050_GSBI10_boardinfo_XB));
	} else
		i2c_register_board_info(MSM_GSBI10_QUP_I2C_BUS_ID,
				mpu3050_GSBI10_boardinfo, ARRAY_SIZE(mpu3050_GSBI10_boardinfo));*/


#endif
}

static void __init msm8x60_init_buses(void)
{
#ifdef CONFIG_I2C_QUP
	msm_gsbi4_qup_i2c_device.dev.platform_data = &msm_gsbi4_qup_i2c_pdata;
	msm_gsbi7_qup_i2c_device.dev.platform_data = &msm_gsbi7_qup_i2c_pdata;
	msm_gsbi10_qup_i2c_device.dev.platform_data = &msm_gsbi10_qup_i2c_pdata;
#endif
#if defined(CONFIG_SPI_QUP) || defined(CONFIG_SPI_QUP_MODULE)
	msm_gsbi1_qup_spi_device.dev.platform_data = &msm_gsbi1_qup_spi_pdata;
	msm_gsbi8_qup_spi_device.dev.platform_data = &msm_gsbi8_qup_spi_pdata;
#endif
#ifdef CONFIG_MSM8X60_SSBI
	msm_device_ssbi1.dev.platform_data = &msm_ssbi1_pdata;
	msm_device_ssbi2.dev.platform_data = &msm_ssbi2_pdata;
	msm_device_ssbi3.dev.platform_data = &msm_ssbi3_pdata;
	msm_device_pm8058.dev.platform_data = &pm8058_platform_data;
	msm_device_pm8901.dev.platform_data = &pm8901_platform_data;
#else
#ifdef CONFIG_I2C_SSBI
	msm_device_ssbi1.dev.platform_data = &msm_ssbi1_pdata;
	msm_device_ssbi2.dev.platform_data = &msm_ssbi2_pdata;
	msm_device_ssbi3.dev.platform_data = &msm_ssbi3_pdata;
#endif
#endif /* CONFIG_MSM8X60_SSBI */
#if defined(CONFIG_USB_GADGET_MSM_72K) || defined(CONFIG_USB_EHCI_HCD)
	msm_device_otg.dev.platform_data = &msm_otg_pdata;
#endif

#if defined(CONFIG_USB_F_SERIAL_SDIO) || defined(CONFIG_USB_F_SERIAL_SMD)
	{
		struct usb_gadget_fserial_platform_data *fserial_pdata =
				usb_gadget_fserial_device.dev.platform_data;
		fserial_pdata->no_ports = 4;
		fserial_pdata->transport[0] =
			USB_GADGET_FSERIAL_TRANSPORT_SDIO; /* 9k modem */
		fserial_pdata->transport[1] =
			USB_GADGET_FSERIAL_TRANSPORT_TTY;
		fserial_pdata->transport[2] =
			USB_GADGET_FSERIAL_TRANSPORT_TTY;
		fserial_pdata->transport[3] =
			USB_GADGET_FSERIAL_TRANSPORT_TTY;

		fserial_pdata->func_type[0] = USB_FSER_FUNC_MODEM;
		fserial_pdata->func_type[1] = USB_FSER_FUNC_NONE;
		fserial_pdata->func_type[2] = USB_FSER_FUNC_NONE;
		fserial_pdata->func_type[3] = USB_FSER_FUNC_SERIAL;
	}
#endif

#ifdef CONFIG_SERIAL_MSM_HS
	msm_uart_dm1_pdata.rx_wakeup_irq = gpio_to_irq(VERDI_LTE_GPIO_BT_HOST_WAKE);
	msm_device_uart_dm1.name = "msm_serial_hs_brcm"; /* for brcm */
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
#endif
#ifdef CONFIG_MSM_GSBI9_UART
	msm_device_uart_gsbi9.dev.platform_data =
					&msm_uart_gsbi9_pdata;
	platform_device_register(&msm_device_uart_gsbi9);
#endif

#ifdef CONFIG_MSM_BUS_SCALING

	/* RPM calls are only enabled on V2 */
	if (SOCINFO_VERSION_MAJOR(socinfo_get_version()) == 2) {
		msm_bus_apps_fabric_pdata.rpm_enabled = 1;
		msm_bus_sys_fabric_pdata.rpm_enabled = 1;
		msm_bus_mm_fabric_pdata.rpm_enabled = 1;
		msm_bus_sys_fpb_pdata.rpm_enabled = 1;
		msm_bus_cpss_fpb_pdata.rpm_enabled = 1;
	}

	msm_bus_apps_fabric.dev.platform_data = &msm_bus_apps_fabric_pdata;
	msm_bus_sys_fabric.dev.platform_data = &msm_bus_sys_fabric_pdata;
	msm_bus_mm_fabric.dev.platform_data = &msm_bus_mm_fabric_pdata;
	msm_bus_sys_fpb.dev.platform_data = &msm_bus_sys_fpb_pdata;
	msm_bus_cpss_fpb.dev.platform_data = &msm_bus_cpss_fpb_pdata;
#endif

}

static void __init verdi_lte_map_io(void)
{
	msm_shared_ram_phys = MSM_SHARED_RAM_PHYS;
	msm_map_msm8x60_io();
	msm8x60_allocate_memory_regions();
}

static void __init msm8x60_init_tlmm(void)
{
}

#define GPIO_SDC3_WP_SWITCH (GPIO_EXPANDER_GPIO_BASE + (16 * 1) + 6)
#if (defined(CONFIG_MMC_MSM_SDC1_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC2_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC3_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC4_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC5_SUPPORT))

/* 8x60 is having 5 SDCC controllers */
#define MAX_SDCC_CONTROLLER	5

struct msm_sdcc_gpio {
	/* maximum 10 GPIOs per SDCC controller */
	s16 no;
	/* name of this GPIO */
	const char *name;
};

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
static struct msm_sdcc_gpio sdc1_gpio_cfg[] = {
	{159, "sdc1_dat_0"},
	{160, "sdc1_dat_1"},
	{161, "sdc1_dat_2"},
	{162, "sdc1_dat_3"},
#ifdef CONFIG_MMC_MSM_SDC1_8_BIT_SUPPORT
	{163, "sdc1_dat_4"},
	{164, "sdc1_dat_5"},
	{165, "sdc1_dat_6"},
	{166, "sdc1_dat_7"},
#endif
	{167, "sdc1_clk"},
	{168, "sdc1_cmd"}
};
static uint32_t sdc1_on_gpio_table[] = {
	GPIO_CFG(159, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA), /* DAT0 */
	GPIO_CFG(160, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA), /* DAT1 */
	GPIO_CFG(161, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA), /* DAT2 */
	GPIO_CFG(162, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA), /* DAT3 */
#ifdef CONFIG_MMC_MSM_SDC1_8_BIT_SUPPORT
	GPIO_CFG(163, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA), /* DAT4 */
	GPIO_CFG(164, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA), /* DAT5 */
	GPIO_CFG(165, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA), /* DAT6 */
	GPIO_CFG(166, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA), /* DAT7 */
#endif
	GPIO_CFG(167, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), /* CLK */
	GPIO_CFG(168, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA), /* CMD */
};
#endif

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
static struct msm_sdcc_gpio sdc2_gpio_cfg[] = {
	{143, "sdc2_dat_0"},
	{144, "sdc2_dat_1"},
	{145, "sdc2_dat_2"},
	{146, "sdc2_dat_3"},
#ifdef CONFIG_MMC_MSM_SDC2_8_BIT_SUPPORT
	{147, "sdc2_dat_4"},
	{148, "sdc2_dat_5"},
	{149, "sdc2_dat_6"},
	{150, "sdc2_dat_7"},
#endif
	{151, "sdc2_cmd"},
	{152, "sdc2_clk"}
};
#endif

#ifdef CONFIG_MMC_MSM_SDC5_SUPPORT
static struct msm_sdcc_gpio sdc5_gpio_cfg[] = {
	{95, "sdc5_cmd"},
	{96, "sdc5_dat_3"},
	{97, "sdc5_clk"},
	{98, "sdc5_dat_2"},
	{99, "sdc5_dat_1"},
	{100, "sdc5_dat_0"}
};
#endif

struct msm_sdcc_pad_pull_cfg {
	enum msm_tlmm_pull_tgt pull;
	u32 pull_val;
};

struct msm_sdcc_pad_drv_cfg {
	enum msm_tlmm_hdrive_tgt drv;
	u32 drv_val;
};

#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
static struct msm_sdcc_pad_drv_cfg sdc3_pad_on_drv_cfg[] = {
	{TLMM_HDRV_SDC3_CLK, GPIO_CFG_16MA},
	{TLMM_HDRV_SDC3_CMD, GPIO_CFG_16MA},
	{TLMM_HDRV_SDC3_DATA, GPIO_CFG_16MA}
};

static struct msm_sdcc_pad_pull_cfg sdc3_pad_on_pull_cfg[] = {
	{TLMM_PULL_SDC3_CMD, GPIO_CFG_PULL_UP},
	{TLMM_PULL_SDC3_DATA, GPIO_CFG_PULL_UP}
};

static struct msm_sdcc_pad_drv_cfg sdc3_pad_off_drv_cfg[] = {
	{TLMM_HDRV_SDC3_CLK, GPIO_CFG_2MA},
	{TLMM_HDRV_SDC3_CMD, GPIO_CFG_2MA},
	{TLMM_HDRV_SDC3_DATA, GPIO_CFG_2MA}
};

static struct msm_sdcc_pad_pull_cfg sdc3_pad_off_pull_cfg[] = {
	{TLMM_PULL_SDC3_CMD, GPIO_CFG_PULL_DOWN},
	{TLMM_PULL_SDC3_DATA, GPIO_CFG_PULL_DOWN}
};
#endif

#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
static struct msm_sdcc_pad_drv_cfg sdc4_pad_on_drv_cfg[] = {
	{TLMM_HDRV_SDC4_CLK, GPIO_CFG_8MA},
	{TLMM_HDRV_SDC4_CMD, GPIO_CFG_8MA},
	{TLMM_HDRV_SDC4_DATA, GPIO_CFG_8MA}
};

static struct msm_sdcc_pad_pull_cfg sdc4_pad_on_pull_cfg[] = {
	{TLMM_PULL_SDC4_CMD, GPIO_CFG_PULL_UP},
	{TLMM_PULL_SDC4_DATA, GPIO_CFG_PULL_UP}
};

static struct msm_sdcc_pad_drv_cfg sdc4_pad_off_drv_cfg[] = {
	{TLMM_HDRV_SDC4_CLK, GPIO_CFG_2MA},
	{TLMM_HDRV_SDC4_CMD, GPIO_CFG_2MA},
	{TLMM_HDRV_SDC4_DATA, GPIO_CFG_2MA}
};

static struct msm_sdcc_pad_pull_cfg sdc4_pad_off_pull_cfg[] = {
	{TLMM_PULL_SDC4_CMD, GPIO_CFG_PULL_DOWN},
	{TLMM_PULL_SDC4_DATA, GPIO_CFG_PULL_DOWN}
};
#endif

struct msm_sdcc_pin_cfg {
	/*
	 * = 1 if controller pins are using gpios
	 * = 0 if controller has dedicated MSM pins
	 */
	u8 is_gpio;
	u8 cfg_sts;
	u8 gpio_data_size;
	struct msm_sdcc_gpio *gpio_data;
	struct msm_sdcc_pad_drv_cfg *pad_drv_on_data;
	struct msm_sdcc_pad_drv_cfg *pad_drv_off_data;
	struct msm_sdcc_pad_pull_cfg *pad_pull_on_data;
	struct msm_sdcc_pad_pull_cfg *pad_pull_off_data;
	u8 pad_drv_data_size;
	u8 pad_pull_data_size;
};


static struct msm_sdcc_pin_cfg sdcc_pin_cfg_data[MAX_SDCC_CONTROLLER] = {
#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
	[0] = {
		.is_gpio = 1,
		.gpio_data_size = ARRAY_SIZE(sdc1_gpio_cfg),
		.gpio_data = sdc1_gpio_cfg
	},
#endif
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	[1] = {
		.is_gpio = 1,
		.gpio_data_size = ARRAY_SIZE(sdc2_gpio_cfg),
		.gpio_data = sdc2_gpio_cfg
	},
#endif
#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
	[2] = {
		.is_gpio = 0,
		.pad_drv_on_data = sdc3_pad_on_drv_cfg,
		.pad_drv_off_data = sdc3_pad_off_drv_cfg,
		.pad_pull_on_data = sdc3_pad_on_pull_cfg,
		.pad_pull_off_data = sdc3_pad_off_pull_cfg,
		.pad_drv_data_size = ARRAY_SIZE(sdc3_pad_on_drv_cfg),
		.pad_pull_data_size = ARRAY_SIZE(sdc3_pad_on_pull_cfg)
	},
#endif
#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
	[3] = {
		.is_gpio = 0,
		.pad_drv_on_data = sdc4_pad_on_drv_cfg,
		.pad_drv_off_data = sdc4_pad_off_drv_cfg,
		.pad_pull_on_data = sdc4_pad_on_pull_cfg,
		.pad_pull_off_data = sdc4_pad_off_pull_cfg,
		.pad_drv_data_size = ARRAY_SIZE(sdc4_pad_on_drv_cfg),
		.pad_pull_data_size = ARRAY_SIZE(sdc4_pad_on_pull_cfg)
	},
#endif
#ifdef CONFIG_MMC_MSM_SDC5_SUPPORT
	[4] = {
		.is_gpio = 1,
		.gpio_data_size = ARRAY_SIZE(sdc5_gpio_cfg),
		.gpio_data = sdc5_gpio_cfg
	}
#endif
};

static int msm_sdcc_setup_gpio(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct msm_sdcc_pin_cfg *curr;
	int n;

	curr = &sdcc_pin_cfg_data[dev_id - 1];
	if (!curr->gpio_data)
		goto out;

	for (n = 0; n < curr->gpio_data_size; n++) {
		if (enable) {
			rc = gpio_request(curr->gpio_data[n].no,
				curr->gpio_data[n].name);
			if (rc) {
				pr_err("%s: gpio_request(%d, %s)"
					"failed", __func__,
					curr->gpio_data[n].no,
					curr->gpio_data[n].name);
				goto free_gpios;
			}
			/* set direction as output for all GPIOs */
			rc = gpio_direction_output(
				curr->gpio_data[n].no, 1);
			if (rc) {
				pr_err("%s: gpio_direction_output"
					"(%d, 1) failed\n", __func__,
					curr->gpio_data[n].no);
				goto free_gpios;
			}
		} else {
			/*
			 * now free this GPIO which will put GPIO
			 * in low power mode and will also put GPIO
			 * in input mode
			 */
			gpio_free(curr->gpio_data[n].no);
		}
	}
	curr->cfg_sts = enable;
	goto out;

free_gpios:
	for (; n >= 0; n--)
		gpio_free(curr->gpio_data[n].no);
out:
	return rc;
}

static int msm_sdcc_setup_pad(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct msm_sdcc_pin_cfg *curr;
	int n;

	curr = &sdcc_pin_cfg_data[dev_id - 1];
	if (!curr->pad_drv_on_data || !curr->pad_pull_on_data)
		goto out;

	if (enable) {
		/*
		 * set up the normal driver strength and
		 * pull config for pads
		 */
		for (n = 0; n < curr->pad_drv_data_size; n++)
			msm_tlmm_set_hdrive(curr->pad_drv_on_data[n].drv,
				curr->pad_drv_on_data[n].drv_val);
		for (n = 0; n < curr->pad_pull_data_size; n++)
			msm_tlmm_set_pull(curr->pad_pull_on_data[n].pull,
				curr->pad_pull_on_data[n].pull_val);
	} else {
		/* set the low power config for pads */
		for (n = 0; n < curr->pad_drv_data_size; n++)
			msm_tlmm_set_hdrive(
				curr->pad_drv_off_data[n].drv,
				curr->pad_drv_off_data[n].drv_val);
		for (n = 0; n < curr->pad_pull_data_size; n++)
			msm_tlmm_set_pull(
				curr->pad_pull_off_data[n].pull,
				curr->pad_pull_off_data[n].pull_val);
	}
	curr->cfg_sts = enable;
out:
	return rc;
}

struct sdcc_reg {
	/* VDD/VCC/VCCQ regulator name on PMIC8058/PMIC8089*/
	const char *reg_name;
	/*
	 * is set voltage supported for this regulator?
	 * 0 = not supported, 1 = supported
	 */
	unsigned char set_voltage_sup;
	/* voltage level to be set */
	unsigned int level;
	/* VDD/VCC/VCCQ voltage regulator handle */
	struct regulator *reg;
	/* is this regulator enabled? */
	bool enabled;
	/* is this regulator needs to be always on? */
	bool always_on;
	/* is operating power mode setting required for this regulator? */
	bool op_pwr_mode_sup;
	/* Load values for low power and high power mode */
	unsigned int lpm_uA;
	unsigned int hpm_uA;
};
/* all SDCC controllers requires VDD/VCC voltage */
static struct sdcc_reg sdcc_vdd_reg_data[MAX_SDCC_CONTROLLER];
/* only SDCC1 requires VCCQ voltage */
static struct sdcc_reg sdcc_vccq_reg_data[1];

struct sdcc_reg_data {
	struct sdcc_reg *vdd_data; /* keeps VDD/VCC regulator info */
	struct sdcc_reg *vccq_data; /* keeps VCCQ regulator info */
	struct sdcc_reg *vddp_data; /* keeps VDD Pad regulator info */
	unsigned char sts; /* regulator enable/disable status */
};
/* msm8x60 have 5 SDCC controllers */
static struct sdcc_reg_data sdcc_vreg_data[MAX_SDCC_CONTROLLER];

static int msm_sdcc_vreg_init_reg(struct sdcc_reg *vreg)
{
	int rc = 0;

	/* Get the regulator handle */
	vreg->reg = regulator_get(NULL, vreg->reg_name);
	if (IS_ERR(vreg->reg)) {
		rc = PTR_ERR(vreg->reg);
		pr_err("%s: regulator_get(%s) failed. rc=%d\n",
			__func__, vreg->reg_name, rc);
		goto out;
	}

	/* Set the voltage level if required */
	if (vreg->set_voltage_sup) {
		rc = regulator_set_voltage(vreg->reg, vreg->level,
					vreg->level);
		if (rc) {
			pr_err("%s: regulator_set_voltage(%s) failed rc=%d\n",
				__func__, vreg->reg_name, rc);
			goto vreg_put;
		}
	}
	goto out;

vreg_put:
	regulator_put(vreg->reg);
out:
	return rc;
}

static inline void msm_sdcc_vreg_deinit_reg(struct sdcc_reg *vreg)
{
	regulator_put(vreg->reg);
}

/* this init function should be called only once for each SDCC */
static int msm_sdcc_vreg_init(int dev_id, unsigned char init)
{
	int rc = 0;
	struct sdcc_reg *curr_vdd_reg, *curr_vccq_reg, *curr_vddp_reg;
	struct sdcc_reg_data *curr;

	curr = &sdcc_vreg_data[dev_id - 1];
	curr_vdd_reg = curr->vdd_data;
	curr_vccq_reg = curr->vccq_data;
	curr_vddp_reg = curr->vddp_data;

	if (init) {
		/*
		 * get the regulator handle from voltage regulator framework
		 * and then try to set the voltage level for the regulator
		 */
		if (curr_vdd_reg) {
			rc = msm_sdcc_vreg_init_reg(curr_vdd_reg);
			if (rc)
				goto out;
		}
		if (curr_vccq_reg) {
			rc = msm_sdcc_vreg_init_reg(curr_vccq_reg);
			if (rc)
				goto vdd_reg_deinit;
		}
		if (curr_vddp_reg) {
			rc = msm_sdcc_vreg_init_reg(curr_vddp_reg);
			if (rc)
				goto vccq_reg_deinit;
		}
		goto out;
	} else
		/* deregister with all regulators from regulator framework */
		goto vddp_reg_deinit;

vddp_reg_deinit:
	if (curr_vddp_reg)
		msm_sdcc_vreg_deinit_reg(curr_vddp_reg);
vccq_reg_deinit:
	if (curr_vccq_reg)
		msm_sdcc_vreg_deinit_reg(curr_vccq_reg);
vdd_reg_deinit:
	if (curr_vdd_reg)
		msm_sdcc_vreg_deinit_reg(curr_vdd_reg);
out:
	return rc;
}

static int msm_sdcc_vreg_enable(struct sdcc_reg *vreg)
{
	int rc;

	if (!vreg->enabled) {
		mdelay(5);
		rc = regulator_enable(vreg->reg);
		if (rc) {
			pr_err("%s: regulator_enable(%s) failed. rc=%d\n",
				__func__, vreg->reg_name, rc);
			goto out;
		}
		vreg->enabled = 1;
	}

	/* Put always_on regulator in HPM (high power mode) */
	if (vreg->always_on && vreg->op_pwr_mode_sup) {
		rc = regulator_set_optimum_mode(vreg->reg, vreg->hpm_uA);
		if (rc < 0) {
			pr_err("%s: reg=%s: HPM setting failed"
				" hpm_uA=%d, rc=%d\n",
				__func__, vreg->reg_name,
				vreg->hpm_uA, rc);
			goto vreg_disable;
		}
		rc = 0;
	}
	goto out;

vreg_disable:
	regulator_disable(vreg->reg);
	vreg->enabled = 0;
out:
	return rc;
}

static int msm_sdcc_vreg_disable(struct sdcc_reg *vreg)
{
	int rc;

	/* Never disable always_on regulator */
	if (!vreg->always_on) {
		rc = regulator_disable(vreg->reg);
		if (rc) {
			pr_err("%s: regulator_disable(%s) failed. rc=%d\n",
				__func__, vreg->reg_name, rc);
			goto out;
		}
		vreg->enabled = 0;
	}

	/* Put always_on regulator in LPM (low power mode) */
	if (vreg->always_on && vreg->op_pwr_mode_sup) {
		rc = regulator_set_optimum_mode(vreg->reg, vreg->lpm_uA);
		if (rc < 0) {
			pr_err("%s: reg=%s: LPM setting failed"
				" lpm_uA=%d, rc=%d\n",
				__func__,
				vreg->reg_name,
				vreg->lpm_uA, rc);
			goto out;
		}
		rc = 0;
	}

out:
	return rc;
}

static int msm_sdcc_setup_vreg(int dev_id, unsigned char enable)
{
	int rc = 0;
	struct sdcc_reg *curr_vdd_reg, *curr_vccq_reg, *curr_vddp_reg;
	struct sdcc_reg_data *curr;

	curr = &sdcc_vreg_data[dev_id - 1];
	curr_vdd_reg = curr->vdd_data;
	curr_vccq_reg = curr->vccq_data;
	curr_vddp_reg = curr->vddp_data;

	/* check if regulators are initialized or not? */
	if ((curr_vdd_reg && !curr_vdd_reg->reg) ||
		(curr_vccq_reg && !curr_vccq_reg->reg) ||
		(curr_vddp_reg && !curr_vddp_reg->reg)) {
		/* initialize voltage regulators required for this SDCC */
		rc = msm_sdcc_vreg_init(dev_id, 1);
		if (rc) {
			pr_err("%s: regulator init failed = %d\n",
				__func__, rc);
			goto out;
		}
	}

	if (curr->sts == enable)
		goto out;

	if (curr_vdd_reg) {
		if (enable) {
			if(dev_id == 3)
				printk(KERN_INFO "%s: Enabling SD slot power\n", __func__);
			rc = msm_sdcc_vreg_enable(curr_vdd_reg);
		} else {
			if(dev_id == 3)
				printk(KERN_INFO "%s: Disabling SD slot power\n", __func__);
			rc = msm_sdcc_vreg_disable(curr_vdd_reg);
		}
		if (rc)
			goto out;
	}

	if (curr_vccq_reg) {
		if (enable)
			rc = msm_sdcc_vreg_enable(curr_vccq_reg);
		else
			rc = msm_sdcc_vreg_disable(curr_vccq_reg);
		if (rc)
			goto out;
	}

	if (curr_vddp_reg) {
		if (enable)
			rc = msm_sdcc_vreg_enable(curr_vddp_reg);
		else
			rc = msm_sdcc_vreg_disable(curr_vddp_reg);
		if (rc)
			goto out;
	}
	curr->sts = enable;

out:
	return rc;
}

#define EMMC_ID 1
#define SD_ID 3
static u32 msm_sdcc_setup_power(struct device *dv, unsigned int vdd)
{
	u32 rc_pin_cfg = 0;
	u32 rc_vreg_cfg = 0;
	u32 rc = 0;
	struct platform_device *pdev;
	struct msm_sdcc_pin_cfg *curr_pin_cfg;

	pdev = container_of(dv, struct platform_device, dev);
	if ((pdev->id == EMMC_ID) && (vdd == 0))
		return rc;
	/* SD: Do not setup power in kernel */
	if (pdev->id == SD_ID)
		return rc;
	/* setup gpio/pad */
	curr_pin_cfg = &sdcc_pin_cfg_data[pdev->id - 1];
	if (curr_pin_cfg->cfg_sts == !!vdd)
		goto setup_vreg;

	if (curr_pin_cfg->is_gpio)
		rc_pin_cfg = msm_sdcc_setup_gpio(pdev->id, !!vdd);
	else
		rc_pin_cfg = msm_sdcc_setup_pad(pdev->id, !!vdd);

setup_vreg:
	/* setup voltage regulators */
	rc_vreg_cfg = msm_sdcc_setup_vreg(pdev->id, !!vdd);

	if (rc_pin_cfg || rc_vreg_cfg)
		rc = rc_pin_cfg ? rc_pin_cfg : rc_vreg_cfg;

	return rc;
}

static int msm_sdc3_get_wpswitch(struct device *dev)
{
	struct platform_device *pdev;
	int status;
	pdev = container_of(dev, struct platform_device, dev);

	status = gpio_request(GPIO_SDC3_WP_SWITCH, "SD_WP_Switch");
	if (status) {
		pr_err("%s:Failed to request GPIO %d\n",
					__func__, GPIO_SDC3_WP_SWITCH);
	} else {
		status = gpio_get_value_cansleep(GPIO_SDC3_WP_SWITCH);
		pr_info("%s: WP Status for Slot %d = %d\n", __func__,
							pdev->id, status);
		gpio_free(GPIO_SDC3_WP_SWITCH);
	}
	return (unsigned int) status;
}

#ifdef CONFIG_MMC_MSM_SDC5_SUPPORT
int sdc5_register_status_notify(void (*callback)(int, void *),
	void *dev_id)
{
	sdc5_status_notify_cb = callback;
	sdc5_status_notify_cb_devid = dev_id;
	return 0;
}
#endif

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
int sdc2_register_status_notify(void (*callback)(int, void *),
	void *dev_id)
{
	sdc2_status_notify_cb = callback;
	sdc2_status_notify_cb_devid = dev_id;
	return 0;
}
#endif

/* Interrupt handler for SDC2 and SDC5 detection
 * This function uses dual-edge interrputs settings in order
 * to get SDIO detection when the GPIO is rising and SDIO removal
 * when the GPIO is falling */
static irqreturn_t msm8x60_multi_sdio_slot_status_irq(int irq, void *dev_id)
{
	int status;

	if (!machine_is_verdi_lte())
		return IRQ_NONE;

	status = gpio_get_value(VERDI_LTE_MDM2AP_SYNC);
	pr_info("%s: VERDI_LTE_MDM2AP_SYNC Status = %d\n",
		 __func__, status);

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	if (sdc2_status_notify_cb) {
		pr_info("%s: calling sdc2_status_notify_cb\n", __func__);
		sdc2_status_notify_cb(status,
			sdc2_status_notify_cb_devid);
	}
#endif

#ifdef CONFIG_MMC_MSM_SDC5_SUPPORT
	if (sdc5_status_notify_cb) {
		pr_info("%s: calling sdc5_status_notify_cb\n", __func__);
		sdc5_status_notify_cb(status,
			sdc5_status_notify_cb_devid);
	}
#endif
	return IRQ_HANDLED;
}

static int msm8x60_multi_sdio_init(void)
{
	int ret, irq_num;

	if (!machine_is_verdi_lte())
		return 0;

	ret = msm_gpiomux_get(VERDI_LTE_MDM2AP_SYNC);
	if (ret) {
		pr_err("%s:Failed to request GPIO %d, ret=%d\n",
					__func__, VERDI_LTE_MDM2AP_SYNC, ret);
		return ret;
	}

	irq_num = gpio_to_irq(VERDI_LTE_MDM2AP_SYNC);

	ret = request_irq(irq_num,
		msm8x60_multi_sdio_slot_status_irq,
		IRQ_TYPE_EDGE_BOTH,
		"sdio_multidetection", NULL);

	if (ret) {
		pr_err("%s:Failed to request irq, ret=%d\n",
					__func__, ret);
		return ret;
	}

	return ret;
}

#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
static unsigned int msm8x60_sdcc_slot_status(struct device *dev)
{
	struct platform_device *pdev;
	int status;
	pdev = container_of(dev, struct platform_device, dev);

	status = gpio_request(PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_SDC3_DET - 1)
				, "SD_HW_Detect");
	if (status) {
		pr_err("%s:Failed to request GPIO %d\n", __func__,
				PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_SDC3_DET - 1));
	} else {
		status = !(gpio_get_value_cansleep(
			PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_SDC3_DET - 1)));
		/*pr_info("%s: WP Status for Slot %d = %d\n", __func__,
							pdev->id, status);*/
		gpio_free(PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_SDC3_DET - 1));
	}
	return (unsigned int) status;
}
#endif
#endif
#endif

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
static unsigned int verdi_lte_emmcslot_type = MMC_TYPE_MMC;
static struct mmc_platform_data msm8x60_sdc1_data = {
	.ocr_mask       = MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd  = msm_sdcc_setup_power,
#ifdef CONFIG_MMC_MSM_SDC1_8_BIT_SUPPORT
	.mmc_bus_width  = MMC_CAP_8_BIT_DATA,
#else
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#endif
	.slot_type		= &verdi_lte_emmcslot_type,
	.msmsdcc_fmin	= 400000,
	.msmsdcc_fmid	= 24000000,
	.msmsdcc_fmax	= 48000000,
	.nonremovable	= 1,
	.pclk_src_dfab	= 1,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
static unsigned int verdilte_sdc2_slot_type = MMC_TYPE_SDIO_SVLTE;
static struct mmc_platform_data msm8x60_sdc2_data = {
	.ocr_mask       = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_165_195,
	.translate_vdd  = msm_sdcc_setup_power,
#ifdef CONFIG_MMC_MSM_SDC2_8_BIT_SUPPORT
	.mmc_bus_width  = MMC_CAP_8_BIT_DATA,
#else
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#endif
	.msmsdcc_fmin	= 400000,
	.msmsdcc_fmid	= 24000000,
	.msmsdcc_fmax	= 48000000,
	.nonremovable	= 0,
	.slot_type	= &verdilte_sdc2_slot_type,
	.register_status_notify = sdc2_register_status_notify,
	.pclk_src_dfab	= 1,
	.dummy52_required = 1,
	.is_sdio_al_client = 1,
	.trigger_mdm_fatal = trigger_mdm_fatal,
	.get_mdm2ap_status = get_mdm2ap_status,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
static unsigned int verdi_lte_sdslot_type = MMC_TYPE_SD;
static struct mmc_platform_data msm8x60_sdc3_data = {
	.ocr_mask       = MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd  = msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.wpswitch  	= msm_sdc3_get_wpswitch,
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
	.status      = msm8x60_sdcc_slot_status,
	.status_irq  = PM8058_GPIO_IRQ(PM8058_IRQ_BASE,
				       PMIC_GPIO_SDC3_DET - 1),
	.irq_flags   = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
#endif
	.slot_type		= &verdi_lte_sdslot_type,
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24000000,
	.msmsdcc_fmax	= 48000000,
	.nonremovable	= 0,
	.pclk_src_dfab	= 1,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
#if 0
static struct mmc_platform_data msm8x60_sdc4_data = {
	.ocr_mask       = MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd  = msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.msmsdcc_fmin	= 400000,
	.msmsdcc_fmid	= 24000000,
	.msmsdcc_fmax	= 48000000,
	.nonremovable	= 1,
};
#endif
#endif

#ifdef CONFIG_MMC_MSM_SDC5_SUPPORT
static unsigned int verdilte_sdc5_slot_type = MMC_TYPE_SDIO_SVLTE;
static struct mmc_platform_data msm8x60_sdc5_data = {
	.ocr_mask       = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_165_195,
	.translate_vdd  = msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.msmsdcc_fmin	= 400000,
	.msmsdcc_fmid	= 24000000,
	.msmsdcc_fmax	= 48000000,
	.nonremovable	= 0,
	.slot_type	= &verdilte_sdc5_slot_type,
	.register_status_notify = sdc5_register_status_notify,
	.pclk_src_dfab	= 1,
	.dummy52_required = 1,
	.is_sdio_al_client = 1,
	.trigger_mdm_fatal = trigger_mdm_fatal,
	.get_mdm2ap_status = get_mdm2ap_status,
};
#endif

static void __init msm8x60_init_mmc(void)
{
	int ret = 0;
#ifdef CONFIG_MSM_SDIO_AL
	config_gpio_table(mdm2ap_gpio_table,
			  ARRAY_SIZE(mdm2ap_gpio_table));
#endif
#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
	config_gpio_table(sdc1_on_gpio_table, ARRAY_SIZE(sdc1_on_gpio_table));
	/* SDCC1 : eMMC card connected */
	sdcc_vreg_data[0].vdd_data = &sdcc_vdd_reg_data[0];
	sdcc_vreg_data[0].vdd_data->reg_name = "8901_l5";
	sdcc_vreg_data[0].vdd_data->set_voltage_sup = 1;
	sdcc_vreg_data[0].vdd_data->level = 2850000;
	sdcc_vreg_data[0].vccq_data = &sdcc_vccq_reg_data[0];
	sdcc_vreg_data[0].vccq_data->reg_name = "8901_lvs0";
	sdcc_vreg_data[0].vccq_data->set_voltage_sup = 0;
	msm_add_sdcc(1, &msm8x60_sdc1_data);
#endif
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	/*
	 * MDM SDIO client is connected to SDC2 on Verdilte
	 */
	sdcc_vreg_data[1].vdd_data = &sdcc_vdd_reg_data[1];
	sdcc_vreg_data[1].vdd_data->reg_name = "8058_s3";
	sdcc_vreg_data[1].vdd_data->set_voltage_sup = 1;
	sdcc_vreg_data[1].vdd_data->level = 1800000;
	sdcc_vreg_data[1].vccq_data = NULL;
	msm8x60_sdc2_data.msmsdcc_fmax = 24000000;
	msm8x60_sdc2_data.sdiowakeup_irq = gpio_to_irq(144);
	msm_sdcc_setup_gpio(2, 1);
	msm_add_sdcc(2, &msm8x60_sdc2_data);
#endif
#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
	/* SDCC3 : External card slot connected */
	sdcc_vreg_data[2].vdd_data = &sdcc_vdd_reg_data[2];
	sdcc_vreg_data[2].vdd_data->reg_name = "8058_l14";
	sdcc_vreg_data[2].vdd_data->set_voltage_sup = 1;
	sdcc_vreg_data[2].vdd_data->level = 2850000;
	sdcc_vreg_data[2].vccq_data = NULL;
	msm_add_sdcc(3, &msm8x60_sdc3_data);
	msm_sdcc_setup_pad(3, 1);
#endif
#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
	/* SDCC4 : WLAN WCN1314 chip is connected */
/*	sdcc_vreg_data[3].vdd_data = &sdcc_vdd_reg_data[3];
	sdcc_vreg_data[3].vdd_data->reg_name = "8058_s3";
	sdcc_vreg_data[3].vdd_data->set_voltage_sup = 1;
	sdcc_vreg_data[3].vdd_data->level = 1800000;
	sdcc_vreg_data[3].vccq_data = NULL;
	msm_add_sdcc(4, &msm8x60_sdc4_data);*/
	ret = verdi_lte_init_mmc();
	if (ret != 0)
		pr_crit("%s: Unable to initialize MMC (SDCC4)\n", __func__);
#endif
#ifdef CONFIG_MMC_MSM_SDC5_SUPPORT
	/*
	 * MDM SDIO client is connected to SDC5 on Verdilte
	 */
	sdcc_vreg_data[4].vdd_data = &sdcc_vdd_reg_data[4];
	sdcc_vreg_data[4].vdd_data->reg_name = "8058_s3";
	sdcc_vreg_data[4].vdd_data->set_voltage_sup = 1;
	sdcc_vreg_data[4].vdd_data->level = 1800000;
	sdcc_vreg_data[4].vccq_data = NULL;
	msm8x60_sdc5_data.msmsdcc_fmax = 24000000;
	msm8x60_sdc5_data.sdiowakeup_irq = gpio_to_irq(99);
	msm_sdcc_setup_gpio(5, 1);
	msm_add_sdcc(5, &msm8x60_sdc5_data);
#endif
}

static uint32_t lcd_panel_gpios[] = {
	GPIO_CFG(0,  1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_10MA), /* lcdc_pclk */
	GPIO_CFG(1,  1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_10MA), /* lcdc_hsync*/
	GPIO_CFG(2,  1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_10MA), /* lcdc_vsync*/
	GPIO_CFG(3,  1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_10MA), /* lcdc_den */
	GPIO_CFG(4,  1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_10MA), /* lcdc_red7 */
	GPIO_CFG(5,  1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_10MA), /* lcdc_red6 */
	GPIO_CFG(6,  1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_10MA), /* lcdc_red5 */
	GPIO_CFG(7,  1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_10MA), /* lcdc_red4 */
	GPIO_CFG(8,  1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_10MA), /* lcdc_red3 */
	GPIO_CFG(9,  1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_10MA), /* lcdc_red2 */
	GPIO_CFG(10, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_10MA), /* lcdc_red1 */
	GPIO_CFG(11, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_10MA), /* lcdc_red0 */
	GPIO_CFG(12, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_10MA), /* lcdc_grn7 */
	GPIO_CFG(13, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_10MA), /* lcdc_grn6 */
	GPIO_CFG(14, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_10MA), /* lcdc_grn5 */
	GPIO_CFG(15, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_10MA), /* lcdc_grn4 */
	GPIO_CFG(16, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_10MA), /* lcdc_grn3 */
	GPIO_CFG(17, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_10MA), /* lcdc_grn2 */
	GPIO_CFG(18, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_10MA), /* lcdc_grn1 */
	GPIO_CFG(19, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_10MA), /* lcdc_grn0 */
	GPIO_CFG(20, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_10MA), /* lcdc_blu7 */
	GPIO_CFG(21, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_10MA), /* lcdc_blu6 */
	GPIO_CFG(22, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_10MA), /* lcdc_blu5 */
	GPIO_CFG(23, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_10MA), /* lcdc_blu4 */
	GPIO_CFG(24, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_10MA), /* lcdc_blu3 */
	GPIO_CFG(25, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_10MA), /* lcdc_blu2 */
	GPIO_CFG(26, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_10MA), /* lcdc_blu1 */
	GPIO_CFG(27, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_10MA), /* lcdc_blu0 */
};

static uint32_t lcd_panel_gpios_sleep_mode[] = {
	GPIO_CFG(0,  0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), /* lcdc_pclk */
	GPIO_CFG(1,  0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), /* lcdc_hsync*/
	GPIO_CFG(2,  0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), /* lcdc_vsync*/
	GPIO_CFG(3,  0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), /* lcdc_den */
	GPIO_CFG(4,  0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), /* lcdc_red7 */
	GPIO_CFG(5,  0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), /* lcdc_red6 */
	GPIO_CFG(6,  0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), /* lcdc_red5 */
	GPIO_CFG(7,  0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), /* lcdc_red4 */
	GPIO_CFG(8,  0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), /* lcdc_red3 */
	GPIO_CFG(9,  0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), /* lcdc_red2 */
	GPIO_CFG(10, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), /* lcdc_red1 */
	GPIO_CFG(11, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), /* lcdc_red0 */
	GPIO_CFG(12, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), /* lcdc_grn7 */
	GPIO_CFG(13, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), /* lcdc_grn6 */
	GPIO_CFG(14, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), /* lcdc_grn5 */
	GPIO_CFG(15, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), /* lcdc_grn4 */
	GPIO_CFG(16, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), /* lcdc_grn3 */
	GPIO_CFG(17, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), /* lcdc_grn2 */
	GPIO_CFG(18, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), /* lcdc_grn1 */
	GPIO_CFG(19, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), /* lcdc_grn0 */
	GPIO_CFG(20, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), /* lcdc_blu7 */
	GPIO_CFG(21, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), /* lcdc_blu6 */
	GPIO_CFG(22, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), /* lcdc_blu5 */
	GPIO_CFG(23, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), /* lcdc_blu4 */
	GPIO_CFG(24, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), /* lcdc_blu3 */
	GPIO_CFG(25, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), /* lcdc_blu2 */
	GPIO_CFG(26, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), /* lcdc_blu1 */
	GPIO_CFG(27, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_10MA), /* lcdc_blu0 */
};

static void lcdc_samsung_panel_power(int on)
{
	int n, rc;
	static int init;

	pr_info("%s(%d): init=%d\n", __func__, on, init);
	/* If panel is already on (or off), do nothing. */
	if (!init) {
		if (system_rev == 0)
		{
			/* LCM Reset */
			rc = gpio_request(VERDI_LTE_LVDS_ON,
				"VERDI_LTE_LVDS_ON");
			if (rc) {
				printk(KERN_ERR "%s:LCM gpio %d request"
					"failed\n", __func__,
					 VERDI_LTE_LVDS_ON);
				return;
			}

			gpio_direction_output(VERDI_LTE_LVDS_ON, 0);
		}
		else
		{
			/* LCM Reset */
			rc = gpio_request(VERDI_LTE_LVDS_ON_XB,
				"VERDI_LTE_LVDS_ON_XB");
			if (rc) {
				printk(KERN_ERR "%s:LCM gpio %d request"
					"failed\n", __func__,
					 VERDI_LTE_LVDS_ON_XB);
				return;
			}

			gpio_direction_output(VERDI_LTE_LVDS_ON_XB, 0);
		}

		init = 1;
	}
	if (on) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_LCM_EN), 1);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_V_LCM_3V3_EN), 1);
		if (system_rev == 0)
			gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_V_LCM_3V3_SYNC), 1);
		mdelay(50);
		if (system_rev == 0)
			gpio_set_value(VERDI_LTE_LVDS_ON, 1);
		else
			gpio_set_value(VERDI_LTE_LVDS_ON_XB, 1);
		/*TODO if on = 0 free the gpio's */
		for (n = 0; n < ARRAY_SIZE(lcd_panel_gpios); ++n)
			gpio_tlmm_config(lcd_panel_gpios[n], GPIO_CFG_ENABLE);
	} else {
		for (n = 0; n < ARRAY_SIZE(lcd_panel_gpios); ++n)
			gpio_tlmm_config(lcd_panel_gpios[n], GPIO_CFG_DISABLE);
		if (system_rev == 0)
			gpio_set_value(VERDI_LTE_LVDS_ON, 0);
		else
			gpio_set_value(VERDI_LTE_LVDS_ON_XB, 0);
		mdelay(1);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_V_LCM_3V3_EN), 0);
		if (system_rev == 0)
			gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_V_LCM_3V3_SYNC), 0);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_LCM_EN), 0);
	}
}

struct dcr_reg {
	uint32_t addr;
	uint32_t reg;
	uint32_t val;
};

struct dcr_reg dcr_init_lut_seq[] = {
	{0x7E, 0x120, 0  },
	{0x7E, 0x121, 2  },
	{0x7E, 0x122, 4  },
	{0x7E, 0x123, 6  },
	{0x7E, 0x124, 8  },
	{0x7E, 0x125, 10 },
	{0x7E, 0x126, 12 },
	{0x7E, 0x127, 14 },
	{0x7E, 0x128, 16 },
	{0x7E, 0x129, 18 },
	{0x7E, 0x12A, 20 },
	{0x7E, 0x12B, 22 },
	{0x7E, 0x12C, 24 },
	{0x7E, 0x12D, 26 },
	{0x7E, 0x12E, 28 },
	{0x7E, 0x12F, 30 },
	{0x7E, 0x130, 32 },
	{0x7E, 0x131, 34 },
	{0x7E, 0x132, 36 },
	{0x7E, 0x133, 38 },
	{0x7E, 0x134, 40 },
	{0x7E, 0x135, 42 },
	{0x7E, 0x136, 44 },
	{0x7E, 0x137, 46 },
	{0x7E, 0x138, 48 },
	{0x7E, 0x139, 50 },
	{0x7E, 0x13A, 52 },
	{0x7E, 0x13B, 54 },
	{0x7E, 0x13C, 56 },
	{0x7E, 0x13D, 58 },
	{0x7E, 0x13E, 60 },
	{0x7E, 0x13F, 62 },
	{0x7E, 0x140, 64 },
	{0x7E, 0x141, 66 },
	{0x7E, 0x142, 68 },
	{0x7E, 0x143, 70 },
	{0x7E, 0x144, 72 },
	{0x7E, 0x145, 74 },
	{0x7E, 0x146, 76 },
	{0x7E, 0x147, 78 },
	{0x7E, 0x148, 80 },
	{0x7E, 0x149, 82 },
	{0x7E, 0x14A, 84 },
	{0x7E, 0x14B, 86 },
	{0x7E, 0x14C, 88 },
	{0x7E, 0x14D, 90 },
	{0x7E, 0x14E, 92 },
	{0x7E, 0x14F, 94 },
	{0x7E, 0x150, 96 },
	{0x7E, 0x151, 98 },
	{0x7E, 0x152, 100},
	{0x7E, 0x153, 102},
	{0x7E, 0x154, 104},
	{0x7E, 0x155, 106},
	{0x7E, 0x156, 108},
	{0x7E, 0x157, 110},
	{0x7E, 0x158, 112},
	{0x7E, 0x159, 114},
	{0x7E, 0x15A, 116},
	{0x7E, 0x15B, 118},
	{0x7E, 0x15C, 120},
	{0x7E, 0x15D, 122},
	{0x7E, 0x15E, 124},
	{0x7E, 0x15F, 126},
	{0x7E, 0x160, 129},
	{0x7E, 0x161, 131},
	{0x7E, 0x162, 133},
	{0x7E, 0x163, 135},
	{0x7E, 0x164, 137},
	{0x7E, 0x165, 139},
	{0x7E, 0x166, 141},
	{0x7E, 0x167, 143},
	{0x7E, 0x168, 145},
	{0x7E, 0x169, 147},
	{0x7E, 0x16A, 149},
	{0x7E, 0x16B, 151},
	{0x7E, 0x16C, 153},
	{0x7E, 0x16D, 155},
	{0x7E, 0x16E, 157},
	{0x7E, 0x16F, 159},
	{0x7E, 0x170, 161},
	{0x7E, 0x171, 163},
	{0x7E, 0x172, 165},
	{0x7E, 0x173, 167},
	{0x7E, 0x174, 169},
	{0x7E, 0x175, 171},
	{0x7E, 0x176, 173},
	{0x7E, 0x177, 175},
	{0x7E, 0x178, 177},
	{0x7E, 0x179, 179},
	{0x7E, 0x17A, 181},
	{0x7E, 0x17B, 183},
	{0x7E, 0x17C, 185},
	{0x7E, 0x17D, 187},
	{0x7E, 0x17E, 189},
	{0x7E, 0x17F, 191},
	{0x7E, 0x180, 193},
	{0x7E, 0x181, 195},
	{0x7E, 0x182, 197},
	{0x7E, 0x183, 199},
	{0x7E, 0x184, 201},
	{0x7E, 0x185, 203},
	{0x7E, 0x186, 205},
	{0x7E, 0x187, 207},
	{0x7E, 0x188, 209},
	{0x7E, 0x189, 211},
	{0x7E, 0x18A, 213},
	{0x7E, 0x18B, 215},
	{0x7E, 0x18C, 217},
	{0x7E, 0x18D, 219},
	{0x7E, 0x18E, 221},
	{0x7E, 0x18F, 223},
	{0x7E, 0x190, 225},
	{0x7E, 0x191, 227},
	{0x7E, 0x192, 229},
	{0x7E, 0x193, 231},
	{0x7E, 0x194, 233},
	{0x7E, 0x195, 235},
	{0x7E, 0x196, 237},
	{0x7E, 0x197, 239},
	{0x7E, 0x198, 241},
	{0x7E, 0x199, 243},
	{0x7E, 0x19A, 245},
	{0x7E, 0x19B, 247},
	{0x7E, 0x19C, 249},
	{0x7E, 0x19D, 251},
	{0x7E, 0x19E, 253},
	{0x7E, 0x19F, 255},
	{0x7E, 0x1A0, 38 },
	{0x7E, 0x1A1, 147},
	{0x7E, 0x1A2, 46 },
	{0x7E, 0x1A3, 183},
	{0x7E, 0x1A4, 178},
	{0x7E, 0x1A5, 41 },
	{0x7E, 0x1A6, 136},
	{0x7E, 0x1A7, 162},
	{0x7E, 0x1A8, 39 },
	{0x7E, 0x1A9, 111},
	{0x7E, 0x1AA, 82 },
	{0x7E, 0x1AB, 38 },
	{0x7E, 0x1AC, 93 },
	{0x7E, 0x1AD, 98 },
	{0x7E, 0x1AE, 37 },
	{0x7E, 0x1AF, 80 },
	{0x7E, 0x1B0, 162},
	{0x7E, 0x1B1, 36 },
	{0x7E, 0x1B2, 69 },
	{0x7E, 0x1B3, 2  },
	{0x7E, 0x1B4, 36 },
	{0x7E, 0x1B5, 60 },
	{0x7E, 0x1B6, 130},
	{0x7E, 0x1B7, 35 },
	{0x7E, 0x1B8, 53 },
	{0x7E, 0x1B9, 18 },
	{0x7E, 0x1BA, 35 },
	{0x7E, 0x1BB, 46 },
	{0x7E, 0x1BC, 178},
	{0x7E, 0x1BD, 34 },
	{0x7E, 0x1BE, 40 },
	{0x7E, 0x1BF, 82 },
	{0x7E, 0x1C0, 34 },
	{0x7E, 0x1C1, 35 },
	{0x7E, 0x1C2, 2  },
	{0x7E, 0x1C3, 34 },
	{0x7E, 0x1C4, 30 },
	{0x7E, 0x1C5, 194},
	{0x7E, 0x1C6, 33 },
	{0x7E, 0x1C7, 26 },
	{0x7E, 0x1C8, 130},
	{0x7E, 0x1C9, 33 },
	{0x7E, 0x1CA, 22 },
	{0x7E, 0x1CB, 66 },
	{0x7E, 0x1CC, 33 },
	{0x7E, 0x1CD, 20 },
	{0x7E, 0x1CE, 50 },
	{0x7E, 0x1CF, 33 },
	{0x7E, 0x1D0, 19 },
	{0x7E, 0x1D1, 50 },
	{0x7E, 0x1D2, 33 },
	{0x7E, 0x1D3, 19 },
	{0x7E, 0x1D4, 34 },
	{0x7E, 0x1D5, 33 },
	{0x7E, 0x1D6, 18 },
	{0x7E, 0x1D7, 34 },
	{0x7E, 0x1D8, 33 },
	{0x7E, 0x1D9, 18 },
	{0x7E, 0x1DA, 18 },
	{0x7E, 0x1DB, 33 },
	{0x7E, 0x1DC, 17 },
	{0x7E, 0x1DD, 18 },
	{0x7E, 0x1DE, 33 },
	{0x7E, 0x1DF, 17 },
	{0x7E, 0x1E0, 18 },
	{0x7E, 0x1E1, 33 },
	{0x7E, 0x1E2, 16 },
	{0x7E, 0x1E3, 2  },
	{0x7E, 0x1E4, 33 },
	{0x7E, 0x1E5, 16 },
	{0x7E, 0x1E6, 2  },
	{0x7E, 0x1E7, 33 },
	{0x7E, 0x1E8, 15 },
	{0x7E, 0x1E9, 242},
	{0x7E, 0x1EA, 32 },
	{0x7E, 0x1EB, 15 },
	{0x7E, 0x1EC, 242},
	{0x7E, 0x1ED, 32 },
	{0x7E, 0x1EE, 14 },
	{0x7E, 0x1EF, 226},
	{0x7E, 0x1F0, 32 },
	{0x7E, 0x1F1, 14 },
	{0x7E, 0x1F2, 226},
	{0x7E, 0x1F3, 32 },
	{0x7E, 0x1F4, 14 },
	{0x7E, 0x1F5, 210},
	{0x7E, 0x1F6, 32 },
	{0x7E, 0x1F7, 13 },
	{0x7E, 0x1F8, 210},
	{0x7E, 0x1F9, 32 },
	{0x7E, 0x1FA, 13 },
	{0x7E, 0x1FB, 194},
	{0x7E, 0x1FC, 32 },
	{0x7E, 0x1FD, 12 },
	{0x7E, 0x1FE, 194},
	{0x7E, 0x1FF, 32 },
	{0x7E, 0x200, 12 },
	{0x7E, 0x201, 194},
	{0x7E, 0x202, 32 },
	{0x7E, 0x203, 11 },
	{0x7E, 0x204, 178},
	{0x7E, 0x205, 32 },
	{0x7E, 0x206, 11 },
	{0x7E, 0x207, 178},
	{0x7E, 0x208, 32 },
	{0x7E, 0x209, 11 },
	{0x7E, 0x20A, 162},
	{0x7E, 0x20B, 32 },
	{0x7E, 0x20C, 10 },
	{0x7E, 0x20D, 162},
	{0x7E, 0x20E, 32 },
	{0x7E, 0x20F, 10 },
	{0x7E, 0x210, 162},
	{0x7E, 0x211, 32 },
	{0x7E, 0x212, 9  },
	{0x7E, 0x213, 146},
	{0x7E, 0x214, 32 },
	{0x7E, 0x215, 9  },
	{0x7E, 0x216, 146},
	{0x7E, 0x217, 32 },
	{0x7E, 0x218, 9  },
	{0x7E, 0x219, 130},
	{0x7E, 0x21A, 32 },
	{0x7E, 0x21B, 8  },
	{0x7E, 0x21C, 130},
	{0x7E, 0x21D, 32 },
	{0x7E, 0x21E, 8  },
	{0x7E, 0x21F, 130},
	{0x7E, 0x220, 32 },
	{0x7E, 0x221, 7  },
	{0x7E, 0x222, 114},
	{0x7E, 0x223, 32 },
	{0x7E, 0x224, 7  },
	{0x7E, 0x225, 114},
	{0x7E, 0x226, 32 },
	{0x7E, 0x227, 7  },
	{0x7E, 0x228, 98 },
	{0x7E, 0x229, 32 },
	{0x7E, 0x22A, 6  },
	{0x7E, 0x22B, 98 },
	{0x7E, 0x22C, 32 },
	{0x7E, 0x22D, 6  },
	{0x7E, 0x22E, 98 },
	{0x7E, 0x22F, 32 },
	{0x7E, 0x230, 5  },
	{0x7E, 0x231, 82 },
	{0x7E, 0x232, 32 },
	{0x7E, 0x233, 5  },
	{0x7E, 0x234, 82 },
	{0x7E, 0x235, 32 },
	{0x7E, 0x236, 5  },
	{0x7E, 0x237, 82 },
	{0x7E, 0x238, 32 },
	{0x7E, 0x239, 4  },
	{0x7E, 0x23A, 66 },
	{0x7E, 0x23B, 32 },
	{0x7E, 0x23C, 4  },
	{0x7E, 0x23D, 66 },
	{0x7E, 0x23E, 32 },
	{0x7E, 0x23F, 4  },
	{0x7E, 0x240, 50 },
	{0x7E, 0x241, 32 },
	{0x7E, 0x242, 3  },
	{0x7E, 0x243, 50 },
	{0x7E, 0x244, 32 },
	{0x7E, 0x245, 3  },
	{0x7E, 0x246, 50 },
	{0x7E, 0x247, 32 },
	{0x7E, 0x248, 3  },
	{0x7E, 0x249, 34 },
	{0x7E, 0x24A, 32 },
	{0x7E, 0x24B, 2  },
	{0x7E, 0x24C, 34 },
	{0x7E, 0x24D, 32 },
	{0x7E, 0x24E, 2  },
	{0x7E, 0x24F, 34 },
	{0x7E, 0x250, 32 },
	{0x7E, 0x251, 2  },
	{0x7E, 0x252, 18 },
	{0x7E, 0x253, 32 },
	{0x7E, 0x254, 1  },
	{0x7E, 0x255, 18 },
	{0x7E, 0x256, 32 },
	{0x7E, 0x257, 1  },
	{0x7E, 0x258, 18 },
	{0x7E, 0x259, 32 },
	{0x7E, 0x25A, 1  },
	{0x7E, 0x25B, 2  },
	{0x7E, 0x25C, 32 },
	{0x7E, 0x25D, 0  },
	{0x7E, 0x25E, 2  },
	{0x7E, 0x25F, 32 },
};

struct dcr_reg dcr_enable_bl_smooth_seq[] = {
	{0x7E, 0x26B, 2  },
	{0x7E, 0x263, 2  },
	{0x7E, 0x264, 1 },
};

struct dcr_reg dcr_disable_bl_smooth_seq[] = {
	{0x7E, 0x26B, 0  },
};

struct dcr_reg dcr_default_mode_seq[] = {
	{0x7E, 0x36, 0  },
	{0x7E, 0x26D, 114  },
	{0x7E, 0x37, 255 },
	{0x7E, 0x2B, 242  },
	{0x7E, 0xC8, 6  },
};

struct dcr_reg dcr_video_mode_seq[] = {
	{0x7E, 0x36, 0  },
	{0x7E, 0x26D, 87  },
	{0x7E, 0x37, 255 },
	{0x7E, 0x2B, 242  },
	{0x7E, 0xC8, 20  },
};

long dcr_i2c_read_1byte(long dev, long addr)
{
	struct i2c_adapter *adap = i2c_get_adapter(MSM_GSBI10_QUP_I2C_BUS_ID);
	struct i2c_msg msg;
	char buf[1];//read 1 byte
	char data=0x1;
	int err;
	if( board_mfg_mode() == 4 ) {
		return 0;
	}
	buf[0] = addr & 0xff;
	msg.addr = dev >> 1;    // I2C address of chip
	msg.flags = 0;
	msg.len = sizeof(buf);
	msg.buf = buf;
	err = i2c_transfer(adap, &msg, 1);

	msg.addr = dev >> 1;    // I2C address
	msg.flags = I2C_M_RD;
	msg.len = sizeof(data);
	msg.buf = &data;
	err = i2c_transfer(adap, &msg, 1);

	i2c_put_adapter(adap);

	PR_DISP_INFO("%s: i2c read: addr=0x%lx, value=0x%x (5V -> 0xC2 ; 4V -> others)\n", __func__, addr, data);
	return data;
}

long dcr_i2c_read_2byte(long dev, long addr)
{
	struct i2c_adapter *adap = i2c_get_adapter(MSM_GSBI10_QUP_I2C_BUS_ID);
	struct i2c_msg msg;
	char buf[2]; // read 2 bytes
	char data=0xFE;
	int err;
	if( board_mfg_mode() == 4 ) {
		return 0;
	}
	buf[0] = (addr >> 8) & 0xff;
	buf[1] = addr & 0xff;
	msg.addr = dev >> 1;    // I2C address of chip
	msg.flags = 0;
	msg.len = sizeof(buf);
	msg.buf = buf;
	err = i2c_transfer(adap, &msg, 1);

	msg.addr = dev >> 1;    // I2C address
	msg.flags = I2C_M_RD;
	msg.len = sizeof(data);
	msg.buf = &data;
	err = i2c_transfer(adap, &msg, 1);

	i2c_put_adapter(adap);

	PR_DISP_INFO("%s: i2c read: addr=0x%lx, value=0x%x (5V->C2; 4V->others)\n", __func__, addr, data);
	return data;
}

void dcr_i2c_write(long dev, long addr, long val)
{
	struct i2c_adapter *adap = i2c_get_adapter(MSM_GSBI10_QUP_I2C_BUS_ID);
	struct i2c_msg msg;
	char buf[3];
	int ret = -EIO, max_retry = 5;
	if( board_mfg_mode() == 4 ) {
		return;
	}
	buf[0] = (addr >> 8) & 0xff;
	buf[1] = addr & 0xff;
	buf[2] = val & 0xff;

	msg.addr = dev >> 1;
	msg.flags = 0;
	msg.len = sizeof(buf);
	msg.buf = buf;

	while (max_retry--) {
		ret = i2c_transfer(adap, &msg, 1);
		if (ret != 1)
			hr_msleep(1);
		else {
			ret = 0;
			break;
		}
		ret = -EIO;
	}
	if (ret)
		PR_DISP_INFO("dcr_i2c_Write failed, addr=0x%lx\n", addr);
}

void dcr_video_mode(bool on)
{
	int n=0;

	if( board_mfg_mode() == 4 ) {
		return;
	}
	if(mdp_pdata.dcr_panel_pinfo) {
		if(on==0)
			(mdp_pdata.dcr_panel_pinfo)->auto_bkl_stat = 1; //default mode
		else if(on == 1)
			(mdp_pdata.dcr_panel_pinfo)->auto_bkl_stat = 2; //video mode
	}

	PR_DISP_INFO("%s: on(%d) auto_bkl_stat(%ld)\n", __func__, on, (mdp_pdata.dcr_panel_pinfo)->auto_bkl_stat);

	//down(&dcr_lock);
	if(on) {
		for (n = 0; n < ARRAY_SIZE(dcr_video_mode_seq); ++n)
			dcr_i2c_write(dcr_video_mode_seq[n].addr, dcr_video_mode_seq[n].reg, dcr_video_mode_seq[n].val);
	} else {
		for (n = 0; n < ARRAY_SIZE(dcr_default_mode_seq); ++n)
			dcr_i2c_write(dcr_default_mode_seq[n].addr, dcr_default_mode_seq[n].reg, dcr_default_mode_seq[n].val);
	}
	//up(&dcr_lock);
}

void dcr_power(bool on)
{
	if( board_mfg_mode() == 4 ) {
		return;
	}
	if(on) {
		if(system_rev >= 1)
			gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_LCM_DCR_EN_XB), 1);
		else
			gpio_set_value(GPIO_LCM_DCR, 1);
	} else {
		if(system_rev >= 1)
			gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_LCM_DCR_EN_XB), 0);
		else
			gpio_set_value(GPIO_LCM_DCR, 0);
		if(mdp_pdata.dcr_panel_pinfo) {
			(mdp_pdata.dcr_panel_pinfo)->auto_bkl_stat = 0; //off mode
		}
	}
	if(system_rev >= 1)
		PR_DISP_INFO("%s: on(%d) auto_bkl_stat(%ld) gpio_dcr(%d)\n", __func__, on, (mdp_pdata.dcr_panel_pinfo)->auto_bkl_stat, gpio_get_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_LCM_DCR_EN_XB)));
	else
		PR_DISP_INFO("%s: on(%d) auto_bkl_stat(%ld) gpio_dcr(%d)\n", __func__, on, (mdp_pdata.dcr_panel_pinfo)->auto_bkl_stat, gpio_get_value(GPIO_LCM_DCR));
}

static bool bkl_smooth_on;
void bkl_smooth(bool on)
{
	int n=0;
	static bool bkl_pre_smooth_state = 0;
	if( board_mfg_mode() == 4 ) {
		return;
	}
	if( bkl_pre_smooth_state == on )
		return;
	PR_DISP_INFO("%s: on(%d)\n", __func__, on);
	bkl_smooth_on = bkl_pre_smooth_state = on;
	//down(&dcr_lock);
	if(on) {
		for (n = 0; n < ARRAY_SIZE(dcr_enable_bl_smooth_seq); ++n)
			dcr_i2c_write(dcr_enable_bl_smooth_seq[n].addr, dcr_enable_bl_smooth_seq[n].reg, dcr_enable_bl_smooth_seq[n].val);
	} else {
		for (n = 0; n < ARRAY_SIZE(dcr_disable_bl_smooth_seq); ++n)
			dcr_i2c_write(dcr_disable_bl_smooth_seq[n].addr, dcr_disable_bl_smooth_seq[n].reg, dcr_disable_bl_smooth_seq[n].val);
	}
	//up(&dcr_lock);
}

bool get_bkl_smooth_status(void)
{
	return bkl_smooth_on;
}

static int lut_table(void)
{
	int n=0;
	PR_DISP_INFO("%s\n", __func__);
	if( board_mfg_mode() == 4 ) {
		return 0;
	}
	//down(&dcr_lock);
	for (n = 0; n < ARRAY_SIZE(dcr_init_lut_seq); ++n)
		dcr_i2c_write(dcr_init_lut_seq[n].addr, dcr_init_lut_seq[n].reg, dcr_init_lut_seq[n].val);
	//up(&dcr_lock);

	return 0;
}

struct panel_dcr_info dcr_panel = {
	.lut_table = lut_table,
	.dcr_video_mode = dcr_video_mode,
	.dcr_power = dcr_power,
	.bkl_smooth = bkl_smooth,
	.get_bkl_smooth_status = get_bkl_smooth_status,
	.video_mode = ATOMIC_INIT(0),
	.auto_bkl_stat	= 0,
};

void vee_i2c_write(long addr, long val)
{
	struct i2c_adapter *adap = i2c_get_adapter(MSM_GSBI10_QUP_I2C_BUS_ID);
	struct i2c_msg msg;
	u8 buf[8];
	int ret = -EIO, max_retry = 5;

	if( board_mfg_mode() == 4 ) {
		return;
	}
	buf[0] = addr & 0xff;
	buf[1] = (addr>>8) & 0xff;
	buf[2] = (addr>>16) & 0xff;
	buf[3] = (addr>>24) & 0xff;
	buf[4] = val & 0xff;
	buf[5] = (val>>8) & 0xff;
	buf[6] = (val>>16) & 0xff;
	buf[7] = (val>>24) & 0xff;

	msg.addr = 0x54 >> 1;
	msg.flags = 0;
	msg.len = sizeof(buf);
	msg.buf = buf;

	while (max_retry--) {
		ret = i2c_transfer(adap, &msg, 1);
		if (ret != 1)
			hr_msleep(1);
		else {
			ret = 0;
			break;
		}
		ret = -EIO;
	}
	if (ret)
		PR_DISP_INFO("vee_i2c_write fail: addr=0x%lx\n", addr);
}

void init_vee_chip(void)
{
	//PR_DISP_INFO("%s\n", __func__);

	vee_i2c_write(0x210004, 0x06);
	//bypass mode
	vee_i2c_write(0x30020, 0x1FC2);
	vee_i2c_write(0x20000, 0x0);
}

static void lcdc_auo_panel_power(int on)
{
	int n=0, rc;
	static int init = 0;

	PR_DISP_INFO("%s(%d): init=%d\n", __func__, on, init);
	/* If panel is already on (or off), do nothing. */
	if (!init) {
		if (system_rev == 0)
		{
			/* LCM Reset */
			rc = gpio_request(VERDI_LTE_LVDS_ON,
				"VERDI_LTE_LVDS_ON");
			if (rc) {
				printk(KERN_ERR "%s:LCM gpio %d request"
					"failed\n", __func__,
					VERDI_LTE_LVDS_ON);
				return;
			}

			gpio_direction_output(VERDI_LTE_LVDS_ON, 0);
		}
		else
		{
			/* LCM Reset */
			rc = gpio_request(VERDI_LTE_LVDS_ON_XB,
				"VERDI_LTE_LVDS_ON_XB");
			if (rc) {
				printk(KERN_ERR "%s:LCM gpio %d request"
					"failed\n", __func__,
					VERDI_LTE_LVDS_ON_XB);
				return;
			}

			gpio_direction_output(VERDI_LTE_LVDS_ON_XB, 0);
		}
	}

	if (on) {
		if(system_rev < 2) {
			gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_V_LCM_3V3_EN), 1);
			hr_msleep(2);
		}
		else
			gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_V_LCM_3V3_EN_1_XC), 1);

		if (system_rev == 0)
			gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_V_LCM_3V3_SYNC), 1);

		hr_msleep(1);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_LCM_EN), 1);
		hr_msleep(1);
		if (system_rev == 0)
			gpio_set_value(VERDI_LTE_LVDS_ON, 1);
		else
			gpio_set_value(VERDI_LTE_LVDS_ON_XB, 1);

		/*TODO if on = 0 free the gpio's */
		for (n = 0; n < ARRAY_SIZE(lcd_panel_gpios); ++n)
			gpio_tlmm_config(lcd_panel_gpios[n], GPIO_CFG_ENABLE);

		hr_msleep(200);

		if (system_rev == 2)//XC
			init_vee_chip();

		if(b5v_lc == 0)
			lut_table();

		if(init) {
			if(mdp_pdata.dcr_panel_pinfo && (mdp_pdata.dcr_panel_pinfo)->auto_bkl_stat != 0) {
				if( (mdp_pdata.dcr_panel_pinfo)->auto_bkl_stat == 1)
					dcr_video_mode(0);
				else if ( (mdp_pdata.dcr_panel_pinfo)->auto_bkl_stat == 2 )
					dcr_video_mode(1);

				dcr_power(1);
			}
		}
	} else {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_LCM_BL_EN), 0);
		hr_msleep(10);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_LCM_BL_PWM), 0);
		hr_msleep(10);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_V_LED_EN), 0);

		for (n = 0; n < ARRAY_SIZE(lcd_panel_gpios_sleep_mode); ++n)
			gpio_tlmm_config(lcd_panel_gpios_sleep_mode[n], GPIO_CFG_DISABLE);

		hr_msleep(1);
		if (system_rev == 0)
			gpio_set_value(VERDI_LTE_LVDS_ON, 0);
		else
			gpio_set_value(VERDI_LTE_LVDS_ON_XB, 0);
		hr_msleep(1);

		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_LCM_EN), 0);

		if(system_rev >= 1)
			gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_LCM_DCR_EN_XB), 0);
		else
			gpio_set_value(GPIO_LCM_DCR, 0);

		if(system_rev >= 2)
			gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_V_LCM_3V3_EN_1_XC), 0);
		else
			gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_V_LCM_3V3_EN), 0);

		if (system_rev == 0)
			gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_V_LCM_3V3_SYNC), 0);
		hr_msleep(300);
	}

	if(!init)
		init = 1;
}

#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
static int hdmi_enable_5v(int on)
{
	static struct regulator *reg_8901_hdmi_mvs;	/* HDMI_5V */
	static int prev_on;
	int rc;

	if (on == prev_on)
		return 0;

	if (!reg_8901_hdmi_mvs)
		_GET_REGULATOR(reg_8901_hdmi_mvs, "8901_hdmi_mvs");

	if (on) {
		rc = regulator_enable(reg_8901_hdmi_mvs);
		if (rc) {
			pr_err("'%s' regulator enable failed, rc=%d\n",
				"8901_hdmi_mvs", rc);
			return rc;
		}
		pr_info("%s(on): success\n", __func__);
	} else {
		rc = regulator_disable(reg_8901_hdmi_mvs);
		if (rc)
			pr_warning("'%s' regulator disable failed, rc=%d\n",
				"8901_hdmi_mvs", rc);
		pr_info("%s(off): success\n", __func__);
	}

	prev_on = on;

	return 0;
}

static int hdmi_core_power(int on, int show)
{
	static struct regulator *reg_8058_l16;		/* VDD_HDMI */
	static int prev_on;
	int rc;

	if (on == prev_on)
		return 0;

	if (!reg_8058_l16)
		_GET_REGULATOR(reg_8058_l16, "8058_l16");

	if (on) {
		rc = regulator_set_voltage(reg_8058_l16, 1800000, 1800000);
		if (!rc)
			rc = regulator_enable(reg_8058_l16);
		if (rc) {
			pr_err("'%s' regulator enable failed, rc=%d\n",
				"8058_l16", rc);
			return rc;
		}

		pr_info("%s(on): success\n", __func__);
	} else {
		rc = regulator_disable(reg_8058_l16);
		if (rc)
			pr_warning("'%s' regulator disable failed, rc=%d\n",
				"8058_l16", rc);
		pr_info("%s(off): success\n", __func__);
	}

	prev_on = on;

	return 0;
}

static int hdmi_cec_power(int on)
{
	static struct regulator *reg_8901_l3;		/* HDMI_CEC */
	static int prev_on;
	int rc;

	if (on == prev_on)
		return 0;

	if (!reg_8901_l3)
		_GET_REGULATOR(reg_8901_l3, "8901_l3");

	if (on) {
		rc = regulator_set_voltage(reg_8901_l3, 3300000, 3300000);
		if (!rc)
			rc = regulator_enable(reg_8901_l3);
		if (rc) {
			pr_err("'%s' regulator enable failed, rc=%d\n",
				"8901_l3", rc);
			return rc;
		}
		pr_info("%s(on): success\n", __func__);
	} else {
		rc = regulator_disable(reg_8901_l3);
		if (rc)
			pr_warning("'%s' regulator disable failed, rc=%d\n",
				"8901_l3", rc);
		pr_info("%s(off): success\n", __func__);
	}

	return 0;
}
#endif /* CONFIG_FB_MSM_HDMI_MSM_PANEL */

static int lcdc_panel_power(int on)
{
	int flag_on = !!on;
	static int lcdc_power_save_on;

	if (lcdc_power_save_on == flag_on)
		return 0;

	lcdc_power_save_on = flag_on;

	if(panel_type == PANEL_ID_VERDI_AUO || panel_type == PANEL_ID_VERDI_AUO_RGB888)
		lcdc_auo_panel_power(on);
	else
		lcdc_samsung_panel_power(on);

	return 0;
}
/*
static int lcdc_config_gpio(int on)
{
	int n=0;
	int flag_on = !!on;
	static int lcdc_config_gpio_done;

	if (lcdc_config_gpio_done == flag_on)
		return 0;
	lcdc_config_gpio_done = flag_on;

	if(on) {
		for (n = 0; n < ARRAY_SIZE(lcd_panel_gpios); ++n)
			gpio_tlmm_config(lcd_panel_gpios[n], GPIO_CFG_ENABLE);
	} else {
		for (n = 0; n < ARRAY_SIZE(lcd_panel_gpios_sleep_mode); ++n)
			gpio_tlmm_config(lcd_panel_gpios_sleep_mode[n], GPIO_CFG_DISABLE);
	}
	return 0;
}
*/
#ifdef CONFIG_MSM_BUS_SCALING
static struct msm_bus_vectors mdp_init_vectors[] = {
	/* For now, 0th array entry is reserved.
	 * Please leave 0 as is and don't use it
	 */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 0,
		.ib = 0,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};

static struct msm_bus_vectors mdp_sd_smi_vectors[] = {
	/* Default case static display/UI/2d/3d if FB SMI */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 147460000,
		.ib = 184325000,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};

static struct msm_bus_vectors mdp_sd_ebi_vectors[] = {
	/* Default case static display/UI/2d/3d if FB SMI */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 0,
		.ib = 0,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 334080000,
		.ib = 417600000,
	},
};
static struct msm_bus_vectors mdp_vga_vectors[] = {
	/* VGA and less video */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 273408000,
		.ib = 341760000,
	},
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 273408000,
		.ib = 341760000,
	},
};

static struct msm_bus_vectors mdp_720p_vectors[] = {
	/* 720p and less video */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 328704000,
		.ib = 410880000,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 328704000,
		.ib = 410880000,
	},
};

static struct msm_bus_vectors mdp_1080p_vectors[] = {
	/* 1080p and less video */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 432384000,
		.ib = 540480000,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 432384000,
		.ib = 540480000,
	},
};
static struct msm_bus_paths mdp_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(mdp_init_vectors),
		mdp_init_vectors,
	},
	{
		ARRAY_SIZE(mdp_sd_smi_vectors),
		mdp_sd_smi_vectors,
	},
	{
		ARRAY_SIZE(mdp_sd_ebi_vectors),
		mdp_sd_ebi_vectors,
	},
	{
		ARRAY_SIZE(mdp_vga_vectors),
		mdp_vga_vectors,
	},
	{
		ARRAY_SIZE(mdp_720p_vectors),
		mdp_720p_vectors,
	},
	{
		ARRAY_SIZE(mdp_1080p_vectors),
		mdp_1080p_vectors,
	},
};
static struct msm_bus_scale_pdata mdp_bus_scale_pdata = {
	mdp_bus_scale_usecases,
	ARRAY_SIZE(mdp_bus_scale_usecases),
	.name = "mdp",
};

#endif
#ifdef CONFIG_MSM_BUS_SCALING
static struct msm_bus_vectors dtv_bus_init_vectors[] = {
	/* For now, 0th array entry is reserved.
	 * Please leave 0 as is and don't use it
	 */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 0,
		.ib = 0,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};
static struct msm_bus_vectors dtv_bus_def_vectors[] = {
	/* For now, 0th array entry is reserved.
	 * Please leave 0 as is and don't use it
	 */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 435456000,
		.ib = 544320000,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 435456000,
		.ib = 544320000,
	},
};
static struct msm_bus_paths dtv_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(dtv_bus_init_vectors),
		dtv_bus_init_vectors,
	},
	{
		ARRAY_SIZE(dtv_bus_def_vectors),
		dtv_bus_def_vectors,
	},
};
static struct msm_bus_scale_pdata dtv_bus_scale_pdata = {
	dtv_bus_scale_usecases,
	ARRAY_SIZE(dtv_bus_scale_usecases),
	.name = "dtv",
};

static struct lcdc_platform_data dtv_pdata = {
	.bus_scale_table = &dtv_bus_scale_pdata,
};
#endif

static struct lcdc_platform_data lcdc_pdata = {
	.lcdc_power_save   = lcdc_panel_power,
};

static uint32_t msm_spi_gpio[] = {
	GPIO_CFG(VERDI_LTE_SPI_DO,  1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(VERDI_LTE_SPI_DI,  1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(VERDI_LTE_SPI_CS,  1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(VERDI_LTE_SPI_CLK, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
#ifdef CONFIG_MSM8X60_AUDIO_LTE
	GPIO_CFG(VERDI_LTE_AUD_SPI_CS, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_AUD_SPI_CS_XB, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
#endif
};

#ifdef CONFIG_MSM8X60_AUDIO_LTE
void msm_snddev_voltage_on(void)
{
}

void msm_snddev_voltage_off(void)
{
}

#endif /* CONFIG_MSM8X60_AUDIO_LTE */

int mdp_core_clk_rate_table[] = {
	85330000,
	128000000,
	160000000,
	200000000,
};
static struct msm_panel_common_pdata mdp_pdata = {
	.mdp_core_clk_rate = 85330000,
	.mdp_core_clk_table = mdp_core_clk_rate_table,
	.num_mdp_clk = ARRAY_SIZE(mdp_core_clk_rate_table),
#ifdef CONFIG_MSM_BUS_SCALING
	.mdp_bus_scale_table = &mdp_bus_scale_pdata,
#endif
	.mdp_img_stick_wa	=	img_stick_wa,
	.update_interval	=	1800000,
	.img_stick_on		=	ATOMIC_INIT(0),
	.dcr_panel_pinfo	=	&dcr_panel,
};

static void __init msm_fb_add_devices(void)
{
	printk(KERN_INFO "panel ID= 0x%x\n", panel_type);
	msm_fb_register_device("mdp", &mdp_pdata);

	msm_fb_register_device("lcdc", &lcdc_pdata);
#ifdef CONFIG_MSM_BUS_SCALING
	msm_fb_register_device("dtv", &dtv_pdata);
#endif
}

#ifdef CONFIG_MSM_RPM
static struct msm_rpm_platform_data msm_rpm_data = {
	.reg_base_addrs = {
		[MSM_RPM_PAGE_STATUS] = MSM_RPM_BASE,
		[MSM_RPM_PAGE_CTRL] = MSM_RPM_BASE + 0x400,
		[MSM_RPM_PAGE_REQ] = MSM_RPM_BASE + 0x600,
		[MSM_RPM_PAGE_ACK] = MSM_RPM_BASE + 0xa00,
		[MSM_RPM_PAGE_STAT] = MSM_RPM_BASE + 0x3E04,
	},

	.irq_ack = RPM_SCSS_CPU0_GP_HIGH_IRQ,
	.irq_err = RPM_SCSS_CPU0_GP_LOW_IRQ,
	.irq_vmpm = RPM_SCSS_CPU0_GP_MEDIUM_IRQ,
};
#endif

#if 0
static void __init msm_gfx3d_clk_init(void)
{
	struct clk *clk;
	int rc;

	/* Disabling or changing the rate of the gfx3d_clk may causes
	 * instability. Turn it on and leave it on until this is resolved. */
	WARN((kgsl_pdata.max_grp3d_freq != kgsl_pdata.min_grp3d_freq),
		"gfx3d_clk scaling is not allowed, re-setting "
		"min_grp3d_freq to match max_grp3d_freq.\n");
	kgsl_pdata.min_grp3d_freq = kgsl_pdata.max_grp3d_freq;

	clk = clk_get(NULL, "gfx3d_clk");
	if (IS_ERR(clk))
		goto err;
	rc = clk_set_rate(clk, kgsl_pdata.max_grp3d_freq);
	if (rc)
		goto err;
	rc = clk_enable(clk);
	if (rc)
		goto err;

	return;
err:
	pr_err("%s: Failed to set up gfx3d_clk.\n", __func__);
}
#endif

#ifdef CONFIG_USB_ANDROID
static void verdi_lte_add_usb_devices(void)
{
	printk("%s\n", __func__);
	android_usb_pdata.products[0].product_id =
		android_usb_pdata.product_id;

	config_verdi_lte_mhl_gpios();

#if defined(CONFIG_USB_OTG)
	verdi_lte_otg_overcurrent_gpio();
	msm_otg_pdata.idgnd_gpio = VERDI_LTE_GPIO_USB_ID;
	msm_device_otg.dev.platform_data = &msm_otg_pdata;
	msm_device_hsusb_host.dev.platform_data = &msm_usb_host_pdata;

	platform_device_register(&msm_device_otg);
#endif

	msm_device_hsusb.dev.platform_data = &msm_hsusb_pdata;
	platform_device_register(&msm_device_hsusb);
	platform_device_register(&usb_mass_storage_device);
	platform_device_register(&android_usb_device);
#if defined(CONFIG_USB_F_SERIAL_SDIO) || defined(CONFIG_USB_F_SERIAL_SMD)
	platform_device_register(&usb_gadget_fserial_device);
#endif
}
#endif

int __initdata irq_ignore_tbl[] =
{
	MSM_GPIO_TO_INT(133),
	MSM_GPIO_TO_INT(134),
};
unsigned __initdata irq_num_ignore_tbl = ARRAY_SIZE(irq_ignore_tbl);

int __initdata clk_ignore_tbl[] =
{
	L_GSBI12_UART_CLK,
	L_SDC4_CLK,
	L_SDC4_P_CLK,
};

unsigned __initdata clk_num_ignore_tbl = ARRAY_SIZE(clk_ignore_tbl);

#define PM8058_LPM_SET(id)	(1 << RPM_VREG_ID_##id)
#define PM8901_LPM_SET(id)	(1 << (RPM_VREG_ID_##id - RPM_VREG_ID_PM8901_L0))

uint32_t __initdata regulator_lpm_set[] =
{
	PM8058_LPM_SET(PM8058_L0) | PM8058_LPM_SET(PM8058_L1) | PM8058_LPM_SET(PM8058_L2) |
	PM8058_LPM_SET(PM8058_L5) | PM8058_LPM_SET(PM8058_L6) | PM8058_LPM_SET(PM8058_L7) |
	PM8058_LPM_SET(PM8058_L8) | PM8058_LPM_SET(PM8058_L9) | PM8058_LPM_SET(PM8058_L10) |
	PM8058_LPM_SET(PM8058_L11) | PM8058_LPM_SET(PM8058_L12) | PM8058_LPM_SET(PM8058_L13) |
	PM8058_LPM_SET(PM8058_L15) | PM8058_LPM_SET(PM8058_L16) | PM8058_LPM_SET(PM8058_L17) |
	PM8058_LPM_SET(PM8058_L18) | PM8058_LPM_SET(PM8058_L19) | PM8058_LPM_SET(PM8058_L20) |
	PM8058_LPM_SET(PM8058_L21) | PM8058_LPM_SET(PM8058_L22) | PM8058_LPM_SET(PM8058_L23) |
	PM8058_LPM_SET(PM8058_L24) | PM8058_LPM_SET(PM8058_L25),
	PM8901_LPM_SET(PM8901_L0) | PM8901_LPM_SET(PM8901_L1) | PM8901_LPM_SET(PM8901_L2) |
	PM8901_LPM_SET(PM8901_L4) | PM8901_LPM_SET(PM8901_L6),
};

static void __init verdi_lte_init(void)
{
	int i;
	int mfg_mode = 0;
	msm_mpm_defer_ignore_list = 1;
	/*
	 * Initialize RPM first as other drivers and devices may need
	 * it for their initialization.
	 */
#ifdef CONFIG_MSM_RPM
	BUG_ON(msm_rpm_init(&msm_rpm_data));
	msm_rpm_lpm_init(regulator_lpm_set, ARRAY_SIZE(regulator_lpm_set));
#endif
	if (msm_xo_init())
		pr_err("Failed to initialize XO votes\n");

	mfg_mode = board_mfg_mode();

	printk(KERN_INFO "verdi_lte_init() revision=%d\n", system_rev);


	if (socinfo_init() < 0)
		printk(KERN_ERR "%s: socinfo_init() failed!\n",
		       __func__);
	msm8x60_check_2d_hardware();

	/*
	 * Initialize SPM before acpuclock as the latter calls into SPM
	 * driver to set ACPU voltages.
	 */
	if (SOCINFO_VERSION_MAJOR(socinfo_get_version()) != 1)
		msm_spm_init(msm_spm_data, ARRAY_SIZE(msm_spm_data));
	else
		msm_spm_init(msm_spm_data_v1, ARRAY_SIZE(msm_spm_data_v1));

	/*
	 * Disable regulator info printing so that regulator registration
	 * messages do not enter the kmsg log.
	 */
	regulator_suppress_info_printing();

	if (system_rev >= 1) {
		flashlight_data.torch_set1 = PM8058_GPIO_PM_TO_SYS(VERDI_LTE_TORCH_SET1_XB);
	}

	/* Initialize regulators needed for clock_init. */
	platform_add_devices(early_regulators, ARRAY_SIZE(early_regulators));

	msm_clock_init(msm_clocks_8x60, msm_num_clocks_8x60);

	/* Buses need to be initialized before early-device registration
	 * to get the platform data for fabrics.
	 */
	msm8x60_init_buses();
#ifdef CONFIG_BT
	bt_export_bd_address();
#endif
	platform_add_devices(early_devices, ARRAY_SIZE(early_devices));
	/* CPU frequency control is not supported on simulated targets. */
	msm_acpu_clock_init(&msm8x60_acpu_clock_data);

#ifdef CONFIG_PERFLOCK
        perflock_init(&puccini_perflock_data);
#endif

	msm8x60_init_tlmm();
	msm8x60_init_gpiomux(msm8x60_htc_gpiomux_cfgs);
	msm8x60_init_mmc();

#ifdef CONFIG_MSM_DSPS
	msm8x60_init_dsps();
#endif
	platform_add_devices(msm_footswitch_devices,
					     msm_num_footswitch_devices);

	msm8x60_init_camera();

	/* Accessory */
	pr_info("[HS_BOARD] (%s) mfg_mode = %d\n", __func__, mfg_mode);
	if (mfg_mode != 0 || (get_kernel_flag() & BIT1)) {
		pr_info("[HS_BOARD] (%s) Disable H2W GPIO\n", __func__);
		htc_headset_misc_data.driver_flag = 0;
	} else {
		pr_info("[HS_BOARD] (%s) Enable H2W GPIO\n", __func__);
		config_gpio_table(headset_h2w_gpio_table,
				  ARRAY_SIZE(headset_h2w_gpio_table));
	}
	pr_info("[HS_BOARD] (%s) system_rev = %d\n", __func__, system_rev);
	if (system_rev >= 1) {
		htc_headset_pmic_data.key_gpio =
			PM8058_GPIO_PM_TO_SYS(VERDI_LTE_AUD_REMO_PRES);
		htc_headset_pmic_data.key_enable_gpio =
			PM8058_GPIO_PM_TO_SYS(VERDI_LTE_AUD_REMO_EN_PM);
		htc_headset_8x60.dev.platform_data =
			&htc_headset_8x60_data_xb;
		htc_headset_mgr_data.headset_config_num =
			ARRAY_SIZE(htc_headset_mgr_config);
		htc_headset_mgr_data.headset_config = htc_headset_mgr_config;
		pr_info("[HS_BOARD] (%s) Set MEMS config\n", __func__);
	}

	if (machine_is_verdi_lte()) {
		platform_add_devices(surf_devices,
				     ARRAY_SIZE(surf_devices));
#ifdef CONFIG_USB_EHCI_MSM
		msm_add_host(0, &msm_usb_host_pdata);
#endif
	}

	platform_add_devices(charm_devices, ARRAY_SIZE(charm_devices));

	if (system_rev == 0)
		cable_detect_pdata.mhl_version_ctrl_flag = true;

	/* [PUCCINI_LTE#6957] Aftermarket Charger Issues*/
	if (mfg_mode != 5) /* In the offmode charging, no LCM device registered */
		msm_fb_add_devices();
	else
		pr_info("OffMode Chargin - No LCM device registered\n");

	register_i2c_devices();
#ifdef CONFIG_USB_ANDROID
	verdi_lte_add_usb_devices();
#endif

#ifdef CONFIG_SP3D
	{
	spi_register_board_info(sp3d_spi_board_info,
			ARRAY_SIZE(sp3d_spi_board_info));
	}
#endif
	msm_pm_set_platform_data(msm_pm_data, ARRAY_SIZE(msm_pm_data));
	msm_cpuidle_set_states(msm_cstates, ARRAY_SIZE(msm_cstates),
				msm_pm_data);

#ifdef CONFIG_MSM8X60_AUDIO_LTE
	msm_snddev_init();
#endif

	for (i = 0; i < (sizeof(verdi_lte_ts_ntrig_data)/sizeof(struct ntrig_spi_platform_data)); i++) {
		if (system_rev == 0) {
			verdi_lte_ts_ntrig_data[i].spi_enable = PM8058_GPIO_PM_TO_SYS(VERDI_LTE_SPI_ENABLE);
			verdi_lte_ts_ntrig_data[i].irq_gpio = VERDI_LTE_TP_ATT_N;
			verdi_lte_ts_ntrig_data[i].lcm_ts_power = NULL;
		} else if (system_rev == 1) {
			verdi_lte_ts_ntrig_data[i].spi_enable = VERDI_LTE_SPI_ENABLE_XB;
			verdi_lte_ts_ntrig_data[i].irq_gpio = VERDI_LTE_TP_ATT_N_XB;
			verdi_lte_ts_ntrig_data[i].lcm_ts_power = NULL;
		}
	}


	if (system_rev == 0)
		spi_register_board_info(msm_spi_board_info, ARRAY_SIZE(msm_spi_board_info));
	else
		spi_register_board_info(msm_spi_board_info_xb, ARRAY_SIZE(msm_spi_board_info_xb));

	gpio_tlmm_config(msm_spi_gpio[0], GPIO_CFG_ENABLE);
	gpio_tlmm_config(msm_spi_gpio[1], GPIO_CFG_ENABLE);
	gpio_tlmm_config(msm_spi_gpio[2], GPIO_CFG_ENABLE);
	gpio_tlmm_config(msm_spi_gpio[3], GPIO_CFG_ENABLE);
#ifdef CONFIG_MSM8X60_AUDIO_LTE
	if (system_rev == 0) {
		gpio_tlmm_config(msm_spi_gpio[4], GPIO_CFG_ENABLE);
		gpio_set_value(VERDI_LTE_AUD_SPI_CS, 1);
	} else {
		gpio_tlmm_config(msm_spi_gpio[5], GPIO_CFG_ENABLE);
		gpio_set_value(VERDI_LTE_AUD_SPI_CS_XB, 1);
	}

	verdi_lte_audio_init();
#endif
	verdi_lte_init_keypad();
	verdi_lte_wifi_init();
	msm8x60_multi_sdio_init();

	if (system_rev > 0){
		printk(KERN_INFO "device=%d change led_config to XB\n",system_rev);
		pm8058_leds_data.led_config = pm_led_config_XB;
		pm8058_leds_data.num_leds = ARRAY_SIZE(pm_led_config_XB);
	}

	sysinfo_proc_init();
	mdm_loaded_info();

	msm_mpm_set_irq_ignore_list(irq_ignore_tbl, irq_num_ignore_tbl);
	msm_clk_soc_set_ignore_list(clk_ignore_tbl, clk_num_ignore_tbl);
	htc_monitor_init();
	htc_PM_monitor_init();
}

#define PHY_BASE_ADDR1  0x48000000
#define SIZE_ADDR1      0x34000000

static void __init verdi_lte_fixup(struct machine_desc *desc, struct tag *tags,
				 char **cmdline, struct meminfo *mi)
{
	mi->nr_banks = 1;
	mi->bank[0].start = PHY_BASE_ADDR1;
	mi->bank[0].node = PHYS_TO_NID(PHY_BASE_ADDR1);
	mi->bank[0].size = SIZE_ADDR1;
}

MACHINE_START(VERDI_LTE, "verdilte")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.fixup = verdi_lte_fixup,
	.map_io = verdi_lte_map_io,
	.init_irq = msm8x60_init_irq,
	.init_machine = verdi_lte_init,
	.timer = &msm_timer,
MACHINE_END

