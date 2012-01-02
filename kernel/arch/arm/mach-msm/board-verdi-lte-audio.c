/* linux/arch/arm/mach-msm/board-verdi-lte-audio.c
 *
 * Copyright (C) 2010-2011 HTC Corporation.
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

#include <linux/android_pmem.h>
#include <linux/mfd/pmic8058.h>
#include <linux/mfd/marimba.h>
#include <linux/delay.h>
#include <linux/pmic8058-othc.h>
#include <linux/spi/spi_aic3254.h>
#include <linux/regulator/pmic8058-regulator.h>
#include <linux/regulator/pmic8901-regulator.h>
#include <mach/gpio.h>
#include <mach/dal.h>
#include <mach/tpa2026.h>
#include <mach/tpa2028.h>
#include <mach/qdsp6v2/snddev_icodec.h>
#include <mach/qdsp6v2/snddev_ecodec.h>
#include <mach/qdsp6v2/snddev_hdmi.h>
#include <mach/htc_acoustic_8x60.h>
#include <mach/qdsp6v2/snddev_hdmi.h>

#include "board-verdi-lte.h"
#include "board-verdi-lte-audio-data.h"

static struct mutex bt_sco_lock;
static struct mutex mic_lock;
static int curr_rx_mode;
static atomic_t aic3254_ctl = ATOMIC_INIT(0);
static int audio_2v85_usage_counter;
static int audio_first_boot_counter;
static struct mutex audio_2v85_usage_lock;

#define BIT_SPEAKER	(1 << 0)
#define BIT_HEADSET	(1 << 1)
#define BIT_RECEIVER	(1 << 2)
#define BIT_FM_SPK	(1 << 3)
#define BIT_FM_HS	(1 << 4)


static uint32_t msm_snddev_gpio_on[] = {
	GPIO_CFG(VERDI_LTE_AUD_RX_MCLK1, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,\
		GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_AUD_QTR_TX_MCLK1, 1, GPIO_CFG_OUTPUT, \
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_AUD_QTR_TX_I2S_SD2, 1, GPIO_CFG_OUTPUT, \
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static uint32_t msm_snddev_gpio_off[] = {
	GPIO_CFG(VERDI_LTE_AUD_RX_MCLK1, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,\
		GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_AUD_QTR_TX_MCLK1, 0, GPIO_CFG_OUTPUT, \
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_AUD_QTR_TX_I2S_SD2, 0, GPIO_CFG_OUTPUT, \
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static uint32_t verdi_lte_a1028_gpio[] = {
	GPIO_CFG(VERDI_LTE_AUD_A1028_RESET, 0, GPIO_CFG_OUTPUT, \
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_AUD_A1028_INT, 0, GPIO_CFG_INPUT, \
		GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_AUD_A1028_WAKEUP, 0, GPIO_CFG_OUTPUT, \
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static uint32_t verdi_lte_i2s_rx2_gpio[] = {
	GPIO_CFG(VERDI_LTE_AUD_RX_CLK2, 0, GPIO_CFG_OUTPUT, \
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_AUD_RX_WS2, 0, GPIO_CFG_OUTPUT, \
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_AUD_RX_SD2, 0, GPIO_CFG_OUTPUT, \
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_AUD_RX_MCLK2, 0, GPIO_CFG_OUTPUT, \
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static uint32_t aux_pcm_gpio_off[] = {
	GPIO_CFG(VERDI_LTE_GPIO_BT_PCM_OUT, 0, GPIO_CFG_OUTPUT, \
			GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_GPIO_BT_PCM_IN, 0, GPIO_CFG_INPUT, \
			GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_GPIO_BT_PCM_SYNC, 0, GPIO_CFG_OUTPUT, \
			GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_GPIO_BT_PCM_CLK, 0, GPIO_CFG_OUTPUT, \
			GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static uint32_t aux_pcm_gpio_on[] = {
	GPIO_CFG(VERDI_LTE_GPIO_BT_PCM_OUT, 1, GPIO_CFG_OUTPUT, \
			GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_GPIO_BT_PCM_IN, 1, GPIO_CFG_INPUT, \
			GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_GPIO_BT_PCM_SYNC, 1, GPIO_CFG_OUTPUT, \
			GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VERDI_LTE_GPIO_BT_PCM_CLK, 1, GPIO_CFG_OUTPUT, \
			GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static uint32_t msm_qtr_reset_gpio[] = {
	GPIO_CFG(VERDI_LTE_AUD_QTR_RESET, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
		if (rc) {
			pr_aud_info("%s gpio_tlmm_config(%#x)=%d", \
				__func__, table[n], rc);
			break;
		}
	}
}
static struct regulator *vreg_audio_2v85;
static unsigned int verdi_lte_audio_2v85_set_voltage(int en)
{
	int rc;

	pr_aud_info("%s %d", __func__, en);

	if (en) {
		vreg_audio_2v85 = regulator_get(NULL, "8058_l11");
		if (IS_ERR(vreg_audio_2v85)) {
			pr_err("%s: Unable to get 8058_l11\n", __func__);
			return -ENODEV;
		}

		rc = regulator_set_voltage(vreg_audio_2v85, 2850000, 2850000);
		if (rc) {
			pr_err("%s: unable to set V_AUD_2V85 voltage to 2.85V\n",
				__func__);
			regulator_put(vreg_audio_2v85);
		}
	} else {
		rc = regulator_disable(vreg_audio_2v85);
		if (rc)
			pr_err("%s: Disable regulator 8058_l11 failed\n",
				__func__);

		regulator_put(vreg_audio_2v85);
	}

	return rc;
}

static unsigned int verdi_lte_audio_2v85_power_on(int en)
{
	int rc;

	pr_aud_info("%s %d", __func__, en);

	if (en) {
		rc = regulator_enable(vreg_audio_2v85);
		if (rc) {
			pr_err("%s: Enable regulator 8058_l11 failed\n", \
				__func__);
			regulator_put(vreg_audio_2v85);
		}
	} else {
		rc = regulator_disable(vreg_audio_2v85);
		if (rc) {
			pr_err("%s: Disable regulator 8058_l11 failed\n", \
				__func__);
			regulator_put(vreg_audio_2v85);
		}
	}

	return rc;
}

static unsigned int verdi_lte_audio_2v85_enable(int en)
{
	int rc = 0;

	mutex_lock(&audio_2v85_usage_lock);
	if (en) {
		if (audio_2v85_usage_counter == 0)
			rc = verdi_lte_audio_2v85_power_on(1);
		audio_2v85_usage_counter++;
	} else {
		if (audio_2v85_usage_counter > 0) {
			audio_2v85_usage_counter--;
			if (audio_2v85_usage_counter == 0)
				rc = verdi_lte_audio_2v85_power_on(0);
		} else
			pr_aud_info("%s: counter error!\n", __func__);
	}
	mutex_unlock(&audio_2v85_usage_lock);

	return rc;
}

void verdi_lte_snddev_poweramp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		if (audio_first_boot_counter == 0) {
			pr_aud_info("%s skip first time turning on speaker\n", \
				__func__);
			audio_first_boot_counter = 1;
		} else {
			verdi_lte_audio_2v85_enable(en);
			msleep(50);
			gpio_set_value(PM8058_GPIO_PM_TO_SYS(\
				VERDI_LTE_AUD_SPK1_EN), 1);
			msleep(1);
			set_speaker_amp_2026(1);
			set_speaker_amp_2028(1);

			if (!atomic_read(&aic3254_ctl))
				curr_rx_mode |= BIT_SPEAKER;
		}
	} else {
		if (audio_first_boot_counter == 1) {
			pr_aud_info("%s skip first time turning off speaker\n",\
				__func__);
			audio_first_boot_counter = 2;
		} else {
			set_speaker_amp_2026(0);
			set_speaker_amp_2028(0);
			gpio_set_value(PM8058_GPIO_PM_TO_SYS(\
				VERDI_LTE_AUD_SPK1_EN), 0);
			if (!atomic_read(&aic3254_ctl))
				curr_rx_mode &= ~BIT_SPEAKER;

			verdi_lte_audio_2v85_enable(en);
		}
	}
}

void verdi_lte_snddev_hsed_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		msleep(50);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_AUD_HP_EN), 1);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode |= BIT_HEADSET;
	} else {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_AUD_HP_EN), 0);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode &= ~BIT_HEADSET;
	}
}

void verdi_lte_snddev_hs_spk_pamp_on(int en)
{
	verdi_lte_snddev_poweramp_on(en);
	verdi_lte_snddev_hsed_pamp_on(en);
}

void verdi_lte_snddev_receiver_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode |= BIT_RECEIVER;
	} else {
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode &= ~BIT_RECEIVER;
	}
}

void verdi_lte_snddev_bt_sco_pamp_on(int en)
{
	static int bt_sco_refcount;
	pr_aud_info("%s %d\n", __func__, en);
	mutex_lock(&bt_sco_lock);
	if (en) {
		if (++bt_sco_refcount == 1)
			config_gpio_table(aux_pcm_gpio_on, \
				ARRAY_SIZE(aux_pcm_gpio_on));
	} else {
		if (--bt_sco_refcount == 0) {
			config_gpio_table(aux_pcm_gpio_off,	\
				ARRAY_SIZE(aux_pcm_gpio_off));
			gpio_set_value(VERDI_LTE_GPIO_BT_PCM_OUT, 0);
			gpio_set_value(VERDI_LTE_GPIO_BT_PCM_SYNC, 0);
			gpio_set_value(VERDI_LTE_GPIO_BT_PCM_CLK, 0);
		}
	}
	mutex_unlock(&bt_sco_lock);
}

/* power on/off externnal mic bias */
void verdi_lte_mic_enable(int en, int shift)
{
	pr_aud_info("%s: %d, shift %d\n", __func__, en, shift);

	mutex_lock(&mic_lock);

	if (en)	{
		verdi_lte_audio_2v85_enable(en);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_AUD_REMO_EN_PM), 1);
		pm8058_micbias_enable(OTHC_MICBIAS_2, OTHC_SIGNAL_ALWAYS_ON);
	} else {
		pm8058_micbias_enable(OTHC_MICBIAS_2, OTHC_SIGNAL_OFF);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_AUD_REMO_EN_PM), 0);
		verdi_lte_audio_2v85_enable(en);
	}
	mutex_unlock(&mic_lock);
}

void verdi_lte_snddev_imic_pamp_on(int en)
{
	int ret;

	pr_aud_info("%s %d\n", __func__, en);

	if (en) {
		verdi_lte_audio_2v85_enable(en);

		ret = pm8058_micbias_enable(OTHC_MICBIAS_0, OTHC_SIGNAL_ALWAYS_ON);
		if (ret)
			pr_aud_err("%s: Enabling int mic power failed\n", __func__);

		/* select internal mic path */
		gpio_set_value(PM8058_GPIO_PM_TO_SYS( \
			VERDI_LTE_AUD_STEREO_REC), 1);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_AUD_MIC_SEL), 0);
	} else {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS( \
			VERDI_LTE_AUD_STEREO_REC), 0);

		ret = pm8058_micbias_enable(OTHC_MICBIAS_0, OTHC_SIGNAL_OFF);
		if (ret)
			pr_aud_err("%s: Enabling int mic power failed\n", __func__);

		verdi_lte_audio_2v85_enable(en);
	}
}

void verdi_lte_snddev_bmic_pamp_on(int en)
{
	int ret;

	pr_aud_info("%s %d\n", __func__, en);

	if (en) {
		verdi_lte_audio_2v85_enable(en);

		ret = pm8058_micbias_enable(OTHC_MICBIAS_1, OTHC_SIGNAL_ALWAYS_ON);
		if (ret)
			pr_aud_err("%s: Enabling int mic power failed\n", __func__);

		/* select internal mic path */
		gpio_set_value(PM8058_GPIO_PM_TO_SYS( \
			VERDI_LTE_AUD_STEREO_REC), 0);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_AUD_MIC_SEL), 0);
	} else {
		ret = pm8058_micbias_enable(OTHC_MICBIAS_1, OTHC_SIGNAL_OFF);
		if (ret)
			pr_aud_err("%s: Enabling int mic power failed\n", __func__);

		verdi_lte_audio_2v85_enable(en);
	}
}

void verdi_lte_snddev_emic_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);

	if (en) {
		verdi_lte_audio_2v85_enable(en);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_AUD_MIC_SEL), 1);
	} else {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_AUD_MIC_SEL), 0);
		verdi_lte_audio_2v85_enable(en);
	}
}

void verdi_lte_snddev_stereo_mic_pamp_on(int en)
{
	int ret;

	pr_aud_info("%s %d\n", __func__, en);

	if (en) {
		verdi_lte_audio_2v85_enable(en);

		ret = pm8058_micbias_enable(OTHC_MICBIAS_0, \
			OTHC_SIGNAL_ALWAYS_ON);
		if (ret)
			pr_aud_err("%s: Enabling int mic power failed\n", \
				__func__);

		ret = pm8058_micbias_enable(OTHC_MICBIAS_1, \
			OTHC_SIGNAL_ALWAYS_ON);
		if (ret)
			pr_aud_err("%s: Enabling int mic power failed\n", \
				__func__);

		/* select internal mic path */
		gpio_set_value(PM8058_GPIO_PM_TO_SYS( \
			VERDI_LTE_AUD_STEREO_REC), 1);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_AUD_MIC_SEL), 0);
	} else {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS( \
			VERDI_LTE_AUD_STEREO_REC), 0);

		ret = pm8058_micbias_enable(OTHC_MICBIAS_0, OTHC_SIGNAL_OFF);
		if (ret)
			pr_aud_err("%s: Disabling int mic power failed\n", \
				__func__);
		ret = pm8058_micbias_enable(OTHC_MICBIAS_1, OTHC_SIGNAL_OFF);
		if (ret)
			pr_aud_err("%s: Disabling int mic power failed\n", \
				__func__);

		verdi_lte_audio_2v85_enable(en);
	}
}
void verdi_lte_snddev_fmspk_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_AUD_SPK1_EN), 1);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode |= BIT_FM_SPK;
	} else {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_AUD_SPK1_EN), 0);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode &= ~BIT_FM_SPK;
	}
}

void verdi_lte_snddev_fmhs_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_AUD_HP_EN), 1);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode |= BIT_FM_HS;
	} else {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_AUD_HP_EN), 0);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode &= ~BIT_FM_HS;
	}
}

void verdi_lte_voltage_on (int en)
{
	/* to be implemented */
}

int verdi_lte_get_rx_vol(uint8_t hw, int network, int level)
{
	int vol = 0;

	/* to be implemented */

	pr_aud_info("%s(%d, %d, %d) => %d\n", __func__, hw, network, level, vol);

	return vol;
}

void verdi_lte_rx_amp_enable(int en)
{
	if (curr_rx_mode != 0) {
		atomic_set(&aic3254_ctl, 1);
		pr_aud_info("%s: curr_rx_mode 0x%x, en %d\n",
			__func__, curr_rx_mode, en);
		if (curr_rx_mode & BIT_SPEAKER)
			verdi_lte_snddev_poweramp_on(en);
		if (curr_rx_mode & BIT_HEADSET)
			verdi_lte_snddev_hsed_pamp_on(en);
		if (curr_rx_mode & BIT_RECEIVER)
			verdi_lte_snddev_receiver_pamp_on(en);
		if (curr_rx_mode & BIT_FM_SPK)
			verdi_lte_snddev_fmspk_pamp_on(en);
		if (curr_rx_mode & BIT_FM_HS)
			verdi_lte_snddev_fmhs_pamp_on(en);
		atomic_set(&aic3254_ctl, 0);;
	}
}

int verdi_lte_support_aic3254(void)
{
	return 1;
}

int verdi_lte_support_back_mic(void)
{
	return 1;
}

int verdi_lte_is_msm_i2s_slave(void)
{
	/* 1 - CPU slave, 0 - CPU master */
	return 1;
}

int verdi_lte_support_audience(void)
{
	return 1;
}

int verdi_lte_support_aic3254_use_mclk(void)
{
	if (system_rev == 0)
		return 1;
	else
		return 0;
}

int verdi_lte_support_receiver(void)
{
	return 0;
}

int verdi_lte_support_adie(void)
{
	return 1;
}

void verdi_lte_reset_3254(void)
{
	pr_aud_err("hard reset aic3254\n");
	gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_AUD_CODEC_RST), 0);
	mdelay(1);
	gpio_set_value(PM8058_GPIO_PM_TO_SYS(VERDI_LTE_AUD_CODEC_RST), 1);
}

void verdi_lte_get_acoustic_tables(struct acoustic_tables *tb)
{
	if (system_rev == 0)
		strcpy(tb->aic3254,	"IOTable.txt\0");
	else
		strcpy(tb->aic3254,	"IOTable_XB.txt\0");
	strcpy(tb->tpa2026, "TPA2026_CFG.csv");
	strcpy(tb->tpa2028, "TPA2028_CFG.csv");
	if (system_rev == 0x80)
		strcpy(tb->codecdspid, "CodecDSPID_PVT.txt");
}

void verdi_lte_reset_timpani(void)
{
	pr_aud_err("hard reset timpani\n");
	gpio_set_value(VERDI_LTE_AUD_QTR_RESET, 0);
	mdelay(10);
	gpio_set_value(VERDI_LTE_AUD_QTR_RESET, 1);
}

static struct q6v2audio_analog_ops ops = {
	.speaker_enable	        = verdi_lte_snddev_poweramp_on,
	.headset_enable	        = verdi_lte_snddev_hsed_pamp_on,
	.handset_enable	        = verdi_lte_snddev_receiver_pamp_on,
	.headset_speaker_enable	= verdi_lte_snddev_hs_spk_pamp_on,
	.bt_sco_enable	        = verdi_lte_snddev_bt_sco_pamp_on,
	.int_mic_enable         = verdi_lte_snddev_imic_pamp_on,
	.back_mic_enable        = verdi_lte_snddev_bmic_pamp_on,
	.ext_mic_enable         = verdi_lte_snddev_emic_pamp_on,
	.stereo_mic_enable      = verdi_lte_snddev_stereo_mic_pamp_on,
	.fm_headset_enable      = verdi_lte_snddev_fmhs_pamp_on,
	.fm_speaker_enable      = verdi_lte_snddev_fmspk_pamp_on,
	.voltage_on		= verdi_lte_voltage_on,
};

static struct q6v2audio_icodec_ops iops = {
	.support_aic3254 = verdi_lte_support_aic3254,
	.is_msm_i2s_slave = verdi_lte_is_msm_i2s_slave,
	.support_aic3254_use_mclk = verdi_lte_support_aic3254_use_mclk,
	.support_adie			= verdi_lte_support_adie,
};

static struct q6v2audio_ecodec_ops eops = {
	.bt_sco_enable  = verdi_lte_snddev_bt_sco_pamp_on,
};

static struct aic3254_ctl_ops cops = {
	.rx_amp_enable        = verdi_lte_rx_amp_enable,
	.lb_dsp_init          = &LOOPBACK_DSP_INIT_PARAM,
	.lb_receiver_imic     = &LOOPBACK_Receiver_IMIC_PARAM,
	.lb_speaker_imic      = &LOOPBACK_Speaker_IMIC_PARAM,
	.lb_headset_emic      = &LOOPBACK_Headset_EMIC_PARAM,
	.lb_speaker_bmic      = &LOOPBACK_Speaker_BMIC_PARAM,
};

static struct acoustic_ops acoustic = {
	.enable_mic_bias = verdi_lte_mic_enable,
	.support_aic3254 = verdi_lte_support_aic3254,
	.support_back_mic = verdi_lte_support_back_mic,
	.get_acoustic_tables = verdi_lte_get_acoustic_tables,
	.support_audience = verdi_lte_support_audience,
	.support_receiver = verdi_lte_support_receiver,
	.support_adie = verdi_lte_support_adie,
	.reset_timpani = verdi_lte_reset_timpani,
};

void verdi_aic3254_set_mode(int config, int mode)
{
	aic3254_set_mode(config, mode);
}


static struct q6v2audio_aic3254_ops aops = {
	.aic3254_set_mode = verdi_aic3254_set_mode,
};

static uint32_t msm_aic3254_reset_gpio[] = {
	GPIO_CFG(VERDI_LTE_AUD_LDO_SEL, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
};

void __init verdi_lte_audio_init(void)
{
	mutex_init(&bt_sco_lock);
	mutex_init(&mic_lock);

#ifdef CONFIG_MSM8X60_AUDIO_LTE
	pr_aud_info("%s\n", __func__);
	htc_8x60_register_analog_ops(&ops);
	htc_8x60_register_icodec_ops(&iops);
	htc_8x60_register_ecodec_ops(&eops);
	acoustic_register_ops(&acoustic);
	htc_8x60_register_aic3254_ops(&aops);
#endif

	aic3254_register_ctl_ops(&cops);

	/* PMIC GPIO Init (See board-verdi-lte.c) */

	mutex_init(&audio_2v85_usage_lock);
	audio_2v85_usage_counter = 0;
	verdi_lte_audio_2v85_set_voltage(1);

	/* Reset AIC3254 */
	gpio_tlmm_config(msm_aic3254_reset_gpio[0], GPIO_CFG_ENABLE);

	/* this PIN is only defined in XA board */
	if (system_rev == 0) {
		gpio_set_value(VERDI_LTE_AUD_LDO_SEL, 0);

		mdelay(1);
		config_gpio_table(msm_snddev_gpio_on, \
			ARRAY_SIZE(msm_snddev_gpio_on));
		gpio_set_value(VERDI_LTE_AUD_RX_MCLK1, 0);
		gpio_set_value(VERDI_LTE_AUD_QTR_TX_MCLK1, 0);
		gpio_set_value(VERDI_LTE_AUD_QTR_TX_I2S_SD2, 0);
	} else {
		config_gpio_table(msm_snddev_gpio_off, \
			ARRAY_SIZE(msm_snddev_gpio_off));
		gpio_set_value(VERDI_LTE_AUD_RX_MCLK1, 0);
		gpio_set_value(VERDI_LTE_AUD_QTR_TX_MCLK1, 0);
		gpio_set_value(VERDI_LTE_AUD_QTR_TX_I2S_SD2, 0);
	}

	config_gpio_table(verdi_lte_a1028_gpio, \
		ARRAY_SIZE(verdi_lte_a1028_gpio));
	gpio_set_value(VERDI_LTE_AUD_A1028_RESET, 0);
	gpio_set_value(VERDI_LTE_AUD_A1028_WAKEUP, 0);
	gpio_set_value(VERDI_LTE_AUD_A1028_INT, 0);

	config_gpio_table(verdi_lte_i2s_rx2_gpio, \
		ARRAY_SIZE(verdi_lte_i2s_rx2_gpio));
	gpio_set_value(VERDI_LTE_AUD_RX_CLK2, 0);
	gpio_set_value(VERDI_LTE_AUD_RX_WS2, 0);
	gpio_set_value(VERDI_LTE_AUD_RX_SD2, 0);
	gpio_set_value(VERDI_LTE_AUD_RX_MCLK2, 0);

	gpio_tlmm_config(msm_qtr_reset_gpio[0], GPIO_CFG_ENABLE);
	gpio_set_value(VERDI_LTE_AUD_QTR_RESET, 1);

	/* BT AUX PCM PIN init */
	config_gpio_table(aux_pcm_gpio_off, ARRAY_SIZE(aux_pcm_gpio_off));
	gpio_set_value(VERDI_LTE_GPIO_BT_PCM_OUT, 0);
	gpio_set_value(VERDI_LTE_GPIO_BT_PCM_SYNC, 0);
	gpio_set_value(VERDI_LTE_GPIO_BT_PCM_CLK, 0);

	audio_first_boot_counter = 0;
}
