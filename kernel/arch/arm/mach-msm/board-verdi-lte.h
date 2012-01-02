/* linux/arch/arm/mach-msm/board-verdi-lte.h
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
 */

#ifndef __ARCH_ARM_MACH_MSM_BOARD_VERDI_LTE_H
#define __ARCH_ARM_MACH_MSM_BOARD_VERDI_LTE_H

#include <mach/board.h>

#define VERDI_LTE_PROJECT_NAME	"verdi_lte"

#define VERDI_LTE_AP2MDM_PMIC_RESET_TIME_MS		1400

/*Move to irqs-8x60.h*/
/* Macros assume PMIC GPIOs start at 0 */
#define PM8058_GPIO_BASE			NR_MSM_GPIOS
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)		(pm_gpio + PM8058_GPIO_BASE)
#define PM8058_GPIO_SYS_TO_PM(sys_gpio)		(sys_gpio - PM8058_GPIO_BASE)
#define PM8058_IRQ_BASE				(NR_MSM_IRQS + NR_GPIO_IRQS)

#define PM8901_GPIO_BASE			(PM8058_GPIO_BASE + \
						PM8058_GPIOS + PM8058_MPPS)
#define PM8901_GPIO_PM_TO_SYS(pm_gpio)		(pm_gpio + PM8901_GPIO_BASE)
#define PM8901_GPIO_SYS_TO_PM(sys_gpio)		(sys_gpio - PM901_GPIO_BASE)
#define PM8901_IRQ_BASE				(PM8058_IRQ_BASE + \
						NR_PMIC8058_IRQS)

#define GPIO_EXPANDER_GPIO_BASE \
	(PM8901_GPIO_BASE + PM8901_MPPS)
#define GPIO_EXPANDER_IRQ_BASE (PM8901_IRQ_BASE + NR_PMIC8901_IRQS)

#define VERDI_LTE_LAYOUTS			{ \
		{ { 0,  1, 0}, {-1,  0,  0}, { 0, 0,  1} }, \
		{ { 0, -1, 0}, { 1,  0,  0}, { 0, 0, -1} }, \
		{ {-1,  0, 0}, { 0, -1,  0}, { 0, 0,  1} }, \
		{ {-1,  0, 0}, { 0,  0, -1}, { 0, 1,  0} }  \
					}
#if 0
#define MSM_LINUX_BASE1		0x04000000
#define MSM_LINUX_SIZE1		0x0C000000
#define MSM_LINUX_BASE2		0x20000000
#define MSM_LINUX_SIZE2		0x0BB00000
#define MSM_MEM_256MB_OFFSET	0x10000000

#define MSM_GPU_MEM_BASE	0x00100000
#define MSM_GPU_MEM_SIZE	0x00300000

#define MSM_RAM_CONSOLE_BASE	0x00500000
#define MSM_RAM_CONSOLE_SIZE	0x00100000

#define MSM_PMEM_ADSP_BASE  	0x2BB00000
#define MSM_PMEM_ADSP_SIZE	0x01C00000 /* for 8M(4:3) + gpu effect */
#define PMEM_KERNEL_EBI1_BASE   0x2D700000
#define PMEM_KERNEL_EBI1_SIZE   0x00600000

#define MSM_PMEM_CAMERA_BASE	0x2DD00000
#define MSM_PMEM_CAMERA_SIZE	0x00000000

#define MSM_PMEM_MDP_BASE	0x2DD00000
#define MSM_PMEM_MDP_SIZE	0x02000000

#define MSM_FB_BASE		0x2FD00000
#define MSM_FB_SIZE		0x00300000
#endif

#define MSM_RAM_CONSOLE_BASE	MSM_HTC_RAM_CONSOLE_PHYS
#define MSM_RAM_CONSOLE_SIZE	MSM_HTC_RAM_CONSOLE_SIZE

/* GPIO definition */


/* Direct Keys */
#define VERDI_LTE_GPIO_KEY_POWER          (125)
#define VERDI_LTE_VOL_UP                  (104)
#define VERDI_LTE_VOL_DN                  (103)

/* Battery */
#define VERDI_LTE_GPIO_MBAT_IN		   (61)

/* Wifi */
#define VERDI_LTE_GPIO_WIFI_IRQ              (46)
#define VERDI_LTE_GPIO_WIFI_SHUTDOWN_N       (57)
/* Sensors */
#define VERDI_LTE_ECOMPASS_INT           (128)
#define VERDI_LTE_GSENSOR_INT           (127)

#define VERDI_LTE_SENSOR_I2C_SDA           (72)
#define VERDI_LTE_SENSOR_I2C_SCL           (73)

/* Microp */

/* TP */
#define VERDI_LTE_TP_I2C_SDA           (51)
#define VERDI_LTE_TP_I2C_SCL           (52)
#define VERDI_LTE_TP_ATT_N             (44)
#define VERDI_LTE_TP_ATT_N_XB       (51)
#define VERDI_LTE_SPI_ENABLE_XB	(156)

/* General */
#define VERDI_LTE_GENERAL_I2C_SDA           (59)
#define VERDI_LTE_GENERAL_I2C_SCL           (60)
#define VERDI_LTE_CAM_ID           (102)

/* LCD */
#define VERDI_LTE_LVDS_ON		(51)
#define VERDI_LTE_LVDS_ON_XB	(126)
#define GPIO_LCM_RST_N			(66)
#define GPIO_LCM_ID			(50)
#define GPIO_LCM_DCR		(157)

/* Audio */
#define VERDI_LTE_AUD_LDO_SEL	        (116)
#define VERDI_LTE_AUD_RX_MCLK1          (108)
#define VERDI_LTE_AUD_QTR_TX_MCLK1      (109)
#define VERDI_LTE_AUD_QTR_TX_I2S_SD2    (110)
#define VERDI_LTE_AUD_SPI_CS            (126)
#define VERDI_LTE_AUD_SPI_CS_XB         (139)
#define VERDI_LTE_AUD_QTR_RESET         (50)
#define VERDI_LTE_AUD_A1028_RESET       (43)
#define VERDI_LTE_AUD_A1028_INT         (49)
#define VERDI_LTE_AUD_A1028_WAKEUP      (105)
#define VERDI_LTE_AUD_RX_CLK2           (119)
#define VERDI_LTE_AUD_RX_WS2            (120)
#define VERDI_LTE_AUD_RX_SD2            (121)
#define VERDI_LTE_AUD_RX_MCLK2          (122)

/* BT */
#define VERDI_LTE_GPIO_BT_HOST_WAKE      (45)
#define VERDI_LTE_GPIO_BT_UART1_TX       (53)
#define VERDI_LTE_GPIO_BT_UART1_RX       (54)
#define VERDI_LTE_GPIO_BT_UART1_CTS      (55)
#define VERDI_LTE_GPIO_BT_UART1_RTS      (56)
#define VERDI_LTE_GPIO_BT_SHUTDOWN_N     (28)
#define VERDI_LTE_GPIO_BT_CHIP_WAKE      (130)
#define VERDI_LTE_GPIO_BT_RESET_N        (142)
#define VERDI_LTE_GPIO_BT_PCM_OUT        (111)
#define VERDI_LTE_GPIO_BT_PCM_IN         (112)
#define VERDI_LTE_GPIO_BT_PCM_SYNC       (113)
#define VERDI_LTE_GPIO_BT_PCM_CLK        (114)

/* USB */
#define VERDI_LTE_GPIO_USB_ID        (52)
#define VERDI_LTE_GPIO_MHL_RESET        (70)
#define VERDI_LTE_GPIO_MHL_USB_SW        (154)
#define VERDI_LTE_GPIO_CABLE_IN1_XD	(123)

/* Camera */

/* SPI */
#define VERDI_LTE_SPI_DO                 (33)
#define VERDI_LTE_SPI_DI                 (34)
#define VERDI_LTE_SPI_CS                 (35)
#define VERDI_LTE_SPI_CLK                (36)

/* CAMERA SPI */
#define VERDI_LTE_SP3D_SPI_DO                 (62)
#define VERDI_LTE_SP3D_SPI_DI                 (63)
#define VERDI_LTE_SP3D_SPI_CS                 (64)
#define VERDI_LTE_SP3D_SPI_CLK                (65)
#define VERDI_LTE_CAM_SPI_DO                 (62)
#define VERDI_LTE_CAM_SPI_DI                 (63)
#define VERDI_LTE_CAM_SPI_CS                 (64)
#define VERDI_LTE_CAM_SPI_CLK                (65)


/* CAMERA GPIO */
#define VERDI_LTE_CAM_I2C_SDA           (47)
#define VERDI_LTE_CAM_I2C_SCL           (48)

#define VERDI_LTE_SP3D_GATE              (107)
#define VERDI_LTE_SP3D_CORE_GATE         (58)
#define VERDI_LTE_SP3D_SYS_RST           (102)
#define VERDI_LTE_SP3D_PDX               (137)
#define VERDI_LTE_SP3D_VCM			(158)
#define VERDI_LTE_WEBCAM_RST		(138)
#define VERDI_LTE_WEBCAM_STB		(101)
#define VERDI_LTE_CLK_SWITCH 		(124)
#define VERDI_LTE_MCLK		(32)
#define VERDI_LTE_CAM_INT 		(106)
#define VERDI_LTE_CAM_GATE              (107)
#define VERDI_LTE_CAM_CORE_GATE         (58)
#define VERDI_LTE_CAM_MIX_RST           (102)
#define VERDI_LTE_CAM_RST               (137)
#define VERDI_LTE_CAM_VCM			(158)

#define VERDI_LTE_CAM1_PWD			(137)
#define VERDI_LTE_CAM1_VCM_PWD		(58)


/* Flashlight */
#define VERDI_LTE_FLASH_EN               (29)
#define VERDI_LTE_TORCH_EN               (30)

/* Vibrator */
#define VERDI_LTE_GPIO_VIBRATOR_ON       (155)

/* Accessory */
#define VERDI_LTE_GPIO_AUD_HP_DET	(31)
#define VERDI_H2W_IO1_CLK		(118)
#define VERDI_H2W_IO2_DAT		(117)
#define VERDI_H2W_CABLE_IN1		(123)
#define VERDI_H2W_CABLE_IN2		(153)

/* SPI */
#define VERDI_LTE_SPI_DO                 (33)
#define VERDI_LTE_SPI_DI                 (34)
#define VERDI_LTE_SPI_CS                 (35)
#define VERDI_LTE_SPI_CLK                (36)


/* LTE */
#define VERDI_LTE_AP2MDM_STATUS         (136)
#define VERDI_LTE_MDM2AP_STATUS         (134)
#define VERDI_LTE_MDM2AP_WAKEUP          (40)
#define VERDI_LTE_MDM2AP_ERRFATAL       (133)
#define VERDI_LTE_AP2MDM_ERRFATAL        (93)

#define VERDI_LTE_AP2MDM_PMIC_RESET_N   (131)
#define VERDI_LTE_AP2MDM_KPDPWR_N       (132)
#define VERDI_LTE_AP2PMIC_TMPNI_CKEN    (141)

#define VERDI_LTE_MDM2AP_VDDMIN		(140)
#define VERDI_LTE_MDM2AP_SYNC           (129)
#define VERDI_LTE_AP2MDM_WAKEUP         (135)
#define VERDI_LTE_MDM2AP_VFR             (94)

/* PMIC */

/* PMIC GPIO definition */
#define PMGPIO(x) (x-1)
#define VERDI_LTE_H2W_PWR_EN		PMGPIO(1)
#define VERDI_LTE_SAR_RST		PMGPIO(3)
#define VERDI_LTE_AUD_STEREO_REC     PMGPIO(5)
#define VERDI_LTE_AUD_MIC_SEL        PMGPIO(14)
#define VERDI_LTE_AUD_REMO_EN_PM     PMGPIO(15)
#define VERDI_LTE_V_LCM_3V3_EN_1_XC       PMGPIO(16)
#define VERDI_LTE_AUD_HP_EN          PMGPIO(18)
#define VERDI_LTE_AUD_SPK1_EN        PMGPIO(19)
#define VERDI_LTE_AUD_WSPK_EN        PMGPIO(20)
#define VERDI_LTE_AUD_CODEC_RST      PMGPIO(21)
#define VERDI_LTE_9V_AC_DETECT      PMGPIO(22)
#define VERDI_LTE_TP_RST             PMGPIO(23)
#define VERDI_LTE_GREEN_LED          PMGPIO(24)
#define VERDI_LTE_AMBER_LED          PMGPIO(25)
#define VERDI_LTE_CAMERA_LED	PMGPIO(28)
#define VERDI_LTE_SPI_ENABLE         PMGPIO(25)
#define VERDI_LTE_CHG_STAT		PMGPIO(33)
#define VERDI_LTE_PLS_INT            PMGPIO(35)
#define VERDI_LTE_AUD_REMO_PRES      PMGPIO(37)
#define VERDI_LTE_WIFI_BT_SLEEP_CLK  PMGPIO(38)

#define VERDI_LTE_CAPSENSE_INT_XB         PMGPIO(3)
#define VERDI_LTE_V_LCM_3V3_SYNC          PMGPIO(3)
#define VERDI_LTE_LCM_BL_EN               PMGPIO(4)
#define VERDI_LTE_LCM_EN                  PMGPIO(6)
#define VERDI_LTE_V_LED_EN                PMGPIO(13)
#define VERDI_LTE_LCM_BL_PWM              PMGPIO(26)
#define VERDI_LTE_LCM_DCR_EN_XB	PMGPIO(36)
#define VERDI_LTE_V_LCM_3V3_EN            PMGPIO(40)

#define VERDI_LTE_CHG_5VAC		  PMGPIO(9)
#define VERDI_LTE_CHG_BATT		  PMGPIO(10)
#define VERDI_LTE_CHG_USB		  PMGPIO(11)
#define VERDI_LTE_CHG_EN		  PMGPIO(12)
#define VERDI_LTE_AC_DET		  PMGPIO(22)
#define VERDI_LTE_TORCH_SET1	          PMGPIO(24)
#define VERDI_LTE_TORCH_SET1_XB	          PMGPIO(35)
#define VERDI_LTE_TORCH_SET2	 	  PMGPIO(31)
#define VERDI_LTE_SP3D_PMIC_AVDD          PMGPIO(7)

int __init verdi_lte_init_mmc(void);
void __init verdi_lte_audio_init(void);
int __init verdi_lte_init_keypad(void);
int __init verdi_lte_wifi_init(void);

#endif /* __ARCH_ARM_MACH_MSM_BOARD_VERDI_LTE_H */
