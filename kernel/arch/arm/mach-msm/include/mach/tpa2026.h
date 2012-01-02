/*
 * Definitions for tpa2026 speaker amp chip.
 */
#ifndef TPA2026_H
#define TPA2026_H

#include <linux/ioctl.h>

#define TPA2026_I2C_NAME "tpa2026"
#define SPKR_OUTPUT 0
#define HEADSET_OUTPUT 1
#define DUAL_OUTPUT 2
#define HANDSET_OUTPUT 3
#define MODE_CMD_LEM 9

struct tpa2026_platform_data {
	uint32_t gpio_tpa2026_spk_en;
	unsigned char spkr_cmd[7];
	unsigned char hsed_cmd[7];
	unsigned char rece_cmd[7];
};

struct tpa2026_config_data {
	unsigned int data_len;
	unsigned int mode_num;
	unsigned char *cmd_data;  /* [mode][mode_kind][reserve][cmds..] */
};

enum TPA2026_Mode {
	TPA2026_MODE_OFF,
	TPA2026_MODE_PLAYBACK_SPKR,
	TPA2026_MODE_PLAYBACK_HEADSET,
	TPA2026_MODE_RING,
	TPA2026_MODE_VOICECALL_SPKR,
	TPA2026_MODE_VOICECALL_HEADSET,
	TPA2026_MODE_FM_SPKR,
	TPA2026_MODE_FM_HEADSET,
	TPA2026_MODE_HANDSET,
	TPA2026_MAX_MODE
};
#define TPA2026_IOCTL_MAGIC 'a'
#define TPA2026_SET_CONFIG	_IOW(TPA2026_IOCTL_MAGIC, 0x01,	unsigned)
#define TPA2026_READ_CONFIG	_IOW(TPA2026_IOCTL_MAGIC, 0x02, unsigned)
#define TPA2026_SET_MODE        _IOW(TPA2026_IOCTL_MAGIC, 0x03, unsigned)
#define TPA2026_SET_PARAM       _IOW(TPA2026_IOCTL_MAGIC, 0x04, unsigned)
#define TPA2026_SET_MFG_LOOPBACK_ON _IOW(TPA2026_IOCTL_MAGIC, 0x05, unsigned)
#define TPA2026_SET_MFG_LOOPBACK_OFF _IOW(TPA2026_IOCTL_MAGIC, 0x06, unsigned)
#define TPA2026_WRITE_REG	_IOW(TPA2026_IOCTL_MAGIC, 0x07,	unsigned)
#define TPA2026_SET_MFG_LOOPBACK_L _IOW(TPA2026_IOCTL_MAGIC, 0x08, unsigned)
#define TPA2026_SET_MFG_LOOPBACK_R _IOW(TPA2026_IOCTL_MAGIC, 0x09, unsigned)
#define TPA2026_SET_MFG_LOOPBACK_STOP _IOW(TPA2026_IOCTL_MAGIC, 0x0a, unsigned)
#define TPA2026_RESET	_IOW(TPA2026_IOCTL_MAGIC, 0x0b, unsigned)

void set_speaker_amp_2026(int on);
void set_headset_amp_2026(int on);
void set_speaker_headset_amp_2026(int on);
void set_handset_amp_2026(int on);
#endif

