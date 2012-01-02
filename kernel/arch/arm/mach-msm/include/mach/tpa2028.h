/*
 * Definitions for tpa2028 speaker amp chip.
 */
#ifndef TPA2028_H
#define TPA2028_H

#include <linux/ioctl.h>

#define TPA2028_I2C_NAME "tpa2028"
#define SPKR_OUTPUT 0
#define HEADSET_OUTPUT 1
#define DUAL_OUTPUT 2
#define HANDSET_OUTPUT 3
#define MODE_CMD_LEM 9

struct tpa2028_platform_data {
	uint32_t gpio_tpa2028_spk_en;
	unsigned char spkr_cmd[7];
	unsigned char hsed_cmd[7];
	unsigned char rece_cmd[7];
	unsigned char off_cmd[7];
};

struct tpa2028_config_data {
	unsigned int data_len;
	unsigned int mode_num;
	unsigned char *cmd_data;  /* [mode][mode_kind][reserve][cmds..] */
};

enum TPA2028_Mode {
	TPA2028_MODE_OFF,
	TPA2028_MODE_PLAYBACK_SPKR,
	TPA2028_MODE_PLAYBACK_HEADSET,
	TPA2028_MODE_RING,
	TPA2028_MODE_VOICECALL_SPKR,
	TPA2028_MODE_VOICECALL_HEADSET,
	TPA2028_MODE_FM_SPKR,
	TPA2028_MODE_FM_HEADSET,
	TPA2028_MODE_HANDSET,
	TPA2028_MAX_MODE
};
#define TPA2028_IOCTL_MAGIC 'a'
#define TPA2028_SET_CONFIG	_IOW(TPA2028_IOCTL_MAGIC, 0x01,	unsigned)
#define TPA2028_READ_CONFIG	_IOW(TPA2028_IOCTL_MAGIC, 0x02, unsigned)
#define TPA2028_SET_MODE        _IOW(TPA2028_IOCTL_MAGIC, 0x03, unsigned)
#define TPA2028_SET_PARAM       _IOW(TPA2028_IOCTL_MAGIC, 0x04, unsigned)
#define TPA2028_SET_MFG_LOOPBACK_ON _IOW(TPA2028_IOCTL_MAGIC, 0x06, unsigned)
#define TPA2028_SET_MFG_LOOPBACK_OFF _IOW(TPA2028_IOCTL_MAGIC, 0x05, unsigned)
#define TPA2028_WRITE_REG	_IOW(TPA2028_IOCTL_MAGIC, 0x07,	unsigned)
#define TPA2028_SET_MFG_LOOPBACK_STOP _IOW(TPA2028_IOCTL_MAGIC, 0x0a, unsigned)
#define TPA2028_RESET	_IOW(TPA2028_IOCTL_MAGIC, 0x0b, unsigned)

void set_speaker_amp_2028(int on);
void set_headset_amp_2028(int on);
void set_speaker_headset_amp_2028(int on);
void set_handset_amp_2028(int on);
#endif

