/* arch/arm/mach-msm/htc_simhotswap.c
 *
 * Copyright (C) 2011 HTC Corporation.
 * Author: YaWen Su <YaWen_Su@htc.com>
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
#include <linux/init.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <mach/board.h>
#include <mach/board_htc.h>

#define VERDI_LTE_SIM_DETECT 61
static uint32_t sim_gpio_table[] = {
	GPIO_CFG(VERDI_LTE_SIM_DETECT, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static int
htc_hotswap_open( struct inode * inode, struct file * file )
{
	return 0;
}

const struct file_operations htc_hotswap_fops = {
	.owner = THIS_MODULE,
	.open = htc_hotswap_open,
};

static struct miscdevice sim_hotswap_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "htc_simhotswap",
	.fops = &htc_hotswap_fops,
};

struct htc_simhotswap_info {
	struct kobject simhotswap_kobj;
	struct work_struct hotswap_work;
	struct workqueue_struct *hotswap_wq;
};

static struct htc_simhotswap_info htc_hotswap_info;
static struct kset *htc_hotswap_kset;

static void htc_simhotswap_kobject_release(struct kobject *kobj)
{
	printk(KERN_ERR "htc_hotswap_kobject_release.\n");
	return;
}

static struct kobj_type htc_hotswap_ktype = {
	.release = htc_simhotswap_kobject_release,
};

static void hotswap_work_func(struct work_struct *work)
{
	int status;
	char message[20] = "SIMHOTSWAP=";
	char *envp[] = { message, NULL };
	status = gpio_get_value(VERDI_LTE_SIM_DETECT);
	pr_info("VERDI_LTE_SIM_DETECT = %d\n", status);

	if (status)
		strncat(message, "REMOVE", 6);
	else
		strncat(message, "INSERT", 6);

	kobject_uevent_env(&htc_hotswap_info.simhotswap_kobj, KOBJ_CHANGE, envp);

	return;
}

static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

static irqreturn_t verdi_lte_sim_detect_irq(int irq, void *dev_id)
{
	schedule_work(&htc_hotswap_info.hotswap_work);
	return IRQ_HANDLED;
}

static int htc_simhotswap_probe(struct platform_device *pdev)
{
	int ret = 0;

	config_gpio_table(sim_gpio_table, ARRAY_SIZE(sim_gpio_table));
	ret = request_irq(gpio_to_irq(VERDI_LTE_SIM_DETECT),
			verdi_lte_sim_detect_irq,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"sim_detect", NULL);
	if (ret) {
		pr_err("%s:Failed to request irq, ret=%d\n", __func__, ret);
	}

	INIT_WORK(&htc_hotswap_info.hotswap_work, hotswap_work_func);
	htc_hotswap_info.hotswap_wq = create_singlethread_workqueue("htc_simhotswap");

	ret = misc_register(&sim_hotswap_misc);
	if (ret) {
		pr_err("failed to register misc device!\n");
		goto fail;
	}

	htc_hotswap_kset = kset_create_and_add("event", NULL,
			kobject_get(&sim_hotswap_misc.this_device->kobj));
	if (!htc_hotswap_kset) {
		ret = -ENOMEM;
		goto fail;
	}

	htc_hotswap_info.simhotswap_kobj.kset = htc_hotswap_kset;

	ret = kobject_init_and_add(&htc_hotswap_info.simhotswap_kobj,
			&htc_hotswap_ktype, NULL, "simhotswap");
	if (ret) {
		kobject_put(&htc_hotswap_info.simhotswap_kobj);
		goto fail;
	}
	pr_info("htc_simhotswap_probe(): finish\n");

fail:
	return ret;
}

static struct platform_driver htc_simhotswap_driver = {
	.probe	= htc_simhotswap_probe,
	.driver	= {
		.name	= "htc_simhotswap",
		.owner	= THIS_MODULE,
	},
};

static int __init sim_hotswap_init(void)
{
	platform_driver_register(&htc_simhotswap_driver);
	return 0;
}

module_init(sim_hotswap_init);
MODULE_DESCRIPTION("HTC SIMHOTSWAP Driver");
