/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/spmi.h>
#include <linux/err.h>

#define VMBMS_BATT_PRES_STATUS	   0x1208
#define BAT_PRES_BIT			   BIT(7)

#if defined(CONFIG_BOARD_HELEN)
#include <linux/module.h>
#define GPIO_RED_LED_CONTROL		1
#define MPP2_MODE_CTL_ADDRESS		0xA140
#define MPP2_MODE_CTL_VALUE		0x61
#define MPP2_EN_CTL_ADDRESS		0xA146
#define MPP2_EN_CTL_ENABLE_VALUE	0x80
#define MPP2_EN_CTL_DISABLE_VALUE	0x00
#define MPP2_SINK_CTL_ADDRESS		0xA14C
#define MPP2_SINK_CTL_VALUE		0x00
#endif

struct spmi_lite {
	struct spmi_device *spmi;
	u16				reg_bat_pres;
};

struct spmi_lite *chip;

static int qpnp_read_wrapper(struct spmi_lite *chip, u8 *val,
					u16 base, int count)
{
	int rc;
	struct spmi_device *spmi = chip->spmi;

	rc = spmi_ext_register_readl(spmi->ctrl, spmi->sid, base, val, count);
	if (rc)
		pr_err("SPMI read failed rc=%d\n", rc);

	return rc;
}

#if defined(GPIO_RED_LED_CONTROL)
static int qpnp_write_wrapper(struct spmi_lite *chip, u8 *val,
			u16 base, int count)
{
	int rc;
	struct spmi_device *spmi = chip->spmi;

	rc = spmi_ext_register_writel(spmi->ctrl, spmi->sid, base, val, count);
	if (rc)
		pr_err("SPMI write failed rc=%d\n", rc);

	return rc;
}
#endif

bool spmi_lite_is_battery_present(void)
{
	  int rc;
	  u8 batt_pres;

	  rc = qpnp_read_wrapper(chip, &batt_pres,
				chip->reg_bat_pres, 1);
	  pr_debug("reg_bat_pres=%d\n", batt_pres);
	  if (!rc && (batt_pres & BAT_PRES_BIT))
			return true;
	  else
			return false;
}

#if defined(GPIO_RED_LED_CONTROL)

static void zte_misc_register_green_led_init(void)
{
	int rc;
	u8 reg_wr = MPP2_MODE_CTL_VALUE;
	u8 reg_rd;

	rc = qpnp_write_wrapper(chip, &reg_wr, MPP2_MODE_CTL_ADDRESS, 1);
	if (rc) {
		pr_err("Write failed MPP2_MODE_CTL_ADDRESS = %03X, reg_wr = %02x, rc = %d\n",
				MPP2_MODE_CTL_ADDRESS, reg_wr, rc);
	}

	rc = qpnp_read_wrapper(chip, &reg_rd, MPP2_MODE_CTL_ADDRESS, 1);
	if (rc) {
		pr_err("Read failed MPP2_MODE_CTL_ADDRESS = %03X, reg_rd = %02x, rc = %d\n",
				MPP2_MODE_CTL_ADDRESS, reg_rd, rc);
	}
}

void zte_misc_green_led_control(bool value)
{
	int rc;
	u8 reg_wr;
	u8 reg_rd;

	zte_misc_register_green_led_init();

	if (value == true) {
		reg_wr = MPP2_EN_CTL_ENABLE_VALUE;
		rc = qpnp_write_wrapper(chip, &reg_wr, MPP2_EN_CTL_ADDRESS, 1);
		if (rc) {
			pr_err("Write failed MPP2_EN_CTL_ADDRESS = %03X, reg_wr = %02x, rc = %d\n",
					MPP2_EN_CTL_ADDRESS, reg_wr, rc);
		}

		reg_wr = MPP2_SINK_CTL_VALUE;
		rc = qpnp_write_wrapper(chip, &reg_wr, MPP2_SINK_CTL_ADDRESS, 1);
		if (rc) {
			pr_err("Write failed MPP2_SINK_CTL_ADDRESS = %03X, reg_wr = %02x, rc = %d\n",
					MPP2_SINK_CTL_ADDRESS, reg_wr, rc);
		}
	} else {
		reg_wr = MPP2_EN_CTL_DISABLE_VALUE;
		rc = qpnp_write_wrapper(chip, &reg_wr, MPP2_EN_CTL_ADDRESS, 1);
		if (rc) {
			pr_err("Write failed MPP2_EN_CTL_ADDRESS = %03X, reg_wr = %02x, rc = %d\n",
					MPP2_EN_CTL_ADDRESS, reg_wr, rc);
		}

		rc = qpnp_read_wrapper(chip, &reg_rd, MPP2_EN_CTL_ADDRESS, 1);
		if (rc) {
			pr_err("Read failed MPP2_EN_CTL_ADDRESS = %03X, reg_rd = %02x, rc = %d\n",
					MPP2_EN_CTL_ADDRESS, reg_rd, rc);
		}
	}
}
EXPORT_SYMBOL(zte_misc_green_led_control);

static int green_led_mode = 0;
static int set_green_led_mode(const char *val, struct kernel_param *kp)
{
	int ret;

	zte_misc_register_green_led_init();
	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("Fail to setting value %d\n", ret);
		return ret;
	}

	if (green_led_mode == 0) {
		zte_misc_green_led_control(false);
	} else if (green_led_mode == 1) {
		zte_misc_green_led_control(true);
	}
	return 0;
}
module_param_call(green_led_mode, set_green_led_mode, param_get_int, &green_led_mode, 0644);

#endif

static int spmi_lite_probe(struct spmi_device *spmi)
{
	pr_info("%s enter\n", __func__);

	chip = devm_kzalloc(&spmi->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->spmi = spmi;
	chip->reg_bat_pres = VMBMS_BATT_PRES_STATUS;

	pr_info("batt_present=%d\n", spmi_lite_is_battery_present());
	return 0;
}

static int spmi_lite_remove(struct spmi_device *spmi)
{

	return 0;
}

static const struct of_device_id spmi_match_table[] = {
	{	.compatible = "zte,spmi-lite",
	},
	{}
};

static struct spmi_driver spmi_lite_driver = {
	.driver		= {
		.name	= "zte,spmi-lite",
		.of_match_table = spmi_match_table,
	},
	.probe		= spmi_lite_probe,
	.remove		= spmi_lite_remove,
};

static int __init spmi_lite_init(void)
{
	return spmi_driver_register(&spmi_lite_driver);
}
fs_initcall(spmi_lite_init);

static void __exit spmi_lite_exit(void)
{
	return spmi_driver_unregister(&spmi_lite_driver);
}
module_exit(spmi_lite_exit);

MODULE_DESCRIPTION("zte spmi lite driver");
MODULE_LICENSE("GPL v2");

