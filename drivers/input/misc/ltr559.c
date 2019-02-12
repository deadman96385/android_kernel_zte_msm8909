/* Lite-On LTR-559ALS Android / Linux Driver
 *
 * Copyright (C) 2013-2014 Lite-On Technology Corp (Singapore)
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2,	as published by the Free Software Foundation,	and
 * may be copied,	distributed,	and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,	but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 */


#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/gfp.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/types.h>
#include <asm/setup.h>
#include <linux/version.h>
#include <linux/sensors.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/sysfs.h>

#include "ltr559.h"
#define XTALK_HI_LIMIT		700
#define TARGET_XTALK		(2/3)

/*copy from touchscreen_fw.h*/
enum TOUCH_MOUDLE {
	TPK = 0,
	TRULY,
	SUCCESS,
	OFILM,
	LEAD,
	WINTEK,
	LAIBAO,
	CMI,
	ECW,
	GOWORLD,
	BAOMING,
	JUNDA,
	JIAGUAN,
	MUDONG,
	EACHOPTO,
	AVC,
	LIANCHUANG,
	LCE,
	UNKNOWN = 0xff
};
int syna_touch_module_for_lsensor = UNKNOWN;
int fts_touch_module_for_lsensor = UNKNOWN;

static int is_ltr559_probe_succ_flag = 0;

static int ltr559_ps_detection_threshold = 0;
static int ltr559_ps_hsyteresis_threshold = 0;
/*static int ltr559_ps_cross_talk = 0;
static int ltr559_ps_startup_cross_talk = 0;*/
static atomic_t ltr559_resume_poweron_flag;
static unsigned long ltr559_poweron_endt = 0;

#define LTR559_ALS_SOFT_GAIN_ZTE	1

#if CALLING_DYNAMIC_CALI
static int last_min_value = 2047;
#define MAX_ELM_PS_1 20
static unsigned int record_ps_1[MAX_ELM_PS_1];
static int rct_ps_1 = 0, full_ps_1 = 0;
static long ps_sum_1 = 0;
static int j_ps = 0;
static int oil_far_cal = 0;
static int oil_close = 0;
static int intr_flag_value = 0;
#endif

#define LTR559_TAG					" [ltr559] "
#define LTR559_DBG(fmt,	args...)	printk(LTR559_TAG fmt,	##args)
#define LTR559_DBG2(fmt,	args...)

#if CALLING_DYNAMIC_CALI
static int ltr559_dynamic_calibrate(void);
#else
static int ltr559_calibrate_dial(void);
#endif
static int ltr559_poweron_reg_config(void);
/*static uint16_t offset_cancellation(uint16_t);*/
static void check_prox_mean(int, int *, int *);
uint16_t getCrosstalk(uint16_t);
int updateThreshold(uint16_t);
static int als_enable_falg = 0;

/*
 * Magic Number
 * ============
 * Refer to file ioctl-number.txt for allocation
 */
#define LTR559_IOCTL_MAGIC		'c'

/* IOCTLs for ltr559 device */
#define LTR559_IOCTL_PS_ENABLE		_IOR(LTR559_IOCTL_MAGIC,	1,	int *)
#define LTR559_IOCTL_PS_GET_ENABLED	_IOW(LTR559_IOCTL_MAGIC,	2,	int *)
#define LTR559_IOCTL_ALS_ENABLE		_IOR(LTR559_IOCTL_MAGIC,	3,	int *)
#define LTR559_IOCTL_ALS_GET_ENABLED	_IOW(LTR559_IOCTL_MAGIC,	4,	int *)

/* Add for dumping reg informaton */
/* #define LTR559_REG_DUMP_ENABLE */

struct ltr559_data {
	/* Device */
	struct i2c_client *i2c_client;
	struct regulator *vdd;
	struct regulator *vcc_i2c;
	struct input_dev *als_input_dev;
	struct input_dev *ps_input_dev;
	struct workqueue_struct *als_wq;
	struct delayed_work	als_dwork; /* for ALS polling */
	/*struct wake_lock ps_wake_lock;*/
	struct mutex bus_lock;
	struct sensors_classdev als_cdev;
	struct sensors_classdev ps_cdev;

	/* control flag from HAL */
	unsigned int enable_ps_sensor;
	unsigned int enable_als_sensor;

	/* Device mode * 0 = ALS * 1 = PS */
	uint8_t mode;

	/* ALS */
	uint8_t als_enable_flag;
	uint8_t als_suspend_enable_flag;
	uint8_t als_irq_flag;
	uint8_t als_opened;
	uint16_t als_lowthresh;
	uint16_t als_highthresh;
	uint16_t default_als_lowthresh;
	uint16_t default_als_highthresh;
	uint16_t *adc_levels;
	/* Flag to suspend ALS on suspend or not */
	uint8_t disable_als_on_suspend;
	unsigned int als_poll_delay;

	/* PS */
	uint8_t ps_enable_flag;
	uint8_t ps_suspend_enable_flag;
	uint8_t ps_irq_flag;
	uint8_t ps_opened;
	uint16_t default_ps_lowthresh;
	uint16_t default_ps_highthresh;
#if CALLING_DYNAMIC_CALI
	atomic_t ps_persist_val_high;
	atomic_t ps_persist_val_low;
#endif
	/* Flag to suspend PS on suspend or not */
	uint8_t disable_ps_on_suspend;

	/* LED */
	int led_pulse_freq;
	int led_duty_cyc;
	int led_peak_curr;
	int led_pulse_count;

	/* Interrupt */
	int irq;
	int gpio_int_no;
	int is_suspend;
	/*bool power_enabled;*/
#ifdef ZTE_LTR559_PINCTRL
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_default;
	struct pinctrl_state *pin_sleep;
#endif
};

struct ltr559_data *sensor_info;
static struct sensors_classdev sensors_light_cdev = {
	.name = "ltr559-light",
	.vendor = "liteon",
	.version = 1,
	.handle = SENSORS_LIGHT_HANDLE,
	.type = SENSOR_TYPE_LIGHT,
	.max_range = "30000",
	.resolution = "0.0125",
	.sensor_power = "0.20",
	.min_delay = 1000,	/* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static struct sensors_classdev sensors_proximity_cdev = {
	.name = "ltr559-proximity",
	.vendor = "liteon",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "5",
	.resolution = "5.0",
	.sensor_power = "3",
	.min_delay = 1000,	/* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

#define	PS_MAX_INIT_KEPT_DATA_COUNTER		8
#define	PS_MAX_MOV_AVG_KEPT_DATA_CTR		7

uint16_t winfac1 = 70;
uint16_t winfac2 = 70;
uint16_t winfac3 = 20;

uint8_t eqn_prev;
uint8_t ratio_old;
uint16_t ps_init_kept_data[PS_MAX_INIT_KEPT_DATA_COUNTER];
uint16_t ps_ct_avg;
uint8_t ps_grabData_stage;
uint32_t ftn_init;
uint32_t ftn_final;
uint32_t ntf_final;
uint16_t lux_val_prev = 100;
uint8_t ps_kept_data_counter;
uint16_t ps_movavg_data[PS_MAX_MOV_AVG_KEPT_DATA_CTR];
uint8_t ps_movavg_data_counter;
uint16_t ps_movct_avg;
/*uint16_t ps_thresh_hi,	ps_thresh_lo;*/
static int8_t cal_als_winfac(void)
{
	LTR559_DBG("%s: touch_module=%d\n", __func__, syna_touch_module_for_lsensor);

#if defined(CONFIG_BOARD_NOAH)
	if (ECW == syna_touch_module_for_lsensor) {
		winfac1 = 80;
		winfac2 = 80;
		winfac3 = 20;
	} else if (SUCCESS == syna_touch_module_for_lsensor) {
		winfac1 = 88;
		winfac2 = 78;
		winfac3 = 20;
	} else {
		winfac1 = 70;
		winfac2 = 70;
		winfac3 = 20;
	}
#elif defined(CONFIG_BOARD_MIMIR)
	if (OFILM == syna_touch_module_for_lsensor) {
		winfac1 = 67;
		winfac2 = 70;
		winfac3 = 20;
	} else {
		winfac1 = 65;
		winfac2 = 65;
		winfac3 = 20;
	}
#elif defined(CONFIG_BOARD_CHAPEL)
	if (OFILM == syna_touch_module_for_lsensor) {
		winfac1 = 67;
		winfac2 = 70;
		winfac3 = 20;
	} else {
		winfac1 = 65;
		winfac2 = 65;
		winfac3 = 20;
	}
#elif defined(CONFIG_BOARD_XRAY50)
	if (OFILM == syna_touch_module_for_lsensor) {
		winfac1 = 70;
		winfac2 = 78;
		winfac3 = 20;
	} else {
		winfac1 = 70;
		winfac2 = 70;
		winfac3 = 20;
	}
#elif defined(CONFIG_BOARD_FORTUNE)
	if (JUNDA == syna_touch_module_for_lsensor) {
		winfac1 = 70;
		winfac2 = 56;
		winfac3 = 20;
	} else {
		winfac1 = 70;
		winfac2 = 70;
		winfac3 = 20;
	}
#elif (defined(CONFIG_BOARD_STARK) || defined(CONFIG_BOARD_MARTELL))
	if (LCE == syna_touch_module_for_lsensor) {
		winfac1 = 70;
		winfac2 = 88;
		winfac3 = 20;
	} else {
		winfac1 = 70;
		winfac2 = 70;
		winfac3 = 20;
	}
#elif defined(CONFIG_BOARD_LEMON)
	if (LIANCHUANG == fts_touch_module_for_lsensor) {
		winfac1 = 68;
		winfac2 = 70;
		winfac3 = 20;
	} else {
		winfac1 = 70;
		winfac2 = 70;
		winfac3 = 20;
	}
#else
		winfac1 = 70;
		winfac2 = 70;
		winfac3 = 20;
#endif
	return 0;
}

/* I2C Read */
/* take note ---------------------------------------
 for i2c read,	need to send the register address follwed by buffer over
 to register.
 There should not be a stop in between register address and buffer.
 There should not be release of lock in between register address and buffer.
 take note ---------------------------------------*/
static int8_t I2C_Read(uint8_t *rxData,	uint8_t length)
{
	int8_t index;
	struct i2c_msg data[] = {
		{
			.addr = sensor_info->i2c_client->addr,
			.flags = 0,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = sensor_info->i2c_client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};

	for (index = 0; index < I2C_RETRY; index++) {
		if (i2c_transfer(sensor_info->i2c_client->adapter,	data,	2) >  0)
			break;

		usleep(10000);
	}

	if (index >= I2C_RETRY) {
		pr_alert("%s I2C Read Fail !!!!\n", __func__);
		return -EIO;
	}

	return 0;
}


/* I2C Write */
static int8_t I2C_Write(uint8_t *txData,	uint8_t length)
{
	int8_t index;
	struct i2c_msg data[] = {
		{
			.addr = sensor_info->i2c_client->addr,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};

	for (index = 0; index < I2C_RETRY; index++) {
		if (i2c_transfer(sensor_info->i2c_client->adapter,	data,	1) >  0)
			break;

		usleep_range(10000 , 10500);
	}

	if (index >= I2C_RETRY) {
		pr_alert("%s I2C Write Fail !!!!\n", __func__);
		return -EIO;
	}

	return 0;
}


/* Set register bit */
static int8_t _ltr559_set_bit(struct i2c_client *client,	uint8_t set,
						uint8_t cmd,	uint8_t data)
{
	uint8_t buffer[2];
	uint8_t value;
	int8_t ret = 0;

	buffer[0] = cmd;
	ret = I2C_Read(buffer,	1);
	if (ret < 0) {
		dev_err(&client->dev,	"%s | 0x%02X", __func__,	buffer[0]);
		return ret;
	}

	value = buffer[0];

	if (set)
		value |= data;
	else
		value &= ~data;

	buffer[0] = cmd;
	buffer[1] = value;
	ret = I2C_Write(buffer,	2);
	if (ret < 0) {
		dev_err(&client->dev,	"%s | 0x%02X", __func__,	buffer[0]);
		return -EIO;
	}

	return ret;
}

/* Begin: Add for dumping reg informaton */
#ifdef LTR559_REG_DUMP_ENABLE
static int8_t ltr559_reg_dump(void)
{
	int8_t ret = 0, i;
	uint8_t buffer[30];

	LTR559_DBG("%s *********Start Dump**********\n", __func__);

	buffer[0] = 0x80;
	ret = I2C_Read(buffer, 22);
	for (i = 0; i < 22; i++)	{
		LTR559_DBG("%s: R[0x%X]=0x%X\n", __func__, 0X80+i, buffer[i]);
	}

	LTR559_DBG("%s *********End Dump**********\n", __func__);

	if (ret < 0) {
		LTR559_DBG("%s, I2C_Read error\n", __func__);
		return -EPERM;
	}

	return 0;
}
#endif
/* End: Add for dumping reg informaton */

static uint16_t lux_formula(uint16_t ch0_adc,	uint16_t ch1_adc,	uint8_t eqtn)
{
	uint32_t luxval = 0;
	uint32_t luxval_i = 0;
	uint32_t luxval_f = 0;
	uint16_t ch0_coeff_i = 0;
	uint16_t ch1_coeff_i = 0;
	uint16_t ch0_coeff_f = 0;
	uint16_t ch1_coeff_f = 0;
	int8_t ret;
	uint8_t gain = 1,	als_int_fac;
	uint8_t buffer[2];
	uint16_t win_fac = 0;
	int8_t fac = 1;

	buffer[0] = LTR559_ALS_PS_STATUS;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		pr_alert("%s | 0x%02X", __func__,	buffer[0]);
		return ret;
	}

	gain = (buffer[0] & 0x70);
	gain >>= 4;

	if (gain == 0)			/*gain 1*/
		gain = 1;
	else if (gain == 1)		/*gain 2*/
		gain = 2;
	else if (gain == 2)	/*gain 4*/
		gain = 4;
	else if (gain == 3)	/*gain 8*/
		gain = 8;
	else if (gain == 6)	/*gain 48*/
		gain = 48;
	else if (gain == 7)	/*gain 96*/
		gain = 96;

	buffer[0] = LTR559_ALS_MEAS_RATE;
	ret = I2C_Read(buffer,	1);
	if (ret < 0) {
		pr_alert("%s | 0x%02X", __func__,	buffer[0]);
		return ret;
	}
	als_int_fac = buffer[0] & 0x38;
	als_int_fac >>= 3;

	if (als_int_fac == 0)
		als_int_fac = 10;
	else if (als_int_fac == 1)
		als_int_fac = 5;
	else if (als_int_fac == 2)
		als_int_fac = 20;
	else if (als_int_fac == 3)
		als_int_fac = 40;
	else if (als_int_fac == 4)
		als_int_fac = 15;
	else if (als_int_fac == 5)
		als_int_fac = 25;
	else if (als_int_fac == 6)
		als_int_fac = 30;
	else if (als_int_fac == 7)
		als_int_fac = 35;

	if (eqtn == 1) {
		ch0_coeff_i = 1;
		ch1_coeff_i = 1;
		ch0_coeff_f = 7743;
		ch1_coeff_f = 1059;
		fac = 1;
		win_fac = winfac1;
		luxval_i = ((ch0_adc * ch0_coeff_i) +
			(ch1_adc * ch1_coeff_i)) * win_fac;
		luxval_f = (((ch0_adc * ch0_coeff_f) +
			(ch1_adc * ch1_coeff_f)) / 100) * win_fac;
		/*luxval = ((17743 * ch0_calc) + (11059 * ch1_adc));*/
		/*luxval = ((1.7743 * ch0_calc) + (1.1059 * ch1_adc)) /
			(gain * (als_int_fac / 10));*/
	} else if (eqtn == 2) {
		ch0_coeff_i = 4;
		ch1_coeff_i = 1;
		ch0_coeff_f = 2785;
		ch1_coeff_f = 696;
		win_fac = winfac2;
		if ((ch1_coeff_f * ch1_adc) < (ch0_adc * ch0_coeff_f)) {
			fac = 1;
			luxval_f = (((ch0_adc * ch0_coeff_f) -
			 (ch1_adc * ch1_coeff_f)) / 100) * win_fac;
		} else {
			fac = -1;
			luxval_f = (((ch1_adc * ch1_coeff_f) -
			 (ch0_adc * ch0_coeff_f)) / 100) * win_fac;
		}
		luxval_i = ((ch0_adc * ch0_coeff_i) - (ch1_adc * ch1_coeff_i))
				 * win_fac;
		/*luxval = ((42785 * ch0_calc) - (10696 * ch1_adc));*/
		/*luxval = ((4.2785 * ch0_calc) - (1.9548 * ch1_adc)) /
					 (gain * (als_int_fac / 10));*/
	} else if (eqtn == 3) {
		ch0_coeff_i = 0;
		ch1_coeff_i = 0;
		ch0_coeff_f = 5926;
		ch1_coeff_f = 1185;/*1300;*/
		fac = 1;
		win_fac = winfac3;
		luxval_i = ((ch0_adc * ch0_coeff_i) +
				(ch1_adc * ch1_coeff_i)) * win_fac;
		luxval_f = (((ch0_adc * ch0_coeff_f) +
				(ch1_adc * ch1_coeff_f)) / 100) * win_fac;
		/*luxval = ((5926 * ch0_calc) + (1185 * ch1_adc));*/
		/*luxval = ((0.5926 * ch0_calc) + (0.1185 * ch1_adc)) /
					 (gain * (als_int_fac / 10));*/
	} else if (eqtn == 4) {
		ch0_coeff_i = 0;
		ch1_coeff_i = 0;
		ch0_coeff_f = 0;
		ch1_coeff_f = 0;
		fac = 1;
		luxval_i = 0;
		luxval_f = 0;
		/*luxval = 0;*/
	}
	LTR559_DBG2("%s: gain=%d, als_int_fac=%d, fac=%d,	ch0_adc=%d, ch1_adc=%d\n",
	 __func__,	gain,	als_int_fac,	fac,	ch0_adc,	ch1_adc);
	LTR559_DBG2("%s: eqtn=%d, luxval_i=%d, luxval_f=%d\n", __func__,	eqtn,	luxval_i,	luxval_f);
	if (fac < 0)
		luxval = (luxval_i - (luxval_f  / 100)) / (gain * als_int_fac);
	else if (fac == 1)
		luxval = (luxval_i + ((fac) * luxval_f) /  100) / (gain * als_int_fac);

	return luxval;
}


static uint16_t ratioHysterisis(uint16_t ch0_adc, uint16_t ch1_adc)
{
	#define	RATIO_HYSVAL	2
	int ratio;
	uint8_t buffer[2],	eqn_now;
	int8_t ret;
	uint16_t ch0_calc;
	uint32_t luxval = 0;
	int abs_ratio_now_old;

	buffer[0] = LTR559_ALS_CONTR;
	ret = I2C_Read(buffer,	1);
	if (ret < 0) {
		pr_alert("%s | 0x%02X", __func__,	buffer[0]);
		return ret;
	}

	ch0_calc = ch0_adc;
	if ((buffer[0] & 0x20) == 0x20)
		ch0_calc = ch0_adc - ch1_adc;

	if ((ch1_adc + ch0_calc) == 0)
		ratio = 100;
	else
		ratio = (ch1_adc*100) / (ch1_adc + ch0_calc);

	if (ratio < 45)
		eqn_now = 1;
	else if ((ratio >= 45) && (ratio < 74))
		eqn_now = 2;
	else if ((ratio >= 74) && (ratio < 99))
		eqn_now = 3;
	else if (ratio >= 99)
		eqn_now = 4;

	LTR559_DBG2("%s: ratio=%d, eqn_now=%d, eqn_prev=%d, ratio_old=%d\n",
			 __func__,	ratio,	eqn_now,	eqn_prev, ratio_old);
	if (eqn_prev == 0) {
		luxval = lux_formula(ch0_calc,	ch1_adc,	eqn_now);
		ratio_old = ratio;
		eqn_prev = eqn_now;
	} else {
		if (eqn_now == eqn_prev) {
			luxval = lux_formula(ch0_calc,	ch1_adc,	eqn_now);
			ratio_old = ratio;
			eqn_prev = eqn_now;
		} else {
			abs_ratio_now_old = ratio - ratio_old;
			if (abs_ratio_now_old < 0)
				abs_ratio_now_old *= (-1);
			if (abs_ratio_now_old >  RATIO_HYSVAL) {
				luxval = lux_formula(ch0_calc,
				ch1_adc,	eqn_now);
				ratio_old = ratio;
				eqn_prev = eqn_now;
			} else {
				luxval = lux_formula(ch0_calc,
				ch1_adc,	eqn_prev);
			}
		}
	}

	return luxval;
}

static uint16_t read_als_adc_value(struct ltr559_data *ltr559)
{

	int8_t ret = -99;
	uint16_t value = -99;
	int ch0_val;
	int ch1_val;
	uint8_t gain,	value_temp;
	uint8_t buffer[4],	temp,	als_ps_status;

#define AGC_UP_THRESHOLD		40000
#define AGC_DOWN_THRESHOLD	5000
#define AGC_HYS					15
#define MAX_VAL					50000

	buffer[0] = LTR559_ALS_PS_STATUS;
	ret = I2C_Read(buffer,	1);
	als_ps_status = buffer[0];
	if (0 == (buffer[0] & 0X04)) {
		LTR559_DBG("%s: 0x8C=%X,	ALS data is not ready, return lux_val_prev=%d\n",
				 __func__,	buffer[0],	lux_val_prev);
		return lux_val_prev;
	}

	/* ALS */
	buffer[0] = LTR559_ALS_DATA_CH1_0;

	/* read data bytes from data regs */
	ret = I2C_Read(buffer,	4);

	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s | 0x%02X", __func__,	buffer[0]);

		return ret;
	}

	/* ALS Ch0 */
	ch0_val = (uint16_t)buffer[2] | ((uint16_t)buffer[3] << 8);
	dev_dbg(&ltr559->i2c_client->dev,
			"%s | als_ch0 value = 0x%04X\n", __func__,
				ch0_val);

	if (ch0_val >  ALS_MAX_MEASURE_VAL) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: ALS Value Error: 0x%X\n", __func__,
					ch0_val);
	}
	ch0_val &= ALS_VALID_MEASURE_MASK;
	/*input_report_abs(ltr559->als_input_dev,	ABS_MISC,	ch0_val);*/
	/*input_sync(ltr559->als_input_dev);*/

	/* ALS Ch1 */
	ch1_val = (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
	dev_dbg(&ltr559->i2c_client->dev,
			"%s | als_ch1 value = 0x%04X\n", __func__,
				ch1_val);

	if (ch1_val >  ALS_MAX_MEASURE_VAL) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: ALS Value Error: 0x%X\n", __func__,
					ch1_val);
	}
	ch1_val &= ALS_VALID_MEASURE_MASK;
	/*input_report_abs(ltr559->als_input_dev, ABS_MISC, ch1_val);*/
	/*input_sync(ltr559->als_input_dev);*/
	/*buffer[0] = LTR559_ALS_PS_STATUS;
	ret = I2C_Read(buffer,	1);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s | 0x%02X", __func__,	buffer[0]);

		return ret;
	}*/
	value_temp = als_ps_status;/*buffer[0];*/
	temp = als_ps_status;/*buffer[0];*/
	gain = (value_temp & 0x70);
	gain >>= 4;

	if (gain == 0) {			/*gain 1*/
		gain = 1;
	} else if (gain == 1) {		/*gain 2*/
		gain = 2;
	} else if (gain == 2) {		/*gain 4*/
		gain = 4;
	} else if (gain == 3) {		/*gain 8*/
		gain = 8;
	} else if (gain == 6) {		/*gain 48*/
		gain = 48;
	} else if (gain == 7) {		/*gain 96*/
		gain = 96;
	}

	buffer[0] = LTR559_ALS_CONTR;
	ret = I2C_Read(buffer,	1);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s | 0x%02X", __func__,	buffer[0]);

		return ret;
	}
	value_temp = buffer[0];
	value_temp &= 0xE3;

	if ((ch0_val == 0) && (ch1_val >  50))
		value = lux_val_prev;
	else {
		value = ratioHysterisis(ch0_val,	ch1_val);

		/*if (gain == 1) {
			if ((ch0_val + ch1_val) <
				((AGC_DOWN_THRESHOLD * 10) / AGC_HYS)) {
				value = ratioHysterisis(ch0_val,	ch1_val);
				value_temp |= ALS_GAIN_8x;
				gain_chg_req = 1;
			} else {
				value = ratioHysterisis(ch0_val,	ch1_val);
			}
		} else if (gain == 8) {
			if ((ch0_val + ch1_val) >  AGC_UP_THRESHOLD) {
				value = ratioHysterisis(ch0_val,	ch1_val);
				value_temp |= ALS_GAIN_1x;
				gain_chg_req = 1;
			} else {
				value = ratioHysterisis(ch0_val,	ch1_val);
			}
		} else {
			value = ratioHysterisis(ch0_val,	ch1_val);
		}
		if (gain_chg_req) {
			buffer[0] = LTR559_ALS_CONTR;
			buffer[1] = value_temp;
			ret = I2C_Write(buffer,	2);
			if (ret < 0) {
				dev_err(&ltr559->i2c_client->dev,
					"%s | 0x%02X", __func__,	buffer[0]);
				return ret;
			}
		}*/

	}

	if ((value > MAX_VAL) || (((ch0_val + ch1_val) > MAX_VAL) && (temp & 0x80)))
		value = MAX_VAL;

	lux_val_prev = value;

	LTR559_DBG2("%s: ch0=%d,	ch1=%d,	value=%d\n",
	 __func__,	ch0_val,	ch1_val,	value);
	return value;
}


static uint16_t read_ps_adc_value(struct ltr559_data *ltr559)
{
	int8_t ret = -99;
	uint16_t value = -99;
	uint16_t ps_val;
	uint8_t buffer[4];

	buffer[0] = LTR559_PS_DATA_0;
	/* read data bytes from data regs */
	ret = I2C_Read(buffer,	2);

	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s | 0x%02X", __func__,	buffer[0]);

		return ret;
	}

	ps_val = (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
	dev_dbg(&ltr559->i2c_client->dev,
		"%s | ps value = 0x%04X\n", __func__,
		ps_val);

	if (ps_val >  PS_MAX_MEASURE_VAL) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: PS Value Error: 0x%X\n", __func__,
					ps_val);
	}
	ps_val &= PS_VALID_MEASURE_MASK;

	value = ps_val;

	LTR559_DBG("%s: ps_val=%d\n", __func__,	ps_val);

	return value;
}


static int8_t als_mode_setup(uint8_t alsMode_set_reset,
					 struct ltr559_data *ltr559)
{
	int8_t ret = 0;

	ret = _ltr559_set_bit(ltr559->i2c_client,	alsMode_set_reset,
				LTR559_ALS_CONTR,	ALS_MODE_ACTIVE);

	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s ALS mode setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t als_sw_reset_setup(uint8_t alsSWReset_set_reset,
				 struct ltr559_data *ltr559)
{
	int8_t ret = 0;
	ret = _ltr559_set_bit(ltr559->i2c_client,	alsSWReset_set_reset,
				LTR559_ALS_CONTR,	ALS_SW_RESET);

	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s ALS sw reset setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t als_gain_setup(uint8_t alsgain_range,	struct ltr559_data *ltr559)
{
	int8_t ret = 0;
	uint8_t buffer[2],	value;

	buffer[0] = LTR559_ALS_CONTR;
	ret = I2C_Read(buffer,	1);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s | 0x%02X", __func__,	buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0xE3;

	if (alsgain_range == 1)
		value |= ALS_GAIN_1x;
	else if (alsgain_range == 2)
		value |= ALS_GAIN_2x;
	else if (alsgain_range == 4)
		value |= ALS_GAIN_4x;
	else if (alsgain_range == 8)
		value |= ALS_GAIN_8x;
	else if (alsgain_range == 48)
		value |= ALS_GAIN_48x;
	else if (alsgain_range == 96)
		value |= ALS_GAIN_96x;

	buffer[0] = LTR559_ALS_CONTR;
	buffer[1] = value;
	ret = I2C_Write(buffer,	2);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,	"%s ALS gain setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t als_contr_setup(uint8_t als_contr_val,	struct ltr559_data *ltr559)
{
	int8_t ret = 0;
	uint8_t buffer[3];

	buffer[0] = LTR559_ALS_CONTR;

	/* Default settings used for now. */
	buffer[1] = als_contr_val;
	buffer[1] &= 0x1F;
	ret = I2C_Write(buffer,	2);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s | ALS_CONTR (0x%02X) setup fail...",
			 __func__,	buffer[0]);
	}

	return ret;
}


static int8_t als_contr_readback(uint8_t rdbck_type,	uint8_t *retVal,
				struct ltr559_data *ltr559)
{
	int8_t ret = 0;
	uint8_t buffer[2],	value;

	buffer[0] = LTR559_ALS_CONTR;
	ret = I2C_Read(buffer,	1);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s | 0x%02X", __func__,	buffer[0]);
		return ret;
	}

	value = buffer[0];

	if (rdbck_type == ALS_MODE_RDBCK)
		*retVal = value & 0x01;
	else if (rdbck_type == ALS_SWRT_RDBCK)
		*retVal = (value & 0x02) >> 1;
	else if (rdbck_type == ALS_GAIN_RDBCK)
		*retVal = (value & 0x1C) >> 2;
	else if (rdbck_type == ALS_CONTR_RDBCK)
		*retVal = value & 0x1F;

	return ret;
}


static int8_t ps_mode_setup(uint8_t psMode_set_reset,
				struct ltr559_data *ltr559)
{
	int8_t ret = 0;
	ret = _ltr559_set_bit(ltr559->i2c_client,	psMode_set_reset,
					LTR559_PS_CONTR,	PS_MODE_ACTIVE);

	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s PS mode setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t ps_gain_setup(uint8_t psgain_range,	struct ltr559_data *ltr559)
{
	int8_t ret = 0;
	uint8_t buffer[2],	value;

	buffer[0] = LTR559_PS_CONTR;
	ret = I2C_Read(buffer,	1);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
		"%s | 0x%02X", __func__,	buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0xF3;

	if (psgain_range == 16)
		value |= PS_GAIN_16x;
	else if (psgain_range == 32)
		value |= PS_GAIN_32x;
	else if (psgain_range == 64)
		value |= PS_GAIN_64x;

	buffer[0] = LTR559_PS_CONTR;
	buffer[1] = value;
	ret = I2C_Write(buffer,	2);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s PS gain setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t ps_satu_indica_setup(uint8_t pssatuindica_enable,
				struct ltr559_data *ltr559)
{
	int8_t ret = 0;
	ret = _ltr559_set_bit(ltr559->i2c_client,	pssatuindica_enable,
				LTR559_PS_CONTR,	PS_SATUR_INDIC_EN);

	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s PS saturation indicator setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t ps_contr_setup(uint8_t ps_contr_val,
				struct ltr559_data *ltr559)
{
	int8_t ret = 0;
	uint8_t buffer[3];

	buffer[0] = LTR559_PS_CONTR;

	/* Default settings used for now. */
	buffer[1] = ps_contr_val;
	buffer[1] &= 0x2F;
	ret = I2C_Write(buffer,	2);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s | PS_CONTR (0x%02X) setup fail...",
		 __func__,	buffer[0]);
	}

	return ret;
}


static int8_t ps_contr_readback(uint8_t rdbck_type,	uint8_t *retVal,
					struct ltr559_data *ltr559)
{
	int8_t ret = 0;
	uint8_t buffer[2],	value;

	buffer[0] = LTR559_PS_CONTR;
	ret = I2C_Read(buffer,	1);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s | 0x%02X", __func__,	buffer[0]);
		return ret;
	}

	value = buffer[0];

	if (rdbck_type == PS_MODE_RDBCK)
		*retVal = (value & 0x03);
	else if (rdbck_type == PS_GAIN_RDBCK)
		*retVal = (value & 0x0C) >> 2;
	else if (rdbck_type == PS_SATUR_RDBCK)
		*retVal = (value & 0x20) >> 5;
	else if (rdbck_type == PS_CONTR_RDBCK)
		*retVal = value & 0x2F;

	return ret;
}


static int8_t ps_ledCurrent_setup(uint8_t psledcurr_val,
					struct ltr559_data *ltr559)
{
	int8_t ret = 0;
	uint8_t buffer[2],	value;

	buffer[0] = LTR559_PS_LED;
	ret = I2C_Read(buffer,	1);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s | 0x%02X", __func__,	buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0xF8;

	if (psledcurr_val == 5)
		value |= LED_CURR_5MA;
	else if (psledcurr_val == 10)
		value |= LED_CURR_10MA;
	else if (psledcurr_val == 20)
		value |= LED_CURR_20MA;
	else if (psledcurr_val == 50)
		value |= LED_CURR_50MA;
	else if (psledcurr_val == 100)
		value |= LED_CURR_100MA;

	buffer[0] = LTR559_PS_LED;
	buffer[1] = value;
	ret = I2C_Write(buffer,	2);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s PS LED current setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t ps_ledCurrDuty_setup(uint8_t psleddutycycle_val,
					struct ltr559_data *ltr559)
{
	int8_t ret = 0;
	uint8_t buffer[2],	value;

	buffer[0] = LTR559_PS_LED;
	ret = I2C_Read(buffer,	1);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s | 0x%02X", __func__,	buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0xE7;

	if (psleddutycycle_val == 25)
		value |= LED_CURR_DUTY_25PC;
	else if (psleddutycycle_val == 50)
		value |= LED_CURR_DUTY_50PC;
	else if (psleddutycycle_val == 75)
		value |= LED_CURR_DUTY_75PC;
	else if (psleddutycycle_val == 100)
		value |= LED_CURR_DUTY_100PC;

	buffer[0] = LTR559_PS_LED;
	buffer[1] = value;
	ret = I2C_Write(buffer,	2);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s PS LED current duty setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t ps_ledPulseFreq_setup(uint8_t pspulreq_val,
				struct ltr559_data *ltr559)
{
	int8_t ret = 0;
	uint8_t buffer[2],	value;

	buffer[0] = LTR559_PS_LED;
	ret = I2C_Read(buffer,	1);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s | 0x%02X", __func__,	buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0x1F;

	if (pspulreq_val == 30)
		value |= LED_PUL_FREQ_30KHZ;
	else if (pspulreq_val == 40)
		value |= LED_PUL_FREQ_40KHZ;
	else if (pspulreq_val == 50)
		value |= LED_PUL_FREQ_50KHZ;
	else if (pspulreq_val == 60)
		value |= LED_PUL_FREQ_60KHZ;
	else if (pspulreq_val == 70)
		value |= LED_PUL_FREQ_70KHZ;
	else if (pspulreq_val == 80)
		value |= LED_PUL_FREQ_80KHZ;
	else if (pspulreq_val == 90)
		value |= LED_PUL_FREQ_90KHZ;
	else if (pspulreq_val == 100)
		value |= LED_PUL_FREQ_100KHZ;

	buffer[0] = LTR559_PS_LED;
	buffer[1] = value;
	ret = I2C_Write(buffer,	2);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s PS LED pulse frequency setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


/* LED Setup */
static int8_t ps_led_setup(uint8_t ps_led_val,	struct ltr559_data *ltr559)
{
	int8_t ret = 0;
	uint8_t buffer[3];

	buffer[0] = LTR559_PS_LED;

	/* Default settings used for now. */
	buffer[1] = ps_led_val;
	ret = I2C_Write(buffer,	2);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s | PS_LED (0x%02X) setup fail...",
			 __func__,	buffer[0]);
	}

	return ret;
}


static int8_t ps_led_readback(uint8_t rdbck_type,	uint8_t *retVal,
				struct ltr559_data *ltr559)
{
	int8_t ret = 0;
	uint8_t buffer[2],	value;

	buffer[0] = LTR559_PS_LED;
	ret = I2C_Read(buffer,	1);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s | 0x%02X", __func__,	buffer[0]);
		return ret;
	}

	value = buffer[0];

	if (rdbck_type == LED_CURR_RDBCK)
		*retVal = (value & 0x07);
	else if (rdbck_type == LED_CURR_DUTY_RDBCK)
		*retVal = (value & 0x18) >> 3;
	else if (rdbck_type == LED_PUL_FREQ_RDBCK)
		*retVal = (value & 0xE0) >> 5;
	else if (rdbck_type == PS_LED_RDBCK)
		*retVal = value;

	return ret;
}


static int8_t ps_ledPulseCount_setup(uint8_t pspulsecount_val,
				struct ltr559_data *ltr559)
{
	int8_t ret = 0;
	uint8_t buffer[3];

	buffer[0] = LTR559_PS_N_PULSES;

	/* Default settings used for now. */
	if (pspulsecount_val >  15)
		pspulsecount_val = 15;

	buffer[1] = pspulsecount_val;
	buffer[1] &= 0x0F;
	ret = I2C_Write(buffer,	2);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s | PS_LED_COUNT (0x%02X) setup fail...",
			 __func__,	buffer[0]);
	}

	return ret;
}


static int8_t ps_ledPulseCount_readback(uint8_t *retVal,
				struct ltr559_data *ltr559)
{
	int8_t ret = 0;
	uint8_t buffer[3],	value;

	buffer[0] = LTR559_PS_N_PULSES;
	ret = I2C_Read(buffer,	1);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s | 0x%02X", __func__,	buffer[0]);
		return ret;
	}

	value = buffer[0];
	*retVal = value;

	return ret;
}


static int8_t ps_meas_rate_setup(uint16_t meas_rate_val,
				struct ltr559_data *ltr559)
{
	int8_t ret = 0;
	uint8_t buffer[2],	value;

	buffer[0] = LTR559_PS_MEAS_RATE;
	ret = I2C_Read(buffer,	1);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s | 0x%02X", __func__,	buffer[0]);

		return ret;
	}

	value = buffer[0];
	value &= 0xF0;

	if (meas_rate_val == 50)
		value |= PS_MEAS_RPT_RATE_50MS;
	else if (meas_rate_val == 70)
		value |= PS_MEAS_RPT_RATE_70MS;
	else if (meas_rate_val == 100)
		value |= PS_MEAS_RPT_RATE_100MS;
	else if (meas_rate_val == 200)
		value |= PS_MEAS_RPT_RATE_200MS;
	else if (meas_rate_val == 500)
		value |= PS_MEAS_RPT_RATE_500MS;
	else if (meas_rate_val == 1000)
		value |= PS_MEAS_RPT_RATE_1000MS;
	else if (meas_rate_val == 2000)
		value |= PS_MEAS_RPT_RATE_2000MS;
	else if (meas_rate_val == 10)
		value |= PS_MEAS_RPT_RATE_10MS;

	buffer[0] = LTR559_PS_MEAS_RATE;
	buffer[1] = value;
	ret = I2C_Write(buffer,	2);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s PS measurement rate setup fail...\n", __func__);

		return ret;
	}

	return ret;
}


static int8_t ps_meas_rate_readback(uint8_t *retVal,
				struct ltr559_data *ltr559)
{
	int8_t ret = 0;
	uint8_t buffer[3],	value;

	buffer[0] = LTR559_PS_MEAS_RATE;
	ret = I2C_Read(buffer,	1);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s | 0x%02X", __func__,	buffer[0]);
		return ret;
	}

	value = buffer[0];
	*retVal = (value & 0x0F);

	return ret;
}


static int8_t als_meas_rate_setup(uint16_t meas_rate_val,
					struct ltr559_data *ltr559)
{
	int8_t ret = 0;
	uint8_t buffer[2],	value;

	buffer[0] = LTR559_ALS_MEAS_RATE;
	ret = I2C_Read(buffer,	1);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s | 0x%02X", __func__,	buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0xF8;

	if (meas_rate_val == 50)
		value |= ALS_MEAS_RPT_RATE_50MS;
	else if (meas_rate_val == 100)
		value |= ALS_MEAS_RPT_RATE_100MS;
	else if (meas_rate_val == 200)
		value |= ALS_MEAS_RPT_RATE_200MS;
	else if (meas_rate_val == 500)
		value |= ALS_MEAS_RPT_RATE_500MS;
	else if (meas_rate_val == 1000)
		value |= ALS_MEAS_RPT_RATE_1000MS;
	else if (meas_rate_val == 2000)
		value |= ALS_MEAS_RPT_RATE_2000MS;

	buffer[0] = LTR559_ALS_MEAS_RATE;
	buffer[1] = value;
	ret = I2C_Write(buffer,	2);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s ALS measurement rate setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t als_integ_time_setup(uint16_t integ_time_val,
					struct ltr559_data *ltr559)
{
	int8_t ret = 0;
	uint8_t buffer[2],	value;

	buffer[0] = LTR559_ALS_MEAS_RATE;
	ret = I2C_Read(buffer,	1);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
				"%s | 0x%02X", __func__,	buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0xC7;

	if (integ_time_val == 100)
		value |= ALS_INTEG_TM_100MS;
	else if (integ_time_val == 50)
		value |= ALS_INTEG_TM_50MS;
	else if (integ_time_val == 200)
		value |= ALS_INTEG_TM_200MS;
	else if (integ_time_val == 400)
		value |= ALS_INTEG_TM_400MS;
	else if (integ_time_val == 150)
		value |= ALS_INTEG_TM_150MS;
	else if (integ_time_val == 250)
		value |= ALS_INTEG_TM_250MS;
	else if (integ_time_val == 300)
		value |= ALS_INTEG_TM_300MS;
	else if (integ_time_val == 350)
		value |= ALS_INTEG_TM_350MS;

	buffer[0] = LTR559_ALS_MEAS_RATE;
	buffer[1] = value;
	ret = I2C_Write(buffer,	2);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s ALS integration time setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t als_meas_rate_reg_setup(uint8_t als_meas_rate_reg_val,
					struct ltr559_data *ltr559)
{
	int8_t ret = 0;
	uint8_t buffer[3];

	buffer[0] = LTR559_ALS_MEAS_RATE;

	buffer[1] = als_meas_rate_reg_val;
	buffer[1] &= 0x3F;
	ret = I2C_Write(buffer,	2);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s | ALS_MEAS_RATE (0x%02X) setup fail...",
			 __func__,	buffer[0]);
	}

	return ret;
}


static int8_t als_meas_rate_readback(uint8_t rdbck_type,	uint8_t *retVal,
						struct ltr559_data *ltr559)
{
	int8_t ret = 0;
	uint8_t buffer[2],	value;

	buffer[0] = LTR559_ALS_MEAS_RATE;
	ret = I2C_Read(buffer,	1);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s | 0x%02X", __func__,	buffer[0]);
		return ret;
	}

	value = buffer[0];

	if (rdbck_type == ALS_MEAS_RPT_RATE_RDBCK)
		*retVal = (value & 0x07);
	else if (rdbck_type == ALS_INTEG_TM_RDBCK)
		*retVal = (value & 0x38) >> 3;
	else if (rdbck_type == ALS_MEAS_RATE_RDBCK)
		*retVal = (value & 0x3F);

	return ret;
}


static int8_t part_ID_reg_readback(uint8_t rdbck_type,	uint8_t *retVal,
					struct ltr559_data *ltr559)
{
	int8_t ret = 0;
	uint8_t buffer[1],	value;

	buffer[0] = LTR559_PART_ID;
	ret = I2C_Read(buffer,	1);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s | 0x%02X", __func__,	buffer[0]);
		return ret;
	}

	value = buffer[0];

	if (rdbck_type == PART_NUM_ID_RDBCK)
		*retVal = (value & 0xF0) >> 4;
	else if (rdbck_type == REVISION_ID_RDBCK)
		*retVal = value & 0x0F;
	else if (rdbck_type == PART_ID_REG_RDBCK)
		*retVal = value;

	return ret;
}


static int8_t manu_ID_reg_readback(uint8_t *retVal,	struct ltr559_data *ltr559)
{
	int8_t ret = 0;
	uint8_t buffer[1],	value;

	buffer[0] = LTR559_MANUFACTURER_ID;
	ret = I2C_Read(buffer,	1);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s | 0x%02X", __func__,	buffer[0]);
		return ret;
	}

	value = buffer[0];
	*retVal = value;

	return ret;
}


static int8_t als_ps_status_reg(uint8_t data_status_type,	uint8_t *retVal,
						struct ltr559_data *ltr559)
{
	int8_t ret = 0;
	uint8_t buffer[2],	value;

	buffer[0] = LTR559_ALS_PS_STATUS;
	ret = I2C_Read(buffer,	1);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s | 0x%02X", __func__,	buffer[0]);
		return ret;
	}

	value = buffer[0];

	if (data_status_type == PS_DATA_STATUS_RDBCK)
		*retVal = (value & 0x01);
	else if (data_status_type == PS_INTERR_STATUS_RDBCK)
		*retVal = (value & 0x02) >> 1;
	else if (data_status_type == ALS_DATA_STATUS_RDBCK)
		*retVal = (value & 0x04) >> 2;
	else if (data_status_type == ALS_INTERR_STATUS_RDBCK)
		*retVal = (value & 0x08) >> 3;
	else if (data_status_type == ALS_GAIN_STATUS_RDBCK)
		*retVal = (value & 0x70) >> 4;
	else if (data_status_type == ALS_VALID_STATUS_RDBCK)
		*retVal = (value & 0x80) >> 7;
	else if (data_status_type == ALS_PS_STATUS_RDBCK)
		*retVal = value;

	return ret;
}


static int8_t als_ch0ch1raw_calc_readback(uint16_t *retVal1,	uint16_t *retVal2,
			uint16_t *retVal3,	struct ltr559_data *ltr559)
{
	int8_t ret = 0;
	uint8_t buffer[11];
	uint16_t value1,	value2,	value3;

	buffer[0] = LTR559_ALS_DATA_CH1_0;
	ret = I2C_Read(buffer,	4);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s | 0x%02X", __func__,	buffer[0]);
		return ret;
	}

	value1 = ((int)buffer[2]) + ((int)buffer[3] << 8); /* CH0*/
	value2 = ((int)buffer[0]) + ((int)buffer[1] << 8); /* CH1*/

	value3 = ratioHysterisis(value1,	value2);

	*retVal1 = value1;
	*retVal2 = value2;
	*retVal3 = value3;

	return ret;
}


static int8_t interrupt_mode_setup(uint8_t interr_mode_val,
					struct ltr559_data *ltr559)
{
	int8_t ret = 0;
	uint8_t buffer[2],	value;

	buffer[0] = LTR559_INTERRUPT;
	ret = I2C_Read(buffer,	1);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s | 0x%02X", __func__,	buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0xFC;

	if (interr_mode_val == 0)
		value |= INT_MODE_00;
	else if (interr_mode_val == 1)
		value |= INT_MODE_PS_TRIG;
	else if (interr_mode_val == 2)
		value |= INT_MODE_ALS_TRIG;
	else if (interr_mode_val == 3)
		value |= INT_MODE_ALSPS_TRIG;

	buffer[0] = LTR559_INTERRUPT;
	buffer[1] = value;
	ret = I2C_Write(buffer,	2);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s Interrupt mode setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t interrupt_polarity_setup(uint8_t interr_polar_val,
						struct ltr559_data *ltr559)
{
	int8_t ret = 0;
	uint8_t buffer[2],	value;

	buffer[0] = LTR559_INTERRUPT;
	ret = I2C_Read(buffer,	1);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s | 0x%02X", __func__,	buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0xFB;

	if (interr_polar_val == 0)
		value |= INT_POLAR_ACT_LO;
	else if (interr_polar_val == 1)
		value |= INT_POLAR_ACT_HI;

	buffer[0] = LTR559_INTERRUPT;
	buffer[1] = value;
	ret = I2C_Write(buffer,	2);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s Interrupt polarity setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t interrupt_setup(uint8_t interrupt_val,
			struct ltr559_data *ltr559)
{
	int8_t ret = 0;
	uint8_t buffer[3];

	buffer[0] = LTR559_INTERRUPT;

	/* Default settings used for now. */
	buffer[1] = interrupt_val;
	buffer[1] &= 0x07;
	ret = I2C_Write(buffer,	2);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s |Interrupt (0x%02X) setup fail...",
			 __func__,	buffer[0]);
	}

	return ret;
}


static int8_t interrupt_readback(uint8_t rdbck_type,	uint8_t *retVal,
					struct ltr559_data *ltr559)
{
	int8_t ret = 0;
	uint8_t buffer[2],	value;

	buffer[0] = LTR559_INTERRUPT;
	ret = I2C_Read(buffer,	1);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s | 0x%02X", __func__,	buffer[0]);
		return ret;
	}

	value = buffer[0];

	if (rdbck_type == INT_MODE_RDBCK)
		*retVal = (value & 0x03);
	else if (rdbck_type == INT_POLAR_RDBCK)
		*retVal = (value & 0x04) >> 2;
	else if (rdbck_type == INT_INTERRUPT_RDBCK)
		*retVal = (value & 0x07);

	return ret;
}

/*
static int8_t ps_offset_setup2(uint16_t ps_offset_val)
{
	int8_t ret = 0;
	uint8_t buffer[3];

	buffer[0] = LTR559_PS_OFFSET_1;
	buffer[1] = (ps_offset_val >> 8) & 0x03;
	buffer[2] = (ps_offset_val & 0xFF);

	ret = I2C_Write(buffer,	3);
	if (ret < 0) {
		printk("cgh something error happened\n");
		return ret;
	}
printk("cgh ps_offset_setup ps_offset_val = %d\n",	ps_offset_val);
	return ret;
}*/
static int8_t ps_offset_setup(uint16_t ps_offset_val,
					struct ltr559_data *ltr559)
{
	int8_t ret = 0;
	uint8_t buffer[3];

	buffer[0] = LTR559_PS_OFFSET_1;
	buffer[1] = (ps_offset_val >> 8) & 0x03;
	buffer[2] = (ps_offset_val & 0xFF);

	ret = I2C_Write(buffer,	3);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s PS offset setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t ps_offset_readback(uint16_t *offsetval,
				struct ltr559_data *ltr559)
{
	int8_t ret = 0;
	uint8_t buffer[2];
	uint16_t value;

	buffer[0] = LTR559_PS_OFFSET_1;
	ret = I2C_Read(buffer,	2);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s | 0x%02X", __func__,	buffer[0]);
		return ret;
	}

	value = buffer[0];
	value <<= 8;
	value += buffer[1];

	*offsetval = value;

	return ret;
}


static int8_t interrupt_persist_setup(uint8_t interr_persist_val,
					struct ltr559_data *ltr559)
{
	int8_t ret = 0;
	uint8_t buffer[2],	value;

	value = interr_persist_val;

	buffer[0] = LTR559_INTERRUPT_PRST;
	buffer[1] = value;
	ret = I2C_Write(buffer,	2);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s Interrupt persist setup fail...\n", __func__);
		return ret;
	}

	return ret;
}


static int8_t interrupt_prst_readback(uint8_t *retVal,
						struct ltr559_data *ltr559)
{
	int8_t ret = 0;
	uint8_t buffer[2],	value;

	buffer[0] = LTR559_INTERRUPT_PRST;
	ret = I2C_Read(buffer,	1);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s | 0x%02X", __func__,	buffer[0]);
		return ret;
	}

	value = buffer[0];

	*retVal = value;

	return ret;
}


/* Set ALS range */
static int8_t set_als_range(uint16_t lt,	uint16_t ht,	uint8_t lo_hi)
{
	int8_t ret;
	uint8_t buffer[5],	num_data = 0;

	if (lo_hi == LO_LIMIT) {
		buffer[0] = LTR559_ALS_THRES_LOW_0;
		buffer[1] = lt & 0xFF;
		buffer[2] = (lt >> 8) & 0xFF;
		num_data = 3;
	} else if (lo_hi == HI_LIMIT) {
		buffer[0] = LTR559_ALS_THRES_UP_0;
		buffer[1] = ht & 0xFF;
		buffer[2] = (ht >> 8) & 0xFF;
		num_data = 3;
	} else if (lo_hi == LO_N_HI_LIMIT) {
		buffer[0] = LTR559_ALS_THRES_UP_0;
		buffer[1] = ht & 0xFF;
		buffer[2] = (ht >> 8) & 0xFF;
		buffer[3] = lt & 0xFF;
		buffer[4] = (lt >> 8) & 0xFF;
		num_data = 5;
	}

	ret = I2C_Write(buffer,	num_data);
	if (ret < 0) {
		pr_alert("%s | 0x%02X", __func__,	buffer[0]);
		return ret;
	}
	dev_dbg(&sensor_info->i2c_client->dev,
		"%s Set als range:0x%04x - 0x%04x\n", __func__,	lt,	ht);

	return ret;
}


static int8_t als_range_readback(uint16_t *lt,	uint16_t *ht,
					struct ltr559_data *ltr559)
{
	int8_t ret = 0;
	uint8_t buffer[5];
	uint16_t value_lo,	value_hi;

	buffer[0] = LTR559_ALS_THRES_UP_0;
	ret = I2C_Read(buffer,	4);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
		"%s | 0x%02X", __func__,	buffer[0]);
		return ret;
	}

	value_lo = buffer[3];
	value_lo <<= 8;
	value_lo += buffer[2];
	*lt = value_lo;

	value_hi = buffer[1];
	value_hi <<= 8;
	value_hi += buffer[0];
	*ht = value_hi;

	return ret;
}


/* Set PS range */
static int8_t set_ps_range(uint16_t lt,	uint16_t ht,	uint8_t lo_hi,
					struct ltr559_data *ltr559)
{
	int8_t ret;
	uint8_t buffer[5],	num_data = 0;

	if (lo_hi == LO_LIMIT) {
		buffer[0] = LTR559_PS_THRES_LOW_0;
		buffer[1] = lt & 0xFF;
		buffer[2] = (lt >> 8) & 0x07;
		num_data = 3;
		LTR559_DBG("%s: REG LOW PS thresh is %d\n", __func__,	lt);
	} else if (lo_hi == HI_LIMIT) {
		buffer[0] = LTR559_PS_THRES_UP_0;
		buffer[1] = ht & 0xFF;
		buffer[2] = (ht >> 8) & 0x07;
		num_data = 3;
		LTR559_DBG("%s: REG HIGH PS thresh is %d\n", __func__,	ht);
	} else if (lo_hi == LO_N_HI_LIMIT) {
		buffer[0] = LTR559_PS_THRES_UP_0;
		buffer[1] = ht & 0xFF;
		buffer[2] = (ht >> 8) & 0x07;
		buffer[3] = lt & 0xFF;
		buffer[4] = (lt >> 8) & 0x07;
		num_data = 5;
		LTR559_DBG("%s: REG PS thresh is %d %d\n", __func__,	ht,	lt);
	}

	ret = I2C_Write(buffer,	num_data);
	if (ret < 0) {
		pr_alert("%s | 0x%02X", __func__,	buffer[0]);
		return ret;
	}

	return ret;
}

static int8_t ps_range_readback(uint16_t *lt,	uint16_t *ht,
						struct ltr559_data *ltr559)
{
	int8_t ret = 0;
	uint8_t buffer[5];
	uint16_t value_lo,	value_hi;

	buffer[0] = LTR559_PS_THRES_UP_0;
	ret = I2C_Read(buffer,	4);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s | 0x%02X", __func__,	buffer[0]);

		return ret;
	}

	value_lo = buffer[3];
	value_lo <<= 8;
	value_lo += buffer[2];
	*lt = value_lo;

	value_hi = buffer[1];
	value_hi <<= 8;
	value_hi += buffer[0];
	*ht = value_hi;

	return ret;
}

/*
static uint16_t discardMinMax_findCTMov_Avg(uint16_t *ps_val)
{
#define MAX_NUM_PS_DATA1		PS_MAX_MOV_AVG_KEPT_DATA_CTR
#define STARTING_PS_INDEX1		0
#define ENDING_PS_INDEX1		5
#define NUM_AVG_DATA1			5

	uint8_t i_ctr,	i_ctr2,	maxIndex,	minIndex;
	uint16_t maxVal,	minVal,	_ps_val[MAX_NUM_PS_DATA1];
	uint16_t temp = 0;

	for (i_ctr = STARTING_PS_INDEX1; i_ctr < MAX_NUM_PS_DATA1; i_ctr++)
		_ps_val[i_ctr] = ps_val[i_ctr];

	maxVal = ps_val[STARTING_PS_INDEX1];
	maxIndex = STARTING_PS_INDEX1;
	minVal = ps_val[STARTING_PS_INDEX1];
	minIndex = STARTING_PS_INDEX1;

	for (i_ctr = STARTING_PS_INDEX1; i_ctr < MAX_NUM_PS_DATA1; i_ctr++) {
		if (ps_val[i_ctr] >  maxVal) {
			maxVal = ps_val[i_ctr];
			maxIndex = i_ctr;
		}
	}

	for (i_ctr = STARTING_PS_INDEX1; i_ctr < MAX_NUM_PS_DATA1; i_ctr++) {
		if (ps_val[i_ctr] < minVal) {
			minVal = ps_val[i_ctr];
			minIndex = i_ctr;
		}
	}

	i_ctr2 = 0;

	if (minIndex != maxIndex) {
		for (i_ctr = STARTING_PS_INDEX1;
				i_ctr < MAX_NUM_PS_DATA1; i_ctr++) {
			if ((i_ctr != minIndex) && (i_ctr != maxIndex)) {
				ps_val[i_ctr2] = _ps_val[i_ctr];
				i_ctr2++;
			}
		}
	}
	ps_val[MAX_NUM_PS_DATA1 - 1] = 0;
	ps_val[MAX_NUM_PS_DATA1 - 2] = 0;

	for (i_ctr = STARTING_PS_INDEX1; i_ctr < ENDING_PS_INDEX1; i_ctr++)
		temp += ps_val[i_ctr];

	temp = (temp / NUM_AVG_DATA1);

	return temp;
}

static uint16_t findCT_Avg(uint16_t *ps_val)
{
#define MAX_NUM_PS_DATA2		PS_MAX_INIT_KEPT_DATA_COUNTER
#define STARTING_PS_INDEX2		3
#define NUM_AVG_DATA2			3

	uint8_t i_ctr,	min_Index,	max_Index;
	uint16_t max_val,	min_val;
	uint16_t temp = 0;

	max_val = ps_val[STARTING_PS_INDEX2];
	max_Index = STARTING_PS_INDEX2;
	min_val = ps_val[STARTING_PS_INDEX2];
	min_Index = STARTING_PS_INDEX2;

	for (i_ctr = STARTING_PS_INDEX2; i_ctr < MAX_NUM_PS_DATA2; i_ctr++) {
		if (ps_val[i_ctr] >  max_val) {
			max_val = ps_val[i_ctr];
			max_Index = i_ctr;
		}
	}

	for (i_ctr = STARTING_PS_INDEX2; i_ctr < MAX_NUM_PS_DATA2; i_ctr++) {
		if (ps_val[i_ctr] < min_val) {
			min_val = ps_val[i_ctr];
			min_Index = i_ctr;
		}
	}

	if (min_val == max_val)
		temp = ps_val[STARTING_PS_INDEX2];
	else {
		for (i_ctr = STARTING_PS_INDEX2;
				i_ctr < MAX_NUM_PS_DATA2; i_ctr++) {
			if ((i_ctr != min_Index) && (i_ctr != max_Index))
				temp += ps_val[i_ctr];
		}
		temp = (temp / NUM_AVG_DATA2);
	}

	return temp;
}
*/
int updateThreshold(uint16_t crosstalk)
{
	struct ltr559_data *data = sensor_info;
	int ret = 0;

	check_prox_mean(crosstalk,	&ltr559_ps_detection_threshold,	&ltr559_ps_hsyteresis_threshold);
	data->default_ps_highthresh = ltr559_ps_detection_threshold;
	data->default_ps_lowthresh = ltr559_ps_hsyteresis_threshold;
	LTR559_DBG("%s: new ps thresh is %d %d\n", __func__,
				data->default_ps_highthresh,	data->default_ps_lowthresh);

	ret = set_ps_range(data->default_ps_lowthresh,	data->default_ps_highthresh,	LO_N_HI_LIMIT,	data);
	return ret;
}

uint16_t getCrosstalk(uint16_t count)
{
	uint16_t raw_ct = 0;
	uint16_t sum = 0;
	uint16_t avg = 0;
	uint16_t max = 0;
	uint16_t min = 0XFFFF;
	int i;
	struct ltr559_data *ltr559 = sensor_info;

	if (0 == count)
		return 0;

	/*Check whether raw crosstalk is zero for the first 3 times*/
	for (i = 0; i < 3; i++) {
		usleep_range(10000 , 10500);
		raw_ct = read_ps_adc_value(ltr559);
		if (raw_ct > 0 && raw_ct < 2048) {
			LTR559_DBG("%s: raw crosstalk is not zero!\n", __func__);
			break;
		 }
	}
	if (i == 3)
		return 0;

	/*Get	raw and total crosstalk*/
	sum += raw_ct;
	for (i = 1; i < count; i++) {
		msleep(20);
		raw_ct = read_ps_adc_value(ltr559);
		sum += raw_ct;
		if (max < raw_ct)
			max = raw_ct;
		if (min > raw_ct)
			min = raw_ct;
	}

	/*Get avg crosstalk*/
	avg = sum/count;

	LTR559_DBG("%s: Avg_CT=%d,	Sum_CT=%d,	Count=%d,	Max_CT=%d,	Min_CT=%d\n", __func__,
					avg,	sum,	count,	max,	min);

	return avg;

}


/* take note ------------------------------------------
 This function should be called in the function which is called
 when the CALL button is pressed.
 take note ------------------------------------------*/
static void setThrDuringCall(void)
{
	int8_t ret;
	struct ltr559_data *ltr559 = sensor_info;

	/* set ps measurement rate to 10ms*/
	ret = ps_meas_rate_setup(10,	ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: PS MeasRate Setup Fail...\n", __func__);
	}

	ps_grabData_stage = 0;
	ps_kept_data_counter = 0;
	ps_movavg_data_counter = 0;

	ret = set_ps_range(PS_MIN_MEASURE_VAL,	PS_MIN_MEASURE_VAL,
						LO_N_HI_LIMIT,	ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s : PS thresholds setting Fail...\n", __func__);
	}

	ret = ps_contr_setup(0x03,	ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: PS Enable Fail...\n", __func__);
	}
}

#ifdef DYNAMIC_PS_CALI
#define PS_COUNTER_DELTA	100
static uint16_t sample_times = 0;
static uint16_t record_ps_count = 0;
#endif
#if CALLING_DYNAMIC_CALI
static int get_stable_ps(unsigned int ps_data_c_1)
{
	int ps_d_1;
	int ps_d_high;
	int i;

	if (rct_ps_1 >= MAX_ELM_PS_1)
		full_ps_1 = 1;

	if (full_ps_1) {
		rct_ps_1 %= MAX_ELM_PS_1;
		ps_sum_1 -= record_ps_1[rct_ps_1];
	}
	ps_sum_1 += ps_data_c_1;
	record_ps_1[rct_ps_1] = ps_data_c_1;
	rct_ps_1++;

	if (full_ps_1)
		ps_d_1 = ps_sum_1 / MAX_ELM_PS_1;
	else
		ps_d_1 = ps_sum_1 / rct_ps_1;

	ps_d_high = ps_d_1 + 20;

	for (i = 0; i < MAX_ELM_PS_1; i++) {
		if (record_ps_1[i] < ps_d_high)
			j_ps = j_ps + 1;
		else
			j_ps = 0;
	}

	if (full_ps_1) {
		if (j_ps >= MAX_ELM_PS_1)
			return 1;
		else
			return 0;
	} else {
		return 0;
	}

}

static int ltr559_get_ps_value(struct ltr559_data *obj, u16 ps)
{
	int val;
	static int val_temp = 1;

	if (ps >= atomic_read(&obj->ps_persist_val_high)) {/* modified by steven*/
		val = 2;  /* persist oil close*/
		val_temp = 2;
		intr_flag_value = 2;
		oil_far_cal = 0;
		oil_close = 1;
	} else if (ps >= obj->default_ps_highthresh) {
		if (oil_close == 0) {
				val = 0;  /*normal close*/
				val_temp = 0;
				intr_flag_value = 1;
				oil_far_cal = 0;
			}

		if ((ps <= atomic_read(&obj->ps_persist_val_low)) && (oil_close == 1)) {
				val = 3;  /* persist oil far away*/
				val_temp = 3;
				intr_flag_value = 3;
			}
		} else if (ps <= obj->default_ps_lowthresh) {
				val = 1;  /*normal far away*/
				val_temp = 1;
				intr_flag_value = 0;
				oil_far_cal = 0;

				oil_close = 0;
		} else if (oil_close == 1) {
				val = 3;  /* persist oil far away*/
				val_temp = 3;
				intr_flag_value = 3;
		} else {
				val = val_temp;
				oil_far_cal = 0;
		}

	if (val == 3  && oil_far_cal <= (MAX_ELM_PS_1 + 5)) {/* modified by steven stable data*/
		oil_far_cal++;

		val = 2;  /* persist oil close*/
		val_temp = 2;
		intr_flag_value = 2;

		if (1 == oil_far_cal || 13 == oil_far_cal || 26 == oil_far_cal) {
			LTR559_DBG("%s: oil_far_cal=%d\n", __func__, oil_far_cal);
		}

		if (get_stable_ps(ps) == 1) {

			val = 3;  /* persist oil far away*/
			val_temp = 3;
			intr_flag_value = 3;

		}
	}

	return val;
}

static void dynamic_update_ps_threshold(struct ltr559_data *ltr559, uint16_t crosstalk)
{
	LTR559_DBG("%s: CT=%d\n", __func__, crosstalk);
#if defined(CONFIG_BOARD_SWEET)
	if (crosstalk < 1200) {
		ltr559->default_ps_highthresh = crosstalk + 105;
		ltr559->default_ps_lowthresh = crosstalk + 55;
		atomic_set(&ltr559->ps_persist_val_high, crosstalk + 1000);
		atomic_set(&ltr559->ps_persist_val_low, crosstalk + 900);
	} else {
		ltr559->default_ps_highthresh = 1800;
		ltr559->default_ps_lowthresh = 1600;
		atomic_set(&ltr559->ps_persist_val_high, crosstalk + 2000);
		atomic_set(&ltr559->ps_persist_val_low, crosstalk + 1900);
	}
#else
	if (crosstalk < 1200) {
		ltr559->default_ps_highthresh = crosstalk + 180;
		ltr559->default_ps_lowthresh = crosstalk + 95;
		atomic_set(&ltr559->ps_persist_val_high, crosstalk + 1000);
		atomic_set(&ltr559->ps_persist_val_low, crosstalk + 900);
	} else {
		ltr559->default_ps_highthresh = 1800;
		ltr559->default_ps_lowthresh = 1600;
		atomic_set(&ltr559->ps_persist_val_high, crosstalk + 2000);
		atomic_set(&ltr559->ps_persist_val_low, crosstalk + 1900);
	}
#endif
}
#endif

static void report_ps_input_event_zte(struct ltr559_data *ltr559)
{
	int8_t ret;
	uint16_t adc_value;
	uint16_t als_adc_value;
	uint16_t ch0_val;
	uint8_t buffer[4];
#if CALLING_DYNAMIC_CALI
	uint8_t ps_value_nf;
#endif
	adc_value = read_ps_adc_value(ltr559);
	/* Fix ps wrong report near in sunshine mode */
	/*als_adc_value = read_als_adc_value(ltr559);*/

	buffer[0] = LTR559_ALS_DATA_CH1_0;
	/* read data bytes from data regs */
	ret = I2C_Read(buffer, 4);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s | 0x%02X", __func__, buffer[0]);
		return;
	}
	/* ALS Ch0 */
	ch0_val = (uint16_t)buffer[2] | ((uint16_t)buffer[3] << 8);
	dev_dbg(&ltr559->i2c_client->dev,
			"%s | als_ch0 value = 0x%04X\n", __func__, ch0_val);
	als_adc_value = ch0_val;
	LTR559_DBG("%s:  als_adc_value ch0_val :%d\n", __func__, als_adc_value);

#if CALLING_DYNAMIC_CALI
	ps_value_nf = ltr559_get_ps_value(ltr559, adc_value);
	LTR559_DBG("%s:  ltr559_get_ps_value ps_value_nf :%d\n", __func__, ps_value_nf);

	if (ps_value_nf == 0) {/*normal near*/
		/* Begin: Fix ps wrong report near in sunshine mode */
		/* temp set 6000lux, you can change it */
		LTR559_DBG("%s: CT=%d, Lux=%d\n", __func__, adc_value, als_adc_value);
		#ifdef LTR559_REG_DUMP_ENABLE
		ltr559_reg_dump();
		#endif
		if (als_adc_value > 6000) {
			LTR559_DBG("%s: ps wrong report NEAR, just return\n", __func__);
			return;
		}
		/* End: Fix ps wrong report near in sunshine mode */
		input_report_abs(ltr559->ps_input_dev,	ABS_DISTANCE, NEAR_VAL);
		input_sync(ltr559->ps_input_dev);
		LTR559_DBG("%s: ps report %d NORMAL NEAR, default_ps_lowthresh=%d, default_ps_highthresh=%d ",
			__func__, adc_value, ltr559->default_ps_lowthresh,	ltr559->default_ps_highthresh);
		LTR559_DBG("%s: oil far thresh = %d , oil near thresh = %d\n",
			__func__, atomic_read(&ltr559->ps_persist_val_low), atomic_read(&ltr559->ps_persist_val_high));

#ifdef DYNAMIC_PS_CALI
		ret = set_ps_range(ltr559->default_ps_lowthresh + PS_COUNTER_DELTA,
							PS_MAX_MEASURE_VAL,	LO_N_HI_LIMIT,	ltr559);
		sample_times = 0;
		record_ps_count = adc_value;
#else
		ret = set_ps_range(ltr559->default_ps_lowthresh,
		atomic_read(&ltr559->ps_persist_val_high), LO_N_HI_LIMIT, ltr559);
#endif
	} else if (ps_value_nf == 1) {/*normal far*/
		input_report_abs(ltr559->ps_input_dev,	ABS_DISTANCE, FAR_VAL);
		input_sync(ltr559->ps_input_dev);
		LTR559_DBG("%s: ps report %d NORMAL FAR, default_ps_lowthresh=%d, default_ps_highthresh=%d ",
			__func__, adc_value, ltr559->default_ps_lowthresh,	ltr559->default_ps_highthresh);
		LTR559_DBG("%s: oil far thresh = %d , oil near thresh = %d\n",
			__func__, atomic_read(&ltr559->ps_persist_val_low), atomic_read(&ltr559->ps_persist_val_high));

		if (adc_value > 20 && adc_value < last_min_value - 100) {
			LTR559_DBG("%s: ENTER FAR CALIBRATE\n", __func__);
			dynamic_update_ps_threshold(ltr559, adc_value);
		}
		ret = set_ps_range(0, ltr559->default_ps_highthresh, LO_N_HI_LIMIT,	ltr559);
	} else if (ps_value_nf == 2) {/*oil near*/
		input_report_abs(ltr559->ps_input_dev,	ABS_DISTANCE, NEAR_VAL);
		input_sync(ltr559->ps_input_dev);
		LTR559_DBG("%s: ps report %d OIL NEAR, default_ps_lowthresh=%d, default_ps_highthresh=%d ",
			__func__, adc_value, ltr559->default_ps_lowthresh,	ltr559->default_ps_highthresh);
		LTR559_DBG("%s: oil far thresh = %d , oil near thresh = %d\n",
			__func__, atomic_read(&ltr559->ps_persist_val_low), atomic_read(&ltr559->ps_persist_val_high));
		ret = set_ps_range(atomic_read(&ltr559->ps_persist_val_low), 2047, LO_N_HI_LIMIT,	ltr559);

	} else if (ps_value_nf == 3) {/*oil far*/
		if (adc_value > 20) {
			LTR559_DBG("%s: ENTER  OIL FAR CALIBRATE---\n", __func__);
			dynamic_update_ps_threshold(ltr559, adc_value);
			last_min_value = adc_value;
		}
		input_report_abs(ltr559->ps_input_dev,	ABS_DISTANCE, FAR_VAL);
		input_sync(ltr559->ps_input_dev);
		LTR559_DBG("%s: ps report %d OIL FAR, default_ps_lowthresh=%d, default_ps_highthresh=%d ",
			__func__, adc_value, ltr559->default_ps_lowthresh,	ltr559->default_ps_highthresh);
		LTR559_DBG("%s: oil far thresh = %d , oil near thresh = %d\n",
			__func__, atomic_read(&ltr559->ps_persist_val_low), atomic_read(&ltr559->ps_persist_val_high));
		ret = set_ps_range(ltr559->default_ps_lowthresh, ltr559->default_ps_highthresh, LO_N_HI_LIMIT,	ltr559);
	} else {
#ifdef DYNAMIC_PS_CALI
		LTR559_DBG("%s: ps_val between threshold, adc_value=%d, recodepscnt=%d\n",
					 __func__,	adc_value,	record_ps_count);
		if (adc_value >= record_ps_count + 20 || adc_value <= record_ps_count - 20) {
			record_ps_count = adc_value;
			sample_times = 0;
		} else {
			sample_times++;
		}

		if (sample_times >= 10) {
			input_report_abs(ltr559->ps_input_dev,	ABS_DISTANCE, FAR_VAL);
			sample_times = 0;
			LTR559_DBG("%s: ps report %d FAR and reset threshold\n", __func__,	adc_value);
			updateThreshold(adc_value);
		}
#else
		LTR559_DBG("%s: ps_val between threshold\n", __func__);
#endif
	}
#else  /*CALLING_DYNAMIC_CALI*/
	if (adc_value >  ltr559->default_ps_highthresh) {
		/* Begin: Fix ps wrong report near in sunshine mode */
		/* temp set 6000lux, you can change it */
		LTR559_DBG("%s: CT=%d, Lux=%d\n", __func__, adc_value, als_adc_value);
		#ifdef LTR559_REG_DUMP_ENABLE
		ltr559_reg_dump();
		#endif
		if (als_adc_value > 6000) {
			LTR559_DBG("%s: ps wrong report NEAR, just return\n", __func__);
			return;
		}
		/* End: Fix ps wrong report near in sunshine mode */
		input_report_abs(ltr559->ps_input_dev,	ABS_DISTANCE, NEAR_VAL);
		input_sync(ltr559->ps_input_dev);
		LTR559_DBG("%s: ps report %d NEAR,	default_ps_lowthresh=%d,	default_ps_highthresh=%d\n",
		 __func__,	adc_value,	ltr559->default_ps_lowthresh,	ltr559->default_ps_highthresh);

#ifdef DYNAMIC_PS_CALI
		ret = set_ps_range(ltr559->default_ps_lowthresh + PS_COUNTER_DELTA,
							PS_MAX_MEASURE_VAL,	LO_N_HI_LIMIT,	ltr559);
		sample_times = 0;
		record_ps_count = adc_value;
#else
		ret = set_ps_range(ltr559->default_ps_lowthresh, PS_MAX_MEASURE_VAL,	LO_N_HI_LIMIT,	ltr559);
#endif
	} else if (adc_value < ltr559->default_ps_lowthresh) {
		input_report_abs(ltr559->ps_input_dev,	ABS_DISTANCE, FAR_VAL);
		input_sync(ltr559->ps_input_dev);
		LTR559_DBG("%s: ps report %d FAR,	default_ps_lowthresh=%d, default_ps_highthresh=%d\n",
		 __func__,	adc_value,	ltr559->default_ps_lowthresh,	ltr559->default_ps_highthresh);
		ret = set_ps_range(0, ltr559->default_ps_highthresh, LO_N_HI_LIMIT,	ltr559);
	} else {
#ifdef DYNAMIC_PS_CALI
		LTR559_DBG("%s: ps_val between threshold, adc_value=%d, recodepscnt=%d\n",
					 __func__,	adc_value,	record_ps_count);
		if (adc_value >= record_ps_count + 20 || adc_value <= record_ps_count - 20) {
			record_ps_count = adc_value;
			sample_times = 0;
		} else {
			sample_times++;
		}

		if (sample_times >= 10) {
			input_report_abs(ltr559->ps_input_dev,	ABS_DISTANCE, FAR_VAL);
			sample_times = 0;
			LTR559_DBG("%s: ps report %d FAR and reset threshold\n", __func__,	adc_value);
			updateThreshold(adc_value);
		}
#else
		LTR559_DBG("%s: ps_val between threshold\n", __func__);
#endif
	}
#endif  /*CALLING_DYNAMIC_CALI*/
}

/*
static void report_ps_input_event(struct ltr559_data *ltr559)
{
	int8_t ret;
	uint16_t adc_value;

	adc_value = read_ps_adc_value(ltr559);

	if (ps_grabData_stage == 0) {
		if (ps_kept_data_counter < PS_MAX_INIT_KEPT_DATA_COUNTER) {
			if (adc_value != 0) {
				ps_init_kept_data[ps_kept_data_counter] =
					adc_value;
				ps_kept_data_counter++;
			}
		}

		if (ps_kept_data_counter >= PS_MAX_INIT_KEPT_DATA_COUNTER) {
			ps_ct_avg = findCT_Avg(ps_init_kept_data);
			ftn_init = ps_ct_avg * 15;
			ps_grabData_stage = 1;
		}
	}

	if (ps_grabData_stage == 1) {
		if ((ftn_init - (ps_ct_avg * 10)) < 1000)
			ftn_final = (ps_ct_avg * 10) + 1000;
		else {
		if ((ftn_init - (ps_ct_avg * 10)) >  1500)
				ftn_final = (ps_ct_avg * 10) + 1500;
			else
				ftn_final = ftn_init;
		}
		ntf_final = (ftn_final - (ps_ct_avg * 10));
		ntf_final *= 4;
		ntf_final /= 100;
		ntf_final += ps_ct_avg;
		ftn_final /= 10;
		if (ntf_final >= PS_MAX_MEASURE_VAL)
			ntf_final = PS_MAX_MEASURE_VAL;

		if (ftn_final >= PS_MAX_MEASURE_VAL)
			ftn_final = PS_MAX_MEASURE_VAL;

		ret = ps_meas_rate_setup(50,	ltr559);
		if (ret < 0) {
			dev_err(&ltr559->i2c_client->dev,
				"%s: PS MeasRate Setup Fail...\n", __func__);
		}

		ps_grabData_stage = 2;
	}

	if (ps_grabData_stage == 2) {
		if ((adc_value >  ftn_final) || (adc_value < ntf_final)) {

			if (adc_value >  ftn_final) {
				input_report_abs(ltr559->ps_input_dev,
					ABS_DISTANCE,	NEAR_VAL);
				input_sync(ltr559->ps_input_dev);
			}

			if (adc_value < ntf_final) {
				input_report_abs(ltr559->ps_input_dev,
					ABS_DISTANCE,	FAR_VAL);
				input_sync(ltr559->ps_input_dev);
			}

		}

		if (ps_movavg_data_counter < PS_MAX_MOV_AVG_KEPT_DATA_CTR) {
			if (adc_value != 0) {
				ps_movavg_data[ps_movavg_data_counter] =
					adc_value;
				ps_movavg_data_counter++;
			}
		}

		if (ps_movavg_data_counter >= PS_MAX_MOV_AVG_KEPT_DATA_CTR) {
			ps_movct_avg =
				discardMinMax_findCTMov_Avg(ps_movavg_data);

			if (ps_movct_avg < ps_ct_avg) {
				ps_ct_avg = ps_movct_avg;
				ftn_init = ps_ct_avg * 17;
				ps_grabData_stage = 1;
			}
			ps_movavg_data_counter = 5;
		}

	}
}
*/
/* Report ALS input event */
static void report_als_input_event(struct ltr559_data *ltr559)
{
	int adc_value;
	static int max_lux_diff = 0;

	adc_value = read_als_adc_value(ltr559);

	if (adc_value == 50000) {
		adc_value = adc_value - max_lux_diff;
		max_lux_diff = !max_lux_diff;
	}

	if (1 == als_enable_falg) {
		if (adc_value == 0) {
			LTR559_DBG("%s: als_enable_falg=%d\n", __func__,	als_enable_falg);
			adc_value = adc_value + 2;
		}
		als_enable_falg = 0;
	}

	LTR559_DBG2("%s: adc_value_raw=%d,	als_enable_falg=%d\n", __func__,
				adc_value,	als_enable_falg);

	input_report_abs(ltr559->als_input_dev,	ABS_MISC,	(adc_value));
	input_sync(ltr559->als_input_dev);

}
/* ALS polling routine */
static void ltr559_als_polling_work_func(struct work_struct *work)
{
	struct ltr559_data *data = container_of(work,
			struct ltr559_data,	als_dwork.work);

	report_als_input_event(data);

	queue_delayed_work(data->als_wq,
			&data->als_dwork,
			msecs_to_jiffies(data->als_poll_delay));
}

/* Work when interrupt */
static void ltr559_irq_work_func(struct work_struct *work)
{
	int8_t ret;
	uint8_t status;
	uint8_t	interrupt_stat,	newdata;
	struct ltr559_data *ltr559 = sensor_info;
	uint8_t buffer[2];

	LTR559_DBG2("%s:\n", __func__);

	buffer[0] = LTR559_ALS_PS_STATUS;
	ret = I2C_Read(buffer,	1);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s | 0x%02X", __func__,	buffer[0]);
		return;
	}
	status = buffer[0];
	interrupt_stat = status & 0x0A;
	newdata = status & 0x05;
	LTR559_DBG("%s: read status=0x%X\n", __func__,	status);

	/* PS interrupt and PS with new data*/
	if ((interrupt_stat & 0x02) && (newdata & 0x01)) {
		LTR559_DBG("%s: is ps interrupt\n", __func__);
		ltr559->ps_irq_flag = 1;

		report_ps_input_event_zte(ltr559);
		ltr559->ps_irq_flag = 0;
	}
	/* ALS interrupt and ALS with new data*/
	if ((interrupt_stat & 0x08) && (newdata & 0x04)) {
		LTR559_DBG("%s: als interrupt\n", __func__);
		ltr559->als_irq_flag = 1;
		report_als_input_event(ltr559);
		ltr559->als_irq_flag = 0;
	}
	enable_irq(ltr559->irq);
}

static DECLARE_WORK(irq_workqueue,	ltr559_irq_work_func);


/* IRQ Handler */
static irqreturn_t ltr559_irq_handler(int irq,	void *data)
{
	struct ltr559_data *ltr559 = data;
	if (ltr559->enable_ps_sensor == 0)
		return IRQ_HANDLED;

	/* disable an irq without waiting */
	disable_irq_nosync(ltr559->irq);

	schedule_work(&irq_workqueue);

	return IRQ_HANDLED;
}

static int ltr559_gpio_irq(struct ltr559_data *ltr559)
{
	int rc = 0;

	rc = gpio_request(ltr559->gpio_int_no,	LTR559_DEVICE_NAME);
	if (rc < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: GPIO %d Request Fail (%d)\n",
			 __func__,	ltr559->gpio_int_no,	rc);
		return rc;
	}

	rc = gpio_direction_input(ltr559->gpio_int_no);
	if (rc < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: Set GPIO %d as Input Fail (%d)\n", __func__,
					ltr559->gpio_int_no,	rc);
		goto out1;
	}

	/* Configure an active low trigger interrupt for the device */
	rc = request_irq(ltr559->irq,	ltr559_irq_handler,	IRQF_TRIGGER_FALLING,
				LTR559_DEVICE_NAME,	ltr559);
	if (rc < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: Request IRQ (%d) for GPIO %d Fail (%d)\n",
			 __func__,	ltr559->irq,
					ltr559->gpio_int_no,	rc);
		goto out1;
	}
	disable_irq(ltr559->irq);

	return rc;

out1:
	gpio_free(ltr559->gpio_int_no);

	return rc;
}

/* PS Enable */
static int8_t ps_enable_init(struct ltr559_data *ltr559)
{
	int8_t rc = 0;
	uint8_t buffer[1]; /* for dummy read*/

	LTR559_DBG("%s:\n", __func__);

	if (0) {
		setThrDuringCall();
	}

	if (ltr559->ps_enable_flag) {
		dev_info(&ltr559->i2c_client->dev,
			"%s: already enabled\n", __func__);
		return 0;
	}

	/* Set thresholds where interrupt will *not* be generated */
#if ACT_INTERRUPT
	/*rc = set_ps_range(PS_MIN_MEASURE_VAL,	PS_MIN_MEASURE_VAL,
				LO_N_HI_LIMIT);*/
	/*rc = set_ps_range(PS_MIN_MEASURE_VAL,	400,	LO_N_HI_LIMIT);*/
	rc = set_ps_range(ltr559->default_ps_highthresh,
		ltr559->default_ps_lowthresh,	LO_N_HI_LIMIT,	ltr559);
#else
	rc = set_ps_range(PS_MIN_MEASURE_VAL,	PS_MAX_MEASURE_VAL,
			LO_N_HI_LIMIT,	ltr559);
#endif
	if (rc < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s : PS Thresholds Write Fail...\n", __func__);
		return rc;
	}

	rc = ps_led_setup(0x7F,	ltr559);
	if (rc < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: PS LED Setup Fail...\n", __func__);
		return rc;
	}

	rc = ps_ledPulseCount_setup(8,	ltr559);
	if (rc < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: PS LED pulse count setup Fail...\n", __func__);
	}

	rc = ps_meas_rate_setup(10,	ltr559);
	if (rc < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: PS MeasRate Setup Fail...\n", __func__);
		return rc;
	}

	rc = ps_contr_setup(0x00,	ltr559);
	if (rc < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: PS Enable Fail...\n", __func__);
		return rc;
	}

	rc = interrupt_persist_setup(0x50, ltr559);
	if (rc < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: PS Enable Fail...\n", __func__);
		return rc;
	}

	/* dummy read*/
	buffer[0] = LTR559_PS_CONTR;
	I2C_Read(buffer,	1);
	/* dummy read*/

	ltr559->ps_enable_flag = 0;

	return rc;
}


/* PS Disable */
static int8_t ps_disable(struct ltr559_data *ltr559)
{
	int8_t rc = 0;

	if (ltr559->ps_enable_flag == 0) {
		dev_info(&ltr559->i2c_client->dev,
			"%s: already disabled\n", __func__);
		return 0;
	}

	/*rc = _ltr559_set_bit(ltr559->i2c_client,
					CLR_BIT,	LTR559_PS_CONTR,	PS_MODE);*/
	rc = ps_mode_setup(CLR_BIT,	ltr559);
	if (rc < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: PS Disable Fail...\n", __func__);
		return rc;
	}

	ltr559->ps_enable_flag = 0;

	return rc;
}


/* PS open fops */
int ps_open(struct inode *inode,	struct file *file)
{
	struct ltr559_data *ltr559 = sensor_info;

	if (ltr559->ps_opened)
		return -EBUSY;

	ltr559->ps_opened = 1;

	return 0;
}


/* PS release fops */
int ps_release(struct inode *inode,	struct file *file)
{
	struct ltr559_data *ltr559 = sensor_info;

	ltr559->ps_opened = 0;

	return ps_disable(ltr559);
}

/* PS IOCTL */
static long ps_ioctl(struct file *file,	unsigned int cmd,	unsigned long arg)
{
	int rc = 0,	val = 0;
	struct ltr559_data *ltr559 = sensor_info;

	LTR559_DBG("%s: cmd %d\n", __func__,	_IOC_NR(cmd));

	switch (cmd) {
	case LTR559_IOCTL_PS_ENABLE:
			if (get_user(val,	(unsigned long __user *)arg)) {
				rc = -EFAULT;
				break;
			}
			rc = val ? ps_enable_init(ltr559) : ps_disable(ltr559);

			break;
	case LTR559_IOCTL_PS_GET_ENABLED:
			rc = put_user(ltr559->ps_enable_flag,
					(unsigned long __user *)arg);

			break;
	default:
			pr_err("%s: INVALID COMMAND %d\n",
			 __func__,	_IOC_NR(cmd));
			rc = -EINVAL;
	}

	return rc;
}

static const struct file_operations ps_fops = {
	.owner = THIS_MODULE,
	.open = ps_open,
	.release = ps_release,
	.unlocked_ioctl = ps_ioctl
};

struct miscdevice ps_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ltr559_ps",
	.fops = &ps_fops
};


static int8_t als_enable_init(struct ltr559_data *ltr559)
{
	int8_t rc = 0;
	uint8_t buffer[1]; /* for dummy read*/

	/* if device not enabled,	enable it */
	if (ltr559->als_enable_flag) {
		dev_info(&ltr559->i2c_client->dev,
				"%s: ALS already enabled...\n", __func__);
		return rc;
	}

	rc = als_meas_rate_reg_setup(0x01,	ltr559);
	if (rc < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: ALS_Meas_Rate register Setup Fail...\n", __func__);
		return rc;
	}

	/* Set minimummax thresholds where interrupt will *not* be generated */

	rc = als_contr_setup(0x0C,	ltr559);
	if (rc < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: ALS Enable Fail...\n", __func__);
		return rc;
	}

	/* dummy read*/
	buffer[0] = LTR559_ALS_CONTR;
	I2C_Read(buffer,	1);
	/* dumy read*/

	ltr559->als_enable_flag = 0;

	return rc;
}

#include "ltr559-sysfs.c"

static int als_setup(struct ltr559_data *ltr559)
{
	int ret;

	LTR559_DBG("%s:\n", __func__);

	ltr559->als_input_dev = input_allocate_device();
	if (!ltr559->als_input_dev) {
		dev_err(&ltr559->i2c_client->dev,
		"%s: ALS Input Allocate Device Fail...\n", __func__);
		return -ENOMEM;
	}
	ltr559->als_input_dev->name = "light";
	set_bit(EV_ABS,	ltr559->als_input_dev->evbit);
	input_set_abs_params(ltr559->als_input_dev,	ABS_MISC,
		ALS_MIN_MEASURE_VAL,	ALS_MAX_MEASURE_VAL,	0,	0);

	ret = input_register_device(ltr559->als_input_dev);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: ALS Register Input Device Fail...\n", __func__);
		goto err_als_register_input_device;
	}
/*
	ret = misc_register(&als_misc);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
		"%s: ALS Register Misc Device Fail...\n", __func__);
		goto err_als_register_misc_device;
	}
*/
	return ret;

/*err_als_register_misc_device:
	input_unregister_device(ltr559->als_input_dev);*/
err_als_register_input_device:
	input_free_device(ltr559->als_input_dev);

	return ret;
}


static int ps_setup(struct ltr559_data *ltr559)
{
	int ret;

	LTR559_DBG("%s:\n", __func__);

	ltr559->ps_input_dev = input_allocate_device();
	if (!ltr559->ps_input_dev) {
		dev_err(&ltr559->i2c_client->dev,
		"%s: PS Input Allocate Device Fail...\n", __func__);
		return -ENOMEM;
	}
	ltr559->ps_input_dev->name = "proximity";
	set_bit(EV_ABS,	ltr559->ps_input_dev->evbit);
	input_set_abs_params(ltr559->ps_input_dev,
		ABS_DISTANCE,	PS_MIN_MEASURE_VAL,	PS_MAX_MEASURE_VAL,	0,	0);
	ltr559->ps_input_dev->absinfo[ABS_DISTANCE].value = 1;

	ret = input_register_device(ltr559->ps_input_dev);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
		"%s: PS Register Input Device Fail...\n", __func__);
		goto err_ps_register_input_device;
	}

	ret = misc_register(&ps_misc);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
		"%s: PS Register Misc Device Fail...\n", __func__);
		goto err_ps_register_misc_device;
	}

	return ret;

err_ps_register_misc_device:
	input_unregister_device(ltr559->ps_input_dev);
err_ps_register_input_device:
	input_free_device(ltr559->ps_input_dev);

	return ret;
}


static int _check_part_id(struct ltr559_data *ltr559)
{
	int ret;
	uint8_t buffer[2];

	buffer[0] = LTR559_PART_ID;
	ret = I2C_Read(buffer,	1);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,	"%s: Read failure :0x%02X",
	 __func__,	buffer[0]);
		return -EPERM;
	}
	LTR559_DBG("%s: read LTR559_PART_ID = 0x%X\n", __func__,	buffer[0]);

	if (buffer[0] != PARTID) {
		dev_err(&ltr559->i2c_client->dev,
		"%s: Part failure miscompare act:0x%02x exp:0x%02x\n",
	 __func__,	buffer[0],	PARTID);
		return -ENOENT;
	}
	return 0;
}


static int ltr559_setup(struct ltr559_data *ltr559)
{
	int ret = 0;

	LTR559_DBG("%s:\n", __func__);

	/* Reset the devices */
	ret = _ltr559_set_bit(ltr559->i2c_client,	SET_BIT,
						LTR559_ALS_CONTR,	ALS_SW_RESET);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
		"%s: ALS reset fail...\n", __func__);
		goto err_out1;
	}

	ret = _ltr559_set_bit(ltr559->i2c_client,	CLR_BIT,
					LTR559_PS_CONTR,	PS_MODE_ACTIVE);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: PS reset fail...\n", __func__);
		goto err_out1;
	}

	msleep(PON_DELAY);
	dev_dbg(&ltr559->i2c_client->dev,
		"%s: Reset ltr559 device\n", __func__);

	/* Do another part read to ensure we have exited reset */
	if (_check_part_id(ltr559) < 0) {
		dev_err(&ltr559->i2c_client->dev,
		"%s: Part ID Read Fail after reset...\n", __func__);
		goto err_out1;
	}

	ret = ltr559_gpio_irq(ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
		"%s: GPIO Request Fail...\n", __func__);
		goto err_out1;
	}
	dev_dbg(&ltr559->i2c_client->dev,
		"%s: Requested interrupt\n", __func__);

	/* Set count of measurements outside data range before interrupt is generated */
	ret = _ltr559_set_bit(ltr559->i2c_client,	SET_BIT,
						LTR559_INTERRUPT_PRST,	0x01);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: ALS Set Persist Fail...\n", __func__);
	goto err_out2;
	}

	ret = _ltr559_set_bit(ltr559->i2c_client,
					SET_BIT,	LTR559_INTERRUPT_PRST,	0x10);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
		"%s: PS Set Persist Fail...\n", __func__);
		goto err_out2;
	}
	dev_dbg(&ltr559->i2c_client->dev,
		"%s: Set ltr559 persists\n", __func__);

	/* Enable interrupts on the device and clear only when status is read */
#if ACT_INTERRUPT
	/*ret = _ltr559_set_bit(ltr559->i2c_client,
			SET_BIT,	LTR559_INTERRUPT,	INT_MODE_ALSPS_TRIG);*/
	ret = _ltr559_set_bit(ltr559->i2c_client,	SET_BIT,
				LTR559_INTERRUPT,	INT_MODE_PS_TRIG);
#else
	ret = _ltr559_set_bit(ltr559->i2c_client,	SET_BIT,
					LTR559_INTERRUPT,	INT_MODE_00);
#endif
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: Enabled interrupts failed...\n", __func__);
		goto err_out2;
	}
	dev_dbg(&ltr559->i2c_client->dev,
			"%s Enabled interrupt to device\n", __func__);

	/* Turn on ALS and PS */
	ret = als_enable_init(ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s Unable to enable ALS", __func__);
		goto err_out2;
	}

	ret = ps_enable_init(ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
		"%s Unable to enable PS", __func__);
		goto err_out2;
	}

	return ret;

err_out2:
	free_irq(ltr559->irq,	ltr559);
	gpio_free(ltr559->gpio_int_no);

err_out1:
	dev_err(&ltr559->i2c_client->dev,
		"%s Unable to setup device\n", __func__);

	return ret;
}

static int ltr559_als_set_enable(struct sensors_classdev *sensors_cdev,
			unsigned int enable)
{
	struct ltr559_data *data = container_of(sensors_cdev,
			struct ltr559_data,	als_cdev);
	int ret;

	if ((enable != 0) && (enable != 1)) {
		pr_err("%s: invalid value(%d)\n", __func__,	enable);
		return -EINVAL;
	}
	LTR559_DBG("%s:+++ enable=%d\n", __func__,	enable);
	LTR559_DBG("%s: [ZTE-SNS] touch_module=%d, winfac1=%d, winfac2=%d, winfac3=%d\n", __func__,
					 syna_touch_module_for_lsensor,	winfac1,	winfac2, winfac3);

	if ((1 == atomic_read(&ltr559_resume_poweron_flag)) && (enable)) {
		atomic_set(&ltr559_resume_poweron_flag,	0);
		if (!(time_after(jiffies,	ltr559_poweron_endt))) {
			LTR559_DBG("%s: msleep 100ms\n", __func__);
			msleep(100);
		}
		ltr559_poweron_reg_config();
	}

	ret = als_mode_setup((uint8_t)enable,	data);
	if (0 == ret)
		data->enable_als_sensor = enable;

	if (1 == enable) {
		data->als_poll_delay = 200;
		queue_delayed_work(data->als_wq,
			&data->als_dwork,
			msecs_to_jiffies(data->als_poll_delay));
		als_enable_falg = 1;
	}

	if (0 == enable) {
		cancel_delayed_work(&data->als_dwork);
		flush_delayed_work(&data->als_dwork);
	}
	LTR559_DBG("%s: ---\n", __func__);
	return ret;
}

/* Begin: Fix ps wrong report near in sunshine mode */
static int ltr559_en_dis_als_for_ps(struct sensors_classdev *sensors_cdev,
			unsigned int ps_enable, unsigned int als_enable)
{
	struct ltr559_data *data = container_of(sensors_cdev,
			struct ltr559_data, als_cdev);

	if (((ps_enable != 0) && (ps_enable != 1))
		|| ((als_enable != 0) && (als_enable != 1))) {
		LTR559_DBG("%s: [ZTE-SNS]invalid value ps_enable=%d,als_enable=%d\n", __func__,
			ps_enable, als_enable);
		return -EINVAL;
	}

	LTR559_DBG("%s: [ZTE-SNS]ps_enable=%d,als_enable=%d\n", __func__, ps_enable, als_enable);

	if (1 == ps_enable && 1 == als_enable) {
		LTR559_DBG("%s: [ZTE-SNS]Als is already enabled, just return!\n", __func__);
		return 0;
	}
	if (1 == ps_enable && 0 == als_enable) {
		als_mode_setup(1, data);
		LTR559_DBG("%s: [ZTE-SNS]Enable als when ps is enabled\n", __func__);
	}
	if (0 == ps_enable && 1 == als_enable) {
		LTR559_DBG("%s: [ZTE-SNS]Als is still enabled, just return!\n", __func__);
		return 0;
	}
	if (0 == ps_enable && 0 == als_enable) {
		als_mode_setup(0, data);
		LTR559_DBG("%s: [ZTE-SNS]Disable als when ps is disabled\n", __func__);
	}

	return 0;

}
/* End: Fix ps wrong report near in sunshine mode */

static int ltr559_ps_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct ltr559_data *data = container_of(sensors_cdev,
			struct ltr559_data,	ps_cdev);
	int ret;

	if ((enable != 0) && (enable != 1)) {
		pr_err("%s: invalid value(%d)\n", __func__,	enable);
		return -EINVAL;
	}
	LTR559_DBG("%s:+++ enable=%d\n", __func__,	enable);

	if ((1 == atomic_read(&ltr559_resume_poweron_flag)) && (enable)) {
		atomic_set(&ltr559_resume_poweron_flag,	0);
		if (!(time_after(jiffies,	ltr559_poweron_endt))) {
			LTR559_DBG("%s: msleep 100ms\n", __func__);
			msleep(100);
		}
		ltr559_poweron_reg_config();
	}

	ret = ps_mode_setup((uint8_t)enable,	data);
	LTR559_DBG("%s:+++ enable  ps mode setup--ret=%d\n", __func__,	ret);

	if (0 == ret) {
		data->enable_ps_sensor = enable;

		if (enable) {
#if CALLING_DYNAMIC_CALI
			ltr559_dynamic_calibrate();
#else
			ltr559_calibrate_dial();
#endif
			enable_irq(data->irq);
			irq_set_irq_wake(data->i2c_client->irq,	1);
			} else {
			irq_set_irq_wake(data->i2c_client->irq,	0);
			disable_irq(data->irq);
		}

		/* Fix ps wrong report near in sunshine mode */
		ltr559_en_dis_als_for_ps(sensors_cdev, data->enable_ps_sensor, data->enable_als_sensor);
	}
	LTR559_DBG("%s: ---\n", __func__);
	return ret;
}

static int ltr559_set_als_poll_delay(struct i2c_client *client,
		unsigned int val)
{

	struct ltr559_data *als_ps = i2c_get_clientdata(client);

	/*int ret;
	int poll_delay = 0;
	unsigned long flags;*/
	LTR559_DBG("%s: delay=%d ms\n", __func__,	val);

	if (val < 100)
		val = 100;

	als_ps->als_poll_delay = val;

	if (als_ps->enable_als_sensor == 1) {

		/* we need this polling timer routine for sunlight canellation */
		/*spin_lock_irqsave(&als_ps->update_lock.wait_lock,	flags);*/

		/*
		 * If work is already scheduled then subsequent schedules will not
		 * change the scheduled time that's why we have to cancel it first.
		 */
		cancel_delayed_work(&als_ps->als_dwork);
		flush_delayed_work(&als_ps->als_dwork);
		queue_delayed_work(als_ps->als_wq,
				&als_ps->als_dwork,
				msecs_to_jiffies(als_ps->als_poll_delay));

		/*spin_unlock_irqrestore(&als_ps->update_lock.wait_lock,	flags);*/

	}
	return 0;
}

static int ltr559_als_poll_delay(struct sensors_classdev *sensors_cdev,
		unsigned int delay_msec)
{
	struct ltr559_data *data = container_of(sensors_cdev,
			struct ltr559_data,	als_cdev);
	LTR559_DBG("%s: %d ms\n", __func__,	delay_msec);
	ltr559_set_als_poll_delay(data->i2c_client,	delay_msec);
	return 0;
}
static int ltr_power_on(struct ltr559_data *data,	bool on)
{
	int rc;

	LTR559_DBG("%s: on=%d\n", __func__,	on);

	if (!on)
		goto power_off;

	rc = regulator_enable(data->vdd);
	if (rc) {
		dev_err(&data->i2c_client->dev,
			"Regulator vdd enable failed rc=%d\n",	rc);
		return rc;
	}

	rc = regulator_enable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->i2c_client->dev,
			"Regulator vcc_i2c enable failed rc=%d\n",	rc);
		regulator_disable(data->vdd);
	}

#ifdef ZTE_LTR559_PINCTRL
	rc = pinctrl_select_state(data->pinctrl,	data->pin_default);
	if (rc) {
		dev_err(&data->i2c_client->dev,
			"Can't select pinctrl default state\n");
	}
#endif

	return rc;

power_off:
	rc = regulator_disable(data->vdd);
	if (rc) {
		dev_err(&data->i2c_client->dev,
			"Regulator vdd disable failed rc=%d\n",	rc);
		return rc;
	}

	rc = regulator_disable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->i2c_client->dev,
			"Regulator vcc_i2c disable failed rc=%d\n",	rc);
	}

#ifdef ZTE_LTR559_PINCTRL
	rc = pinctrl_select_state(data->pinctrl,	data->pin_sleep);
	if (rc) {
		dev_err(&data->i2c_client->dev,
			"Can't select pinctrl sleep state\n");
	}
#endif

	return rc;
}
static int ltr_power_init(struct ltr559_data *data,	bool on)
{
	int rc;

	LTR559_DBG("%s: on=%d\n", __func__, on);
	if (!on)
		goto pwr_deinit;

	data->vdd = regulator_get(&data->i2c_client->dev,	"vdd");
	if (IS_ERR(data->vdd)) {
		rc = PTR_ERR(data->vdd);
		dev_err(&data->i2c_client->dev,
			"Regulator get failed vdd rc=%d\n",	rc);

		return rc;
	}

	if (regulator_count_voltages(data->vdd) >  0) {
		rc = regulator_set_voltage(data->vdd,	FT_VTG_MIN_UV,
						 FT_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->i2c_client->dev,
				"Regulator set_vtg failed vdd rc=%d\n",	rc);
			goto reg_vdd_put;
		}
	}

	data->vcc_i2c = regulator_get(&data->i2c_client->dev,	"vio");
	if (IS_ERR(data->vcc_i2c)) {
		rc = PTR_ERR(data->vcc_i2c);
		dev_err(&data->i2c_client->dev,
			"Regulator get failed vcc_i2c rc=%d\n",	rc);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(data->vcc_i2c) >  0) {
		rc = regulator_set_voltage(data->vcc_i2c,	FT_I2C_VTG_MIN_UV,
						 FT_I2C_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->i2c_client->dev,
			"Regulator set_vtg failed vcc_i2c rc=%d\n",	rc);
			goto reg_vcc_i2c_put;
		}
	}

	return 0;

reg_vcc_i2c_put:
	regulator_put(data->vcc_i2c);
reg_vdd_set_vtg:
	if (regulator_count_voltages(data->vdd) >  0)
		regulator_set_voltage(data->vdd,	0,	FT_VTG_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;

pwr_deinit:
	if (regulator_count_voltages(data->vdd) >  0)
		regulator_set_voltage(data->vdd,	0,	FT_VTG_MAX_UV);

	regulator_put(data->vdd);

	if (regulator_count_voltages(data->vcc_i2c) >  0)
		regulator_set_voltage(data->vcc_i2c,	0,	FT_I2C_VTG_MAX_UV);

	regulator_put(data->vcc_i2c);
	return 0;
}

static void check_prox_mean(int prox_mean , int *detection_threshold,	int *hsyteresis_threshold)
{
	int prox_threshold_hi_param,	prox_threshold_lo_param;

#if defined(CONFIG_BOARD_FORTUNE)
	if (prox_mean < 1200) {
		prox_threshold_hi_param = prox_mean + 150;
		prox_threshold_lo_param = prox_mean + 95;
	} else {
		prox_threshold_hi_param = 1900;
		prox_threshold_lo_param = 1500;
	}
#elif defined(CONFIG_BOARD_CAPTAIN) || defined(CONFIG_BOARD_BENZ)
	if (prox_mean < 1200) {
		prox_threshold_hi_param = prox_mean + 160;
		prox_threshold_lo_param = prox_mean + 90;
	} else {
		prox_threshold_hi_param = 1900;
		 prox_threshold_lo_param = 1500;
	}
#elif defined(CONFIG_BOARD_SWEET)
	if (prox_mean < 1200) {
		prox_threshold_hi_param = prox_mean + 115;
		prox_threshold_lo_param = prox_mean + 60;
	} else {
		prox_threshold_hi_param = 1900;
		prox_threshold_lo_param = 1500;
	}
#else
	if (prox_mean < 1200) {
		prox_threshold_hi_param = prox_mean + 180;
		prox_threshold_lo_param = prox_mean + 95;
	} else {
		prox_threshold_hi_param = 1900;
		prox_threshold_lo_param = 1500;
	}
#endif
	*detection_threshold = prox_threshold_hi_param;
	*hsyteresis_threshold = prox_threshold_lo_param;

	LTR559_DBG("%s: cross_talk=%d,	high_threshold=%d,	low_threshold=%d\n",
	 __func__,	prox_mean,	prox_threshold_hi_param, prox_threshold_lo_param);
}

#if CALLING_DYNAMIC_CALI
static int ltr559_dynamic_calibrate(void)
{
	int i = 0;
	int j = 0;
	int data = 0;
	uint16_t noise = 0;
	int max = 0;
	unsigned long data_total = 0;
	struct ltr559_data *ltr559 = sensor_info;

	msleep(20);
	for (i = 0; i < 5; i++) {
		if (max++ > 5)
			goto err;
		usleep_range(15000, 15500);

		data = read_ps_adc_value(ltr559);
		if (data < 0) {
				dev_err(&ltr559->i2c_client->dev, "%s: read PS data Fail...\n", __func__);
				return data;
		}

		if (data == 0)
			j++;
		data_total += data;
	}
	noise = data_total/(5 - j);

	dev_info(&ltr559->i2c_client->dev, "%s: read PS data  noise = %d\n", __func__, noise);

	if ((noise < last_min_value + 300)) {
		last_min_value = noise;
		dynamic_update_ps_threshold(ltr559, noise);

	}
	set_ps_range(ltr559->default_ps_lowthresh, ltr559->default_ps_highthresh, LO_N_HI_LIMIT, ltr559);

	return 0;
err:
	return -EINVAL;
}
#else
static int ltr559_calibrate_dial(void)
{
	struct ltr559_data *data = sensor_info;
	uint16_t cross_talk = 0;
		 int ret = 0;

	LTR559_DBG("%s:+++\n", __func__);
	LTR559_DBG("%s: old ps thresh is %d %d\n",
	 __func__,	data->default_ps_highthresh,	data->default_ps_lowthresh);

	if (NULL == data) {
		LTR559_DBG("%s: data is NULL!\n", __func__);
		return -EINVAL;
	}

	cross_talk = getCrosstalk(3);
	LTR559_DBG("%s: cross_talk=%d\n", __func__,	cross_talk);

		/*Check whether it is one good prox or not*/
	if ((cross_talk < CT_LIMIT_LOW) || (cross_talk > CT_LIMIT_HIGH)) {
		ret = -110;
		LTR559_DBG("%s: cross_talk is not in the range of 10-1200\n", __func__);
		LTR559_DBG("%s: ---return %d\n", __func__,	ret);
		return ret;
	}

		/*Update prox threshold*/
	ret = updateThreshold(cross_talk);
	LTR559_DBG("%s: --- return ret=%d\n", __func__, ret);

	return ret;
}
#endif

static int ltr559_poweron_reg_config(void)
{
	int ret = 0;
	struct ltr559_data *ltr559 = sensor_info;

	LTR559_DBG("%s:\n", __func__);
	/* Reset the devices */
	ret = _ltr559_set_bit(ltr559->i2c_client,	SET_BIT,
						LTR559_ALS_CONTR,	ALS_SW_RESET);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
		"%s: ALS reset fail...\n", __func__);
	}

	ret = _ltr559_set_bit(ltr559->i2c_client,	CLR_BIT,
					LTR559_PS_CONTR,	PS_MODE_ACTIVE);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: PS reset fail...\n", __func__);
	}

/*
	ret = ltr559_gpio_irq(ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
		"%s: GPIO Request Fail...\n", __func__);
		goto err_out1;
	}
*/
/* Set count of measurements outside data range before interrupt is generated */
	ret = _ltr559_set_bit(ltr559->i2c_client,
					SET_BIT,	LTR559_INTERRUPT_PRST,	0x10);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
		"%s: PS Set Persist Fail...\n", __func__);
	}

	/* Enable interrupts on the device and clear only when status is read */
#if ACT_INTERRUPT
	/*ret = _ltr559_set_bit(ltr559->i2c_client,
			SET_BIT,	LTR559_INTERRUPT,	INT_MODE_ALSPS_TRIG);*/
	ret = _ltr559_set_bit(ltr559->i2c_client,	SET_BIT,
				LTR559_INTERRUPT,	INT_MODE_PS_TRIG);
#else
	ret = _ltr559_set_bit(ltr559->i2c_client,	SET_BIT,
					LTR559_INTERRUPT,	INT_MODE_00);
#endif
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: Enabled interrupts failed...\n", __func__);
	}

	/* Turn on ALS and PS */
	ret = als_enable_init(ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s Unable to enable ALS", __func__);
	}

	ret = ps_enable_init(ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
		"%s Unable to enable PS", __func__);
	}

	return ret;
}
static int ltr559_suspend(struct i2c_client *client,	pm_message_t mesg)
{
	struct ltr559_data *als_ps = i2c_get_clientdata(client);

	LTR559_DBG("%s\n", __func__);

	if (is_ltr559_probe_succ_flag == 0)
		return 0;

	if (als_ps->enable_ps_sensor == 0 && als_ps->enable_als_sensor == 0) {
		ltr_power_on(als_ps,	false);
		LTR559_DBG("%s: ltr559 LDO power off\n", __func__);
	}
	return 0;
}

static int ltr559_resume(struct i2c_client *client)
{
	struct ltr559_data *als_ps = i2c_get_clientdata(client);

	LTR559_DBG("%s\n", __func__);

	if (is_ltr559_probe_succ_flag == 0)
		return 0;

	if (als_ps->enable_ps_sensor == 0 && als_ps->enable_als_sensor == 0) {
		LTR559_DBG("%s: ready to power on LDO\n", __func__);
		ltr_power_on(als_ps, true);
		atomic_set(&ltr559_resume_poweron_flag,	1);
		ltr559_poweron_endt = jiffies + 100 / (1000/HZ);
	}

	return 0;
}

static int ltr_parse_dt(struct device *dev,
				struct ltr559_platform_data *ltr_pdata)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int rc;

	rc = of_get_named_gpio_flags(dev->of_node,
				"liteon,irq-gpio", 0, NULL);
	if (rc < 0) {
		dev_err(dev,	"Unable to read liteon, irq-gpio\n");
		return rc;
	}
	ltr_pdata->pfd_gpio_int_no = rc;

	rc = of_property_read_u32(np, "liteon,highthr", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev,	"Unable to read high threshold\n");
		return rc;
	} else {
		ltr_pdata->pfd_ps_highthresh = temp_val;
	}

	rc = of_property_read_u32(np, "liteon,lowthr", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev,	"Unable to read low threshold\n");
		return rc;
	} else {
		ltr_pdata->pfd_ps_lowthresh = temp_val;
	}

	return 0;
}

#ifdef ZTE_LTR559_PINCTRL
static int ltr559_pinctrl_init(struct ltr559_data *ltr559)
{
	struct i2c_client *client = ltr559->i2c_client;

	ltr559->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(ltr559->pinctrl)) {
		dev_err(&client->dev,	"Failed to get pinctrl\n");
		return PTR_ERR(ltr559->pinctrl);
	}

	ltr559->pin_default =
		pinctrl_lookup_state(ltr559->pinctrl, "lpsensor_default");
	if (IS_ERR_OR_NULL(ltr559->pin_default)) {
		dev_err(&client->dev,	"Failed to look up default state\n");
		return PTR_ERR(ltr559->pin_default);
	}

	ltr559->pin_sleep =
		pinctrl_lookup_state(ltr559->pinctrl, "lpsensor_sleep");
	if (IS_ERR_OR_NULL(ltr559->pin_sleep)) {
		dev_err(&client->dev,	"Failed to look up sleep state\n");
		return PTR_ERR(ltr559->pin_sleep);
	}

	return 0;
}
#endif

static int ltr559_probe(struct i2c_client *client,
						const struct i2c_device_id *id)
{
	int ret = 0;
	struct ltr559_data *ltr559;
	struct ltr559_platform_data *platdata;

	LTR559_DBG("%s:+++\n", __func__);

	atomic_set(&ltr559_resume_poweron_flag,	0);

	ltr559 = kzalloc(sizeof(struct ltr559_data), GFP_KERNEL);
	if (!ltr559) {
		dev_err(&ltr559->i2c_client->dev,	"%s: Mem Alloc Fail...\n", __func__);
		return -ENOMEM;
	}

	platdata = kzalloc(sizeof(*platdata),	GFP_KERNEL);
	if (!platdata) {
		dev_err(&client->dev, "failed to allocate memory for platform data\n");
		ret = -ENOMEM;
		goto err_free_ltr559;
	}
	if (client->dev.of_node) {
		memset(platdata,	0 ,	sizeof(*platdata));
		ret = ltr_parse_dt(&client->dev,	platdata);
		if (ret) {
			dev_err(&client->dev,	"Unable to parse platform data err=%d\n",	ret);
			goto err_free_platdata;
		}
	}

	/* Global pointer for this device */
	sensor_info = ltr559;

	/* Set initial defaults */
	ltr559->als_enable_flag = 0;
	ltr559->ps_enable_flag = 0;

	ltr559->enable_als_sensor = 0;
	ltr559->enable_ps_sensor = 0;

	ltr559->i2c_client = client;
	ltr559->irq = client->irq;

	i2c_set_clientdata(client,	ltr559);

#ifdef ZTE_LTR559_PINCTRL
	LTR559_DBG("%s: pinctrl init\n", __func__);
	/* initialize pinctrl */
	ret = ltr559_pinctrl_init(ltr559);
	if (ret) {
		dev_err(&client->dev,	"Can't initialize pinctrl\n");
			goto err_free_platdata;
	}

	ret = pinctrl_select_state(ltr559->pinctrl,	ltr559->pin_sleep);
	if (ret) {
		dev_err(&client->dev,
			"Can't select pinctrl sleep state\n");
		goto err_free_platdata;
	}
#endif

	cal_als_winfac();

	ret = ltr_power_init(ltr559,	true);
	if (ret) {
		dev_err(&client->dev,	"power init failed");
		goto err_free_platdata;
	}

	ret = ltr_power_on(ltr559,	true);
	if (ret) {
		dev_err(&client->dev,	"power on failed");
		goto err_power_uninit;
	}

	/* Parse the platform data */
	ltr559->gpio_int_no = platdata->pfd_gpio_int_no;
	/*ltr559->adc_levels = platdata->pfd_levels;*/
	ltr559->default_ps_lowthresh = platdata->pfd_ps_lowthresh;
	ltr559->default_ps_highthresh = platdata->pfd_ps_highthresh;

	if (_check_part_id(ltr559) < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: Part ID Read Fail...\n", __func__);
		goto err_power_off;
	}

	/* Setup the input subsystem for the ALS */
	ret = als_setup(ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: ALS Setup Fail...\n", __func__);
		goto err_power_off;
	}

	/* Setup the input subsystem for the PS */
	ret = ps_setup(ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: PS Setup Fail...\n", __func__);
		goto err_power_off;
	}

	/* Create the workqueue for the als polling */
	ltr559->als_wq = create_singlethread_workqueue("ltr559_als_wq");
	if (!ltr559->als_wq) {
		dev_err(&ltr559->i2c_client->dev,
		"%s: Create als_wq Fail...\n", __func__);
		ret = -ENOMEM;
		goto err_out;
	}

	/* Wake lock option for promity sensor */
	/*wake_lock_init(&(ltr559->ps_wake_lock),	WAKE_LOCK_SUSPEND,	"proximity");*/

	/* Setup and configure both the ALS and PS on the ltr559 device */
	ret = ltr559_setup(ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
		"%s: Setup Fail...\n", __func__);
		goto err_ltr559_setup;
	}

	INIT_DELAYED_WORK(&ltr559->als_dwork,	ltr559_als_polling_work_func);

	/* Register the sysfs files */
	ret = create_sysfs_interfaces(&client->dev);
	if (ret < 0) {
		dev_err(&client->dev,
			 "sysfs register failed\n");
		goto err_ltr559_setup;
	}
	/* Register to sensors class */
	ltr559->als_cdev = sensors_light_cdev;
	ltr559->als_cdev.sensors_enable = ltr559_als_set_enable;
	ltr559->als_cdev.sensors_poll_delay = ltr559_als_poll_delay;

	ltr559->ps_cdev = sensors_proximity_cdev;
	ltr559->ps_cdev.sensors_enable = ltr559_ps_set_enable;
	ltr559->ps_cdev.sensors_poll_delay = NULL,

	ret = sensors_classdev_register(&client->dev,	&ltr559->als_cdev);
	if (ret) {
		pr_err("%s: Unable to register to sensors class: %d\n",
			 __func__,	ret);
		goto err_ltr559_sysfs_create;
	}

	ret = sensors_classdev_register(&client->dev,	&ltr559->ps_cdev);
	if (ret) {
		pr_err("%s: Unable to register to sensors class: %d\n",
					 __func__,	ret);
		goto err_ltr559_class_sysfs;
	}

	dev_dbg(&ltr559->i2c_client->dev,	"%s: probe complete\n", __func__);
	is_ltr559_probe_succ_flag = 1;
	LTR559_DBG("%s: --- ok\n", __func__);
	return ret;

err_ltr559_class_sysfs:
	sensors_classdev_unregister(&ltr559->als_cdev);

err_ltr559_sysfs_create:
	remove_sysfs_interfaces(&client->dev);

err_ltr559_setup:
	destroy_workqueue(ltr559->als_wq);
err_out:
err_power_off:
	ltr_power_on(ltr559,	false);
err_power_uninit:
	ltr_power_init(ltr559,	false);
err_free_platdata:
	kfree(platdata);
err_free_ltr559:
	kfree(ltr559);
	is_ltr559_probe_succ_flag = 0;
	LTR559_DBG("%s: --- error %d\n", __func__,	ret);
	return ret;
}


static const struct i2c_device_id ltr559_id[] = {
	{ LTR559_DEVICE_NAME, 0 },
	{}
};

#ifdef CONFIG_OF
static struct of_device_id liteon_match_table[] = {
		{ .compatible = "liteon,ltr559",},
		{ },
};
#else
#define liteon_match_table NULL
#endif

static struct i2c_driver ltr559_driver = {
	.probe = ltr559_probe,
	.id_table = ltr559_id,
	.driver = {
		.owner = THIS_MODULE,
		.name = LTR559_DEVICE_NAME,
		.of_match_table = liteon_match_table,

	},
	.suspend = ltr559_suspend,
	.resume	= ltr559_resume,
};


static int __init ltr559_init(void)
{
	return i2c_add_driver(&ltr559_driver);
}

static void __exit ltr559_exit(void)
{
	i2c_del_driver(&ltr559_driver);
}


module_init(ltr559_init)
module_exit(ltr559_exit)

MODULE_AUTHOR("Lite-On Technology Corp");
MODULE_DESCRIPTION("LTR-559ALSPS Driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION(DRIVER_VERSION);
