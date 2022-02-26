// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2019-2020, The Linux Foundation. All rights reserved.
 */

#include <linux/irq.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/hrtimer.h>

#define DEBUG_LOG 1
#define USE_DEVICE_ATTR 1
#define USE_MOTOR_FAULT 1

#define STDRV_MAGIC_NUMBER				0xEFECE5EA
#define STDRV_IOCTL_GROUP				0xDC	//220
#define STDRV_POPUP_CAM_UP				_IO(STDRV_IOCTL_GROUP, 1)
#define STDRV_POPUP_CAM_DOWN			_IO(STDRV_IOCTL_GROUP, 2)
#define STDRV_POPUP_CAM_MOVE			_IO(STDRV_IOCTL_GROUP, 3)
#define STDRV_POPUP_CAM_SET_DIR			_IO(STDRV_IOCTL_GROUP, 4)
#define STDRV_POPUP_CAM_GET_DIR			_IO(STDRV_IOCTL_GROUP, 5)
#define STDRV_POPUP_CAM_GET_PPS_LEN		_IO(STDRV_IOCTL_GROUP, 6)
#define STDRV_POPUP_CAM_SET_PERIOD		_IO(STDRV_IOCTL_GROUP, 7)
#define STDRV_POPUP_CAM_SET_DUTY_CYCLE	_IO(STDRV_IOCTL_GROUP, 8)

#define STEP_MOTOR_PPS_LEN 1472	/* motor spec. */

/* Data Struct / Enum */
enum motor_status {
	/* motor status */
	MOTOR_DISABLED = 0,
	MOTOR_ENABLED = 1,

	/* motor direction  */
	MOTOR_DIR_BACKWARD = 0,
	MOTOR_DIR_FORWARD = 1,

	/* fault dection on motor */
	MOTOR_FAULT = 10,
};

struct stspin220_chip {
	struct platform_device *pdev;
	struct pwm_device *pwm_dev;

	int gpio_motor_en;		// output
	int gpio_motor_dir;		// output
	int gpio_motor_sleep;	// output
#ifdef USE_MOTOR_FAULT
	int gpio_motor_fault;	// input as interrupt
	int irq_motor_fault;	// IRQ number
#endif

	int percent_duty;		// range 1 ~ 100
	int current_direction;	// status direction for move pps function
	u32 motor_len_in_pps;

	struct mutex lock;
	struct mutex irq_lock;
	struct hrtimer st_hrtimer;

	struct workqueue_struct *work_queue;
	struct work_struct st_work;
};

u64 array_period_ns[] = {
	/*  PWM Freq. list from PMIC
	Best Period		PMIC Output		Motor Input */
	19968,		 /*	50080.12821		12520.03205 */
	26624,		 /*	37560.09615		9390.024038 */
	39936,		 /*	25040.0641		6260.016026 */
	53248,		 /*	18780.04808		4695.012019 */
	66560,		 /*	15024.03846		3756.009615 */
	79872,		 /*	12520.03205		3130.008013 */
	106496,		 /*	9390.024038		2347.50601  */
	133120,		 /*	7512.019231		1878.004808 */
	159744,		 /*	6260.016026		1565.004006 */
	212992,		 /*	4695.012019		1173.753005 */
	266240,		 /*	3756.009615		939.0024038 */
	319488,		 /*	3130.008013		782.5020032 */	/* < ---- */
	425984,		 /*	2347.50601		586.8765024 */
	532480,		 /*	1878.004808		469.5012019 */	/* < ---- */
	638976,		 /*	1565.004006		391.2510016 */
	851968,		 /*	1173.753005		293.4382512 */
	1064960,	 /*	939.0024038		234.750601	*/
};

struct stspin220_chip *chip_fop; // for file operation
static int test_msleep = 0;

/* motor operator function */
static void power_up(struct stspin220_chip *chip);
static void power_down(struct stspin220_chip *chip);
static int move_motor(struct stspin220_chip *chip);
static int move_motor_pps(struct stspin220_chip *chip, int num_pps);
static u64 calc_duty_cycle_ns(u64 period_ns, int percent);
static int set_direction(struct stspin220_chip *chip, int direction);
static int set_period(struct stspin220_chip *chip, unsigned int index);
static int set_duty_cycle(struct stspin220_chip *chip, unsigned int percent);

/* File IO */
static int stspin220_open(struct inode *inode, struct file *file);
static int stspin220_release(struct inode *inode, struct file *file);
static ssize_t stspin220_read(struct file *file, char *buf, size_t count, loff_t *ppos);
static ssize_t stspin220_write(struct file *file, const char *buf, size_t count, loff_t *ppos);
#if HAVE_UNLOCKED_IOCTL
static long stspin220_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
#else
static int stspin220_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);
#endif
static struct file_operations fops =
{
	.owner =            THIS_MODULE,
	.open =             stspin220_open,
	.release =          stspin220_release,
	.read =             stspin220_read,
	.write =            stspin220_write,
#if HAVE_UNLOCKED_IOCTL
	.unlocked_ioctl =   stspin220_unlocked_ioctl,
#if HAVE_COMPAT_IOCTL
	.compat_ioctl   =   stspin220_unlocked_ioctl,
#endif
#else
	.ioctl =            stspin220_ioctl,
#endif
	.llseek =           default_llseek    /* using default implementation as declared in linux/fs.h */
};

static struct miscdevice miscdev =
{
	.minor =	MISC_DYNAMIC_MINOR,
	.name =		"stdrv",
	.fops =		&fops,
};

static int stspin220_open(struct inode *inode, struct file *file)
{
	if (!try_module_get(THIS_MODULE)) return -ENODEV;

#ifdef DEBUG_LOG
	pr_info("%s, /dev/stdrv is opened.\n", __func__);
#endif
	return 0;
}

static int stspin220_release(struct inode *inode, struct file *file)
{
	module_put(THIS_MODULE);

#ifdef DEBUG_LOG
	pr_info("%s, /dev/stdrv is released.\n", __func__);
#endif
	return 0;
}

static ssize_t stspin220_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
#ifdef DEBUG_LOG
	pr_info("%s, /dev/stdrv is not implemented for read().\n", __func__);
#endif

	return 0;
}

static ssize_t stspin220_write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
#ifdef DEBUG_LOG
	pr_info("%s, /dev/stdrv is not implemented for write().\n", __func__);
#endif
	return 0;
}

#if HAVE_UNLOCKED_IOCTL
static long stspin220_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
#else
static int stspin220_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
#endif
{
	int rc;
#ifdef DEBUG_LOG
	pr_info("%s, begin ioctl cmd = %d\n", __func__, cmd);
#endif

	switch (cmd) {
		case STDRV_POPUP_CAM_UP:
			// power up
			power_up(chip_fop);

			// set a direction
			rc = set_direction(chip_fop, MOTOR_DIR_FORWARD);

			// enable motor and disable within 1394 pps
			rc = move_motor(chip_fop);
			break;

		case STDRV_POPUP_CAM_DOWN:
			// power up
			power_up(chip_fop);

			// set direction
			rc = set_direction(chip_fop, MOTOR_DIR_BACKWARD);

			// enable motor and disable within 1394 pps
			rc = move_motor(chip_fop);
			break;

		case STDRV_POPUP_CAM_MOVE:
			// power up
			power_up(chip_fop);

			// set direction
			rc = set_direction(chip_fop, chip_fop->current_direction);

			// arg is a number of pps.
			// enable and disble a motor within pps requested from userspace.
			rc = move_motor_pps(chip_fop, arg);
			break;

		case STDRV_POPUP_CAM_SET_DIR:
			// set direction
			rc = set_direction(chip_fop, arg);
			break;

		case STDRV_POPUP_CAM_GET_PPS_LEN:
			return chip_fop->motor_len_in_pps;

		case STDRV_POPUP_CAM_SET_PERIOD:
			// arg is a index of array_period_ns.
			rc = set_period(chip_fop, arg);
			break;

		case STDRV_POPUP_CAM_SET_DUTY_CYCLE:
			// arg is a percent of duty cycle.
			rc = set_duty_cycle(chip_fop, arg);
			break;
	}

#ifdef DEBUG_LOG
	pr_info("%s, end ioctl rc = %d\n", __func__, rc);
#endif

	return rc;
}


#ifdef USE_DEVICE_ATTR
static ssize_t motor_en_show(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	struct stspin220_chip *chip = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "MOTOR_EN status :%d\n", gpio_get_value(chip->gpio_motor_en));
}

static ssize_t motor_en_store(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct stspin220_chip *chip = dev_get_drvdata(dev);
	bool gpio_value;
	int rc;

	rc = kstrtobool(buf, &gpio_value);
	if (rc < 0)
		return rc;

	gpio_direction_output(chip->gpio_motor_en, gpio_value);
	usleep_range(7, 10);

	pr_info("%s : MOTOR_EN set with %d\n", __func__, gpio_value);

	return (rc < 0) ? rc : count;
}

static ssize_t motor_dir_show(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	struct stspin220_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "MOTOR_DIR status :%d\n", gpio_get_value(chip->gpio_motor_dir));
}

static ssize_t motor_dir_store(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct stspin220_chip *chip = dev_get_drvdata(dev);
	bool gpio_value;
	int rc;

	rc = kstrtobool(buf, &gpio_value);
	if (rc < 0)
		return rc;

	rc = set_direction(chip, gpio_value);

	pr_info("%s : MOTOR_DIR set with %d\n", __func__, gpio_value);

	return (rc < 0) ? rc : count;
}

static ssize_t motor_sleep_show(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	struct stspin220_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "MOTOR_SLEEP status :%d\n", gpio_get_value(chip->gpio_motor_sleep));
}

static ssize_t motor_sleep_store(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct stspin220_chip *chip = dev_get_drvdata(dev);
	bool gpio_value;
	int rc;

	rc = kstrtobool(buf, &gpio_value);
	if (rc < 0)
		return rc;

	gpio_direction_output(chip->gpio_motor_sleep, gpio_value);
	usleep_range(7, 10);

	pr_info("%s : MOTOR_SLEEP set with %d\n", __func__, gpio_value);

	return (rc < 0) ? rc : count;
}

static ssize_t period_show(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	struct stspin220_chip *chip = dev_get_drvdata(dev);
	struct pwm_state state;

	pwm_get_state(chip->pwm_dev, &state);

	return sprintf(buf, "%llu\n", state.period);
}

static ssize_t period_store(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct stspin220_chip *chip = dev_get_drvdata(dev);
	struct pwm_device *pwm = chip->pwm_dev;
	struct pwm_state state;
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;

#if 0	// for test
	ret = set_period(chip, val);
#else
	mutex_lock(&chip->lock);
	pwm_get_state(pwm, &state);
	state.period = val;
	state.duty_cycle = calc_duty_cycle_ns(state.period, chip->percent_duty);
	ret = pwm_apply_state(pwm, &state);
	mutex_unlock(&chip->lock);
#endif

	pr_info("%s : period %llu duty_cycle %llu, duty %d %%\n", __func__, state.period, state.duty_cycle, chip->percent_duty);
	return ret ? : count;
}

static ssize_t duty_cycle_show(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	struct stspin220_chip *chip = dev_get_drvdata(dev);
	struct pwm_device *pwm = chip->pwm_dev;
	struct pwm_state state;

	pwm_get_state(pwm, &state);

	return sprintf(buf, "%llu ns, %d %c\n", state.duty_cycle, chip->percent_duty, '%');
}

static ssize_t duty_cycle_store(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct stspin220_chip *chip = dev_get_drvdata(dev);
	struct pwm_device *pwm = chip->pwm_dev;
	struct pwm_state state;
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;

#if 0	// for test
	ret = set_duty_cycle(chip, val);
#else
	mutex_lock(&chip->lock);
	pwm_get_state(pwm, &state);
	state.duty_cycle = val;
	ret = pwm_apply_state(pwm, &state);
	chip->percent_duty = state.duty_cycle*100/state.period;
	mutex_unlock(&chip->lock);
#endif

	pr_info("%s : period %llu duty_cycle %llu, duty %d %%\n", __func__, state.period, state.duty_cycle, chip->percent_duty);
	return ret ? : count;
}

static ssize_t able_pwm_show(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	struct stspin220_chip *chip = dev_get_drvdata(dev);
	struct pwm_device *pwm = chip->pwm_dev;
	struct pwm_state state;

	pwm_get_state(pwm, &state);

	return sprintf(buf, "%d\n", state.enabled);
}

static ssize_t able_pwm_store(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct stspin220_chip *chip = dev_get_drvdata(dev);
	struct pwm_device *pwm = chip->pwm_dev;
	struct pwm_state state;
	int val, ret;

	ret = kstrtoint(buf, 0, &val);
	if (ret)
		return ret;

	mutex_lock(&chip->lock);
	pwm_get_state(pwm, &state);

	switch (val) {
		case 0:
			state.enabled = false;
			break;
		case 1:
			state.enabled = true;
			break;
		default:
			ret = -EINVAL;
			goto unlock;
	}

	ret = pwm_apply_state(pwm, &state);

unlock:
	mutex_unlock(&chip->lock);
	return ret ? : count;
}

static ssize_t polarity_show(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	struct stspin220_chip *chip = dev_get_drvdata(dev);
	struct pwm_device *pwm = chip->pwm_dev;
	const char *polarity = "unknown";
	struct pwm_state state;

	pwm_get_state(pwm, &state);

	switch (state.polarity) {
	case PWM_POLARITY_NORMAL:
		polarity = "normal";
		break;

	case PWM_POLARITY_INVERSED:
		polarity = "inversed";
		break;
	}

	return sprintf(buf, "%s\n", polarity);
}

static ssize_t polarity_store(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct stspin220_chip *chip = dev_get_drvdata(dev);
	struct pwm_device *pwm = chip->pwm_dev;
	enum pwm_polarity polarity;
	struct pwm_state state;
	int ret;

	if (sysfs_streq(buf, "normal"))
		polarity = PWM_POLARITY_NORMAL;
	else if (sysfs_streq(buf, "inversed"))
		polarity = PWM_POLARITY_INVERSED;
	else
		return -EINVAL;

	mutex_lock(&chip->lock);
	pwm_get_state(pwm, &state);
	state.polarity = polarity;
	ret = pwm_apply_state(pwm, &state);
	mutex_unlock(&chip->lock);

	return ret ? : count;
}

static ssize_t capture_show(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	struct stspin220_chip *chip = dev_get_drvdata(dev);
	struct pwm_device *pwm = chip->pwm_dev;
	struct pwm_capture result;
	int ret;

	ret = pwm_capture(pwm, &result, jiffies_to_msecs(HZ));
	if (ret)
		return ret;

	return sprintf(buf, "period_ns %llu, duty_cycle_ns %llu\n", result.period, result.duty_cycle);
}

static ssize_t output_type_show(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	struct stspin220_chip *chip = dev_get_drvdata(dev);
	struct pwm_device *pwm = chip->pwm_dev;
	const char *output_type = "unknown";
	struct pwm_state state;

	pwm_get_state(pwm, &state);
	switch (state.output_type) {
	case PWM_OUTPUT_FIXED:
		output_type = "fixed";
		break;
	case PWM_OUTPUT_MODULATED:
		output_type = "modulated";
		break;
	default:
		break;
	}

	return snprintf(buf, PAGE_SIZE, "%s\n", output_type);
}

static ssize_t output_type_store(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct stspin220_chip *chip = dev_get_drvdata(dev);
	struct pwm_device *pwm = chip->pwm_dev;
	struct pwm_state state;
	int ret = -EINVAL;

	mutex_lock(&chip->lock);
	pwm_get_state(pwm, &state);
	if (sysfs_streq(buf, "fixed"))
		state.output_type = PWM_OUTPUT_FIXED;
	else if (sysfs_streq(buf, "modulated"))
		state.output_type = PWM_OUTPUT_MODULATED;
	else
		goto unlock;

	ret = pwm_apply_state(pwm, &state);

unlock:
	mutex_unlock(&chip->lock);

	return ret ? : count;
}

static ssize_t test_ns_show(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	struct stspin220_chip *chip = dev_get_drvdata(dev);
	struct pwm_device *pwm = chip->pwm_dev;
	struct pwm_state state;

	pwm_get_state(pwm, &state);

	return sprintf(buf, "en/dis %d, period_ns %d, duty_cycle_ns %d, duty cycle %d %%.\n", state.enabled, state.period, state.duty_cycle, chip->percent_duty);
}

static ssize_t test_ns_store(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct stspin220_chip *chip = dev_get_drvdata(dev);
	struct pwm_device *pwm = chip->pwm_dev;
	struct pwm_state state;
	int val, ret;

	ret = kstrtoint(buf, 0, &val);
	if (ret)
		return ret;

	if( val == -1 ) {	// power up
		power_up(chip);
		return count;
	} else if(val == -2) {	// power down
		power_down(chip);
		return count;
	} else if(val == -3) {	// popup_cam=on
		// power up
		power_up(chip);

		// set a direction
		set_direction(chip, MOTOR_DIR_FORWARD);

		if( test_msleep > 0 )
			msleep(test_msleep);

		// enable motor and disable within 1394 pps
		move_motor(chip);
		return count;
	} else if(val == -4) {	// popup_cam=off
		// power up
		power_up(chip);

		// set a direction
		set_direction(chip, MOTOR_DIR_BACKWARD);
			if( test_msleep > 0 )
				msleep(test_msleep);

		// enable motor and disable within 1394 pps
		move_motor(chip);
		return count;
	} else if(val == -5) {	// set dir_forward
		// set a direction
		set_direction(chip, MOTOR_DIR_FORWARD);
		return count;
	} else if(val == -6) {	// set dir_backward
		// set a direction
		set_direction(chip, MOTOR_DIR_BACKWARD);
		return count;
	} else if(val < -4) {	// set delay time afer setting gpio
		test_msleep = val*-1;
		return count;
	} else {	// set move pps
		// move remained pps
		// power up
		power_up(chip);

		// set direction
		set_direction(chip, chip->current_direction);
		pr_info("%s : cur. dir. %d \n", __func__, chip->current_direction);

		// arg is a number of pps.
		// enable and disble a motor within pps requested from userspace.
		move_motor_pps(chip, val);
		return count;
	}

	mutex_lock(&chip->lock);
	pwm_get_state(pwm, &state);
	state.enabled = true;
	ret = pwm_apply_state(pwm, &state);
	mutex_unlock(&chip->lock);
	//pr_info("%s : enable pwm\n", __func__);

	if( val > 0 ) {
		usleep_range(val, val+10);
		//msleep(100);
		state.enabled = false;
		ret = pwm_apply_state(pwm, &state);
		//pr_info("%s : disable pwm\n", __func__);
	}

	return ret ? : count;
}

static DEVICE_ATTR_RW(motor_en);
static DEVICE_ATTR_RW(motor_dir);
static DEVICE_ATTR_RW(motor_sleep);
static DEVICE_ATTR_RW(period);
static DEVICE_ATTR_RW(duty_cycle);
static DEVICE_ATTR_RW(able_pwm);
static DEVICE_ATTR_RW(polarity);
static DEVICE_ATTR_RO(capture);
static DEVICE_ATTR_RW(output_type);
static DEVICE_ATTR_RW(test_ns);

static const struct attribute *motor_attrs[] = {
	&dev_attr_motor_en.attr,
	&dev_attr_motor_dir.attr,
	&dev_attr_motor_sleep.attr,
	&dev_attr_period.attr,
	&dev_attr_duty_cycle.attr,
	&dev_attr_able_pwm.attr,
	&dev_attr_polarity.attr,
	&dev_attr_capture.attr,
	&dev_attr_output_type.attr,
	&dev_attr_test_ns.attr,
	NULL
};
#endif /* USE_DEVICE_ATTR */

static void power_up(struct stspin220_chip *chip)
{
	struct pwm_device *pwm = chip->pwm_dev;
	struct pwm_state state;

	/*
		use only a full-step mode.

		The recommended power-up sequence is following:
		1. Power-up the device applying the VS supply voltage but keeping both STBY and EN/FAULT inputs low.
		2. Set the MODEx inputs according to the target step resolution (see Table 1).
		3. Wait for at least 1 レs (minimum tMODEsu time).
		4. Set the STBY high. The MODEx configuration is now latched inside the device.
		5. Wait for at least 100 レs (minimum tMODEho time).
		6. Enable the power stage releasing the EN/FAULT input and start the operation.
	*/

	// 1. set MOTOR_EN and MOTRO_SLEEP LOW
	//gpio_direction_output(chip->gpio_motor_en, 0);
	//gpio_direction_output(chip->gpio_motor_sleep, 0);

	// 2. Set the MODEx inputs as Full step mode
	gpio_direction_output(chip->gpio_motor_dir, 0);
	usleep_range(7, 10);

	pwm_get_state(pwm, &state);
	state.enabled = false;
	pwm_apply_state(pwm, &state);

	// 3. Wait for at least 1 レs (minimum tMODEsu time).
	usleep_range(5, 10);

	// 4. Set the STBY high. The MODEx configuration is now latched inside the device.
	gpio_direction_output(chip->gpio_motor_sleep, 1);

	//5. Wait for at least 100 レs (minimum tMODEho time).
	usleep_range(100, 110);

	// 6. Enable the power stage releasing the EN/FAULT input and start the operation.
	gpio_direction_output(chip->gpio_motor_en, 1);
	usleep_range(7, 10);

#ifdef DEBUG_LOG
	pr_info("%s : power up a motor \n", __func__);
#endif

#ifdef USE_MOTOR_FAULT
	// enable IRQ
	if (chip->irq_motor_fault > 0) {
		mutex_lock(&chip->irq_lock);
		enable_irq(chip->irq_motor_fault);
		mutex_unlock(&chip->irq_lock);
		pr_info("%s : enable IRQ of motor fault\n", __func__);
	}
#endif

	if( test_msleep > 0 )
		msleep(test_msleep);
}


static void power_down(struct stspin220_chip *chip)
{
	struct pwm_state state;
	struct pwm_device *pwm;// = chip->pwm_dev;

	// 0. disable pwm
	pwm = chip->pwm_dev;
	pwm_get_state(pwm, &state);

	mutex_lock(&chip->lock);
	state.enabled = false;
	pwm_apply_state(pwm, &state);
	pr_info("%s : disable PWM.\n", __func__);
	mutex_unlock(&chip->lock);

#ifdef USE_MOTOR_FAULT
	// 1. disalbe IRQ
	mutex_lock(&chip->irq_lock);
	if (!irqd_irq_disabled(
		irq_get_irq_data(chip->irq_motor_fault)))
			disable_irq_nosync(chip->irq_motor_fault);
	mutex_unlock(&chip->irq_lock);
	pr_info("%s : disable IRQ of motor fault\n", __func__);
#endif

	// 2. reset MOTOR_EN and MOTRO_SLEEP LOW
	gpio_direction_output(chip->gpio_motor_en, 0);
	gpio_direction_output(chip->gpio_motor_sleep, 0);

#ifdef DEBUG_LOG
	pr_info("%s : power down a motor.\n", __func__);
#endif
}

/* move a motor with max pps */
static int move_motor(struct stspin220_chip *chip)
{
	int rc;
	struct pwm_state state;
	struct pwm_device *pwm = chip->pwm_dev;
	ktime_t interval;
	u64 timer_interval_ns, period_ns;

	mutex_lock(&chip->lock);
	pwm_get_state(pwm, &state);
	period_ns = state.period;
	state.enabled = true;
	// enable PWM
	rc = pwm_apply_state(pwm, &state);
	pr_info("%s : enable PWM.\n", __func__);
	mutex_unlock(&chip->lock);

	// will disble PWM within 1394 pps
	timer_interval_ns = period_ns * chip->motor_len_in_pps;
	interval = ktime_set(0, timer_interval_ns);
	hrtimer_start(&chip_fop->st_hrtimer, interval, HRTIMER_MODE_REL );

#ifdef DEBUG_LOG
	pr_info("%s : timer set.\n", __func__);
#endif

	return rc;
}

/* move a motor with pps which is requested by userspace */
static int move_motor_pps(struct stspin220_chip *chip, int num_pps)
{
	int rc;
	struct pwm_state state;
	struct pwm_device *pwm = chip->pwm_dev;
	ktime_t interval;
	u64 timer_interval_ns, period_ns;

	mutex_lock(&chip->lock);
	pwm_get_state(pwm, &state);
	period_ns = state.period;
	state.enabled = true;
	// enable PWM
	rc = pwm_apply_state(pwm, &state);
	pr_info("%s : enable PWM.\n", __func__);
	mutex_unlock(&chip->lock);

	// will disble PWM within a number of pps
	timer_interval_ns = period_ns * num_pps;
	interval = ktime_set(0, timer_interval_ns);
	hrtimer_start(&chip_fop->st_hrtimer, interval, HRTIMER_MODE_REL);

#ifdef DEBUG_LOG
	pr_info("%s : timer set.\n", __func__);
#endif

	return rc;
}

static u64 calc_duty_cycle_ns(u64 period_ns, int percent)
{
	return period_ns*percent/100;
}

static int set_direction(struct stspin220_chip *chip, int direction)
{
	switch(direction) {
		case MOTOR_DIR_BACKWARD:
			gpio_direction_output(chip_fop->gpio_motor_dir, 0);
			usleep_range(7, 10);
			break;

		case MOTOR_DIR_FORWARD:
			gpio_direction_output(chip_fop->gpio_motor_dir, 1);
			usleep_range(7, 10);
			break;

		default:
			return 1;
	}

	chip->current_direction = direction;
	pr_info("%s : direction %d\n", __func__, direction);

	return 0;
}

static int set_period(struct stspin220_chip *chip, unsigned int index)
{
	int rc;
	struct pwm_state state;
	struct pwm_device *pwm = chip->pwm_dev;

	mutex_lock(&chip->lock);
	pwm_get_state(pwm, &state);
	state.period = array_period_ns[index];
	//state.period = period_ns;
	rc = pwm_apply_state(pwm, &state);
	mutex_unlock(&chip->lock);

	return rc;
}

static int set_duty_cycle(struct stspin220_chip *chip, unsigned int percent)
{
	int rc;
	struct pwm_state state;
	struct pwm_device *pwm = chip->pwm_dev;

	mutex_lock(&chip->lock);
	pwm_get_state(pwm, &state);
	state.duty_cycle = calc_duty_cycle_ns(state.period, percent);
	rc = pwm_apply_state(pwm, &state);
	chip->percent_duty = percent;
	mutex_unlock(&chip->lock);

	return rc;
}

static void disable_motor_work(struct work_struct *work)
{
	struct stspin220_chip *chip;

	chip = container_of(work, struct stspin220_chip,
			      st_work);

	// power down
	power_down(chip);
}

enum hrtimer_restart st_hrtimer_callback(struct hrtimer *timer_for_restart)
{
	// invoke a st_work for disabling a motor
	queue_work(chip_fop->work_queue, &chip_fop->st_work);

	// add wakelock not to enter power collapse mode

	return  HRTIMER_NORESTART;
}

#ifdef USE_MOTOR_FAULT
static irqreturn_t motor_fault(int irq, void *dev)
{
	int ret = IRQ_HANDLED;

	pr_info("%s : happened motor fault\n", __func__);

	return ret;
}
#endif

static int stspin220_motor_parse_dt(struct stspin220_chip *chip)
{
	struct device_node *node = chip->pdev->dev.of_node;
	int rc;

#ifdef DEBUG_LOG
	pr_info("%s : begin parse dt\n", __func__);
#endif

	rc = of_property_read_u32(node, "lge,motor-len-in-pps", &chip->motor_len_in_pps);

	if (rc)
		chip->motor_len_in_pps = STEP_MOTOR_PPS_LEN;

	pr_info("%s : motor len %lu pps\n", __func__, chip->motor_len_in_pps);

	// MOTOR_EN
	chip->gpio_motor_en = of_get_named_gpio(node, "lge,motor-enable", 0);
	if (chip->gpio_motor_en < 0) {
		pr_err("%s : Can't find enable motor voltage gpio\n", __func__);

		return chip->gpio_motor_en;
	}

	// MOTOR_DIR
	chip->gpio_motor_dir = of_get_named_gpio(node, "lge,motor-direction", 0);
	if (chip->gpio_motor_dir < 0) {
		pr_err("%s : Can't find motor direction gpio\n", __func__);

		return chip->gpio_motor_dir;
	}

	// MOTOR_SLEEP
	chip->gpio_motor_sleep = of_get_named_gpio(node, "lge,motor-sleep", 0);
	if (chip->gpio_motor_sleep < 0) {
		pr_err("%s : Can't find motor cycle gpio\n", __func__);

		return chip->gpio_motor_sleep;
	}

#ifdef USE_MOTOR_FAULT
	// MOTOR_FAULT
	chip->gpio_motor_fault = of_get_named_gpio(node, "lge,motor-fault", 0);
	if (chip->gpio_motor_fault < 0) {
		pr_err("%s : Can't find enable motor voltage gpio\n", __func__);

		return chip->gpio_motor_fault;
	}

	if (gpio_is_valid(chip->gpio_motor_fault)) {
		chip->irq_motor_fault =  gpio_to_irq(chip->gpio_motor_fault);
		if (chip->irq_motor_fault < 0) {
			pr_err("Unable to configure irq\n");
			return chip->irq_motor_fault;
		}
	}
#endif

#ifdef DEBUG_LOG
	pr_info("%s : leave parse dt\n", __func__);
#endif

	return 0;
}

static int stspin220_motor_probe(struct platform_device *pdev)
{
	struct stspin220_chip *chip = NULL;
	struct pwm_state pstate;
	int rc = 0;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->pdev = pdev;
	rc = stspin220_motor_parse_dt(chip);
	if (rc < 0) {
		dev_err(&pdev->dev, "%s :Error in parsing DT parameters, rc=%d\n", __func__, rc);
		goto fail;
	}

	chip->percent_duty = 50;				// 50%, by defualt

	chip->pwm_dev = devm_of_pwm_get(&pdev->dev, pdev->dev.of_node, NULL);

	pwm_get_state(chip->pwm_dev, &pstate);

	pstate.enabled = false;
	pstate.period = array_period_ns[13];	// by default
	pstate.duty_cycle = calc_duty_cycle_ns(pstate.period, chip->percent_duty);
	pstate.output_type = PWM_OUTPUT_FIXED;
	pstate.output_pattern = NULL;

	rc = pwm_apply_state(chip->pwm_dev, &pstate);
	if (rc) {
		dev_err(&pdev->dev, "%s :Failed to apply initial PWM pstate: %d\n", __func__, rc);
		goto fail;
	}

	mutex_init(&chip->lock);

#ifdef USE_MOTOR_FAULT
	mutex_init(&chip->irq_lock);
#endif

	gpio_direction_output(chip->gpio_motor_en, 0);
	gpio_direction_output(chip->gpio_motor_dir, 0);
	gpio_direction_output(chip->gpio_motor_sleep, 0);

#ifdef USE_MOTOR_FAULT
	// need to register IRQ on a GPIO, MOTOR_FAULT, druing motor working.
	rc = request_threaded_irq(chip->irq_motor_fault, NULL,
					   motor_fault,
					   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					   "step_motor_irq", chip);

	mutex_lock(&chip->irq_lock);
	if (!irqd_irq_disabled(
		irq_get_irq_data(chip->irq_motor_fault)))
			disable_irq_nosync(chip->irq_motor_fault);
	mutex_unlock(&chip->irq_lock);
#endif

	// init. a hrtimer
	hrtimer_init( &chip->st_hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
	chip->st_hrtimer.function = st_hrtimer_callback;

	// make a work_queue
	chip->work_queue = create_singlethread_workqueue("step_motor_wq");
	if (!chip->work_queue) {
		dev_err(&pdev->dev, "%s: Error creating step_motor_wq\n",
			__func__);
		return -EINVAL;
	}

	// make a work
	INIT_WORK(&chip->st_work, disable_motor_work);

	chip_fop = chip;
	dev_set_drvdata(&pdev->dev, chip);

	rc = misc_register(&miscdev);
	if (rc) {
		dev_err(&pdev->dev, "%s :Failed to register miscdev: %d\n", __func__, rc);
		goto fail;
	}

#ifdef USE_DEVICE_ATTR
	rc = sysfs_create_files(&pdev->dev.kobj, motor_attrs);
	if (rc) {
		dev_err(&pdev->dev, "%s :Failed to create sysfs files: %d\n", __func__, rc);
		goto fail;
	}
#endif

#ifdef DEBUG_LOG
	pr_info("%s : STDRV_POPUP_CAM_UP 0x%x ~ STDRV_POPUP_CAM_SET_DUTY_CYCLE 0x%x \n", __func__,
		STDRV_POPUP_CAM_UP,
		STDRV_POPUP_CAM_SET_DUTY_CYCLE);
#endif

	pr_info("%s : done.\n", __func__);

	return 0;
fail:
	dev_set_drvdata(&pdev->dev, NULL);
	return rc;
}

static int stspin220_motor_remove(struct platform_device *pdev)
{
	struct stspin220_chip *chip = dev_get_drvdata(&pdev->dev);
	int ret;

	power_down(chip);

  	ret = hrtimer_cancel( &chip->st_hrtimer);
  	if( ret )
		pr_err("The timer was still in use...\n");

	mutex_destroy(&chip->lock);

#ifdef USE_MOTOR_FAULT
	if (chip->irq_motor_fault > 0)
		free_irq(chip->irq_motor_fault, chip);

	mutex_destroy(&chip->irq_lock);
#endif

#ifdef USE_DEVICE_ATTR
	sysfs_remove_files(&pdev->dev.kobj, motor_attrs);
#endif

	misc_deregister(&miscdev);

	flush_workqueue(chip->work_queue);
	destroy_workqueue(chip->work_queue);

	pr_info("%s : done.\n", __func__);

	return 0;
}


static const struct of_device_id stspin220_motor_of_match[] = {
	{.compatible = "lge,motor-stspin220", },
	{},
};
MODULE_DEVICE_TABLE(of, stspin220_motor_of_match);

struct platform_driver step_motor_driver = {
	.driver             = {
		.name           = "lge,motor-stspin220",
		.owner          = THIS_MODULE,
		.of_match_table = stspin220_motor_of_match,
	},
	.probe              = stspin220_motor_probe,
	.remove              = stspin220_motor_remove,
};
module_platform_driver(step_motor_driver);

MODULE_DESCRIPTION("LGE Step Motor driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("motors:lge-motor-stspin220");
