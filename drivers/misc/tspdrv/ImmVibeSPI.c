/*
 ** =========================================================================
 ** Copyright (c) 2007-2013  Immersion Corporation.  All rights reserved.
 **                          Immersion Corporation Confidential and Proprietary
 **
 ** File:
 **     ImmVibeSPI.c
 **
 ** Description:
 **     Device-dependent functions called by Immersion TSP API
 **     to control PWM duty cycle, amp enable/disable, save IVT file, etc...
 **
 **     This file is provided for Generic Project
 **
 ** =========================================================================
 */

#include <linux/string.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/time.h>       /* for NSEC_PER_SEC */
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include "../../staging/android/timed_output.h"
#include <linux/regulator/consumer.h>
#include <soc/qcom/clock-local2.h>
#include <linux/io.h>
#include <linux/of_gpio.h>

#define NUM_ACTUATORS		1
#define ISA1000_BOARD_NAME	"ISA1000"
#define PWM_FREQUENCY		30000
#define DEFAULT_PWM_DUTY	127
#define ISA1000_VIB_DEFAULT_TIMEOUT	15000

struct isa1000_vib {
	struct hrtimer vib_timer;
	struct timed_output_dev timed_dev;
	struct work_struct work;

	int timeout;
	int curr_state;
	int next_state;
	bool clk_enabled;
	struct mutex lock;
	struct clk* vibrator_clk;

	u32 gpio_en;
	u32 gpio_pwm_en;
};

static struct isa1000_vib *vib_dev = NULL;

static int isa1000_vib_set_duty(int pwm_duty)
{
	struct rcg_clk *rcg;
	struct clk_freq_tbl *nf;

	if (IS_ERR_OR_NULL(vib_dev->vibrator_clk ) || !vib_dev->vibrator_clk ->parent) {
		pr_err("vibrator_clk  error!\n");
		return -ENODEV;
	}

	rcg = to_rcg_clk(vib_dev->vibrator_clk ->parent);
	nf = rcg->current_freq;

	nf->d_val = (~(pwm_duty * 2 * 80 / 100)) & 0xff;
	set_rate_mnd(rcg, nf);

	return 0;
}

int isa1000_vib_set_clock(struct platform_device *pdev)
{
	int ret;

	vib_dev->vibrator_clk  = devm_clk_get(&pdev->dev, "vibrator_pwm");
	if (IS_ERR_OR_NULL(vib_dev->vibrator_clk ))
		return -1;

	ret = clk_set_rate(vib_dev->vibrator_clk , PWM_FREQUENCY);
	if (ret)
		return ret;

	/* Set default PWM duty */
	isa1000_vib_set_duty(DEFAULT_PWM_DUTY);

	return 0;
}

int isa1000_vib_enable_clock()
{
	int ret;

	if (!vib_dev)
		return -ENODEV;

	mutex_lock(&vib_dev->lock);
	if (likely(!vib_dev->clk_enabled)) {
		ret = clk_prepare_enable(vib_dev->vibrator_clk );
		vib_dev->clk_enabled = true;
	}
	mutex_unlock(&vib_dev->lock);

	return ret;
}

void isa1000_vib_disable_clock()
{
	if (!vib_dev)
		return;

	mutex_lock(&vib_dev->lock);
	if (likely(vib_dev->clk_enabled)) {
		clk_disable_unprepare(vib_dev->vibrator_clk );
		vib_dev->clk_enabled = false;
	}
	mutex_unlock(&vib_dev->lock);
}

static int isa1000_vib_set(struct isa1000_vib *vib, int on)
{
	int rc = 0;

	if (vib->curr_state == on)
		return rc;

	if (on) {
		isa1000_vib_enable_clock();

		gpio_set_value_cansleep(vib_dev->gpio_pwm_en, true);
		udelay(10);
		gpio_set_value_cansleep(vib_dev->gpio_en, true);
	} else {
		isa1000_vib_disable_clock();

		gpio_set_value_cansleep(vib_dev->gpio_en, false);
		gpio_set_value_cansleep(vib_dev->gpio_pwm_en, false);
	}

	vib->curr_state = on;

	return rc;
}

static void isa1000_vib_enable(struct timed_output_dev *dev, int value)
{
	struct isa1000_vib *vib = container_of(dev, struct isa1000_vib,
			timed_dev);

	mutex_lock(&vib->lock);
	hrtimer_cancel(&vib->vib_timer);

	if (value == 0)
		vib->next_state = 0;
	else {
		value = (value > vib->timeout ? vib->timeout : value);
		vib->next_state = 1;
		hrtimer_start(&vib->vib_timer,
				ktime_set(value / 1000, (value % 1000) * 1000000),
				HRTIMER_MODE_REL);
	}

	mutex_unlock(&vib->lock);
	schedule_work(&vib->work);
}

static void isa1000_vib_update(struct work_struct *work)
{
	struct isa1000_vib *vib = container_of(work, struct isa1000_vib,
			work);

	isa1000_vib_set(vib, vib->next_state);
}

static int isa1000_vib_get_time(struct timed_output_dev *dev)
{
	struct isa1000_vib *vib = container_of(dev, struct isa1000_vib,
			timed_dev);

	if (hrtimer_active(&vib->vib_timer)) {
		ktime_t r = hrtimer_get_remaining(&vib->vib_timer);
		return (int)ktime_to_us(r);
	} else
		return 0;
}

static enum hrtimer_restart isa1000_vib_timer_func(struct hrtimer *timer)
{
	struct isa1000_vib *vib = container_of(timer, struct isa1000_vib,
			vib_timer);

	vib->next_state = 0;
	schedule_work(&vib->work);

	return HRTIMER_NORESTART;
}

/* This function assumes an input of [-127, 127] and mapped to [1,255] */
/* If your PWM module does not take an input of [1,255] and mapping to [1%,99%]
 **    Please modify accordingly
 **/
static void isa1000_vib_set_level(int level)
{
	int rc = 0;
	static int last_level = 0;

	if (level == last_level)
		return;

	if  (level != 0) {
		/* Set PWM duty cycle corresponding to the input 'level' */
		/* mapping from [-127, 127] to [1%, 99%] */
		rc = isa1000_vib_set_duty((level + 127) * 49 / 127 + 1);
		if (rc < 0) {
			pr_err("%s: set duty failed\n", __func__);
			goto chip_dwn;
		}

		isa1000_vib_enable_clock();

		/* Enable ISA1000 */
		gpio_set_value_cansleep(vib_dev->gpio_pwm_en, 1);
		udelay(10);
		gpio_set_value_cansleep(vib_dev->gpio_en, 1);
	} else {
		/* Disable ISA1000 */
		gpio_set_value_cansleep(vib_dev->gpio_en, 0);
		gpio_set_value_cansleep(vib_dev->gpio_pwm_en, 0);

		/* Disable the PWM output */
		isa1000_vib_disable_clock();
	}

	last_level = level;

	return;

chip_dwn:
	gpio_set_value_cansleep(vib_dev->gpio_en, 0);
	gpio_set_value_cansleep(vib_dev->gpio_pwm_en, 0);
}

static int isa1000_setup(void)
{
	int ret;

	ret = gpio_is_valid(vib_dev->gpio_en);
	if (ret) {
		ret = gpio_request(vib_dev->gpio_en, "isa1000_gpio_en");
		if (ret) {
			pr_err("%s: gpio %d request failed\n",
					__func__, vib_dev->gpio_en);
			return ret;
		}
	} else {
		pr_err("%s: Invalid gpio %d\n", __func__, vib_dev->gpio_en);
		return ret;
	}

	ret = gpio_is_valid(vib_dev->gpio_pwm_en);
	if (ret) {
		ret = gpio_request(vib_dev->gpio_pwm_en, "isa1000_gpio_pwr_en");
		if (ret) {
			pr_err("%s: gpio %d request failed\n",
					__func__, vib_dev->gpio_pwm_en);
			return ret;
		}
	} else {
		pr_err("%s: Invalid gpio %d\n", __func__, vib_dev->gpio_pwm_en);
		return ret;
	}

	gpio_direction_output(vib_dev->gpio_en, 0);
	gpio_direction_output(vib_dev->gpio_pwm_en, 0);

	pr_info("%s: %s set up\n", __func__, "isa1000");
	return 0;
}

/************ ISA1000 specific section end **********/

/*
 ** This function is necessary for the TSP Designer Bridge.
 ** Called to allocate the specified amount of space in the DIAG output buffer.
 ** OEM must review this function and verify if the proposed function is used in their software package.
 */
void* ImmVibeSPI_Diag_BufPktAlloc(int nLength)
{
	/* No need to be implemented */

	/* return allocate_a_buffer(nLength); */
	return NULL;
}

/*
 ** Called to disable amp (disable output force)
 */
VibeStatus ImmVibeSPI_ForceOut_AmpDisable(VibeUInt8 nActuatorIndex)
{
	/* Disable amp */
	/* Disable the ISA1000 output */
	isa1000_vib_set_level(0);

	return VIBE_S_SUCCESS;
}

/*
 ** Called to enable amp (enable output force)
 */
VibeStatus ImmVibeSPI_ForceOut_AmpEnable(VibeUInt8 nActuatorIndex)
{
	/* Reset PWM frequency (only if necessary on Hong Mi 2A)*/
	/* To be implemented with appropriate hardware access macros */

	/* Set duty cycle to 50% (which correspond to the output level 0) */
	isa1000_vib_set_level(0);

	return VIBE_S_SUCCESS;
}

/*
 ** Called at initialization time to set PWM freq, disable amp, etc...
 */
VibeStatus ImmVibeSPI_ForceOut_Initialize(struct platform_device *pdev)
{
	int ret;

	vib_dev = kzalloc(sizeof(struct isa1000_vib), GFP_KERNEL);
	if (!vib_dev) {
		pr_err("%s: alloc vib_dev failed\n", __func__);
		return VIBE_E_FAIL;
	}

	ret = of_get_named_gpio(pdev->dev.of_node, "isa1000,gpio-en", 0);
	if (ret < 0) {
		pr_err("%s: unable to read isa1000,gpio-en gpio\n", __func__);
		return VIBE_E_FAIL;
	}
	vib_dev->gpio_en = ret;

	ret = of_get_named_gpio(pdev->dev.of_node, "isa1000,gpio-pwm-en", 0);
	if (ret < 0) {
		pr_err("%s: unable to read isa1000,gpio-pwm-en gpio\n", __func__);
		return VIBE_E_FAIL;
	}
	vib_dev->gpio_pwm_en = ret;

	ret = isa1000_vib_set_clock(pdev);
	if(ret < 0) {
		pr_err("%s: ISA1000 clock initialization failed\n", __func__);
		return VIBE_E_FAIL;
	}

	/* Kick start the ISA1000 set up */
	ret = isa1000_setup();
	if(ret < 0) {
		pr_err("%s, ISA1000 initialization failed\n", __func__);
		return VIBE_E_FAIL;
	}

	mutex_init(&vib_dev->lock);
	INIT_WORK(&vib_dev->work, isa1000_vib_update);

	hrtimer_init(&vib_dev->vib_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vib_dev->vib_timer.function = isa1000_vib_timer_func;
	vib_dev->timeout = ISA1000_VIB_DEFAULT_TIMEOUT;
	vib_dev->timed_dev.name = "vibrator";
	vib_dev->timed_dev.get_time = isa1000_vib_get_time;
	vib_dev->timed_dev.enable = isa1000_vib_enable;
	vib_dev->clk_enabled = false;
	ret = timed_output_dev_register(&vib_dev->timed_dev);
	if (ret < 0){
		pr_err("%s, ISA1000 register failed\n", __func__);
		return VIBE_E_FAIL;
	}

	/* Disable amp */
	ImmVibeSPI_ForceOut_AmpDisable(0);

	return VIBE_S_SUCCESS;
}

/*
 ** Called at termination time to set PWM freq, disable amp, etc...
 */
VibeStatus ImmVibeSPI_ForceOut_Terminate(void)
{
	/* Disable amp */
	ImmVibeSPI_ForceOut_AmpDisable(0);

	/* Set PWM frequency (only if necessary on Hong Mi 2A) */
	/* To be implemented with appropriate hardware access macros */

	/* Set duty cycle to 50% */
	/* i.e. output level to 0 */
	isa1000_vib_set_level(0);

	return VIBE_S_SUCCESS;
}

/*
 ** Called by the real-time loop to set force output, and enable amp if required
 */
VibeStatus ImmVibeSPI_ForceOut_SetSamples(VibeUInt8 nActuatorIndex, VibeUInt16 nOutputSignalBitDepth, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer)
{
	/*
	 ** For ERM:
	 **      nBufferSizeInBytes should be equal to 1 if nOutputSignalBitDepth is equal to 8
	 **      nBufferSizeInBytes should be equal to 2 if nOutputSignalBitDepth is equal to 16
	 */

	/* Below based on assumed 8 bit PWM, other implementation are possible */

	/* M = 1, N = 256, 1 <= nDutyCycle <= (N-M) */

	/* Output force: nForce is mapped from [-127, 127] to [1, 255] */
	int level;

	if(nOutputSignalBitDepth == 8) {
		if( nBufferSizeInBytes != 1) {
			pr_err("%s: Only support single sample for ERM\n",__func__);
			return VIBE_E_FAIL;
		} else {
			level = (signed char)(*pForceOutputBuffer);
		}
	} else if(nOutputSignalBitDepth == 16) {
		if( nBufferSizeInBytes != 2) {
			pr_err("%s: Only support single sample for ERM\n",__func__);
			return VIBE_E_FAIL;
		} else {
			level = (signed short)(*((signed short*)(pForceOutputBuffer)));
			/* Quantize it to 8-bit value as ISA1000 only support 8-bit levels */
			level >>= 8;
		}
	} else {
		pr_info("%s: Invalid Output Force Bit Depth\n",__func__);
		return VIBE_E_FAIL;
	}

	isa1000_vib_set_level(level);

	return VIBE_S_SUCCESS;
}

/*
 ** Called to set force output frequency parameters
 */
VibeStatus ImmVibeSPI_ForceOut_SetFrequency(VibeUInt8 nActuatorIndex, VibeUInt16 nFrequencyParameterID, VibeUInt32 nFrequencyParameterValue)
{
	return VIBE_S_SUCCESS;
}

/*
 ** Called to save an IVT data file (pIVT) to a file (szPathName)
 */
VibeStatus ImmVibeSPI_IVTFile_Save(const VibeUInt8 *pIVT, VibeUInt32 nIVTSize, const char *szPathname)
{
	return VIBE_S_SUCCESS;
}

/*
 ** Called to delete an IVT file
 */
VibeStatus ImmVibeSPI_IVTFile_Delete(const char *szPathname)
{
	return VIBE_S_SUCCESS;
}

/*
 ** Called to get the device name (device name must be returned as ANSI char)
 */
VibeStatus ImmVibeSPI_Device_GetName(VibeUInt8 nActuatorIndex, char *szDevName, int nSize)
{
	if ((!szDevName) || (nSize < 1))
		return VIBE_E_FAIL;

	strncpy(szDevName, ISA1000_BOARD_NAME, nSize-1);
	szDevName[nSize - 1] = '\0';

	return VIBE_S_SUCCESS;
}

/*
 ** Called at initialization time to get the number of actuators
 */
VibeStatus ImmVibeSPI_Device_GetNum(void)
{
	return NUM_ACTUATORS;
}
