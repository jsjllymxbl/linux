// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm PM8921 (PMIC4) switch mode battery charger driver
 *
 * The PM8921 integrates a single cell lithium ion "SMBC" (switch mode
 * battery charger) with an USB input path, an over voltage protected DC
 * input path and a buck converter that regulates the battery.
 *
 * The charger is driven by a hardware state machine: once the battery
 * limits and the input current limit are programmed and charging is
 * enabled, the FSM autonomously runs the charge cycle (trickle, fast
 * charge, end of charge), handles the JEITA battery temperature
 * comparators and resumes charging when the battery voltage drops below
 * the VBAT_DET threshold.  Software only has to configure the limits
 * and monitor the FSM.
 *
 * Register layout, encodings and the magic initialisation sequences are
 * taken from the downstream PM8921 charger driver
 * (drivers/power/pm8921-charger.c of the Qualcomm msm8960 kernel) and
 * cross checked against the Xiaomi Mi 2 (aries) board file, which
 * programs 4.2 V float voltage, 1000 mA charge current, 100 mA
 * termination current and the "high" JEITA thresholds.
 *
 * Copyright (c) 2026 jsjllymxbl <jsjllymxbl@gmail.com>
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/extcon.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>

#define PM8921_CHG_HWREV		0x002
#define PM8921_CHG_HWREV_2		0x0e8

#define PM8921_PBL_ACCESS2		0x005
#define PM8921_SYS_CONFIG_2		0x007
#define PM8921_CHG_BUCK_CLOCK_CTRL	0x014

#define PM8921_CHG_CNTRL		0x204
#define PM8921_CHG_IBAT_MAX		0x205
#define PM8921_CHG_TEST			0x206
#define PM8921_CHG_BUCK_CTRL_TEST3	0x209
#define PM8921_CHG_IBAT_SAFE		0x210
#define PM8921_CHG_ITRICKLE		0x211
#define PM8921_CHG_CNTRL_2		0x212
#define PM8921_CHG_VBAT_DET		0x213
#define PM8921_CHG_ITERM		0x215
#define PM8921_CHG_CNTRL_3		0x216
#define PM8921_CHG_TWDOG		0x218
#define PM8921_CHG_TCHG_MAX		0x21b
#define PM8921_CHG_VDD_MAX		0x220
#define PM8921_CHG_VDD_SAFE		0x221

/* CHG_CNTRL */
#define CHG_CHARGE_DIS			BIT(1)
#define VREF_BATT_THERM_FORCE_ON	BIT(7)

/* CHG_CNTRL_2 */
#define CHG_BAT_TEMP_DIS		BIT(2)
#define BATT_TEMP_THR_COLD		BIT(1)
#define BATT_TEMP_THR_HOT		BIT(0)

/* CHG_CNTRL_3 */
#define CHG_USB_SUSPEND			BIT(2)
#define CHG_EN				BIT(7)
#define CHG_FAILED_CLEAR		BIT(0)
#define ATC_FAILED_CLEAR		BIT(1)

/* SYS_CONFIG_2 */
#define BOOT_DONE			BIT(6)
#define BOOT_TIMER_EN			BIT(1)

/* PBL_ACCESS2 */
#define ENUM_TIMER_STOP			BIT(1)
#define IUSB_MAX_MASK			0x1c
#define IUSB_MAX_SHIFT			2

/* voltage encoding shared by VDD_MAX, VDD_SAFE and VBAT_DET */
#define CHG_V_MIN_MV			3240
#define CHG_V_STEP_MV			20
#define CHG_V_STEP_10MV_OFFSET		BIT(7)
#define CHG_V_MASK			0x7f
#define CHG_VDD_MAX_MIN			3400
#define CHG_VDD_MAX_MAX			4500
#define CHG_VDD_SAFE_MAX		4500

/* current encoding shared by IBAT_MAX and IBAT_SAFE */
#define CHG_I_MIN_MA			225
#define CHG_I_STEP_MA			50
#define CHG_I_MASK			0x3f
#define CHG_IBAT_MIN_MA			325
#define CHG_IBAT_MAX_MA			2000

#define CHG_ITERM_MIN_MA		50
#define CHG_ITERM_STEP_MA		10
#define CHG_ITERM_MASK			0x0f

#define CHG_WDOG_MASK			0x1f

#define CHG_TCHG_MIN			4
#define CHG_TCHG_MAX			512
#define CHG_TCHG_STEP			4
#define CHG_TCHG_MASK			0x7f

/* charger FSM states, only the low nibble is readable */
#define FSM_STATE_OFF			0
#define FSM_STATE_ON_CHG_HIGHL		1
#define FSM_STATE_ATC			2
#define FSM_STATE_ON_BAT		3
#define FSM_STATE_ATC_FAIL		4
#define FSM_STATE_DELAY			5
#define FSM_STATE_ON_CHG_AND_BAT	6
#define FSM_STATE_FAST_CHG		7
#define FSM_STATE_TRKL_CHG		8
#define FSM_STATE_CHG_FAIL		9
#define FSM_STATE_EOC			10
#define FSM_STATE_ON_CHG_VREGOK		11

/* commands to read the FSM state out of the test register */
#define CAPTURE_FSM_STATE_CMD		0xc2
#define READ_BANK_7			0x70
#define READ_BANK_4			0x40

/* PMIC4 die 3.0 needs a charger buck workaround */
#define PM8921_VERSION_MASK		0xfff0
#define PM8921_VERSION_VALUE		0x06f0
#define PM8921_REVISION_MASK		0x000f
#define PM8921_REVISION_3P0		0x4

enum pm8921_chg_irq_id {
	PM8921_IRQ_CHG_GONE,
	PM8921_IRQ_BATT_TEMP_OK,
	PM8921_IRQ_BATT_HOT,
	PM8921_IRQ_BATT_COLD,
	PM8921_IRQ_FAST_CHG,
	PM8921_IRQ_CHG_DONE,
	PM8921_IRQ_CHG_FAIL,
	PM8921_IRQ_VBAT_DET,
	PM8921_IRQ_WDOG,
	PM8921_IRQ_BATT_INSERTED,
	PM8921_IRQ_MAX,
};

static const char *const pm8921_chg_irq_names[] = {
	[PM8921_IRQ_CHG_GONE] = "chg-gone",
	[PM8921_IRQ_BATT_TEMP_OK] = "batt-temp-ok",
	[PM8921_IRQ_BATT_HOT] = "batt-hot",
	[PM8921_IRQ_BATT_COLD] = "batt-cold",
	[PM8921_IRQ_FAST_CHG] = "fast-chg",
	[PM8921_IRQ_CHG_DONE] = "chg-done",
	[PM8921_IRQ_CHG_FAIL] = "chg-fail",
	[PM8921_IRQ_VBAT_DET] = "vbat-det",
	[PM8921_IRQ_WDOG] = "wdog",
	[PM8921_IRQ_BATT_INSERTED] = "batt-inserted",
};

/**
 * struct pm8921_chg - charger instance
 * @dev:		device pointer
 * @map:		regmap of the parent PMIC
 * @lock:		serialises register updates and property reads
 * @usb_psy:		registered USB power supply
 * @batt_psy:		registered battery power supply
 * @edev:		extcon device reporting the USB cable
 * @usb_nb:		notifier receiving the cable events of @edev
 * @irqs:		charger interrupt numbers, indexed by pm8921_chg_irq_id
 * @max_voltage_mv:	battery float voltage
 * @resume_delta_mv:	voltage drop below the float voltage that restarts
 *			charging
 * @term_ma:		end of charge current
 * @ibat_max_ma:	max battery charge current
 * @iusb_max_ma:	USB input current limit
 * @safety_min:	max total charge time in minutes
 * @usb_online:		cached USB input presence
 */
struct pm8921_chg {
	struct device		*dev;
	struct regmap		*map;
	struct mutex		lock;	/* serialises register and FSM access */

	struct power_supply	*usb_psy;
	struct power_supply	*batt_psy;
	struct extcon_dev	*edev;
	struct notifier_block	usb_nb;

	int			irqs[PM8921_IRQ_MAX];

	unsigned int		max_voltage_mv;
	unsigned int		resume_delta_mv;
	unsigned int		term_ma;
	unsigned int		ibat_max_ma;
	unsigned int		iusb_max_ma;
	unsigned int		safety_min;

	bool			usb_online;
};

/*
 * USB input current limits, indexed by the IUSB_MAX field of PBL_ACCESS2.
 * The chip has a 4 bit field where the LSB selects a 50 mA fine step that
 * the PM8921 does not implement, so only the even codes are reachable.
 */
static const unsigned int pm8921_iusb_ma[] = {
	100, 500, 700, 850, 900, 1100, 1300, 1500,
};

static int pm8921_chg_write(struct pm8921_chg *chg, u16 addr, u8 val)
{
	return regmap_write(chg->map, addr, val);
}

static int pm8921_chg_masked_write(struct pm8921_chg *chg, u16 addr, u8 mask,
				   u8 val)
{
	return regmap_update_bits(chg->map, addr, mask, val);
}

static int pm8921_chg_read(struct pm8921_chg *chg, u16 addr, unsigned int *val)
{
	return regmap_read(chg->map, addr, val);
}

static int pm8921_chg_rt_status(struct pm8921_chg *chg, enum pm8921_chg_irq_id id)
{
	bool state = false;
	int ret;

	if (!chg->irqs[id])
		return -ENXIO;

	ret = irq_get_irqchip_state(chg->irqs[id], IRQCHIP_STATE_LINE_LEVEL,
				    &state);
	if (ret)
		return ret;

	return state;
}

/*
 * The battery charger FSM state is not mapped in the register space; it
 * has to be latched into CHG_TEST with the "capture" command and read
 * back from bank 7 of the test mux.
 */
static int pm8921_chg_get_fsm_state(struct pm8921_chg *chg)
{
	unsigned int val;
	int ret;

	ret = pm8921_chg_write(chg, PM8921_CHG_TEST, CAPTURE_FSM_STATE_CMD);
	if (ret)
		return ret;

	ret = pm8921_chg_write(chg, PM8921_CHG_TEST, READ_BANK_7);
	if (ret)
		return ret;

	ret = pm8921_chg_read(chg, PM8921_CHG_TEST, &val);
	if (ret)
		return ret;

	/* restore the test mux */
	pm8921_chg_write(chg, PM8921_CHG_TEST, READ_BANK_4);

	/*
	 * Only the low nibble carries the state, bit 4 is always set while
	 * the charger runs (a FAST_CHG state reads back as 0x17).
	 */
	return val & 0x0f;
}

/*
 * Force the charger buck onto the always on 19.2 MHz clock.  The charger
 * must not run on the switched clock while it is being reprogrammed and
 * the FSM needs a stable clock for the charge cycle; the magic sequence
 * and the delays are required by the hardware, plain writes to the clock
 * control register do not do the job.
 */
static void pm8921_chg_force_19p2mhz(struct pm8921_chg *chg)
{
	pm8921_chg_write(chg, PM8921_CHG_TEST, 0xd1);
	pm8921_chg_write(chg, PM8921_CHG_TEST, 0xd3);
	pm8921_chg_write(chg, PM8921_CHG_TEST, 0xd1);
	pm8921_chg_write(chg, PM8921_CHG_TEST, 0xd5);
	usleep_range(183, 300);
	pm8921_chg_write(chg, PM8921_CHG_TEST, 0xd1);
	pm8921_chg_write(chg, PM8921_CHG_TEST, 0xd0);
	usleep_range(32, 100);
	pm8921_chg_write(chg, PM8921_CHG_TEST, 0xd1);
	pm8921_chg_write(chg, PM8921_CHG_TEST, 0xd3);
}

static int pm8921_chg_set_voltage(struct pm8921_chg *chg, u16 addr,
				  unsigned int mv)
{
	u8 val;

	if (mv < CHG_VDD_MAX_MIN || mv > CHG_VDD_MAX_MAX)
		return -EINVAL;

	val = (mv - CHG_V_MIN_MV) / CHG_V_STEP_MV;
	if (mv % CHG_V_STEP_MV >= CHG_V_STEP_MV / 2)
		val |= CHG_V_STEP_10MV_OFFSET;

	return pm8921_chg_masked_write(chg, addr, CHG_V_MASK, val);
}

static int pm8921_chg_set_current(struct pm8921_chg *chg, u16 addr,
				  unsigned int ma)
{
	u8 val;

	if (ma < CHG_IBAT_MIN_MA || ma > CHG_IBAT_MAX_MA)
		return -EINVAL;

	val = (ma - CHG_I_MIN_MA) / CHG_I_STEP_MA;

	return pm8921_chg_masked_write(chg, addr, CHG_I_MASK, val);
}

static int pm8921_chg_set_iusb_max(struct pm8921_chg *chg, unsigned int ma)
{
	unsigned int code = 0;
	int i;

	for (i = ARRAY_SIZE(pm8921_iusb_ma) - 1; i > 0; i--) {
		if (pm8921_iusb_ma[i] <= ma)
			break;
	}
	code = i;

	return pm8921_chg_masked_write(chg, PM8921_PBL_ACCESS2,
				       IUSB_MAX_MASK, code << IUSB_MAX_SHIFT);
}

static unsigned int pm8921_chg_get_iusb_max(struct pm8921_chg *chg)
{
	unsigned int val;
	int ret;

	ret = pm8921_chg_read(chg, PM8921_PBL_ACCESS2, &val);
	if (ret)
		return chg->iusb_max_ma;

	val = (val & IUSB_MAX_MASK) >> IUSB_MAX_SHIFT;

	return pm8921_iusb_ma[val];
}

static int pm8921_chg_hw_init(struct pm8921_chg *chg)
{
	unsigned int hwrev, hwrev2, fsm;
	int ret;

	pm8921_chg_force_19p2mhz(chg);

	/* tell the FSM that the boot loader handed over to the OS */
	ret = pm8921_chg_masked_write(chg, PM8921_SYS_CONFIG_2,
				      BOOT_DONE | BOOT_TIMER_EN,
				      BOOT_DONE | BOOT_TIMER_EN);
	if (ret)
		return ret;

	/* safe voltage slightly above the float voltage */
	ret = pm8921_chg_set_voltage(chg, PM8921_CHG_VDD_SAFE,
				     min(chg->max_voltage_mv + 40,
					 (unsigned int)CHG_VDD_SAFE_MAX));
	if (ret)
		return ret;

	/* restart charging when the battery falls below this level */
	ret = pm8921_chg_set_voltage(chg, PM8921_CHG_VBAT_DET,
				     chg->max_voltage_mv - chg->resume_delta_mv);
	if (ret)
		return ret;

	ret = pm8921_chg_set_voltage(chg, PM8921_CHG_VDD_MAX,
				     chg->max_voltage_mv);
	if (ret)
		return ret;

	ret = pm8921_chg_set_current(chg, PM8921_CHG_IBAT_SAFE, 1500);
	if (ret)
		return ret;

	ret = pm8921_chg_set_current(chg, PM8921_CHG_IBAT_MAX,
				     chg->ibat_max_ma);
	if (ret)
		return ret;

	ret = pm8921_chg_masked_write(chg, PM8921_CHG_ITERM, CHG_ITERM_MASK,
				      (chg->term_ma - CHG_ITERM_MIN_MA) /
				      CHG_ITERM_STEP_MA);
	if (ret)
		return ret;

	/* the USB enumeration timer is managed by the USB driver */
	ret = pm8921_chg_masked_write(chg, PM8921_PBL_ACCESS2,
				      ENUM_TIMER_STOP, ENUM_TIMER_STOP);
	if (ret)
		return ret;

	/* charge safety timer */
	ret = pm8921_chg_masked_write(chg, PM8921_CHG_TCHG_MAX, CHG_TCHG_MASK,
				      (chg->safety_min - 1) / CHG_TCHG_STEP);
	if (ret)
		return ret;

	/* the watchdog is kicked from software only when needed */
	ret = pm8921_chg_masked_write(chg, PM8921_CHG_TWDOG, CHG_WDOG_MASK, 0);
	if (ret)
		return ret;

	/* battery temperature comparators, "high" cold/hot thresholds */
	ret = pm8921_chg_masked_write(chg, PM8921_CHG_CNTRL_2,
				      CHG_BAT_TEMP_DIS | BATT_TEMP_THR_COLD |
				      BATT_TEMP_THR_HOT,
				      BATT_TEMP_THR_COLD | BATT_TEMP_THR_HOT);
	if (ret)
		return ret;

	/* run the buck at 3.2 MHz */
	ret = pm8921_chg_write(chg, PM8921_CHG_BUCK_CLOCK_CTRL, 0x15);
	if (ret)
		return ret;

	ret = pm8921_chg_read(chg, PM8921_CHG_HWREV, &hwrev);
	if (ret)
		return ret;
	ret = pm8921_chg_read(chg, PM8921_CHG_HWREV_2, &hwrev2);
	if (ret)
		return ret;

	hwrev |= hwrev2 << 8;

	if ((hwrev & PM8921_VERSION_MASK) == PM8921_VERSION_VALUE &&
	    (hwrev & PM8921_REVISION_MASK) == PM8921_REVISION_3P0) {
		dev_info(chg->dev, "PM8921 die 3.0, applying the buck workaround\n");
		pm8921_chg_write(chg, PM8921_CHG_BUCK_CTRL_TEST3, 0xac);
	}

	/*
	 * The end of charge FSM processing is disabled and the charger is
	 * left to the voltage/current regulation loops; this is what the
	 * downstream driver does for every PM8921 revision.
	 */
	pm8921_chg_write(chg, PM8921_CHG_BUCK_CTRL_TEST3, 0xd9);
	pm8921_chg_write(chg, PM8921_CHG_BUCK_CTRL_TEST3, 0x91);

	/* keep the battery thermistor reference on, it also works as a
	 * "battery was not removed" marker for the boot loader
	 */
	ret = pm8921_chg_masked_write(chg, PM8921_CHG_CNTRL,
				      VREF_BATT_THERM_FORCE_ON,
				      VREF_BATT_THERM_FORCE_ON);
	if (ret)
		return ret;

	/* enable the USB input path at the default current limit */
	ret = pm8921_chg_masked_write(chg, PM8921_CHG_CNTRL_3,
				      CHG_USB_SUSPEND, 0);
	if (ret)
		return ret;

	ret = pm8921_chg_set_iusb_max(chg, chg->iusb_max_ma);
	if (ret)
		return ret;

	ret = pm8921_chg_masked_write(chg, PM8921_CHG_CNTRL,
				      CHG_CHARGE_DIS, 0);
	if (ret)
		return ret;

	/* clear a stale charge/charger detection failure from the boot loader */
	ret = pm8921_chg_masked_write(chg, PM8921_CHG_CNTRL_3,
				      CHG_FAILED_CLEAR | ATC_FAILED_CLEAR,
				      CHG_FAILED_CLEAR | ATC_FAILED_CLEAR);
	if (ret)
		return ret;

	ret = pm8921_chg_masked_write(chg, PM8921_CHG_CNTRL_3, CHG_EN, CHG_EN);
	if (ret)
		return ret;

	fsm = pm8921_chg_get_fsm_state(chg);

	dev_dbg(chg->dev,
		"initialised: %umV float, %umA charge, %umA USB, %umA EOC, fsm=%u\n",
		chg->max_voltage_mv, chg->ibat_max_ma, chg->iusb_max_ma,
		chg->term_ma, fsm);

	return 0;
}

/* charger state machine to power supply status, as the vendor driver maps it */
static const int pm8921_chg_fsm_status[] = {
	[FSM_STATE_OFF]			= POWER_SUPPLY_STATUS_UNKNOWN,
	[FSM_STATE_ON_CHG_HIGHL]	= POWER_SUPPLY_STATUS_FULL,
	[FSM_STATE_ATC]			= POWER_SUPPLY_STATUS_CHARGING,
	[FSM_STATE_ON_BAT]		= POWER_SUPPLY_STATUS_DISCHARGING,
	[FSM_STATE_ATC_FAIL]		= POWER_SUPPLY_STATUS_DISCHARGING,
	[FSM_STATE_DELAY]		= POWER_SUPPLY_STATUS_UNKNOWN,
	[FSM_STATE_ON_CHG_AND_BAT]	= POWER_SUPPLY_STATUS_CHARGING,
	[FSM_STATE_FAST_CHG]		= POWER_SUPPLY_STATUS_CHARGING,
	[FSM_STATE_TRKL_CHG]		= POWER_SUPPLY_STATUS_CHARGING,
	[FSM_STATE_CHG_FAIL]		= POWER_SUPPLY_STATUS_DISCHARGING,
	[FSM_STATE_EOC]			= POWER_SUPPLY_STATUS_FULL,
	[FSM_STATE_ON_CHG_VREGOK]	= POWER_SUPPLY_STATUS_NOT_CHARGING,
	[12]				= POWER_SUPPLY_STATUS_UNKNOWN,
	[13]				= POWER_SUPPLY_STATUS_NOT_CHARGING,
	[14]				= POWER_SUPPLY_STATUS_NOT_CHARGING,
	[15]				= POWER_SUPPLY_STATUS_NOT_CHARGING,
};

static int pm8921_chg_status(struct pm8921_chg *chg)
{
	int fsm, ret;

	if (!chg->usb_online)
		return POWER_SUPPLY_STATUS_DISCHARGING;

	ret = pm8921_chg_rt_status(chg, PM8921_IRQ_BATT_TEMP_OK);
	if (ret == 0)
		return POWER_SUPPLY_STATUS_NOT_CHARGING;

	fsm = pm8921_chg_get_fsm_state(chg);
	if (fsm < 0 || fsm >= (int)ARRAY_SIZE(pm8921_chg_fsm_status))
		return POWER_SUPPLY_STATUS_UNKNOWN;

	return pm8921_chg_fsm_status[fsm];
}

static int pm8921_chg_charge_type(struct pm8921_chg *chg)
{
	int fsm = pm8921_chg_get_fsm_state(chg);

	switch (fsm) {
	case FSM_STATE_FAST_CHG:
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	case FSM_STATE_TRKL_CHG:
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	case FSM_STATE_ATC:
	case FSM_STATE_ON_CHG_AND_BAT:
		return POWER_SUPPLY_CHARGE_TYPE_STANDARD;
	default:
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	}
}

static int pm8921_chg_health(struct pm8921_chg *chg)
{
	if (pm8921_chg_rt_status(chg, PM8921_IRQ_BATT_TEMP_OK) == 0) {
		if (pm8921_chg_rt_status(chg, PM8921_IRQ_BATT_HOT) == 1)
			return POWER_SUPPLY_HEALTH_HOT;
		if (pm8921_chg_rt_status(chg, PM8921_IRQ_BATT_COLD) == 1)
			return POWER_SUPPLY_HEALTH_COLD;
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	}

	return POWER_SUPPLY_HEALTH_GOOD;
}

static enum power_supply_property pm8921_chg_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
};

static int pm8921_chg_usb_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct pm8921_chg *chg = power_supply_get_drvdata(psy);
	int ret = 0;

	mutex_lock(&chg->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = chg->usb_online;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = pm8921_chg_status(chg);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = pm8921_chg_charge_type(chg);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = pm8921_chg_health(chg);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		val->intval = pm8921_chg_get_iusb_max(chg) * 1000;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&chg->lock);

	return ret;
}

static int pm8921_chg_usb_set_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       const union power_supply_propval *val)
{
	struct pm8921_chg *chg = power_supply_get_drvdata(psy);
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		mutex_lock(&chg->lock);
		ret = pm8921_chg_set_iusb_max(chg, val->intval / 1000);
		if (!ret)
			chg->iusb_max_ma = pm8921_chg_get_iusb_max(chg);
		mutex_unlock(&chg->lock);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int
pm8921_chg_usb_property_is_writeable(struct power_supply *psy,
				     enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		return 1;
	default:
		return 0;
	}
}

static const struct power_supply_desc pm8921_chg_usb_desc = {
	.name			= "pm8921-usb",
	.type			= POWER_SUPPLY_TYPE_USB,
	.properties		= pm8921_chg_usb_props,
	.num_properties		= ARRAY_SIZE(pm8921_chg_usb_props),
	.get_property		= pm8921_chg_usb_get_property,
	.set_property		= pm8921_chg_usb_set_property,
	.property_is_writeable	= pm8921_chg_usb_property_is_writeable,
};

/*
 * The battery pack of this machine has no fuel gauge that the kernel can
 * talk to (the BQ27520 of the reference design does not acknowledge its
 * I2C address, and the mainline PM8921 XOADC currently reads a constant
 * ground code on every channel), so only the properties the charger
 * hardware itself knows about are reported.  Voltage, charge counters and
 * state of charge need the PMIC BMS/coulomb counter to be brought up.
 */
static enum power_supply_property pm8921_chg_batt_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
};

static int pm8921_chg_batt_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct pm8921_chg *chg = power_supply_get_drvdata(psy);
	int ret = 0;

	mutex_lock(&chg->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = pm8921_chg_rt_status(chg,
						   PM8921_IRQ_BATT_INSERTED) == 1;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = pm8921_chg_status(chg);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = pm8921_chg_charge_type(chg);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = pm8921_chg_health(chg);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&chg->lock);

	return ret;
}

static const struct power_supply_desc pm8921_chg_batt_desc = {
	.name			= "pm8921-battery",
	.type			= POWER_SUPPLY_TYPE_BATTERY,
	.properties		= pm8921_chg_batt_props,
	.num_properties		= ARRAY_SIZE(pm8921_chg_batt_props),
	.get_property		= pm8921_chg_batt_get_property,
};

static void pm8921_chg_notify(struct pm8921_chg *chg)
{
	if (chg->usb_psy)
		power_supply_changed(chg->usb_psy);
	if (chg->batt_psy)
		power_supply_changed(chg->batt_psy);
}

/*
 * The cable state is watched by the PMIC extcon device (its VBUS valid
 * comparator sits in the always on power domain), which frees this driver
 * from owning the interrupt and lets the USB controller share it.
 */
static int pm8921_chg_usb_notifier(struct notifier_block *nb,
				   unsigned long event, void *ptr)
{
	struct pm8921_chg *chg = container_of(nb, struct pm8921_chg, usb_nb);
	bool online = event;

	if (online == chg->usb_online)
		return NOTIFY_DONE;

	chg->usb_online = online;
	dev_dbg(chg->dev, "USB %s\n", online ? "connected" : "disconnected");

	if (online) {
		mutex_lock(&chg->lock);
		/* restart a charge cycle that failed earlier */
		pm8921_chg_masked_write(chg, PM8921_CHG_CNTRL_3,
					CHG_FAILED_CLEAR | ATC_FAILED_CLEAR,
					CHG_FAILED_CLEAR | ATC_FAILED_CLEAR);
		pm8921_chg_masked_write(chg, PM8921_CHG_CNTRL_3,
					CHG_USB_SUSPEND, 0);
		pm8921_chg_set_iusb_max(chg, chg->iusb_max_ma);
		mutex_unlock(&chg->lock);
	}

	pm8921_chg_notify(chg);

	return NOTIFY_OK;
}

static irqreturn_t pm8921_chg_isr(int irq, void *data)
{
	struct pm8921_chg *chg = data;

	dev_dbg(chg->dev, "event: usb=%d fsm=%d status=%d health=%d\n",
		chg->usb_online, pm8921_chg_get_fsm_state(chg),
		pm8921_chg_status(chg), pm8921_chg_health(chg));

	pm8921_chg_notify(chg);

	return IRQ_HANDLED;
}

static irqreturn_t pm8921_chg_wdog_isr(int irq, void *data)
{
	struct pm8921_chg *chg = data;

	dev_dbg(chg->dev, "charger watchdog fired, rearming\n");

	mutex_lock(&chg->lock);
	pm8921_chg_masked_write(chg, PM8921_CHG_TWDOG, CHG_WDOG_MASK, 0);
	pm8921_chg_masked_write(chg, PM8921_CHG_CNTRL_3,
				CHG_FAILED_CLEAR | ATC_FAILED_CLEAR,
				CHG_FAILED_CLEAR | ATC_FAILED_CLEAR);
	pm8921_chg_masked_write(chg, PM8921_CHG_CNTRL_3, CHG_EN, CHG_EN);
	mutex_unlock(&chg->lock);

	pm8921_chg_notify(chg);

	return IRQ_HANDLED;
}

static irqreturn_t pm8921_chg_chg_gone_isr(int irq, void *data)
{
	struct pm8921_chg *chg = data;

	dev_dbg(chg->dev, "charge source gone\n");
	pm8921_chg_notify(chg);

	return IRQ_HANDLED;
}

static int pm8921_chg_parse_dt(struct pm8921_chg *chg)
{
	struct device_node *np = chg->dev->of_node;
	int ret;

	ret = of_property_read_u32(np, "qcom,max-voltage-mv",
				   &chg->max_voltage_mv);
	if (ret)
		chg->max_voltage_mv = 4200;

	ret = of_property_read_u32(np, "qcom,resume-voltage-delta-mv",
				   &chg->resume_delta_mv);
	if (ret)
		chg->resume_delta_mv = 20;

	ret = of_property_read_u32(np, "qcom,term-current-ma", &chg->term_ma);
	if (ret)
		chg->term_ma = 100;

	ret = of_property_read_u32(np, "qcom,max-charge-current-ma",
				   &chg->ibat_max_ma);
	if (ret)
		chg->ibat_max_ma = 1000;

	ret = of_property_read_u32(np, "qcom,usb-current-limit-ma",
				   &chg->iusb_max_ma);
	if (ret)
		chg->iusb_max_ma = 500;

	ret = of_property_read_u32(np, "qcom,safety-time-min", &chg->safety_min);
	if (ret)
		chg->safety_min = 480;

	if (chg->safety_min < CHG_TCHG_MIN || chg->safety_min > CHG_TCHG_MAX) {
		dev_err(chg->dev, "invalid safety time %u\n", chg->safety_min);
		return -EINVAL;
	}

	if (chg->term_ma < CHG_ITERM_MIN_MA || chg->term_ma > 200) {
		dev_err(chg->dev, "invalid termination current %u\n",
			chg->term_ma);
		return -EINVAL;
	}

	return 0;
}

static int pm8921_chg_probe(struct platform_device *pdev)
{
	struct pm8921_chg *chg;
	struct power_supply_config psy_cfg = {};
	int i, ret;

	chg = devm_kzalloc(&pdev->dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	chg->dev = &pdev->dev;
	mutex_init(&chg->lock);

	chg->map = dev_get_regmap(pdev->dev.parent, NULL);
	if (!chg->map) {
		dev_err(&pdev->dev, "no regmap for the parent PMIC\n");
		return -ENODEV;
	}

	ret = pm8921_chg_parse_dt(chg);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, chg);

	/*
	 * Resolve the interrupts first: the real time state of the charger
	 * (battery present, temperature) is read through them, so they are
	 * needed before the power supplies can answer a single property.
	 */
	for (i = 0; i < PM8921_IRQ_MAX; i++) {
		chg->irqs[i] = platform_get_irq_byname(pdev, pm8921_chg_irq_names[i]);
		if (chg->irqs[i] < 0)
			return dev_err_probe(&pdev->dev, chg->irqs[i],
					     "missing %s interrupt\n",
					     pm8921_chg_irq_names[i]);
	}

	chg->edev = extcon_get_edev_by_phandle(&pdev->dev, 0);
	if (IS_ERR(chg->edev))
		return dev_err_probe(&pdev->dev, PTR_ERR(chg->edev),
				     "failed to get the USB extcon\n");

	chg->usb_online = extcon_get_state(chg->edev, EXTCON_USB) > 0;

	psy_cfg.drv_data = chg;
	psy_cfg.fwnode = dev_fwnode(&pdev->dev);

	/*
	 * The power supplies have to be up before any interrupt is
	 * requested: the PMIC latches the charger state at boot and the
	 * first event can fire immediately.
	 */
	chg->usb_psy = devm_power_supply_register(&pdev->dev,
						  &pm8921_chg_usb_desc,
						  &psy_cfg);
	if (IS_ERR(chg->usb_psy))
		return dev_err_probe(&pdev->dev, PTR_ERR(chg->usb_psy),
				     "failed to register the USB supply\n");

	chg->batt_psy = devm_power_supply_register(&pdev->dev,
						   &pm8921_chg_batt_desc,
						   &psy_cfg);
	if (IS_ERR(chg->batt_psy))
		return dev_err_probe(&pdev->dev, PTR_ERR(chg->batt_psy),
				     "failed to register the battery supply\n");

	for (i = 0; i < PM8921_IRQ_MAX; i++) {
		irq_handler_t handler = pm8921_chg_isr;

		if (i == PM8921_IRQ_WDOG)
			handler = pm8921_chg_wdog_isr;
		else if (i == PM8921_IRQ_CHG_GONE)
			handler = pm8921_chg_chg_gone_isr;

		ret = devm_request_threaded_irq(&pdev->dev, chg->irqs[i], NULL,
						handler, IRQF_ONESHOT,
						pm8921_chg_irq_names[i], chg);
		if (ret) {
			dev_err(&pdev->dev, "failed to request %s: %d\n",
				pm8921_chg_irq_names[i], ret);
			return ret;
		}
	}

	ret = pm8921_chg_hw_init(chg);
	if (ret) {
		dev_err(&pdev->dev, "failed to initialise the charger: %d\n", ret);
		return ret;
	}

	chg->usb_nb.notifier_call = pm8921_chg_usb_notifier;
	ret = devm_extcon_register_notifier(&pdev->dev, chg->edev, EXTCON_USB,
					    &chg->usb_nb);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to watch the USB extcon\n");

	chg->usb_online = extcon_get_state(chg->edev, EXTCON_USB) > 0;
	chg->iusb_max_ma = pm8921_chg_get_iusb_max(chg);

	dev_dbg(&pdev->dev, "USB %s, input limit %umA\n",
		chg->usb_online ? "connected" : "disconnected", chg->iusb_max_ma);

	return 0;
}

static const struct of_device_id pm8921_chg_of_match[] = {
	{ .compatible = "qcom,pm8921-charger" },
	{ }
};
MODULE_DEVICE_TABLE(of, pm8921_chg_of_match);

static struct platform_driver pm8921_chg_driver = {
	.probe = pm8921_chg_probe,
	.driver = {
		.name = "pm8921-charger",
		.of_match_table = pm8921_chg_of_match,
	},
};
module_platform_driver(pm8921_chg_driver);

MODULE_DESCRIPTION("Qualcomm PM8921 switch mode battery charger");
MODULE_LICENSE("GPL");
