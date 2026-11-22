// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm PM8921 (PMIC4) battery monitor driver
 *
 * The PM8921 integrates a "BMS" (battery monitor system): a coulomb
 * counter with its own ADC that measures the battery voltage, the
 * voltage across the pack current sense resistor and the charge that
 * went in or out of the pack.  The values are published through a small
 * register window (BMS_CONTROL / BMS_OUTPUT0-1) where software selects
 * which quantity it wants to read.
 *
 * The coulomb counter is the main source of the state of charge: it is
 * integrated from an anchor that is taken from the open circuit voltage
 * whenever the pack is at rest, which keeps the estimate from drifting
 * without needing a full charge cycle to learn the capacity of the pack.
 *
 * Register layout, encodings and the scaling of the counter are taken
 * from the downstream pm8921-bms driver of the Qualcomm msm8960 kernel
 * and validated against it on a Xiaomi Mi 2 (aries), which uses a 10
 * mOhm sense resistor: the raw counter moves 892363 counts over 24 s at
 * about 570 mA, so one count is 0.00412 uAh.
 *
 * Copyright (c) 2026 jsjllymxbl <jsjllymxbl@gmail.com>
 */

#include <linux/delay.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

#define PM8921_BMS_CONTROL		0x224
#define PM8921_BMS_S1_DELAY		0x225
#define PM8921_BMS_OUTPUT0		0x230
#define PM8921_BMS_OUTPUT1		0x231
#define PM8921_BMS_TOLERANCES		0x232
/* survives a reboot: the PMIC stays powered as long as the pack is in */
#define PM8921_BMS_SOC_STORAGE		0x107

/* BMS_CONTROL */
#define BMS_EN				BIT(7)
#define BMS_MODE			BIT(6)
#define BMS_EN_VBAT			BIT(5)
#define BMS_SELECT_MASK			0x1c
#define BMS_SELECT_SHIFT		2
#define BMS_HOLD_OUTPUT			BIT(1)
#define BMS_EN_PON_HS			BIT(0)

/* BMS_TOLERANCES */
#define BMS_IBAT_TOL_MASK		0x0f
#define BMS_IBAT_TOL_DEFAULT		0x03
#define BMS_OCV_TOL_MASK		0xf0
#define BMS_OCV_TOL_DEFAULT		0x20

/* quantities that can be selected in BMS_CONTROL */
#define BMS_SELECT_VSENSE_AVG		6
#define BMS_SELECT_VBATT_AVG		7
#define BMS_SELECT_CC_MSB		3
#define BMS_SELECT_CC_LSB		4

/*
 * The BMS publishes the battery voltage as a 16 bit code with an
 * intrinsic offset, scaled so that one bit is 97.656 uV.  The divider in
 * front of the BMS input is 1/3.
 */
#define BMS_VBATT_CODE_OFFSET		0x6000
#define BMS_CODE_UV_MUL			97656
#define BMS_CODE_UV_DIV			1000
#define BMS_VBATT_DIV			3

/* the current sense ADC has its own scale: 5.42535 uV per bit */
#define BMS_VSENSE_UV_MUL		542535
#define BMS_VSENSE_UV_DIV		100000

/*
 * One count of the coulomb counter is
 *   868056 / 10000 uV  *  56 / 32764 Hz  /  r_sense
 * of charge, i.e. 0.00412 uAh with the 10 mOhm sense resistor of the
 * Mi 2.  Keep it as a fraction so that the value follows the resistor.
 */
#define BMS_CC_UA_MUL			412124
#define BMS_CC_UA_DIV			10000000

#define BMS_FORCE_MEASURE_DELAY_MS	20
#define BMS_POLL_INTERVAL_MS		30000

/* below this current the pack is considered at rest */
#define BMS_REST_CURRENT_UA		50000
/* deviation from the open circuit voltage estimate that re-anchors it */
#define BMS_REANCHOR_PERCENT		10
/* a stored capacity further than this from the voltage is not trusted */
#define BMS_STORED_SOC_TOLERANCE	30
/* 0xff marks an empty pack, 0 means that nothing is stored */
#define BMS_SOC_STORAGE_EMPTY		0xff

/**
 * struct pm8921_bms - battery monitor instance
 * @dev:	device pointer
 * @map:	regmap of the parent PMIC
 * @lock:	serialises register accesses
 * @r_sense_mohm:	value of the current sense resistor
 * @batt_psy:	registered battery power supply
 * @work:	periodic sampler
 * @volt_uv:	last sampled battery voltage
 * @curr_ua:	last sampled battery current, positive while charging
 * @ocv_uv:	open circuit voltage derived from @volt_uv and @curr_ua
 * @cc:		coulomb counter of the last sample
 * @charge_anchor_uah:	charge of the pack when the counter was anchored
 * @cc_anchor:	counter value at the anchor
 * @charge_uah:	charge of the pack of the last sample
 * @stored_capacity:	state of charge left in the PMIC by the last boot
 * @capacity:	state of charge in percent, -1 until the first anchor
 * @anchored:	whether the counter has been anchored at least once
 */
struct pm8921_bms {
	struct device		*dev;
	struct regmap		*map;
	struct mutex		lock;
	unsigned int		r_sense_mohm;

	struct power_supply	*batt_psy;
	struct delayed_work	work;

	int			volt_uv;
	int			curr_ua;
	int			ocv_uv;

	s32			cc;
	s64			charge_anchor_uah;
	s32			cc_anchor;
	s64			charge_uah;
	int			stored_capacity;
	int			capacity;
	bool			anchored;
};

static int pm8921_bms_read_output(struct pm8921_bms *bms, unsigned int type,
				  u16 *value)
{
	unsigned int lsb, msb;
	int ret;

	ret = regmap_update_bits(bms->map, PM8921_BMS_CONTROL,
				 BMS_SELECT_MASK, type << BMS_SELECT_SHIFT);
	if (ret)
		return ret;

	ret = regmap_read(bms->map, PM8921_BMS_OUTPUT0, &lsb);
	if (ret)
		return ret;

	ret = regmap_read(bms->map, PM8921_BMS_OUTPUT1, &msb);
	if (ret)
		return ret;

	*value = (msb << 8) | lsb;

	return 0;
}

/*
 * Ask the BMS for a fresh set of readings.  In its normal mode the block
 * only refreshes its output registers when the battery current or
 * voltage moves further than the configured tolerances, so a busy system
 * can otherwise read the same sample for minutes.  "BMS_MODE" together
 * with "EN_VBAT" makes it measure voltage and current at once and the
 * output registers have to be held while they are read.
 */
static int pm8921_bms_sample(struct pm8921_bms *bms, int *volt_uv,
			     int *curr_ua, s32 *cc)
{
	u16 vbat_raw, vsense_raw, msb, lsb;
	s64 uv;
	int ret, curr;

	mutex_lock(&bms->lock);

	ret = regmap_write(bms->map, PM8921_BMS_S1_DELAY, 0);
	if (ret)
		goto out_unlock;

	ret = regmap_update_bits(bms->map, PM8921_BMS_CONTROL,
				 BMS_MODE | BMS_EN_VBAT,
				 BMS_MODE | BMS_EN_VBAT);
	if (ret)
		goto out_unlock;

	msleep(BMS_FORCE_MEASURE_DELAY_MS);

	ret = regmap_update_bits(bms->map, PM8921_BMS_CONTROL, BMS_HOLD_OUTPUT,
				 BMS_HOLD_OUTPUT);
	if (ret)
		goto out_clear_mode;

	ret = pm8921_bms_read_output(bms, BMS_SELECT_VBATT_AVG, &vbat_raw);
	if (ret)
		goto out_release;

	ret = pm8921_bms_read_output(bms, BMS_SELECT_VSENSE_AVG, &vsense_raw);
	if (ret)
		goto out_release;

	ret = pm8921_bms_read_output(bms, BMS_SELECT_CC_MSB, &msb);
	if (ret)
		goto out_release;

	ret = pm8921_bms_read_output(bms, BMS_SELECT_CC_LSB, &lsb);
	if (ret)
		goto out_release;

	regmap_update_bits(bms->map, PM8921_BMS_CONTROL, BMS_HOLD_OUTPUT, 0);
	regmap_update_bits(bms->map, PM8921_BMS_CONTROL,
			   BMS_MODE | BMS_EN_VBAT, 0);
	mutex_unlock(&bms->lock);

	uv = div_s64((s64)((int)vbat_raw - BMS_VBATT_CODE_OFFSET) *
		     BMS_CODE_UV_MUL, BMS_CODE_UV_DIV);
	if (uv < 0)
		uv = 0;
	*volt_uv = min_t(s64, uv * BMS_VBATT_DIV, INT_MAX);

	uv = div_s64((s64)(s16)vsense_raw * BMS_VSENSE_UV_MUL,
		     BMS_VSENSE_UV_DIV);
	curr = div_s64(uv * 1000, bms->r_sense_mohm);
	/* the sense voltage is positive while the pack is being discharged */
	*curr_ua = -curr;

	*cc = (s32)(((u32)msb << 16) | lsb);

	return 0;

out_release:
	regmap_update_bits(bms->map, PM8921_BMS_CONTROL, BMS_HOLD_OUTPUT, 0);
out_clear_mode:
	regmap_update_bits(bms->map, PM8921_BMS_CONTROL,
			   BMS_MODE | BMS_EN_VBAT, 0);
out_unlock:
	mutex_unlock(&bms->lock);

	return ret;
}

/* charge that went into the pack since the counter was anchored */
static s64 pm8921_bms_cc_delta_uah(struct pm8921_bms *bms, s32 cc)
{
	return div_s64(((s64)bms->cc_anchor - cc) * BMS_CC_UA_MUL,
		       (s64)BMS_CC_UA_DIV * bms->r_sense_mohm);
}

/*
 * The state of charge of the last session is left in a PMIC register that
 * keeps its content as long as the pack is connected, which lets the
 * estimate survive a reboot instead of starting over from the voltage.
 */
static int pm8921_bms_read_stored_capacity(struct pm8921_bms *bms)
{
	unsigned int val;
	int ret;

	ret = regmap_read(bms->map, PM8921_BMS_SOC_STORAGE, &val);
	if (ret)
		return -EINVAL;

	if (val == BMS_SOC_STORAGE_EMPTY)
		return 0;
	if (val == 0 || val > 100)
		return -EINVAL;

	return val;
}

static void pm8921_bms_store_capacity(struct pm8921_bms *bms, int capacity)
{
	unsigned int val = capacity ? capacity : BMS_SOC_STORAGE_EMPTY;

	regmap_write(bms->map, PM8921_BMS_SOC_STORAGE, val);
}

static void pm8921_bms_anchor(struct pm8921_bms *bms, int percent)
{
	struct power_supply_battery_info *info = bms->batt_psy->battery_info;

	bms->charge_anchor_uah =
		div_s64((s64)percent * info->charge_full_design_uah, 100);
	bms->cc_anchor = bms->cc;
	bms->anchored = true;
}

static int pm8921_bms_charger_property(enum power_supply_property psp)
{
	union power_supply_propval val;
	struct power_supply *usb;
	int ret;

	usb = power_supply_get_by_name("pm8921-usb");
	if (!usb)
		return -ENODEV;

	ret = power_supply_get_property(usb, psp, &val);
	power_supply_put(usb);

	return ret ? ret : val.intval;
}

static void pm8921_bms_update(struct pm8921_bms *bms)
{
	struct power_supply_battery_info *info = bms->batt_psy->battery_info;
	s64 charge, ocv;
	int volt_uv, curr_ua, percent, capacity;
	s32 cc;
	int ret;

	ret = pm8921_bms_sample(bms, &volt_uv, &curr_ua, &cc);
	if (ret)
		return;

	bms->volt_uv = volt_uv;
	bms->curr_ua = curr_ua;
	bms->cc = cc;

	if (!info)
		return;

	/*
	 * Open circuit voltage of the pack: the internal resistance drops
	 * the terminal voltage while discharging and raises it while
	 * charging.
	 */
	ocv = volt_uv - div_s64((s64)curr_ua *
				info->factory_internal_resistance_uohm,
				1000000);
	bms->ocv_uv = clamp_t(s64, ocv, 0, INT_MAX);

	percent = power_supply_batinfo_ocv2cap(info, bms->ocv_uv, 25);
	if (percent < 0)
		return;

	if (!bms->anchored) {
		if (bms->stored_capacity >= 0 &&
		    abs(bms->stored_capacity - percent) < BMS_STORED_SOC_TOLERANCE)
			pm8921_bms_anchor(bms, bms->stored_capacity);
		else
			pm8921_bms_anchor(bms, percent);
	} else if (pm8921_bms_charger_property(POWER_SUPPLY_PROP_STATUS) ==
		   POWER_SUPPLY_STATUS_FULL) {
		/* the charge is done, the counter is at the top */
		pm8921_bms_anchor(bms, 100);
	} else if (abs(curr_ua) < BMS_REST_CURRENT_UA &&
		   abs(bms->capacity - percent) > BMS_REANCHOR_PERCENT) {
		/* the pack is at rest, trust the voltage again */
		pm8921_bms_anchor(bms, percent);
	}

	charge = bms->charge_anchor_uah + pm8921_bms_cc_delta_uah(bms, cc);
	bms->charge_uah = charge;

	capacity = clamp(div_s64(charge * 100, info->charge_full_design_uah),
			 0, 100);
	if (capacity != bms->capacity) {
		bms->capacity = capacity;
		pm8921_bms_store_capacity(bms, capacity);
	}
}

static void pm8921_bms_work(struct work_struct *work)
{
	struct pm8921_bms *bms = container_of(to_delayed_work(work),
					      struct pm8921_bms, work);
	int prev = bms->capacity;

	pm8921_bms_update(bms);

	if (bms->capacity != prev)
		power_supply_changed(bms->batt_psy);

	queue_delayed_work(system_power_efficient_wq, &bms->work,
			   msecs_to_jiffies(BMS_POLL_INTERVAL_MS));
}

static enum power_supply_property pm8921_bms_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_SCOPE,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
};

static int pm8921_bms_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct pm8921_bms *bms = power_supply_get_drvdata(psy);
	struct power_supply_battery_info *info = psy->battery_info;
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
	case POWER_SUPPLY_PROP_HEALTH:
		/*
		 * The state machine of the charger knows whether a charge
		 * cycle runs and the temperature comparators of the PMIC
		 * define the health, so do not duplicate that here.
		 */
		ret = pm8921_bms_charger_property(psp);
		if (ret < 0)
			return ret;
		val->intval = ret;
		return 0;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		return 0;
	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_SYSTEM;
		return 0;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = bms->volt_uv;
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		val->intval = bms->ocv_uv;
		return 0;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = bms->curr_ua;
		return 0;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (bms->capacity < 0)
			return -ENODATA;
		val->intval = bms->capacity;
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		if (!info || !bms->anchored)
			return -ENODATA;
		val->intval = clamp_t(s64, bms->charge_uah, 0, INT_MAX);
		return 0;
	default:
		return -EINVAL;
	}
}

static const struct power_supply_desc pm8921_bms_desc = {
	.name			= "pm8921-bms",
	.type			= POWER_SUPPLY_TYPE_BATTERY,
	.properties		= pm8921_bms_props,
	.num_properties		= ARRAY_SIZE(pm8921_bms_props),
	.get_property		= pm8921_bms_get_property,
};

static int pm8921_bms_hw_init(struct pm8921_bms *bms)
{
	int ret;

	/* let the block run and keep the "power on" detect running */
	ret = regmap_update_bits(bms->map, PM8921_BMS_CONTROL,
				 BMS_EN | BMS_EN_PON_HS,
				 BMS_EN | BMS_EN_PON_HS);
	if (ret)
		return ret;

	return regmap_update_bits(bms->map, PM8921_BMS_TOLERANCES,
				  BMS_IBAT_TOL_MASK | BMS_OCV_TOL_MASK,
				  BMS_IBAT_TOL_DEFAULT | BMS_OCV_TOL_DEFAULT);
}

static int pm8921_bms_probe(struct platform_device *pdev)
{
	struct power_supply_config psy_cfg = {};
	struct pm8921_bms *bms;
	int ret;

	bms = devm_kzalloc(&pdev->dev, sizeof(*bms), GFP_KERNEL);
	if (!bms)
		return -ENOMEM;

	bms->dev = &pdev->dev;
	bms->capacity = -1;
	bms->stored_capacity = -EINVAL;
	mutex_init(&bms->lock);
	platform_set_drvdata(pdev, bms);

	bms->map = dev_get_regmap(pdev->dev.parent, NULL);
	if (!bms->map) {
		dev_err(&pdev->dev, "no regmap for the parent PMIC\n");
		return -ENODEV;
	}

	ret = device_property_read_u32(&pdev->dev, "qcom,r-sense-mohm",
				       &bms->r_sense_mohm);
	if (ret)
		bms->r_sense_mohm = 10;

	ret = pm8921_bms_hw_init(bms);
	if (ret)
		return ret;

	bms->stored_capacity = pm8921_bms_read_stored_capacity(bms);
	if (bms->stored_capacity >= 0)
		dev_info(&pdev->dev, "state of charge of the last session: %d%%\n",
			 bms->stored_capacity);

	psy_cfg.drv_data = bms;
	psy_cfg.fwnode = dev_fwnode(&pdev->dev);

	bms->batt_psy = devm_power_supply_register(&pdev->dev, &pm8921_bms_desc,
						   &psy_cfg);
	if (IS_ERR(bms->batt_psy))
		return dev_err_probe(&pdev->dev, PTR_ERR(bms->batt_psy),
				     "failed to register the battery\n");

	if (!bms->batt_psy->battery_info)
		dev_warn(&pdev->dev, "no battery description, no capacity\n");

	INIT_DELAYED_WORK(&bms->work, pm8921_bms_work);
	queue_delayed_work(system_power_efficient_wq, &bms->work, 0);

	dev_info(&pdev->dev, "sense %u mOhm\n", bms->r_sense_mohm);

	return 0;
}

static void pm8921_bms_remove(struct platform_device *pdev)
{
	struct pm8921_bms *bms = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&bms->work);
}

static int pm8921_bms_suspend(struct device *dev)
{
	struct pm8921_bms *bms = dev_get_drvdata(dev);

	cancel_delayed_work_sync(&bms->work);

	return 0;
}

static int pm8921_bms_resume(struct device *dev)
{
	struct pm8921_bms *bms = dev_get_drvdata(dev);

	queue_delayed_work(system_power_efficient_wq, &bms->work, 0);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(pm8921_bms_pm_ops, pm8921_bms_suspend,
				pm8921_bms_resume);

static const struct of_device_id pm8921_bms_of_match[] = {
	{ .compatible = "qcom,pm8921-bms" },
	{ }
};
MODULE_DEVICE_TABLE(of, pm8921_bms_of_match);

static struct platform_driver pm8921_bms_driver = {
	.probe = pm8921_bms_probe,
	.remove = pm8921_bms_remove,
	.driver = {
		.name = "pm8921-bms",
		.of_match_table = pm8921_bms_of_match,
		.pm = &pm8921_bms_pm_ops,
	},
};
module_platform_driver(pm8921_bms_driver);

MODULE_DESCRIPTION("Qualcomm PM8921 battery monitor");
MODULE_LICENSE("GPL");
