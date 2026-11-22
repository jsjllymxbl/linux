// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm PM8921 (PMIC4) battery monitor driver
 *
 * The PM8921 integrates a "BMS" (battery monitor system): a coulomb
 * counter with its own ADC that continuously measures the battery
 * voltage, the voltage across the current sense resistor and the charge
 * that went in or out of the pack.  The values are published through a
 * small register window (BMS_CONTROL / BMS_OUTPUT0-1) where software
 * selects which quantity it wants to read.
 *
 * Register layout and the encodings of the values are taken from the
 * downstream pm8921-bms driver of the Qualcomm msm8960 kernel and cross
 * checked against the Xiaomi Mi 2 (aries) board file, which uses a 10
 * mOhm sense resistor and the "Samsung 2000" style battery profile.
 *
 * Copyright (c) 2026 jsjllymxbl <jsjllymxbl@gmail.com>
 */

#include <linux/delay.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>

#include "adc-battery-helper.h"

#define PM8921_BMS_CONTROL		0x224
#define PM8921_BMS_S1_DELAY		0x225
#define PM8921_BMS_OUTPUT0		0x230
#define PM8921_BMS_OUTPUT1		0x231
#define PM8921_BMS_TOLERANCES		0x232

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
#define BMS_SELECT_OCV			0
#define BMS_SELECT_VSENSE_AVG		6
#define BMS_SELECT_VBATT_AVG		7

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

#define BMS_FORCE_MEASURE_DELAY_MS	20

/**
 * struct pm8921_bms - battery monitor instance
 * @dev:	device pointer
 * @map:	regmap of the parent PMIC
 * @lock:	serialises register accesses
 * @r_sense_mohm:	value of the current sense resistor
 * @batt_psy:	registered battery power supply
 * @volt_uv:	last sampled battery voltage
 * @curr_ua:	last sampled battery current, positive while charging
 */
struct pm8921_bms {
	/* must be the first member: the battery helper casts the drvdata */
	struct adc_battery_helper helper;

	struct device		*dev;
	struct regmap		*map;
	struct mutex		lock;
	unsigned int		r_sense_mohm;

	struct power_supply	*batt_psy;

	int			volt_uv;
	int			curr_ua;
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
 * Ask the BMS for a fresh pair of readings.  In its normal mode the
 * block only refreshes its output registers when the battery current or
 * voltage moves further than the configured tolerances, so a busy
 * system can otherwise read the same sample for minutes.  "BMS_MODE"
 * together with "EN_VBAT" makes it measure both quantities at once and
 * the caller has to hold the output registers while reading them.
 */
static int pm8921_bms_sample(struct pm8921_bms *bms, int *volt_uv,
			     int *curr_ua)
{
	u16 vbat_raw;
	s16 vsense_raw;
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

	ret = pm8921_bms_read_output(bms, BMS_SELECT_VSENSE_AVG,
				     (u16 *)&vsense_raw);
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

	uv = div_s64((s64)vsense_raw * BMS_VSENSE_UV_MUL, BMS_VSENSE_UV_DIV);
	curr = div_s64(uv * 1000, bms->r_sense_mohm);
	/* the sense voltage is positive while the pack is being discharged */
	*curr_ua = -curr;

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

static enum power_supply_property pm8921_bms_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_SCOPE,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
};

/*
 * The charge cycle, the battery presence and the temperature comparators
 * are tracked by the charger, so the state of the battery is read from
 * its USB power supply instead of being duplicated here.
 */
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

static int pm8921_bms_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		return 0;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = pm8921_bms_charger_property(psp);
		if (ret < 0)
			break;
		val->intval = ret;
		return 0;
	default:
		break;
	}

	return adc_battery_helper_get_property(psy, psp, val);
}

/* called by the battery helper whenever it needs a fresh sample */
static int pm8921_bms_get_voltage_and_current(struct power_supply *psy,
					      int *volt_uv, int *curr_ua)
{
	struct pm8921_bms *bms = power_supply_get_drvdata(psy);

	return pm8921_bms_sample(bms, volt_uv, curr_ua);
}

static const struct power_supply_desc pm8921_bms_desc = {
	.name			= "pm8921-bms",
	.type			= POWER_SUPPLY_TYPE_BATTERY,
	.properties		= pm8921_bms_props,
	.num_properties		= ARRAY_SIZE(pm8921_bms_props),
	.get_property		= pm8921_bms_get_property,
	.external_power_changed	= adc_battery_helper_external_power_changed,
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

	psy_cfg.drv_data = bms;
	psy_cfg.fwnode = dev_fwnode(&pdev->dev);

	bms->batt_psy = devm_power_supply_register(&pdev->dev, &pm8921_bms_desc,
						   &psy_cfg);
	if (IS_ERR(bms->batt_psy))
		return dev_err_probe(&pdev->dev, PTR_ERR(bms->batt_psy),
				     "failed to register the battery\n");

	ret = adc_battery_helper_init(&bms->helper, bms->batt_psy,
				      pm8921_bms_get_voltage_and_current, NULL);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to start the battery helper\n");

	dev_info(&pdev->dev, "sense %u mOhm, %d%% (%d uV, %d uA)\n",
		 bms->r_sense_mohm, bms->helper.capacity, bms->helper.volt_uv,
		 bms->helper.curr_ua);

	return 0;
}

static const struct of_device_id pm8921_bms_of_match[] = {
	{ .compatible = "qcom,pm8921-bms" },
	{ }
};
MODULE_DEVICE_TABLE(of, pm8921_bms_of_match);

static const struct dev_pm_ops pm8921_bms_pm_ops = {
	.suspend = adc_battery_helper_suspend,
	.resume = adc_battery_helper_resume,
};

static struct platform_driver pm8921_bms_driver = {
	.probe = pm8921_bms_probe,
	.driver = {
		.name = "pm8921-bms",
		.of_match_table = pm8921_bms_of_match,
		.pm = &pm8921_bms_pm_ops,
	},
};
module_platform_driver(pm8921_bms_driver);

MODULE_DESCRIPTION("Qualcomm PM8921 battery monitor");
MODULE_LICENSE("GPL");
