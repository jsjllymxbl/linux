// SPDX-License-Identifier: GPL-2.0-only
/* Qualcomm APQ8064 with WCD9310, using Apps-managed SLIMbus playback. */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <sound/jack.h>
#include <sound/soc.h>

#include "apq8064-slim.h"

struct apq8064_tabla {
	struct snd_soc_card card;
	struct snd_soc_dai_link link;
	struct snd_soc_dai_link_component cpu;
	struct snd_soc_dai_link_component codec;
	struct snd_soc_dai_link_component platform;
	struct gpio_desc *spk_en;
	struct gpio_desc *spk_en2;
	struct snd_soc_jack headphone_jack;
	struct snd_soc_jack_gpio headphone_gpio;
	struct snd_soc_dapm_route *routes;
	unsigned int num_routes;
};

static int apq8064_tabla_spk_event(struct snd_soc_dapm_widget *w,
				   struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_card *card = snd_soc_dapm_to_card(w->dapm);
	struct apq8064_tabla *priv = snd_soc_card_get_drvdata(card);
	int ret;

	ret = gpiod_set_value_cansleep(priv->spk_en,
				      event == SND_SOC_DAPM_PRE_PMU);
	if (ret)
		return ret;
	ret = gpiod_set_value_cansleep(priv->spk_en2,
				      event == SND_SOC_DAPM_PRE_PMU);
	if (ret) {
		gpiod_set_value_cansleep(priv->spk_en, 0);
		return ret;
	}
	usleep_range(4000, 4500);
	return 0;
}

static const struct snd_kcontrol_new apq8064_tabla_controls[] = {
	SOC_DAPM_PIN_SWITCH("Speaker"),
	SOC_DAPM_PIN_SWITCH("Earpiece"),
	SOC_DAPM_PIN_SWITCH("Headphone"),
};

static const struct snd_soc_dapm_widget apq8064_tabla_widgets[] = {
	/* OUT_DRV sequences the external amp after both codec lineout PAs. */
	SND_SOC_DAPM_OUT_DRV_E("Speaker Amplifier", SND_SOC_NOPM, 0, 0,
			       NULL, 0, apq8064_tabla_spk_event,
			       SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SPK("Speaker", NULL),
	SND_SOC_DAPM_SPK("Earpiece", NULL),
	SND_SOC_DAPM_HP("Headphone", NULL),
	/*
	 * Always-on sinks that stop at the codec DACs.  They pin the shared
	 * SLIMbus RX / interpolator / bias chain while UCM switches the real
	 * board outputs, without driving any speaker or headphone PA.
	 */
	SND_SOC_DAPM_SPK("RX1 Keepalive", NULL),
	SND_SOC_DAPM_SPK("RX2 Keepalive", NULL),
};

static int apq8064_tabla_parse_routes(struct apq8064_tabla *priv,
				      struct device *dev)
{
	static const char * const defaults[] = { "LINEOUT2", "LINEOUT4" };
	struct snd_soc_dapm_route *routes;
	unsigned int nsources = ARRAY_SIZE(defaults);
	unsigned int n = 0;
	int count, i, ret;

	count = of_property_count_strings(dev->of_node, "speaker-lineouts");
	if (count < 0 && count != -ENOENT)
		return dev_err_probe(dev, count,
				     "failed to count speaker-lineouts\n");
	if (count > 4)
		return dev_err_probe(dev, -EINVAL,
				     "too many speaker-lineouts\n");
	if (count > 0)
		nsources = count;

	routes = devm_kcalloc(dev, nsources + 6, sizeof(*routes), GFP_KERNEL);
	if (!routes)
		return -ENOMEM;

	for (i = 0; i < nsources; i++) {
		const char *source;

		if (count > 0) {
			ret = of_property_read_string_index(dev->of_node,
							    "speaker-lineouts",
							    i, &source);
			if (ret)
				return ret;
		} else {
			source = defaults[i];
		}

		routes[n++] = (struct snd_soc_dapm_route){
			.sink = "Speaker Amplifier",
			.source = source,
		};
	}

	routes[n++] = (struct snd_soc_dapm_route){
		.sink = "Speaker", .source = "Speaker Amplifier" };
	routes[n++] = (struct snd_soc_dapm_route){
		.sink = "Earpiece", .source = "EAR" };
	routes[n++] = (struct snd_soc_dapm_route){
		.sink = "Headphone", .source = "HPHL" };
	routes[n++] = (struct snd_soc_dapm_route){
		.sink = "Headphone", .source = "HPHR" };
	routes[n++] = (struct snd_soc_dapm_route){
		.sink = "RX1 Keepalive", .source = "HPHL DAC" };
	routes[n++] = (struct snd_soc_dapm_route){
		.sink = "RX2 Keepalive", .source = "HPHR DAC" };

	priv->routes = routes;
	priv->num_routes = n;
	return 0;
}

static int apq8064_tabla_link_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_card *card = rtd->card;
	struct snd_soc_dapm_context *dapm = snd_soc_card_to_dapm(card);
	struct apq8064_tabla *priv = snd_soc_card_get_drvdata(card);
	const unsigned int rx_slots[] = { 138, 139 };
	int ret;

	ret = snd_soc_dai_set_channel_map(snd_soc_rtd_to_codec(rtd, 0),
					 0, NULL, ARRAY_SIZE(rx_slots), rx_slots);
	if (ret)
		return ret;

	/* Pin the shared RX chain before selecting a board output. */
	ret = snd_soc_dapm_enable_pin(dapm, "RX1 Keepalive");
	if (ret)
		return ret;
	ret = snd_soc_dapm_enable_pin(dapm, "RX2 Keepalive");
	if (ret)
		return ret;

	/* Start with the rear speaker; other outputs are selected explicitly. */
	ret = snd_soc_dapm_enable_pin(dapm, "Speaker");
	if (ret)
		return ret;
	ret = snd_soc_dapm_disable_pin(dapm, "Earpiece");
	if (ret)
		return ret;
	ret = snd_soc_dapm_disable_pin(dapm, "Headphone");
	if (ret)
		return ret;

	if (!device_property_present(card->dev, "hp-det-gpios"))
		return 0;

	/*
	 * Report the jack state to userspace only.  UCM owns the output pins;
	 * the keepalive sinks above keep the SLIMbus RX chain powered across
	 * those switches, even while a stream is active.
	 */
	ret = snd_soc_card_jack_new(card, "Headphone Jack", SND_JACK_HEADPHONE,
				    &priv->headphone_jack);
	if (ret)
		return ret;

	priv->headphone_gpio.name = "hp-det";
	priv->headphone_gpio.report = SND_JACK_HEADPHONE;
	priv->headphone_gpio.debounce_time = 150;

	return snd_soc_jack_add_gpiods(card->dev, &priv->headphone_jack, 1,
				       &priv->headphone_gpio);
}

static void apq8064_tabla_link_exit(struct snd_soc_pcm_runtime *rtd)
{
	struct apq8064_tabla *priv = snd_soc_card_get_drvdata(rtd->card);

	/* Stop GPIO work before the card and its DAPM widgets are removed. */
	if (priv->headphone_gpio.jack) {
		snd_soc_jack_free_gpios(&priv->headphone_jack, 1,
					&priv->headphone_gpio);
		/*
		 * The jack core puts the descriptor but keeps the pointer, and
		 * would reuse it unrequested (active-low flag cleared) if the
		 * card is registered again, e.g. after a codec rebind.
		 */
		priv->headphone_gpio.desc = NULL;
	}
}

static void apq8064_tabla_put_codec_node(void *data)
{
	of_node_put(data);
}

static int apq8064_tabla_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct apq8064_tabla *priv;
	struct snd_soc_card *card;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->spk_en = devm_gpiod_get(dev, "speaker-en", GPIOD_OUT_LOW);
	if (IS_ERR(priv->spk_en))
		return dev_err_probe(dev, PTR_ERR(priv->spk_en),
				     "failed to get speaker enable GPIO\n");

	priv->spk_en2 = devm_gpiod_get_optional(dev, "speaker-bottom-en",
						 GPIOD_OUT_LOW);
	if (IS_ERR(priv->spk_en2))
		return dev_err_probe(dev, PTR_ERR(priv->spk_en2),
				     "failed to get bottom speaker enable GPIO\n");

	priv->codec.of_node = of_parse_phandle(dev->of_node, "codec", 0);
	if (!priv->codec.of_node)
		return -EINVAL;
	ret = devm_add_action_or_reset(dev, apq8064_tabla_put_codec_node,
				       priv->codec.of_node);
	if (ret)
		return ret;

	priv->cpu.of_node = dev->of_node;
	priv->cpu.dai_name = "apq8064-slim";
	priv->codec.dai_name = "tabla_rx1";
	priv->platform.of_node = dev->of_node;
	priv->link.name = "Tabla";
	priv->link.stream_name = "Tabla";
	priv->link.init = apq8064_tabla_link_init;
	priv->link.exit = apq8064_tabla_link_exit;
	priv->link.cpus = &priv->cpu;
	priv->link.num_cpus = 1;
	priv->link.codecs = &priv->codec;
	priv->link.num_codecs = 1;
	priv->link.platforms = &priv->platform;
	priv->link.num_platforms = 1;
	/* Codec port setup precedes host activation; shutdown reverses this. */
	priv->link.trigger_start = SND_SOC_TRIGGER_ORDER_LDC;
	priv->link.trigger_stop = SND_SOC_TRIGGER_ORDER_LDC;
	priv->link.nonatomic = true;

	card = &priv->card;
	card->dev = dev;
	card->owner = THIS_MODULE;
	/* Codec pins are intermediate nodes; only board outputs are sinks. */
	card->fully_routed = true;
	card->dai_link = &priv->link;
	card->num_links = 1;
	card->controls = apq8064_tabla_controls;
	card->num_controls = ARRAY_SIZE(apq8064_tabla_controls);
	card->dapm_widgets = apq8064_tabla_widgets;
	card->num_dapm_widgets = ARRAY_SIZE(apq8064_tabla_widgets);
	snd_soc_card_set_drvdata(card, priv);

	ret = apq8064_tabla_parse_routes(priv, dev);
	if (ret)
		return ret;
	card->dapm_routes = priv->routes;
	card->num_dapm_routes = priv->num_routes;

	ret = snd_soc_of_parse_card_name(card, "model");
	if (ret)
		return ret;
	ret = devm_snd_soc_register_component(dev, &apq8064_slim_component,
					      &apq8064_slim_dai, 1);
	if (ret)
		return ret;
	return devm_snd_soc_register_card(dev, card);
}

static const struct of_device_id apq8064_tabla_of_match[] = {
	{ .compatible = "qcom,apq8064-tabla-snd-card" },
	{ }
};
MODULE_DEVICE_TABLE(of, apq8064_tabla_of_match);

static struct platform_driver apq8064_tabla_driver = {
	.probe = apq8064_tabla_probe,
	.driver = {
		.name = "apq8064-tabla-snd-card",
		.of_match_table = apq8064_tabla_of_match,
	},
};
module_platform_driver(apq8064_tabla_driver);

MODULE_DESCRIPTION("Qualcomm APQ8064 with WCD9310 audio");
MODULE_LICENSE("GPL");
