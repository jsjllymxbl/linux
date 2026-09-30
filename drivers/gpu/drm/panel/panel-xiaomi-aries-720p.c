// SPDX-License-Identifier: GPL-2.0-only
/*
 * DRM panel driver for the 4.3" 720x1280 DSI panel of the Xiaomi Mi 2
 * ("aries"), driven by a Renesas IC of the MCAP family (vendor page switch
 * 0xB0; exact part number TBD, final naming and binding doc come at
 * upstreaming time).
 *
 * Downstream reference:
 *   - arch/arm/mach-msm/board-aries-display.c mipi_dsi_mipanel_power():
 *     power rails / reset timing
 *   - drivers/video/msm/mipi_renesas.c: the Hitachi/JDI command stream
 *     (renesas_hitachi_on_cmds[]) and renesas_videomode_on_cmds[]
 *
 * NOTE ON THE DSI VIDEO ENGINE: the msm DSI bridge starts the video engine
 * in its pre_enable hook, i.e. before the MDP has started feeding pixel
 * data, and on APQ8064 the engine then never completes a frame by itself
 * (no VIDEO_DONE irq).  A controller soft reset after the encoder enable --
 * i.e. with the MDP already streaming -- makes it start up cleanly, so the
 * panel driver performs a dsi_sw_reset() in its .enable() callback.  This
 * is required every time the display pipeline is (re)enabled, including
 * system resume.
 *
 * NOTE ON THE DSI MODE: downstream drives this panel in command mode, but
 * the mainline MDP4 driver only implements DSI video mode (mdp4 always
 * selects INTF_DSI_VIDEO and has no command mode CRTC).  The Renesas IC
 * supports both, so the init stream is the downstream command mode one plus
 * the downstream "enter video mode" sequence (MCAP 0xB3 0x40) at the end.
 *
 * NOTE ON THE PANEL IMAGE SETTINGS: the CE (0xCA) image enhancement block
 * is restored from the downstream command mode stream (the stock defconfig
 * ships with CONFIG_FB_MSM_MIPI_DSI_CE=y).  It is only valid right after
 * sleep out, wrapped in the MCAP page switch, with the exact mode byte
 * used below; writing 0xCA with a mode byte the IC does not support
 * gradually fades the panel to black.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

enum aries_720p_cmd_type {
	ARIES_720P_GENERIC,	/* generic write */
	ARIES_720P_DCS,		/* DCS write */
};

struct aries_720p_cmd {
	enum aries_720p_cmd_type type;
	u16 delay_ms;
	u8 len;
	u8 data[39];
};

/*
 * Panel init sequence, the downstream renesas_hitachi_on_cmds[] (the
 * APQ8064_MTP / "LCD_ID high" variant with CONFIG_FB_MSM_MIPI_DSI_CE=y,
 * CE_HITACHI_MODE3), sent entry by entry including the delays.
 *
 * The CE (0xCA) image enhancement block is only valid right after sleep
 * out, wrapped in the MCAP page switch, with the exact mode byte 0x01
 * below; writing 0xCA with any other mode byte gradually fades the panel
 * to black.
 *
 * The NVM/GIP configuration of the Hitachi panel variant matches the
 * downstream Hitachi 720p frame structure (966x1295 total), so this stream
 * must be combined with the APQ8064_MTP display timing below.  Running it
 * with the LGD/Renesas video porches (739x1303) displays a picture, but
 * with horizontal ghosting and constant frame jitter.
 */
static const struct aries_720p_cmd aries_720p_init_cmds[] = {
	{ ARIES_720P_DCS, 120, 1, { 0x11 } },		/* sleep out */
	{ ARIES_720P_GENERIC, 0, 2, { 0xB0, 0x04 } },	/* MCAP start */
	{ ARIES_720P_GENERIC, 0, 33, {
	  0xCA, 0x01, 0x80, 0x88, 0x8C, 0xBC, 0x8C, 0x8C,
	  0x8C, 0x18, 0x3F, 0x14, 0xFF, 0x0A, 0x4A, 0x37,
	  0xA0, 0x55, 0xF8, 0x0C, 0x0C, 0x20, 0x10, 0x3F,
	  0x3F, 0x00, 0x00, 0x10, 0x10, 0x3F, 0x3F, 0x3F,
	  0x3F } },					/* CE mode 3 */
	{ ARIES_720P_GENERIC, 0, 2, { 0xB0, 0x03 } },	/* MCAP end */
	{ ARIES_720P_DCS, 0, 3, { 0x51, 0x0E, 0xFF } },	/* brightness */
	{ ARIES_720P_DCS, 0, 2, { 0x53, 0x2C } },		/* ctrl display */
	{ ARIES_720P_DCS, 0, 2, { 0x55, 0x01 } },		/* CABC UI */
	{ ARIES_720P_DCS, 0, 5, { 0x2A, 0x00, 0x00, 0x02, 0xCF } },
	{ ARIES_720P_DCS, 20, 5, { 0x2B, 0x00, 0x00, 0x04, 0xFF } },
	{ ARIES_720P_DCS, 20, 2, { 0x36, 0x00 } },	/* address mode */
	{ ARIES_720P_DCS, 0, 2, { 0x3A, 0x77 } },		/* RGB888 */
	{ ARIES_720P_DCS, 20, 1, { 0x29 } },		/* display on */
};

/*
 * Switch the Renesas IC from DSI command mode to DSI video mode, the
 * downstream renesas_videomode_on_cmds[] (mcap_start = 0xB0 0x04,
 * set_interface_video = 0xB3 0x40, mcap_end = 0xB0 0x03, tear on).
 */
static const struct aries_720p_cmd aries_720p_video_mode_cmds[] = {
	{ ARIES_720P_GENERIC, 0, 2, { 0xB0, 0x04 } },	/* MCAP start */
	{ ARIES_720P_GENERIC, 0, 2, { 0xB3, 0x40 } },	/* enter video mode */
	{ ARIES_720P_GENERIC, 0, 2, { 0xB0, 0x03 } },	/* MCAP end */
	{ ARIES_720P_DCS, 0, 2, { 0x35, 0x00 } },	/* tear on (VBLANK) */
};

/*
 * Display timing, the downstream APQ8064_MTP profile (mipi_hitachi_cmd_720p_pt.c
 * / mipi_sharp_video_720p_pt.c): hbp=14 hfp=220 hsw=12, vbp=7 vfp=7 vsw=1,
 * 75 MHz pclk -> 450 Mbps/lane.  Note that the back/front porch assignment
 * follows the downstream panel data (h_back_porch=14, h_front_porch=220):
 * the large blanking interval sits in front of the active area, not behind
 * it.
 */
static const struct drm_display_mode aries_720p_mode = {
	.clock = (720 + 220 + 12 + 14) * (1280 + 7 + 1 + 7) * 60 / 1000,
	.hdisplay = 720,
	.hsync_start = 720 + 220,
	.hsync_end = 720 + 220 + 12,
	.htotal = 720 + 220 + 12 + 14,
	.vdisplay = 1280,
	.vsync_start = 1280 + 7,
	.vsync_end = 1280 + 7 + 1,
	.vtotal = 1280 + 7 + 1 + 7,
	.width_mm = 54,
	.height_mm = 96,
};

struct aries_720p {
	struct drm_panel panel;
	struct device *dev;
	struct mipi_dsi_device *dsi;

	struct regulator *vddio;	/* PM8921 L23, 1.8 V panel IO */
	struct regulator *vsp;		/* EXT_5P4V on-board boost, 5.4 V */

	struct gpio_desc *reset_gpio;	/* PM8921 GPIO25, DISP_RESET_N, active low */
	struct gpio_desc *lcd_id_gpio;	/* PM8921 GPIO12, LCD_ID_DET */

	struct mutex mutex; /* protects ->prepared */
	bool prepared;

	/*
	 * DSI controller registers, used by the video engine soft reset
	 * workaround in .enable() (see the note at the top of the file).
	 */
	void __iomem *dsi_base;
};

static inline struct aries_720p *to_aries_720p(struct drm_panel *panel)
{
	return container_of(panel, struct aries_720p, panel);
}

static int aries_720p_send_cmds(struct aries_720p *ctx,
				const struct aries_720p_cmd *cmds,
				unsigned int ncmds)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	unsigned int i;
	int ret;

	for (i = 0; i < ncmds; i++) {
		const struct aries_720p_cmd *cmd = &cmds[i];

		if (cmd->type == ARIES_720P_DCS) {
			if (cmd->len == 1)
				ret = mipi_dsi_dcs_write(dsi, cmd->data[0],
							 NULL, 0);
			else
				ret = mipi_dsi_dcs_write(dsi, cmd->data[0],
							 &cmd->data[1],
							 cmd->len - 1);
		} else {
			ret = mipi_dsi_generic_write(dsi, cmd->data, cmd->len);
		}

		if (ret < 0) {
			dev_err(ctx->dev,
				"failed to send command %u (0x%02x): %d\n",
				i, cmd->data[0], ret);
			return ret;
		}

		if (cmd->delay_ms)
			msleep(cmd->delay_ms);
	}

	return 0;
}

/*
 * Power-on, downstream board-aries-display.c mipi_dsi_mipanel_power(on=1):
 * L23 -> 1 ms -> VSP -> 10 ms -> reset high -> 3 ms -> read LCD_ID.  The
 * L2/LVS7/L11 DSI host rails belong to the dsi0 node and are enabled by the
 * DSI host before the panel is prepared.
 *
 * The whole init stream is sent here and not in .enable(): the msm DSI
 * bridge starts the video engine in its pre_enable hook, i.e. before the
 * panel is prepared, so the panel has to be out of sleep and in video mode
 * by the time prepare() returns.
 */
static int aries_720p_prepare(struct drm_panel *panel)
{
	struct aries_720p *ctx = to_aries_720p(panel);
	int ret;

	mutex_lock(&ctx->mutex);
	if (ctx->prepared) {
		mutex_unlock(&ctx->mutex);
		return 0;
	}

	ret = regulator_enable(ctx->vddio);
	if (ret < 0) {
		dev_err(ctx->dev, "failed to enable vddio: %d\n", ret);
		goto out;
	}
	usleep_range(1000, 2000);

	ret = regulator_enable(ctx->vsp);
	if (ret < 0) {
		dev_err(ctx->dev, "failed to enable vsp: %d\n", ret);
		regulator_disable(ctx->vddio);
		goto out;
	}
	usleep_range(10000, 11000);

	/* deassert reset: DISP_RESET_N physical high */
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(3000, 4000);

	ret = gpiod_get_value_cansleep(ctx->lcd_id_gpio);
	dev_info(ctx->dev, "LCD_ID (PM8921 GPIO12) = %d\n", ret);

	ret = aries_720p_send_cmds(ctx, aries_720p_init_cmds,
				   ARRAY_SIZE(aries_720p_init_cmds));
	if (ret)
		goto err_poweroff;

	ret = aries_720p_send_cmds(ctx,
				   aries_720p_video_mode_cmds,
				   ARRAY_SIZE(aries_720p_video_mode_cmds));
	if (ret)
		goto err_poweroff;

	dev_info(ctx->dev, "panel initialized, DSI video mode\n");

	ctx->prepared = true;
	ret = 0;
out:
	mutex_unlock(&ctx->mutex);
	return ret;

err_poweroff:
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_disable(ctx->vsp);
	usleep_range(10000, 11000);
	regulator_disable(ctx->vddio);
	goto out;
}

/*
 * Power-off, downstream mipi_dsi_mipanel_power(on=0):
 * reset low -> VSP off -> 10 ms -> LVS7 (dsi0 side) -> L23 -> L2.
 */
static int aries_720p_unprepare(struct drm_panel *panel)
{
	struct aries_720p *ctx = to_aries_720p(panel);

	mutex_lock(&ctx->mutex);
	if (!ctx->prepared) {
		mutex_unlock(&ctx->mutex);
		return 0;
	}

	/* assert reset: DISP_RESET_N physical low */
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);

	regulator_disable(ctx->vsp);
	usleep_range(10000, 11000);

	regulator_disable(ctx->vddio);

	ctx->prepared = false;
	mutex_unlock(&ctx->mutex);

	return 0;
}

/*
 * The msm DSI bridge enables the video engine in its pre_enable hook, i.e.
 * before the MDP has started feeding it pixel data, and on APQ8064 the
 * engine then never completes a frame (no VIDEO_DONE irq, the panel stays
 * black) even though all visible controller state looks fine.  A controller
 * soft reset at this point -- after the encoder enable, i.e. with the MDP
 * already streaming -- makes it start up cleanly.
 *
 * This replicates msm dsi_sw_reset() on the DSI controller registers.
 */
static void aries_720p_dsi_sw_reset(struct aries_720p *ctx)
{
	void __iomem *b = ctx->dsi_base;
	const u32 clk_ctrl = 0x23f;	/* DSI_CLK_CTRL_ENABLE_CLKS */
	u32 ctrl;

	ctrl = readl(b + 0x000);	/* REG_DSI_CTRL */
	if (ctrl & 0x1)			/* DSI_CTRL_ENABLE */
		writel(ctrl & ~0x1, b + 0x000);

	writel(clk_ctrl, b + 0x118);	/* REG_DSI_CLK_CTRL */
	/* make sure the clock request has reached the hardware before the reset */
	wmb();

	writel(1, b + 0x114);		/* REG_DSI_RESET */
	msleep(20);
	writel(0, b + 0x114);
	/* order the reset deassert before re-enabling the controller */
	wmb();

	writel(ctrl, b + 0x000);	/* enable again */
}

static int aries_720p_enable(struct drm_panel *panel)
{
	struct aries_720p *ctx = to_aries_720p(panel);

	/*
	 * The panel itself is switched on by prepare(); this runs after the
	 * encoder enable, so it is the first point where the MDP is feeding
	 * the DSI video engine.
	 */
	aries_720p_dsi_sw_reset(ctx);

	return 0;
}

/*
 * Deliberately empty.  disable() runs while the DSI video engine is still
 * streaming, and any command write at that point wedges the APQ8064
 * command engine -- which hangs the entire (pre)suspend path.  The panel
 * is switched off by the power cycle in unprepare(), which resets the IC
 * out of video mode anyway.
 */
static int aries_720p_disable(struct drm_panel *panel)
{
	return 0;
}

static int aries_720p_get_modes(struct drm_panel *panel,
				struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &aries_720p_mode);
	if (!mode)
		return -ENOMEM;

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs aries_720p_panel_funcs = {
	.prepare = aries_720p_prepare,
	.unprepare = aries_720p_unprepare,
	.enable = aries_720p_enable,
	.disable = aries_720p_disable,
	.get_modes = aries_720p_get_modes,
};

static int aries_720p_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct aries_720p *ctx;
	struct resource *res;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct aries_720p, panel,
				   &aries_720p_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->dev = dev;
	ctx->dsi = dsi;
	mutex_init(&ctx->mutex);

	ctx->vddio = devm_regulator_get(dev, "vddio");
	if (IS_ERR(ctx->vddio))
		return dev_err_probe(dev, PTR_ERR(ctx->vddio),
				     "failed to get vddio regulator\n");

	ctx->vsp = devm_regulator_get(dev, "vsp");
	if (IS_ERR(ctx->vsp))
		return dev_err_probe(dev, PTR_ERR(ctx->vsp),
				     "failed to get vsp regulator\n");

	/* logical high == physical low (active low) == reset asserted */
	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "failed to get reset-gpios\n");

	ctx->lcd_id_gpio = devm_gpiod_get(dev, "lcd-id", GPIOD_IN);
	if (IS_ERR(ctx->lcd_id_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->lcd_id_gpio),
				     "failed to get lcd-id-gpios\n");

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	/*
	 * Video mode, burst mode traffic with TX EoT and a continuous clock
	 * lane, matching the downstream link profile this panel has been
	 * verified with.  The init stream is sent in low power mode.
	 */
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO |
			  MIPI_DSI_MODE_VIDEO_BURST |
			  MIPI_DSI_MODE_LPM;

	/*
	 * The init stream has to go out over DSI, so the DSI host must be
	 * powered up (PHY up, lanes at LP-11) before prepare() runs.  The
	 * bridge chain calls pre_enable in reverse order, i.e. the panel
	 * would be prepared before the DSI host otherwise and every transfer
	 * would fail with -EINVAL.
	 */
	ctx->panel.prepare_prev_first = true;

	/*
	 * Map the DSI controller registers for the video engine soft reset
	 * workaround in .enable() (see the note at the top of the file).
	 */
	res = platform_get_resource(to_platform_device(dsi->host->dev),
				    IORESOURCE_MEM, 0);
	ctx->dsi_base = devm_ioremap_resource(&dsi->dev, res);
	if (IS_ERR(ctx->dsi_base))
		return PTR_ERR(ctx->dsi_base);

	mipi_dsi_set_drvdata(dsi, ctx);

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	return 0;
}

static void aries_720p_remove(struct mipi_dsi_device *dsi)
{
	struct aries_720p *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id aries_720p_of_match[] = {
	{ .compatible = "xiaomi,aries-720p-panel" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, aries_720p_of_match);

static struct mipi_dsi_driver aries_720p_driver = {
	.probe = aries_720p_probe,
	.remove = aries_720p_remove,
	.driver = {
		.name = "panel-xiaomi-aries-720p",
		.of_match_table = aries_720p_of_match,
	},
};
module_mipi_dsi_driver(aries_720p_driver);

MODULE_DESCRIPTION("DRM panel driver for the Renesas MCAP 720x1280 panel in Xiaomi Mi 2");
MODULE_LICENSE("GPL");
