// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) CET 2019
 *
 * Authors: Artsiom Asadchy
 */

#include <linux/gpio/consumer.h>
#include <linux/backlight.h>

#include <video/mipi_display.h>

#include <drm/drmP.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

struct jd9365 {
	struct device *dev;
	struct drm_panel panel;
	struct gpio_desc *reset_gpio;
	struct backlight_device *backlight;
	bool prepared;
	bool enabled;
};

static const struct drm_display_mode default_mode = {
	.clock = 68430,
	.hdisplay = 800,
	.hsync_start = 800 + 40,
	.hsync_end = 800 + 40 + 20,
	.htotal = 800 + 40 + 20 + 20,
	.vdisplay = 1280,
	.vsync_start = 1280 + 8,
	.vsync_end = 1280 + 8 + 4,
	.vtotal = 1280 + 8 + 4 + 8,
	.vrefresh = 60,
	.flags = 0,
	.width_mm = 135,
	.height_mm = 216,
};

static inline struct jd9365 *panel_to_jd9365(struct drm_panel *panel)
{
	return container_of(panel, struct jd9365, panel);
}

static void jd9365_dcs_write_cmd(struct jd9365 *ctx, u8 cmd)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	int err;

	err = mipi_dsi_dcs_write(dsi, cmd, NULL, 0);
	if (err < 0)
		DRM_ERROR_RATELIMITED("MIPI DSI DCS write failed: %d\n", err);
}

static void jd9365_dcs_write_single(struct jd9365 *ctx, u8 cmd, u8 value)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	int err;

	err = mipi_dsi_dcs_write(dsi, cmd, &value, 1);
	if (err < 0)
		DRM_ERROR_RATELIMITED("MIPI DSI DCS write failed: %d\n", err);
}

static void jd9365_init_sequence(struct jd9365 *ctx)
{
	/* Page0 */
	jd9365_dcs_write_single(ctx,0xE0,0x00);
	/* PASSWORD */
	jd9365_dcs_write_single(ctx,0xE1,0x93);
	jd9365_dcs_write_single(ctx,0xE2,0x65);
	jd9365_dcs_write_single(ctx,0xE3,0xF8);
	jd9365_dcs_write_single(ctx,0x80,0x03);

	/* Page1 */
	jd9365_dcs_write_single(ctx,0xE0,0x01);

	//Set VCOM
	jd9365_dcs_write_single(ctx,0x00,0x00);
	jd9365_dcs_write_single(ctx,0x01,0x6F);

	/* Set VCOM_Reverse */
	jd9365_dcs_write_single(ctx,0x03,0x00);
	jd9365_dcs_write_single(ctx,0x04,0x6A);

	/* Set Gamma Power, VGMP,VGMN,VGSP,VGSN */
	jd9365_dcs_write_single(ctx,0x17,0x00);
	jd9365_dcs_write_single(ctx,0x18,0xAF);
	jd9365_dcs_write_single(ctx,0x19,0x01);
	jd9365_dcs_write_single(ctx,0x1A,0x00);
	jd9365_dcs_write_single(ctx,0x1B,0xAF);
	jd9365_dcs_write_single(ctx,0x1C,0x01);

	/* Set Gate Power */
	jd9365_dcs_write_single(ctx,0x1F,0x3E);
	jd9365_dcs_write_single(ctx,0x20,0x28);
	jd9365_dcs_write_single(ctx,0x21,0x28);
	jd9365_dcs_write_single(ctx,0x22,0x7E);

	/* SETPANEL */
	jd9365_dcs_write_single(ctx,0x35,0x26);

	/* SETPANEL */
	jd9365_dcs_write_single(ctx,0x37,0x09);

	/* SET RGBCYC */
	jd9365_dcs_write_single(ctx,0x38,0x04);
	jd9365_dcs_write_single(ctx,0x39,0x00);
	jd9365_dcs_write_single(ctx,0x3A,0x01);
	jd9365_dcs_write_single(ctx,0x3C,0x7C);
	jd9365_dcs_write_single(ctx,0x3D,0xFF);
	jd9365_dcs_write_single(ctx,0x3E,0xFF);
	jd9365_dcs_write_single(ctx,0x3F,0x7F);

	/* Set TCON */
	jd9365_dcs_write_single(ctx,0x40,0x06);
	jd9365_dcs_write_single(ctx,0x41,0xA0);
	jd9365_dcs_write_single(ctx,0x42,0x81);
	jd9365_dcs_write_single(ctx,0x43,0x08);
	jd9365_dcs_write_single(ctx,0x44,0x0B);
	jd9365_dcs_write_single(ctx,0x45,0x28);

	/* power voltage */
	jd9365_dcs_write_single(ctx,0x55,0x01);
	jd9365_dcs_write_single(ctx,0x57,0x69);
	jd9365_dcs_write_single(ctx,0x59,0x0A);
	jd9365_dcs_write_single(ctx,0x5A,0x28);
	jd9365_dcs_write_single(ctx,0x5B,0x14);

	/* Gamma */
	jd9365_dcs_write_single(ctx,0x5D,0x7C);
	jd9365_dcs_write_single(ctx,0x5E,0x65);
	jd9365_dcs_write_single(ctx,0x5F,0x55);
	jd9365_dcs_write_single(ctx,0x60,0x47);
	jd9365_dcs_write_single(ctx,0x61,0x43);
	jd9365_dcs_write_single(ctx,0x62,0x32);
	jd9365_dcs_write_single(ctx,0x63,0x34);
	jd9365_dcs_write_single(ctx,0x64,0x1C);
	jd9365_dcs_write_single(ctx,0x65,0x33);
	jd9365_dcs_write_single(ctx,0x66,0x31);
	jd9365_dcs_write_single(ctx,0x67,0x30);
	jd9365_dcs_write_single(ctx,0x68,0x4E);
	jd9365_dcs_write_single(ctx,0x69,0x3C);
	jd9365_dcs_write_single(ctx,0x6A,0x44);
	jd9365_dcs_write_single(ctx,0x6B,0x35);
	jd9365_dcs_write_single(ctx,0x6C,0x31);
	jd9365_dcs_write_single(ctx,0x6D,0x23);
	jd9365_dcs_write_single(ctx,0x6E,0x11);
	jd9365_dcs_write_single(ctx,0x6F,0x00);
	jd9365_dcs_write_single(ctx,0x70,0x7C);
	jd9365_dcs_write_single(ctx,0x71,0x65);
	jd9365_dcs_write_single(ctx,0x72,0x55);
	jd9365_dcs_write_single(ctx,0x73,0x47);
	jd9365_dcs_write_single(ctx,0x74,0x43);
	jd9365_dcs_write_single(ctx,0x75,0x32);
	jd9365_dcs_write_single(ctx,0x76,0x34);
	jd9365_dcs_write_single(ctx,0x77,0x1C);
	jd9365_dcs_write_single(ctx,0x78,0x33);
	jd9365_dcs_write_single(ctx,0x79,0x31);
	jd9365_dcs_write_single(ctx,0x7A,0x30);
	jd9365_dcs_write_single(ctx,0x7B,0x4E);
	jd9365_dcs_write_single(ctx,0x7C,0x3C);
	jd9365_dcs_write_single(ctx,0x7D,0x44);
	jd9365_dcs_write_single(ctx,0x7E,0x35);
	jd9365_dcs_write_single(ctx,0x7F,0x31);
	jd9365_dcs_write_single(ctx,0x80,0x23);
	jd9365_dcs_write_single(ctx,0x81,0x11);
	jd9365_dcs_write_single(ctx,0x82,0x00);

	/* Page2, for GIP */
	jd9365_dcs_write_single(ctx,0xE0,0x02);

	/* GIP_L Pin mapping */
	jd9365_dcs_write_single(ctx,0x00,0x1E);
	jd9365_dcs_write_single(ctx,0x01,0x1E);
	jd9365_dcs_write_single(ctx,0x02,0x41);
	jd9365_dcs_write_single(ctx,0x03,0x41);
	jd9365_dcs_write_single(ctx,0x04,0x1F);
	jd9365_dcs_write_single(ctx,0x05,0x1F);
	jd9365_dcs_write_single(ctx,0x06,0x1F);
	jd9365_dcs_write_single(ctx,0x07,0x1F);
	jd9365_dcs_write_single(ctx,0x08,0x1F);
	jd9365_dcs_write_single(ctx,0x09,0x1F);
	jd9365_dcs_write_single(ctx,0x0A,0x1E);
	jd9365_dcs_write_single(ctx,0x0B,0x1E);
	jd9365_dcs_write_single(ctx,0x0C,0x1F);
	jd9365_dcs_write_single(ctx,0x0D,0x47);
	jd9365_dcs_write_single(ctx,0x0E,0x47);
	jd9365_dcs_write_single(ctx,0x0F,0x45);
	jd9365_dcs_write_single(ctx,0x10,0x45);
	jd9365_dcs_write_single(ctx,0x11,0x4B);
	jd9365_dcs_write_single(ctx,0x12,0x4B);
	jd9365_dcs_write_single(ctx,0x13,0x49);
	jd9365_dcs_write_single(ctx,0x14,0x49);
	jd9365_dcs_write_single(ctx,0x15,0x1F);

	/* GIP_R Pin mapping */
	jd9365_dcs_write_single(ctx,0x16,0x1E);
	jd9365_dcs_write_single(ctx,0x17,0x1E);
	jd9365_dcs_write_single(ctx,0x18,0x40);
	jd9365_dcs_write_single(ctx,0x19,0x40);
	jd9365_dcs_write_single(ctx,0x1A,0x1F);
	jd9365_dcs_write_single(ctx,0x1B,0x1F);
	jd9365_dcs_write_single(ctx,0x1C,0x1F);
	jd9365_dcs_write_single(ctx,0x1D,0x1F);
	jd9365_dcs_write_single(ctx,0x1E,0x1F);
	jd9365_dcs_write_single(ctx,0x1F,0x1F);
	jd9365_dcs_write_single(ctx,0x20,0x1E);
	jd9365_dcs_write_single(ctx,0x21,0x1E);
	jd9365_dcs_write_single(ctx,0x22,0x1f);
	jd9365_dcs_write_single(ctx,0x23,0x46);
	jd9365_dcs_write_single(ctx,0x24,0x46);
	jd9365_dcs_write_single(ctx,0x25,0x44);
	jd9365_dcs_write_single(ctx,0x26,0x44);
	jd9365_dcs_write_single(ctx,0x27,0x4A);
	jd9365_dcs_write_single(ctx,0x28,0x4A);
	jd9365_dcs_write_single(ctx,0x29,0x48);
	jd9365_dcs_write_single(ctx,0x2A,0x48);
	jd9365_dcs_write_single(ctx,0x2B,0x1F);

	/* GIP_L_GS Pin mapping */
	jd9365_dcs_write_single(ctx,0x2C,0x1F);
	jd9365_dcs_write_single(ctx,0x2D,0x1F);
	jd9365_dcs_write_single(ctx,0x2E,0x40);
	jd9365_dcs_write_single(ctx,0x2F,0x40);
	jd9365_dcs_write_single(ctx,0x30,0x1F);
	jd9365_dcs_write_single(ctx,0x31,0x1F);
	jd9365_dcs_write_single(ctx,0x32,0x1E);
	jd9365_dcs_write_single(ctx,0x33,0x1E);
	jd9365_dcs_write_single(ctx,0x34,0x1F);
	jd9365_dcs_write_single(ctx,0x35,0x1F);
	jd9365_dcs_write_single(ctx,0x36,0x1E);
	jd9365_dcs_write_single(ctx,0x37,0x1E);
	jd9365_dcs_write_single(ctx,0x38,0x1F);
	jd9365_dcs_write_single(ctx,0x39,0x48);
	jd9365_dcs_write_single(ctx,0x3A,0x48);
	jd9365_dcs_write_single(ctx,0x3B,0x4A);
	jd9365_dcs_write_single(ctx,0x3C,0x4A);
	jd9365_dcs_write_single(ctx,0x3D,0x44);
	jd9365_dcs_write_single(ctx,0x3E,0x44);
	jd9365_dcs_write_single(ctx,0x3F,0x46);
	jd9365_dcs_write_single(ctx,0x40,0x46);
	jd9365_dcs_write_single(ctx,0x41,0x1F);

	/* GIP_R_GS Pin mapping */
	jd9365_dcs_write_single(ctx,0x42,0x1F);
	jd9365_dcs_write_single(ctx,0x43,0x1F);
	jd9365_dcs_write_single(ctx,0x44,0x41);
	jd9365_dcs_write_single(ctx,0x45,0x41);
	jd9365_dcs_write_single(ctx,0x46,0x1F);
	jd9365_dcs_write_single(ctx,0x47,0x1F);
	jd9365_dcs_write_single(ctx,0x48,0x1E);
	jd9365_dcs_write_single(ctx,0x49,0x1E);
	jd9365_dcs_write_single(ctx,0x4A,0x1E);
	jd9365_dcs_write_single(ctx,0x4B,0x1F);
	jd9365_dcs_write_single(ctx,0x4C,0x1E);
	jd9365_dcs_write_single(ctx,0x4D,0x1E);
	jd9365_dcs_write_single(ctx,0x4E,0x1F);
	jd9365_dcs_write_single(ctx,0x4F,0x49);
	jd9365_dcs_write_single(ctx,0x50,0x49);
	jd9365_dcs_write_single(ctx,0x51,0x4B);
	jd9365_dcs_write_single(ctx,0x52,0x4B);
	jd9365_dcs_write_single(ctx,0x53,0x45);
	jd9365_dcs_write_single(ctx,0x54,0x45);
	jd9365_dcs_write_single(ctx,0x55,0x47);
	jd9365_dcs_write_single(ctx,0x56,0x47);
	jd9365_dcs_write_single(ctx,0x57,0x1F);

	/* GIP Timing */
	jd9365_dcs_write_single(ctx,0x58,0x40);
	jd9365_dcs_write_single(ctx,0x5B,0x30);
	jd9365_dcs_write_single(ctx,0x5C,0x03);
	jd9365_dcs_write_single(ctx,0x5D,0x30);
	jd9365_dcs_write_single(ctx,0x5E,0x01);
	jd9365_dcs_write_single(ctx,0x5F,0x02);
	jd9365_dcs_write_single(ctx,0x63,0x14);
	jd9365_dcs_write_single(ctx,0x64,0x6A);
	jd9365_dcs_write_single(ctx,0x67,0x73);
	jd9365_dcs_write_single(ctx,0x68,0x05);
	jd9365_dcs_write_single(ctx,0x69,0x14);
	jd9365_dcs_write_single(ctx,0x6A,0x6A);
	jd9365_dcs_write_single(ctx,0x6B,0x08);

	jd9365_dcs_write_single(ctx,0x6C,0x00);
	jd9365_dcs_write_single(ctx,0x6D,0x00);
	jd9365_dcs_write_single(ctx,0x6E,0x00);
	jd9365_dcs_write_single(ctx,0x6F,0x88);

	jd9365_dcs_write_single(ctx,0x77,0xDD);
	jd9365_dcs_write_single(ctx,0x79,0x0E);
	jd9365_dcs_write_single(ctx,0x7A,0x03);
	jd9365_dcs_write_single(ctx,0x7D,0x14);
	jd9365_dcs_write_single(ctx,0x7E,0x6A);

	/* Page4 */
	jd9365_dcs_write_single(ctx,0xE0,0x04);
	jd9365_dcs_write_single(ctx,0x09,0x11);
	jd9365_dcs_write_single(ctx,0x0E,0x48);
	jd9365_dcs_write_single(ctx,0x2B,0x2B);
	jd9365_dcs_write_single(ctx,0x2D,0x03);
	jd9365_dcs_write_single(ctx,0x2E,0x44);

	/* Page0 */
	jd9365_dcs_write_single(ctx,0xE0,0x00);
	jd9365_dcs_write_single(ctx,0xE6,0x02);
	jd9365_dcs_write_single(ctx,0xE7,0x0C);

	/* SLP OUT */
	jd9365_dcs_write_cmd(ctx,0x11);
	msleep(120);

	/* DISP ON */
	jd9365_dcs_write_cmd(ctx,0x29);
	msleep(5);
}

static int jd9365_disable(struct drm_panel *panel)
{
	struct jd9365 *ctx = panel_to_jd9365(panel);

	if (!ctx->enabled)
		return 0;

	backlight_disable(ctx->backlight);

	ctx->enabled = false;

	return 0;
}

static int jd9365_enable(struct drm_panel *panel)
{
	struct jd9365 *ctx = panel_to_jd9365(panel);

	if (ctx->enabled)
		return 0;

	backlight_enable(ctx->backlight);

	ctx->enabled = true;

	return 0;
}

static int jd9365_unprepare(struct drm_panel *panel)
{
	struct jd9365 *ctx = panel_to_jd9365(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	int ret;

	if (!ctx->prepared)
		return 0;

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret)
		DRM_WARN("failed to set display off: %d\n", ret);

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret)
		DRM_WARN("failed to enter sleep mode: %d\n", ret);

	msleep(120);

	if (ctx->reset_gpio) {
		gpiod_set_value_cansleep(ctx->reset_gpio, 0);
		msleep(20);
	}

	ctx->prepared = false;

	return 0;
}

static int jd9365_prepare(struct drm_panel *panel)
{
	struct jd9365 *ctx = panel_to_jd9365(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	int ret;

	if (ctx->prepared)
		return 0;

	if (ctx->reset_gpio) {
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		msleep(10);
		gpiod_set_value_cansleep(ctx->reset_gpio, 0);
		msleep(10);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		msleep(120);
	}

	jd9365_init_sequence(ctx);

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret)
		return ret;

	msleep(125);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret)
		return ret;

	msleep(20);

	ctx->prepared = true;

	pr_info("JD9365 Panel prepare is finished\n");

	return 0;
}

static int jd9365_get_modes(struct drm_panel *panel)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(panel->drm, &default_mode);
	if (!mode) {
		DRM_ERROR("failed to add mode %ux%ux@%u\n",
			  default_mode.hdisplay, default_mode.vdisplay,
			  default_mode.vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(panel->connector, mode);

	panel->connector->display_info.width_mm = mode->width_mm;
	panel->connector->display_info.height_mm = mode->height_mm;

	return 1;
}

static const struct drm_panel_funcs jd9365_drm_funcs = {
	.disable = jd9365_disable,
	.unprepare = jd9365_unprepare,
	.prepare = jd9365_prepare,
	.enable = jd9365_enable,
	.get_modes = jd9365_get_modes,
};

static int jd9365_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct jd9365 *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpio)) {
		ret = PTR_ERR(ctx->reset_gpio);
		dev_err(dev, "cannot get reset GPIO: %d\n", ret);
		return ret;
	}

	ctx->backlight = devm_of_find_backlight(dev);
	if (IS_ERR(ctx->backlight))
		return PTR_ERR(ctx->backlight);

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
			  MIPI_DSI_MODE_LPM;

	drm_panel_init(&ctx->panel);
	ctx->panel.dev = dev;
	ctx->panel.funcs = &jd9365_drm_funcs;

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "mipi_dsi_attach() failed: %d\n", ret);
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	pr_info("JD9365 Panel was initialized successfully!\n");

	return 0;
}

static int jd9365_remove(struct mipi_dsi_device *dsi)
{
	struct jd9365 *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);

	pr_info("JD9365 Panel was removed!\n");

	return 0;
}

static const struct of_device_id fiti_jd9365_of_match[] = {
	{ .compatible = "fiti,jd9365" },
	{ }
};
MODULE_DEVICE_TABLE(of, fiti_jd9365_of_match);

static struct mipi_dsi_driver fiti_jd9365_driver = {
	.probe = jd9365_probe,
	.remove = jd9365_remove,
	.driver = {
		.name = "panel-fiti-jd9365",
		.of_match_table = fiti_jd9365_of_match,
	},
};
module_mipi_dsi_driver(fiti_jd9365_driver);

MODULE_AUTHOR("Artsiom Asadchy");
MODULE_DESCRIPTION("DRM Driver for Fiti JD9365 MIPI DSI panel");
MODULE_LICENSE("GPL v2");
