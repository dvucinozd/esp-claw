/*
 * SPDX-FileCopyrightText: 2026 Guiton
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include "esp_log.h"
#include "dev_display_lcd.h"
#include "esp_lcd_jd9365_10_1.h"
#include "esp_lcd_touch_gt911.h"

static const char *TAG = "JC_ESP32P4_M3_SETUP";

esp_err_t lcd_dsi_panel_factory_entry_t(esp_lcd_dsi_bus_handle_t dsi_handle, dev_display_lcd_config_t *lcd_cfg, dev_display_lcd_handles_t *lcd_handles)
{
    // Board-manager calls this factory to instantiate the concrete JD9365 panel object.
    // We map generic display_lcd config from YAML into vendor-specific jd9365 config.
    jd9365_vendor_config_t vendor_config = {
        .mipi_config = {
            // Reuse the DSI bus already created by board-manager.
            .dsi_bus = dsi_handle,
            // Forward DPI timing/profile from board_devices.yaml (no duplication here).
            .dpi_config = &lcd_cfg->sub_cfg.dsi.dpi_config,
            // Hardware is wired for 2 data lanes.
            .lane_num = 2,
        },
        .flags = {
            .use_mipi_interface = 1,
        },
    };

    // Generic panel configuration passed to esp_lcd panel driver.
    esp_lcd_panel_dev_config_t lcd_dev_config = {
        .reset_gpio_num = lcd_cfg->sub_cfg.dsi.reset_gpio_num,
        .rgb_ele_order = lcd_cfg->rgb_ele_order,
        .bits_per_pixel = lcd_cfg->bits_per_pixel,
        .data_endian = lcd_cfg->data_endian,
        .flags = {
            .reset_active_high = lcd_cfg->sub_cfg.dsi.reset_active_high,
        },
        .vendor_config = &vendor_config,
    };

    // Create panel handle; io_handle comes from DSI command channel setup in board-manager.
    esp_err_t ret = esp_lcd_new_panel_jd9365(lcd_handles->io_handle, &lcd_dev_config, &lcd_handles->panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create JD9365 panel: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t lcd_touch_factory_entry_t(esp_lcd_panel_io_handle_t io, const esp_lcd_touch_config_t *touch_dev_config, esp_lcd_touch_handle_t *ret_touch)
{
    // Touch controller is GT911 on shared I2C bus. Driver handles runtime probe/address.
    esp_err_t ret = esp_lcd_touch_new_i2c_gt911(io, touch_dev_config, ret_touch);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create GT911 touch driver: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}
