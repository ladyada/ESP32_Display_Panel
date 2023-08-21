/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "soc/soc_caps.h"

#if SOC_LCD_RGB_SUPPORTED
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_lcd_panel_commands.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"

#include "../private/CheckResult.h"
#include "../bus/RGB.h"
#include "ST7701.h"

static const char *TAG = "st7701";

/**
 * @brief LCD configuration data structure type
 *
 */
typedef struct {
    uint8_t cmd;            // LCD command
    uint8_t data[52];       // LCD data
    uint8_t data_bytes;     // Length of data in above data array; 0xFF = end of cmds.
} lcd_init_cmd_t;

// *INDENT-OFF*
const static lcd_init_cmd_t vendor_specific_init[] = {
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x10}, 5},
    {0xC0, {0x3b, 0x00}, 2},
    {0xC1, {0x0B, 0x02}, 2},
    {0xC2, {0x00, 0x02}, 2},
    {0xCC, {0x10}, 1},
    {0xCD, {0x08}, 1},
    {0xB0, {0x02, 0x13, 0x1B, 0x0D, 0x10, 0x05, 0x09, 0x07, 0x07, 0x24, 0x04, 0x11, 0x0E, 0x2C, 0x33, 0x1D}, 16},
    {0xB1, {0x05, 0x13, 0x1B, 0x0D, 0x11, 0x05, 0x08, 0x07, 0x07, 0x24, 0x04, 0x11, 0x0E, 0x2C, 0x33, 0x1D}, 16},

    {0xff, {0x77, 0x01, 0x00, 0x00, 0x11}, 5},
    {0xb0, {0x5D}, 1},
    {0xb1, {0x43}, 1},
    {0xb2, {0x81}, 1},
    {0xb3, {0x80}, 1},
    {0xb5, {0x43}, 1},
    {0xb7, {0x85}, 1},
    {0xb8, {0x20}, 1},
    {0xC1, {0x78}, 1},
    {0xC2, {0x78}, 1},
    {0xD0, {0x88}, 1},

    {0xE0, {0x00, 0x00, 0x02}, 3},
    {0xE1, {0x03, 0xA0, 0x00, 0x00, 0x04, 0xA0, 0x00, 0x00, 0x00, 0x20, 0x20}, 11},
    {0xE2, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 13},
    {0xE3, {0x00, 0x00, 0x11, 0x00}, 4},
    {0xE4, {0x22, 0x00}, 2},
    {0xE5, {0x05, 0xEC, 0xA0, 0xA0, 0x07, 0xEE, 0xA0, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 16},
    {0xE6, {0x00, 0x00, 0x11, 0x00}, 4},
    {0xE7, {0x22, 0x00}, 2},
    {0xE8, {0x06, 0xED, 0xA0, 0xA0, 0x08, 0xEF, 0xA0, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 16},
    {0xEB, {0x00, 0x00, 0x40, 0x40, 0x00, 0x00, 0x00}, 7},
    {0xED, {0xFF, 0xFF, 0xFF, 0xBA, 0x0A, 0xBF, 0x45, 0xFF, 0xFF, 0x54, 0xFB, 0xA0, 0xAB, 0xFF, 0xFF, 0xFF}, 16},
    {0xEF, {0x10, 0x0D, 0x04, 0x08, 0x3F, 0x1F}, 6},

    {0xff, {0x77, 0x01, 0x00, 0x00, 0x13}, 5},
    {0xEF, {0x08}, 1},

    {0xff, {0x77, 0x01, 0x00, 0x00, 0x00}, 5},

    {0x29, {0x00}, 0},
    {0x36, {0x00}, 1},
    {0x3A, {0x60}, 1},

    {0x11, {0x00}, 0},
    {0x00, {0x00}, 0xff},
};
// *INDENT-OFF*

static esp_err_t esp_lcd_new_panel_st7701(esp_lcd_panel_io_handle_t io_handle, const esp_lcd_rgb_panel_config_t *rgb_config, esp_lcd_panel_handle_t *ret_panel);

ESP_PanelLcd_ST7701::~ESP_PanelLcd_ST7701()
{
    if (handle) {
        del();
    }
}

void ESP_PanelLcd_ST7701::init()
{
    CHECK_NULL_RETURN(bus);
    CHECK_ERROR_RETURN(esp_lcd_new_panel_st7701(bus->getHandle(), static_cast<ESP_PanelBus_RGB *>(bus)->getRGBConfig(), &handle));

    if (config.dev_config.reset_gpio_num >= 0) {
        gpio_config_t gpio_conf = {
            .pin_bit_mask = BIT64(config.dev_config.reset_gpio_num),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        CHECK_ERROR_RETURN(gpio_config(&gpio_conf));
    }
}

void ESP_PanelLcd_ST7701::reset()
{
    if(config.dev_config.reset_gpio_num >= 0) {
        gpio_set_level((gpio_num_t)config.dev_config.reset_gpio_num, config.dev_config.flags.reset_active_high);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level((gpio_num_t)config.dev_config.reset_gpio_num, !config.dev_config.flags.reset_active_high);
        vTaskDelay(pdMS_TO_TICKS(120));
    }
    CHECK_ERROR_RETURN(esp_lcd_panel_reset(handle));
}

static esp_err_t esp_lcd_new_panel_st7701(esp_lcd_panel_io_handle_t io_handle, const esp_lcd_rgb_panel_config_t *rgb_config, esp_lcd_panel_handle_t *ret_panel)
{
  ESP_LOGE(TAG, "Initializing ST7701\n\r");
    // Initialize LCD
    // Vendor specific initialization, it can be different between manufacturers
    // Should consult the LCD supplier for initialization sequence code
    int cmd = 0;
    while (vendor_specific_init[cmd].data_bytes != 0xff) {
      ESP_LOGE(TAG, "Sending command 0x%02X - ", vendor_specific_init[cmd].cmd);
      for (int i=0; i<vendor_specific_init[cmd].data_bytes; i++) {
        ESP_LOGE(TAG, "\t0x%02X", vendor_specific_init[cmd].data[i]);
      }

        esp_lcd_panel_io_tx_param(io_handle, vendor_specific_init[cmd].cmd, vendor_specific_init[cmd].data, vendor_specific_init[cmd].data_bytes);
        cmd++;
    }
    vTaskDelay(pdMS_TO_TICKS(120));
    esp_lcd_panel_io_tx_param(io_handle, LCD_CMD_DISPON, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(20));

    // Create RGB panel
    ESP_RETURN_ON_ERROR(esp_lcd_new_rgb_panel(rgb_config, ret_panel), TAG, "Failed to create RGB panel");

    return ESP_OK;
}

#endif /* SOC_LCD_RGB_SUPPORTED */
