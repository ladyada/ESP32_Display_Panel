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
#include "NV3052CGRB.h"

static const char *TAG = "nv3052cgrb";

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
    {0xFF, {0x30}, 1},
    {0xFF, {0x52}, 1},
    {0xFF, {0x01}, 1},
    {0xE3, {0x00}, 1},
    {0x0A, {0x11}, 1},
    {0x23, {0xA0}, 1},//A2
    {0x24, {0x32}, 1},
    {0x25, {0x12}, 1},
    {0x26, {0x2E}, 1},
    {0x27, {0x2E}, 1},
    {0x29, {0x02}, 1},
    {0x2A, {0xCF}, 1},
    {0x32, {0x34}, 1},
    {0x38, {0x9C}, 1},
    {0x39, {0xA7}, 1},
    {0x3A, {0x27}, 1},
    {0x3B, {0x94}, 1},
    {0x42, {0x6D}, 1},
    {0x43, {0x83}, 1},
    {0x81, {0x00}, 1},
    {0x91, {0x67}, 1},
    {0x92, {0x67}, 1},
    {0xA0, {0x52}, 1},
    {0xA1, {0x50}, 1},
    {0xA4, {0x9C}, 1},
    {0xA7, {0x02}, 1},
    {0xA8, {0x02}, 1},
    {0xA9, {0x02}, 1},
    {0xAA, {0xA8}, 1},
    {0xAB, {0x28}, 1},
    {0xAE, {0xD2}, 1},
    {0xAF, {0x02}, 1},
    {0xB0, {0xD2}, 1},
    {0xB2, {0x26}, 1},
    {0xB3, {0x26}, 1},
    {0xFF, {0x30}, 1},
    {0xFF, {0x52}, 1},
    {0xFF, {0x02}, 1},
    {0xB1, {0x0A}, 1},
    {0xD1, {0x0E}, 1},
    {0xB4, {0x2F}, 1},
    {0xD4, {0x2D}, 1},
    {0xB2, {0x0C}, 1},
    {0xD2, {0x0C}, 1},
    {0xB3, {0x30}, 1},
    {0xD3, {0x2A}, 1},
    {0xB6, {0x1E}, 1},
    {0xD6, {0x16}, 1},
    {0xB7, {0x3B}, 1},
    {0xD7, {0x35}, 1},
    {0xC1, {0x08}, 1},
    {0xE1, {0x08}, 1},
    {0xB8, {0x0D}, 1},
    {0xD8, {0x0D}, 1},
    {0xB9, {0x05}, 1},
    {0xD9, {0x05}, 1},
    {0xBD, {0x15}, 1},
    {0xDD, {0x15}, 1},
    {0xBC, {0x13}, 1},
    {0xDC, {0x13}, 1},
    {0xBB, {0x12}, 1},
    {0xDB, {0x10}, 1},
    {0xBA, {0x11}, 1},
    {0xDA, {0x11}, 1},
    {0xBE, {0x17}, 1},
    {0xDE, {0x17}, 1},
    {0xBF, {0x0F}, 1},
    {0xDF, {0x0F}, 1},
    {0xC0, {0x16}, 1},
    {0xE0, {0x16}, 1},
    {0xB5, {0x2E}, 1},
    {0xD5, {0x3F}, 1},
    {0xB0, {0x03}, 1},
    {0xD0, {0x02}, 1},
    {0xFF, {0x30}, 1},
    {0xFF, {0x52}, 1},
    {0xFF, {0x03}, 1},
    {0x08, {0x09}, 1},		
    {0x09, {0x0A}, 1},		
    {0x0A, {0x0B}, 1},		
    {0x0B, {0x0C}, 1},		
    {0x28, {0x22}, 1},		
    {0x2A, {0xE9}, 1},	
    {0x2B, {0xE9}, 1},							  				  
    {0x34, {0x51}, 1},
    {0x35, {0x01}, 1},
    {0x36, {0x26}, 1},  
    {0x37, {0x13}, 1},
    {0x40, {0x07}, 1},  
    {0x41, {0x08}, 1},  
    {0x42, {0x09}, 1},  
    {0x43, {0x0A}, 1},
    {0x44, {0x22}, 1},
    {0x45, {0xDB}, 1},  
    {0x46, {0xdC}, 1},  
    {0x47, {0x22}, 1},
    {0x48, {0xDD}, 1},  
    {0x49, {0xDE}, 1}, 
    {0x50, {0x0B}, 1},  
    {0x51, {0x0C}, 1},  
    {0x52, {0x0D}, 1},  
    {0x53, {0x0E}, 1}, 
    {0x54, {0x22}, 1},
    {0x55, {0xDF}, 1},  
    {0x56, {0xE0}, 1},  
    {0x57, {0x22}, 1},
    {0x58, {0xE1}, 1},  
    {0x59, {0xE2}, 1}, 
    {0x80, {0x1E}, 1},   
    {0x81, {0x1E}, 1},   
    {0x82, {0x1F}, 1},   
    {0x83, {0x1F}, 1},   
    {0x84, {0x05}, 1},   
    {0x85, {0x0A}, 1},   
    {0x86, {0x0A}, 1},   
    {0x87, {0x0C}, 1},   
    {0x88, {0x0C}, 1},   
    {0x89, {0x0E}, 1},   
    {0x8A, {0x0E}, 1},    
    {0x8B, {0x10}, 1},   
    {0x8C, {0x10}, 1},    
    {0x8D, {0x00}, 1},   
    {0x8E, {0x00}, 1},   
    {0x8F, {0x1F}, 1},   
    {0x90, {0x1F}, 1},   
    {0x91, {0x1E}, 1},   
    {0x92, {0x1E}, 1},      
    {0x93, {0x02}, 1},   
    {0x94, {0x04}, 1}, 
    {0x96, {0x1E}, 1},   
    {0x97, {0x1E}, 1},   
    {0x98, {0x1F}, 1},   
    {0x99, {0x1F}, 1},   
    {0x9A, {0x05}, 1},   
    {0x9B, {0x09}, 1},   
    {0x9C, {0x09}, 1},   
    {0x9D, {0x0B}, 1},   
    {0x9E, {0x0B}, 1},   
    {0x9F, {0x0D}, 1},   
    {0xA0, {0x0D}, 1},   
    {0xA1, {0x0F}, 1},   
    {0xA2, {0x0F}, 1},   
    {0xA3, {0x00}, 1},   
    {0xA4, {0x00}, 1},   
    {0xA5, {0x1F}, 1},   
    {0xA6, {0x1F}, 1},   
    {0xA7, {0x1E}, 1},   
    {0xA8, {0x1E}, 1},   
    {0xA9, {0x01}, 1},   
    {0xAA, {0x03}, 1},  

    {0xFF, {0x30}, 1},
    {0xFF, {0x52}, 1},
    {0xFF, {0x00}, 1},
    {0x36, {0x0A}, 1},             
    {0x29, {0x00}, 1},
    {0x11, {0x00}, 1},
    {0x00, {0x00}, 0xff},
};
// *INDENT-OFF*

static esp_err_t esp_lcd_new_panel_nv3052cgrb(esp_lcd_panel_io_handle_t io_handle, const esp_lcd_rgb_panel_config_t *rgb_config, esp_lcd_panel_handle_t *ret_panel);

ESP_PanelLcd_NV3052CGRB::~ESP_PanelLcd_NV3052CGRB()
{
    if (handle) {
        del();
    }
}

void ESP_PanelLcd_NV3052CGRB::init()
{
    CHECK_NULL_RETURN(bus);
    CHECK_ERROR_RETURN(esp_lcd_new_panel_nv3052cgrb(bus->getHandle(), static_cast<ESP_PanelBus_RGB *>(bus)->getRGBConfig(), &handle));

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

void ESP_PanelLcd_NV3052CGRB::reset()
{
    if(config.dev_config.reset_gpio_num >= 0) {
        gpio_set_level((gpio_num_t)config.dev_config.reset_gpio_num, config.dev_config.flags.reset_active_high);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level((gpio_num_t)config.dev_config.reset_gpio_num, !config.dev_config.flags.reset_active_high);
        vTaskDelay(pdMS_TO_TICKS(120));
    }
    CHECK_ERROR_RETURN(esp_lcd_panel_reset(handle));
}

static esp_err_t esp_lcd_new_panel_nv3052cgrb(esp_lcd_panel_io_handle_t io_handle, const esp_lcd_rgb_panel_config_t *rgb_config, esp_lcd_panel_handle_t *ret_panel)
{
  ESP_LOGE(TAG, "Initializing NV3052CGRB\n\r");
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
