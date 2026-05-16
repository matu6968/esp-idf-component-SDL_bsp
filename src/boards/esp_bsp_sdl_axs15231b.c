/**
 * @file esp_bsp_sdl_axs15231b.c
 * @brief AXS15231B 320x480 QSPI panel + touch for Scratch Everywhere / SDL3
 */

#include "esp_bsp_sdl.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_lcd_io_i2c.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_touch.h"
#include "esp_check.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"

#include "esp_lcd_axs15231b.h"

#define SDL_PIXELFORMAT_RGB565 0x15151002u

static const char *TAG = "esp_bsp_sdl_axs15231b";

static esp_lcd_panel_handle_t s_panel = NULL;
static esp_lcd_panel_io_handle_t s_panel_io = NULL;
static esp_lcd_touch_handle_t s_touch = NULL;
static i2c_master_bus_handle_t s_i2c_bus = NULL;

static const axs15231b_lcd_init_cmd_t s_init_cmds[] = {
    {0xBB, (uint8_t[]){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5A, 0xA5}, 8, 0},
    {0xA0, (uint8_t[]){0xC0, 0x10, 0x00, 0x02, 0x00, 0x00, 0x04, 0x3F, 0x20, 0x05, 0x3F, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00}, 17, 0},
    {0xA2, (uint8_t[]){0x30, 0x3C, 0x24, 0x14, 0xD0, 0x20, 0xFF, 0xE0, 0x40, 0x19, 0x80, 0x80, 0x80, 0x20, 0xf9, 0x10, 0x02, 0xff, 0xff, 0xF0, 0x90, 0x01, 0x32, 0xA0, 0x91, 0xE0, 0x20, 0x7F, 0xFF, 0x00, 0x5A}, 31, 0},
    {0xD0, (uint8_t[]){0xE0, 0x40, 0x51, 0x24, 0x08, 0x05, 0x10, 0x01, 0x20, 0x15, 0x42, 0xC2, 0x22, 0x22, 0xAA, 0x03, 0x10, 0x12, 0x60, 0x14, 0x1E, 0x51, 0x15, 0x00, 0x8A, 0x20, 0x00, 0x03, 0x3A, 0x12}, 30, 0},
    {0xA3, (uint8_t[]){0xA0, 0x06, 0xAa, 0x00, 0x08, 0x02, 0x0A, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x00, 0x55, 0x55}, 22, 0},
    {0xC1, (uint8_t[]){0x31, 0x04, 0x02, 0x02, 0x71, 0x05, 0x24, 0x55, 0x02, 0x00, 0x41, 0x00, 0x53, 0xFF, 0xFF, 0xFF, 0x4F, 0x52, 0x00, 0x4F, 0x52, 0x00, 0x45, 0x3B, 0x0B, 0x02, 0x0d, 0x00, 0xFF, 0x40}, 30, 0},
    {0xC3, (uint8_t[]){0x00, 0x00, 0x00, 0x50, 0x03, 0x00, 0x00, 0x00, 0x01, 0x80, 0x01}, 11, 0},
    {0xC4, (uint8_t[]){0x00, 0x24, 0x33, 0x80, 0x00, 0xea, 0x64, 0x32, 0xC8, 0x64, 0xC8, 0x32, 0x90, 0x90, 0x11, 0x06, 0xDC, 0xFA, 0x00, 0x00, 0x80, 0xFE, 0x10, 0x10, 0x00, 0x0A, 0x0A, 0x44, 0x50}, 29, 0},
    {0xC5, (uint8_t[]){0x18, 0x00, 0x00, 0x03, 0xFE, 0x3A, 0x4A, 0x20, 0x30, 0x10, 0x88, 0xDE, 0x0D, 0x08, 0x0F, 0x0F, 0x01, 0x3A, 0x4A, 0x20, 0x10, 0x10, 0x00}, 23, 0},
    {0xC6, (uint8_t[]){0x05, 0x0A, 0x05, 0x0A, 0x00, 0xE0, 0x2E, 0x0B, 0x12, 0x22, 0x12, 0x22, 0x01, 0x03, 0x00, 0x3F, 0x6A, 0x18, 0xC8, 0x22}, 20, 0},
    {0xC7, (uint8_t[]){0x50, 0x32, 0x28, 0x00, 0xa2, 0x80, 0x8f, 0x00, 0x80, 0xff, 0x07, 0x11, 0x9c, 0x67, 0xff, 0x24, 0x0c, 0x0d, 0x0e, 0x0f}, 20, 0},
    {0xC9, (uint8_t[]){0x33, 0x44, 0x44, 0x01}, 4, 0},
    {0xCF, (uint8_t[]){0x2C, 0x1E, 0x88, 0x58, 0x13, 0x18, 0x56, 0x18, 0x1E, 0x68, 0x88, 0x00, 0x65, 0x09, 0x22, 0xC4, 0x0C, 0x77, 0x22, 0x44, 0xAA, 0x55, 0x08, 0x08, 0x12, 0xA0, 0x08}, 27, 0},
    {0xD5, (uint8_t[]){0x40, 0x8E, 0x8D, 0x01, 0x35, 0x04, 0x92, 0x74, 0x04, 0x92, 0x74, 0x04, 0x08, 0x6A, 0x04, 0x46, 0x03, 0x03, 0x03, 0x03, 0x82, 0x01, 0x03, 0x00, 0xE0, 0x51, 0xA1, 0x00, 0x00, 0x00}, 30, 0},
    {0xD6, (uint8_t[]){0x10, 0x32, 0x54, 0x76, 0x98, 0xBA, 0xDC, 0xFE, 0x93, 0x00, 0x01, 0x83, 0x07, 0x07, 0x00, 0x07, 0x07, 0x00, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x00, 0x84, 0x00, 0x20, 0x01, 0x00}, 30, 0},
    {0xD7, (uint8_t[]){0x03, 0x01, 0x0b, 0x09, 0x0f, 0x0d, 0x1E, 0x1F, 0x18, 0x1d, 0x1f, 0x19, 0x40, 0x8E, 0x04, 0x00, 0x20, 0xA0, 0x1F}, 19, 0},
    {0xD8, (uint8_t[]){0x02, 0x00, 0x0a, 0x08, 0x0e, 0x0c, 0x1E, 0x1F, 0x18, 0x1d, 0x1f, 0x19}, 12, 0},
    {0xD9, (uint8_t[]){0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F}, 12, 0},
    {0xDD, (uint8_t[]){0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F}, 12, 0},
    {0xDF, (uint8_t[]){0x44, 0x73, 0x4B, 0x69, 0x00, 0x0A, 0x02, 0x90}, 8, 0},
    {0xE0, (uint8_t[]){0x3B, 0x28, 0x10, 0x16, 0x0c, 0x06, 0x11, 0x28, 0x5c, 0x21, 0x0D, 0x35, 0x13, 0x2C, 0x33, 0x28, 0x0D}, 17, 0},
    {0xE1, (uint8_t[]){0x37, 0x28, 0x10, 0x16, 0x0b, 0x06, 0x11, 0x28, 0x5C, 0x21, 0x0D, 0x35, 0x14, 0x2C, 0x33, 0x28, 0x0F}, 17, 0},
    {0xE2, (uint8_t[]){0x3B, 0x07, 0x12, 0x18, 0x0E, 0x0D, 0x17, 0x35, 0x44, 0x32, 0x0C, 0x14, 0x14, 0x36, 0x3A, 0x2F, 0x0D}, 17, 0},
    {0xE3, (uint8_t[]){0x37, 0x07, 0x12, 0x18, 0x0E, 0x0D, 0x17, 0x35, 0x44, 0x32, 0x0C, 0x14, 0x14, 0x36, 0x32, 0x2F, 0x0F}, 17, 0},
    {0xE4, (uint8_t[]){0x3B, 0x07, 0x12, 0x18, 0x0E, 0x0D, 0x17, 0x39, 0x44, 0x2E, 0x0C, 0x14, 0x14, 0x36, 0x3A, 0x2F, 0x0D}, 17, 0},
    {0xE5, (uint8_t[]){0x37, 0x07, 0x12, 0x18, 0x0E, 0x0D, 0x17, 0x39, 0x44, 0x2E, 0x0C, 0x14, 0x14, 0x36, 0x3A, 0x2F, 0x0F}, 17, 0},
    {0xA4, (uint8_t[]){0x85, 0x85, 0x95, 0x82, 0xAF, 0xAA, 0xAA, 0x80, 0x10, 0x30, 0x40, 0x40, 0x20, 0xFF, 0x60, 0x30}, 16, 0},
    {0xA4, (uint8_t[]){0x85, 0x85, 0x95, 0x85}, 4, 0},
    {0xBB, (uint8_t[]){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 8, 0},
    {0x13, (uint8_t[]){0x00}, 0, 0},
    {0x11, (uint8_t[]){0x00}, 0, 120},
    {0x2C, (uint8_t[]){0x00, 0x00, 0x00, 0x00}, 4, 0},
};

static esp_err_t axs15231b_init(esp_bsp_sdl_display_config_t *config,
                                esp_lcd_panel_handle_t *panel_handle,
                                esp_lcd_panel_io_handle_t *panel_io_handle)
{
    if(!config || !panel_handle || !panel_io_handle) {
        return ESP_ERR_INVALID_ARG;
    }

    const int w = CONFIG_SDL_BSP_AXS_LCD_H_RES;
    const int h = CONFIG_SDL_BSP_AXS_LCD_V_RES;
    const bool landscape = (w > h);
    const int panel_w = landscape ? h : w;
    const int panel_h = landscape ? w : h;
    config->width = w;
    config->height = h;
    config->pixel_format = SDL_PIXELFORMAT_RGB565;
    config->max_transfer_sz = (size_t)w * (size_t)h * sizeof(uint16_t);
    config->has_touch = true;

#if CONFIG_SDL_BSP_AXS_LCD_PIN_BLK >= 0
    gpio_config_t bl_cfg = {
        .pin_bit_mask = 1ULL << CONFIG_SDL_BSP_AXS_LCD_PIN_BLK,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if(gpio_config(&bl_cfg) == ESP_OK) {
        gpio_set_level((gpio_num_t)CONFIG_SDL_BSP_AXS_LCD_PIN_BLK, CONFIG_SDL_BSP_AXS_LCD_BLK_ON_LEVEL);
    }
#endif

    spi_bus_config_t buscfg = {
        .sclk_io_num = CONFIG_SDL_BSP_AXS_LCD_PIN_SCLK,
        .data0_io_num = CONFIG_SDL_BSP_AXS_LCD_PIN_MOSI,
        .data1_io_num = CONFIG_SDL_BSP_AXS_LCD_QSPI_DATA1,
        .data2_io_num = CONFIG_SDL_BSP_AXS_LCD_QSPI_DATA2,
        .data3_io_num = CONFIG_SDL_BSP_AXS_LCD_QSPI_DATA3,
        .max_transfer_sz = (int)config->max_transfer_sz,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO), TAG, "spi init");

    esp_lcd_panel_io_spi_config_t iocfg = {
        .cs_gpio_num = CONFIG_SDL_BSP_AXS_LCD_PIN_CS,
        .dc_gpio_num = GPIO_NUM_NC,
        .spi_mode = 3,
        .pclk_hz = 50 * 1000 * 1000,
        .trans_queue_depth = 1,
        .lcd_cmd_bits = 32,
        .lcd_param_bits = 8,
        .flags = {.quad_mode = true},
    };
    ESP_RETURN_ON_ERROR(
        esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &iocfg, &s_panel_io), TAG, "panel io");

    const axs15231b_vendor_config_t vendor = {
        .init_cmds = s_init_cmds,
        .init_cmds_size = sizeof(s_init_cmds) / sizeof(s_init_cmds[0]),
        .flags = {.use_qspi_interface = 1},
    };
    esp_lcd_panel_dev_config_t panelcfg = {
        .reset_gpio_num = CONFIG_SDL_BSP_AXS_LCD_PIN_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
        .vendor_config = (void *)&vendor,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_axs15231b(s_panel_io, &panelcfg, &s_panel), TAG, "new panel");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(s_panel), TAG, "panel reset");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(s_panel), TAG, "panel init");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(s_panel, true), TAG, "panel on");

    i2c_master_bus_config_t i2c_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = CONFIG_SDL_BSP_AXS_TOUCH_PIN_SCL,
        .sda_io_num = CONFIG_SDL_BSP_AXS_TOUCH_PIN_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_lcd_panel_io_handle_t touch_io = NULL;
    esp_lcd_panel_io_i2c_config_t touch_io_cfg = {
        .dev_addr = ESP_LCD_TOUCH_IO_I2C_AXS15231B_ADDRESS,
        .scl_speed_hz = 400000,
        .control_phase_bytes = 1,
        .dc_bit_offset = 0,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 0,
        .flags = {.disable_control_phase = 1},
    };
    esp_lcd_touch_config_t touch_cfg = {
        .x_max = panel_w,
        .y_max = panel_h,
        .rst_gpio_num = CONFIG_SDL_BSP_AXS_TOUCH_PIN_RST >= 0 ? CONFIG_SDL_BSP_AXS_TOUCH_PIN_RST : GPIO_NUM_NC,
        .int_gpio_num = CONFIG_SDL_BSP_AXS_TOUCH_PIN_INT >= 0 ? CONFIG_SDL_BSP_AXS_TOUCH_PIN_INT : GPIO_NUM_NC,
        .levels.reset = 0,
        .levels.interrupt = 0,
    };
    if(i2c_new_master_bus(&i2c_cfg, &s_i2c_bus) == ESP_OK &&
       esp_lcd_new_panel_io_i2c(s_i2c_bus, &touch_io_cfg, &touch_io) == ESP_OK &&
       esp_lcd_touch_new_i2c_axs15231b(touch_io, &touch_cfg, &s_touch) == ESP_OK) {
        ESP_LOGI(TAG, "touch ready");
    } else {
        ESP_LOGW(TAG, "touch init failed");
        s_touch = NULL;
    }

    *panel_handle = s_panel;
    *panel_io_handle = s_panel_io;
    ESP_LOGI(TAG, "AXS15231B %dx%d QSPI ready", w, h);
    return ESP_OK;
}

static esp_err_t axs15231b_backlight_on(void)
{
#if CONFIG_SDL_BSP_AXS_LCD_PIN_BLK >= 0
    gpio_set_level((gpio_num_t)CONFIG_SDL_BSP_AXS_LCD_PIN_BLK, CONFIG_SDL_BSP_AXS_LCD_BLK_ON_LEVEL);
#endif
    return ESP_OK;
}

static esp_err_t axs15231b_backlight_off(void)
{
#if CONFIG_SDL_BSP_AXS_LCD_PIN_BLK >= 0
    gpio_set_level((gpio_num_t)CONFIG_SDL_BSP_AXS_LCD_PIN_BLK, !CONFIG_SDL_BSP_AXS_LCD_BLK_ON_LEVEL);
#endif
    return ESP_OK;
}

static esp_err_t axs15231b_display_on_off(bool enable)
{
    if(!s_panel) {
        return ESP_ERR_INVALID_STATE;
    }
    return esp_lcd_panel_disp_on_off(s_panel, enable);
}

static esp_err_t axs15231b_touch_init(void)
{
    return s_touch ? ESP_OK : ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t axs15231b_touch_read(esp_bsp_sdl_touch_info_t *touch_info)
{
    if(!touch_info) {
        return ESP_ERR_INVALID_ARG;
    }
    touch_info->pressed = false;
    touch_info->x = 0;
    touch_info->y = 0;
    if(!s_touch) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    if(esp_lcd_touch_read_data(s_touch) != ESP_OK) {
        return ESP_FAIL;
    }
    uint16_t x[1] = {0};
    uint16_t y[1] = {0};
    uint8_t count = 0;
    if(!esp_lcd_touch_get_coordinates(s_touch, x, y, NULL, &count, 1) || count == 0) {
        return ESP_OK;
    }
    touch_info->pressed = true;
    const int raw_x = (int)x[0];
    const int raw_y = (int)y[0];
#if CONFIG_SDL_BSP_AXS_LCD_H_RES > CONFIG_SDL_BSP_AXS_LCD_V_RES
    /* Native 320x480 touch -> logical 480x320 (inverse of SDL framebuffer ROT_90). */
    touch_info->x = raw_y;
    touch_info->y = (CONFIG_SDL_BSP_AXS_LCD_V_RES - 1) - raw_x;
#else
    touch_info->x = raw_x;
    touch_info->y = raw_y;
#endif
    return ESP_OK;
}

static const char *axs15231b_get_name(void)
{
    return "AXS15231B";
}

static esp_err_t axs15231b_deinit(void)
{
    s_touch = NULL;
    s_panel = NULL;
    s_panel_io = NULL;
    return ESP_OK;
}

const esp_bsp_sdl_board_interface_t esp_bsp_sdl_axs15231b_interface = {
    .init = axs15231b_init,
    .backlight_on = axs15231b_backlight_on,
    .backlight_off = axs15231b_backlight_off,
    .display_on_off = axs15231b_display_on_off,
    .touch_init = axs15231b_touch_init,
    .touch_read = axs15231b_touch_read,
    .get_name = axs15231b_get_name,
    .deinit = axs15231b_deinit,
    .board_name = "AXS15231B",
};

esp_lcd_panel_handle_t esp_bsp_sdl_axs15231b_panel(void)
{
    return s_panel;
}

esp_lcd_panel_io_handle_t esp_bsp_sdl_axs15231b_panel_io(void)
{
    return s_panel_io;
}

esp_lcd_touch_handle_t esp_bsp_sdl_axs15231b_touch(void)
{
    return s_touch;
}
