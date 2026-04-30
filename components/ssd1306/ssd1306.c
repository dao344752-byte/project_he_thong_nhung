#include "ssd1306.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#define TAG "SSD1306"

static uint8_t buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

static void i2c_write(ssd1306_t *dev, uint8_t control, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, control, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(dev->i2c_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
}

static void ssd1306_cmd(ssd1306_t *dev, uint8_t cmd)
{
    i2c_write(dev, 0x00, cmd);
}

static void ssd1306_data(ssd1306_t *dev, uint8_t data)
{
    i2c_write(dev, 0x40, data);
}

void ssd1306_init(ssd1306_t *dev, i2c_port_t port, uint8_t addr)
{
    dev->i2c_port = port;
    dev->address = addr;

    vTaskDelay(pdMS_TO_TICKS(100));

    ssd1306_cmd(dev, 0xAE); // OFF
    ssd1306_cmd(dev, 0x20); // Memory mode
    ssd1306_cmd(dev, 0x00);
    ssd1306_cmd(dev, 0xB0);
    ssd1306_cmd(dev, 0xC8);
    ssd1306_cmd(dev, 0x00);
    ssd1306_cmd(dev, 0x10);
    ssd1306_cmd(dev, 0x40);
    ssd1306_cmd(dev, 0x81);
    ssd1306_cmd(dev, 0xFF);
    ssd1306_cmd(dev, 0xA1);
    ssd1306_cmd(dev, 0xA6);
    ssd1306_cmd(dev, 0xA8);
    ssd1306_cmd(dev, 0x3F);
    ssd1306_cmd(dev, 0xA4);
    ssd1306_cmd(dev, 0xD3);
    ssd1306_cmd(dev, 0x00);
    ssd1306_cmd(dev, 0xD5);
    ssd1306_cmd(dev, 0xF0);
    ssd1306_cmd(dev, 0xD9);
    ssd1306_cmd(dev, 0x22);
    ssd1306_cmd(dev, 0xDA);
    ssd1306_cmd(dev, 0x12);
    ssd1306_cmd(dev, 0xDB);
    ssd1306_cmd(dev, 0x20);
    ssd1306_cmd(dev, 0x8D);
    ssd1306_cmd(dev, 0x14);
    ssd1306_cmd(dev, 0xAF); // ON

    memset(buffer, 0, sizeof(buffer));
}

void ssd1306_clear(ssd1306_t *dev)
{
    memset(buffer, 0, sizeof(buffer));
}

void ssd1306_update(ssd1306_t *dev)
{
    for (int page = 0; page < 8; page++) {
        ssd1306_cmd(dev, 0xB0 + page);
        ssd1306_cmd(dev, 0x00);
        ssd1306_cmd(dev, 0x10);

        for (int i = 0; i < SSD1306_WIDTH; i++) {
            ssd1306_data(dev, buffer[page * SSD1306_WIDTH + i]);
        }
    }
}

void ssd1306_draw_pixel(int x, int y, uint8_t color)
{
    if (x < 0 || x >= SSD1306_WIDTH || y < 0 || y >= SSD1306_HEIGHT) return;

    if (color)
        buffer[x + (y / 8) * SSD1306_WIDTH] |= (1 << (y % 8));
    else
        buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
}

// FONT đơn giản (chỉ demo)
extern const uint8_t font5x7[];

void ssd1306_draw_char(int x, int y, char c)
{
    for (int i = 0; i < 5; i++) {
        uint8_t line = font5x7[(c - 32) * 5 + i];
        for (int j = 0; j < 8; j++) {
            ssd1306_draw_pixel(x + i, y + j, line & (1 << j));
        }
    }
}

void ssd1306_draw_string(int x, int y, const char *str)
{
    while (*str) {
        ssd1306_draw_char(x, y, *str++);
        x += 6;
    }
}