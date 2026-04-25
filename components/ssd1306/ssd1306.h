#ifndef __SSD1306_H__
#define __SSD1306_H__

#include "driver/i2c.h"
#include <stdint.h>

#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64

typedef struct {
    i2c_port_t i2c_port;
    uint8_t address;
} ssd1306_t;

void ssd1306_init(ssd1306_t *dev, i2c_port_t port, uint8_t addr);
void ssd1306_clear(ssd1306_t *dev);
void ssd1306_update(ssd1306_t *dev);
void ssd1306_draw_pixel(int x, int y, uint8_t color);
void ssd1306_draw_string(int x, int y, const char *str);

#endif