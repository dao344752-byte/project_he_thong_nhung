#ifndef __HX711_H__
#define __HX711_H__

#include "driver/gpio.h"
#include "esp_err.h"

typedef struct {
    gpio_num_t dout;
    gpio_num_t sck;
    int gain;
    float scale;
    int32_t offset;
} hx711_t;

// Init
esp_err_t hx711_init(hx711_t *dev, gpio_num_t dout, gpio_num_t sck);

// Read raw
int32_t hx711_read(hx711_t *dev);

// Average
int32_t hx711_read_average(hx711_t *dev, int times);

// Tare (zero)
void hx711_tare(hx711_t *dev, int times);

// Set scale
void hx711_set_scale(hx711_t *dev, float scale);

// Get units (gram, kg...)
float hx711_get_units(hx711_t *dev, int times);

// Check ready
bool hx711_is_ready(hx711_t *dev);

#endif