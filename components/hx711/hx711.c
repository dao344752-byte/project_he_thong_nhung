#include "hx711.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static void hx711_delay()
{
    esp_rom_delay_us(1);
}

esp_err_t hx711_init(hx711_t *dev, gpio_num_t dout, gpio_num_t sck)
{
    dev->dout = dout;
    dev->sck = sck;
    dev->gain = 128;
    dev->scale = 1.0f;
    dev->offset = 0;

    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << sck)
    };
    gpio_config(&io_conf);

    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << dout);
    gpio_config(&io_conf);

    gpio_set_level(dev->sck, 0);

    return ESP_OK;
}

bool hx711_is_ready(hx711_t *dev)
{
    return gpio_get_level(dev->dout) == 0;
}

int32_t hx711_read(hx711_t *dev)
{
    while (!hx711_is_ready(dev)) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    int32_t data = 0;

    for (int i = 0; i < 24; i++) {
        gpio_set_level(dev->sck, 1);
        hx711_delay();
        data = data << 1;
        gpio_set_level(dev->sck, 0);
        if (gpio_get_level(dev->dout)) {
            data++;
        }
    }

    // Set gain = 128 (1 extra pulse)
    gpio_set_level(dev->sck, 1);
    hx711_delay();
    gpio_set_level(dev->sck, 0);

    // Convert 24-bit signed
    if (data & 0x800000) {
        data |= ~0xFFFFFF;
    }

    return data;
}

int32_t hx711_read_average(hx711_t *dev, int times)
{
    int64_t sum = 0;
    for (int i = 0; i < times; i++) {
        sum += hx711_read(dev);
    }
    return sum / times;
}

void hx711_tare(hx711_t *dev, int times)
{
    dev->offset = hx711_read_average(dev, times);
}

void hx711_set_scale(hx711_t *dev, float scale)
{
    dev->scale = scale;
}

float hx711_get_units(hx711_t *dev, int times)
{
    int32_t raw = hx711_read_average(dev, times);
    return (raw - dev->offset) / dev->scale;
}