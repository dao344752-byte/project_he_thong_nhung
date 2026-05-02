#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/uart.h"

#include "nvs_flash.h"
#include "nvs.h"

#include "hx711.h"
#include "ssd1306.h"

// ======================================================
// CONFIG
// ======================================================
#define I2C_SDA         2
#define I2C_SCL         3

#define HX711_DT        4
#define HX711_SCK       5

// UART1
#define UART_PORT       UART_NUM_1
#define UART_TX_PIN     18
#define UART_RX_PIN     19
#define UART_BUF_SIZE   256

#define BUTTON_PIN      0
#define BUZZER_PIN      1

// ======================================================
// STRUCT
// ======================================================
typedef struct
{
    float bpm;
    float spo2;
    float temp;
    float hum;
    float weight;
    int32_t raw;

} sensor_data_t;

// ======================================================
// GLOBAL
// ======================================================
hx711_t scale;
ssd1306_t oled;

sensor_data_t g_data = {0};

// ======================================================
// RTOS OBJECT
// ======================================================
SemaphoreHandle_t data_mutex;
SemaphoreHandle_t oled_mutex;
SemaphoreHandle_t nvs_mutex;
SemaphoreHandle_t buzzer_semaphore;
QueueHandle_t oled_queue;

// ======================================================
// I2C INIT
// ======================================================
void i2c_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .master.clk_speed = 400000
    };

    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);

}

// ======================================================
// UART INIT
// ======================================================
void uart_init(void)
{
    uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB
    };

    uart_param_config(UART_PORT, &cfg);

    // ESP-IDF: TX trước RX
    uart_set_pin(
        UART_PORT,
        UART_TX_PIN,
        UART_RX_PIN,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    );

    uart_driver_install(UART_PORT, UART_BUF_SIZE * 2, 0, 0, NULL, 0);

    uart_flush(UART_PORT);
}

// ======================================================
// NVS
// ======================================================
void save_offset(int32_t offset)
{
    if (xSemaphoreTake(nvs_mutex, portMAX_DELAY))
    {
        nvs_handle_t nvs;

        if (nvs_open("hx711", NVS_READWRITE, &nvs) == ESP_OK)
        {
            nvs_set_i32(nvs, "offset", offset);
            nvs_commit(nvs);
            nvs_close(nvs);
        }

        xSemaphoreGive(nvs_mutex);
    }
}

int32_t load_offset(void)
{
    int32_t offset = 0;

    if (xSemaphoreTake(nvs_mutex, portMAX_DELAY))
    {
        nvs_handle_t nvs;

        if (nvs_open("hx711", NVS_READONLY, &nvs) == ESP_OK)
        {
            nvs_get_i32(nvs, "offset", &offset);
            nvs_close(nvs);
        }

        xSemaphoreGive(nvs_mutex);
    }

    return offset;
}

// ======================================================
// ======================================================
void process_uart(char *line)
{
    if (xSemaphoreTake(data_mutex, portMAX_DELAY))
    {
        if (strncmp(line, "BPM=", 4) == 0)
        {
            sscanf(line, "BPM=%f, SpO2=%f",
                   &g_data.bpm,
                   &g_data.spo2);
        }
        else if (strncmp(line, "T=", 2) == 0)
        {
            sscanf(line, "T=%f H=%f",
                   &g_data.temp,
                   &g_data.hum);
        }
        else if (strncmp(line, "STATUS=", 7) == 0)
        {
            // 
        }

        xSemaphoreGive(data_mutex);
    }

    xQueueSend(oled_queue, &g_data, 0);
}

// ======================================================
// TASK UART
// ======================================================
void task_uart(void *arg)
{
    uint8_t data[64];

    char line[128];
    int idx = 0;

    printf("UART TASK STARTED\n");

    while (1)
    {
        int len = uart_read_bytes(
            UART_PORT,
            data,
            sizeof(data),
            pdMS_TO_TICKS(50)
        );

        if (len > 0)
        {
            for (int i = 0; i < len; i++)
            {
                char c = data[i];

                if (c == '\n')
                {
                    if (idx > 0)
                    {
                        line[idx] = 0;

                        printf("UART RX: %s\n", line);

                        process_uart(line);

                        idx = 0;
                    }
                }
                else if (c != '\r')
                {
                    if (idx < sizeof(line) - 1)
                    {
                        line[idx++] = c;
                    }
                    else
                    {
                        idx = 0;
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// ======================================================
// TASK HX711
// ======================================================
void task_hx711(void *arg)
{
    int32_t offset = load_offset();

    while (1)
    {
        if (hx711_is_ready(&scale))
        {
            float raw = hx711_read_average(&scale, 20);
            printf("RAW = %.2f\n", raw);
            float weight =
                (raw - offset) / (-380000.0f);

            if (xSemaphoreTake(data_mutex, portMAX_DELAY))
            {
                g_data.raw = raw;
                g_data.weight = weight;

                xSemaphoreGive(data_mutex);
            }

            xQueueSend(oled_queue, &g_data, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// ======================================================
// TASK OLED
// ======================================================
void task_oled(void *arg)
{
    sensor_data_t data;

    char buf[32];

    while (1)
    {
        if (xQueueReceive(oled_queue, &data, portMAX_DELAY))
        {
            if (xSemaphoreTake(oled_mutex, portMAX_DELAY))
            {
                ssd1306_clear(&oled);

                sprintf(buf, "WEIGHT: %.2f", data.weight);
                ssd1306_draw_string(0, 0, buf);

                sprintf(buf, "BPM: %.0f", data.bpm);
                ssd1306_draw_string(0, 16, buf);

                sprintf(buf, "SPO2: %.0f", data.spo2);
                ssd1306_draw_string(0, 32, buf);

                sprintf(buf, "T: %.1f H: %.0f",
                        data.temp,
                        data.hum);
                ssd1306_draw_string(0, 48, buf);

                ssd1306_update(&oled);

                xSemaphoreGive(oled_mutex);
            }
        }
    }
}

void task_buzzer(void *arg)
{
    int buzzer_on = 0;
    TickType_t start_time = 0;

    // debounce
    TickType_t last_press_time = 0;
    const TickType_t debounce_delay = pdMS_TO_TICKS(200);

    for (;;)
    {
        // chờ event hoặc timeout để check auto-off
        if (xSemaphoreTake(buzzer_semaphore, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            TickType_t now = xTaskGetTickCount();

            // debounce
            if ((now - last_press_time) > debounce_delay)
            {
                last_press_time = now;

                buzzer_on = !buzzer_on;

                if (buzzer_on)
                {
                    gpio_set_level(BUZZER_PIN, 1);
                    start_time = now;

                    // KHÔNG nên printf trực tiếp
                    // sendUart("BUZZER ON\n");
                }
                else
                {
                    gpio_set_level(BUZZER_PIN, 0);
                    // sendUart("BUZZER OFF\n");
                }
            }
        }

        // auto OFF sau 10s
        if (buzzer_on)
        {
            if ((xTaskGetTickCount() - start_time) > pdMS_TO_TICKS(10000))
            {
                buzzer_on = 0;
                gpio_set_level(BUZZER_PIN, 0);

                // sendUart("BUZZER AUTO OFF\n");
            }
        }

        // yield nhẹ cho scheduler
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
static void IRAM_ATTR button_isr_handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(buzzer_semaphore, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

// ======================================================
// MAIN
// ======================================================
void app_main(void)
{
    nvs_flash_init();

    i2c_init();
    uart_init();

    // mutex
    data_mutex = xSemaphoreCreateMutex();
    oled_mutex = xSemaphoreCreateMutex();
    nvs_mutex  = xSemaphoreCreateMutex();
    buzzer_semaphore = xSemaphoreCreateBinary();

    // queue
    oled_queue = xQueueCreate(5, sizeof(sensor_data_t));

    // OLED
    ssd1306_init(&oled, I2C_NUM_0, 0x3C);

    // HX711
    hx711_init(&scale, HX711_DT, HX711_SCK);
    vTaskDelay(pdMS_TO_TICKS(300));
    
    /*int32_t raw = hx711_read_average(&scale, 20);
    save_offset(raw);
    scale.offset = raw;
    printf("Saved offset: %ld\n", raw);*/

    gpio_set_direction(BUZZER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(BUZZER_PIN, 0);

    // BUTTON (GPIO0)
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .pull_up_en = 1,
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN, button_isr_handler, NULL);
    uart_write_bytes(UART_PORT, "SYNC\n", strlen("SYNC\n"));
    // TASKS
    xTaskCreate(task_uart,  "uart",  4096, NULL, 3, NULL);
    xTaskCreate(task_hx711, "hx711", 4096, NULL, 3, NULL);
    xTaskCreate(task_oled,  "oled",  4096, NULL, 2, NULL);
    xTaskCreate(task_buzzer, "buzzer", 4096, NULL, 2, NULL);
    printf("SYSTEM STARTED\n");
}

