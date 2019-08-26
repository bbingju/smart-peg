#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#include "esp_log.h"

#include "leds.h"
#include "sensors.h"

static const char *TAG = "smart-peg";

#define RED   0x00FF00
#define GREEN 0xFF0000
#define BLUE  0x0000FF

static void led_task(void *arg)
{
    leds_handle_t leds = leds_create(DEFAULT_PIXEL_WIDTH, DEFAULT_PIXEL_HEIGHT);

    leds_fill_rect(leds,3,5,7,9, 0x000102);
    leds_draw(leds);
    leds_fill_rect(leds,0,2,4,5, 0x100002);
    leds_draw(leds);
    /* leds_set_color(leds,0,0,0x010000); */
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    while (1) {
        /* leds_draw(leds); */
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

static void sensor_task(void *arg)
{
    while (1) {
        struct sensors_data *data =  sensors_read();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_ERROR_CHECK(sensors_init());

    // LED init
    /* sk6812_init(); */

    xTaskCreate(led_task, "led_task", 1024 * 2, (void *)0, 10, NULL);
    xTaskCreate(sensor_task, "sensor_task", 1024 * 2, (void *)0, 10, NULL);
}
