#include "leds.h"
#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/rmt.h"
#include "esp_log.h"

static const char *TAG = "leds";

#define LED_RMT_TX_CHANNEL RMT_CHANNEL_0
#define LED_RMT_TX_GPIO 5

#define BITS_PER_LED_CMD (24)

/*******************
* SK6812 Timings
********************/
#define T0H	3               // T0H for SK6812 -> 0.3 us
#define T0L	9               // T0L for SK6812 -> 0.9 us
#define T1H	6               // T1H for SK6812 -> 0.6 us
#define T1L	6               // T1L for SK6812 -> 0.6 us
#define TRS	800             // TRES for SK6812 -> 80 us

struct leds {
    int width, height;
    struct pixel *pixels;
    rmt_item32_t *rmt_buffer;
    int rmt_buffer_items;
};

static struct leds onlyone_inst = {
    .width       = DEFAULT_PIXEL_WIDTH,
    .height      = DEFAULT_PIXEL_HEIGHT,
    .pixels      = NULL,
};


static void sk6812_init(void)
{
    rmt_config_t config;
    config.rmt_mode = RMT_MODE_TX;
    config.channel = LED_RMT_TX_CHANNEL;
    config.gpio_num = LED_RMT_TX_GPIO;
    config.mem_block_num = 1;
    config.tx_config.loop_en = false;
    config.tx_config.carrier_en = false;
    config.tx_config.idle_output_en = true;
    config.tx_config.idle_level = (rmt_idle_level_t)0; // output level to 0 when idle
    config.tx_config.carrier_duty_percent = 50; // must be set to prevent errors - not used
    config.tx_config.carrier_freq_hz = 10000; // must be set to prevent errors - not used
    config.tx_config.carrier_level = (rmt_carrier_level_t)0; // must be set to prevent errors - not used
    config.clk_div = 8;

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));
}

void setup_rmt_data_buffer(rmt_item32_t *data_buffer,
                           struct pixel *pixels, int pixel_num)
{
    uint32_t i;
    for (i = 0; i < pixel_num; i++) {
        uint32_t bits_to_send = (pixels + i)->color;
        uint32_t mask = 1 << (BITS_PER_LED_CMD - 1);
        for (uint32_t bit = 0; bit < BITS_PER_LED_CMD; bit++) {
            uint32_t bit_is_set = bits_to_send & mask;
            data_buffer[i * BITS_PER_LED_CMD + bit] = bit_is_set ?
                (rmt_item32_t){{{T1H, 1, T1L, 0}}} :
            (rmt_item32_t){{{T0H, 1, T0L, 0}}};
            mask >>= 1;
        }
    }

    /* When the 1 is set to 0 ticks, it stops working */
    data_buffer[i * BITS_PER_LED_CMD] = (rmt_item32_t){{{1, 1, TRS, 0}}};
    /* RMT end marker */
    data_buffer[i * BITS_PER_LED_CMD + 1] = (rmt_item32_t){{{0, 0, 0, 0}}};
}

static struct pixel * create_pixels(int width, int height)
{
    struct pixel *pixels = calloc(1, sizeof(struct pixel) * width * height);
    if (!pixels)
        return NULL;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int id = (y * x) + x;
            struct pixel *p = pixels + id;
            p->id = id;
            p->x = x;
            p->y = y;
            p->color = 0x000000;
        }
    }

    return pixels;
}


static void destroy_pixels(struct pixel *p)
{
    if (p) {
        free(p);
    }
}


static rmt_item32_t * create_rmt_buffers(int items)
{
    rmt_item32_t *obj = calloc(1, sizeof(rmt_item32_t) * items);
    if (!obj)
        return NULL;

    return obj;
}


leds_handle_t leds_create(int width, int height)
{
    struct leds *obj = &onlyone_inst;

    if (obj->pixels)
        destroy_pixels(obj->pixels);

    obj->pixels = create_pixels(width, height);
    if (!obj->pixels)
        return NULL;

    obj->width  = width;
    obj->height = height;

    obj->rmt_buffer_items = obj->width * obj->height * BITS_PER_LED_CMD + 2;
    obj->rmt_buffer = create_rmt_buffers(obj->rmt_buffer_items);
    if (!obj->rmt_buffer) {

        goto err_ret;
    }

    sk6812_init();

    return obj;

err_ret:
    if (obj->pixels)
        destroy_pixels(obj->pixels);

    return NULL;
}


void leds_destroy(leds_handle_t handle)
{
    struct leds *obj = &onlyone_inst;

    if (obj->pixels)
        destroy_pixels(obj->pixels);
}


void leds_clear(leds_handle_t handle)
{
    struct leds *obj = &onlyone_inst;

    for (int i = 0; i < obj->height; i++) {
        for (int j = 0; j < obj->width; j++) {
            int id = (i * obj->width) + j;
            struct pixel *p = obj->pixels + id;
            p->color = 0x000000;
        }
    }
}


void leds_set_color(leds_handle_t handle, int x, int y, color_t color)
{
    struct leds *obj = &onlyone_inst;

    if (x < 0 || x >= obj->width)
        return;

    if (y < 0 || y >= obj->height)
        return;

    int id = (y * obj->width) + x;
    struct pixel *p = obj->pixels + id;
    p->color = color;
}


void leds_fill_rect(leds_handle_t handle, int x1, int y1, int x2, int y2, color_t color)
{
    struct leds *obj = &onlyone_inst;

    if (x1 < 0 || x1 >= obj->width)
        return;

    if (y1 < 0 || y1 >= obj->height)
        return;

    if (x2 >= obj->width || y2 >= obj->height)
        return;

    if (x2 <= x1 || y2 <= y1)
        return;

    int w = x2 - x1 + 1;
    int h = y2 - y1 + 1;

    for (int i = y1; i < y1 + h; i++) {
        for (int j = x1; j < x1 + w; j++) {
            int id = (i * obj->width) + j;
            struct pixel *p = obj->pixels + id;
            p->color = color;
        }
    }
}


void leds_draw(leds_handle_t handle)
{
    struct leds *obj = &onlyone_inst;

    setup_rmt_data_buffer(obj->rmt_buffer, obj->pixels, obj->width * obj->height);
    ESP_ERROR_CHECK(rmt_write_items(LED_RMT_TX_CHANNEL, obj->rmt_buffer, obj->rmt_buffer_items, false));
    ESP_ERROR_CHECK(rmt_wait_tx_done(LED_RMT_TX_CHANNEL, portMAX_DELAY));
}
