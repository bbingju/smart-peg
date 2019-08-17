#include "sk6812.h"
#include "driver/gpio.h"
#include "driver/rmt.h"
#include "esp_log.h"

#define LED_RMT_TX_CHANNEL RMT_CHANNEL_0
#define LED_RMT_TX_GPIO 5

#define BITS_PER_LED_CMD (24)
#define LED_BUFFER_ITEMS ((NUM_LEDS * BITS_PER_LED_CMD) + 2)

/*******************
* SK6812 Timings
********************/
#define T0H	3               // T0H for SK6812 -> 0.3 us
#define T0L	9               // T0L for SK6812 -> 0.9 us
#define T1H	6               // T1H for SK6812 -> 0.6 us
#define T1L	6               // T1L for SK6812 -> 0.6 us
#define TRS	800             // TRES for SK6812 -> 80 us

// This is the buffer which the hw peripheral will access while
// pulsing the output pin
rmt_item32_t led_data_buffer[LED_BUFFER_ITEMS];

void setup_rmt_data_buffer(struct led_state new_state);

void sk6812_init(void)
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

void print_table(rmt_item32_t *table, int length)
{
    for (int i = 0; i < length; i++) {
        ESP_LOGE("sk6812", "{ %04d: 0x%x }", i, (table + i)->val);
    }
    ESP_LOGE("sk6812", "\n");
}

void sk6812_write_leds(struct led_state new_state) {
    setup_rmt_data_buffer(new_state);
//    print_table(led_data_buffer, LED_BUFFER_ITEMS);
    ESP_ERROR_CHECK(rmt_write_items(LED_RMT_TX_CHANNEL, led_data_buffer, LED_BUFFER_ITEMS, false));
    ESP_ERROR_CHECK(rmt_wait_tx_done(LED_RMT_TX_CHANNEL, portMAX_DELAY));
}

void setup_rmt_data_buffer(struct led_state new_state)
{
    uint32_t led;
    for (led = 0; led < NUM_LEDS; led++) {
        uint32_t bits_to_send = new_state.leds[led];
        uint32_t mask = 1 << (BITS_PER_LED_CMD - 1);
        for (uint32_t bit = 0; bit < BITS_PER_LED_CMD; bit++) {
            uint32_t bit_is_set = bits_to_send & mask;
            led_data_buffer[led * BITS_PER_LED_CMD + bit] = bit_is_set ?
                (rmt_item32_t){{{T1H, 1, T1L, 0}}} : 
            (rmt_item32_t){{{T0H, 1, T0L, 0}}};
            mask >>= 1;
        }
    }

    /* When the 1 is set to 0 ticks, it stops working */
    led_data_buffer[led * BITS_PER_LED_CMD] = (rmt_item32_t){{{1, 1, TRS, 0}}};
    /* RMT end marker */
    led_data_buffer[led * BITS_PER_LED_CMD + 1] = (rmt_item32_t){{{0, 0, 0, 0}}};
}
