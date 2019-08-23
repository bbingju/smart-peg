#if 1
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#include "esp_log.h"

#include "sensor_addr_tlb.h"

#include "leds.h"

static const char *TAG = "smart-peg";

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1           /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0          /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                /*!< I2C ack value */
#define NACK_VAL 0x1               /*!< I2C nack value */

#define NUM_LEDS 5
#include "sk6812.h"

#define RED   0x00FF00
#define GREEN 0xFF0000
#define BLUE  0x0000FF

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}


static esp_err_t pca9535_read_register(uint8_t address, uint8_t *data_h, uint8_t *data_l)
{
    i2c_cmd_handle_t cmd;

    cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data_l, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, data_h, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

static uint16_t sensor_read(int id)
{
    struct sensor_addr_tlb *r = &sensor_addr_tlb[id];
    uint8_t data_h, data_l;
    ESP_ERROR_CHECK(pca9535_read_register(r->address, &data_h, &data_l));

    data_h = ~(data_h);
    data_l = ~(data_l);

    return ((data_h << 8) | (data_l));
}

static void pca9535_write_register(uint8_t address, uint8_t data)
{
    i2c_cmd_handle_t cmd;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x20 << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, address, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 10 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
}

static void led_task(void *arg)
{
    leds_handle_t leds = leds_create(DEFAULT_PIXEL_WIDTH, DEFAULT_PIXEL_HEIGHT);

    leds_fill_rect(leds,3,5,7,9, 0x000102);
    leds_fill_rect(leds,0,2,4,5, 0x100002);
    /* leds_set_color(leds,0,0,0x010000); */
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    while (1) {
        leds_draw(leds);
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

static void sensor_task(void *arg)
{
    while (1) {

        /* for (int i = 0; i < SENSOR_NUMBER; i++) { */
        /*     uint16_t data = sensor_read(i); */
        /*     if (i == 0) { */
        /*         new_state.leds[0] = (data & 0x0001) ? BLUE : 0x000000; */
        /*         new_state.leds[1] = (data & 0x0002) ? BLUE : 0x000000; */
        /*         new_state.leds[2] = (data & 0x0004) ? BLUE : 0x000000; */
        /*         new_state.leds[3] = (data & 0x0008) ? BLUE : 0x000000; */
        /*         new_state.leds[4] = (data & 0x0010) ? BLUE : 0x000000; */
        /*     } */
        /*     ESP_LOGI(TAG, "read sensor #%d data: 0x%04x", i, data); */
        /* } */
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());

    // LED init
    /* sk6812_init(); */

    xTaskCreate(led_task, "led_task", 1024 * 2, (void *)0, 10, NULL);
    xTaskCreate(sensor_task, "sensor_task", 1024 * 2, (void *)0, 10, NULL);
}

#else

// Std. C
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

//FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//ESP Specific
#include "esp_log.h"
#include "driver/rmt.h"
#include <driver/gpio.h>
#include "sdkconfig.h"

#define RMT_TX_CHANNEL1 RMT_CHANNEL_0

#define RMT_TX_GPIO 5

#define DIVIDER 8; 	/*set the maximum clock divider to be able to output
                          RMT pulses in range of about one hundred nanoseconds
                          a divider of 8 will give us 80 MHz / 8 = 10 MHz -> 0.1 us*/

#define striplen 5

/*******************
 *SK6812 Timings
 ********************/
#define T0H	3// T0H for SK6812 -> 0.3 us
#define T0L	9 // T0L for SK6812 -> 0.9 us
#define T1H	6 // T1H for SK6812 -> 0.6 us
#define T1L	6 // T1L for SK6812 -> 0.6 us
#define TRS	800 // TRES for SK6812 -> 80 us


rmt_item32_t pixels[] = {

     /* 0 */
                                // 8 Bit G
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},

    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */

    // 8 Bit R
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},

    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */

    // 8 Bit B
    {{{T1H, 1, T1L, 0}}},
    {{{T1H, 1, T1L, 0}}},
    {{{T1H, 1, T1L, 0}}},
    {{{T1H, 1, T1L, 0}}},
    {{{T1H, 1, T1L, 0}}},
    {{{T1H, 1, T1L, 0}}},
    {{{T1H, 1, T1L, 0}}},
    {{{T1H, 1, T1L, 0}}},

    /* // 8 Bit W */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */


    /* 1 */
    // 8 Bit G
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},

    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */

    // 8 Bit R
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},

    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */

    // 8 Bit B
    {{{T1H, 1, T1L, 0}}},
    {{{T1H, 1, T1L, 0}}},
    {{{T1H, 1, T1L, 0}}},
    {{{T1H, 1, T1L, 0}}},
    {{{T1H, 1, T1L, 0}}},
    {{{T1H, 1, T1L, 0}}},
    {{{T1H, 1, T1L, 0}}},
    {{{T1H, 1, T1L, 0}}},

    /* // 8 Bit W */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */

    /* 2 */
    // 8 Bit G
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},

    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */

    // 8 Bit R
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},

    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */

    // 8 Bit B
    {{{T1H, 1, T1L, 0}}},
    {{{T1H, 1, T1L, 0}}},
    {{{T1H, 1, T1L, 0}}},
    {{{T1H, 1, T1L, 0}}},
    {{{T1H, 1, T1L, 0}}},
    {{{T1H, 1, T1L, 0}}},
    {{{T1H, 1, T1L, 0}}},
    {{{T1H, 1, T1L, 0}}},

    /* // 8 Bit W */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */


    /* 3 */
    // 8 Bit G
    /* {{{T0H, 1, T0L, 0}}}, */
    /* {{{T0H, 1, T0L, 0}}}, */
    /* {{{T0H, 1, T0L, 0}}}, */
    /* {{{T0H, 1, T0L, 0}}}, */
    /* {{{T0H, 1, T0L, 0}}}, */
    /* {{{T0H, 1, T0L, 0}}}, */
    /* {{{T0H, 1, T0L, 0}}}, */
    /* {{{T0H, 1, T0L, 0}}}, */

    {{{T1H, 1, T1L, 0}}},
    {{{T1H, 1, T1L, 0}}},
    {{{T1H, 1, T1L, 0}}},
    {{{T1H, 1, T1L, 0}}},
    {{{T1H, 1, T1L, 0}}},
    {{{T1H, 1, T1L, 0}}},
    {{{T1H, 1, T1L, 0}}},
    {{{T1H, 1, T1L, 0}}},

    // 8 Bit R
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},

    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */

    // 8 Bit B
    {{{T1H, 1, T1L, 0}}},
    {{{T1H, 1, T1L, 0}}},
    {{{T1H, 1, T1L, 0}}},
    {{{T1H, 1, T1L, 0}}},
    {{{T1H, 1, T1L, 0}}},
    {{{T1H, 1, T1L, 0}}},
    {{{T1H, 1, T1L, 0}}},
    {{{T1H, 1, T1L, 0}}},

    /* // 8 Bit W */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */


    /* 4 */
    // 8 Bit G
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},

    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */

    // 8 Bit R
    /* {{{T0H, 1, T0L, 0}}}, */
    /* {{{T0H, 1, T0L, 0}}}, */
    /* {{{T0H, 1, T0L, 0}}}, */
    /* {{{T0H, 1, T0L, 0}}}, */
    /* {{{T0H, 1, T0L, 0}}}, */
    /* {{{T0H, 1, T0L, 0}}}, */
    /* {{{T0H, 1, T0L, 0}}}, */
    /* {{{T0H, 1, T0L, 0}}}, */

    {{{T1H, 1, T1L, 0}}},
    {{{T1H, 1, T1L, 0}}},
    {{{T1H, 1, T1L, 0}}},
    {{{T1H, 1, T1L, 0}}},
    {{{T1H, 1, T1L, 0}}},
    {{{T1H, 1, T1L, 0}}},
    {{{T1H, 1, T1L, 0}}},
    {{{T1H, 1, T1L, 0}}},

    // 8 Bit B
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},
    {{{T0H, 1, T0L, 0}}},

    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */

    /* // 8 Bit W */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */
    /* {{{T1H, 1, T1L, 0}}}, */

    {{{1, 1, TRS, 0}}}, // <- When the 1 is set to 0 ticks, it stops working

    // RMT end marker
    {{{ 0, 0, 0, 0 }}}

};


/*
 * @brief Initialize each RMT_Channel
 * @param [in] number of Channels to be used. GPIO starting at pin 12
 */

static bool initPixels(uint8_t numChan)
{
    rmt_config_t config[numChan];
    for (uint8_t i=0; i<numChan; i++) {

        config[i].rmt_mode = RMT_MODE_TX;
        config[i].channel = i; // Could be defined via the enumeration: rmt_channel_t
        config[i].gpio_num = (RMT_TX_GPIO+i);
        config[i].mem_block_num = 1;
        config[i].tx_config.loop_en = false;
        config[i].tx_config.carrier_en = false; // disable carrier
        config[i].tx_config.idle_output_en = true; // activate output while idle
        config[i].tx_config.idle_level = (rmt_idle_level_t)0; // output level to 0 when idle
        config[i].tx_config.carrier_duty_percent = 50; // must be set to prevent errors - not used
        config[i].tx_config.carrier_freq_hz = 10000; // must be set to prevent errors - not used
        config[i].tx_config.carrier_level = (rmt_carrier_level_t)0; // must be set to prevent errors - not used
        config[i].clk_div = DIVIDER;

        ESP_ERROR_CHECK(rmt_config(&config[i]));
        ESP_ERROR_CHECK(rmt_driver_install(config[i].channel, 0, 0));
    } // end for

    return 1;
}

void app_main(void *ignore)
{
    if (initPixels(1)) {
        while (1) {
            ESP_ERROR_CHECK(rmt_write_items(RMT_TX_CHANNEL1, pixels, (striplen*24+2), false));
            ESP_ERROR_CHECK(rmt_wait_tx_done(RMT_TX_CHANNEL1, portMAX_DELAY));
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
        vTaskDelete(NULL);
    }

}
#endif
