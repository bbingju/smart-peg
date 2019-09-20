#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "event_source.h"
#include "esp_event_base.h"
#include "driver/gpio.h"

#include "time.h"
#include "sys/time.h"

#include "leds.h"
#include "sensors.h"
#include "cmd_parser.h"

static const char *TAG = "smart-peg";

#define SPP_SERVER_NAME "SMART_PEG_SPP_SERVER"
#define DEVICE_NAME "SMART_PEG"
#define SPP_SHOW_DATA 0
#define SPP_SHOW_SPEED 1
//#define SPP_SHOW_MODE SPP_SHOW_SPEED    /*Choose show mode: show data or speed*/
#define SPP_SHOW_MODE SPP_SHOW_DATA    /*Choose show mode: show data or speed*/

#define POWERON_GPIO   GPIO_NUM_16
#define BT_GPIO        GPIO_NUM_17
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<POWERON_GPIO) | (1ULL<<BT_GPIO))

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;

static uint32_t bt_connection_handle;

static struct timeval time_new, time_old;
static long data_num = 0;

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

static leds_handle_t leds;
static int leds_direct_draw = true;

esp_event_loop_handle_t app_loop;

/* Event source task related definitions */
ESP_EVENT_DEFINE_BASE(PEG_EVENTS);


static void print_speed(void)
{
    float time_old_s = time_old.tv_sec + time_old.tv_usec / 1000000.0;
    float time_new_s = time_new.tv_sec + time_new.tv_usec / 1000000.0;
    float time_interval = time_new_s - time_old_s;
    float speed = data_num * 8 / time_interval / 1000.0;
    ESP_LOGI(TAG, "speed(%fs ~ %fs): %f kbit/s" , time_old_s, time_new_s, speed);
    data_num = 0;
    time_old.tv_sec = time_new.tv_sec;
    time_old.tv_usec = time_new.tv_usec;
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event) {
    case ESP_SPP_INIT_EVT:
        ESP_LOGI(TAG, "ESP_SPP_INIT_EVT");
        esp_bt_dev_set_device_name(DEVICE_NAME);
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        esp_spp_start_srv(sec_mask,role_slave, 0, SPP_SERVER_NAME);
        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        ESP_LOGI(TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
        break;
    case ESP_SPP_OPEN_EVT:
        ESP_LOGI(TAG, "ESP_SPP_OPEN_EVT");
        break;
    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(TAG, "ESP_SPP_CLOSE_EVT");
        gpio_set_level(BT_GPIO, 0);
        break;
    case ESP_SPP_START_EVT:
        ESP_LOGI(TAG, "ESP_SPP_START_EVT");
        break;
    case ESP_SPP_CL_INIT_EVT:
        ESP_LOGI(TAG, "ESP_SPP_CL_INIT_EVT");
        break;
    case ESP_SPP_DATA_IND_EVT:
/* #if (SPP_SHOW_MODE == SPP_SHOW_DATA) */
/*         ESP_LOGI(TAG, "ESP_SPP_DATA_IND_EVT len=%d handle=%d", */
/*                  param->data_ind.len, param->data_ind.handle); */
/*         esp_log_buffer_hex("",param->data_ind.data,param->data_ind.len); */

/*         #else */
/*         gettimeofday(&time_new, NULL); */
/*         data_num += param->data_ind.len; */
/*         if (time_new.tv_sec - time_old.tv_sec >= 3) { */
/*             print_speed(); */
/*         } */
/* #endif */
        cmd_parser_eval((const char *) param->data_ind.data, param->data_ind.len);
        break;
    case ESP_SPP_CONG_EVT:
        ESP_LOGI(TAG, "ESP_SPP_CONG_EVT");
        break;
    case ESP_SPP_WRITE_EVT:
        ESP_LOGI(TAG, "ESP_SPP_WRITE_EVT");
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(TAG, "ESP_SPP_SRV_OPEN_EVT");
        gettimeofday(&time_old, NULL);
        bt_connection_handle = param->srv_open.handle;
        gpio_set_level(BT_GPIO, 1);
        break;
    default:
        break;
    }
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:{
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "authentication success: %s", param->auth_cmpl.device_name);
            esp_log_buffer_hex(TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        } else {
            ESP_LOGE(TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT:{
        ESP_LOGI(TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) {
            ESP_LOGI(TAG, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        } else {
            ESP_LOGI(TAG, "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %d", param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%d", param->key_notif.passkey);
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
        break;
#endif

    default: {
        ESP_LOGI(TAG, "event: %d", event);
        break;
    }
    }
    return;
}

static char* get_id_string(esp_event_base_t base, int32_t id) {
    char* event = "";
    if (base == PEG_EVENTS) {
        switch (id) {
        case PEG_EVENT_LED_SET_DIRECT_DRAW:
            event = "PEG_EVENT_LED_SET_DIRECT_DRAW";
            break;
        case PEG_EVENT_LED_DRAW:
            event = "PEG_EVENT_LED_DRAW";
            break;
        case PEG_EVENT_LED_CLEAR:
            event = "PEG_EVENT_LED_CLEAR";
            break;
        case PEG_EVENT_LED_SET_PIXEL:
            event = "PEG_EVENT_LED_SET_PIXEL";
            break;
        case PEG_EVENT_LED_FILL_RECT:
            event = "PEG_EVENT_LED_FILL_RECT";
            break;
        case PEG_EVENT_MAG_STATUS:
            event = "PEG_EVENT_MAG_STATUS";
            break;
        }
    }
    return event;
}

static void app_loop_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data)
{
    ESP_LOGI(TAG, "%s:%s: %s", base, get_id_string(base, id), __func__);

    switch (id) {

    case PEG_EVENT_LED_SET_DIRECT_DRAW: {
        struct peg_event_arg *d = event_data;
        leds_direct_draw = d->args[0] ? true : false;
        break;
    }
    case PEG_EVENT_LED_DRAW:
        leds_draw(NULL);
        break;
    case PEG_EVENT_LED_CLEAR:
        leds_clear(NULL);
        if (leds_direct_draw)
            leds_draw(NULL);
        break;
    case PEG_EVENT_LED_SET_PIXEL: {
        struct peg_event_arg *d = event_data;
        leds_set_color(NULL, (int) d->args[0], (int) d->args[1], (color_t) d->args[2]);
        if (leds_direct_draw)
            leds_draw(NULL);
        break;
    }
    case PEG_EVENT_LED_FILL_RECT: {
        struct peg_event_arg *d = event_data;
        leds_fill_rect(NULL, (int) d->args[0], (int) d->args[1], (int) d->args[2], (int) d->args[3], (color_t) d->args[4]);
        if (leds_direct_draw)
            leds_draw(NULL);
        break;
    }
    case PEG_EVENT_MAG_STATUS: {
        struct sensors_data *d =  sensors_read();
        char buffer[100] = { 0 };
        sprintf(buffer, "(mag_status,0x%04x,0x%04x,0x%04x,0x%04x,0x%04x,0x%04x,0x%04x,0x%04x,0x%04x,0x%04x,0x%04x)\r\n",
                d->data[0],d->data[1],d->data[2],d->data[3],d->data[4],
                d->data[5],d->data[6],d->data[7],d->data[8],d->data[9],d->data[10]);
        esp_spp_write(bt_connection_handle, strlen(buffer), (uint8_t *) buffer);
        break;
    }

    }
}

void app_main()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    gpio_set_level(POWERON_GPIO, 1);
    gpio_set_level(BT_GPIO, 0);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
        ESP_LOGE(TAG, "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
        ESP_LOGE(TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_init(esp_spp_mode)) != ESP_OK) {
        ESP_LOGE(TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    /* Set default parameters for Secure Simple Pairing */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif

    /*
     * Set default parameters for Legacy Pairing
     * Use variable pin, input pin code when pairing
     */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);

    cmd_parser_init();

    /*
     * Initialize LEDs & Sensors
     */
    leds = leds_create(DEFAULT_PIXEL_WIDTH, DEFAULT_PIXEL_HEIGHT);
    ESP_ERROR_CHECK(sensors_init());

    esp_event_loop_args_t app_loop_args = {
        .queue_size = 5,
        .task_name = "app_loop_task",
        .task_priority = uxTaskPriorityGet(NULL),
        .task_stack_size = 2048,
        .task_core_id = tskNO_AFFINITY
    };

    ESP_ERROR_CHECK(esp_event_loop_create(&app_loop_args, &app_loop));
    ESP_ERROR_CHECK(esp_event_handler_register_with(app_loop, PEG_EVENTS, ESP_EVENT_ANY_ID, app_loop_handler, app_loop));
}
