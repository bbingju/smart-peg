#include "sensors.h"
#include "esp_log.h"
#include "driver/i2c.h"

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

static const char *TAG = "sensors";

struct sensor_addr_tlb {
    int id;
    uint8_t address;
};

#define SENSOR_NUMBER 7

struct sensor_addr_tlb sensor_addr_tlb[SENSOR_NUMBER] = {
    { 0, 0x20 },
    { 1, 0x21 },
    { 2, 0x22 },
    { 3, 0x23 },
    { 4, 0x24 },
    { 5, 0x25 },
    { 6, 0x26 },
};

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

static uint16_t sensor_read(int id)
{
    struct sensor_addr_tlb *r = &sensor_addr_tlb[id];
    uint8_t data_h, data_l;
    ESP_ERROR_CHECK(pca9535_read_register(r->address, &data_h, &data_l));

    data_h = ~(data_h);
    data_l = ~(data_l);

    return ((data_h << 8) | (data_l));
}

esp_err_t sensors_init( )
{
    return i2c_master_init();
}

static struct sensors_data sd = { 0 };

struct sensors_data * sensors_read()
{
    struct sensors_data d = { 0 };
    for (int i = 0; i < SENSOR_NUMBER; i++) {
        uint16_t v = sensor_read(i);
        /* ESP_LOGI(TAG, "read sensor #%d data: 0x%04x", i, v); */

        switch (i) {
        case 0:
            d.data[0] = (v & 0b0000001111111111);       /* 0x03FF */
            d.data[1] = (v & 0b1111110000000000) >> 10; /* 0xFC00 */
            break;
        case 1:
            d.data[1] |= (v & 0b0000000000001111) << 6;  /* 0x000F */
            d.data[2]  = (v & 0b0011111111110000) >> 4;  /* 0x3FF0 */
            d.data[3]  = (v & 0b1100000000000000) >> 14; /* 0xC000 */
            break;
        case 2:
            d.data[3] |= (v & 0b0000000011111111) << 2;
            d.data[4]  = (v & 0b1111111100000000) >> 8;
            break;
        case 3:
            d.data[4] |= (v & 0b0000000000000011) << 8;
            d.data[5]  = (v & 0b0000111111111100) >> 2;
            d.data[6]  = (v & 0b1111000000000000) >> 12;
            break;
        case 4:
            d.data[6] |= (v & 0b0000000000111111) << 4;
            d.data[7]  = (v & 0b1111111111000000) >> 6;
            break;
        case 5:
            d.data[8]  = (v & 0b0000001111111111);       /* 0x03FF */
            d.data[9]  = (v & 0b1111110000000000) >> 10; /* 0xFC00 */
            break;
        case 6:
            d.data[9] |= (v & 0b0000000000001111) << 6; /* 0x000F */
            break;
        default:
            break;
        }
    }

    sd = d;

    /* ESP_LOGI(TAG, "\nrow #0: 0x%04x\nrow #1: 0x%04x\nrow #2: 0x%04x\nrow #3: 0x%04x\nrow #4: 0x%04x\n" */
    /*          "row #5: 0x%04x\nrow #6: 0x%04x\nrow #7: 0x%04x\nrow #8: 0x%04x\nrow #9: 0x%04x", */
    /*          d.data[0],d.data[1],d.data[2],d.data[3],d.data[4],d.data[5],d.data[6],d.data[7],d.data[8],d.data[9]); */

    return &sd;
}
