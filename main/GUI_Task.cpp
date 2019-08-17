#include <string.h>

#include "Arduino.h"
#include "Common.h"

/*
#include "../components/arduino/libraries/TFT_eSPI-master/TFT_eSPI.h"
#include "../components/arduino/libraries/TFT_eSPI-master/examples/320 x 240/All_Free_Fonts_Demo/Free_Fonts.h" // Include the header file attached to this sketch
*/

#include "../components/TFT_eSPI-master/TFT_eSPI.h"
#include "../components/TFT_eSPI-master/examples/320 x 240/All_Free_Fonts_Demo/Free_Fonts.h" // Include the header file attached to this sketch

#include "tempcare_logo.h"
#include <string.h>

#include "driver/i2c.h"
#include <tcpip_adapter.h>

//--------------------------------------------------------------------------------------------
TFT_eSPI tft = TFT_eSPI(); // Invoke custom library
//--------------------------------------------------------------------------------------------

extern uint8_t WIFI_MAC_STA[6];
extern uint8_t WIFI_MAC_AP[6];
extern bool POWER_SW_ON_STATE;
extern bool isBatteryPowered;

extern bool HTTP_STATUS_UPLOAD_OK;
extern bool HTTP_STATUS_ERROR;
extern bool HTTP_DNS_QUERY_ERROR;

extern EventGroupHandle_t wifi_event_group;
//--------------------------------------------------------------------------------------------
typedef enum
{
    GUI_STATE_NORMAL,
    GUI_STATE_ERROR_STAT,
    GUI_STATE_SENSOR_STAT
} GUI_STATES;

static char temp_str[256];
GUI_STATES CURRENT_GUI_STATE = GUI_STATE_NORMAL;

// 'empty-battery'
extern const unsigned char bmp_battery_empty[] PROGMEM;
// 'charging-battery'
extern const unsigned char bmp_battery_charging[] PROGMEM;
// 'full-battery'
extern const unsigned char bmp_battery_full[] PROGMEM;
// 'battery-almost-full'
extern const unsigned char bmp_battery_almost_full[] PROGMEM;
// 'low-battery'
extern const unsigned char bmp_battery_low[] PROGMEM;
// 'half-battery'
extern const unsigned char bmp_battery_half[] PROGMEM;

extern float batt_percent;
extern int batt_charging;

//--------------------------------------------------------------------------------------------
void GUI_State_Normal();

bool CheckStateChange();
void UpdateLCD();
void UpdateLED();

void IRAM_ATTR WriteRegPCA9555(uint8_t address, uint8_t data);
uint8_t IRAM_ATTR ReadRegPCA9555(uint8_t address);
//--------------------------------------------------------------------------------------------
IRAM_ATTR void GUI_Task(void *param)
{
    UpdateLED();

    tft.begin();
    tft.setRotation(1);

    tft.drawBitmap(0, 0, tempCareLogo, 320, 240, TFT_BLACK);

    if (POWER_SW_ON_STATE == true)
    {
        gpio_set_level(GPIO_NUM_33, 1);
    }

    if (isBatteryPowered == 1)
    {
        gpio_set_level(GPIO_NUM_33, 0);
    }

    vTaskDelay(400);

    WriteRegPCA9555(0x06, 0x00 | 1 << 0 | 1 << 1 | 1 << 2 | 1 << 3 | 1 << 4);
    WriteRegPCA9555(0x07, 0x00);

    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE, TFT_NAVY);
    tft.fillRect(0, 0, 320, 30, TFT_NAVY);
    tft.setTextDatum(TC_DATUM);
    tft.drawString(CONFIG_DATA.ModelName, 160, 2, 4); // Font 4 for fast drawing with background

    while (true)
    {
        switch (CURRENT_GUI_STATE)
        {
        case GUI_STATE_NORMAL:
        {
            GUI_State_Normal();
            break;
        }
        default:
        {
            break;
        }
        }
    }
}

IRAM_ATTR void GUI_State_Normal()
{
    tft.fillRect(0, 30, 320, 240, TFT_BLACK);

    tft.drawFastHLine(0, 180, 320, TFT_WHITE);

    setenv("TZ", "KST-9", 1);
    tzset();

    tft.setTextDatum(TL_DATUM);

    tft.setFreeFont(FF34); // Select Free Serif 12 point font
    tft.setTextColor(TFT_WHITE, TFT_BLACK);

    /*
	sprintf(temp_str, "Internal Temp. : ");
	tft.drawString(temp_str, 5, 35, GFXFF);
	sprintf(temp_str, "Internal Hum.   : ");
	tft.drawString(temp_str, 5, 55, GFXFF);
	*/

    /*
	sprintf(temp_str, "  Temp. CH1      :");
	tft.drawString(temp_str, 30, 90, GFXFF);
	sprintf(temp_str, "  Temp. CH2      :");
	tft.drawString(temp_str, 30, 110, GFXFF);
	*/

    tft.setTextDatum(TC_DATUM);

    /*
    sprintf(temp_str, "INTERNAL SENSOR");
    tft.drawString(temp_str, 160, 32, 2);

    tft.drawRect(20, 50, 280, 25, TFT_LIGHTGREY);
    tft.drawFastVLine(160, 50, 25, TFT_LIGHTGREY);
    */
    //if (CONFIG_DATA.RS485_SensorTypes[0] == RS485_SENSOR_CO2)

    if (CONFIG_DATA.AM2305_Enabled == false)
    {
        if (CONFIG_DATA.RS485_SensorTypes[0] == RS485_SENSOR_CO2_O2)
        {
            sprintf(temp_str, "CO2 Incubator Sensor");
            tft.drawString(temp_str, 160, 32, 2);

            tft.drawRect(20, 50, 280, 25, TFT_LIGHTGREY);
            tft.drawFastVLine(160, 50, 25, TFT_LIGHTGREY);
        }
    }
    else
    {
        sprintf(temp_str, "External Temp. / Hum.");
        tft.drawString(temp_str, 160, 32, 2);

        tft.drawRect(20, 50, 280, 25, TFT_LIGHTGREY);
        tft.drawFastVLine(160, 50, 25, TFT_LIGHTGREY);
    }
    if (CONFIG_DATA.RS485_SensorTypes[0] == RS485_SENSOR_LN3)
    {
        //sprintf(temp_str, "LN2 Level(inch) / LN2 Level(%%)");
        sprintf(temp_str, "LN2 Level(inch)                  ");
        tft.drawString(temp_str, 160, 32, 2);

        tft.drawRect(20, 50, 280, 25, TFT_LIGHTGREY);
        tft.drawFastVLine(160, 50, 25, TFT_LIGHTGREY);
    }

    tft.setTextDatum(TC_DATUM);

    if (CONFIG_DATA.MainSensorTypes[0] == MAIN_SENSOR_PT100)
        sprintf(temp_str, "CH1 : PT100");
    else if (CONFIG_DATA.MainSensorTypes[0] == MAIN_SENSOR_NTC)
        sprintf(temp_str, "CH1 : NTC");
    else if (CONFIG_DATA.MainSensorTypes[0] == MAIN_SENSOR_AM2305_T)
        sprintf(temp_str, "CH1 : AM2305 (T)");
    else if (CONFIG_DATA.MainSensorTypes[0] == MAIN_SENSOR_AM2305_H)
        sprintf(temp_str, "CH1 : AM2305 (H)");
    else if (CONFIG_DATA.MainSensorTypes[0] == MAIN_SENSOR_DISABLED)
        sprintf(temp_str, "CH1 : N/A");
    if (CONFIG_DATA.RS485_SensorTypes[0] == RS485_SENSOR_LN3)
    {
        sprintf(temp_str, "CH1 : Temp A");
    }
    tft.drawString(temp_str, 70, 90, 2);

    if (CONFIG_DATA.MainSensorTypes[1] == MAIN_SENSOR_PT100)
        sprintf(temp_str, "CH2 : PT100");
    else if (CONFIG_DATA.MainSensorTypes[1] == MAIN_SENSOR_NTC)
        sprintf(temp_str, "CH2 : NTC");
    else if (CONFIG_DATA.MainSensorTypes[1] == MAIN_SENSOR_AM2305_T)
        sprintf(temp_str, "CH2 : AM2305 (T)");
    else if (CONFIG_DATA.MainSensorTypes[1] == MAIN_SENSOR_AM2305_H)
        sprintf(temp_str, "CH2 : AM2305 (H)");
    else if (CONFIG_DATA.MainSensorTypes[1] == MAIN_SENSOR_DISABLED)
        sprintf(temp_str, "CH2 : N/A");
    if (CONFIG_DATA.RS485_SensorTypes[0] == RS485_SENSOR_LN3)
    {
        sprintf(temp_str, "CH2 : Temp B");
    }

    tft.drawString(temp_str, 230, 90, 2);

    tft.setTextDatum(TL_DATUM);

    tft.drawRect(0, 110, 320, 60, TFT_LIGHTGREY);
    tft.drawFastVLine(160, 110, 60, TFT_LIGHTGREY);

    int second_cnt = 0;

    while (true)
    {
        second_cnt++;

        UpdateLCD();
        UpdateLED();

        if (POWER_SW_ON_STATE)
        {
            gpio_set_level(GPIO_NUM_33, 1);
        }
        else
        {
            gpio_set_level(GPIO_NUM_33, 0);
        }
        if (isBatteryPowered)
        {
            gpio_set_level(GPIO_NUM_33, 0);
        }

        for (int i = 0; i < 100; i++)
        {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }

        tft.fillRect(300, 220, 16, 16, TFT_BLACK);

        if (batt_charging && (second_cnt % 2 == 0))
        {
            tft.drawBitmap(300, 220, bmp_battery_charging, 16, 16, TFT_WHITE);
        }
        else
        {
            if (batt_percent >= 95)
            {
                tft.drawBitmap(300, 220, bmp_battery_full, 16, 16, TFT_WHITE);
            }
            else if (batt_percent > 70 && batt_percent <= 95)
            {
                tft.drawBitmap(300, 220, bmp_battery_almost_full, 16, 16, TFT_WHITE);
            }
            else if (batt_percent > 20 && batt_percent <= 70)
            {
                tft.drawBitmap(300, 220, bmp_battery_half, 16, 16, TFT_WHITE);
            }
            else if (batt_percent > 10 && batt_percent <= 20)
            {
                tft.drawBitmap(300, 220, bmp_battery_low, 16, 16, TFT_WHITE);
            }
            else if (batt_percent <= 10)
            {
                tft.drawBitmap(300, 220, bmp_battery_empty, 16, 16, TFT_WHITE);
            }
        }

        if (CheckStateChange() == true)
            return;
    }
}

IRAM_ATTR bool CheckStateChange()
{

    return false;
}

IRAM_ATTR void UpdateLED()
{
    uint8_t temp;
    uint8_t temp2;

    static uint8_t prev_temp = 0x00;
    static uint8_t prev_temp2 = 0x00;

    // P0.5 : R --> LED_CPU  --> ERROR
    // P0.6 : G
    // P0.7 : B

    // P1.0 : R --> LED_NET  --> DATA
    // P1.1 : G
    // P1.2 : B

    // P1.3 : R --> LED_STAT --> POWER
    // P1.4 : G
    // P1.5 : B

    temp = 0xFF;
    temp2 = 0xFF;

    // POWER LED
    if (POWER_SW_ON_STATE)
    {
        if (isBatteryPowered)
        {
            // YELLOW
            temp &= ~(1 << 3);
            temp &= ~(1 << 4);
        }
        else
        {
            // GREEN
            temp &= ~(1 << 4);
        }
    }

    if (POWER_SW_ON_STATE)
    {
        //if (WiFi.status() == WL_CONNECTED)
        if (xEventGroupGetBits(wifi_event_group) & WIFI_WIFI_AP_CONNECTED_BIT)
        {
            temp |= (1 << 0);

            if (HTTP_STATUS_UPLOAD_OK)
                temp &= ~(1 << 2);
            else
            {
                if (isBatteryPowered == false)
                {
                    temp &= ~(1 << 0);
                    temp &= ~(1 << 1);
                    temp &= ~(1 << 2);
                }
            }
        }
        else
        {
            temp &= ~(1 << 0);
        }

        if (HTTP_STATUS_ERROR)
            temp2 &= ~(1 << 5);
        else
            temp2 |= (1 << 5);
    }

    if (prev_temp != temp)
    {
        //WriteRegPCA9555(0x03, 0xFF);
        WriteRegPCA9555(0x03, temp);
        prev_temp = temp;
    }

    if (prev_temp2 != temp2)        
    {
        //WriteRegPCA9555(0x02, 1 << 5 | 1 << 6 | 1 << 7);
        WriteRegPCA9555(0x02, temp2);
        prev_temp2 = temp2;
    }
}

static float prev_level = -9999.9;
static float prev_levelp = -9999.9;

IRAM_ATTR void UpdateLCD()
{
    //if (WiFi.isConnected())
    {
        tft.setTextColor(TFT_WHITE, TFT_BLACK);

        tft.setFreeFont(FF17);
        tft.setTextDatum(TC_DATUM);

        if (CONFIG_DATA.AM2305_Enabled == true && CONFIG_DATA.RS485_SensorTypes[0] != RS485_SENSOR_LN3)
        {
            if (AM2301_Hum < 9999)
                sprintf(temp_str, "   %3.1f'C   ", AM2301_Temp);
            else
                sprintf(temp_str, "      ---      ");
            tft.drawString(temp_str, 85, 55, GFXFF);

            if (AM2301_Hum < 9999)
                sprintf(temp_str, "   %3.1f%%   ", AM2301_Hum);
            else
                sprintf(temp_str, "      ---      ");
            tft.drawString(temp_str, 230, 55, GFXFF);
        }
        else
        {
            if (CONFIG_DATA.RS485_SensorTypes[0] == RS485_SENSOR_CO2_O2)
            {
                if (CO2_Value[0] >= 0)
                    sprintf(temp_str, "   %3.3f%%   ", CO2_Value[0] / 1000.0f);
                else
                    sprintf(temp_str, "      ---      ");
                tft.drawString(temp_str, 85, 55, GFXFF);

                if (CO2_Value[1] >= 0)
                    sprintf(temp_str, "   %3.3f%%   ", CO2_Value[1] / 1000.0f);
                else
                    sprintf(temp_str, "      ---      ");
                tft.drawString(temp_str, 230, 55, GFXFF);
            }
            else if (CONFIG_DATA.RS485_SensorTypes[0] != RS485_SENSOR_LN3)
            {
                sprintf(temp_str, "      ---      ");
                tft.drawString(temp_str, 85, 55, GFXFF);
                tft.drawString(temp_str, 230, 55, GFXFF);
            }
        }
        if (CONFIG_DATA.RS485_SensorTypes[0] == RS485_SENSOR_LN3)
        {
            //Serial.printf("prev_level = %f, level = %f\r\n", prev_level, LN3_Level[0]);

            if (LN3_Level[0] >= 0)
                sprintf(temp_str, "   %3.1f   ", LN3_Level[0]);
            else
                sprintf(temp_str, "      ---      ");

            if (prev_level != LN3_Level[0])
            {
                //Serial.printf("[UPDATE!!!]\r\n");
                prev_level = LN3_Level[0];
                tft.drawString(temp_str, 85, 55, GFXFF);
            }
    
            /*
            if (LN3_LevelPercent[0] >= 0)
                sprintf(temp_str, "   %3.1f%%   ", LN3_LevelPercent[0]);
            else
                sprintf(temp_str, "      ---      ");

            if (prev_levelp != LN3_LevelPercent[0])
            {
                prev_levelp = LN3_LevelPercent[0];
                tft.drawString(temp_str, 230, 55, GFXFF);
            }*/

        }

        tft.setTextColor(TFT_RED, TFT_BLACK);
        tft.setTextDatum(TL_DATUM);

        float t = 9999.9;

        if (CONFIG_DATA.MainSensorTypes[0] == MAIN_SENSOR_PT100)
            t = PT100_Temp[0];
        else if (CONFIG_DATA.MainSensorTypes[0] == MAIN_SENSOR_NTC)
            t = NTC_Temp[0];
        else if (CONFIG_DATA.MainSensorTypes[0] == MAIN_SENSOR_AM2305_T)
            t = AM2301_Temp;
        else if (CONFIG_DATA.MainSensorTypes[0] == MAIN_SENSOR_AM2305_H)
            t = AM2301_Hum;
        else if (CONFIG_DATA.MainSensorTypes[0] == MAIN_SENSOR_DISABLED)
            t = 9999.9;
        else
            t = 9999.9;
        if (CONFIG_DATA.RS485_SensorTypes[0] == RS485_SENSOR_LN3)
            t = LN3_TempA[0];

        if (t > -100.0f)
        {
            sprintf(temp_str, "%3.1f  ", t);
        }
        else
        {
            sprintf(temp_str, "%3i ", (int)t);
        }
        if (t > 999)
        {
            sprintf(temp_str, "  -----  ");
        }
        //tft.drawString(temp_str, 250, 90, GFXFF);
        tft.drawString(temp_str, 10, 115, 7);

        if (CONFIG_DATA.MainSensorTypes[1] == MAIN_SENSOR_PT100)
            t = PT100_Temp[1];
        else if (CONFIG_DATA.MainSensorTypes[1] == MAIN_SENSOR_NTC)
            t = NTC_Temp[1];
        else if (CONFIG_DATA.MainSensorTypes[1] == MAIN_SENSOR_AM2305_T)
            t = AM2301_Temp;
        else if (CONFIG_DATA.MainSensorTypes[1] == MAIN_SENSOR_AM2305_H)
            t = AM2301_Hum;
        else if (CONFIG_DATA.MainSensorTypes[1] == MAIN_SENSOR_DISABLED)
            t = 9999.9;
        else
            t = 9999.9;

        if (CONFIG_DATA.RS485_SensorTypes[0] == RS485_SENSOR_LN3)
            t = LN3_TempB[0];

        if (t > -100.0f)
        {
            sprintf(temp_str, "%3.1f    ", t);
        }
        else
        {
            sprintf(temp_str, "%3i   ", (int)t);
        }
        if (t > 999)
        {
            sprintf(temp_str, "  -----  ");
        }

        //tft.drawString(temp_str, 250, 110, GFXFF);
        tft.drawString(temp_str, 170, 115, 7);
        tft.drawFastHLine(0, 180, 320, TFT_WHITE);

        tft.drawFastVLine(160, 110, 60, TFT_LIGHTGREY);

        tft.setTextDatum(TL_DATUM);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setFreeFont(FSS9);
        sprintf(temp_str, "SSID: %s", (char *)CONFIG_DATA.WIFI_STA_SSID);
        tft.drawString(temp_str, 5, 185, 2);

        char ipv4_str[32];
        tcpip_adapter_ip_info_t if_info;
        tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &if_info);
        ip4addr_ntoa_r(&(if_info.ip), ipv4_str, 32);

        //sprintf(temp_str, "IP: %s                  ", WiFi.localIP().toString().c_str());
        sprintf(temp_str, "IP: %s                  ", ipv4_str);
        tft.drawString(temp_str, 160, 185, 2);

        sprintf(temp_str,
                "MAC: %02X%02X%02X%02X%02X%02X",
                WIFI_MAC_STA[0],
                WIFI_MAC_STA[1],
                WIFI_MAC_STA[2],
                WIFI_MAC_STA[3],
                WIFI_MAC_STA[4],
                WIFI_MAC_STA[5]);
        tft.drawString(temp_str, 5, 200, 2);

        wifi_ap_record_t wifidata;
        esp_wifi_sta_get_ap_info(&wifidata);
        sprintf(temp_str, "RSSI: %03i  ", wifidata.rssi);
        tft.drawString(temp_str, 160, 200, 2);

        time_t now = 0;
        struct tm timeinfo;
        time(&now);
        localtime_r(&now, &timeinfo);
        char strftime_buf[64];
        strftime(strftime_buf, sizeof(strftime_buf), "%F %H:%M:%S", &timeinfo);

        sprintf(temp_str,
                "[%s]",
                strftime_buf);
        tft.drawString(temp_str, 5, 220, 2);

        if (CONFIG_DATA.CON1_Enabled)
        {
            sprintf(temp_str, "[CON1]");
            if (CONT_Value[0] == true)
                tft.setTextColor(TFT_WHITE, TFT_RED);
            else
                tft.setTextColor(TFT_WHITE, TFT_BLACK);

            tft.drawString(temp_str, 160, 220, 2);
            tft.setTextColor(TFT_WHITE, TFT_BLACK);
        }

        if (CONFIG_DATA.CON2_Enabled)
        {
            sprintf(temp_str, "[CON2]");
            if (CONT_Value[1] == true)
                tft.setTextColor(TFT_WHITE, TFT_RED);
            else
                tft.setTextColor(TFT_WHITE, TFT_BLACK);

            tft.drawString(temp_str, 220, 220, 2);
            tft.setTextColor(TFT_WHITE, TFT_BLACK);
        }
    }
    //else
    {
        //gpio_set_level(GPIO_NUM_33, 0);
    }
}
// --------------------------------------------------------------------
#define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ   /*!< I2C master read */
#define ACK_CHECK_EN 0x1           /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0          /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                /*!< I2C ack value */
#define NACK_VAL 0x1               /*!< I2C nack value */

//--------------------------------------------------------------------------------------------
void IRAM_ATTR WriteRegPCA9555(uint8_t address, uint8_t data)
{
    i2c_cmd_handle_t cmd;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x20) << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, address, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin((i2c_port_t)I2C_NUM_0, cmd, 10 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
}

uint8_t IRAM_ATTR ReadRegPCA9555(uint8_t address)
{
    i2c_cmd_handle_t cmd;

    uint8_t data;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x20) << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, address, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &data, I2C_MASTER_ACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin((i2c_port_t)I2C_NUM_0, cmd, 10 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return data;
}

//--------------------------------------------------------------------------------------------
void IRAM_ATTR ExternalDisplay_Task(void *param)
{
    while (true)
    {
        vTaskDelay(1);
    }
}

// 'empty-battery'
const unsigned char bmp_battery_empty[] PROGMEM =
    {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xfe, 0x80, 0x06, 0x80, 0x07, 0x80, 0x07,
        0x80, 0x07, 0x80, 0x07, 0x80, 0x06, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// 'charging-battery'
const unsigned char bmp_battery_charging[] PROGMEM = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xfe, 0x80, 0x06, 0x80, 0x87, 0x81, 0x07,
    0x81, 0x07, 0x82, 0x07, 0x80, 0x06, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// 'full-battery'
const unsigned char bmp_battery_full[] PROGMEM = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xfe, 0x80, 0x06, 0xb6, 0x97, 0xb6, 0x97,
    0xb6, 0x97, 0xb6, 0x97, 0x80, 0x06, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
// 'battery-almost-full'
const unsigned char bmp_battery_almost_full[] PROGMEM = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xfe, 0x80, 0x06, 0xb6, 0x87, 0xb6, 0x87,
    0xb6, 0x87, 0xb6, 0x87, 0x80, 0x06, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
// 'low-battery'
const unsigned char bmp_battery_low[] PROGMEM = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xfe, 0x80, 0x06, 0xb0, 0x07, 0xb0, 0x07,
    0xb0, 0x07, 0xb0, 0x07, 0x80, 0x06, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
// 'half-battery'
const unsigned char bmp_battery_half[] PROGMEM = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xfe, 0x80, 0x06, 0xb6, 0x07, 0xb6, 0x07,
    0xb6, 0x07, 0xb6, 0x07, 0x80, 0x06, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
