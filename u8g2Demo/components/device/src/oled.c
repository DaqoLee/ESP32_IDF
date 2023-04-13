/*
 * @Author: xmprocat
 * @Date: 2023-02-19 22:33:18
 * @LastEditors: xmprocat
 * @LastEditTime: 2023-02-20 23:04:36
 * @Description: 
 */
#include"oled.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "i2cconfig.h"


#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */


static const char *TAG = "u8g2";


void esp32_i2c_write(uint8_t addr, uint32_t idx, uint8_t *data)
{
    i2c_cmd_handle_t handler = i2c_cmd_link_create();
    i2c_master_start(handler);
    i2c_master_write_byte(handler, addr | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(handler, data, idx, 2);
    i2c_master_stop(handler);
    i2c_master_cmd_begin(I2C_NUM_0, handler, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(handler);
}

// u8g2用到的系统资源
uint8_t u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    switch (msg)
    {
    case U8X8_MSG_GPIO_AND_DELAY_INIT:
        i2c_master_init();          //调用iic初始化
	    break;
    case U8X8_MSG_DELAY_MILLI:
        vTaskDelay(arg_int);
        break;
    default:
        return 0;
    }
    return 1;
}
// u8g2用到的显示屏控制接口
uint8_t u8x8_byte_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    static uint8_t buffer[32]; /* u8g2/u8x8 will never send more than 32 bytes between START_TRANSFER and END_TRANSFER */
    static uint8_t buf_idx;
    uint8_t *data;

    switch (msg)
    {
    case U8X8_MSG_BYTE_SEND:
        data = (uint8_t *)arg_ptr;
        while (arg_int > 0)
        {
            buffer[buf_idx++] = *data;
            data++;
            arg_int--;
        }
        break;
    case U8X8_MSG_BYTE_INIT:
        /* add your custom code to init i2c subsystem */
        break;
    case U8X8_MSG_BYTE_SET_DC:
        /* ignored for i2c */
        break;
    case U8X8_MSG_BYTE_START_TRANSFER:
        buf_idx = 0;
        break;
    case U8X8_MSG_BYTE_END_TRANSFER:
        esp32_i2c_write(u8x8_GetI2CAddress(u8x8), buf_idx, buffer);
        break;
    default:
        return 0;
    }
    return 1;
}

void u8g2Init(u8g2_t *u8g2)
{
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(u8g2, U8G2_R0, u8x8_byte_i2c, u8x8_gpio_and_delay); // 初始化 u8g2 结构体
    u8g2_InitDisplay(u8g2);                                                                    // 根据所选的芯片进行初始化工作，初始化完成后，显示器处于关闭状态
    u8g2_SetPowerSave(u8g2, 0);                                                                // 打开显示器
    u8g2_ClearBuffer(u8g2);
    u8g2_SendBuffer(u8g2);		// 清屏
    ESP_LOGI(TAG, "u8g2 init done");
}
