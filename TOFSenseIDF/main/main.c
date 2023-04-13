/* i2c - Simple example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a MPU9250 inertial measurement unit.

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "oled.h"
#include "button.h"
#include "tofsense.h"
#include "i2cconfig.h"

static const char *TAG = "tofsense";



void app_main(void)
{
    uint8_t mark = 0;
    gpio_reset_pin(14);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(14, GPIO_MODE_OUTPUT);
    gpio_set_level(14,1);
   // uint8_t data[2];
    ESP_ERROR_CHECK(i2c_master_init());//i2c_slave_init i2c_master_init
    ESP_LOGI(TAG, "I2C initialized successfully");
    Button_init(100);

    OLED_Init();
    OLED_ShowString(40, 3, "power on", 16);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    OLED_Clear();
    // 显示汉字
    // OLED_ShowCHinese(0 * 18, 0, 0);
    // OLED_ShowCHinese(1 * 18, 0, 1);
    // OLED_ShowCHinese(2 * 18, 0, 2);

    // // 显示单个字符
    // OLED_ShowChar(0, 2, 'Q', 16);

    // // 显示字符串
    // OLED_ShowString(0, 4, "Fairy tale", 16);

    // 显示数字
   // OLED_ShowNum(0, 6, 8266, 6, 16);

   // uint8_t write_buf[19] = {0x11,0x02};
    while (1)
    {
        Button_process();
        if(Button_getItemData(BUTTON_USERBTN)->longPressed)
        {
            OLED_Clear();
            OLED_ShowString(40, 3, "power off", 16);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            gpio_set_level(14,0);
        }
        else
        {
            IIC_Unpack_Data(iic_read_buff,ADDR_SLAVE1,&tof_f1);//通过IIC读取所有寄存器信息并进行解码，将解码后变量存入结构体成员变量中
            mark = IIC_Unpack_Data(iic_read_buff,ADDR_SLAVE2,&tof_f2);//通过IIC读取所有寄存器信息并进行解码，将解码后变量存入结构体成员变量中
            printf("mark : %d  id: %d   dis1: %f    dis2: %f\n", mark, tof_f2.id, tof_f1.dis, tof_f2.dis);
          //  OLED_ShowNum(0, 2, Button_getItemData(BUTTON_USERBTN)->longPressed, 8, 16);
            OLED_ShowNum(30, 2, tof_f1.dis*100, 4, 16);
            OLED_ShowNum(30, 5, tof_f2.dis*100, 4, 16);
            OLED_ShowString(64, 2, "cm", 16);
            OLED_ShowString(64, 5, "cm", 16);

        }

        
        vTaskDelay(10 / portTICK_PERIOD_MS);
        /* code */
    }
    

    /* Read the MPU9250 WHO_AM_I register, on power up the register should have the value 0x71 */
    // ESP_ERROR_CHECK(mpu9250_register_read(MPU9250_WHO_AM_I_REG_ADDR, data, 1));
    // ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);

    // /* Demonstrate writing by reseting the MPU9250 */
    // ESP_ERROR_CHECK(mpu9250_register_write_byte(MPU9250_PWR_MGMT_1_REG_ADDR, 1 << MPU9250_RESET_BIT));

    // ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    // ESP_LOGI(TAG, "I2C unitialized successfully");
}
