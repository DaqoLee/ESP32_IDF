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
#include "guipaint.h"
#include "OLED_0in96.h"
#include "u8g2.h"
#include "u8x8.h"

static const char *TAG = "tofsense";



void app_main(void)
{
    gpio_reset_pin(14);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(14, GPIO_MODE_OUTPUT);
    gpio_set_level(14,1);
   // uint8_t data[2];
    ESP_ERROR_CHECK(i2c_master_init());//i2c_slave_init i2c_master_init
    ESP_LOGI(TAG, "I2C initialized successfully");
 
    vTaskDelay(pdMS_TO_TICKS(200));
   
    u8g2_t u8g2;
    u8g2Init(&u8g2);

    u8g2_SetFont(&u8g2, u8g2_font_10x20_tf); // 设置英文字体
   // u8g2_DrawUTF8(&u8g2,20,40,"你好world");
    u8g2_SendBuffer(&u8g2);


    Button_init(10);


    char strx[20];

    while (1)
    {
        Button_process();

        for(int i=0; i<1000; i++)
        {
           
          sprintf(strx,"i = %d",i);
          u8g2_DrawStr(&u8g2, 0, 16, strx);
          u8g2_DrawStr(&u8g2, 0, 32, strx);
          u8g2_DrawStr(&u8g2, 0, 48, strx);
          u8g2_SendBuffer(&u8g2);
          vTaskDelay(5 / portTICK_PERIOD_MS);
        }


      
        vTaskDelay(100 / portTICK_PERIOD_MS);
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
