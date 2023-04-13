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
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"
#include "lv_port_disp.h"
#include "lv_demos.h"
#include "driver/gpio.h"
#include "lv_port_indev.h"
#include "encoder.h"

 #include "DendoStepper.h"
 DendoStepper step1;
//static const char *TAG = "lvgltest";


static void lv_tick_task(void *arg)
{
    (void)arg;
    lv_tick_inc(10);
}

static void event_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_PRESSED) {
        // uint32_t id = lv_btnmatrix_get_selected_btn(obj);
        // const char * txt = lv_btnmatrix_get_btn_text(obj, id);
      //  lv_led_on(led);
        printf(" was PRESSED\n");
      
    }
    else if(code == LV_EVENT_RELEASED) {
        // uint32_t id = lv_btnmatrix_get_selected_btn(obj);
        // const char * txt = lv_btnmatrix_get_btn_text(obj, id);
       // lv_led_off(led);
        printf(" was RELEASED\n");
       // lv_obj_add_state(but1, LV_STATE_DEFAULT);
    }
    
}
 
 
 
static const char * btnm_map[] = {"1", "2", "3", "4", "5", "\n",
                                  "6", "7", "8", "9", "0", "\n",
                                  "Action1", "Action2", ""
                                 };

extern "C" void app_main(void)
{
    lv_init();
    lv_port_disp_init();
    lv_port_indev_init();



     /*  esp_register_freertos_tick_hook(lv_tick_task);
    Create and start a periodic timer interrupt to call lv_tick_inc
    */
    // const esp_timer_create_args_t periodic_timer_args = {
    //         .callback = &lv_tick_task,
    //         .name = "periodic_gui"};
    // esp_timer_handle_t periodic_timer;
    // ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    // ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 10 * 1000));
    // // lv_demo_music();
    // // lv_demo_stress();
    // // lv_demo_benchmark();


    // lv_group_t *group = lv_group_create();



    // lv_obj_t * btnm1 = lv_btnmatrix_create(lv_scr_act());
    // lv_btnmatrix_set_map(btnm1, btnm_map);
    // lv_btnmatrix_set_btn_width(btnm1, 10, 2);        /*Make "Action1" twice as wide as "Action2"*/
    // lv_btnmatrix_set_btn_ctrl(btnm1, 10, LV_BTNMATRIX_CTRL_CHECKABLE);
    // lv_btnmatrix_set_btn_ctrl(btnm1, 11, LV_BTNMATRIX_CTRL_CHECKED);
    // lv_obj_align(btnm1, LV_ALIGN_CENTER, 0, 0);
   

    // lv_obj_add_event_cb(btnm1, event_handler, LV_EVENT_ALL, NULL);
    // lv_indev_set_group(indev_keypad, group);
    //  lv_group_add_obj(get_group(),btnm1);
 
 
    //  lv_group_add_obj(group ,btnm1);
   // input_task_create();

    DendoStepper_config_t step1_cfg = {
        .stepPin = 18,
        .dirPin = 19,
        .enPin = 20,
        .timer_group = TIMER_GROUP_0,
        .timer_idx = TIMER_1,
        .miStep = MICROSTEP_32,
        .stepAngle = 1.8};


    step1.config(&step1_cfg);


    step1.init();

    step1.setSpeed(5000, 1000, 1000);

    step1.runPos(400000);
    while (1)
    {

       
         vTaskDelay(1000 / portTICK_PERIOD_MS);
        // step.runAbs(5000);
    }

    
}

