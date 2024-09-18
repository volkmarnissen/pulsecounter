/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* ULP Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <inttypes.h>
#include "esp_sleep.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc_periph.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "ulp.h"
#include "ulp_main.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

static void init_ulp_program(void);
static void update_pulse_count(void);
#define SENSOR_POWER_PIN GPIO_NUM_14
gpio_num_t gpio_nums_all[]={GPIO_NUM_36,GPIO_NUM_39,GPIO_NUM_34,GPIO_NUM_35,GPIO_NUM_32,GPIO_NUM_33,GPIO_NUM_25,GPIO_NUM_26,GPIO_NUM_27,
       GPIO_NUM_14,GPIO_NUM_12,GPIO_NUM_13,GPIO_NUM_2,GPIO_NUM_4,GPIO_NUM_15};

/* devkit */
gpio_num_t gpio_nums[]={
    GPIO_NUM_33,
    GPIO_NUM_32,
    GPIO_NUM_26,
    GPIO_NUM_13};
void app_main(void)
{
    /* If user is using USB-serial-jtag then idf monitor needs some time to
    *  re-connect to the USB port. We wait 1 sec here to allow for it to make the reconnection
    *  before we print anything. Otherwise the chip will go back to sleep again before the user
    *  has time to monitor any output.
    */
    vTaskDelay(pdMS_TO_TICKS(1000));

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause != ESP_SLEEP_WAKEUP_ULP) {
        init_ulp_program();
    } else {
        update_pulse_count();
    }

    printf("Entering deep sleep\n\n");
    ESP_ERROR_CHECK( esp_sleep_enable_ulp_wakeup() );
    esp_deep_sleep_start();
}
static uint16_t buildPinMask(){
    uint32_t mask = 0;
    for(int idx=0; idx < sizeof(gpio_nums)/sizeof(gpio_nums[0]); idx++){ 
        gpio_num_t gpio_num =  gpio_nums[idx]; 
        uint32_t rtcio_num = rtc_io_number_get(gpio_num);
        mask = mask | (1 << rtcio_num);
    }
    return mask;
}
static void init_ulp_program(void)
{
    esp_err_t err = ulp_load_binary(0, ulp_main_bin_start,
            (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);

    /* GPIO used for pulse counting. */

    /* Initialize selected GPIO as RTC IO, enable input, disable pullup and pulldown */
     for(int idx=0; idx < sizeof(gpio_nums)/sizeof(gpio_nums[0]); idx++){
        gpio_num_t gpio_num =  gpio_nums[idx];
        int rtcio_num = rtc_io_number_get(gpio_num);
        printf("%d %d %4x\n", gpio_num, rtcio_num, 1 << rtcio_num);
         rtc_gpio_init(gpio_num);
        rtc_gpio_set_direction(gpio_num, RTC_GPIO_MODE_INPUT_ONLY);
        rtc_gpio_pulldown_dis(gpio_num);
        rtc_gpio_pullup_en(gpio_num);
       // rtc_gpio_hold_en(gpio_num); 
    }
     ulp_current_low_registers = 0xefc0;
        
    ulp_pin_mask = buildPinMask();
     printf("pinMask: %4lx\n", ulp_pin_mask);
    
    rtc_gpio_init(SENSOR_POWER_PIN);
    rtc_gpio_set_direction(SENSOR_POWER_PIN, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_pulldown_dis(SENSOR_POWER_PIN);
    rtc_gpio_pullup_dis(SENSOR_POWER_PIN);
    

#if CONFIG_IDF_TARGET_ESP32
    /* Disconnect GPIO12 and GPIO15 to remove current drain through
     * pullup/pulldown resistors on modules which have these (e.g. ESP32-WROVER)
     * GPIO12 may be pulled high to select flash voltage.
     */
    rtc_gpio_isolate(GPIO_NUM_12);
    rtc_gpio_isolate(GPIO_NUM_15);
#endif // CONFIG_IDF_TARGET_ESP32

    esp_deep_sleep_disable_rom_logging(); // suppress boot messages
    /* Set ULP wake up period to T = 1s.
     * Minimum pulse width has to be T * (ulp_debounce_counter + 1) = 80ms.
     */
      REG_CLR_BIT(RTC_CNTL_INT_ENA_REG, RTC_CNTL_ULP_CP_INT_ENA_M);

  
    ulp_set_wakeup_period(0, 1000*1000);

    /* Start the program */
    err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
    ESP_ERROR_CHECK(err);
}


static void update_pulse_count(void)
{
     /* In case of an odd number of edges, keep one until next time */
    printf("This should be visible %04lx  %04lx  \n", ulp_previous_low_registers & ulp_pin_mask,ulp_current_low_registers & ulp_pin_mask);
    printf(" debug0 (curr): %04lx debug1: %04lx debug2: %04lx debug3: %04lx\n", ulp_debug0 & ulp_pin_mask, ulp_debug1 & ulp_pin_mask, ulp_debug2 & ulp_pin_mask, ulp_debug3& 0xFFFF);
   
    if((ulp_current_low_registers & ulp_pin_mask) !=(ulp_previous_low_registers & ulp_pin_mask) )
    {
       
  
        for(int idx=0; idx < sizeof(gpio_nums)/sizeof(gpio_nums[0]); idx++){ 
            gpio_num_t gpio_num =  gpio_nums[idx]; 
            uint32_t rtcio_num = rtc_io_number_get(gpio_num);
            uint32_t mask = 1 << rtcio_num;
            if( ulp_current_low_registers & mask )
                printf( "m: %4lx: %d " , ulp_current_low_registers & mask, gpio_num);
            if((ulp_current_low_registers & mask) !=(ulp_previous_low_registers & mask) ){
                printf("Changed RTC: %lu GPIO: %d %lx %lx\n", rtcio_num, gpio_num, mask , ulp_previous_low_registers & mask);
                if(ulp_current_low_registers & mask)
                    printf("Falling RTC: %lu GPIO: %d\n", rtcio_num, gpio_num);
                 if(ulp_previous_low_registers & mask)
                    printf("Rising RTC: %lu GPIO: %d\n", rtcio_num, gpio_num);
            }
            /* This is the new previous value */
        }
            printf("\nwrite prev %4lx\n", ulp_current_low_registers & ulp_pin_mask);
            ulp_previous_low_registers = ulp_current_low_registers & ulp_pin_mask;
         
    }

  
}
