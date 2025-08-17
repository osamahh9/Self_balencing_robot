#include <stdio.h>
#include "iot_servo.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"


#define SERVO_CH0_PIN  18


servo_config_t servo_cfg = {
    .max_angle = 180,
    .min_width_us = 500,
    .max_width_us = 2500,
    .freq = 50,
    .timer_number = LEDC_TIMER_0,
    .channels = {
        .servo_pin = {
            SERVO_CH0_PIN,
        },
        .ch = {
            LEDC_CHANNEL_0,
        },
    },
    .channel_number = 1,
    
} ;
void app_main(void)
{
    
    iot_servo_init(LEDC_LOW_SPEED_MODE, &servo_cfg);
    
    gpio_set_direction(32,GPIO_MODE_INPUT);
    gpio_set_pull_mode(32, GPIO_PULLUP_ONLY);
    
    float angle;
    int i=1;
    while(1){
        
        printf("Current duty: %d\n", i);
        // Set duty based on the current value of i
        // 1: 0%, 2: 50%, 3: 100%, 4: 150%
        
        switch (i)
        {
            case 1:
                angle = 0;       
            break;
            case 2:
                angle = 45; 
            break;
            case 3:
                angle = 90; 
            break;
            case 4:
                angle = 135;
            break;
            case 5:
                angle = 180;
            break;
        }

        // Set angle to 100 degree
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, angle);
        
        // Get current angle of servo
        iot_servo_read_angle(LEDC_LOW_SPEED_MODE, 0, &angle);

        while (gpio_get_level(32))
        {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        while (!gpio_get_level(32))
        {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }

        i++;
        if (i ==6) i= 1; // Reset i to 1 after reaching 4
    }

    //deinit servo
    iot_servo_deinit(LEDC_LOW_SPEED_MODE);
    printf("Servo deinitialized\n");
}