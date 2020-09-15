#include "freertos/FreeRTOS.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "driver/gpio.h"
#include "freertos/semphr.h"
#include "esp_types.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "esp_log.h"
#include "knx_tp1_frame.h"
#include "esp_heap_caps.h"
#include "knx.h"

#define PIN_LED_0                       5    
#define PIN_LED_1                       19
#define PIN_LED_2_DEBUG_OSCILLOSCOPE    4
#define PIN_LED_3                       21
#define PIN_LED_4_CLONE_LIGHT           18
#define PIN_LED_5                       22
#define PIN_LED_6_KNX_RXTX              23

void setup_leds() {
    gpio_config_t io_conf;

    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;

    io_conf.pin_bit_mask = 
        1ULL << PIN_LED_0 |
        1ULL << PIN_LED_1 |
        1ULL << PIN_LED_2_DEBUG_OSCILLOSCOPE |
        1ULL << PIN_LED_3 |
        1ULL << PIN_LED_4_CLONE_LIGHT |
        1ULL << PIN_LED_5 |
        1ULL << PIN_LED_6_KNX_RXTX;

    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
}

void blink_all_leds() {
    for (int i=1; i>=0; i--) {
        gpio_set_level(PIN_LED_0, i);
        vTaskDelay(75 / portTICK_RATE_MS);
        gpio_set_level(PIN_LED_1, i);
        vTaskDelay(75 / portTICK_RATE_MS);
        gpio_set_level(PIN_LED_2_DEBUG_OSCILLOSCOPE, i);
        vTaskDelay(75 / portTICK_RATE_MS);
        gpio_set_level(PIN_LED_3, i);
        vTaskDelay(75 / portTICK_RATE_MS);
        gpio_set_level(PIN_LED_4_CLONE_LIGHT, i);
        vTaskDelay(75 / portTICK_RATE_MS);
        gpio_set_level(PIN_LED_5, i);
        vTaskDelay(75 / portTICK_RATE_MS);
        gpio_set_level(PIN_LED_6_KNX_RXTX, i);
        vTaskDelay(75 / portTICK_RATE_MS);
    }
}

void knx_frame_received(KNX_TP1_Frame frame) {
    gpio_set_level(PIN_LED_6_KNX_RXTX, 1);
    vTaskDelay(20 / portTICK_RATE_MS);
    gpio_set_level(PIN_LED_6_KNX_RXTX, 0);

    if (frame.dstZ == 5 && frame.dstL == 9 && frame.dstI == 3) {
        gpio_set_level(PIN_LED_4_CLONE_LIGHT, frame.first_databyte & 0x1);
    }
    

    printf("->main->KNX TP1 FRAME: [");
    
    printf("R:%d Prio:%d ",
        frame.control_r,
        frame.priority);

    if (frame.destination_address_flag==0)
        printf("%d.%d.%d %d.%d.%d", 
            frame.srcZ, 
            frame.srcL, 
            frame.srcI,
            frame.dstZ, 
            frame.dstL, 
            frame.dstI);
    else
        printf("%d/%d/%d\t%d/%d/%d", 
            frame.srcZ, 
            frame.srcL, 
            frame.srcI,
            frame.dstZ, 
            frame.dstL, 
            frame.dstI);

    printf(" routing:%d len:%d chk:0x%02X", 
            frame.routing_counter, 
            frame.length, 
            frame.checksum
            );
    printf("]");

    printf("\n");
    fflush(stdout);
}

void app_main()
{
    setup_leds();
    blink_all_leds();

    setup_knx_reading(knx_frame_received);
    setup_knx_writing();

    printf("waiting for KNX frames ...\n");
    long uptime = 0;

    int delay = 5000;
    while(1) {
        vTaskDelay(delay / portTICK_RATE_MS);
        uptime++;

        int sec = ( uptime * delay / 1000 / 1 ) % 60;
        int min = ( uptime * delay / 1000 / 60 ) % 60;
        int hour = ( uptime * delay / 1000 / 60 / 60 ) % 60;
        int day = ( uptime * delay / 1000 / 60 / 60 / 24 ) % 24;

        //printf("heap_caps_get_free_size %dbytes read:%d write:%d uptime:%ddays %dh %d' %d\" ...\n", heap_caps_get_free_size(MALLOC_CAP_8BIT), octet_read_index, octet_write_index, day, hour, min, sec);
    }
}
