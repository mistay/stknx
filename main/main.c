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

#define GPIO_OUTPUT_IO_1    4
#define GPIO_INPUT_IO_0     17

int toggle=0;

static xQueueHandle gpio_evt_queue = NULL;

static intr_handle_t s_timer_handle;

#define MAX_FRAMES 1000
int knxframes[MAX_FRAMES];
int framecount=0;

int bits_read=0;
static void timer_tg0_isr(void* arg)
{
    portBASE_TYPE xHigherPriorityTaskWoken;

    if (bits_read==0) {
        // starbit read. todo: check level

        // reading w/ 9600baud. 1/9600*1000*1000 is about 104
        timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 104);
        knxframes[framecount]=0;
    }

    if (bits_read >= 1) {
        // databits

        // for debugging w/ oscilloscope
        gpio_set_level(GPIO_OUTPUT_IO_1, ++toggle % 2);

        // sample knx bit
        int bit = gpio_get_level(GPIO_INPUT_IO_0);
        bit = bit == 1 ? 0 : 1; // invert. knx sends 1 as logic 0 and vice-versa, see knx spec

        // first bit read is LSB. last bit is MSB. we read octets (8 bits)
        knxframes[framecount] = knxframes[framecount] >> 1;
        knxframes[framecount] |=  bit<<7;
    }

    // restart timer
    TIMERG0.int_clr_timers.t0 = 1;
    TIMERG0.hw_timer[0].config.alarm_en = 1;

    if (++bits_read==9) {
        // all bits read.
        
        // pause timer and wait for new startbit. this triggers a new interrupt that will reenable the timer
        timer_pause(TIMER_GROUP_0, TIMER_0);

        // reenable interrupts for next startbit
        gpio_intr_enable(GPIO_INPUT_IO_0);

        // prepare next octet
        bits_read=0;

        // knx has a 30ms high-level. move timer half bit-time foreward and try to read in "the middle"
        timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 15);

        //dummy, killme.
        int byKeyDown = 0;

        // move counter to next octet and pause timer.
        framecount++;

        // debug read octet to serial console
        xQueueSendToFrontFromISR( gpio_evt_queue, &byKeyDown, &xHigherPriorityTaskWoken );
    }
}

void timer_tg0_init()
{
    timer_config_t config = {
            .alarm_en = true,				//Alarm Enable
            .counter_en = false,			//If the counter is enabled it will start incrementing / decrementing immediately after calling timer_init()
            .intr_type = TIMER_INTR_LEVEL,	//Is interrupt is triggered on timer’s alarm (timer_intr_mode_t)
            .counter_dir = TIMER_COUNT_UP,	//Does counter increment or decrement (timer_count_dir_t)
            .auto_reload = true,			//If counter should auto_reload a specific initial value on the timer’s alarm, or continue incrementing or decrementing.
            .divider = 80   				//Divisor of the incoming 80 MHz (12.5nS) APB_CLK clock. E.g. 80 = 1uS per timer tick
    };
    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, &timer_tg0_isr, NULL, 0, &s_timer_handle);

}


#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_1))
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) )
#define ESP_INTR_FLAG_DEFAULT 0

// interrupt that is triggered on each rising edge on knx read
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    // assuming starbit

    // weitere interpretationen eines startbits verhindern
    gpio_intr_disable(GPIO_INPUT_IO_0);

    // kurz warten (30us) um in die mitte der bits zu kommen
    timer_start(TIMER_GROUP_0, TIMER_0);
}

int knxframes_printed = 0;
static void task_debug_console(void* arg)
{
    for(;;) {

        // not needed. killme.
        int byDummy=0;

        if( pdTRUE == xQueueReceive( gpio_evt_queue, &byDummy, portMAX_DELAY) ) 
        {
            // debug received bytes.
            for (; knxframes_printed < framecount; knxframes_printed++) {
                printf("%#2X ", knxframes[knxframes_printed]);
            }
        }
    }
}

int timercount=0;

void app_main()
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    // rising edge indicates knx startbit
    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;

    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(task_debug_console, "task_debug_console", 2048, NULL, 10, NULL);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);

    // knx rx timer
    timer_tg0_init();

    while(1) {
        printf("waiting...\n");
        vTaskDelay(2000 / portTICK_RATE_MS);
    }
}
