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

#define PIN_LED_0                       5    
#define PIN_LED_1                       19
#define PIN_LED_2_DEBUG_OSCILLOSCOPE    4
#define PIN_LED_3                       21
#define PIN_LED_4_CLONE_LIGHT           18
#define PIN_LED_5                       22
#define PIN_LED_6_KNX_RXTX              23

#define PIN_KNX_RX                      17
#define PIN_KNX_TX                      16
#define KNX_HALF_BIT_TIME_MS            15

#define ESP_INTR_FLAG_DEFAULT 0

static xQueueHandle queue_knxrx = NULL;
static xQueueHandle queue_knxtx = NULL;

static intr_handle_t s_timer_handle_knxrx;
static intr_handle_t s_timer_handle_knxtx;

#define MAX_OCTETS 1000
int octets[MAX_OCTETS];
int current_octet=0;

int bits_read=0;
int startbit_level=-1;
int stopbit_level=-1;
int paritybit=-1;
int bits_send=0;
int tmp_knx_tx=0;
int sendcount=0;
int tmp_sendbyte=0;
int waitcount=0;
int octets_printed = 0;
int index_octets_processed = 0;
int timercount=0;


unsigned char calc_even_parity_bit(unsigned char x){
    unsigned int count = 0, i, b = 1;
    for(i = 0; i < 8; i++)
        if( x & (b << i) ){count++;}
    return (count % 2);
}

unsigned char calc_odd_horizontal_parity_byte(int len, unsigned char* data) {
    int tmp=0;
    for ( int i=0; i<len; i++ )
        tmp ^= data[i];
    return (~tmp & 0xFF);
}

#define SENDBUFFER_LEN 8
unsigned char sendbuffer[100] = {
    0xBC, // knx control byte
    0x12, 0x03, 0x58, 0x03, 0xE1, 0x00, 0x81, 
    0x69 // knx checksum calc_odd_horizontal_parity_byte()
    };

static void isr_knx_tx_timer(void* arg)
{
    portBASE_TYPE xHigherPriorityTaskWoken;

    // re-enable timer
    TIMERG1.int_clr_timers.t1 = 1;
    TIMERG1.hw_timer[1].config.alarm_en = 1;

    waitcount++;
    if (waitcount == 2) {
        gpio_set_level(PIN_KNX_TX, 0);
        return;
    }
    if (waitcount == 3) {
        waitcount=0;
        return;
    }

    if (tmp_sendbyte > SENDBUFFER_LEN) {
        // frame done.

        // wait x2 bytes
        bits_send=-30;
        tmp_sendbyte = 0;
        sendcount++;
    }

    if (sendcount == 2) {
        return;
    }

    char octet = sendbuffer[tmp_sendbyte];

    if (bits_send == 0) {
        gpio_set_level(PIN_KNX_TX, 1);
    }

    if (bits_send > 0 && bits_send < 9)
        gpio_set_level(PIN_KNX_TX, (~octet >> (bits_send-1)) & 0x1);

    if (bits_send == 9)
        gpio_set_level(PIN_KNX_TX, calc_even_parity_bit(octet) == 0 ? 1 : 0);

    if (bits_send == 10)
        gpio_set_level(PIN_KNX_TX, 0);


    //dummy, killme.
    //int byKeyDown = 0;
    //xQueueSendToFrontFromISR( queue_knxtx, &byKeyDown, &xHigherPriorityTaskWoken );

    if (bits_send > 10)   {
        bits_send=-2;
        tmp_sendbyte++;
    } 

    if (false)
    {
        timer_config_t config;
        timer_get_config(TIMER_GROUP_1, TIMER_1, &config);
        if (config.alarm_en)
            timer_pause(TIMER_GROUP_1, TIMER_1);
    }

    bits_send++;

    return 0;
}


static void isr_knx_rx_timer(void* arg)
{
    portBASE_TYPE xHigherPriorityTaskWoken;

    // restart timer
    TIMERG0.int_clr_timers.t0 = 1;
    TIMERG0.hw_timer[0].config.alarm_en = 1;

    // for debugging w/ oscilloscope
    //gpio_set_level(PIN_LED_2_DEBUG_OSCILLOSCOPE, ++toggle % 2);

    // starbit
    if (bits_read==0) { 
        startbit_level = gpio_get_level(PIN_KNX_RX);

        // reading w/ 9600baud. 1/9600*1000*1000 is about 104
        timer_group_set_alarm_value_in_isr(TIMER_GROUP_0, TIMER_0, 104);
        octets[current_octet]=0;
    }

    // databits
    if (bits_read >= 1 && bits_read <= 8) {

        // read knx bit
        int bit = gpio_get_level(PIN_KNX_RX);
        bit = bit == 1 ? 0 : 1; // invert. knx sends 1 as logic 0 and vice-versa, see knx spec

        // first bit read is LSB. last bit is MSB. we read octets (8 bits)
        // knx databits: lsb first, msb last.
        octets[current_octet] = octets[current_octet] >> 1;
        octets[current_octet] |=  bit<<7;
    }

    if (bits_read==9) {
        paritybit = gpio_get_level(PIN_KNX_RX);
        paritybit = paritybit == 1 ? 0 : 1; // invert. knx sends 1 as logic 0 and vice-versa, see knx spec
    }


    if (bits_read==10) {
        // all bits read.

        stopbit_level = gpio_get_level(PIN_KNX_RX);
        
        // pause timer and wait for new startbit. this triggers a new interrupt that will reenable the timer

        timer_config_t config;
        timer_get_config(TIMER_GROUP_0, TIMER_0, &config);
        if (config.alarm_en)
            timer_pause(TIMER_GROUP_0, TIMER_0);

        // reenable interrupts for next startbit
        gpio_intr_enable(PIN_KNX_RX);

        // prepare next octet
        bits_read=-1;

        // stknx signals 30ms high-level. move timer half bit-time foreward and try to read in "the middle"
        timer_group_set_alarm_value_in_isr(TIMER_GROUP_0, TIMER_0, KNX_HALF_BIT_TIME_MS);

        //dummy, killme.
        int byKeyDown = 0;

        current_octet++;

        // debug read octet to serial console
        xQueueSendToFrontFromISR( queue_knxrx, &byKeyDown, &xHigherPriorityTaskWoken );
    }

    bits_read++;
}

void init_knx_tx_timer()
{
    timer_config_t config = {
            .alarm_en = TIMER_ALARM_EN,			//Alarm Enable
            .counter_en = TIMER_PAUSE,			//If the counter is enabled it will start incrementing / decrementing immediately after calling timer_init()
            .intr_type = TIMER_INTR_LEVEL,	    //Is interrupt is triggered on timer’s alarm (timer_intr_mode_t)
            .counter_dir = TIMER_COUNT_UP,	    //Does counter increment or decrement (timer_count_dir_t)
            .auto_reload = TIMER_AUTORELOAD_EN,	//If counter should auto_reload a specific initial value on the timer’s alarm, or continue incrementing or decrementing.
            .divider = 80   				    //Divisor of the incoming 80 MHz (12.5nS) APB_CLK clock. E.g. 80 = 1uS per timer tick
    };
    timer_init(TIMER_GROUP_1, TIMER_1, &config);
    timer_set_counter_value(TIMER_GROUP_1, TIMER_1, 0);

    timer_isr_register(TIMER_GROUP_1, TIMER_1, &isr_knx_tx_timer, NULL, ESP_INTR_FLAG_LEVEL3, &s_timer_handle_knxtx);
    timer_set_alarm_value(TIMER_GROUP_1, TIMER_1, 35); //104/3us

    timer_enable_intr(TIMER_GROUP_1, TIMER_1);
}

void init_knx_rx_timer()
{
    timer_config_t config = {
            .alarm_en = TIMER_ALARM_EN,			//Alarm Enable
            .counter_en = TIMER_PAUSE,			//If the counter is enabled it will start incrementing / decrementing immediately after calling timer_init()
            .intr_type = TIMER_INTR_LEVEL,	    //Is interrupt is triggered on timer’s alarm (timer_intr_mode_t)
            .counter_dir = TIMER_COUNT_UP,	    //Does counter increment or decrement (timer_count_dir_t)
            .auto_reload = TIMER_AUTORELOAD_EN,	//If counter should auto_reload a specific initial value on the timer’s alarm, or continue incrementing or decrementing.
            .divider = 80   				    //Divisor of the incoming 80 MHz (12.5nS) APB_CLK clock. E.g. 80 = 1uS per timer tick
    };
    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, &isr_knx_rx_timer, NULL, ESP_INTR_FLAG_LEVEL3, &s_timer_handle_knxrx);

    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
}

// interrupt that is triggered on each rising edge on knx read
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    // assuming starbit

    // process octet, this isr will be re-enabled within timer isr
    gpio_intr_disable(PIN_KNX_RX);

    // wait about 15us, will be reconfigured to 104us within timer isr
    timer_start(TIMER_GROUP_0, TIMER_0);
}

static void task_knxrx(void* arg)
{
    for(;;) {

        // not needed. killme.
        int byDummy=0;

        int index_start_frame=-1;
        int index_end_frame=-1;

        if( pdTRUE == xQueueReceive( queue_knxrx, &byDummy, portMAX_DELAY) ) 
        {
            // print received bytes.
            for (; octets_printed < current_octet; octets_printed++) {
                printf("%02x ", octets[octets_printed]);
            }
            
            uint8_t tmp=0;
            int horizontal_parity=-1;
            for ( int i=index_octets_processed; i<MAX_OCTETS; i++ ) {

                // start of KNX TP1 Frame
                if ((octets[i] & 0xD3) == 0x90) // e.g. 0xbc
                    index_start_frame = i;


                if (index_start_frame >= 0) {
                    tmp ^= octets[i];

                    if ( (~tmp & 0xFF) == octets[i] ) {
                        index_end_frame=i;
                        horizontal_parity = ~tmp;
                        break;
                    }
                }
            }

            if (index_start_frame>0 && index_end_frame>0) {
                index_octets_processed = index_end_frame;

                struct KNX_TP1_Frame *frame = malloc(sizeof *frame);
                frame -> control_r =        (octets[index_start_frame + 0] & 0x20) > 0;
                frame -> priority =         (octets[index_start_frame + 0] & 0xC) >> 2;
                frame -> srcZ =             (octets[index_start_frame + 1] & 0xF0) >> 4;
                frame -> srcL =             (octets[index_start_frame + 1] & 0x0F) >> 0;
                frame -> srcI =             octets[index_start_frame + 2];
                frame -> dstZ =             (octets[index_start_frame + 3] & 0xF0) >> 4;
                frame -> dstL =             (octets[index_start_frame + 3] & 0x0F) >> 0;
                frame -> dstI =             octets[index_start_frame + 4];
                frame -> destination_address_flag = (octets[index_start_frame + 5] & 0x80) > 0;
                frame -> routing_counter = (octets[index_start_frame + 5] & 0x70) >> 4;
                frame -> length =           (octets[index_start_frame + 5] & 0x0F);
                frame -> checksum =         octets[index_end_frame];

                gpio_set_level(PIN_LED_6_KNX_RXTX, 1);
                vTaskDelay(20 / portTICK_RATE_MS);
                gpio_set_level(PIN_LED_6_KNX_RXTX, 0);
                
                debug_knx_tp1_frame(*frame);

                if (frame -> dstZ == 5 && frame -> dstL == 9 && frame -> dstI == 1) {
                    gpio_set_level(PIN_LED_4_CLONE_LIGHT, octets[index_start_frame + 7] & 0x1);
                }

                free(frame);
            }
        }
    }
}

static void task_knxtx(void* arg)
{
    for(;;) {
        // not needed. killme.
        int byDummy=0;

        if( pdTRUE == xQueueReceive( queue_knxtx, &byDummy, portMAX_DELAY) ) 
        {
            //printf("task_knxtx...\n");
        }
    }
}

static void task_knx_send(void* arg)
{

    init_knx_tx_timer();
    timer_start(TIMER_GROUP_1, TIMER_1);

    for (;;) {

        vTaskDelay(10000 / portTICK_RATE_MS);
    }
}

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

void setup_knx_rx_pin() {
    gpio_config_t io_conf;

    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    io_conf.pin_bit_mask = 1ULL << PIN_KNX_RX;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;

    gpio_config(&io_conf);
}

void setup_knx_tx_pin() {
    gpio_config_t io_conf;

    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 1ULL << PIN_KNX_TX;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;

    gpio_config(&io_conf);
}

void setup_knx_reading() {
    setup_knx_rx_pin();

    queue_knxrx = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(task_knxrx, "task_knxrx", 2048, NULL, 10, NULL);

    init_knx_rx_timer();

    // rising edge indicates knx startbit
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(PIN_KNX_RX, gpio_isr_handler, (void*) PIN_KNX_RX);
}

void setup_knx_writing() {
    setup_knx_tx_pin();

    queue_knxtx = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(task_knxtx, "task_knxtx", 2048, NULL, 10, NULL);

    xTaskCreate(task_knx_send, "task_knx_send", 2048, NULL, 10, NULL);
}

void app_main()
{
    setup_leds();
    blink_all_leds();

    setup_knx_reading();
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

        printf("heap_caps_get_free_size %dbytes uptime:%ddays %dh %d' %d\" ...\n", heap_caps_get_free_size(MALLOC_CAP_8BIT), day, hour, min, sec);
    }
}
