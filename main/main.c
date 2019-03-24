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


long micros() { return system_get_time();}
long millis() { return micros()/1000; }

// function and variables to manage push button signal debounce
static inline long TimeDeltaDword(long now, long before) { return (long)(now - before); }
long lastEvent=0;
           

#define ROUTING_FIELD_PAYLOAD_LENGTH_MASK    0xF// B00001111
#define KNX_TELEGRAM_LENGTH_OFFSET      8

#define KNX_TELEGRAM_HEADER_SIZE        6
#define KNX_TELEGRAM_PAYLOAD_MAX_SIZE  16
#define KNX_TELEGRAM_MIN_SIZE           9
#define KNX_TELEGRAM_MAX_SIZE          23      
      
struct SKnxTelegram {
    union {
        unsigned char _telegram[KNX_TELEGRAM_MAX_SIZE]; // byte 0 to 22
        struct {
        unsigned char _controlField; // byte 0
        unsigned char _sourceAddrH;  // byte 1
        unsigned char _sourceAddrL;  // byte 2
        unsigned char _targetAddrH;  // byte 3
        unsigned char _targetAddrL;  // byte 4
        unsigned char _routing;      // byte 5
        unsigned char _commandH;     // byte 6
        unsigned char _commandL;     // byte 7
        unsigned char _payloadChecksum[KNX_TELEGRAM_PAYLOAD_MAX_SIZE-1]; // byte 8 to 22
      };
    };
};



#define MAX_FRAMES 1000
int knxframes[MAX_FRAMES];
int framecount=0;
int knxTelegramSize=0;
int telegramBufferIndex=0;

int knxprinted=0;
int bits_read=0;
int parity_bit=0, stop_bit=0;
int data_byte =0;
int calc_parity = 0;

static void timer_tg0_isr(void* arg)
{
    int payloadSize;
    portBASE_TYPE xHigherPriorityTaskWoken;

    long nowTime = (long) micros();
    if(TimeDeltaDword(nowTime,lastEvent) >= 10000) {
        framecount=knxprinted=telegramBufferIndex=0;
    }

    lastEvent=nowTime;

    if (bits_read==0) {
        // starbit read. todo: check level

        // reading w/ 9600baud. 1/9600*1000*1000 is about 104
        timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 104);
        data_byte=0;
        calc_parity=1;
    } else if (bits_read >= 1 && bits_read <=8) {
        // databits

        // for debugging w/ oscilloscope
        gpio_set_level(GPIO_OUTPUT_IO_1, ++toggle % 2);

        // sample knx bit
        int bit = gpio_get_level(GPIO_INPUT_IO_0);
        bit = bit == 1 ? 0 : 1; // invert. knx sends 1 as logic 0 and vice-versa, see knx spec

        // first bit read is LSB. last bit is MSB. we read octets (8 bits)
        data_byte = data_byte >> 1;
        data_byte |=  bit<<7;
        
        if (bit)
            calc_parity = !calc_parity;

     // read for parity and stop bit
    } else if (bits_read==9) {
        parity_bit = gpio_get_level(GPIO_INPUT_IO_0);
    } else if (bits_read==10) {
        stop_bit = gpio_get_level(GPIO_INPUT_IO_0);
    } else {
        // all bits read.
        // move counter to next octet and pause timer.
        // check parity bit
        if (calc_parity==parity_bit)
            knxframes[framecount++] = data_byte;
        
        // pause timer and wait for new startbit. this triggers a new interrupt that will reenable the timer
        timer_pause(TIMER_GROUP_0, TIMER_0);

        // reenable interrupts for next startbit
        gpio_intr_enable(GPIO_INPUT_IO_0);
    }

    if (bits_read == 11) {
        // prepare next octet
        bits_read=0;
    } else {
        bits_read++;
    }
    // restart timer
    TIMERG0.int_clr_timers.t0 = 1;
    TIMERG0.hw_timer[0].config.alarm_en = 1;
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
    // assuming startbit

    // weitere interpretationen eines startbits verhindern
    gpio_intr_disable(GPIO_INPUT_IO_0);

    // kurz warten (35us/2) um in die mitte der bits zu kommen
    // start timer
    TIMERG0.int_clr_timers.t0 = 1;
    TIMERG0.hw_timer[0].config.alarm_en = 1;
     // read all data bits in the midddle of the zero : 35/2
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 17);
    timer_start(TIMER_GROUP_0, TIMER_0);
}



int getFrameSize(unsigned char b) {

    // byte 5 is routing byte containing payloadlength
    int payloadSize = b & ROUTING_FIELD_PAYLOAD_LENGTH_MASK;
    /*if (payloadSize > KNX_TELEGRAM_PAYLOAD_MAX_SIZE
        || payloadSize < 1)
        return 0;*/
    return KNX_TELEGRAM_LENGTH_OFFSET + payloadSize;  
}


#define EIB_CONTROL_FIELD_PATTERN_MASK  0xD3 // B11010011
#define EIB_CONTROL_FIELD_VALID_PATTERN  0x90 // B10010000

bool validControlField(unsigned char b) {
    return (b & EIB_CONTROL_FIELD_PATTERN_MASK) == EIB_CONTROL_FIELD_VALID_PATTERN;
}

unsigned short GetSourceAddress(struct SKnxTelegram s) {
  // WARNING : works with little endianness only
  // The adresses within KNX telegram are big endian
  unsigned short addr; addr = s._sourceAddrL + (s._sourceAddrH<<8); 
  return addr;
}

unsigned short GetTargetAddress(struct SKnxTelegram s)  {
 // WARNING : endianess sensitive!! Code below is for LITTLE ENDIAN chip
 // The KNX telegram uses BIG ENDIANNESS (Hight byte placed before Low Byte)
  unsigned short addr; 
  addr = s._targetAddrL + (s._targetAddrH<<8); 
  return addr;
}

int GetPayloadLength(struct SKnxTelegram s) {
    return s._routing & ROUTING_FIELD_PAYLOAD_LENGTH_MASK;
}

unsigned char CalculateChecksum(struct SKnxTelegram s) 
{
  unsigned char indexChecksum, xorSum=0;  
  indexChecksum = KNX_TELEGRAM_HEADER_SIZE + GetPayloadLength(s) + 1;
  for (unsigned char i = 0; i < indexChecksum ; i++)   xorSum ^= s._telegram[i]; // XOR Sum of all the databytes
  return (unsigned char)(~xorSum); // Checksum equals 1's complement of databytes XOR sum
}

unsigned char GetChecksum(struct SKnxTelegram s) 
{   
    return (s._payloadChecksum[GetPayloadLength(s) - 1]);
}

bool IsChecksumCorrect(struct SKnxTelegram s) 
{ 
    return (GetChecksum(s)==CalculateChecksum(s));
}



struct SKnxTelegram sTelegram;
//unsigned char telegramBuffer[KNX_TELEGRAM_MAX_SIZE];
unsigned char* telegramBuffer = &sTelegram._telegram;

static void task_debug_console(void* arg)
{
    int i;
    for(;;) {

        // not needed. killme.
        int byDummy=0;

        if( pdTRUE == xQueueReceive( gpio_evt_queue, &byDummy, portMAX_DELAY) ) 
        {
            if (knxprinted != framecount)
                printf("last byte: p: %i =?= %i, s: %i\n", parity_bit, calc_parity, stop_bit);
            for (; knxprinted < framecount; knxprinted++) {
                telegramBuffer[telegramBufferIndex]=knxframes[knxprinted];
                printf("%02X ", telegramBuffer[telegramBufferIndex]);

                if (telegramBufferIndex==5) {
                    knxTelegramSize = getFrameSize(sTelegram._routing);  
                }

                telegramBufferIndex++;
            }
            

            if (telegramBufferIndex>=knxTelegramSize && 
                telegramBufferIndex!=0 && 
                knxTelegramSize!=0) {
                if (validControlField(sTelegram._controlField)){
                    printf("\nValid control field!\n");
                } else {
                    printf("\nERR: invalid control field!\n");
                }

                printf("Checksum: %s\n", IsChecksumCorrect(sTelegram)?"OK":"wrong");
                printf("knxTelegram size: %d, (min 9 byte, max 23byte)\n", knxTelegramSize);

                unsigned short w = GetSourceAddress(sTelegram);
                printf("telegram SourceAddress: %d/%d/%d\n", w/256/8, (w/256)%8, w%256);    
                w = GetTargetAddress(sTelegram);
                printf("telegram TargetAddress: %d/%d/%d\n", w/256/8, (w/256)%8, w%256);

                printf("telegram payload: ");

                for (i=0; i < GetPayloadLength(sTelegram); i++) {
                    printf("%c ", sTelegram._payloadChecksum[i]);
                }
                printf("\n");

                framecount=knxprinted=telegramBufferIndex=0;

                /*framecount=knxprinted=0;
                knxTelegramSize=0;
                telegramBufferIndex = 0;*/
            }

            /*
            printf("knxTelegramSize: %d\n", knxTelegramSize);  

            // debug received bytes.
            for (i=0; i < knxTelegramSize; i++) {
                printf("%02X ", knxframes[i]);
            }
            printf("\n");*/
            
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
        //printf("waiting...\n");
        vTaskDelay(100 / portTICK_RATE_MS);


        long nowTime = (long) micros();
        if(TimeDeltaDword(nowTime,lastEvent) >= 10000) {
           // debug read octet to serial console
             //dummy, killme.
            int byKeyDown = 0;

            portBASE_TYPE xHigherPriorityTaskWoken;
            xQueueSendToFrontFromISR( gpio_evt_queue, &byKeyDown, &xHigherPriorityTaskWoken );
        }
    }
}
