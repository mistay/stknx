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

#define OSCI_OUTPUT_PIN    4
#define STKNX_RXPIN     17
#define STKNX_TXPIN     16

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
int bit_index=0;
int parity_bit=0, stop_bit=0;
int data_byte =0;
int calc_parity = 0;
bool write_mode = false; 
bool cleanup_time = false;
struct SKnxTelegram sendTelegram;
int sendByteIndex=0;



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


int GetTelegramLength(struct SKnxTelegram s)
{ return (KNX_TELEGRAM_LENGTH_OFFSET + GetPayloadLength(s));}


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




bool writeTelegram(struct SKnxTelegram data) {
    long startTime = (long) micros();
    while(1) {
        long nowTime = (long) micros();
        if(TimeDeltaDword(nowTime,lastEvent) >= 3000 
            && !write_mode) {
            break;
        }
        // send timeout, bus is busy
        if(TimeDeltaDword(nowTime,startTime) >= 100000) {
            return false;
        }
    }
    
    // disable read interrupts
    gpio_intr_disable(STKNX_RXPIN);
    memcpy(sendTelegram._telegram, data._telegram, GetTelegramLength(data));
    write_mode=true;
    sendByteIndex=0;
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 1);
    timer_start(TIMER_GROUP_0, TIMER_0);
    return true;
}

static void timer_tg0_isr(void* arg)
{
    int payloadSize;
    portBASE_TYPE xHigherPriorityTaskWoken;

    long nowTime = (long) micros();

    // timeout for sending the longest telegram with 23bytes
    if(TimeDeltaDword(nowTime,lastEvent) >= 100*1000) {
        framecount=knxprinted=telegramBufferIndex=0;
    }

    lastEvent=nowTime;

    if (write_mode) {
        if (cleanup_time) {
            gpio_set_level(STKNX_TXPIN, 0);
            if (bit_index==11) {
                timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 3*104-35);
            } else {
                timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 104-35);
            }
            cleanup_time=!cleanup_time;
            //cleanup gpio
            // restart timer
            TIMERG0.int_clr_timers.t0 = 1;
            TIMERG0.hw_timer[0].config.alarm_en = 1;
            return;
        } else {
            // bytes to send?
            if (sendByteIndex>=GetTelegramLength(sendTelegram)
                || sendByteIndex>=KNX_TELEGRAM_MAX_SIZE) {
                //sending all bytes done, reenable timers
                sendByteIndex=0;
                write_mode=false;
                                
                timer_pause(TIMER_GROUP_0, TIMER_0);
                // reenable interrupts to read data from bus
                gpio_intr_enable(STKNX_RXPIN);
                return;
            }
            cleanup_time=!cleanup_time;
            timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 35);
        }
    }


    if (bit_index==0) {
        // starbit read. todo: check level

        // reading w/ 9600baud. 1/9600*1000*1000 is about 104
        if  (write_mode) {
            gpio_set_level(STKNX_TXPIN, 1);
        } else {
            timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 104);
        }
        data_byte=0;
        calc_parity=1;
    } else if (bit_index >= 1 && bit_index <=8) {
        // databits

        // for debugging w/ oscilloscope
        gpio_set_level(OSCI_OUTPUT_PIN, ++toggle % 2);

        // read/write knx bit
        int bit;
        int databit = bit_index-1;
        if  (write_mode) {
            bit = ((sendTelegram._telegram[sendByteIndex]) >> (databit) ) & 1;
            if (bit)
                calc_parity = !calc_parity;
            bit = bit == 1 ? 0 : 1;
            gpio_set_level(STKNX_TXPIN, bit);
        } else {
            bit = gpio_get_level(STKNX_RXPIN);
            bit = bit == 1 ? 0 : 1; // invert. knx sends 1 as logic 0 and vice-versa, see knx spec
            // first bit read is LSB. last bit is MSB. we read octets (8 bits)
            //data_byte = data_byte >> 1;
            //data_byte |=  bit<<7;
            data_byte |= bit << databit;
            if (bit)
                calc_parity = !calc_parity;
        }
       
    
    // read for parity and stop bit
    } else if (bit_index==9) {
        if (write_mode)
            gpio_set_level(STKNX_TXPIN, calc_parity);
        else
            parity_bit = gpio_get_level(STKNX_RXPIN);
    } else if (bit_index==10) {
        if (write_mode)
            gpio_set_level(STKNX_TXPIN, 0);
        else
            stop_bit = gpio_get_level(STKNX_RXPIN);
    } else {
        if (!write_mode) {
            // all bits read.
            // move counter to next octet and pause timer.
            // check parity bit
            if (calc_parity==parity_bit)
                knxframes[framecount++] = data_byte;
            
            // pause timer and wait for new startbit. this triggers a new interrupt that will reenable the timer
            timer_pause(TIMER_GROUP_0, TIMER_0);

            // reenable interrupts for next startbit
            gpio_intr_enable(STKNX_RXPIN);
        }
    }

    if (bit_index == 11) {
        // prepare next octet
        bit_index=0;
        if (write_mode) {
            sendByteIndex++;
        }
    } else {
        bit_index++;
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


#define ESP_INTR_FLAG_DEFAULT 0

// interrupt that is triggered on each rising edge on knx read
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    // assuming startbit

    // weitere interpretationen eines startbits verhindern
    gpio_intr_disable(STKNX_RXPIN);

    // kurz warten (35us/2) um in die mitte der bits zu kommen
    // start timer
    TIMERG0.int_clr_timers.t0 = 1;
    TIMERG0.hw_timer[0].config.alarm_en = 1;
     // read all data bits in the midddle of the zero : 35/2
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 17);
    timer_start(TIMER_GROUP_0, TIMER_0);
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
    io_conf.pin_bit_mask = ((1ULL<<OSCI_OUTPUT_PIN));
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = ((1ULL<<STKNX_TXPIN));
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    // rising edge indicates knx startbit
    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    io_conf.pin_bit_mask = ((1ULL<<STKNX_RXPIN) );
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(task_debug_console, "task_debug_console", 2048, NULL, 10, NULL);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(STKNX_RXPIN, gpio_isr_handler, (void*) STKNX_RXPIN);
    

    // knx rx timer
    timer_tg0_init();

    while(1) {
        //printf("waiting...\n");
        vTaskDelay(100 / portTICK_RATE_MS);


        long nowTime = (long) micros();
        if(TimeDeltaDword(nowTime,lastEvent) >= 2000 
            && TimeDeltaDword(nowTime,lastEvent) < 100000) {
           // debug read octet to serial console
             //dummy, killme.
            int byKeyDown = 0;

            portBASE_TYPE xHigherPriorityTaskWoken;
            xQueueSendToFrontFromISR( gpio_evt_queue, &byKeyDown, &xHigherPriorityTaskWoken );
        }


        if (write_mode) {
            printf("bit_index: %d, sendByteIndex:%d\n", bit_index, sendByteIndex);
        } /*else {
            printf("last byte: p: %i =?= %i, stop: %i\n", parity_bit, calc_parity, stop_bit);
        }*/
        

        if(TimeDeltaDword(nowTime,lastEvent) >= 5000000) {
            struct SKnxTelegram t;
            const unsigned char offTelegram[] = {0xBC, 0x11, 0x02, 0x01, 0x01, 0xE1, 0x00, 0x81, 0x30};
            memcpy(t._telegram, offTelegram, 9);
            if (writeTelegram(t))
                ESP_LOGI("stknx", "writeTelegram started.");
            else
                ESP_LOGI("stknx", "writeTelegram failed. bus busy.");
        }
    }
}
