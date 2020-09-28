

#ifndef STKNXLIB_H
#define STKNXLIB_H


#include <KnxTelegram.h>
#include "knx_tp1_frame.h"

#ifdef __cplusplus
extern "C"
{
#endif


#define PIN_KNX_RX                      17
#define PIN_KNX_TX                      16
#define KNX_HALF_BIT_TIME_MS            15

#define ESP_INTR_FLAG_DEFAULT 0

#define MAX_OCTETS 1000

#define _IDF_VERSION "V.3.2"

//void setup_knx_reading(void (*knx_frame_received)(KNX_TP1_Frame *frame));
void setup_knx_reading(void (*knx_frame_received)(KnxTelegram &telegram));
void setup_knx_writing();

unsigned char stknx_send_telegram(unsigned char* rawtelegram, int len);

#ifdef __cplusplus
}
#endif

#endif //STKNXLIB_H