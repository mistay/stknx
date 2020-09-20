

#ifndef STKNX_H
#define STKNX_H

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

//#define _IDF_VERSION "V.3.2"

void setup_knx_reading(void (*knx_frame_received)(KNX_TP1_Frame frame));
void setup_knx_writing();

#ifdef __cplusplus
}
#endif

#endif //STKNX_H