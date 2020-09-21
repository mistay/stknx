


#ifndef STKNX_TP1FRAME_H
#define STKNX_TP1FRAME_H

#ifdef __cplusplus
extern "C"
{
#endif


#define KNX_TELEGRAM_MIN_SIZE           9
#define KNX_TELEGRAM_MAX_SIZE          23
#define KNX_TELEGRAM_PAYLOAD_MAX_SIZE  16

typedef struct KNX_TP1_Frame {
    unsigned char control_r;
    unsigned char priority;
    unsigned char srcZ;
    unsigned char srcL;
    unsigned char srcI;
    unsigned char dstZ;
    unsigned char dstL;
    unsigned char dstI;
    unsigned char destination_address_flag;
    unsigned char routing_counter;
    unsigned char length;
    unsigned char first_databyte;
    //unsigned char checksum;
    unsigned char payloadChecksum[KNX_TELEGRAM_PAYLOAD_MAX_SIZE-1]; // byte 8 to 22
} KNX_TP1_Frame;

void debug_knx_tp1_frame(KNX_TP1_Frame frame);


#ifdef __cplusplus
}
#endif

#endif //STKNX_TP1FRAME_H