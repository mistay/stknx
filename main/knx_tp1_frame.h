
#define KNX_TELEGRAM_MIN_SIZE           9
#define KNX_TELEGRAM_MAX_SIZE          23

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
    unsigned char checksum;
} KNX_TP1_Frame;

void debug_knx_tp1_frame(KNX_TP1_Frame frame);