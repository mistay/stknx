#include <stdio.h>
#include "knx_tp1_frame.h"


void debug_knx_tp1_frame(KNX_TP1_Frame frame) {
    
    printf("KNX TP1 FRAME: [");
    
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