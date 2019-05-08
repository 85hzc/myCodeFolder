
#ifndef __MBI5153_H
#define __MBI5153_H

#define TWO_TIMER_PULSE								0
#define GCLKNUM										5

#define CHIP_SIZE                                   47
#define SW_period                                   200

#define SCAN_LINE_1									0x0
#define SCAN_LINE_2									0x1
#define SCAN_LINE_4									0x3
#define SCAN_LINE_8									0x7
#define SCAN_LINE_16								0xf
#define SCAN_LINE_32								0x1f

typedef enum
{
    G = 1<<0,
    R = 1<<1,
    B = 1<<2,
}RGB_Type;


#endif

