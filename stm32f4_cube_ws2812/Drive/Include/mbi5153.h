
#ifndef __MBI5153_H
#define __MBI5153_H

#define TWO_TIMER_PULSE								0
#define GCLKNUM										5

#define CUBE_SIZE                                   4
#define CHIP_SIZE                                   47
#define SW_period                                   200

#define GRAY                                        10

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
    RGB = G|R|B,
}RGB_Type_E;

typedef enum
{
    matrix_X = 0,
    matrix_Y,
    matrix_Z,
}CUBE_Axis_E;

typedef enum
{
    forward = 0,
    backward,
}CUBE_Direct_E;


typedef enum
{
    crawler_ready = 0,
    crawler_running,
    crawler_turn,
    crawler_stop,
    crawler_back
}CUBE_Crawler_Status_E;


#endif

