	/***************************************************************************
	*	@file  	mbi5153.c
	*	@version V1.0.0
	*	@brief   MBI5153驱动相关函数
   ***************************************************************************
   *  @description
	*
	*  驱动时序
	* 
	***************************************************************************
***/

#include "stm32f4xx.h"
#include "led.h"
#include "delay.h"
#include "key.h"
#include "usart.h"
#include "gpio.h"
#include "mbi5153.h"

//extern uint8_t circuit;
extern volatile uint16_t SDI_PIN;

extern volatile uint8_t GData[CHIP_SIZE], RData[CHIP_SIZE], BData[CHIP_SIZE];
extern uint8_t Gmatrix[CUBE_SIZE][CUBE_SIZE][CUBE_SIZE];
extern uint8_t Rmatrix[CUBE_SIZE][CUBE_SIZE][CUBE_SIZE];
extern uint8_t Bmatrix[CUBE_SIZE][CUBE_SIZE][CUBE_SIZE];


extern void Send_2811_totalBuffer();
extern void Send_2811_totalPixels(uint8_t GData,uint8_t RData,uint8_t BData);

CUBE_Direct_E   Direct_x,Direct_y,Direct_z;
CUBE_Axis_E     Axis;
uint8_t         Recent_coord_x,Recent_coord_y,Recent_coord_z;
CUBE_Crawler_Status_E Crawler_Status;


void Driver_Val_Init(void)
{

    Axis = matrix_X;
    Recent_coord_x = 0;
    Recent_coord_y = 0;
    Recent_coord_z = 0;

    Direct_x=Direct_y=Direct_z=forward;

}

void Compose_RGB(RGB_Type_E RGB, uint8_t index, uint8_t gray)
{
    memset(GData, 0, sizeof(GData));
    memset(RData, 0, sizeof(RData));
    memset(BData, 0, sizeof(BData));

    if(RGB&G)
        GData[index] = gray;
    if(RGB&R)
        RData[index] = gray;
    if(RGB&B)
        BData[index] = gray;
}

void STRIP_RunningByPort(RGB_Type_E rgb, uint16_t port)
{
    uint8_t i,j,color;

    SDI_PIN = port;
    for(i = 0; i < CHIP_SIZE; i++)
    {
#if 1
        Compose_RGB(rgb, i, GRAY);
        Send_2811_totalBuffer();
        Delay_ms(2);
#else
        for(j = 0; j < 25; j++)
        {
            Compose_RGB(rgb, i, 4*j);
            RGB_SDI_Display();
            Delay_us(250);
        }
        for(j = 25; j > 0; j--)
        {
            Compose_RGB(rgb, i, 4*j);
            RGB_SDI_Display();
            Delay_us(250);
        }
#endif
    }

    for(i = CHIP_SIZE; i > 0; i--)
    {
#if 1
        Compose_RGB(rgb, i-1, GRAY);
        Send_2811_totalBuffer();
        Delay_ms(2);
#else
        for(j = 0; j < 25; j++)
        {
            Compose_RGB(rgb, i, 4*j);
            RGB_SDI_Display();
            Delay_us(250);
        }
        for(j = 25; j > 0; j--)
        {
            Compose_RGB(rgb, i, 4*j);
            RGB_SDI_Display();
            Delay_us(250);
        }
#endif
    }
}


void STRIP_SwitchByPort(uint16_t port)
{
    SDI_PIN = CUBE_X0_Y0_PIN;
    //rst();
    Send_2811_totalPixels(0xf,0,0);
    Delay_ms(SW_period);
    //rst();
    Send_2811_totalPixels(0,0xf,0);
    Delay_ms(SW_period);
    //rst();
    Send_2811_totalPixels(0,0,0xf);
    Delay_ms(SW_period);
    Send_2811_totalPixels(0xf,0xf,0xf);
    Delay_ms(SW_period);
    Send_2811_totalPixels(0xf,0xf,0);
    Delay_ms(SW_period);
    Send_2811_totalPixels(0,0xf,0xf);
    Delay_ms(SW_period);
    Send_2811_totalPixels(0xf,0,0xf);
    Delay_ms(SW_period);
}


void CUBE_Compose_RGB(RGB_Type_E RGB, uint8_t gray)
{

    memset(Gmatrix, 0, sizeof(Gmatrix));
    memset(Rmatrix, 0, sizeof(Rmatrix));
    memset(Bmatrix, 0, sizeof(Bmatrix));
    
    if(RGB&G)
    {
        Gmatrix[Recent_coord_x][Recent_coord_y][Recent_coord_z] = gray;
    }
    if(RGB&R)
    {
        Rmatrix[Recent_coord_x][Recent_coord_y][Recent_coord_z] = gray;
    }
    if(RGB&B)
    {
        Bmatrix[Recent_coord_x][Recent_coord_y][Recent_coord_z] = gray;
    }
}


void CUBE_Crawler_RGB(RGB_Type_E RGB, uint8_t gray)
{

    memset(Gmatrix, 0, sizeof(Gmatrix));
    memset(Rmatrix, 0, sizeof(Rmatrix));
    memset(Bmatrix, 0, sizeof(Bmatrix));

    if(matrix_X==Axis)
    {
        printf("xxx[%d %d %d]\r\n",Recent_coord_x,Recent_coord_y,Recent_coord_z);

        CUBE_Compose_RGB(RGB, gray);
        
        if(Direct_x==forward)//should to back
        {
            Recent_coord_x++;
            if(CUBE_SIZE-1==Recent_coord_x)
            {
                Axis = matrix_Y;//turn to Y
                Direct_x = backward;
                return;
            }
        }
        else
        {
            Recent_coord_x--;
            if(0==Recent_coord_x)
            {
                Axis = matrix_Y;//turn to Y
                Direct_x = forward;
                return;
            }
        }
    } 
    else if(matrix_Y==Axis)
    {
        printf("yyy[%d %d %d]\r\n",Recent_coord_x,Recent_coord_y,Recent_coord_z);

        CUBE_Compose_RGB(RGB, gray);

        if(Direct_y==forward)//should to back
        {
            Recent_coord_y++;
            if(CUBE_SIZE-1==Recent_coord_y)
            {
                Axis = matrix_Z;//turn to Z
                Direct_y = backward;
                return;
            }
        }
        else
        {
            Recent_coord_y--;
            if(0==Recent_coord_y)
            {
                Axis = matrix_Z;//turn to Z
                Direct_y = forward;
                return;
            }
        }
        
    } 
    else
    {
        printf("zzz[%d %d %d]\r\n",Recent_coord_x,Recent_coord_y,Recent_coord_z);

        CUBE_Compose_RGB(RGB, gray);

        if(Direct_z==forward)//should to back
        {
            Recent_coord_z++;
            if(CUBE_SIZE-1==Recent_coord_z)
            {
                Axis = matrix_X;//turn to X
                Direct_z = backward;
                return;
            }
        }
        else
        {
            Recent_coord_z--;
            if(0==Recent_coord_z)
            {
                Axis = matrix_X;//turn to X
                Direct_z = forward;
                return;
            }
        }
    }
}


void CUBE_crawler(void)
{

    CUBE_Crawler_RGB(G, GRAY);
    Send_2811_cubeBuffer();
    Delay_ms(100);
    CUBE_Crawler_RGB(R, GRAY);
    Send_2811_cubeBuffer();
    Delay_ms(100);
    CUBE_Crawler_RGB(B, GRAY);
    Send_2811_cubeBuffer();
    Delay_ms(100);
    CUBE_Crawler_RGB(RGB, GRAY);
    Send_2811_cubeBuffer();
}


