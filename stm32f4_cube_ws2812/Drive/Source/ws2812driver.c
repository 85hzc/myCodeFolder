    /***************************************************************************
    *   @file   mbi5153.c
    *   @version V1.0.0
    *   @brief   MBI5153驱动相关函数
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

extern uint8_t circuit;
uint8_t GData[CHIP_SIZE], RData[CHIP_SIZE], BData[CHIP_SIZE];
uint8_t Gmatrix[CUBE_SIZE][CUBE_SIZE][CUBE_SIZE];
uint8_t Rmatrix[CUBE_SIZE][CUBE_SIZE][CUBE_SIZE];
uint8_t Bmatrix[CUBE_SIZE][CUBE_SIZE][CUBE_SIZE];


/* USER CODE BEGIN 0 */

void Din_1(void)
{
    //TIM_SetCompare1(TIM14, 26);
    SDI_PIN_H
}
void Din_0(void)
{
    //TIM_SetCompare1(TIM14, 13);
    SDI_PIN_L
}

void Send_8bits(uint8_t dat) 
{   
    uint8_t i; 

    Delay_us(1);
    for(i=0;i<8;i++)
    {
        if(dat & 0x80)//1,for "1",H:0.8us,L:0.45us;
        {
            SDI_PIN_H;
            Delay_us(1);
            SDI_PIN_L;
            delay_ns(1);
        }
        else    //0 ,for "0",H:0.4us,L: 0.85us
        {
            SDI_PIN_H;
            delay_ns(1);
            SDI_PIN_L;
            Delay_us(1);
        }
        dat=dat<<1;
    }
    SDI_PIN_L;
}


 void rst() 
{ 
     SDI_PIN_L;
     Delay_ms (1);
}


#if 0
//G--R--B
//MSB first 
void Send_2811_24bits(uint8_t g,uint8_t r,uint8_t b)
{
    Send_8bits(g);  
    Send_8bits(r);  
    Send_8bits(b);
}
#else
void Send_2811_totalBuffer()
{
    uint8_t i=0,j=0;

    SDI_PIN_L;
    Delay_us(1);
    for(i=0;i<CHIP_SIZE;i++)
    {
        Send_8bits(GData[i]);
        Send_8bits(RData[i]);
        Send_8bits(BData[i]);
    }
    SDI_PIN_L;
}
#endif

void Send_2811_totalPixels(uint8_t GData,uint8_t RData,uint8_t BData)
{
    uint8_t i=0,j=0;

    SDI_PIN_L;
    Delay_us(1);

    for(i=0;i<CHIP_SIZE;i++)
    {
        Send_8bits(GData);
        Send_8bits(RData);
        Send_8bits(BData);
    }
    SDI_PIN_L;
}

void Send_2811_cubeBuffer()
{
    uint8_t x=0,y=0,z=0;

    SDI_PIN_L;
    Delay_us(1);

    //SDI:x=0 y=0
    x=y=0;
    for(z=0;z<CUBE_SIZE;z++)
    {
        Send_8bits(Gmatrix[x][y][z]);
        Send_8bits(Rmatrix[x][y][z]);
        Send_8bits(Bmatrix[x][y][z]);
    }

    SDI_PIN_L;
}


