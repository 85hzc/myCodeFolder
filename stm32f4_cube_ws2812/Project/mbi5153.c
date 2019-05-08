	/***************************************************************************
	*	@file  	mbi5153.c
	*	@version V1.0.0
	*	@brief   MBI5153������غ���
   ***************************************************************************
   *  @description
	*
	*  ����ʱ��
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

void RGB_SDI_Display();
void Send_2811_24bits(uint8_t g,uint8_t r,uint8_t b);


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
		else 	//0 ,for "0",H:0.4us,L:	0.85us
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


void Compose_RGB(RGB_Type RGB, uint8_t index, uint8_t gray)
{
    //uint8_t GData[64], RData[64], BData[64];
    uint8_t i,j;

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

void RGB_SDI_Running(RGB_Type rgb)
{
    uint8_t i,j,color;

    for(i = 0; i < CHIP_SIZE; i++)
    {
#if 1
        Compose_RGB(rgb, i, 100);
        RGB_SDI_Display();
        Delay_ms(1);
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
        //mico_thread_msleep(1);
    }

    for(i = CHIP_SIZE; i > 0; i--)
    {
#if 1
        Compose_RGB(rgb, i-1, 100);
        RGB_SDI_Display();
        Delay_ms(1);
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
            //mico_thread_msleep(1);
    }
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
void Send_2811_24bits(uint8_t g,uint8_t r,uint8_t b)        //传送16位灰度数据    ,三组相同
{
    uint8_t i=0,j=0;

    //__set_FAULTMASK(1); // 关闭调度器
    /****************前置时间*********************/
    SDI_PIN_L;
    Delay_us(1);
    /*****************4*3组192位灰度数据********************************/
    for(i=0;i<CHIP_SIZE;i++)
    {
        Send_8bits(GData[i]);
        Send_8bits(RData[i]);
        Send_8bits(BData[i]);
    }
    SDI_PIN_L;
    //__set_FAULTMASK(0); // 打开调度器
}
#endif

void Send_2811_totalbits(uint8_t GData,uint8_t RData,uint8_t BData)        //传送16位灰度数据    ,三组相同
{
    uint8_t i=0,j=0;

    //__set_FAULTMASK(1); // 关闭调度器

    /****************前置时间*********************/
    SDI_PIN_L;
    Delay_us(1);
    /*****************4*3组192位灰度数据********************************/
    for(i=0;i<CHIP_SIZE;i++)
    {
        Send_8bits(GData);
        Send_8bits(RData);
        Send_8bits(BData);
    }
    SDI_PIN_L;

    //__set_FAULTMASK(0); // 打开调度器
}


void RGB_SDI_Switch(void)
{
    //rst();
    Send_2811_totalbits(0xf,0,0);
    Delay_ms(SW_period);
    //rst();
    Send_2811_totalbits(0,0xf,0);
    Delay_ms(SW_period);
    //rst();
    Send_2811_totalbits(0,0,0xf);
    Delay_ms(SW_period);
    Send_2811_totalbits(0xf,0xf,0xf);
    Delay_ms(SW_period);
    Send_2811_totalbits(0xf,0xf,0);
    Delay_ms(SW_period);
    Send_2811_totalbits(0,0xf,0xf);
    Delay_ms(SW_period);
    Send_2811_totalbits(0xf,0,0xf);
    Delay_ms(SW_period);
}

void RGB_SDI_Display()
{
    uint8_t i,j,color;

    //for(i=0;i<2;i++)
    {
        //printf("Display[%d]:%d %d %d\r\n",i,GData[i],RData[i],BData[i]);
        Send_2811_24bits(GData[i],RData[i],BData[i]);
    }
}


