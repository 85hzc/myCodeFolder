/***
    ***************************************************************************
    *   @file   main.c
    *   @version V1.0   
    *  @date    2019.1.30
    *   @author  反客科技   
    *   @brief   按键控制LED的亮灭
   ***************************************************************************
   *  @description
    *
    *   实验平台：反客STM32F407ZGT6核心板(型号：FK407M2)
    *   淘宝地址：https://shop212360197.taobao.com
    *   QQ交流群：536665479
    *
    *   功能说明：
    *
    *  1.按键每按下一次就改变LED的亮灭状态
    *   2.串口初始化时打印信息到串口助手
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

uint8_t circuit=0;

int main(void)
{
    uint8_t key_flag = 0;   //按键标志
    
    SystemInit();
    
    Delay_Init();       //延时函数初始化
    LED_Init();         //LED初始化
    //KEY_Init();           //按键IO口初始化
    Usart_Config(); // USART初始化函数
    WS2812_GPIO_Init();//初始化MBI驱动pin
    //MX_TIM2_Init();

    printf("system start.\r\n");
    
    SystemCoreClockUpdate();
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//系统中断优先级分组2

    //TIM14_PWM_Init(100-1,21-1);//168M/42=4Mhz的计数频率,重装载值100，所以PWM频率为 4M/100=40Khz.
    //TIM_SetCompare1(TIM14,50);

    Driver_Val_Init();
    
    while (1)
    {
        //circuit = (circuit != 0)?0:15;
        //printf("circuit:%d\r\n",circuit)
#if 0
        STRIP_Switch();
        STRIP_Switch();
        STRIP_Switch();
        STRIP_Switch();

        printf("circuit:%d\r\n",circuit%16);
        circuit++;
        STRIP_Running(G);
        STRIP_Running(R);
        STRIP_Running(B);
        STRIP_Running(R|B|B);
        STRIP_Running(R|B);
        STRIP_Running(R|G);
        STRIP_Running(G|B);
#else
        CUBE_crawler();
        Delay_ms(500);

#endif

#if 0
        //每次按键按下对标志进行取反
        if( KEY_Scan() == KEY_ON )
        {
            key_flag = ~key_flag;
        }
        
        //根据按键标志进行LED的亮灭操作
        if(key_flag == 0)
        {
            LED1_ON;
        }
        else
        {
            LED1_OFF;
        }
#endif
    }
}

