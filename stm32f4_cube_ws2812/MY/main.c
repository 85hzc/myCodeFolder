/***
    ***************************************************************************
    *   @file   main.c
    *   @version V1.0   
    *  @date    2019.1.30
    *   @author  ���ͿƼ�   
    *   @brief   ��������LED������
   ***************************************************************************
   *  @description
    *
    *   ʵ��ƽ̨������STM32F407ZGT6���İ�(�ͺţ�FK407M2)
    *   �Ա���ַ��https://shop212360197.taobao.com
    *   QQ����Ⱥ��536665479
    *
    *   ����˵����
    *
    *  1.����ÿ����һ�ξ͸ı�LED������״̬
    *   2.���ڳ�ʼ��ʱ��ӡ��Ϣ����������
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

//uint8_t circuit=0;

int main(void)
{
    uint8_t key_flag = 0;   //������־
    
    SystemInit();
    
    Delay_Init();       //��ʱ������ʼ��
    //LED_Init();         //LED��ʼ��
    //KEY_Init();       //����IO�ڳ�ʼ��
    Usart_Config();     //USART��ʼ������
    WS2812_GPIO_Init(); //��ʼ������pin
    //MX_TIM2_Init();

    printf("system start.\r\n");

    SystemCoreClockUpdate();

    //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//ϵͳ�ж����ȼ�����2
    //TIM14_PWM_Init(100-1,21-1);//168M/42=4Mhz�ļ���Ƶ��,��װ��ֵ100������PWMƵ��Ϊ 4M/100=40Khz.
    //TIM_SetCompare1(TIM14,50);
    Driver_Val_Init();

    while (1)
    {
        //circuit = (circuit != 0)?0:15;
        //printf("circuit:%d\r\n",circuit)
#if 1
        STRIP_SwitchByPort(CUBE_X0_Y0_PIN);
        STRIP_SwitchByPort(CUBE_X0_Y0_PIN);
        STRIP_SwitchByPort(CUBE_X0_Y0_PIN);
        STRIP_SwitchByPort(CUBE_X0_Y0_PIN);

        //printf("circuit:%d\r\n",circuit%16);
        //circuit++;
        STRIP_RunningByPort(G, CUBE_X0_Y0_PIN);
        STRIP_RunningByPort(R, CUBE_X0_Y0_PIN);
        STRIP_RunningByPort(B, CUBE_X0_Y0_PIN);
        STRIP_RunningByPort(RGB, CUBE_X0_Y0_PIN);
        STRIP_RunningByPort(R|B, CUBE_X0_Y0_PIN);
        STRIP_RunningByPort(R|G, CUBE_X0_Y0_PIN);
        STRIP_RunningByPort(G|B, CUBE_X0_Y0_PIN);

#else
        CUBE_crawler();
#endif
        Delay_ms(200);
#if 0
        //ÿ�ΰ������¶Ա�־����ȡ��
        if( KEY_Scan() == KEY_ON )
        {
            key_flag = ~key_flag;
        }
        
        //���ݰ�����־����LED���������
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

