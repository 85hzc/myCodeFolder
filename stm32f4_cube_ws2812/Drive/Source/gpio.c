/***
	***************************************************************************
	*	@file  	gpio.c
	*	@version V1.0.0
	*	@brief   LED接口相关函数
   ***************************************************************************
   *  @description
	*
	*  初始化GPIO口，GCLK、DCLK、SDI、LE
	* 	
	***************************************************************************
***/

#include "gpio.h"

// 函数：IO初始化
void WS2812_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; //定义结构体
	RCC_AHB1PeriphClockCmd (MBI_CLK, ENABLE); 	//初始化GPIOG时钟

	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;   	//输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  	//推挽输出
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;  	//
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	//上拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; 	//速度选择
	
	//初始化引脚
	GPIO_InitStructure.GPIO_Pin = SDI_PIN;
	GPIO_Init(MBI_PORT, &GPIO_InitStructure);
	
	GPIO_ResetBits(MBI_PORT,SDI_PIN);  //输出低电平
}

