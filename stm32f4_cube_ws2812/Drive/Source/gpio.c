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
	RCC_AHB1PeriphClockCmd (CUBE_CLK, ENABLE); 	//初始化GPIOG时钟

	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;   	//输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  	//推挽输出
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;  	//
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	//上拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; 	//速度选择
	
	//初始化引脚
	GPIO_InitStructure.GPIO_Pin = CUBE_X0_Y0_PIN|CUBE_X0_Y1_PIN|CUBE_X0_Y2_PIN|CUBE_X0_Y3_PIN|
																CUBE_X1_Y0_PIN|CUBE_X1_Y1_PIN|CUBE_X1_Y2_PIN|CUBE_X1_Y3_PIN|
																CUBE_X2_Y0_PIN|CUBE_X2_Y1_PIN|CUBE_X2_Y2_PIN|CUBE_X2_Y3_PIN|
																CUBE_X3_Y0_PIN|CUBE_X3_Y1_PIN|CUBE_X3_Y2_PIN|CUBE_X3_Y3_PIN;
	GPIO_Init(CUBE_PORT, &GPIO_InitStructure);
	
	GPIO_ResetBits(CUBE_PORT,CUBE_X0_Y0_PIN|CUBE_X0_Y1_PIN|CUBE_X0_Y2_PIN|CUBE_X0_Y3_PIN|
														CUBE_X1_Y0_PIN|CUBE_X1_Y1_PIN|CUBE_X1_Y2_PIN|CUBE_X1_Y3_PIN|
														CUBE_X2_Y0_PIN|CUBE_X2_Y1_PIN|CUBE_X2_Y2_PIN|CUBE_X2_Y3_PIN|
														CUBE_X3_Y0_PIN|CUBE_X3_Y1_PIN|CUBE_X3_Y2_PIN|CUBE_X3_Y3_PIN);  //输出低电平
}

