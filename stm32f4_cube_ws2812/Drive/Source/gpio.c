/***
	***************************************************************************
	*	@file  	gpio.c
	*	@version V1.0.0
	*	@brief   LED�ӿ���غ���
   ***************************************************************************
   *  @description
	*
	*  ��ʼ��GPIO�ڣ�GCLK��DCLK��SDI��LE
	* 	
	***************************************************************************
***/

#include "gpio.h"

// ������IO��ʼ��
void WS2812_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; //����ṹ��
	RCC_AHB1PeriphClockCmd (CUBE_CLK, ENABLE); 	//��ʼ��GPIOGʱ��

	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;   	//���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  	//�������
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;  	//
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	//����
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; 	//�ٶ�ѡ��
	
	//��ʼ������
	GPIO_InitStructure.GPIO_Pin = CUBE_X0_Y0_PIN|CUBE_X0_Y1_PIN|CUBE_X0_Y2_PIN|CUBE_X0_Y3_PIN|
																CUBE_X1_Y0_PIN|CUBE_X1_Y1_PIN|CUBE_X1_Y2_PIN|CUBE_X1_Y3_PIN|
																CUBE_X2_Y0_PIN|CUBE_X2_Y1_PIN|CUBE_X2_Y2_PIN|CUBE_X2_Y3_PIN|
																CUBE_X3_Y0_PIN|CUBE_X3_Y1_PIN|CUBE_X3_Y2_PIN|CUBE_X3_Y3_PIN;
	GPIO_Init(CUBE_PORT, &GPIO_InitStructure);
	
	GPIO_ResetBits(CUBE_PORT,CUBE_X0_Y0_PIN|CUBE_X0_Y1_PIN|CUBE_X0_Y2_PIN|CUBE_X0_Y3_PIN|
														CUBE_X1_Y0_PIN|CUBE_X1_Y1_PIN|CUBE_X1_Y2_PIN|CUBE_X1_Y3_PIN|
														CUBE_X2_Y0_PIN|CUBE_X2_Y1_PIN|CUBE_X2_Y2_PIN|CUBE_X2_Y3_PIN|
														CUBE_X3_Y0_PIN|CUBE_X3_Y1_PIN|CUBE_X3_Y2_PIN|CUBE_X3_Y3_PIN);  //����͵�ƽ
}

