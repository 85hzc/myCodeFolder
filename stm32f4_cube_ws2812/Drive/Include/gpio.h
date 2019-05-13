#ifndef __GPIO_H
#define __GPIO_H

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"

/*---------------------- GPIO配置宏 ------------------------*/
#define CUBE_X0_Y0_PIN  	GPIO_Pin_0                // Din0 引脚
#define CUBE_X0_Y1_PIN  	GPIO_Pin_1                // Din0 引脚
#define CUBE_X0_Y2_PIN  	GPIO_Pin_2                // Din0 引脚
#define CUBE_X0_Y3_PIN  	GPIO_Pin_3                // Din0 引脚
#define CUBE_X1_Y0_PIN  	GPIO_Pin_4                // Din0 引脚
#define CUBE_X1_Y1_PIN  	GPIO_Pin_5                // Din0 引脚
#define CUBE_X1_Y2_PIN  	GPIO_Pin_6                // Din0 引脚
#define CUBE_X1_Y3_PIN  	GPIO_Pin_7                // Din0 引脚
#define CUBE_X2_Y0_PIN  	GPIO_Pin_8                // Din0 引脚
#define CUBE_X2_Y1_PIN  	GPIO_Pin_9                // Din0 引脚
#define CUBE_X2_Y2_PIN  	GPIO_Pin_10               // Din0 引脚
#define CUBE_X2_Y3_PIN  	GPIO_Pin_11               // Din0 引脚
#define CUBE_X3_Y0_PIN  	GPIO_Pin_12               // Din0 引脚
#define CUBE_X3_Y1_PIN  	GPIO_Pin_13               // Din0 引脚
#define CUBE_X3_Y2_PIN  	GPIO_Pin_14               // Din0 引脚
#define CUBE_X3_Y3_PIN  	GPIO_Pin_15               // Din0 引脚
#define CUBE_PORT         GPIOF                     // MBI驱动 GPIO端口
#define CUBE_CLK          RCC_AHB1Periph_GPIOF      // LED1 GPIO端口时钟

/*---------------------- GPIO控制宏 ------------------------*/
#define SDI_PIN_H(pin)         CUBE_PORT->BSRRL = pin;        // 输出高电平
#define SDI_PIN_L(pin)         CUBE_PORT->BSRRH = pin;        // 输出低电平


/*---------------------- 函数声明 ----------------------------*/

void MBI_GPIO_Init(void);   //LED初始化函数


#endif //__GPIO_H

