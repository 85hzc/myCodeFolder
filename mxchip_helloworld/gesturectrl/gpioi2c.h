/*
 * simi2c.h
 *
 *  Created on: 2019年3月6日
 *      Author: OP056666
 */

#ifndef GESTURECTRL_GPIOI2C_H_
#define GESTURECTRL_GPIOI2C_H_

//#define GPIO_I2C   //gpio 模拟i2c
//#define CTI2C      //使用模拟i2c的不同方案，都差不多，delay时间不同  CTI2C百微妙级，另一个毫秒级

typedef enum {
    CT_COM_OK = 0,
    CT_ACK_FAIL,
}CT_RET_STATUS;


#endif /* GESTURECTRL_GPIOI2C_H_ */
