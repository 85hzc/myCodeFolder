/*
 * rgbdriver.c
 *
 *  Created on: 2019年4月1日
 *      Author: OP056666
 */
#include "mico.h"
#include "cubedisplay.h"

#define T1 1

#define GPIO_RGB_LE     MICO_GPIO_7  //scl
#define GPIO_RGB_GCLK   MICO_GPIO_14 //pwm5 d4
#define GPIO_RGB_DCLK   MICO_GPIO_13 //pwm4 d8

#define GPIO_RGB_SDI    MICO_GPIO_8 //sda
#define GPIO_RGB_SDI_G1  MICO_GPIO_9
#define GPIO_RGB_SDI_B1  MICO_GPIO_12

#define GPIO_RGB_SDI_R2  MICO_GPIO_21//_21 app can not work
#define GPIO_RGB_SDI_G2  MICO_GPIO_22
#define GPIO_RGB_SDI_B2  MICO_GPIO_10

extern uint8_t circle;
uint8_t GData[64], RData[64], BData[64];

void GPIO_init()
{
    //MicoGpioInitialize(GPIO_RGB_LE, OUTPUT_OPEN_DRAIN_NO_PULL);
    //MicoGpioInitialize(GPIO_RGB_DCLK, OUTPUT_OPEN_DRAIN_NO_PULL);
    MicoGpioInitialize(GPIO_RGB_SDI, OUTPUT_OPEN_DRAIN_NO_PULL); //app can not work
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
#if 0
        Compose_RGB(rgb, i, 50);
        RGB_SDI_Display();
        mico_thread_msleep(1);
#else
        for(j = 0; j < 50; j++)
        {
            Compose_RGB(rgb, i, 3*j);
            RGB_SDI_Display();
            mico_thread_msleep(1);
        }
        for(j = 50; j > 0; j--)
        {
            Compose_RGB(rgb, i, 3*j);
            RGB_SDI_Display();
            mico_thread_msleep(1);
        }
#endif
        //mico_thread_msleep(1);
    }
}

#ifdef __GNUC__
#pragma GCC optimize ("O0") // 关闭优化，否则直接反转 IO 会被优化
#endif /* ifdef __GNUC__ */
/* SDI output R fast */
__attribute__((section(".fast"))) // 代码放到RAM运行
void rst(void)
{
    gpio_output_low(GPIO_RGB_SDI);
    mico_thread_msleep(1);
}


#ifdef __GNUC__
#pragma GCC optimize ("O0") // 关闭优化，否则直接反转 IO 会被优化
#endif /* ifdef __GNUC__ */
/* SDI output R fast */
__attribute__((section(".fast"))) // 代码放到RAM运行
void Send_8bits(uint8_t dat)
{
    uint8_t i;

    myNsDelay(2);
    for(i=0;i<8;i++)
    {
        if(dat & 0x80)//1,for "1",H:0.8us,L:0.45us;
        {
            gpio_output_high(GPIO_RGB_SDI);
            myNsDelay(1);
            gpio_output_low(GPIO_RGB_SDI);
        }
        else    //0 ,for "0",H:0.4us,L: 0.85us
        {
            gpio_output_high(GPIO_RGB_SDI);
            gpio_output_low(GPIO_RGB_SDI);
            myNsDelay(1);
        }
        dat=dat<<1;
    }
    gpio_output_low(GPIO_RGB_SDI);
}
#ifdef __GNUC__
#pragma GCC optimize ("O0") // 关闭优化，否则直接反转 IO 会被优化
#endif /* ifdef __GNUC__ */
/* SDI output R fast */
__attribute__((section(".fast"))) // 代码放到RAM运行
void Send_2811_24bits()        //传送16位灰度数据    ,三组相同
{
    uint8_t i=0,j=0;

    __set_FAULTMASK(1); // 关闭调度器

    /****************前置时间*********************/
    gpio_output_low(GPIO_RGB_SDI);
    myNsDelay(1);
    /*****************4*3组192位灰度数据********************************/
    for(i=0;i<CHIP_SIZE;i++)
    {
        Send_8bits(GData[i]);
        Send_8bits(RData[i]);
        Send_8bits(BData[i]);
    }
    gpio_output_low(GPIO_RGB_SDI);

    __set_FAULTMASK(0); // 打开调度器
}

void Send_2811_totalbits(uint8_t GData,uint8_t RData,uint8_t BData)        //传送16位灰度数据    ,三组相同
{
    uint8_t i=0,j=0;

    __set_FAULTMASK(1); // 关闭调度器

    /****************前置时间*********************/
    gpio_output_low(GPIO_RGB_SDI);
    myNsDelay(1);
    /*****************4*3组192位灰度数据********************************/
    for(i=0;i<CHIP_SIZE;i++)
    {
        Send_8bits(GData);
        Send_8bits(RData);
        Send_8bits(BData);
    }
    gpio_output_low(GPIO_RGB_SDI);

    __set_FAULTMASK(0); // 打开调度器
}

#ifdef __GNUC__
#pragma GCC optimize ("O0") // 关闭优化，否则直接反转 IO 会被优化
#endif /* ifdef __GNUC__ */
/* SDI output R fast */
__attribute__((section(".fast"))) // 代码放到RAM运行
void RGB_SDI_Switch(void)
{
    //rst();
    Send_2811_totalbits(0xf,0,0);
    mico_thread_msleep(500);

    //rst();
    Send_2811_totalbits(0,0xf,0);
    mico_thread_msleep(500);

    //rst();
    Send_2811_totalbits(0,0,0xf);
    mico_thread_msleep(500);

    Send_2811_totalbits(0xf,0xf,0);
    mico_thread_msleep(500);

    Send_2811_totalbits(0,0xf,0xf);
    mico_thread_msleep(500);

    Send_2811_totalbits(0xf,0,0xf);
    mico_thread_msleep(500);

    Send_2811_totalbits(0xf,0xf,0xf);
    mico_thread_msleep(500);
}
#ifdef __GNUC__
#pragma GCC optimize ("O0") // 关闭优化，否则直接反转 IO 会被优化
#endif /* ifdef __GNUC__ */
/* SDI output R fast */
__attribute__((section(".fast"))) // 代码放到RAM运行
void RGB_SDI_Display()
{
    uint8_t i,j,color;

    //for(i=0;i<2;i++)
    {
        //printf("Display[%d]:%d %d %d\r\n",i,GData[i],RData[i],BData[i]);
        Send_2811_24bits(GData[i],RData[i],BData[i]);
    }
}

#ifdef __GNUC__
#pragma GCC optimize ("O0") // 关闭优化，否则直接反转 IO 会被优化
#endif /* ifdef __GNUC__ */
__attribute__((section(".fast"))) // 代码放到RAM运行
void myNsDelay( uint64_t delayns )
{
    int ti;

    ti = 5*delayns;
    do{
        __ASM("NOP");
        __ASM("NOP");
    } while(--ti);
}
