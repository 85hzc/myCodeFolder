/*
 * rgbdriver.c
 *
 *  Created on: 2019年4月1日
 *      Author: OP056666
 */
#include "mico.h"
#include "rgbdisplay.h"

#define T1 1

#define GPIO_RGB_LE     MICO_GPIO_7  //scl
#define GPIO_RGB_GCLK   MICO_GPIO_14 //pwm5 d4
#define GPIO_RGB_DCLK   MICO_GPIO_13 //pwm4 d8

#define GPIO_RGB_SDI_R1  MICO_GPIO_8 //sda
#define GPIO_RGB_SDI_G1  MICO_GPIO_9
#define GPIO_RGB_SDI_B1  MICO_GPIO_12

#define GPIO_RGB_SDI_R2  MICO_GPIO_21//_21 app can not work
#define GPIO_RGB_SDI_G2  MICO_GPIO_22
#define GPIO_RGB_SDI_B2  MICO_GPIO_10

extern uint8_t circle;

void GPIO_RGB_init()
{
/*
    MicoGpioInitialize(GPIO_RGB_LE, OUTPUT_PUSH_PULL);
    MicoGpioInitialize(GPIO_RGB_DCLK, OUTPUT_PUSH_PULL);
    MicoGpioInitialize(GPIO_RGB_SDI_R1, OUTPUT_PUSH_PULL); //app can not work
*/

    MicoGpioInitialize(GPIO_RGB_LE, OUTPUT_OPEN_DRAIN_NO_PULL);
    MicoGpioInitialize(GPIO_RGB_DCLK, OUTPUT_OPEN_DRAIN_NO_PULL);
    MicoGpioInitialize(GPIO_RGB_SDI_R1, OUTPUT_OPEN_DRAIN_NO_PULL); //app can not work

    //MicoGpioInitialize(GPIO_RGB_SDI_G1, OUTPUT_OPEN_DRAIN_PULL_UP);
    //MicoGpioInitialize(GPIO_RGB_SDI_B1, OUTPUT_OPEN_DRAIN_PULL_UP);
}

#ifdef __GNUC__
#pragma GCC optimize ("O0") // 关闭优化，否则直接反转 IO 会被优化
#endif /* ifdef __GNUC__ */
/* SDI output R fast */
__attribute__((section(".fast"))) // 代码放到RAM运行
void MBI_init(void)
{
    __set_FAULTMASK(1); // 关闭调度器
    register_config();
    __set_FAULTMASK(0); // 打开调度器
}

/******************************************************************************
功能说明：        数据锁栓及显示
输入参数：        无
输出参数：        无
返回值：        无
******************************************************************************/
#ifdef __GNUC__
#pragma GCC optimize ("O0") // 关闭优化，否则直接反转 IO 会被优化
#endif /* ifdef __GNUC__ */
/* SDI output R fast */
__attribute__((section(".fast"))) // 代码放到RAM运行
void vsync_data_display(void)
{
    unsigned char i;

    MicoPwmStop(MICO_PWM_3);
    myNsDelay(1);
    gpio_output_low(GPIO_RGB_DCLK);
    myNsDelay(5);

    gpio_output_high(GPIO_RGB_LE);
    myNsDelay(1);
    for(i=14;i<16;i++)
    {
        gpio_output_high(GPIO_RGB_DCLK);
        myNsDelay(1);
        gpio_output_low(GPIO_RGB_DCLK);
        myNsDelay(1);
    }
    gpio_output_low(GPIO_RGB_LE);

    myNsDelay(5);
    //死时间
    MicoPwmStart(MICO_PWM_3);
}

void reg1_config()
{

}

#ifdef __GNUC__
#pragma GCC optimize ("O0") // 关闭优化，否则直接反转 IO 会被优化
#endif /* ifdef __GNUC__ */
/* SDI output R fast */
__attribute__((section(".fast"))) // 代码放到RAM运行
void soft_reset(void)
{
    uint8_t sck_cnt;
#if 1
    //JXI5020_PORT -> BSRRL = JXI5020_LE;//LE置为高电平
    gpio_output_high(GPIO_RGB_LE);
    for(sck_cnt = 10;sck_cnt > 0;sck_cnt --)
    {
        //JXI5020_PORT ->BSRRL = JXI5020_SCK;
        gpio_output_high(GPIO_RGB_DCLK);
        myNsDelay(1);
        //JXI5020_PORT -> BSRRH = JXI5020_SCK;
        gpio_output_low(GPIO_RGB_DCLK);
        myNsDelay(1);
    }
    //JXI5020_PORT -> BSRRH = JXI5020_LE;//LE置为低电平
    gpio_output_low(GPIO_RGB_LE);
#endif
}

#ifdef __GNUC__
#pragma GCC optimize ("O0") // 关闭优化，否则直接反转 IO 会被优化
#endif /* ifdef __GNUC__ */
/* SDI output R fast */
__attribute__((section(".fast"))) // 代码放到RAM运行
void WriteData(uint16_t Dat,uint8_t cntbits,mico_gpio_t pin)  //将数据Dat写入cnt bits
{
    uint8_t i;
    uint16_t temp;
    temp=Dat;

    //os_rgb_log("sdi:%x",temp);
    for(i=0;i<cntbits;i++)
    {
#if 0
        CLK=1;
        DIN=temp&0x8000;
        //delay(10);
        CLK=0;
        temp=temp<<1;
#else
        if(i==15)//
        {
            gpio_output_high(GPIO_RGB_LE);
            myNsDelay(1);
        }

        if(temp&0x8000)
            gpio_output_high(pin);
        else
            gpio_output_low(pin);
        myNsDelay(1);

        gpio_output_high(GPIO_RGB_DCLK);
        myNsDelay(1);
        /*if(temp&0x8000)
            gpio_output_high(pin);
        else
            gpio_output_low(pin);
        myNsDelay(1);*/
        gpio_output_low(GPIO_RGB_DCLK);
        myNsDelay(1);
        if(i==15)
            gpio_output_low(GPIO_RGB_LE);
#endif
        temp=temp<<1;
    }

    gpio_output_low(pin);//reset sdi pin
    myNsDelay(10);
}
#if 0
void state_set_data_16bit()
{
    uint8_t i=0;
/****************前置时间*********************/
    CLK=0;
    DIN=0;
    delay(10);
/*********************************************/
/****************48位标头***************************/
    WriteData(0x8c00,6);    //16位状态设定数据命令，H[5:0]100011
    WriteData(0x0000,10);   //地址数据，A[9:0],10b'0
    WriteData(0x8c00,6);    //确认命令 H[5:0]
    WriteData(0x0200,10);   //设定串行驱动器数量L[9:0]=N-1,N驱动器数量    N=9
    WriteData(0x6000,4);    //4位校验码，0101
    WriteData(0x0000,2);    //X[1:0]任意值 ，建议0
    WriteData(0x0200,10);   //再次确定驱动器数量
/*************************************************/
/*****************48位状态设定数据********************************/
    for(i=0;i<size;i++)
    {
        WriteData(0x0000,6);     //X2[5:0]
        WriteData(0x9f80,10);    //CF1:0x027e  低10位
        WriteData(0x0000,6);     //X2[5:0]
        WriteData(0x9f80,10);    //CF1:0x027e  低10位
        WriteData(0x0000,13);    //X3[12:0]
        WriteData(0xe000,3);     //CF2[2:0]
    }
//  delay(10);
}
void Dot_Check_data_8bit(uint RR,uint GG,uint BB)       //8位点校正数据
{
    uint8_t i=0,j=0;
/****************前置时间*********************/
    CLK=0;
    DIN=0;
    delay(3);
/*********************************************/
/****************48位标头***************************/
    WriteData(0xcc00,6);    //16位状态设定数据命令，H[5:0]110011
    WriteData(0x0000,10);   //地址数据，A[9:0],10b'0
    WriteData(0xcc00,6);    //确认命令 H[5:0]
    WriteData(0x0200,10);   //设定串行驱动器数量L[9:0]=N-1,N驱动器数量 N=9
    WriteData(0x9000,4);    //4位校验码，1001
    WriteData(0x0000,2);    //X[1:0]任意值 ，建议0
    WriteData(0x0200,10);   //再次确定驱动器数量
/*************************************************/
/*****************192位点校正数据********************************/
    for(i=0;i<size;i++)
    {
        for(j=0;j<4;j++)
        {
            WriteData(RR,16);
            WriteData(GG,16);
            WriteData(BB,16);
        }
    }
}
#endif

#ifdef __GNUC__
#pragma GCC optimize ("O0") // 关闭优化，否则直接反转 IO 会被优化
#endif /* ifdef __GNUC__ */
/* SDI output R fast */
__attribute__((section(".fast"))) // 代码放到RAM运行
void send_data_16bit(uint16_t *RR,uint16_t *GG,uint16_t *BB)        //传送16位灰度数据    ,三组相同
{
    uint8_t i=0,j=0;

    /****************前置时间*********************/
    //CLK=0;
    //DIN=0;
    gpio_output_low(GPIO_RGB_DCLK);
    gpio_output_low(GPIO_RGB_SDI_R1);
    //gpio_output_low(GPIO_RGB_SDI_G1);
    //gpio_output_low(GPIO_RGB_SDI_B1);
    myNsDelay(1);
    /*********************************************/
#if 0
    /****************48位标头***************************/
    WriteData(0xfc00,6);    //16位状态设定数据命令，H[5:0]111111
    WriteData(0x0000,10);   //地址数据，A[9:0],10b'0
    WriteData(0xfc00,6);    //确认命令 H[5:0]
    WriteData(0x0200,10);   //设定串行驱动器数量L[9:0]=N-1,N驱动器数量    N=9
    WriteData(0x9000,4);    //4位校验码，1001
    WriteData(0x0000,2);    //X[1:0]任意值 ，建议0
    WriteData(0x0200,10);   //再次确定驱动器数量
    /*************************************************/
#endif

    /*****************4*3组192位灰度数据********************************/
    for(i=0;i<MBI5153_SIZE;i++)
    {
        for(j=0;j<16;j++)//16 channels
        {
            //WriteData(*(RR+i),16,GPIO_RGB_SDI_R1);//16 bits
            WriteData((circle%16==j)?RR[1]:0,16,GPIO_RGB_SDI_R1);//16 bits
            //WriteData(GG,16);
            //WriteData(BB,16);
        }
    }
}
/*
void gclkTrigger_thread(uint32_t inContext)
{
    rgb_display_context_t *context = (rgb_display_context_t *)inContext;

    os_rgb_log("gclkTrigger_thread.");

    while(1){

        mico_rtos_thread_sleep(1);
    }
}
*/
#ifdef __GNUC__
#pragma GCC optimize ("O0") // 关闭优化，否则直接反转 IO 会被优化
#endif /* ifdef __GNUC__ */
/* SDI output R fast */
__attribute__((section(".fast"))) // 代码放到RAM运行
void RGB_SDI_R_schedule( uint16_t *bits, uint32_t H_pixels)
{
    uint16_t    *bit;
    int         H_scan, bit_scan=0;

    bit = bits;
    __set_FAULTMASK(1); // 关闭调度器
    //register_config();
    myNsDelay(10);

    send_data_16bit(bit,NULL,NULL);

    //myNsDelay(1);
    vsync_data_display();//display
    //myNsDelay(1);
    //soft_reset();//0复位

    __set_FAULTMASK(0); // 打开调度器
}

#ifdef __GNUC__
#pragma GCC optimize ("O0") // 关闭优化，否则直接反转 IO 会被优化
#endif /* ifdef __GNUC__ */
/* SDI output R fast */
__attribute__((section(".fast"))) // 代码放到RAM运行
void register_config(void)
{
    uint16_t reg, sck_cnt, j, mask;

    //reg = 0x002B;
    //__set_FAULTMASK(1); // 关闭调度器
    //vsync_data_display();

    /****************前置时间*********************/
    gpio_output_low(GPIO_RGB_DCLK);
    gpio_output_low(GPIO_RGB_LE);
    gpio_output_low(GPIO_RGB_SDI_R1);
    //gpio_output_low(GPIO_RGB_SDI_G1);
    //gpio_output_low(GPIO_RGB_SDI_B1);


    //1写入状态寄存器reg1
    //a写状态寄存器之前的前置设定指定
    reg = 0x06B;
    myNsDelay(1);
    gpio_output_high(GPIO_RGB_LE);
    for(sck_cnt = 14;sck_cnt > 0;sck_cnt --)
    {
        gpio_output_high(GPIO_RGB_DCLK);
        myNsDelay(1);
        gpio_output_low(GPIO_RGB_DCLK);
        myNsDelay(1);
    }
    gpio_output_low(GPIO_RGB_LE);

    myNsDelay(10);
    //b写状态寄存器
    for(j = 0;j < MBI5153_SIZE;j++)//每个通道由4片MBI5052级联，每个通道锁存一次数据
    {
        for(sck_cnt = 0;sck_cnt < 16;sck_cnt ++)
        {
            mask = 0x8000 >> sck_cnt;

            if(j == MBI5153_SIZE-1) {
                if(sck_cnt == 12) {//4 sclk when LE
                    gpio_output_high(GPIO_RGB_LE);
                }
            }

            if(reg & mask){
                gpio_output_high(GPIO_RGB_SDI_R1);
            } else {
                gpio_output_low(GPIO_RGB_SDI_R1);
            }

            myNsDelay(1);
            gpio_output_high(GPIO_RGB_DCLK);
            myNsDelay(1);
            gpio_output_low(GPIO_RGB_DCLK);
        }
    }
    gpio_output_low(GPIO_RGB_LE);
    gpio_output_low(GPIO_RGB_SDI_R1);


    //1写入状态寄存器reg2
    //a写状态寄存器之前的前置设定指定
    reg = 0x400;
    myNsDelay(1);
    gpio_output_high(GPIO_RGB_LE);
    for(sck_cnt = 14;sck_cnt > 0;sck_cnt --)
    {
        gpio_output_high(GPIO_RGB_DCLK);
        myNsDelay(1);
        gpio_output_low(GPIO_RGB_DCLK);
        myNsDelay(1);
    }
    gpio_output_low(GPIO_RGB_LE);

    myNsDelay(10);
    //b写状态寄存器
    for(j = 0;j < MBI5153_SIZE;j++)//每个通道由4片MBI5052级联，每个通道锁存一次数据
    {
        for(sck_cnt = 0;sck_cnt < 16;sck_cnt ++)
        {
            mask = 0x8000 >> sck_cnt;

            if(j == MBI5153_SIZE-1) {
                if(sck_cnt == 8) {//4 sclk when LE
                    gpio_output_high(GPIO_RGB_LE);
                }
            }

            if(reg & mask){
                gpio_output_high(GPIO_RGB_SDI_R1);
            } else {
                gpio_output_low(GPIO_RGB_SDI_R1);
            }

            myNsDelay(1);
            gpio_output_high(GPIO_RGB_DCLK);
            myNsDelay(1);
            gpio_output_low(GPIO_RGB_DCLK);
        }
    }
    gpio_output_low(GPIO_RGB_LE);
    gpio_output_low(GPIO_RGB_SDI_R1);

    //__set_FAULTMASK(0); // 打开调度器
}

__attribute__((section(".fast"))) // 代码放到RAM运行
void myNsDelay( uint64_t delayns )
{
    int ti;

    ti = 10*delayns;
    do{
        __ASM("NOP");
        __ASM("NOP");
    } while(--ti);
}
