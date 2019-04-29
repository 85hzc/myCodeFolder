/*
 * simi2c.c
 *
 *	Created on: 2019年3月6日
 *		Author: OP056666
 */

#include "gpioi2c.h"
#include "mico.h"

#define CT_SDA_PIN   MICO_GPIO_13 //254   //定义SDA所对应的GPIO接口编号
#define CT_SCL_PIN   MICO_GPIO_12 //255   //定义SCL所对应的GPIO接口编号
#define CT_RST_PIN   GPIO_Pin_13
#define CT_INT_PIN   GPIO_Pin_14

//#define RCC_CT_I2C_PORT         RCC_APB2Periph_GPIOF
//#define RCC_CT_CONTROL_PORT     RCC_APB2Periph_GPIOF
#define CT_CONTROL_PORT         GPIOF
#define CT_I2C_PORT             GPIOF

//#define TOUCH_GPIO_PortSource   GPIO_PortSourceGPIOF
#define TOUCH_GPIO_PinSource    GPIO_PinSource12
#define TOUCH_EXTI_Line         EXTI_Line12
#define TOUCH_EXTI_IRQn         EXTI15_10_IRQn


#define CT_ADDR                 (0x53)                  //器件地址
#define CT_WRITE_MASK           (0x00)
#define CT_READ_MASK            (0x01)

#define CT_CACK_TIMEOUT         (3000)                  //等待ACK超时时间

#define SDA  CT_SDA_PIN //254                         //定义SDA所对应的GPIO接口编号
#define SCL  CT_SCL_PIN //255                         //定义SCL所对应的GPIO接口编号
#define OUTP OUTPUT_OPEN_DRAIN_PULL_UP     //1          //表示GPIO接口方向为输出
#define INP  INPUT_PULL_DOWN               //0          //表示GPIO接口方向为输入

#define     SDA_High           MicoGpioOutputHigh(CT_SDA_PIN)
#define     SDA_Low            MicoGpioOutputLow(CT_SDA_PIN)
//#define     SDA_INPUT          0//{CT_I2C_PORT->CRH&=0X0FFFFFFF;CT_I2C_PORT->CRH|= 8ul<<28;}//第15脚，(15-8)*4 = 28
//#define     SDA_OUTPUT         1// {CT_I2C_PORT->CRH&=0X0FFFFFFF;CT_I2C_PORT->CRH|= 3ul<<28;}//第15脚，(15-8)*4 = 28

#define     SDA_INPUT          MicoGpioInitialize(SDA, INP);
#define     SDA_OUTPUT         MicoGpioInitialize(SDA, OUTP);
#define     SCL_OUTPUT         MicoGpioInitialize(SCL, OUTP);

#define     SCL_High           MicoGpioOutputHigh(CT_SCL_PIN)
#define     SCL_Low            MicoGpioOutputLow(CT_SCL_PIN)
#define     GetSDABit          MicoGpioInputGet(CT_SDA_PIN)

#define     CT_DELAY_US(val)            Delay_us(val)
#if 0
/*
**函数名：CTI2C_GPIO_Config
**传入参数：无
**返回值：无
**功能：初始化CTI2C引脚
*/
void CTI2C_GPIO_Config(void)
{
    /*定义一个GPIO_InitTypeDef类型的结构体*/
    GPIO_InitTypeDef    GPIO_InitStructure;
    EXTI_InitTypeDef    EXTI_InitStructure;

    /*开启GPIO的外设时钟*/
    RCC_APB2PeriphClockCmd(RCC_CT_I2C_PORT | RCC_CT_CONTROL_PORT, ENABLE);

    /*选择要控制的引脚*/
    GPIO_InitStructure.GPIO_Pin = CT_SDA_PIN | CT_SCL_PIN;
    /*设置引脚模式为通用推挽输出*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    /*设置引脚速率为10MHz */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    /*调用库函数，初始化GPIO*/
    GPIO_Init(CT_I2C_PORT,&GPIO_InitStructure);

    /*选择要控制的引脚*/
    GPIO_InitStructure.GPIO_Pin = CT_RST_PIN;
    /*设置引脚模式为通用推挽输出*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    /*设置引脚速率为10MHz */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    /*调用库函数，初始化GPIO*/
    GPIO_Init(CT_CONTROL_PORT,&GPIO_InitStructure);

    /*选择要控制的引脚*/
    GPIO_InitStructure.GPIO_Pin = CT_INT_PIN;
    /*设置引脚模式为通用推挽输出*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    /*设置引脚速率为10MHz */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    /*调用库函数，初始化GPIO*/
    GPIO_Init(CT_CONTROL_PORT,&GPIO_InitStructure);

    /*********************初始化外部中断**********************/
    /*开启引脚复用AFIO的外设时钟，因为用到了AFIO外部中断配置寄存器*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO , ENABLE);
    /* Selects as EXTI Line */
    GPIO_EXTILineConfig(TOUCH_GPIO_PortSource, TOUCH_GPIO_PinSource);
    /*选择外部中断线*/
    EXTI_InitStructure.EXTI_Line = TOUCH_EXTI_Line;
    /*设置EXTI线路为中断请求*/
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    /*下降沿触发*/
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    /*使能选中线路*/
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    /*外部中断初始化*/
    EXTI_Init(&EXTI_InitStructure);
/******************************************************/
}
#endif

#ifdef CTI2C
/**************************/
static void CTI2C_start(void)//CTI2C启动函数
{
    SDA_OUTPUT;
    SCL_OUTPUT;
    SCL_High;
    SDA_High;
    CT_DELAY_US(300);
    SDA_Low;
    CT_DELAY_US(300);
    SCL_Low;
}

/**************************/
static void CTI2C_stop(void)// CTI2C停止函数
{
    SDA_OUTPUT;
    SDA_Low;
    SCL_High;
    CT_DELAY_US(300);
    SDA_High;
    CT_DELAY_US(300);
    SDA_Low;
    SCL_Low;
}

/**************************/
static void CTI2C_write_byte(const uint8_t s)//CTI2C写1byte函数,S为需要写的内容
{
      uint8_t temps;
      uint8_t dat,i;
      temps=s;
      dat=0x80;
      SDA_OUTPUT;
      for(i=0;i<8;i++)
      {
        if(dat&temps)//对应位为一就发一
         {
            SDA_High;
            CT_DELAY_US(100);//延时一下，保证数据建立时间大于50ns
            //SCL的频率，高速为400KHz，标准为100KHz
            SCL_High;
            CT_DELAY_US(300);
            SCL_Low;
            CT_DELAY_US(300);
         }
         else
         {
            SDA_Low;
            CT_DELAY_US(100);//延时一下，保证数据建立时间大于50ns
            SCL_High;
            CT_DELAY_US(300);
            SCL_Low;
            CT_DELAY_US(300);
         }
         dat=dat>>1;//注意Keil中将有符号数的右移操作作算术右移处理，我需要的是逻辑右移,因此dat要先定义为无符号数，否则会出错
      }
    SDA_High;//主控制器写完后要预先释放对SDA总线的控制
    CT_DELAY_US(100);//留一点时间接收应答信号
}

/**************************/
static void CTI2C_read_byte(uint8_t *s)//CTI2C读1byte函数，数据放在形参中，成功返回1，失败返回0
{
     uint8_t temps=0,i;
     uint8_t text=0x80;
     uint8_t sdain;
     SDA_INPUT;
     SDA_High;//设SDA为输入方式
     for(i=0;i<8;i++)
     {
         SCL_High;//使SDA上的数据有效
         CT_DELAY_US(100);//延时一下，保证SDA已经稳定
         sdain = GetSDABit;//取得SDA上的数据
         if(1==sdain)
         {
            temps |=(text>>i);//先接收高位
         }
         SCL_Low;//读完后允许SDA上的数据刷新，并延时一下让被读器件有时间更新要输出的数据
         CT_DELAY_US(300);
     }
     *s=temps;
    CT_DELAY_US(300);//留一点时间发送应答信号
}
/**************************/
static uint8_t CTI2C_check_ack(void)//CTI2C应答位检查函数,正常应答返回0，否则返回1
{
    uint8_t sdain;
    SDA_INPUT;
    SDA_High;//置SDA为输入
    SCL_High;//使SDA上的数据有效
    CT_DELAY_US(100);//延时一下，保证SDA已经稳定
    sdain = GetSDABit;//取得SDA上的数据
    SCL_Low;
    if(1==sdain)
        return 1;
    else
        return 0;
}

/**************************/
static void CTI2C_send_ack(void)//CTI2C发送应答信号函数
{
    SDA_OUTPUT;
    SDA_Low;
    CT_DELAY_US(100);//延时一下，保证SDA已经稳定
    SCL_High;
    CT_DELAY_US(300);
    SCL_Low;
    SDA_High;

}
/**************************/
static void CTI2C_send_nack(void)//CTI2C发送非应答信号函数
{
    SDA_OUTPUT;
    SDA_High;
    CT_DELAY_US(100);//延时一下，保证SDA已经稳定
    SCL_High;
    CT_DELAY_US(300);
    SCL_Low;
    SDA_Low;
}
/**************************/

/******************************************************************************
 * FUNCTION: CT_Write_Nbyte ( )
 * DESCRIPTION: 写触摸屏寄存器
 *    Input the description of function:
 * Input Parameters: 器件地址，待写寄存器地址，待写数据数量，存储待写入数据的地址
 * Output Parameters: 无
 * Returns Value:
 *
 * Author: FuDongQiang @ 2015/05/22
 *
 * modification history
 *   ...
 ******************************************************************************/
CT_RET_STATUS CT_Write_Nbyte(const uint8_t sla_add,const uint8_t add,uint16_t n,const uint8_t *s)//CTI2C写nbyte函数，写数据由形参数组传入，成功返回1，失败返回0
{
    uint8_t temps,ack=1;
    uint16_t tempn;
    uint16_t cack_time=0;
    CTI2C_start();//启动CTI2C总线
    CTI2C_write_byte(sla_add | CT_WRITE_MASK);//发送寻址字节
    do
    {
       cack_time++;
       if(cack_time>CT_CACK_TIMEOUT)//在规定时间cack_timeout内收不到应答信号，返回出错信号
        return CT_ACK_FAIL;
    }while(CTI2C_check_ack());

    CTI2C_write_byte(add);//发送要写入的起始地址
    cack_time=0;
    do
    {
       cack_time++;
       if(cack_time>CT_CACK_TIMEOUT)//在规定时间cack_timeout内收不到应答信号，返回出错信号
        return CT_ACK_FAIL;
    }while(CTI2C_check_ack());

    for(tempn=0;tempn<n;tempn++)
    {
        ack=1;//应答位
        cack_time=0;
        temps=*(s+tempn);
        while(ack)
        {
            CTI2C_write_byte(temps);
            ack=CTI2C_check_ack();//检查应答信号，非应答则重发该字节
            cack_time++;
            if(cack_time>CT_CACK_TIMEOUT)//在规定时间cack_timeout内收不到应答信号，返回出错信号
                return CT_ACK_FAIL;
        }
    }
    CTI2C_stop();// CTI2C停止
    return CT_COM_OK;
}

/******************************************************************************
 * FUNCTION: CT_Read_Nbyte ( )
 * DESCRIPTION: 从触摸屏中读出数据
 *    Input the description of function:
 * Input Parameters: 器件地址，待读寄存器地址，待读数据数量，存储待读出数据的地址
 * Output Parameters: 读取的数据
 * Returns Value:
 *
 * Author: FuDongQiang @ 2015/05/22
 *
 * modification history
 *   ...
 ******************************************************************************/
CT_RET_STATUS CT_Read_Nbyte(const uint8_t sla_add,const uint8_t add,uint16_t n,uint8_t *s)//CTI2C读nbyte函数,所读数据放在形参数组中（由程序员设置合适的数组大小），成功返回1，失败返回0
{
    uint8_t temps;
    uint16_t tempn;
    uint16_t cack_time=0;
    CTI2C_start();//启动CTI2C总线
    CTI2C_write_byte(sla_add | CT_WRITE_MASK);//发送寻址字节，伪写
    do
    {
       cack_time++;
       if(cack_time>CT_CACK_TIMEOUT)//在规定时间cack_timeout内收不到应答信号，返回出错信号
        return CT_ACK_FAIL;
    }while(CTI2C_check_ack());

    CTI2C_write_byte(add);//发送要读入的起始地址
    cack_time=0;
    do
    {
       cack_time++;
       if(cack_time>CT_CACK_TIMEOUT)//在规定时间cack_timeout内收不到应答信号，返回出错信号
        return CT_ACK_FAIL;
    }while(CTI2C_check_ack());

    CTI2C_start();//再次启动CTI2C总线
    CTI2C_write_byte(sla_add | CT_READ_MASK);//再次发送寻址字节,读
    cack_time=0;
    do
    {
       cack_time++;
       if(cack_time>CT_CACK_TIMEOUT)//在规定时间cack_timeout内收不到应答信号，返回出错信号
        return CT_ACK_FAIL;
    }
    while(CTI2C_check_ack());

    for(tempn=0;tempn<n;tempn++)
    {
        CTI2C_read_byte(&temps);
        *(s+tempn)=temps;
        if(tempn+1<n)//如果已经读完所有数据了，就不要发应答信号，直接发非应答位和停止位
            CTI2C_send_ack();//CTI2C发送应答信号.准备下一字节的接收
    }
    CTI2C_send_nack();//接收完毕，发送非应答位和停止位
    CTI2C_stop();
    return CT_COM_OK;
}

#else

/* I2C起始条件 */
int i2c_start()
{
	//初始化GPIO口
	MicoGpioInitialize(SDA, OUTP);
	MicoGpioInitialize(SCL, OUTP);

	MicoGpioOutputHigh(SDA);
	MicoGpioOutputHigh(SCL);

	mico_thread_msleep(1);
	MicoGpioOutputLow(SDA);
	mico_thread_msleep(1);
	MicoGpioOutputLow(SCL);
}


/* I2C终止条件 */
void i2c_stop()
{
	MicoGpioInitialize(SDA, OUTP);

	MicoGpioOutputLow(SDA);
	MicoGpioOutputHigh(SCL);

	mico_thread_msleep(1);
	MicoGpioOutputHigh(SDA);
	mico_thread_msleep(1);
	MicoGpioOutputLow(SDA);
	MicoGpioOutputLow(SCL);
}

/*
I2C读取ACK信号(写数据时使用)
返回值 ：0表示ACK信号有效；非0表示ACK信号无效
*/
unsigned char i2c_read_ack()
{
	unsigned char r;
	MicoGpioInitialize(SDA, INP);
	MicoGpioOutputLow(SCL);
	r = MicoGpioInputGet(SDA);
	mico_thread_msleep(1);
	MicoGpioOutputHigh(SCL);
	mico_thread_msleep(1);

	return r;
}


/* I2C发出ACK信号(读数据时使用) */
int i2c_send_ack()
{
	MicoGpioInitialize(SDA, OUTP);
	MicoGpioOutputLow(SCL);
	MicoGpioOutputLow(SDA);
	mico_thread_msleep(1);
	MicoGpioOutputHigh(SCL);
	mico_thread_msleep(1);
}


/* I2C字节写 */
void i2c_write_byte(unsigned char b)
{
	int i;
	MicoGpioInitialize(SDA, OUTP);          //设置SDA方向为输出
	for (i=7; i>=0; i--)
	{
		MicoGpioOutputLow(SCL);             // SCL变低
		mico_thread_msleep(5);
		if(b & (1<<i))
		{
			MicoGpioOutputHigh(SDA);
		}
		else
		{
			MicoGpioOutputLow(SDA);
		}
		MicoGpioOutputHigh(SCL);             // SCL变高
		mico_thread_msleep(5);
	}
	i2c_read_ack();                 //检查目标设备的ACK信号
}


/* I2C字节读 */
unsigned char i2c_read_byte()
{
	int i;
	unsigned char r = 0;
	MicoGpioInitialize(SDA, INP);           //设置SDA方向为输入
	for (i=7; i>=0; i--)
	{
		MicoGpioOutputLow(SCL);         // SCL变低
		mico_thread_msleep(5);
		r = (r <<1) | MicoGpioInputGet(SDA);      //从高位到低位依次准备数据进行读取
		MicoGpioOutputHigh(SCL);         // SCL变高
		mico_thread_msleep(5);
	}
	i2c_send_ack();                 //向目标设备发送ACK信号
	return r;
}


/*
I2C读操作
addr：目标设备地址
buf：读缓冲区
len：读入字节的长度
*/
void i2c_read(unsigned char addr, unsigned char* buf, int len)
{
	int i;
	unsigned char t;
	i2c_start();                        //起始条件，开始数据通信
	//发送地址和数据读写方向
	t = (addr << 1) | 1;                    //低位为1，表示读数据
	i2c_write_byte(t);
	//读入数据
	for (i=0; i<len; i++)
		buf[i] = i2c_read_byte();
	i2c_stop();                     //终止条件，结束数据通信
}

/*
I2C写操作
addr：目标设备地址
buf：写缓冲区
len：写入字节的长度
*/
void i2c_write(unsigned char addr, unsigned char* buf, int len)
{
	int i;
	unsigned char t;
	i2c_start();                        //起始条件，开始数据通信
	//发送地址和数据读写方向
	t = (addr << 1) | 0;                    //低位为0，表示写数据
	i2c_write_byte(t);
	//写入数据
	for (i=0; i<len; i++)
	    i2c_write_byte(buf[i]);
	i2c_stop();                     //终止条件，结束数据通信
}
#endif
