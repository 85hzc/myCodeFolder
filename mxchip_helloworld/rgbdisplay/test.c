/*
 * test.c
 *
 *  Created on: 2019年4月22日
 *      Author: OP056666
 */


void dispaly_process(void) //共7us
{
    unsigned char i, j;
    unsigned char k;
    unsigned char rdata1[64];

    unsigned int wdata1;
    unsigned int wdata2;

    for (j = 0; j < 64; j++)
    {
        JXI5020_PORT ->BRR  = UR14_SDO | UG14_SDO | UB14_SDO | UR13_SDO | UG13_SDO | UB13_SDO;
        JXI5020_PORT ->BSRR = JXI5020_SCK;
        JXI5020_PORT -> BRR = JXI5020_SCK;
    }
    JXI5020_PORT -> BSRR = JXI5020_LE;
    JXI5020_PORT -> BRR  = JXI5020_LE;
    //JXI5020_PORT -> BSRR = JXI5020_OE;
    if(led_dis_cnt < 8)
        D7258_PORT -> BSRR = D7258_EN;
    else
        D7258_PORT -> BRR = D7258_EN;

    switch(led_dis_cnt)
    {
        case 7:
        case 15:
                //D7258_PORT -> BRR = D7258_A | D7258_B | D7258_C;
                D7258_PORT -> BSRR = (D7258_A | D7258_B | D7258_C)<<16;
                break;
        case 6:
        case 14:
                D7258_PORT -> BSRR = D7258_A | ((D7258_B | D7258_C)<<16);
                //D7258_PORT -> BRR = D7258_B | D7258_C;
                break;
        case 5:
        case 13:
                //D7258_PORT -> BRR = D7258_A | D7258_C;
                D7258_PORT -> BSRR = D7258_B | ((D7258_A | D7258_C)<<16);
                break;
        case 4:
        case 12:
                D7258_PORT -> BSRR = D7258_A | D7258_B |((D7258_C)<<16);;
                //D7258_PORT -> BRR = D7258_C;
                break;
        case 3:
        case 11:
                //D7258_PORT -> BRR = D7258_A | D7258_B;
                D7258_PORT -> BSRR = D7258_C | ((D7258_A | D7258_B)<<16);
                break;
        case 2:
        case 10:
                D7258_PORT -> BSRR = D7258_A | D7258_C | (D7258_B<<16);
                //D7258_PORT -> BRR = D7258_B;
                break;
        case 1:
        case 9:
                //D7258_PORT -> BRR = D7258_A;
                D7258_PORT -> BSRR = D7258_B | D7258_C | (D7258_A<<16);
                break;
        case 0:
        case 8:
                D7258_PORT -> BSRR = D7258_A | D7258_B | D7258_C;
                break;
    }

    for (j = 64; j > 0; j--)
    {
        if(leddisbuf[64*led_dis_cnt + j - 1] & 0x01)
            JXI5020_PORT ->BSRR = UR14_SDO;
        else
            JXI5020_PORT ->BRR = UR14_SDO;

        if(leddisbuf[64*led_dis_cnt + j - 1] & 0x02)
            JXI5020_PORT ->BSRR = UG14_SDO;
        else
            JXI5020_PORT ->BRR = UG14_SDO;

        if(leddisbuf[64*led_dis_cnt + j - 1] & 0x04)
            JXI5020_PORT ->BSRR = UB14_SDO;
        else
            JXI5020_PORT ->BRR = UB14_SDO;

        //xia
        if(leddisbuf[64*(16+led_dis_cnt) + j - 1] & 0x01)
            JXI5020_PORT ->BSRR = UR13_SDO;
        else
            JXI5020_PORT ->BRR = UR13_SDO;

        if(leddisbuf[64*(16+led_dis_cnt) + j - 1] & 0x02)
            JXI5020_PORT ->BSRR = UG13_SDO;
        else
            JXI5020_PORT ->BRR = UG13_SDO;

        if(leddisbuf[64*(16+led_dis_cnt) + j - 1] & 0x04)
            JXI5020_PORT ->BSRR = UB13_SDO;
        else
            JXI5020_PORT ->BRR = UB13_SDO;

        JXI5020_PORT ->BSRR = JXI5020_SCK;
        JXI5020_PORT -> BRR = JXI5020_SCK;
    }

    JXI5020_PORT -> BSRR = JXI5020_LE;
    JXI5020_PORT -> BRR  = JXI5020_LE;
    //JXI5020_PORT -> BRR  = JXI5020_OE;
    led_dis_cnt++;
    if (led_dis_cnt == 16)
    {
            led_dis_cnt = 0;
    }
}

  以下是扫描的代码：
{
    unsigned int sck_cnt;
    unsigned short i,j,k,m;
    unsigned short state_reg=(1<<GCLK_FROM_DCLK)|(15<<LINE_16_NUMS)|(0<<PWM_GCLKS16BIT)|(0<<GCLK_DOUBLE)|0x2b;
    unsigned short red1;
    unsigned short green1;
    unsigned short blue1;
    unsigned short red2;
    unsigned short green2;
    unsigned short blue2;
    unsigned int mask;
    unsigned int bufaddA,bufaddB;

    //1写入状态寄存器
    //a写状态寄存器之前的前置设定指定
    JXI5020_PORT -> BSRRL = JXI5020_LE;//LE置为高电平
    for(sck_cnt = 14;sck_cnt > 0;sck_cnt --)
    {
            JXI5020_PORT ->BSRRL = JXI5020_SCK;
            delay(2);
            JXI5020_PORT -> BSRRH = JXI5020_SCK;
            delay(1);
    }
    JXI5020_PORT -> BSRRH = JXI5020_LE;//LE置为低电平
    //b写状态寄存器
    for(j = 0;j < 4;j++)//每个通道由4片MBI5052级联，每个通道锁存一次数据
    {
        for(sck_cnt = 0;sck_cnt < 16;sck_cnt ++)
        {
            mask = 0x8000 >> sck_cnt;

            if(j == 3)
                if(sck_cnt == 12)
                    JXI5020_PORT -> BSRRL = JXI5020_LE;//LE置为高电平
            if(state_reg & mask)
                JXI5020_PORT -> BSRRL  = UR14_SDO | UG14_SDO | UB14_SDO | UR13_SDO | UG13_SDO | UB13_SDO;
            else
                JXI5020_PORT ->BSRRH  = UR14_SDO | UG14_SDO | UB14_SDO | UR13_SDO | UG13_SDO | UB13_SDO;
            delay(1);
            JXI5020_PORT -> BSRRL = JXI5020_SCK;
            delay(1);
            JXI5020_PORT -> BSRRH = JXI5020_SCK;
            //delay(1);
        }
    }
    JXI5020_PORT -> BSRRH = JXI5020_LE;//LE置为低电平

    //2写入16*2*16的数据
    for(m = 0;m < 16;m++)//共有16行
    {
        for(i = 0;i < 16;i++)//每个MBI5052有16个通道，
        //for(i = 0;i < 1;i++)
        {
            for(j = 0;j < 4;j++)//每个通道由4片MBI5052级联，每个通道锁存一次数据
            //for(j = 0;j < 1;j++)
            {
#if 0
                bufaddA = 0x000000ff;
                bufaddB = 0x0000ff00;;//8位原始数据对应到16位PWM数据
#else
                bufaddA = leddisbuf[((m+1)*64-1) - j*16 - i];
                bufaddB = leddisbuf[((m+1)*64-1) - j*16 - i + 64*16];//8位原始数据对应到16位PWM数据
#endif
#if 0
                if((m == 1) && (i == 0) && (j == 0))//m == 4 //用于测试数据显示的位置
                {
                        red1 =   (bufaddA & 0xff0000) >> 8;//2 * (leddisbuf[i*64 + j] & 0xff0000>>16)上半屏数据
                        green1 = (bufaddA & 0x00ff00) >> 0;//2 * (leddisbuf[i*64 + j] & 0x00ff00>>8)上半屏数据
                        blue1 =  (bufaddA & 0x0000ff) << 8;//上半屏数据
                        red2 =   (bufaddB & 0xff0000) >> 8;//上半屏数据
                        green2 = (bufaddB & 0x00ff00) >> 0;//上半屏数据
                        blue2 =  (bufaddB & 0x0000ff) << 8;//上半屏数据
                }
                else
                {
                        red1 =   0x0000;
                        green1 = 0x0000;
                        blue1 =  0x0000;//(leddisbuf[bufaddA] & 0x0000ff) << 1;//上半屏数据
                        red2 =   0x0000;//(leddisbuf[bufaddB] & 0xff0000) >> 15;//上半屏数据
                        green2 = 0x0000;//(leddisbuf[bufaddB] & 0x00ff00) >> 7;//上半屏数据
                        blue2 =  0x0000;//(leddisbuf[bufaddB] & 0x0000ff) << 1;//上半屏数据
                }
#else
                red1 =   (bufaddA & 0xff0000) >> 8;//2 * (leddisbuf[i*64 + j] & 0xff0000>>16)上半屏数据
                green1 = (bufaddA & 0x00ff00) >> 0;//2 * (leddisbuf[i*64 + j] & 0x00ff00>>8)上半屏数据
                blue1 =  (bufaddA & 0x0000ff) << 8;//上半屏数据
                red2 =   (bufaddB & 0xff0000) >> 8;//上半屏数据
                green2 = (bufaddB & 0x00ff00) >> 0;//上半屏数据
                blue2 =  (bufaddB & 0x0000ff) << 8;//上半屏数据
#endif

                //if(m == 0)red1 = 255;
                for(k = 0;k < 16;k++)
                {
                    mask = 0x8000 >> k;
                    if(j == 3)
                        if(k == 15)
                            JXI5020_PORT -> BSRRL = JXI5020_LE;//最后一个位LE置为高电平
#if 1
                    if(red1 & mask)
                        JXI5020_PORT -> BSRRL = UR14_SDO;
                    else
                        JXI5020_PORT -> BSRRH = UR14_SDO;

                    if(green1 & mask)
                        JXI5020_PORT -> BSRRL = UG14_SDO;
                    else
                        JXI5020_PORT -> BSRRH = UG14_SDO;

                    if(blue1 & mask)
                        JXI5020_PORT -> BSRRL = UB14_SDO;
                    else
                        JXI5020_PORT -> BSRRH = UB14_SDO;

                    if(red2 & mask)
                        JXI5020_PORT -> BSRRL = UR13_SDO;
                    else
                        JXI5020_PORT -> BSRRH = UR13_SDO;
                    if(green2 & mask)
                        JXI5020_PORT -> BSRRL = UG13_SDO;
                    else
                        JXI5020_PORT -> BSRRH = UG13_SDO;

                    if(blue2 & mask)
                        JXI5020_PORT -> BSRRL = UB13_SDO;
                    else
                        JXI5020_PORT -> BSRRH = UB13_SDO;

                    delay(1);
                    JXI5020_PORT -> BSRRL = JXI5020_SCK;
#else
                    JXI5020_PORT -> BSRRL = UR14_SDO | UG14_SDO;
#endif
                    delay(1);
                    //delayins++;
                    JXI5020_PORT -> BSRRH = JXI5020_SCK;
                }
            }
            JXI5020_PORT -> BSRRH = JXI5020_LE;//LE置为低电平
        }
    }

    //3发送至少50个GCLK 最后3个CLK拉高LE，发送VSYNC
    for(sck_cnt = 0;sck_cnt < 60;sck_cnt ++)
    {
        if(sck_cnt == 60 - 3)
            JXI5020_PORT -> BSRRL = JXI5020_LE;//LE置为高电平

        //if(state_reg & mask)JXI5020_PORT ->BSRRL  = UR14_SDO | UG14_SDO | UB14_SDO | UR13_SDO | UG13_SDO | UB13_SDO;
        JXI5020_PORT ->BSRRH  = UR14_SDO | UG14_SDO | UB14_SDO | UR13_SDO | UG13_SDO | UB13_SDO;
        delay(1);
        JXI5020_PORT ->BSRRL = JXI5020_SCK;
        delay(1);
        if(sck_cnt < 59)//最后一个不要拉低
            JXI5020_PORT -> BSRRH = JXI5020_SCK;//拉低SCK
    }
    JXI5020_PORT -> BSRRH = JXI5020_LE;//LE置为低电平
    delay(2);//LE下降沿与GCLK上升沿满足要求
    //4消隐
    for(sck_cnt = 0;sck_cnt < 512;sck_cnt ++)
    {
        JXI5020_PORT ->BSRRH  = UR14_SDO | UG14_SDO | UB14_SDO | UR13_SDO | UG13_SDO | UB13_SDO;
        JXI5020_PORT ->BSRRL = JXI5020_SCK;
        delay(1);
        //JXI5020_PORT -> BSRRH = JXI5020_SCK;
    }
    JXI5020_PORT -> BSRRH = JXI5020_SCK;//SCK置为低电平

    //5开始显示
    for(i = 0;i < 16; i++)
    {
        //设置行电平
        if(i < 8)
            D7258_PORT -> BSRRL = D7258_EN;
        else
            D7258_PORT -> BSRRH = D7258_EN;

        switch(i)
        {
            case 7:
            case 15:
                D7258_PORT_BSRR = (D7258_A | D7258_B | D7258_C)<<16;
                //D7258_PORT -> BSRRH = D7258_A | D7258_B | D7258_C;
                //D7258_PORT -> BSRRL = 0;
                break;
            case 6:
            case 14:
                D7258_PORT_BSRR = D7258_A | ((D7258_B | D7258_C)<<16);
                //D7258_PORT -> BSRRH = D7258_B | D7258_C;
                //D7258_PORT -> BSRRL = D7258_A;
                break;
            case 5:
            case 13:
                D7258_PORT_BSRR = D7258_B | ((D7258_A | D7258_C)<<16);
                //D7258_PORT -> BSRRH = D7258_A | D7258_C;
                //D7258_PORT -> BSRRL = D7258_B;
                break;
            case 4:
            case 12:
                D7258_PORT_BSRR = D7258_A | D7258_B |((D7258_C)<<16);
                //D7258_PORT -> BSRRH = D7258_C;
                //D7258_PORT -> BSRRL = D7258_A | D7258_B;
                break;
            case 3:
            case 11:
                D7258_PORT_BSRR = D7258_C | ((D7258_A | D7258_B)<<16);
                //D7258_PORT -> BSRRH = D7258_A | D7258_B;
                //D7258_PORT -> BSRRL = D7258_C;
                break;
            case 2:
            case 10:
                D7258_PORT_BSRR = D7258_A | D7258_C | (D7258_B<<16);
                //D7258_PORT -> BSRRH = D7258_B;
                //D7258_PORT -> BSRRL = D7258_A | D7258_C;
                break;
            case 1:
            case 9:
                D7258_PORT_BSRR = D7258_B | D7258_C | (D7258_A<<16);
                //D7258_PORT -> BSRRH = D7258_A;
                //D7258_PORT -> BSRRL = D7258_B | D7258_C;
                break;
            case 0:
            case 8:
                D7258_PORT_BSRR = D7258_A | D7258_B | D7258_C;
                //D7258_PORT -> BSRRH = 0;
                //D7258_PORT -> BSRRL = D7258_A | D7258_B | D7258_C;
                break;
        }

        //发送1024个脉冲，第1024个脉冲时停止 占时应约为1000/60/16=1.25ms
        for(sck_cnt = 0;sck_cnt < 1025;sck_cnt ++)
        {
            JXI5020_PORT ->BSRRL = JXI5020_SCK;
            delay(2);
            if(sck_cnt < 1025-1)
                JXI5020_PORT -> BSRRH = JXI5020_SCK;
            delay(1);
        }

        for(sck_cnt = 0;sck_cnt < 5;sck_cnt ++)//等待50个脉冲的消隐时间
        {
            JXI5020_PORT ->BSRRH  = UR14_SDO | UG14_SDO | UB14_SDO | UR13_SDO | UG13_SDO | UB13_SDO;
            JXI5020_PORT ->BSRRL = JXI5020_SCK;
            delay(1);
            //JXI5020_PORT -> BSRRH = JXI5020_SCK;
        }
        JXI5020_PORT -> BSRRH = JXI5020_SCK;
    }

    //0复位
#if 1
    JXI5020_PORT -> BSRRL = JXI5020_LE;//LE置为高电平
    for(sck_cnt = 10;sck_cnt > 0;sck_cnt --)
    {
        JXI5020_PORT ->BSRRL = JXI5020_SCK;
        delay(1);
        JXI5020_PORT -> BSRRH = JXI5020_SCK;
        delay(0);
    }
    JXI5020_PORT -> BSRRH = JXI5020_LE;//LE置为低电平
    //for(sck_cnt = 10;sck_cnt > 0;sck_cnt --)
    //{
    //    JXI5020_PORT -> BSRRH = JXI5020_SCK;
    //}
#endif
}
