/*
 * test.c
 *
 *  Created on: 2019��4��22��
 *      Author: OP056666
 */


void dispaly_process(void) //��7us
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

  ������ɨ��Ĵ��룺
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

    //1д��״̬�Ĵ���
    //aд״̬�Ĵ���֮ǰ��ǰ���趨ָ��
    JXI5020_PORT -> BSRRL = JXI5020_LE;//LE��Ϊ�ߵ�ƽ
    for(sck_cnt = 14;sck_cnt > 0;sck_cnt --)
    {
            JXI5020_PORT ->BSRRL = JXI5020_SCK;
            delay(2);
            JXI5020_PORT -> BSRRH = JXI5020_SCK;
            delay(1);
    }
    JXI5020_PORT -> BSRRH = JXI5020_LE;//LE��Ϊ�͵�ƽ
    //bд״̬�Ĵ���
    for(j = 0;j < 4;j++)//ÿ��ͨ����4ƬMBI5052������ÿ��ͨ������һ������
    {
        for(sck_cnt = 0;sck_cnt < 16;sck_cnt ++)
        {
            mask = 0x8000 >> sck_cnt;

            if(j == 3)
                if(sck_cnt == 12)
                    JXI5020_PORT -> BSRRL = JXI5020_LE;//LE��Ϊ�ߵ�ƽ
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
    JXI5020_PORT -> BSRRH = JXI5020_LE;//LE��Ϊ�͵�ƽ

    //2д��16*2*16������
    for(m = 0;m < 16;m++)//����16��
    {
        for(i = 0;i < 16;i++)//ÿ��MBI5052��16��ͨ����
        //for(i = 0;i < 1;i++)
        {
            for(j = 0;j < 4;j++)//ÿ��ͨ����4ƬMBI5052������ÿ��ͨ������һ������
            //for(j = 0;j < 1;j++)
            {
#if 0
                bufaddA = 0x000000ff;
                bufaddB = 0x0000ff00;;//8λԭʼ���ݶ�Ӧ��16λPWM����
#else
                bufaddA = leddisbuf[((m+1)*64-1) - j*16 - i];
                bufaddB = leddisbuf[((m+1)*64-1) - j*16 - i + 64*16];//8λԭʼ���ݶ�Ӧ��16λPWM����
#endif
#if 0
                if((m == 1) && (i == 0) && (j == 0))//m == 4 //���ڲ���������ʾ��λ��
                {
                        red1 =   (bufaddA & 0xff0000) >> 8;//2 * (leddisbuf[i*64 + j] & 0xff0000>>16)�ϰ�������
                        green1 = (bufaddA & 0x00ff00) >> 0;//2 * (leddisbuf[i*64 + j] & 0x00ff00>>8)�ϰ�������
                        blue1 =  (bufaddA & 0x0000ff) << 8;//�ϰ�������
                        red2 =   (bufaddB & 0xff0000) >> 8;//�ϰ�������
                        green2 = (bufaddB & 0x00ff00) >> 0;//�ϰ�������
                        blue2 =  (bufaddB & 0x0000ff) << 8;//�ϰ�������
                }
                else
                {
                        red1 =   0x0000;
                        green1 = 0x0000;
                        blue1 =  0x0000;//(leddisbuf[bufaddA] & 0x0000ff) << 1;//�ϰ�������
                        red2 =   0x0000;//(leddisbuf[bufaddB] & 0xff0000) >> 15;//�ϰ�������
                        green2 = 0x0000;//(leddisbuf[bufaddB] & 0x00ff00) >> 7;//�ϰ�������
                        blue2 =  0x0000;//(leddisbuf[bufaddB] & 0x0000ff) << 1;//�ϰ�������
                }
#else
                red1 =   (bufaddA & 0xff0000) >> 8;//2 * (leddisbuf[i*64 + j] & 0xff0000>>16)�ϰ�������
                green1 = (bufaddA & 0x00ff00) >> 0;//2 * (leddisbuf[i*64 + j] & 0x00ff00>>8)�ϰ�������
                blue1 =  (bufaddA & 0x0000ff) << 8;//�ϰ�������
                red2 =   (bufaddB & 0xff0000) >> 8;//�ϰ�������
                green2 = (bufaddB & 0x00ff00) >> 0;//�ϰ�������
                blue2 =  (bufaddB & 0x0000ff) << 8;//�ϰ�������
#endif

                //if(m == 0)red1 = 255;
                for(k = 0;k < 16;k++)
                {
                    mask = 0x8000 >> k;
                    if(j == 3)
                        if(k == 15)
                            JXI5020_PORT -> BSRRL = JXI5020_LE;//���һ��λLE��Ϊ�ߵ�ƽ
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
            JXI5020_PORT -> BSRRH = JXI5020_LE;//LE��Ϊ�͵�ƽ
        }
    }

    //3��������50��GCLK ���3��CLK����LE������VSYNC
    for(sck_cnt = 0;sck_cnt < 60;sck_cnt ++)
    {
        if(sck_cnt == 60 - 3)
            JXI5020_PORT -> BSRRL = JXI5020_LE;//LE��Ϊ�ߵ�ƽ

        //if(state_reg & mask)JXI5020_PORT ->BSRRL  = UR14_SDO | UG14_SDO | UB14_SDO | UR13_SDO | UG13_SDO | UB13_SDO;
        JXI5020_PORT ->BSRRH  = UR14_SDO | UG14_SDO | UB14_SDO | UR13_SDO | UG13_SDO | UB13_SDO;
        delay(1);
        JXI5020_PORT ->BSRRL = JXI5020_SCK;
        delay(1);
        if(sck_cnt < 59)//���һ����Ҫ����
            JXI5020_PORT -> BSRRH = JXI5020_SCK;//����SCK
    }
    JXI5020_PORT -> BSRRH = JXI5020_LE;//LE��Ϊ�͵�ƽ
    delay(2);//LE�½�����GCLK����������Ҫ��
    //4����
    for(sck_cnt = 0;sck_cnt < 512;sck_cnt ++)
    {
        JXI5020_PORT ->BSRRH  = UR14_SDO | UG14_SDO | UB14_SDO | UR13_SDO | UG13_SDO | UB13_SDO;
        JXI5020_PORT ->BSRRL = JXI5020_SCK;
        delay(1);
        //JXI5020_PORT -> BSRRH = JXI5020_SCK;
    }
    JXI5020_PORT -> BSRRH = JXI5020_SCK;//SCK��Ϊ�͵�ƽ

    //5��ʼ��ʾ
    for(i = 0;i < 16; i++)
    {
        //�����е�ƽ
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

        //����1024�����壬��1024������ʱֹͣ ռʱӦԼΪ1000/60/16=1.25ms
        for(sck_cnt = 0;sck_cnt < 1025;sck_cnt ++)
        {
            JXI5020_PORT ->BSRRL = JXI5020_SCK;
            delay(2);
            if(sck_cnt < 1025-1)
                JXI5020_PORT -> BSRRH = JXI5020_SCK;
            delay(1);
        }

        for(sck_cnt = 0;sck_cnt < 5;sck_cnt ++)//�ȴ�50�����������ʱ��
        {
            JXI5020_PORT ->BSRRH  = UR14_SDO | UG14_SDO | UB14_SDO | UR13_SDO | UG13_SDO | UB13_SDO;
            JXI5020_PORT ->BSRRL = JXI5020_SCK;
            delay(1);
            //JXI5020_PORT -> BSRRH = JXI5020_SCK;
        }
        JXI5020_PORT -> BSRRH = JXI5020_SCK;
    }

    //0��λ
#if 1
    JXI5020_PORT -> BSRRL = JXI5020_LE;//LE��Ϊ�ߵ�ƽ
    for(sck_cnt = 10;sck_cnt > 0;sck_cnt --)
    {
        JXI5020_PORT ->BSRRL = JXI5020_SCK;
        delay(1);
        JXI5020_PORT -> BSRRH = JXI5020_SCK;
        delay(0);
    }
    JXI5020_PORT -> BSRRH = JXI5020_LE;//LE��Ϊ�͵�ƽ
    //for(sck_cnt = 10;sck_cnt > 0;sck_cnt --)
    //{
    //    JXI5020_PORT -> BSRRH = JXI5020_SCK;
    //}
#endif
}
