/******************************************
 �汾: ��żУ���ģ�������������(���Կ���)
*******************************************/
#include "jianPrt.h"
#include "ioCC2530.h"
#include "types.h"
#include "stdarg.h"


#if 1
void PrtTimer3_Init(void)
{                
      T3CTL   = 0x86;
      T3CCTL0 = 0x00;
      T3CC0   = 0x68;
      T3CCTL1 = 0x00;
      T3CC1   = 0x00;
}
#endif

#if 1
void PrtGPIOInit(void)
{
	//P0DIR |= 0x08;//����P0.3Ϊ�����ʽ
	//P0_3 = 1;
	P1DIR |= 0x40;//����P0.3Ϊ�����ʽ
	P1_6 = 1;
}
#endif

#if 0
void ptb(INT8U c)
{
    INT8U i;

    TIM2->CR1 |= BIT0;      //ʹ�ܼ�����TIM2

    TIM2->SR &= ~TIM_FLAG_Update;               //��������жϱ�־  
    GPIOA->ODR |= BIT11;                            //PA11�ø�

    /***** ��ʼλ *****/
    while(!(TIM2->SR &TIM_FLAG_Update));        //�ȴ������жϱ�־
    TIM2->SR &= ~TIM_FLAG_Update;               //��������жϱ�־
    while(!(TIM2->SR &TIM_FLAG_Update));        //�ȴ������жϱ�־
    TIM2->SR &= ~TIM_FLAG_Update;               //��������жϱ�־
    GPIOA->ODR &= ~BIT11;                       //PA11�õ�

    /***** ����λ *****/
    for(i=0;i<8;i++)
    {
        while(!(TIM2->SR &TIM_FLAG_Update));    //�ȴ������жϱ�־
        TIM2->SR &= ~TIM_FLAG_Update;           //������±�־
        while(!(TIM2->SR &TIM_FLAG_Update));    //�ȴ������жϱ�־
        TIM2->SR &= ~TIM_FLAG_Update;           //������±�־
        if(c&0x01)
            GPIOA->ODR |= BIT11;
        else
            GPIOA->ODR &= ~BIT11;
        c = c>>1;
    }

    /***** ��ֹλ *****/
    while(!(TIM2->SR &TIM_FLAG_Update));        //�ȴ������жϱ�־
    TIM2->SR &= ~TIM_FLAG_Update;               //������±�־
    while(!(TIM2->SR &TIM_FLAG_Update));        //�ȴ������жϱ�־
    TIM2->SR &= ~TIM_FLAG_Update;               //������±�־
    GPIOA->ODR |= BIT11;                        //PA11�ø�

    TIM2->CR1 &= ~BIT0;             //�رռ�����TIM2
}
#endif

#if 1

void ptb(INT8U c)
{
    INT8U i;        //����λ����
    //INT8U cnt=0;    //У��λ����
    
    T3CTL |= BIT4;      //ʹ�ܼ�����TIM2

    TXF_OFF;
    TXD_HIGH;

    /***** ��ʼλ *****/
    while(!TXF_ON);
    TXF_OFF;
    while(!TXF_ON);
    TXF_OFF;
    TXD_LOW;

    /***** ����λ *****/
    for(i=0;i<8;i++)
    {
        while(!TXF_ON);
        TXF_OFF;
        while(!TXF_ON);
        TXF_OFF;
        if(c&0x01)
        {
            TXD_HIGH;
            //cnt++;
        }
        else
            TXD_LOW;
        c >>= 1;
    }
 
    /***** У��λ��żУ�� *****/
 /*   while(!TXF_ON);
    TXF_OFF;
    while(!TXF_ON);
    TXF_OFF;
    if((cnt%2)==1)
        TXD_HIGH;
    else
        TXD_LOW;  */


    /***** ��ֹλ *****/
    while(!TXF_ON);
    TXF_OFF;
    while(!TXF_ON);
    TXF_OFF;
    TXD_HIGH;

    //TIM2->CR1 &= ~BIT0;
	T3CTL &= ~BIT4;      //ʹ�ܼ�����TIM3
}
#endif

void ptc(INT8U c)
{
    if(c=='\n')
    ptb('\r');
    ptb(c);
}


void pts(INT8U *s)
{
    while(*s)
    {
        ptc(*s++);
    }
}

#if 0   //�����ַ������żУ��
INT8U RByte(void)
{
    INT8U Output = 0;
    INT8U i = 8;        //����λ����
    INT8U cnt = 0;      //У��λ����

    TIM2->CR1 |= BIT0;          //�򿪶�ʱ��

    while(!TXF_ON);
    TXF_OFF;
    while(!TXF_ON);
    TXF_OFF;
    while(!TXF_ON);
    TXF_OFF;
    
    while(i--)
    {   
        Output >>= 1;     
        if(RXF_ON)
        {
            Output|= 0x80;
            cnt++;
        }
        while(!TXF_ON);
        TXF_OFF;
        while(!TXF_ON);
        TXF_OFF;
    }

    #if 1   //����У��λ����ȷ���ظ�OxAA,���󷵻ظ�0x55
    if(RXF_ON)
        cnt++;
    
    while(!TXF_ON); //����һ��Ҫ�ȴ���������ֹͣλ���������ݴ���ʱ�����ǰһλ��У��Ϊ0�����ܻᱻ������һλ����ʼ
    TXF_OFF;
    while(!TXF_ON);
    TXF_OFF;

    if(cnt%2==0)
    {
        TIM2->CR1 &= ~BIT0;
        return Output;
    }
    else
    {
        TIM2->CR1 &= ~BIT0;
        return 0x55;
    }
    #else
    TIM2->CR1 &= ~BIT0;
    return Output;
    #endif  
}
#endif

#if 0       //������
void prt(INT8U *fmt, ...)
{
    char char_var;  

    INT8U  dec_8U;      /* ʮ����8λ�޷���      */
    INT16U dec_16U;     /* ʮ����16λ�޷���     */
    INT32U dec_32U;     /* ʮ����32λ�޷���     */
    INT8S  dec_8S;      /* ʮ����8λ�з���      */
    INT16S dec_16S;     /* ʮ����16λ�з���     */
    INT32S dec_32S;     /* ʮ����32λ�з���     */

    INT8U  scale_s;
    INT16U scale_m;
    INT32U scale_l;
    INT8U  flg_d;

    INT8U hex_8U;
    INT16U hex_16U;
    INT32U hex_32U;

    INT8U select_s;
    INT16U select_m;
    INT32U select_l;
    
    INT8U move_times_s;
    INT8U move_times_m;
    INT8U move_times_l;
    
    va_list argp;
    va_start(argp,fmt);

    while(*fmt)
    {
        if(*fmt!='%')
        {
            ptc(*fmt);
            fmt++;
            continue;
        }
        
        switch(*(++fmt))
        {
            /* %c���ַ��� */
            case 'c':
                char_var = va_arg(argp,INT32U);
                ptc(char_var);
                fmt++;
            break;

            #if 1   /* %d�����ͣ�ʮ���� */
            /* %dss��ʮ����8λ�з��ţ� %dsu��ʮ����8λ�޷���  */
            /* %dms��ʮ����16λ�з��ţ�%dsu��ʮ����16λ�޷��� */
            /* %dls��ʮ����32λ�з��ţ�%dsu��ʮ����32λ�޷��� */
            case 'd':           //d ���� s,m,l 8λ 16λ 32λ  s,u�з��ţ��޷���
                switch(*(++fmt))
                {
                    case 's':
                        scale_s = 100;
                        flg_d = 0;
                        if(*(++fmt)=='s')
                        {
                            dec_8S = va_arg(argp,INT32S);
                            if(dec_8S & BIT7)
                                ptc('-');
                            dec_8U = -dec_8S;
                        }
                        else
                            dec_8U = va_arg(argp,INT32U);

                        if(!dec_8U)
                        {
                            ptc('0');
                            break;
                        }

                        while(scale_s)
                        {
                            if((dec_8U/scale_s)||flg_d)
                            {
                                char_var = dec_8U/scale_s + '0';
                                ptc(char_var);
                                dec_8U %= scale_s;
                                flg_d = 1;
                            }
                            scale_s/=10;
                        }
                    break;
                    
                    case 'm':
                        scale_m = 10000;
                        flg_d = 0;
                        if(*(++fmt)=='s')
                        {
                            dec_16S = va_arg(argp,INT32S);
                            if(dec_16S&BIT15)
                                ptc('-');
                            dec_16U = -dec_16S;
                        }
                        else
                            dec_16U = va_arg(argp,INT32U);

                        if(!dec_16U)
                        {
                            ptc('0');
                            break;
                        }
                        
                        while(scale_m)
                        {
                            if(dec_16U/scale_m||flg_d)
                            {
                                char_var = dec_16U/scale_m + '0';
                                ptc(char_var);
                                dec_16U %= scale_m;
                                flg_d =1;
                            }
                            scale_m/=10;
                        }                   
                    break;

                    case 'l':
                        scale_l = 1000000000;
                        flg_d = 0;
                        if(*(++fmt)=='s')
                        {
                            dec_32S = va_arg(argp,INT32S);
                            if(dec_32S&BIT31)
                                ptc('-');
                            dec_32U = -dec_32S;
                        }
                        else
                            dec_32U = va_arg(argp,INT32U);

                        while(scale_l)
                        {
                            if(dec_32U/scale_l||flg_d)
                            {
                                char_var = dec_32U/scale_l + '0';
                                ptc(char_var);
                                dec_32U %= scale_l;
                                flg_d =1;
                            }
                            scale_l/=10;
                        }
                    break;
                    
                    default:
                    break;
                    
                }   
            fmt++;  
            break;
            #endif
            
            #if 1  /* %h��ʮ������ */
            /* %hs��ʮ������8λ      */
            /* %hm��ʮ������16λ     */
            /* %hl��ʮ������32λ     */
            case 'h':
                switch(*(++fmt))
                {
                    case 's':
                        ptc('0');
                        ptc('x');
                        select_s = 0xF0;
                        move_times_s = 2;
                        hex_8U = va_arg(argp,INT32U);
                        while(move_times_s--)
                        {
                            char_var = (INT8U)((hex_8U&select_s)>>4) + '0';
                            if(char_var<='9')
                                ptc(char_var);
                            else
                            {
                                char_var+=7;
                                ptc(char_var);
                            }
                            hex_8U<<=4;
                        }
                    break;
                    
                    case 'm':
                        ptc('0');
                        ptc('x');
                        select_m = 0xF000;
                        move_times_m = 4;
                        hex_16U = va_arg(argp,INT32U);
                        while(move_times_m--)
                        {
                            char_var = (INT8U)((hex_16U&select_m)>>4*3) + '0';
                            if(char_var<='9')
                                ptc(char_var);
                            else
                            {
                                char_var+=7;
                                ptc(char_var);
                            }
                            hex_16U<<=4;
                        }

                        break;

                    case 'l':
                        ptc('0');
                        ptc('x');
                        select_l = 0xF0000000;
                        move_times_l = 8;
                        hex_32U = va_arg(argp,INT32U);
                        while(move_times_l--)
                        {
                            char_var = (INT8U)((hex_32U&select_l)>>4*7) + '0';
                            if(char_var<='9')
                                ptc(char_var);
                            else
                            {
                                char_var+=7;
                                ptc(char_var);
                            }
                            hex_32U<<=4;
                        }

                        break;
                    default:
                        break;

                }
            fmt++;
            break;
            #endif
            
            default:
            break;
        }
    }
    va_end(argp);
}
#endif

#if 1
void prt(INT8U *fmt, ...)
{
    char char_var;
    
    INT8U dec;
    va_list argp;
    va_start(argp,fmt);
    
    while(*fmt)
    {
        if(*fmt!='%')
        {
            ptc(*fmt);
            fmt++;
            continue;
        }
        switch(*(++fmt))
        {
            case 'c':
            char_var = va_arg(argp,INT32U);
            ptc(char_var);
            fmt++;
            break;

            case 'd':
            dec = va_arg(argp,INT32U);
            if(dec<10)
            {
                char_var = dec + '0';
                ptc(char_var);
            }
            else if(dec>9 && dec<100)
            {
                char_var = dec/10 + '0';
                ptc(char_var);
                char_var = dec%10 + '0';
                ptc(char_var);
            }
            else if(dec>99)
            {
                char_var = dec/100 + '0';
                ptc(char_var);
                char_var = dec%100/10 +'0';
                ptc(char_var);
                char_var = dec%10 + '0';
                ptc(char_var);
            }

            fmt++;
            break;

            default:
            break;
        }
    }
    va_end(argp);   
}
#endif




