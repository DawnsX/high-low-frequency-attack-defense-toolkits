/******************************************
 版本: 带偶校验的模拟输入输出串口(调试可用)
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
	//P0DIR |= 0x08;//设置P0.3为输出方式
	//P0_3 = 1;
	P1DIR |= 0x40;//设置P0.3为输出方式
	P1_6 = 1;
}
#endif

#if 0
void ptb(INT8U c)
{
    INT8U i;

    TIM2->CR1 |= BIT0;      //使能计数器TIM2

    TIM2->SR &= ~TIM_FLAG_Update;               //清除更新中断标志  
    GPIOA->ODR |= BIT11;                            //PA11置高

    /***** 起始位 *****/
    while(!(TIM2->SR &TIM_FLAG_Update));        //等待更新中断标志
    TIM2->SR &= ~TIM_FLAG_Update;               //清除更新中断标志
    while(!(TIM2->SR &TIM_FLAG_Update));        //等待更新中断标志
    TIM2->SR &= ~TIM_FLAG_Update;               //清除更新中断标志
    GPIOA->ODR &= ~BIT11;                       //PA11置低

    /***** 数据位 *****/
    for(i=0;i<8;i++)
    {
        while(!(TIM2->SR &TIM_FLAG_Update));    //等待更新中断标志
        TIM2->SR &= ~TIM_FLAG_Update;           //清除更新标志
        while(!(TIM2->SR &TIM_FLAG_Update));    //等待更新中断标志
        TIM2->SR &= ~TIM_FLAG_Update;           //清除更新标志
        if(c&0x01)
            GPIOA->ODR |= BIT11;
        else
            GPIOA->ODR &= ~BIT11;
        c = c>>1;
    }

    /***** 终止位 *****/
    while(!(TIM2->SR &TIM_FLAG_Update));        //等待更新中断标志
    TIM2->SR &= ~TIM_FLAG_Update;               //清除更新标志
    while(!(TIM2->SR &TIM_FLAG_Update));        //等待更新中断标志
    TIM2->SR &= ~TIM_FLAG_Update;               //清除更新标志
    GPIOA->ODR |= BIT11;                        //PA11置高

    TIM2->CR1 &= ~BIT0;             //关闭计数器TIM2
}
#endif

#if 1

void ptb(INT8U c)
{
    INT8U i;        //数据位个数
    //INT8U cnt=0;    //校验位计数
    
    T3CTL |= BIT4;      //使能计数器TIM2

    TXF_OFF;
    TXD_HIGH;

    /***** 起始位 *****/
    while(!TXF_ON);
    TXF_OFF;
    while(!TXF_ON);
    TXF_OFF;
    TXD_LOW;

    /***** 数据位 *****/
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
 
    /***** 校验位，偶校验 *****/
 /*   while(!TXF_ON);
    TXF_OFF;
    while(!TXF_ON);
    TXF_OFF;
    if((cnt%2)==1)
        TXD_HIGH;
    else
        TXD_LOW;  */


    /***** 终止位 *****/
    while(!TXF_ON);
    TXF_OFF;
    while(!TXF_ON);
    TXF_OFF;
    TXD_HIGH;

    //TIM2->CR1 &= ~BIT0;
	T3CTL &= ~BIT4;      //使能计数器TIM3
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

#if 0   //接收字符，添加偶校验
INT8U RByte(void)
{
    INT8U Output = 0;
    INT8U i = 8;        //数据位个数
    INT8U cnt = 0;      //校验位计数

    TIM2->CR1 |= BIT0;          //打开定时器

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

    #if 1   //接收校验位，正确返回个OxAA,错误返回个0x55
    if(RXF_ON)
        cnt++;
    
    while(!TXF_ON); //这里一定要等待，是它到停止位，否则传数据串的时候如果前一位的校验为0，可能会被当做后一位的起始
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

#if 0       //待完善
void prt(INT8U *fmt, ...)
{
    char char_var;  

    INT8U  dec_8U;      /* 十进制8位无符号      */
    INT16U dec_16U;     /* 十进制16位无符号     */
    INT32U dec_32U;     /* 十进制32位无符号     */
    INT8S  dec_8S;      /* 十进制8位有符号      */
    INT16S dec_16S;     /* 十进制16位有符号     */
    INT32S dec_32S;     /* 十进制32位有符号     */

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
            /* %c，字符型 */
            case 'c':
                char_var = va_arg(argp,INT32U);
                ptc(char_var);
                fmt++;
            break;

            #if 1   /* %d，整型，十进制 */
            /* %dss，十进制8位有符号， %dsu，十进制8位无符号  */
            /* %dms，十进制16位有符号，%dsu，十进制16位无符号 */
            /* %dls，十进制32位有符号，%dsu，十进制32位无符号 */
            case 'd':           //d 整型 s,m,l 8位 16位 32位  s,u有符号，无符号
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
            
            #if 1  /* %h，十六进制 */
            /* %hs，十六进制8位      */
            /* %hm，十六进制16位     */
            /* %hl，十六进制32位     */
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




