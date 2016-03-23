/**************************************************************************************************
 * @fn      HalSpiInit
 *
 * @brief   Initialize SPI Service
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalSpiInit(void)
{
#if 0
    //spi口
    PERCFG |= 0x02;	//设置USART1的IO在位置2，即P1口
    P1SEL  |= 0xF0;	//选择P1.7—P1.4为SPI引脚,设置为外围设备功能

    U1CSR &= ~0x80;     //bit7:0->选择SPI模式,1->uart模式
    U1CSR |= 0x20;      //bit5:0->SPI主模式，1->SPI从模式

    U1GCR |= BV(5);//bit order to transfers, MSB first

    U1GCR &= ~0xC0;//CPOL=0,CPHA=0,低电平无效，上升沿采样

    P2SEL &= ~0x20;  //bit5设置为0，Give UART1 priority over Timer3.

    dmaInit();    //DMA初始化

    HAL_DMA_ARM_CH(HAL_DMA_CH_RX);
#endif

		CLKCONCMD = 0x80; while (CLKCONSTA != 0x80);        // 32MHz
        // SPI Master Mode
        PERCFG |= 0x02;        // map USART1 to its alternative 2 location. P1_4: SSN, P1_5: SCK, P1_6: MOSI, P1_7: MISO
        P1SEL |= 0xE0;        // P1_5, P1_6, and P1_7 are peripherals
        P1SEL &= ~0x10;        // P1_4 is GPIO (SSN)
        P1DIR |= 0x10;        // SSN is set as output

        U1BAUD = 0x00; U1GCR |= 0x11;        // Set baud rate to max (system clock frequency / 8)
        U1CSR &= ~0xA0;        // SPI Master Mode
//        U1CSR &= ~0x80; U1CSR |= 0x20;        // SPI Slave Mode
        U1GCR &= ~0xC0; U1GCR |= 0x20;        // MSB

        for(;;) {
                P1_4 = 0;        // SSN LOW
                U1DBUF = 0x55; while (!(U1CSR&0x02)); U1CSR &= 0xFD;
                U1DBUF = 0x00; while (!(U1CSR&0x02)); U1CSR &= 0xFD;
                unsigned char temp = U1DBUF;
                P1_4 = 1;        // SSN high
        }


}

