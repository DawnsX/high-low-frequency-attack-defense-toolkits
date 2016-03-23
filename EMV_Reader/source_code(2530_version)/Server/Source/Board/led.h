/******************************************************************************
    Filename: led.h

******************************************************************************/

#ifndef LED_H
#define LED_H
#include "Types.h"
/***********************************************************************************
 * GLOBAL FUNCTIONS
 */
void LedTestInit(void);	

void LedInit(void);

void halLedSet(uint8);
void halLedClear(uint8);
void halLedToggle(uint8 );

void halLedToggle_1(void);
void halLedToggle_2(void);
void halLedToggle_3(void);
void halLedToggle_4(void);

void GetRandomByte_test(void);
void LedChange(void);


/******************************************************************************
 * MACROS
 */

#define MCU_IO_TRISTATE   1             // Used as "func" for the macros below
#define MCU_IO_PULLUP     2
#define MCU_IO_PULLDOWN   3


//-----------------------------------------------------------------------------
//  Macros for simple configuration of IO pins on TI LPW SoCs
//-----------------------------------------------------------------------------
#define MCU_IO_PERIPHERAL(port, pin)   MCU_IO_PERIPHERAL_PREP(port, pin)
#define MCU_IO_INPUT(port, pin, func)  MCU_IO_INPUT_PREP(port, pin, func)
#define MCU_IO_OUTPUT(port, pin, val)  MCU_IO_OUTPUT_PREP(port, pin, val)
#define MCU_IO_SET(port, pin, val)     MCU_IO_SET_PREP(port, pin, val)
#define MCU_IO_SET_HIGH(port, pin)     MCU_IO_SET_HIGH_PREP(port, pin)
#define MCU_IO_SET_LOW(port, pin)      MCU_IO_SET_LOW_PREP(port, pin)
#define MCU_IO_TGL(port, pin)          MCU_IO_TGL_PREP(port, pin)
#define MCU_IO_GET(port, pin)          MCU_IO_GET_PREP(port, pin)


//----------------------------------------------------------------------------------
//  Macros for internal use (the macros above need a new round in the preprocessor)
//----------------------------------------------------------------------------------
#define MCU_IO_PERIPHERAL_PREP(port, pin)  st( P##port##SEL |= BM(pin); )

#define MCU_IO_INPUT_PREP(port, pin, func) st( P##port##SEL &= ~BM(pin); \
                                               P##port##DIR &= ~BM(pin); \
                                               switch (func) { \
                                               case MCU_IO_PULLUP: \
                                                   P##port##INP &= ~BM(pin); \
                                                   P2INP &= ~BM(port + 5); \
                                                   break; \
                                               case MCU_IO_PULLDOWN: \
                                                   P##port##INP &= ~BM(pin); \
                                                   P2INP |= BM(port + 5); \
                                                   break; \
                                               default: \
                                                   P##port##INP |= BM(pin); \
                                                   break; } )

#define MCU_IO_OUTPUT_PREP(port, pin, val) st( P##port##SEL &= ~BM(pin); \
                                               P##port##_##pin## = val; \
                                               P##port##DIR |= BM(pin); )

#define MCU_IO_SET_HIGH_PREP(port, pin)     st( P##port##_##pin## = 1; )
#define MCU_IO_SET_LOW_PREP(port, pin)      st( P##port##_##pin## = 0; )

#define MCU_IO_SET_PREP(port, pin, val)    st( P##port##_##pin## = val; )
#define MCU_IO_TGL_PREP(port, pin)         st( P##port##_##pin## ^= 1; )
#define MCU_IO_GET_PREP(port, pin)         (P##port## & BM(pin))


#endif
