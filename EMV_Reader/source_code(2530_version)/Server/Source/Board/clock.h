/******************************************************************************
    Filename: clock.h

    This file defines interface for clock related functions for the
    CC243x family of RF system-on-chips from Texas Instruments.

******************************************************************************/
#ifndef _CLOCK_H
#define _CLOCK_H


/*******************************************************************************
 * INCLUDES
 */

#include "types.h"
#include "ioCC2530.h"




/*******************************************************************************
 * CONSTANTS
 */

/* SEE DATA SHEET FOR DETAILS ABOUT THE FOLLOWING BIT MASKS */

// Parameters for cc2530ClockSetMainSrc
#define CLOCK_SRC_XOSC      0  // High speed Crystal Oscillator Control
#define CLOCK_SRC_HFRC      1  // Low power RC Oscillator

// Parameters for cc2530ClockSelect32k
#define CLOCK_32K_XTAL      0  // 32.768 Hz crystal oscillator
#define CLOCK_32K_RCOSC     1  // 32 kHz RC oscillator

// Bit masks to check CLKCON register
#define CLKCON_OSC32K_BM    0x80  // bit mask, for the slow 32k clock oscillator
#define CLKCON_OSC_BM       0x40  // bit mask, for the system clock oscillator
#define CLKCON_TICKSPD_BM   0x38  // bit mask, for timer ticks output setting
#define CLKCON_CLKSPD_BM    0x01  // bit maks, for the clock speed

#define TICKSPD_DIV_1       (0x00 << 3)
#define TICKSPD_DIV_2       (0x01 << 3)
#define TICKSPD_DIV_4       (0x02 << 3)
#define TICKSPD_DIV_8       (0x03 << 3)
#define TICKSPD_DIV_16      (0x04 << 3)
#define TICKSPD_DIV_32      (0x05 << 3)
#define TICKSPD_DIV_64      (0x06 << 3)
#define TICKSPD_DIV_128     (0x07 << 3)

// Bit masks to check SLEEPSTA register
#define SLEEP_XOSC_STB_BM   0x40  // bit mask, check the stability of XOSC
#define SLEEP_HFRC_STB_BM   0x20  // bit maks, check the stability of the High-frequency RC oscillator
#define SLEEP_OSC_PD_BM     0x04  // bit mask, power down system clock oscillator(s)


/*******************************************************************************
 * MACROS
 */

// Macro for checking status of the high frequency RC oscillator.
#define CC2530_IS_HFRC_STABLE(x)    (SLEEPSTA & SLEEP_HFRC_STB_BM)

// Macro for checking status of the crystal oscillator
#define CC2530_IS_XOSC_STABLE(x)    (SLEEPSTA & SLEEP_XOSC_STB_BM)

// Macro for getting the clock division factor
#define CC2530_GET_CLKSPD(x)        (CLKCONSTA & CLKCON_CLKSPD_BM)


// Macro for getting the timer tick division factor.
#define CC2530_GET_TICKSPD(x)       ((CLKCONSTA & CLKCON_TICKSPD_BM) >> 3)

// Macro for setting the clock division factor, x value from 0b000 to 0b111
#define CC2530_SET_TICKSPD(x)        st( CLKCONCMD = ((((x) << 3) & 0x38)  \
                                                    | (CLKCONCMD & 0xC7)); \
		)

// Macro for setting the timer tick division factor, x value from 0b000 to 0b111
#define CC2530_SET_CLKSPD(x)         st( CLKCONCMD = (((x) & 0x07)         \
                                                    | (CLKCONCMD & 0xF8)); \
									 )
	// Macro for waiting for clock updates
#define CC2530_WAIT_CLK_UPDATE()    st( uint8 ____clkcon; \
											uint8 ____clkconsta; \
											____clkcon = CLKCONCMD; \
											do { \
											  ____clkconsta = CLKCONSTA; \
											} while (____clkconsta != ____clkcon); )


/******************************************************************************
*******************      Power and clock management        ********************
*******************************************************************************

These macros are used to set power-mode, clock source and clock speed.

******************************************************************************/

// Macro for getting the clock division factor
//#define CLKSPD  (CLKCON & 0x07)
#define CLKSPD  (CLKCON & 0x01)

// Macro for getting the timer tick division factor.
#define TICKSPD ((CLKCON & 0x38) >> 3)

// Macro for checking status of the crystal oscillator
#define XOSC_STABLE (SLEEP & 0x40)

// Macro for checking status of the high frequency RC oscillator.
#define HIGH_FREQUENCY_RC_OSC_STABLE    (SLEEP & 0x20)


// Macro for setting power mode
#define SET_POWER_MODE(mode)                   \
   do {                                        \
      if(mode == 0)        { SLEEPCMD &= ~0x03; } \
      else if (mode == 3)  { SLEEPCMD |= 0x03;  } \
      else { SLEEPCMD &= ~0x03; SLEEPCMD |= mode;  } \
      PCON |= 0x01;                            \
      asm("NOP");                              \
   }while (0)


// Where _mode_ is one of
#define POWER_MODE_0  0x00  // Clock oscillators on, voltage regulator on
#define POWER_MODE_1  0x01  // 32.768 KHz oscillator on, voltage regulator on
#define POWER_MODE_2  0x02  // 32.768 KHz oscillator on, voltage regulator off
#define POWER_MODE_3  0x03  // All clock oscillators off, voltage regulator off

// Macro for setting the 32 KHz clock source
#define SET_32KHZ_CLOCK_SOURCE(source) \
   do {                                \
      if( source ) {                   \
         CLKCON |= 0x80;               \
      } else {                         \
         CLKCON &= ~0x80;              \
      }                                \
   } while (0)

// Where _source_ is one of
#define CRYSTAL 0x00
#define RC      0x01

// Macro for setting the main clock oscillator source,
//turns off the clock source not used
//changing to XOSC will take approx 150 us
#define SET_MAIN_CLOCK_SOURCE(source) \
   do {                               \
      if(source) {                    \
        CLKCON |= 0x40;               \
        while(!HIGH_FREQUENCY_RC_OSC_STABLE); \
        if(TICKSPD == 0){             \
          CLKCON |= 0x08;             \
        }                             \
        SLEEP |= 0x04;                \
      }                               \
      else {                          \
        SLEEP &= ~0x04;               \
        while(!XOSC_STABLE);          \
        asm("NOP");                   \
        CLKCON &= ~0x47;              \
        SLEEP |= 0x04;                \
      }                               \
   }while (0)

/*******************************************************************************
 * FUNCTIONS
 */

void clockSetMainSrc(uint8 source);
void clockSelect32k(uint8 source);


/***********************************************************************************
 
***********************************************************************************/

#endif
