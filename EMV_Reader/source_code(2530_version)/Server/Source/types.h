/***********************************************************************************
  Filename:     types.h

  Description:  type definitions

***********************************************************************************/

#ifndef TYPES_H
#define TYPES_H


#ifdef __cplusplus
extern "C" {
#endif



/***********************************************************************************
 * TYPEDEFS
 */

typedef signed   char   int8;
typedef unsigned char   uint8;

typedef signed   short  int16;
typedef unsigned short  uint16;

typedef signed   long   int32;
typedef unsigned long   uint32;


typedef struct {
  const char* szDescr;      // Textual description
  const uint8  value;        // Value
} menuItem_t;

typedef struct {
  const menuItem_t* pMenuItems;
  const uint8 nItems;
} menu_t;


typedef void (*ISR_FUNC_PTR)(void);
typedef void (*VFPTR)(void);
typedef uint8 (*TimeoutTerminator)( void );
//-----------------------------------------------------------------------------
// Common values
#ifndef FALSE
   #define FALSE 0
#endif

#ifndef TRUE
   #define TRUE 1
#endif

#ifndef NULL
   #define NULL 0
#endif

#ifndef HIGH
   #define HIGH 1
#endif

#ifndef LOW
   #define LOW 0
#endif

#ifndef SUCCESS
#define SUCCESS 0
#endif

#ifndef FAILED
#define FAILED 1
#endif


//-----------------------------------------------------------------------------

#define UPPER_BYTE(a) ((uint8) (((uint16)(a)) >> 8))
#define HIBYTE(a) UPPER_BYTE(a)

#define LOWER_BYTE(a) ((uint8) ( (uint16)(a))      )
#define LOBYTE(a) LOWER_BYTE(a)

#define SET_WORD(regH, regL, word) \
   do{                             \
      (regH) = UPPER_BYTE((word)); \
      (regL) = LOWER_BYTE((word)); \
   }while(0)

#define READ_RFR16(reg) ((((uint16) ##reg##H) << 8) + ##reg##L)
#define WRITE_RFR16(reg, value) do { ##reg##H = HIBYTE(value); ##reg##L = LOBYTE(value); } while (0)

/******************************************************************************
 * CONSTANTS
 */
#define BIT0              0x01
#define BIT1              0x02
#define BIT2              0x04
#define BIT3              0x08
#define BIT4              0x10
#define BIT5              0x20
#define BIT6              0x40
#define BIT7              0x80
#define BIT15			  (1<<15)
#define BIT31			  (1<<31)

// Number of elements in an array
#define N_ITEMS(arr)                sizeof(arr)/sizeof(arr[0])

/***********************************************************************************
* MACROS
*/

#ifndef BV
#define BV(n)      (1 << (n))
#endif

#ifndef BM
#define BM(n)      (1 << (n))
#endif

#ifndef BF
#define BF(x,b,s)  (((x) & (b)) >> (s))
#endif

#ifndef MIN
#define MIN(n,m)   (((n) < (m)) ? (n) : (m))
#endif

#ifndef MAX
#define MAX(n,m)   (((n) < (m)) ? (m) : (n))
#endif

#ifndef ABS
#define ABS(n)     (((n) < 0) ? -(n) : (n))
#endif

/***********************************************************************************
 * Compiler abstraction
 */

/***********************************************************************************
 * IAR 8051
 */
//#if defined __ICC8051__
#define _PRAGMA(x) _Pragma(#x)

#define CODE   __code
#define XDATA  __xdata
#define FAR
//#define NOP()  asm("NOP")

#define HAL_INT_ON(x)      st( EA = 1; )
#define HAL_INT_OFF(x)     st( EA = 0; )
#define HAL_INT_LOCK(x)    st( (x) = EA; EA = 0; )
#define HAL_INT_UNLOCK(x)  st( EA = (x); )


#define HAL_MCU_LITTLE_ENDIAN()   __LITTLE_ENDIAN__
#define HAL_ISR_FUNC_DECLARATION(f,v)   \
    _PRAGMA(vector=v) __near_func __interrupt void f(void)
#define HAL_ISR_FUNC_PROTOTYPE(f,v)     \
    _PRAGMA(vector=v) __near_func __interrupt void f(void)
#define HAL_ISR_FUNCTION(f,v)           \
    HAL_ISR_FUNC_PROTOTYPE(f,v); HAL_ISR_FUNC_DECLARATION(f,v)

#define INTERRUPT_ON(x)      st( EA = 1; )
#define INTERRUPT_OFF(x)     st( EA = 0; )
#define INTERRUPT_LOCK(x)    st( (x) = EA; EA = 0; )
#define INTERRUPT_UNLOCK(x)  st( EA = (x); )

//#else
//#error "Unsupported architecture"
//#endif


#endif

