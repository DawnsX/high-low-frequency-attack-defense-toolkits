/******************************************************************************
    Filename: Key.h

******************************************************************************/
#ifndef KEY_H
#define KEY_H


#include "types.h"

enum Key_status{STKEY_IDLE,STKEY_DETDN,STKEY_CNT,STKEY_DETUP};

typedef struct {
	uint8 Hour;
	uint8 Minute;
	uint8 Second;
	uint16 Millisecond;
} TimeCounter;
void KeyInit(void);
void checkKey(void);
void OpenRf(void);
void SendConfig( void);
#endif