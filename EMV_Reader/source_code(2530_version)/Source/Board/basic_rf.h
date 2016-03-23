/*
* Copyright (c) 2015,Yuanjian
* All rights reserved.
*
* �ļ����ƣ�basic_rf.h
* �ļ���ʶ��
* ժ Ҫ��
*
* ��ǰ�汾��0.0
* �� �ߣ�   Ԭ��
* ������ڣ�2015��7��17��
*
* ȡ���汾��
* ԭ����:
* ������ڣ�
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
#ifndef BASIC_RF_H
#define BASIC_RF_H


/***********************************************************************************
* INCLUDES
*/
#include "types.h"


/***********************************************************************************
* CONSTANTS AND DEFINES
*/

#define BASIC_RF_PACKET_SIZE        125
#define RF_RX_FCS_SIZE              2
#define RF_MAX_RX_SIZE				127
// The length byte
#define PHY_PACKET_SIZE_MASK        0x7F


#define BEGIN     					0x00
#define END							0x01

#define CONNECTED                   0x00
#define DISCONNECTED				0x01
/***********************************************************************************
* TYPEDEFS
*/






/***********************************************************************************
* GLOBAL FUNCTIONS
*/
void basicRfInit(void);
void RF_RxModeOn(void);
void RF_RxModeOff(void);
void SetRFPwr(uint8 );
void ReceivedFrameDecode(void);
//void Send( uint8 cmd);
void SendData(uint8* );


void SendHeartBeat(void);
//void SendOverFlag(void);

void ConnectStatusDisconnect(void);
uint8 ReturnConnectStatus(void);
uint8 * BuildHead(uint8 * data,uint8 *txBuff,uint8 gId,uint8 id,uint8 partAmount);
//void StoreData(uint8 storeBuff[][128],uint8* inputBuff)��
void StoreData(uint8 storeBuff[][128],uint8 *inputBuff);
void SendConfirm(void);

//	uint8 ItemAmount(uint8 buffer[][128]);




/***********************************************************************************

***********************************************************************************/

#endif

