#ifndef CONNECTER_H
#define CONNECTER_H

#include <QList>
#include <QString>
#include <QTextCodec>

#include <list>

#include <QTime>
#include <QDebug>

#include <winscard.h>

#include "TLVPackage.h"

#define RECV_BUFFER_SIZE 1000

typedef struct tagCustumRecord {
    QString Time;
    QString Money;
    QString Position;
    QString Type;
} CUSTUMRECORD, *PCUSTUMRECORD;

class Connecter
{
public:
    Connecter();
    QString ReaderName;
    int  TestConnect();
    int  StartReadInfo();
    void SetHSC(SCARDCONTEXT hSCOrigin);

private:
    int  SendApdu(BYTE byte1, BYTE byte2, BYTE byte3, BYTE byte4, BYTE byte5);
    void PrintConsumerRecord(BYTE * buffer);
    void PrintTLVInfo(const TLVPackage* CurrentPackage);
    void PrintTLVList(std::list<TLVPackage *> &TLVList);
    void ClearReadStatus();


private:
    SCARDHANDLE  hCardHandle;
    SCARDCONTEXT hSC;
    std::list<TLVPackage *> TLVList;

public:
    QString guy_name;
    QString guy_name_ex;
    QString id_num;
    QString id_type;
    QString card_name;
    QString begin_time;
    QString end_time;
    QString card_type;
    QString card_num;
    std::list<CUSTUMRECORD> RecordList;
};

#endif // CONNECTER_H
