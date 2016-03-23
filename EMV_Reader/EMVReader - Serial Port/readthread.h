#ifndef READTHREAD_H
#define READTHREAD_H

#include <QThread>
#include <QStringList>

#include <QtSerialPort/QSerialPort>

#include <QDebug>
#include <QTime>

#include "connecter.h"

#define STATUS_LEVEL_NORMAL  0
#define STATUS_LEVEL_WARNING 1

class ReadThread : public QThread
{
    Q_OBJECT
public:
    explicit ReadThread(QObject *parent = 0);
    QSerialPort *serial;
    //QString  Reader;
    //SCARDCONTEXT hSC;

signals:
    void ClearText();
    void AddText(QString str);
    void AddInfoList(QStringList list);
    void ShowStatus(QString status, int Level);
    void PrintCardNum(QString cardnum);
    void PrintCardType(QString cardType);
    void PrintCardName(QString cardName);
    void PrintGuyName(QString guyName);
    void PrintGuyNameEx(QString guyNameEx);
    void PrintIdNum(QString idNum);
    void PrintIdType(QString idType);
    void PrintValidity(QString from, QString to);

private:
    void PrintAllInfo(Connecter *card);

public slots:

protected:
    void run();
};

#endif // READTHREAD_H
