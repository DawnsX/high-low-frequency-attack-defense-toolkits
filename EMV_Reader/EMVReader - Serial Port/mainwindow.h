#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QTimer>
#include <QByteArray>
#include <QSplitter>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QMainWindow>
#include <QPlainTextEdit>
#include <QScrollBar>
#include <QComboBox>
#include <QPushButton>
#include <QLabel>
#include <QPalette>
#include <QStringList>
#include <QTableWidget>
#include <QTableView>
#include <QStringList>
#include <QList>
#include <QFile>
#include <QDir>
#include <QProcess>
#include <QCryptographicHash>
#include <QtSerialPort/QSerialPort>

#include <map>
#include <string>

#include <winscard.h>

#include "TLVPackage.h"
#include "readthread.h"

#define RECV_BUFFER_SIZE 1000

QT_BEGIN_NAMESPACE

namespace Ui {
class MainWindow;
}

typedef struct tagInfoLabel {
    QWidget *Whole;
    QLabel *LabelTime;
    QLabel *LabelMoney;
    QLabel *LabelPos;
    QLabel *LabelType;
} INFOLABEL, *PINFOLABEL;

QT_END_NAMESPACE

//class Connect;
class ReadThread;
class About;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    void initActionsConnections();
    bool fillReaderInfo();
    int  ListReader(std::map<int, std::wstring> &Readers);
    int  ConnectReader(LPCTSTR Name);
    void initInfoAera();
    void initStatusBar();

    void ProcessData();
    void StartReadingInfo();
    void StartReadingNameEtc();
    void ProcessRealData(QByteArray RealData);
    void StartReadingConsumerRecord();

    void InitHashMap(void);
    void InitLogMode(void);
    void WriteLog(void);


private slots:
    void apply();
    void Timeout();
    //void textChanged();
    void ClearText();
    void AddText(QString str);
    void AddInfoList(QStringList list);
    void ChangePrivacyStatus();
    void changeLogMode();
    void ShowStatus(QString status, int Level);
    void PrintCardNum(QString cardnum);
    void PrintCardType(QString cardType);
    void PrintCardName(QString cardName);
    void PrintGuyName(QString guyName);
    void PrintGuyNameEx(QString guyNameEx);
    void PrintIdNum(QString idNum);
    void PrintIdType(QString idType);
    void PrintValidity(QString from, QString to);
    void readData();
    void showLog();
    void delLog();


protected:
    bool eventFilter(QObject *object, QEvent *event);

private:
    Ui::MainWindow *ui;
    About *about;

    QVBoxLayout *mainLayout;
    QHBoxLayout *topLayout;

    QLabel *Logo;

    QWidget *top;
    QComboBox *ReaderCombo;
    QPushButton *ConnectButton;

    QLabel *Number;
    QLabel *Validity;

    //QPlainTextEdit *host;
    QTextEdit *host;

    QPlainTextEdit *infoTextEdit;
    QWidget *InfoAera;
    QList<INFOLABEL> InfoList;
    QList<INFOLABEL>::Iterator ListIndex;

    //QLabel *statusLabel;

    QString ReaderName;
    QString PortName;
    SCARDCONTEXT hSC;
    QSerialPort *serial;
    ReadThread *readthread;
    QByteArray CurrentData;
    int CurrentProcess;

    QString guy_name;
    QString guy_name_ex;
    QString id_num;
    QString id_type;
    QString card_name;
    QString begin_time;
    QString end_time;
    QString card_type;
    QString card_num;
    QString all_records;
    unsigned int ItemsNumbers;
    unsigned int ReadItemAlready;

    bool bLog;
    QList<QString> LogHashList;

    QTimer *iTimer;

    bool bMasaicsMode;
    QLabel *status_main;
    QLabel *status_masaics;
};

#endif // MAINWINDOW_H
