#-------------------------------------------------
#
# Project created by QtCreator 2014-12-25T14:28:30
#
#-------------------------------------------------

QT       += core gui serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = EMVReader
TEMPLATE = app

RC_ICONS = images/favicon1.ico


SOURCES += main.cpp\
        mainwindow.cpp \
    about.cpp \
    TLVPackage.cpp \
    readthread.cpp \
    connecter.cpp

HEADERS  += mainwindow.h \
    about.h \
    TLVPackage.h \
    readthread.h \
    connecter.h

FORMS    += mainwindow.ui \
    about.ui

OTHER_FILES +=

RESOURCES += \
    EMVReader.qrc

LIBS += \
    Winscard.lib
