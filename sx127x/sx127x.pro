#-------------------------------------------------
#
# Project created by QtCreator 2013-08-08T10:36:41
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = sx127x
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    spi.cpp \
    mylistwidget.cpp \
    cfg.cpp \
    formieee802154g.cpp \
    formlora.cpp

HEADERS  += mainwindow.h \
    spi.h \
    sx1272-common_regs.h \
    sx127x-LoRa_v2a.h \
    sx12xx-Fsk.h \
    mylistwidget.h \
    cfg.h \
    formieee802154g.h \
    formlora.h

FORMS    += mainwindow.ui \
    formieee802154g.ui \
    formlora.ui

unix:!macx:!symbian: LIBS += -lwiringPi
