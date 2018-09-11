#-------------------------------------------------
#
# Project created by QtCreator 2017-01-13T21:21:24
#
# Name format standard:
#       Function:   small letter begin like: serialInit()
#       Signal:     sig- like: sigSerialInit()
#       Slot:       slot- like: slotSerialInit()
#       Constant:   all big letter like: MAXSPEED
#-------------------------------------------------

QT       += core gui
QT       += serialport
QT      += datavisualization

TARGET = li_window_1  printsupport
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    qcustomplot.cpp \
    aboutdialog.cpp \
    getsensordata.cpp \
    motorcontrol.cpp \
    vrdisplay.cpp \
    tensioncontrol.cpp \
    linkdisplay.cpp

HEADERS  += mainwindow.h \
    qcustomplot.h \
    aboutdialog.h \
    DeviceListItem.h \
    ImuData.h \
    LpmsDefinitions.h \
    LpmsSensorI.h \
    LpmsSensorManagerI.h \
    getsensordata.h \
    motorcontrol.h \
    vrdisplay.h \
    tensioncontrol.h \
    linkdisplay.h

FORMS    += mainwindow.ui \
    aboutdialog.ui

RESOURCES += \
    li.qrc

QT  +=opengl

#LIBS += -lglu
#win32:LIBS += -lOpengl32 \
#                -lglu32 \
#                -lglut
QT += axcontainer
QT  +=  printsupport

LIBS += -LD:/Document_LW/Github/TensionControl/li_window_1 -lLpSensor64
LIBS += -LD:/Document_LW/Github/TensionControl/li_window_1 -lLpSensorD64

