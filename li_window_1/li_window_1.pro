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
win32 {
#    LIBS += -LD:/Document_LW/Github/TensionControl/li_window_1/ -lLpSensor64
#    LIBS += -LD:/Document_LW/Github/TensionControl/li_window_1/ -lLpSensorD64
    LIBS += -L$$PWD\lib\x64 -lLpSensor64
    LIBS += -L$$PWD\lib\x64 -lLpSensorD64
}

QT       += core gui
QT       += serialport serialbus
QT      += datavisualization
QT += 3dcore 3drender 3dinput 3dextras
QT       += network

TARGET = li_window_1  printsupport
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    qcustomplot.cpp \
    aboutdialog.cpp \
    getsensordata.cpp \
    vrdisplay.cpp \
    linkdisplay.cpp \
    scatterdatamodifier.cpp \
    emg_tcp.cpp \
    modbus.cpp

HEADERS  += mainwindow.h \
    qcustomplot.h \
    aboutdialog.h \
    DeviceListItem.h \
    ImuData.h \
    LpmsDefinitions.h \
    LpmsSensorI.h \
    LpmsSensorManagerI.h \
    getsensordata.h \
    vrdisplay.h \
    linkdisplay.h \
    scatterdatamodifier.h \
    emg_tcp.h \
    modbus.h

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

#DEFINES += GSL_DLL
#INCLUDEPATH += $$PWD\gsl\include
#LIBS += -L$$PWD\gsl\lib -llibgsl
#LIBS += -L$$PWD\gsl\lib -llibgslcblas




