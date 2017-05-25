TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt
#QMAKE_CXXFLAGS += -std=gnu++11

SOURCES += main.cpp \
    rs232.c

include(deployment.pri)
qtcAddDeployment()

HEADERS += \
    rs232.h \
    stdafx.h \
    ArduinoSerialCommunicator.hpp

LIBS += -L/usr/local/libs -lopencv_core -lopencv_highgui \
-lopencv_imgcodecs -lopencv_imgproc \
-lopencv_video -lopencv_videoio
