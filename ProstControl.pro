
QT       += core gui
QT       += concurrent

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = ProstControl
TEMPLATE = app
#CONFIG   += console
#CONFIG   -= app_bundle

SOURCES += \
    main.cpp \
    ads.cpp \
    hdf5_custom.cpp \
    controller.cpp \
    filterbutterworth.cpp \
    rms.cpp \
    serial.cpp \
    constants.cpp \
    i2c.cpp \
    libs/I2Cdev/I2Cdev.cpp \
    MotionSensor/inv_mpu_lib/inv_mpu.cpp \
    MotionSensor/inv_mpu_lib/inv_mpu_dmp_motion_driver.cpp \
    MotionSensor/sensor.cpp \
    EMGFilters.cpp \
    ultrasound.cpp \
    adc.cpp \
    StepperMotor28BYJ48.cpp

HEADERS  += \
    ads.h \
    hdf5_custom.h \
    main.h \
    controller.h \
    filterbutterworth.h \
    rms.h \
    serial.h \
    constants.h \
    i2c.h \
    libs/I2Cdev/I2Cdev.h \
    MotionSensor/inv_mpu_lib/dmpKey.h \
    MotionSensor/inv_mpu_lib/dmpmap.h \
    MotionSensor/inv_mpu_lib/inv_mpu.h \
    MotionSensor/inv_mpu_lib/inv_mpu_dmp_motion_driver.h \
    MotionSensor/helper_3dmath.h \
    MotionSensor/sensor.h \
    MotionSensor.h \
    EMGFilters.h \
    ultrasound.h \
    adc.h \
    StepperMotor28BYJ48.h

FORMS    += \
    mainwindow.ui

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../../usr/lib/release/ -lphidget21
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../../usr/lib/debug/ -lphidget21
else:unix: LIBS += -L$$PWD/../../../../../usr/lib/ -lphidget21

CONFIG += c++11

INCLUDEPATH += $$PWD/../../../../../usr/include
DEPENDPATH += $$PWD/../../../../../usr/include

# bcm2835 library

unix:!macx: LIBS += -L$$PWD/../../../../../usr/local/lib/ -lbcm2835

INCLUDEPATH += $$PWD/../../../../../usr/local/include
DEPENDPATH += $$PWD/../../../../../usr/local/include

unix:!macx: PRE_TARGETDEPS += $$PWD/../../../../../usr/local/lib/libbcm2835.a


#wiringPi

unix:!macx: LIBS += -L$$PWD/../../../../../usr/local/lib/ -lwiringPi

INCLUDEPATH += $$PWD/../../../../../usr/local/include
DEPENDPATH += $$PWD/../../../../../usr/local/include


#hdf5


unix:!macx: LIBS += -L$$PWD/../../../../../usr/local/hdf5/lib/ -lhdf5

INCLUDEPATH += $$PWD/../../../../../usr/local/hdf5/include
DEPENDPATH += $$PWD/../../../../../usr/local/hdf5/include

unix:!macx: PRE_TARGETDEPS += $$PWD/../../../../../usr/local/hdf5/lib/libhdf5.a


#Looking for .so files in Build Folder
QMAKE_LFLAGS += -Wl,-rpath,"'\$$ORIGIN'"
