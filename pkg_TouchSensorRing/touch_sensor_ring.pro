QT       += core

TARGET = diffDrive_drivetrain
TEMPLATE = app # Technically a plugin, but Intellisense doesn't like that being set

INCLUDEPATH += include ../pkg_Simulator/include

SOURCES += \
    src/touch_sensor.cpp \
    src/touch_sensor_plugin.cpp

HEADERS += \
    include/touch_sensor.h \
    include/touch_sensor_plugin.h

DISTFILES += \
    CMakeLists.txt \
    package.xml
