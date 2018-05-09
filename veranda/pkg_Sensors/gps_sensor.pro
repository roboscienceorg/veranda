QT       += core

TARGET = dummy_target
TEMPLATE = app # Technically a plugin, but Intellisense doesn't like that being set

INCLUDEPATH += include ../pkg_SimulatorAPI/include ../pkg_Box2D/include

SOURCES += \
    src/gps_sensor.cpp \
    src/gps_sensor_plugin.cpp

HEADERS += \
    include/defines.h \
    include/gps_sensor.h \
    include/gps_sensor_plugin.h

DISTFILES += \
    CMakeLists.txt \
    package.xml
