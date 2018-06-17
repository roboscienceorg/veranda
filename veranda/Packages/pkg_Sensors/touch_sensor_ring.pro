QT       += core

TARGET = dummy_target
TEMPLATE = app # Technically a plugin, but Intellisense doesn't like that being set

INCLUDEPATH += include ../pkg_SimulatorAPI/include ../pkg_Box2D/include

SOURCES += \
    src/touch_sensor.cpp \
    src/touch_sensor_plugin.cpp

HEADERS += \
    include/touch_sensor.h \
    include/touch_sensor_plugin.h \
    include/defines.h

DISTFILES += \
    CMakeLists.txt \
    package.xml
