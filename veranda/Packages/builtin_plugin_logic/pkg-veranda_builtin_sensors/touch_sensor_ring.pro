QT       += core

TARGET = dummy_target
TEMPLATE = app # Technically a plugin, but Intellisense doesn't like that being set

INCLUDEPATH += include ../pkg_SimulatorAPI/include ../pkg_Box2D/include

SOURCES += \
    src/touch_sensor.cpp

HEADERS += \
    include/veranda_sensors/touch_sensor.h \
    include/veranda_sensors/dllapi.h

DISTFILES += \
    CMakeLists.txt \
    package.xml
