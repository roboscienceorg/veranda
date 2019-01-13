QT       += core

TARGET = dummy_target
TEMPLATE = app # Technically a plugin, but Intellisense doesn't like that being set

INCLUDEPATH += include
include(../../include_paths.pri)

SOURCES += \
    src/gps_sensor.cpp

HEADERS += \
    include/veranda_sensors/gps_sensor.h \
    include/veranda_sensors/dllapi.h

DISTFILES += \
    CMakeLists.txt \
    package.xml
