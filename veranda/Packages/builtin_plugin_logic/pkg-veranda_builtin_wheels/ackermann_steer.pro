QT       += core

TARGET = dummy_target
TEMPLATE = app # Technically a plugin, but Intellisense doesn't like that being set

INCLUDEPATH += include ../pkg_SimulatorAPI/include ../pkg_Box2D/include

SOURCES += \
    src/ackermann_steer.cpp

HEADERS += \
    include/veranda_wheels/ackermann_steer.h \
    include/basic_wheel.h \
    include/veranda_wheels/dllapi.h

DISTFILES += \
    CMakeLists.txt \
    package.xml
