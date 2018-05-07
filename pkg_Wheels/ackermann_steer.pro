QT       += core

TARGET = dummy_target
TEMPLATE = app # Technically a plugin, but Intellisense doesn't like that being set

INCLUDEPATH += include ../pkg_SimulatorAPI/include ../pkg_Box2D/include

SOURCES += \
    src/ackermann_steer.cpp \
    src/ackermann_steer_plugin.cpp

HEADERS += \
    include/ackermann_steer.h \
    include/ackermann_steer_plugin.h \
    include/basic_wheel.h \
    include/defines.h

DISTFILES += \
    CMakeLists.txt \
    package.xml
