QT       += core

TARGET = dummy_target
TEMPLATE = app # Technically a plugin, but Intellisense doesn't like that being set

INCLUDEPATH += include ../pkg_SimulatorAPI/include ../pkg_Box2D/include

SOURCES += \
    src/fixed_wheel.cpp \
    src/fixed_wheel_plugin.cpp

HEADERS += \
    include/fixed_wheel.h \
    include/fixed_wheel_plugin.h\
    include/basic_wheel.h \
    include/defines.h

DISTFILES += \
    CMakeLists.txt \
    package.xml
