QT       += core

TARGET = diffDrive_drivetrain
TEMPLATE = app # Technically a plugin, but Intellisense doesn't like that being set

INCLUDEPATH += include ../pkg_Simulator/include

SOURCES += \
    src/fixed_wheel.cpp \
    src/fixed_wheel_plugin.cpp

HEADERS += \
    include/fixed_wheel.h \
    include/fixed_wheel_plugin.h

DISTFILES += \
    CMakeLists.txt \
    package.xml