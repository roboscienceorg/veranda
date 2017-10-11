QT       += core

TARGET = floating_drivetrain
TEMPLATE = plugin

INCLUDEPATH += include

SOURCES += \
    src/floating_drivetrain_plugin.cpp \
    src/floating_drivetrain.cpp

HEADERS += \
    include/floating_drivetrain.h \
    include/floating_drivetrain_plugin.h

DISTFILES += \
    CMakeLists.txt \
    package.xml
