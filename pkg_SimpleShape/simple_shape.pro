QT       += core

TARGET = diffDrive_drivetrain
TEMPLATE = app # Technically a plugin, but Intellisense doesn't like that being set

INCLUDEPATH += include ../pkg_Simulator/include

SOURCES += \
    src/simple_shape.cpp \
    src/simple_shape_plugin.cpp

HEADERS += \
    include/simple_shape.h \
    include/simple_shape_plugin.h

DISTFILES += \
    CMakeLists.txt \
    package.xml
