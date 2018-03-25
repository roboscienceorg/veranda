QT       += core

TARGET = diffDrive_drivetrain
TEMPLATE = app # Technically a plugin, but Intellisense doesn't like that being set

INCLUDEPATH += include ../pkg_SimulatorAPI/include ../pkg_Box2D/include

SOURCES += \
    src/rectangle.cpp \
    src/rectangle_plugin.cpp

HEADERS += \
    include/rectangle.h \
    include/rectangle_plugin.h \
    include/defines.h

DISTFILES += \
    CMakeLists.txt \
    package.xml
