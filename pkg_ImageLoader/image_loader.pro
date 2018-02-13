QT       += core

TARGET = diffDrive_drivetrain
TEMPLATE = app # Technically a plugin, but Intellisense doesn't like that being set

INCLUDEPATH += include ../pkg_SimulatorAPI/include ../pkg_Box2D/include

SOURCES += \
    src/image_loader.cpp \
    src/image_loader_plugin.cpp \
    src/polygonscomponent.cpp \
    src/imageparser.cpp

HEADERS += \
    include/image_loader.h \
    include/image_loader_plugin.h \
    include/polygonscomponent.h \
    include/imageparser.h

DISTFILES += \
    CMakeLists.txt \
    package.xml
