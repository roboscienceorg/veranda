QT       += core

TARGET = diffDrive_drivetrain
TEMPLATE = app # Technically a plugin, but Intellisense doesn't like that being set

INCLUDEPATH += include ../pkg_Simulator/include

SOURCES += \
    src/diffDrive_drivetrain_plugin.cpp \
    src/diffDrive_drivetrain.cpp

HEADERS += \
    include/diffDrive_drivetrain.h \
    include/diffDrive_drivetrain_plugin.h

DISTFILES += \
    CMakeLists.txt \
    package.xml
