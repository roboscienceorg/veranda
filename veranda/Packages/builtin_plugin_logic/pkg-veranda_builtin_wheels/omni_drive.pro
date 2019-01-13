QT       += core

TARGET = dummy_target
TEMPLATE = app # Technically a plugin, but Intellisense doesn't like that being set

INCLUDEPATH += include
include(../../include_paths.pri)

SOURCES += \
    src/omni_drive.cpp

HEADERS += \
    include/veranda_wheels/omni_drive.h \
    include/veranda_wheels/dllapi.h

DISTFILES += \
    CMakeLists.txt \
    package.xml
