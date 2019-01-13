QT       += core

TARGET = dummy_target
TEMPLATE = app # Technically a plugin, but Intellisense doesn't like that being set

INCLUDEPATH += include
include(../../include_paths.pri)

SOURCES += \
    src/circle.cpp

HEADERS += \
    include/veranda_shapes/circle.h \
    include/veranda_shapes/dllapi.h

DISTFILES += \
    CMakeLists.txt \
    package.xml
