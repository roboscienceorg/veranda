QT       += core

TARGET = dummy_target
TEMPLATE = app # Technically a plugin, but Intellisense doesn't like that being set

INCLUDEPATH += include ../pkg_SimulatorAPI/include ../pkg_Box2D/include

SOURCES += \
    src/rectangle.cpp

HEADERS += \
    include/veranda_shapes/rectangle.h \
    include/veranda_shapes/dllapi.h

DISTFILES += \
    CMakeLists.txt \
    package.xml
