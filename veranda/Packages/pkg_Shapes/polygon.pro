QT       += core

TARGET = dummy_target
TEMPLATE = app # Technically a plugin, but Intellisense doesn't like that being set

INCLUDEPATH += include ../pkg_SimulatorAPI/include ../pkg_Box2D/include

SOURCES += \
    src/polygon_plugin.cpp \
    src/polygon.cpp \
    src/triangulator.cpp

HEADERS += \
    include/polygon.h \
    include/polygon_plugin.h \
    include/triangulator.h \
    include/defines.h \
    include/psimpl/psimpl.h \
    include/polygon_iterator.h

DISTFILES += \
    CMakeLists.txt \
    package.xml
