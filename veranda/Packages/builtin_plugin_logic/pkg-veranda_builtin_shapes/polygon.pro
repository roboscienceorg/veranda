QT       += core

TARGET = dummy_target
TEMPLATE = app # Technically a plugin, but Intellisense doesn't like that being set

INCLUDEPATH += include ../pkg_SimulatorAPI/include ../pkg_Box2D/include

SOURCES += \
    src/polygon.cpp \
    src/triangulator.cpp

HEADERS += \
    include/veranda_shapes/polygon.h \
    include/triangulator.h \
    include/psimpl/psimpl.h \
    include/polygon_iterator.h \
    include/veranda_shapes/dllapi.h

DISTFILES += \
    CMakeLists.txt \
    package.xml
