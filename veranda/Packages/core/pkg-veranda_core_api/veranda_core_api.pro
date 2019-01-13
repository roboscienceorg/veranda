QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = veranda
TEMPLATE = app

INCLUDEPATH += include
include(../../include_paths.pri)

SOURCES += \
    src/filter.cpp \
    src/world_object.cpp \
    src/world_object_component.cpp \
    tests/test_model.cpp \
    tests/test_property.cpp

HEADERS += \
    include/veranda_core/api/interfaces/simulator_physics_if.h \
    include/veranda_core/api/interfaces/simulator_ui_if.h \
    include/veranda_core/api/interfaces/world_object_component_factory_if.h \
    include/veranda_core/api/interfaces/world_object_wrappers.h \
    include/veranda_core/api/const.h \
    include/veranda_core/api/dllapi.h \
    include/veranda_core/api/filter.h \
    include/veranda_core/api/model.h \
    include/veranda_core/api/property.h \
    include/veranda_core/api/world_object.h \
    include/veranda_core/api/world_object_component.h

DISTFILES += \
    CMakeLists.txt \
    package.xml
