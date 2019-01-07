QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = veranda
TEMPLATE = app

INCLUDEPATH += include \
               # Hack to make box2d work with autocomplete in qtcreator
               ../pkg_Box2D/include \
               ../pkg_CatchTesting/include

SOURCES += \
    src/world_object.cpp \
    src/world_object_component.cpp \
    src/world_object.cpp \
    src/world_object_component.cpp \
    tests/test_model.cpp \
    tests/test_property.cpp \
    src/filter.cpp

HEADERS += \
    include/veranda_core/api/world_object.h \
    include/veranda_core/api/model.h \
    include/veranda_core/api/property.h \
    include/veranda_core/api/world_object_component_plugin.h \
    include/veranda_core/api/dllapi.h \
    include/veranda_core/api/file_handler_plugin.h \
    include/veranda_core/api/object_loader_if.h \
    include/veranda_core/api/object_saver_if.h \
    include/veranda_core/api/world_object_component.h \
    include/veranda_core/api/const.h \
    include/veranda_core/api/const.h \
    include/veranda_core/api/dllapi.h \
    include/veranda_core/api/file_handler_plugin.h \
    include/veranda_core/api/model.h \
    include/veranda_core/api/object_loader_if.h \
    include/veranda_core/api/object_saver_if.h \
    include/veranda_core/api/property.h \
    include/veranda_core/api/world_object.h \
    include/veranda_core/api/world_object_component.h \
    include/veranda_core/api/world_object_component_plugin.h \
    include/veranda_core/api/filter.h

DISTFILES += \
    CMakeLists.txt \
    package.xml
