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
    tests/test_property.cpp

HEADERS += \
    include/veranda/world_object.h \
    include/veranda/model.h \
    include/veranda/property.h \
    include/veranda/world_object_component_plugin.h \
    include/veranda/dllapi.h \
    include/veranda/file_handler_plugin.h \
    include/veranda/object_loader_if.h \
    include/veranda/object_saver_if.h \
    include/veranda/world_object_component.h \
    include/veranda/const.h \
    include/veranda/const.h \
    include/veranda/dllapi.h \
    include/veranda/file_handler_plugin.h \
    include/veranda/model.h \
    include/veranda/object_loader_if.h \
    include/veranda/object_saver_if.h \
    include/veranda/property.h \
    include/veranda/world_object.h \
    include/veranda/world_object_component.h \
    include/veranda/world_object_component_plugin.h

DISTFILES += \
    CMakeLists.txt \
    package.xml
