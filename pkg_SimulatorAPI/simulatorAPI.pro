QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = sdsmt_simulator
TEMPLATE = app

INCLUDEPATH += include \
               # Hack to make box2d work with autocomplete in qtcreator
               ../pkg_Box2D/include

SOURCES += \
    src/world_object.cpp

HEADERS += \
    include/sdsmt_simulator/world_object.h \
    include/sdsmt_simulator/model.h \
    include/sdsmt_simulator/world_object_component_if.h \
    include/sdsmt_simulator/property.h \
    include/sdsmt_simulator/world_object_component_plugin.h \
    include/sdsmt_simulator/world_object_loader_if.h \
    include/sdsmt_simulator/world_object_saver_if.h \
    include/sdsmt_simulator/world_object_file_handler_plugin.h \
    include/sdsmt_simulator/dllapi.h

DISTFILES += \
    CMakeLists.txt \
    package.xml
