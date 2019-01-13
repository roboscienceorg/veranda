QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = veranda
TEMPLATE = app

INCLUDEPATH += include
include(../../include_paths.pri)

DISTFILES += \
    package.xml \
    CMakeLists.txt

HEADERS += \
    include/veranda_qt_plugins/world_object_component_plugin.h
