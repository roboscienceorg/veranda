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
    include/veranda_qt_frontend/file_handler_plugin.h \
    include/veranda_qt_frontend/object_loader_if.h \
    include/veranda_qt_frontend/object_saver_if.h
