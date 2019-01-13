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
    include/ackermann_steer_plugin.h \
    include/circle_plugin.h \
    include/defines.h \
    include/fixed_wheel_plugin.h \
    include/gps_sensor_plugin.h \
    include/lidar_sensor_plugin.h \
    include/omni_drive_plugin.h \
    include/polygon_plugin.h \
    include/rectangle_plugin.h \
    include/touch_sensor_plugin.h

SOURCES += \
    src/ackermann_steer_plugin.cpp \
    src/circle_plugin.cpp \
    src/fixed_wheel_plugin.cpp \
    src/gps_sensor_plugin.cpp \
    src/lidar_sensor_plugin.cpp \
    src/omni_drive_plugin.cpp \
    src/polygon_plugin.cpp \
    src/rectangle_plugin.cpp \
    src/touch_sensor_plugin.cpp
