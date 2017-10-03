QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = sdsmt_simulator
TEMPLATE = app

INCLUDEPATH += include Box2D/Box2D

SOURCES += \
    src/main.cpp


HEADERS += \
    include/sdsmt_simulator.h \
    include/robot.h \
    include/map_loader_if.h \
    include/simulator_visual_if.h \
    include/simulator_ui_if.h \
    include/simulator_physics_if.h \
    include/sensor_if.h \
    include/drivetrain_if.h \
    include/robot_loader_if.h \
    include/robot_interfaces.h


FORMS += \


DISTFILES += \
    CMakeLists.txt \
    package.xml
