QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = sdsmt_simulator
TEMPLATE = app

INCLUDEPATH += include Box2D/Box2D/Box2D

SOURCES += \
    src/main.cpp


HEADERS += \
    include/sdsmt_simulator.h \
    include/simulator_ui.h \
    include/simulator_physics.h \
    include/simulator_visual.h \
    include/robot.h \
    include/sensor.h \
    include/drivetrain.h


FORMS += \


DISTFILES += \
    CMakeLists.txt \
    package.xml
