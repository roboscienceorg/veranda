QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = sdsmt_simulator
TEMPLATE = app

INCLUDEPATH += include Box2D/Box2D

SOURCES += \
    src/main.cpp \
    src/sdsmt_simulator.cpp \
    src/basic_viewer.cpp \
    src/basic_ui.cpp \
    src/basic_physics.cpp \
    src/basic_maploader.cpp \
    src/basic_robotloader.cpp \
    src/robot.cpp \
    \
    src/ui/emptysimwindow.cpp
    src/ui/mainwindow.cpp

HEADERS += \
    include/interfaces/map_loader_if.h \
    include/interfaces/simulator_visual_if.h \
    include/interfaces/simulator_ui_if.h \
    include/interfaces/simulator_physics_if.h \
    include/interfaces/sensor_if.h \
    include/interfaces/drivetrain_if.h \
    include/interfaces/robot_loader_if.h \
    include/interfaces/robot_interfaces.h \
    include/interfaces/screen_model_if.h \
\
    include/basic_physics.h \
    include/basic_ui.h \
    include/basic_viewer.h \
    include/basic_maploader.h \
    include/basic_robotloader.h \
    include/ui/emptysimwindow.h \
\
    include/sdsmt_simulator.h \
    include/robot.h \
    include/interfaces/screen_model_if.h \
    include/ui/mainwindow.h

FORMS    += \
    ui/mainwindow.ui \
    ui/emptysimwindow.ui

RESOURCES += \
    ui/resources.qrc

DISTFILES += \
    CMakeLists.txt \
    package.xml
