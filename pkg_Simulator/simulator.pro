QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = sdsmt_simulator
TEMPLATE = app

INCLUDEPATH += include \
               # Hack to make box2d work with autocomplete in qtcreator
               ../pkg_Box2D/include

SOURCES += \
    src/main.cpp \
    src/basic_viewer.cpp \
    src/basic_physics.cpp \
    src/basic_maploader.cpp \
    src/basic_robotloader.cpp \
    src/robot.cpp \
\
    src/ui/mainwindow.cpp \
    src/simulator_core.cpp \
    src/map.cpp

HEADERS += \
    include/sdsmt_simulator/sensor_if.h \
    include/sdsmt_simulator/drivetrain_if.h \
\
    include/interfaces/map_loader_if.h \
    include/interfaces/simulator_visual_if.h \
    include/interfaces/simulator_ui_if.h \
    include/interfaces/simulator_physics_if.h \
    include/interfaces/robot_loader_if.h \
\
    include/basic_physics.h \
    include/basic_viewer.h \
    include/basic_maploader.h \
    include/basic_robotloader.h \
\
    include/robot.h \
    include/ui/mainwindow.h \
\
    include/simulator_core.h \
    include/sdsmt_simulator/sensor_plugin.h \
    include/sdsmt_simulator/drivetrain_plugin.h \
    include/sdsmt_simulator/robotcomponent_if.h \
    include/map.h \
    include/sdsmt_simulator/properties_if.h \
    include/sdsmt_simulator/model.h \
    include/interfaces/world_object_if.h

FORMS    += \
    ui/mainwindow.ui \
    ui/emptysimwindow.ui

RESOURCES += \
    ui/resources.qrc

DISTFILES += \
    CMakeLists.txt \
    package.xml
