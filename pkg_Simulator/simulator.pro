QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = sdsmt_simulator
TEMPLATE = app

INCLUDEPATH += include \
               # Hack to make autocomplete work in qtcreator
               ../pkg_Box2D/include \
               ../pkg_SimulatorAPI/include \
                /opt/ros/ardent/include

SOURCES += \
    src/main.cpp \
    src/basic_viewer.cpp \
    src/basic_physics.cpp \
\
    src/ui/mainwindow.cpp \
    src/simulator_core.cpp \
    src/ui/joystickprototype.cpp \
    ui/settingspopup.cpp \
    src/ui/settingspopup.cpp \
    src/ui/designer_widget.cpp

HEADERS += \
\
    include/interfaces/simulator_visual_if.h \
    include/interfaces/simulator_ui_if.h \
    include/interfaces/simulator_physics_if.h \
\
    include/basic_physics.h \
    include/basic_viewer.h \
\
    include/ui/mainwindow.h \
\
    include/simulator_core.h \
    include/interfaces/world_object_wrappers.h \
    include/ui/joystickprototype.h \
    ui/settingspopup.h \
    include/ui/settingspopup.h \
    src/ui/designer_widget.h \
    include/ui/designer_widget.h

FORMS    += \
    ui/mainwindow.ui \
    ui/emptysimwindow.ui \
    ui/joystickprototype.ui \
    ui/settingspopup.ui

RESOURCES += \
    ui/resources.qrc

DISTFILES += \
    CMakeLists.txt \
    package.xml
