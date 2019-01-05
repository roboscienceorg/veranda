QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = veranda
TEMPLATE = app

INCLUDEPATH += include \
               # Hack to make autocomplete work in qtcreator
               ../pkg_Box2D/include \
               ../pkg_SimulatorAPI/include \
                /opt/ros/ardent/include

SOURCES += \
    src/main.cpp \
    src/basic_physics.cpp \
\
    src/ui/mainwindow.cpp \
    src/simulator_core.cpp \
    src/ui/joystickprototype.cpp \
    src/ui/settingspopup.cpp \
    src/ui/designer_widget.cpp \
    src/ui/mode_controller.cpp \
    src/ui/settingspopup.cpp \
    src/ui/qgraphicssimulationviewer.cpp

HEADERS += \
\
    include/interfaces/simulator_visual_if.h \
    include/interfaces/simulator_ui_if.h \
    include/interfaces/simulator_physics_if.h \
\
    include/basic_physics.h \
\
    include/ui/mainwindow.h \
\
    include/simulator_core.h \
    include/interfaces/world_object_wrappers.h \
    include/ui/joystickprototype.h \
    include/ui/settingspopup.h \
    include/ui/designer_widget.h \
    include/ui/mode_controller.h \
    include/ui/qgraphicssimulationviewer.h \
    include/ui/settingspopup.h \
    include/ui/designer_widget.h \
    include/ui/mode_controller.h \
    include/ui/customgraphicsview.h

FORMS    += \
    ui/mainwindow.ui \
    ui/emptysimwindow.ui \
    ui/joystickprototype.ui \
    ui/settingspopup.ui \
    ui/qgraphicssimulationviewer.ui

RESOURCES += \
    ui/resources.qrc

DISTFILES += \
    CMakeLists.txt \
    package.xml \
    veranda_core/SimTimer.py
