QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = sdsmt_simulator
TEMPLATE = app

INCLUDEPATH += include \
               # Hack to make autocomplete work in qtcreator
               ../pkg_Box2D/include \
               ../pkg_SimulatorAPI/include

SOURCES += \
    src/main.cpp \
    src/basic_viewer.cpp \
    src/basic_physics.cpp \
\
    src/ui/mainwindow.cpp \
    src/simulator_core.cpp

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
    include/interfaces/world_object_wrappers.h

FORMS    += \
    ui/mainwindow.ui \
    ui/emptysimwindow.ui

RESOURCES += \
    ui/resources.qrc

DISTFILES += \
    CMakeLists.txt \
    package.xml
