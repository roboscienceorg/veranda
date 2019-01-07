QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = veranda
TEMPLATE = app

INCLUDEPATH += include \
               # Hack to make autocomplete work in qtcreator
               ../pkg_Box2D/include \
               ../pkg_SimulatorAPI/include

SOURCES += \
    src/basic_physics.cpp \
    src/simulator_core.cpp

HEADERS += \
    include/veranda_core/implementation/basic_physics.h \
    include/veranda_core/implementation/dllapi.h \
    include/veranda_core/implementation/simulator_core.h

DISTFILES += \
    CMakeLists.txt \
    package.xml \
    veranda/__init__.py \
    veranda/SimTimer.py
