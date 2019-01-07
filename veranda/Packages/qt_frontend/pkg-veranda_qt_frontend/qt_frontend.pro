QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = veranda
TEMPLATE = app

INCLUDEPATH += include \
               # Hack to make box2d work with autocomplete in qtcreator
               ../pkg_Box2D/include \
               ../pkg_CatchTesting/include

RESOURCES += \
    ui/resources.qrc

FORMS += \
    ui/emptysimwindow.ui \
    ui/joystickprototype.ui \
    ui/mainwindow.ui \
    ui/qgraphicssimulationviewer.ui \
    ui/settingspopup.ui

DISTFILES += \
    package.xml \
    ui/images/DeleteFileIcon.png \
    ui/images/DesignerIcon.png \
    ui/images/ExportFileIcon.png \
    ui/images/JoystickIcon.png \
    ui/images/LoadFileIcon.png \
    ui/images/NewFileIcon.png \
    ui/images/PlayIcon.png \
    ui/images/QuickLoadIcon.png \
    ui/images/QuickSaveIcon.png \
    ui/images/SaveFileIcon.png \
    ui/images/ScreenshotIcon.png \
    ui/images/SimulatorIcon.png \
    ui/images/SpeedHalfIcon.png \
    ui/images/SpeedOneIcon.png \
    ui/images/SpeedThreeIcon.png \
    ui/images/SpeedTwoIcon.png \
    ui/images/StopIcon.png \
    ui/images/icon_information.txt \
    CMakeLists.txt

HEADERS += \
    include/interfaces/simulator_visual_if.h \
    include/ui/customgraphicsview.h \
    include/ui/designer_widget.h \
    include/ui/joystickprototype.h \
    include/ui/mainwindow.h \
    include/ui/mode_controller.h \
    include/ui/qgraphicssimulationviewer.h \
    include/ui/settingspopup.h

SOURCES += \
    src/ui/designer_widget.cpp \
    src/ui/joystickprototype.cpp \
    src/ui/mainwindow.cpp \
    src/ui/mode_controller.cpp \
    src/ui/qgraphicssimulationviewer.cpp \
    src/ui/settingspopup.cpp \
    src/main.cpp
