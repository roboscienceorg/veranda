/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QListWidget>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTableView>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QToolButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QWidget *horizontalLayoutWidget;
    QHBoxLayout *mainLayout;
    QHBoxLayout *menuLayout;
    QWidget *menuWidget;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *ModesLayout;
    QToolButton *simulatorButton;
    QToolButton *designerButton;
    QLabel *modeLabel;
    QWidget *simulatorMenuWidget;
    QHBoxLayout *simulationMenuLayout;
    QToolButton *joystickButton;
    QToolButton *saveSimButton;
    QToolButton *screenshotSimButton;
    QToolButton *importMapButton;
    QToolButton *restartSimButton;
    QToolButton *speedSimButton;
    QToolButton *playSimButton;
    QWidget *designerMenuWidget;
    QHBoxLayout *robotModeMenuLayout;
    QToolButton *newObjectButton;
    QToolButton *loadObjectButton;
    QToolButton *saveObjectButton;
    QLabel *activeObjectsLabel;
    QListWidget *robotsWidget;
    QLabel *propertiesLabel;
    QTableView *propertiesTableView;
    QToolButton *showMenuButton;
    QHBoxLayout *worldViewLayout;
    QHBoxLayout *buildObjectsLayout;
    QToolButton *showBuildObjectsButton;
    QWidget *buildToolsWidget;
    QVBoxLayout *buildToolsLayout;
    QLabel *buildToolsLabel;
    QWidget *simulatorToolsMenu;
    QHBoxLayout *worldObjectsMenu;
    QToolButton *addObjectButton;
    QToolButton *deleteObjectButton;
    QWidget *designerToolsMenu;
    QHBoxLayout *designerToolsLayout;
    QToolButton *deleteToolButton;
    QToolButton *addToolButton;
    QToolButton *exportObjectButton;
    QListWidget *buildToolsList;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(1540, 855);
        QSizePolicy sizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(MainWindow->sizePolicy().hasHeightForWidth());
        MainWindow->setSizePolicy(sizePolicy);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        centralWidget->setEnabled(true);
        sizePolicy.setHeightForWidth(centralWidget->sizePolicy().hasHeightForWidth());
        centralWidget->setSizePolicy(sizePolicy);
        horizontalLayoutWidget = new QWidget(centralWidget);
        horizontalLayoutWidget->setObjectName(QStringLiteral("horizontalLayoutWidget"));
        horizontalLayoutWidget->setGeometry(QRect(9, -1, 1521, 794));
        mainLayout = new QHBoxLayout(horizontalLayoutWidget);
        mainLayout->setSpacing(6);
        mainLayout->setContentsMargins(11, 11, 11, 11);
        mainLayout->setObjectName(QStringLiteral("mainLayout"));
        mainLayout->setSizeConstraint(QLayout::SetNoConstraint);
        mainLayout->setContentsMargins(0, 0, 0, 0);
        menuLayout = new QHBoxLayout();
        menuLayout->setSpacing(6);
        menuLayout->setObjectName(QStringLiteral("menuLayout"));
        menuWidget = new QWidget(horizontalLayoutWidget);
        menuWidget->setObjectName(QStringLiteral("menuWidget"));
        verticalLayout = new QVBoxLayout(menuWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        ModesLayout = new QHBoxLayout();
        ModesLayout->setSpacing(6);
        ModesLayout->setObjectName(QStringLiteral("ModesLayout"));
        simulatorButton = new QToolButton(menuWidget);
        simulatorButton->setObjectName(QStringLiteral("simulatorButton"));
        QIcon icon;
        icon.addFile(QStringLiteral(":/modes/SimulatorIcon"), QSize(), QIcon::Normal, QIcon::Off);
        simulatorButton->setIcon(icon);
        simulatorButton->setIconSize(QSize(128, 128));

        ModesLayout->addWidget(simulatorButton);

        designerButton = new QToolButton(menuWidget);
        designerButton->setObjectName(QStringLiteral("designerButton"));
        QIcon icon1;
        icon1.addFile(QStringLiteral(":/modes/DesignerIcon"), QSize(), QIcon::Normal, QIcon::Off);
        designerButton->setIcon(icon1);
        designerButton->setIconSize(QSize(128, 128));

        ModesLayout->addWidget(designerButton);


        verticalLayout->addLayout(ModesLayout);

        modeLabel = new QLabel(menuWidget);
        modeLabel->setObjectName(QStringLiteral("modeLabel"));
        QFont font;
        font.setKerning(true);
        modeLabel->setFont(font);
        modeLabel->setFocusPolicy(Qt::NoFocus);
        modeLabel->setAcceptDrops(false);
        modeLabel->setFrameShape(QFrame::Panel);
        modeLabel->setFrameShadow(QFrame::Raised);
        modeLabel->setAlignment(Qt::AlignCenter);

        verticalLayout->addWidget(modeLabel);

        simulatorMenuWidget = new QWidget(menuWidget);
        simulatorMenuWidget->setObjectName(QStringLiteral("simulatorMenuWidget"));
        simulationMenuLayout = new QHBoxLayout(simulatorMenuWidget);
        simulationMenuLayout->setSpacing(6);
        simulationMenuLayout->setContentsMargins(11, 11, 11, 11);
        simulationMenuLayout->setObjectName(QStringLiteral("simulationMenuLayout"));
        joystickButton = new QToolButton(simulatorMenuWidget);
        joystickButton->setObjectName(QStringLiteral("joystickButton"));
        QIcon icon2;
        icon2.addFile(QStringLiteral(":/sim/JoystickIcon"), QSize(), QIcon::Normal, QIcon::Off);
        joystickButton->setIcon(icon2);
        joystickButton->setIconSize(QSize(32, 32));

        simulationMenuLayout->addWidget(joystickButton);

        saveSimButton = new QToolButton(simulatorMenuWidget);
        saveSimButton->setObjectName(QStringLiteral("saveSimButton"));
        QIcon icon3;
        icon3.addFile(QStringLiteral(":/sim/SaveFileIcon"), QSize(), QIcon::Normal, QIcon::Off);
        saveSimButton->setIcon(icon3);
        saveSimButton->setIconSize(QSize(32, 32));

        simulationMenuLayout->addWidget(saveSimButton);

        screenshotSimButton = new QToolButton(simulatorMenuWidget);
        screenshotSimButton->setObjectName(QStringLiteral("screenshotSimButton"));
        QIcon icon4;
        icon4.addFile(QStringLiteral(":/sim/ScreenshotSimIcon"), QSize(), QIcon::Normal, QIcon::Off);
        screenshotSimButton->setIcon(icon4);
        screenshotSimButton->setIconSize(QSize(32, 32));

        simulationMenuLayout->addWidget(screenshotSimButton);

        importMapButton = new QToolButton(simulatorMenuWidget);
        importMapButton->setObjectName(QStringLiteral("importMapButton"));
        QIcon icon5;
        icon5.addFile(QStringLiteral(":/sim/ImportMapIcon"), QSize(), QIcon::Normal, QIcon::Off);
        importMapButton->setIcon(icon5);
        importMapButton->setIconSize(QSize(32, 32));

        simulationMenuLayout->addWidget(importMapButton);

        restartSimButton = new QToolButton(simulatorMenuWidget);
        restartSimButton->setObjectName(QStringLiteral("restartSimButton"));
        QIcon icon6;
        icon6.addFile(QStringLiteral(":/sim/RestartSimIcon"), QSize(), QIcon::Normal, QIcon::Off);
        restartSimButton->setIcon(icon6);
        restartSimButton->setIconSize(QSize(32, 32));

        simulationMenuLayout->addWidget(restartSimButton);

        speedSimButton = new QToolButton(simulatorMenuWidget);
        speedSimButton->setObjectName(QStringLiteral("speedSimButton"));
        QIcon icon7;
        icon7.addFile(QStringLiteral(":/sim/SpeedOneSimIcon"), QSize(), QIcon::Normal, QIcon::Off);
        speedSimButton->setIcon(icon7);
        speedSimButton->setIconSize(QSize(32, 32));

        simulationMenuLayout->addWidget(speedSimButton);

        playSimButton = new QToolButton(simulatorMenuWidget);
        playSimButton->setObjectName(QStringLiteral("playSimButton"));
        QIcon icon8;
        icon8.addFile(QStringLiteral(":/sim/PlaySimIcon"), QSize(), QIcon::Normal, QIcon::Off);
        playSimButton->setIcon(icon8);
        playSimButton->setIconSize(QSize(32, 32));

        simulationMenuLayout->addWidget(playSimButton);


        verticalLayout->addWidget(simulatorMenuWidget);

        designerMenuWidget = new QWidget(menuWidget);
        designerMenuWidget->setObjectName(QStringLiteral("designerMenuWidget"));
        robotModeMenuLayout = new QHBoxLayout(designerMenuWidget);
        robotModeMenuLayout->setSpacing(6);
        robotModeMenuLayout->setContentsMargins(11, 11, 11, 11);
        robotModeMenuLayout->setObjectName(QStringLiteral("robotModeMenuLayout"));
        newObjectButton = new QToolButton(designerMenuWidget);
        newObjectButton->setObjectName(QStringLiteral("newObjectButton"));
        QIcon icon9;
        icon9.addFile(QStringLiteral(":/des/NewFileIcon"), QSize(), QIcon::Normal, QIcon::Off);
        newObjectButton->setIcon(icon9);
        newObjectButton->setIconSize(QSize(32, 32));

        robotModeMenuLayout->addWidget(newObjectButton);

        loadObjectButton = new QToolButton(designerMenuWidget);
        loadObjectButton->setObjectName(QStringLiteral("loadObjectButton"));
        loadObjectButton->setIcon(icon5);
        loadObjectButton->setIconSize(QSize(32, 32));

        robotModeMenuLayout->addWidget(loadObjectButton);

        saveObjectButton = new QToolButton(designerMenuWidget);
        saveObjectButton->setObjectName(QStringLiteral("saveObjectButton"));
        saveObjectButton->setIcon(icon3);
        saveObjectButton->setIconSize(QSize(32, 32));

        robotModeMenuLayout->addWidget(saveObjectButton);


        verticalLayout->addWidget(designerMenuWidget);

        activeObjectsLabel = new QLabel(menuWidget);
        activeObjectsLabel->setObjectName(QStringLiteral("activeObjectsLabel"));
        activeObjectsLabel->setFrameShape(QFrame::Panel);
        activeObjectsLabel->setFrameShadow(QFrame::Raised);
        activeObjectsLabel->setAlignment(Qt::AlignCenter);

        verticalLayout->addWidget(activeObjectsLabel);

        robotsWidget = new QListWidget(menuWidget);
        robotsWidget->setObjectName(QStringLiteral("robotsWidget"));

        verticalLayout->addWidget(robotsWidget);

        propertiesLabel = new QLabel(menuWidget);
        propertiesLabel->setObjectName(QStringLiteral("propertiesLabel"));
        propertiesLabel->setFrameShape(QFrame::Panel);
        propertiesLabel->setFrameShadow(QFrame::Raised);
        propertiesLabel->setAlignment(Qt::AlignCenter);

        verticalLayout->addWidget(propertiesLabel);

        propertiesTableView = new QTableView(menuWidget);
        propertiesTableView->setObjectName(QStringLiteral("propertiesTableView"));

        verticalLayout->addWidget(propertiesTableView);

        verticalLayout->setStretch(5, 3);
        verticalLayout->setStretch(7, 8);

        menuLayout->addWidget(menuWidget);

        showMenuButton = new QToolButton(horizontalLayoutWidget);
        showMenuButton->setObjectName(QStringLiteral("showMenuButton"));

        menuLayout->addWidget(showMenuButton);


        mainLayout->addLayout(menuLayout);

        worldViewLayout = new QHBoxLayout();
        worldViewLayout->setSpacing(6);
        worldViewLayout->setObjectName(QStringLiteral("worldViewLayout"));

        mainLayout->addLayout(worldViewLayout);

        buildObjectsLayout = new QHBoxLayout();
        buildObjectsLayout->setSpacing(6);
        buildObjectsLayout->setObjectName(QStringLiteral("buildObjectsLayout"));
        showBuildObjectsButton = new QToolButton(horizontalLayoutWidget);
        showBuildObjectsButton->setObjectName(QStringLiteral("showBuildObjectsButton"));

        buildObjectsLayout->addWidget(showBuildObjectsButton);

        buildToolsWidget = new QWidget(horizontalLayoutWidget);
        buildToolsWidget->setObjectName(QStringLiteral("buildToolsWidget"));
        buildToolsLayout = new QVBoxLayout(buildToolsWidget);
        buildToolsLayout->setSpacing(6);
        buildToolsLayout->setContentsMargins(11, 11, 11, 11);
        buildToolsLayout->setObjectName(QStringLiteral("buildToolsLayout"));
        buildToolsLabel = new QLabel(buildToolsWidget);
        buildToolsLabel->setObjectName(QStringLiteral("buildToolsLabel"));
        buildToolsLabel->setFrameShape(QFrame::Panel);
        buildToolsLabel->setFrameShadow(QFrame::Raised);
        buildToolsLabel->setAlignment(Qt::AlignCenter);

        buildToolsLayout->addWidget(buildToolsLabel);

        simulatorToolsMenu = new QWidget(buildToolsWidget);
        simulatorToolsMenu->setObjectName(QStringLiteral("simulatorToolsMenu"));
        worldObjectsMenu = new QHBoxLayout(simulatorToolsMenu);
        worldObjectsMenu->setSpacing(6);
        worldObjectsMenu->setContentsMargins(11, 11, 11, 11);
        worldObjectsMenu->setObjectName(QStringLiteral("worldObjectsMenu"));
        addObjectButton = new QToolButton(simulatorToolsMenu);
        addObjectButton->setObjectName(QStringLiteral("addObjectButton"));
        addObjectButton->setIcon(icon9);
        addObjectButton->setIconSize(QSize(32, 32));

        worldObjectsMenu->addWidget(addObjectButton);

        deleteObjectButton = new QToolButton(simulatorToolsMenu);
        deleteObjectButton->setObjectName(QStringLiteral("deleteObjectButton"));
        QIcon icon10;
        icon10.addFile(QStringLiteral(":/des/DeleteFileIcon"), QSize(), QIcon::Normal, QIcon::Off);
        deleteObjectButton->setIcon(icon10);
        deleteObjectButton->setIconSize(QSize(32, 32));

        worldObjectsMenu->addWidget(deleteObjectButton);


        buildToolsLayout->addWidget(simulatorToolsMenu);

        designerToolsMenu = new QWidget(buildToolsWidget);
        designerToolsMenu->setObjectName(QStringLiteral("designerToolsMenu"));
        designerToolsLayout = new QHBoxLayout(designerToolsMenu);
        designerToolsLayout->setSpacing(6);
        designerToolsLayout->setContentsMargins(11, 11, 11, 11);
        designerToolsLayout->setObjectName(QStringLiteral("designerToolsLayout"));
        deleteToolButton = new QToolButton(designerToolsMenu);
        deleteToolButton->setObjectName(QStringLiteral("deleteToolButton"));
        deleteToolButton->setIcon(icon10);
        deleteToolButton->setIconSize(QSize(32, 32));

        designerToolsLayout->addWidget(deleteToolButton);

        addToolButton = new QToolButton(designerToolsMenu);
        addToolButton->setObjectName(QStringLiteral("addToolButton"));
        addToolButton->setIcon(icon9);
        addToolButton->setIconSize(QSize(32, 32));

        designerToolsLayout->addWidget(addToolButton);

        exportObjectButton = new QToolButton(designerToolsMenu);
        exportObjectButton->setObjectName(QStringLiteral("exportObjectButton"));
        QIcon icon11;
        icon11.addFile(QStringLiteral(":/des/ExportFileIcon"), QSize(), QIcon::Normal, QIcon::Off);
        exportObjectButton->setIcon(icon11);
        exportObjectButton->setIconSize(QSize(32, 32));

        designerToolsLayout->addWidget(exportObjectButton);


        buildToolsLayout->addWidget(designerToolsMenu);

        buildToolsList = new QListWidget(buildToolsWidget);
        buildToolsList->setObjectName(QStringLiteral("buildToolsList"));

        buildToolsLayout->addWidget(buildToolsList);

        buildToolsLayout->setStretch(3, 1);

        buildObjectsLayout->addWidget(buildToolsWidget);


        mainLayout->addLayout(buildObjectsLayout);

        mainLayout->setStretch(1, 1);
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1540, 25));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0));
#ifndef QT_NO_TOOLTIP
        simulatorButton->setToolTip(QApplication::translate("MainWindow", "Simulation Mode", 0));
#endif // QT_NO_TOOLTIP
        simulatorButton->setText(QString());
#ifndef QT_NO_TOOLTIP
        designerButton->setToolTip(QApplication::translate("MainWindow", "Robot Mode", 0));
#endif // QT_NO_TOOLTIP
        designerButton->setText(QString());
        modeLabel->setText(QApplication::translate("MainWindow", "Simulator", 0));
#ifndef QT_NO_TOOLTIP
        joystickButton->setToolTip(QApplication::translate("MainWindow", "New Joystick", 0));
#endif // QT_NO_TOOLTIP
        joystickButton->setText(QApplication::translate("MainWindow", "...", 0));
#ifndef QT_NO_TOOLTIP
        saveSimButton->setToolTip(QApplication::translate("MainWindow", "Save Simulation", 0));
#endif // QT_NO_TOOLTIP
        saveSimButton->setText(QApplication::translate("MainWindow", "...", 0));
#ifndef QT_NO_TOOLTIP
        screenshotSimButton->setToolTip(QApplication::translate("MainWindow", "Take Screenshot", 0));
#endif // QT_NO_TOOLTIP
        screenshotSimButton->setText(QApplication::translate("MainWindow", "...", 0));
#ifndef QT_NO_TOOLTIP
        importMapButton->setToolTip(QApplication::translate("MainWindow", "Import Map", 0));
#endif // QT_NO_TOOLTIP
        importMapButton->setText(QApplication::translate("MainWindow", "Import Map", 0));
#ifndef QT_NO_TOOLTIP
        restartSimButton->setToolTip(QApplication::translate("MainWindow", "Restart Sim", 0));
#endif // QT_NO_TOOLTIP
        restartSimButton->setText(QApplication::translate("MainWindow", "...", 0));
        speedSimButton->setText(QString());
        playSimButton->setText(QString());
#ifndef QT_NO_TOOLTIP
        newObjectButton->setToolTip(QApplication::translate("MainWindow", "New Object", 0));
#endif // QT_NO_TOOLTIP
        newObjectButton->setText(QApplication::translate("MainWindow", "...", 0));
#ifndef QT_NO_TOOLTIP
        loadObjectButton->setToolTip(QApplication::translate("MainWindow", "Load Object", 0));
#endif // QT_NO_TOOLTIP
        loadObjectButton->setText(QString());
#ifndef QT_NO_TOOLTIP
        saveObjectButton->setToolTip(QApplication::translate("MainWindow", "Save Object", 0));
#endif // QT_NO_TOOLTIP
        saveObjectButton->setText(QString());
        activeObjectsLabel->setText(QApplication::translate("MainWindow", "Active Objects", 0));
        propertiesLabel->setText(QApplication::translate("MainWindow", "Simulator Object Properties", 0));
        showMenuButton->setText(QApplication::translate("MainWindow", "\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
".\n"
".\n"
".\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"", 0));
        showBuildObjectsButton->setText(QApplication::translate("MainWindow", "\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
".\n"
".\n"
".\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"", 0));
        buildToolsLabel->setText(QApplication::translate("MainWindow", "Designer Build Tools", 0));
#ifndef QT_NO_TOOLTIP
        addObjectButton->setToolTip(QApplication::translate("MainWindow", "Add Object", 0));
#endif // QT_NO_TOOLTIP
        addObjectButton->setText(QString());
#ifndef QT_NO_TOOLTIP
        deleteObjectButton->setToolTip(QApplication::translate("MainWindow", "Delete Object", 0));
#endif // QT_NO_TOOLTIP
        deleteObjectButton->setText(QString());
#ifndef QT_NO_TOOLTIP
        deleteToolButton->setToolTip(QApplication::translate("MainWindow", "Delete Tool", 0));
#endif // QT_NO_TOOLTIP
        deleteToolButton->setText(QString());
#ifndef QT_NO_TOOLTIP
        addToolButton->setToolTip(QApplication::translate("MainWindow", "Add Tool", 0));
#endif // QT_NO_TOOLTIP
        addToolButton->setText(QString());
#ifndef QT_NO_TOOLTIP
        exportObjectButton->setToolTip(QApplication::translate("MainWindow", "Export Object to Simulator", 0));
#endif // QT_NO_TOOLTIP
        exportObjectButton->setText(QApplication::translate("MainWindow", "...", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
