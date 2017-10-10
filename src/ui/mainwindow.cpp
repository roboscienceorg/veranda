#include "ui/mainwindow.h"
#include "ui_mainwindow.h"

#include <QMessageBox>

MainWindow::MainWindow(visualizerFactory factory, QWidget *parent) :
    Simulator_Ui_If(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    _makeWidget = factory;

    speed = 1;
    play = false;

    //Initialize Widget Settings
    ui->buildToolsWidget->setVisible((false));
    ui->simModeMenuWidget->setVisible(false);
    ui->chooseMapButton->setVisible(false);
    ui->robotModeMenuWidget->setVisible(false);
    ui->playSimButton->setToolTip("Play Simulation");
    ui->speedSimButton->setToolTip("Speed x2");

    ui->mapModeButton->setEnabled(false);
    ui->modeLabel->setText("Map Mode");

    //Main window button signals and slots
    connect(ui->showBuildObjectsButton, SIGNAL (released()), this, SLOT (showBuildObjectsButtonClick()));
    connect(ui->showMenuButton, SIGNAL (released()), this, SLOT (showMenuButtonClick()));
    connect(ui->simModeButton, SIGNAL (released()), this, SLOT (simModeButtonClick()));
    connect(ui->robotModeButton, SIGNAL (released()), this, SLOT (robotModeButtonClick()));
    connect(ui->mapModeButton, SIGNAL (released()), this, SLOT (mapModeButtonClick()));

    //Simulation mode button signals and slots
    connect(ui->playSimButton, SIGNAL (released()), this, SLOT (playSimButtonClick()));
    connect(ui->speedSimButton, SIGNAL (released()), this, SLOT (speedSimButtonClick()));
    connect(ui->recordSimButton, SIGNAL (released()), this, SLOT (recordSimButtonClick()));

    //Simulation build tools widgets and slots
    connect(ui->chooseMapButton, SIGNAL (released()), this, SLOT (chooseMapButtonClick()));
}

MainWindow::~MainWindow()
{
    delete ui;
}

//Menu Mode Button Clicks
void MainWindow::simModeButtonClick()
{
    ui->modeLabel->setText("Simulation Mode");
    ui->availabilityLabel->setText("Available Simulations");
    ui->buildToolsLabel->setText("Simulation Build Tools");

    ui->simModeMenuWidget->setVisible(true);
    ui->mapModeMenuWidget->setVisible(false);
    ui->robotModeMenuWidget->setVisible(false);
    ui->chooseMapButton->setVisible(true);

    //Enable/Disable Mode Buttons
    ui->robotModeButton->setEnabled(true);
    ui->mapModeButton->setEnabled(true);
    ui->simModeButton->setEnabled(false);
}
void MainWindow::mapModeButtonClick()
{
    ui->modeLabel->setText("Map Mode");
    ui->availabilityLabel->setText("Available Maps");
    ui->buildToolsLabel->setText("Map Build Tools");

    ui->simModeMenuWidget->setVisible(false);
    ui->mapModeMenuWidget->setVisible(true);
    ui->robotModeMenuWidget->setVisible(false);
    ui->chooseMapButton->setVisible(false);

    //Enable/Disable Mode Buttons
    ui->robotModeButton->setEnabled(true);
    ui->simModeButton->setEnabled(true);
    ui->mapModeButton->setEnabled(false);
}
void MainWindow::robotModeButtonClick()
{
    ui->modeLabel->setText("Robot Mode");
    ui->availabilityLabel->setText("Available Robots");
    ui->buildToolsLabel->setText("Robot Build Tools");

    ui->simModeMenuWidget->setVisible(false);
    ui->mapModeMenuWidget->setVisible(false);
    ui->robotModeMenuWidget->setVisible(true);
    ui->chooseMapButton->setVisible(false);

    //Enable/Disable Mode Buttons
    ui->simModeButton->setEnabled(true);
    ui->mapModeButton->setEnabled(true);
    ui->robotModeButton->setEnabled(false);
}

//Simulation Mode Button Clicks
void MainWindow::playSimButtonClick()
{
    //Reset Simulation Now
    if (play)
    {
        play = false;
        ui->playSimButton->setToolTip("Play Simulation");
        ui->playSimButton->setIcon(QIcon(":/simModeMenu/PlayIcon.png"));

        //enable options while simulation is running
        ui->newSimButton->setEnabled(true);
        ui->copySimButton->setEnabled(true);
        ui->saveSimButton->setEnabled(true);
        ui->deleteSimButton->setEnabled(true);
        ui->mapModeButton->setEnabled(true);
        ui->robotModeButton->setEnabled(true);
        ui->buildToolsList->setEnabled(true);
    }
    //Play Simulation Now
    else
    {
        play = true;
        ui->playSimButton->setToolTip("Stop Simulation");
        ui->playSimButton->setIcon(QIcon(":/simModeMenu/StopIcon.png"));

        //disable options while simulation is running
        ui->newSimButton->setEnabled(false);
        ui->copySimButton->setEnabled(false);
        ui->saveSimButton->setEnabled(false);
        ui->deleteSimButton->setEnabled(false);
        ui->mapModeButton->setEnabled(false);
        ui->robotModeButton->setEnabled(false);
        ui->buildToolsList->setEnabled(false);
    }
    ui->playSimButton->setIconSize(QSize(32,32));
}
void MainWindow::speedSimButtonClick()
{
    if (speed == 1)
    {
        speed = 2;
        ui->speedSimButton->setToolTip("Speed x3");
        ui->speedSimButton->setIcon(QIcon(":/simModeMenu/SpeedTwoIcon.png"));
    }
    else if (speed == 2)
    {
        speed = 3;
        ui->speedSimButton->setToolTip("Speed 1/2");
        ui->speedSimButton->setIcon(QIcon(":/simModeMenu/SpeedThreeIcon.png"));
    }
    else if (speed == 3)
    {
        speed = 0;
        ui->speedSimButton->setToolTip("Speed x1");
        ui->speedSimButton->setIcon(QIcon(":/simModeMenu/SpeedHalfIcon.png"));
    }
    else if (speed == 0)
    {
        speed = 1;
        ui->speedSimButton->setToolTip("Speed x2");
        ui->speedSimButton->setIcon(QIcon(":/simModeMenu/SpeedOneIcon.png"));
    }
    ui->speedSimButton->setIconSize(QSize(32,32));
}
void MainWindow::recordSimButtonClick()
{
    //Reset Simulation Now
    if (record)
    {
        record = false;
        ui->recordSimButton->setToolTip("Record Simulation");
        ui->recordSimButton->setIcon(QIcon(":/simModeMenu/RecordIcon.png"));
    }
    //Play Simulation Now
    else
    {
        record = true;
        ui->recordSimButton->setToolTip("Dont Record Simulation");
        ui->recordSimButton->setIcon(QIcon(":/simModeMenu/DontRecordIcon.png"));
    }
    ui->recordSimButton->setIconSize(QSize(32,32));
}

//Show Menu Button Clicks
void MainWindow::showBuildObjectsButtonClick()
{
    if (ui->buildToolsWidget->isHidden())
        ui->buildToolsWidget->setVisible(true);
    else
        ui->buildToolsWidget->setVisible((false));
}
void MainWindow::showMenuButtonClick()
{
    if (ui->menuWidget->isHidden())
        ui->menuWidget->setVisible(true);
    else
        ui->menuWidget->setVisible((false));
}

//Simulation Build Tools Button Clicks
void MainWindow::chooseMapButtonClick()
{
    QMessageBox msgBox;
    msgBox.setText("WARNING: Changing the map will delete all robots from this simulation.");
    msgBox.setInformativeText("Would you like to proceed?");
    msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
    msgBox.setDefaultButton(QMessageBox::No);
    int ret = msgBox.exec();

    switch (ret) {
      case QMessageBox::Yes:
          // Save was clicked
          break;
      case QMessageBox::No:
          // Don't Save was clicked
          break;
      default:
          // should never be reached
          break;
    }
}
