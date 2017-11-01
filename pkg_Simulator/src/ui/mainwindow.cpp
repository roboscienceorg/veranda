#include "ui/mainwindow.h"
#include "ui_mainwindow.h"

#include <QMessageBox>
#include <QFileDialog>
#include <QDebug>
#include <QDate>
#include <QStandardItemModel>

MainWindow::MainWindow(visualizerFactory factory, QWidget *parent) :
    Simulator_Ui_If(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    makeWidget = factory;

    speed = 1;
    modelNum = 0;
    play = false;

    //Initialize Widget Settings
    ui->buildToolsWidget->setVisible((false));
    ui->simModeMenuWidget->setVisible(false);
    ui->robotModeMenuWidget->setVisible(false);
    ui->playSimButton->setToolTip("Play Simulation");
    ui->speedSimButton->setToolTip("Speed x2");

    ui->mapModeButton->setEnabled(false);
    ui->modeLabel->setText("Map Mode");

    visual = makeWidget();
    ui->worldViewLayout->addWidget(visual);
    ui->propertiesTableView->verticalHeader()->setVisible(false);

    //Main window button signals and slots
    connect(ui->showBuildObjectsButton, SIGNAL (released()), this, SLOT (showBuildObjectsButtonClick()));
    connect(ui->showMenuButton, SIGNAL (released()), this, SLOT (showMenuButtonClick()));
    connect(ui->simModeButton, SIGNAL (released()), this, SLOT (simModeButtonClick()));
    connect(ui->robotModeButton, SIGNAL (released()), this, SLOT (robotModeButtonClick()));
    connect(ui->mapModeButton, SIGNAL (released()), this, SLOT (mapModeButtonClick()));

    //Simulation mode button signals and slots
    connect(ui->playSimButton, SIGNAL (released()), this, SLOT (playSimButtonClick()));
    connect(ui->speedSimButton, SIGNAL (released()), this, SLOT (speedSimButtonClick()));
    connect(ui->screenshotSimButton, SIGNAL (released()), this, SLOT (screenshotSimButtonClick()));

    //Simulation build tools widgets and slots
    connect(ui->importMapButton, SIGNAL (released()), this, SLOT (importMapButtonClick()));

    //Build tools list and world view slots
    connect(visual, SIGNAL (userSelectedModel(model_id id)), this, SLOT (modelSelected(model_id id)));
}

MainWindow::~MainWindow()
{
    delete ui;
}

//Menu Mode Button Clicks
void MainWindow::simModeButtonClick()
{
    ui->modeLabel->setText("Simulation Mode");
    ui->propertiesLabel->setText("Simulation Properties");
    ui->buildToolsLabel->setText("Simulation Build Tools");

    ui->simModeMenuWidget->setVisible(true);
    ui->mapModeMenuWidget->setVisible(false);
    ui->robotModeMenuWidget->setVisible(false);

    //Enable/Disable Mode Buttons
    ui->robotModeButton->setEnabled(true);
    ui->mapModeButton->setEnabled(true);
    ui->simModeButton->setEnabled(false);
}
void MainWindow::mapModeButtonClick()
{
    ui->modeLabel->setText("Map Mode");
    ui->propertiesLabel->setText("Map Properties");
    ui->buildToolsLabel->setText("Map Build Tools");

    ui->simModeMenuWidget->setVisible(false);
    ui->mapModeMenuWidget->setVisible(true);
    ui->robotModeMenuWidget->setVisible(false);

    //Enable/Disable Mode Buttons
    ui->robotModeButton->setEnabled(true);
    ui->simModeButton->setEnabled(true);
    ui->mapModeButton->setEnabled(false);
}
void MainWindow::robotModeButtonClick()
{
    ui->modeLabel->setText("Robot Mode");
    ui->propertiesLabel->setText("Robot Properties");
    ui->buildToolsLabel->setText("Robot Build Tools");

    ui->simModeMenuWidget->setVisible(false);
    ui->mapModeMenuWidget->setVisible(false);
    ui->robotModeMenuWidget->setVisible(true);

    //Enable/Disable Mode Buttons
    ui->simModeButton->setEnabled(true);
    ui->mapModeButton->setEnabled(true);
    ui->robotModeButton->setEnabled(false);
}

void MainWindow::physicsStarted()
{
    play = true;
    ui->playSimButton->setToolTip("Stop Simulation");
    ui->playSimButton->setIcon(QIcon(":/sim/StopSimIcon"));
    ui->playSimButton->setIconSize(QSize(32,32));

    //disable options while simulation is running
    ui->saveSimButton->setEnabled(false);
    ui->mapModeButton->setEnabled(false);
    ui->robotModeButton->setEnabled(false);
    ui->buildToolsList->setEnabled(false);
}

void MainWindow::physicsStopped()
{
    play = false;
    ui->playSimButton->setToolTip("Play Simulation");
    ui->playSimButton->setIcon(QIcon(":/sim/PlaySimIcon"));
    ui->playSimButton->setIconSize(QSize(32,32));

    //enable options while simulation is running
    ui->saveSimButton->setEnabled(true);
    ui->mapModeButton->setEnabled(true);
    ui->robotModeButton->setEnabled(true);
    ui->buildToolsList->setEnabled(true);
}

//Simulation Mode Button Clicks
void MainWindow::playSimButtonClick()
{
    //Reset Simulation Now
    if (play)
    {
        emit userStopPhysics();
    }
    //Play Simulation Now
    else
    {
        emit userStartPhysics();
    }
}
void MainWindow::speedSimButtonClick()
{
    if (speed == 1)
    {
        speed = 2;
        ui->speedSimButton->setToolTip("Speed x3");
        ui->speedSimButton->setIcon(QIcon(":/sim/SpeedTwoSimIcon"));
    }
    else if (speed == 2)
    {
        speed = 3;
        ui->speedSimButton->setToolTip("Speed 1/2");
        ui->speedSimButton->setIcon(QIcon(":/sim/SpeedThreeSimIcon"));
    }
    else if (speed == 3)
    {
        speed = 0;
        ui->speedSimButton->setToolTip("Speed x1");
        ui->speedSimButton->setIcon(QIcon(":/sim/SpeedHalfSimIcon"));
    }
    else if (speed == 0)
    {
        speed = 1;
        ui->speedSimButton->setToolTip("Speed x2");
        ui->speedSimButton->setIcon(QIcon(":/sim/SpeedOneSimIcon"));
    }
    ui->speedSimButton->setIconSize(QSize(32,32));
}
void MainWindow::screenshotSimButtonClick()
{
    visual->setStyleSheet("border: 2 solid red");
    QPixmap pixmap(visual->size());
    visual->render(&pixmap);
    qint64 current = QDateTime::currentMSecsSinceEpoch();
    //QString fileName = current.toString();

    //std::stringstream ss;
    //ss << current;
    QString fileName = QString::number(current);

    QFile file(fileName);
    file.open(QIODevice::WriteOnly);
    pixmap.save(&file, "PNG");
    visual->setStyleSheet("");
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
void MainWindow::importMapButtonClick()
{
    QMessageBox msgBox;
    msgBox.setText("WARNING: Changing the map will delete all robots from this simulation.");
    msgBox.setInformativeText("Would you like to proceed?");
    msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
    msgBox.setDefaultButton(QMessageBox::No);
    int ret = msgBox.exec();

    switch (ret) {
      case QMessageBox::Yes:
      {
          // Save was clicked
          QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"),"/home", tr("Images (*.png *.xpm *.jpg);;Map (*.map)"));
          break;
      }
      case QMessageBox::No:
          // Don't Save was clicked
          break;
      default:
          // should never be reached
          break;
    }
}

//World View Slots
void MainWindow::modelSelected(model_id id)
{
    try
    {
        selected = id;
    }
    catch (std::exception & e)
    {
      // do something with what...
    }
    catch (...)
    {
      // someone threw something undecypherable
    }

    Robot_Properties* robot = models[selected];
    QStandardItemModel* model;

    model = new QStandardItemModel(robot->getAllProperties().size(),2,this); //N Rows and 2 Columns
    model->setHorizontalHeaderItem(0, new QStandardItem(QString("Property")));
    model->setHorizontalHeaderItem(1, new QStandardItem(QString("Value")));
    ui->propertiesTableView->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    ui->propertiesTableView->setModel(model);

    int i = 0;
    for(auto iter = robot->getAllProperties().begin(); iter != robot->getAllProperties().end(); iter++, i++)
    {
       QModelIndex ind = model->index(i, 0);
       model->setData(ind, iter.key(), Qt::DisplayRole);
       ind = model->index(i, 1);
       bool readOnly = iter.value().info().readOnly;
       qDebug() << iter.key() << iter.value().get();
       model->setData(ind, iter.value().get(), readOnly ? Qt::DisplayRole : Qt::EditRole);
       connect(&iter.value(), &PropertyView::valueSet, [i, model, ind, readOnly](QVariant v)
       {
           qDebug () << "Set model data " << v;
           model->setData(ind, v, readOnly ? Qt::DisplayRole : Qt::EditRole);
       });
    }
}

//Add robot to the simulation world view
void MainWindow::robotAddedToSimulation(Robot_Properties* robot)
{
    models[modelNum] = robot;
    visual->modelAddedToScreen(robot->createRobotBaseModel(), modelNum++);
    models[modelNum] = robot;
    visual->modelAddedToScreen(robot->createRobotSensorsModel(), modelNum++);

    //take out from here to end of function after update to get clickable world view objects
    selected = modelNum-1;
    Robot_Properties* robot2 = models[selected];
    QStandardItemModel* model;

    model = new QStandardItemModel(robot2->getAllProperties().size(),2,this); //N Rows and 2 Columns
    model->setHorizontalHeaderItem(0, new QStandardItem(QString("Property")));
    model->setHorizontalHeaderItem(1, new QStandardItem(QString("Value")));
    ui->propertiesTableView->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    ui->propertiesTableView->setModel(model);

    int i = 0;
    for(auto iter = robot2->getAllProperties().begin(); iter != robot2->getAllProperties().end(); iter++, i++)
    {
       QModelIndex ind = model->index(i, 0);
       model->setData(ind, iter.key(), Qt::DisplayRole);
       ind = model->index(i, 1);
       bool readOnly = iter.value().info().readOnly;
       qDebug() << iter.key() << iter.value().get();
       model->setData(ind, iter.value().get(), readOnly ? Qt::DisplayRole : Qt::EditRole);
       connect(&iter.value(), &PropertyView::valueSet, [i, model, ind, readOnly](QVariant v)
       {
           qDebug () << "Set model data " << v;
           model->setData(ind, v, readOnly ? Qt::DisplayRole : Qt::EditRole);
       });
    }
    //yup, all the way to here - delete
}

void MainWindow::listBuildTools(int mode)
{
    //for(auto iter = robot->getAllProperties().begin(); iter != robot->getAllProperties().end(); iter++)
        //connect(&iter.value(), &PropertyView::valueSet, [](QVariant v){qDebug() << v;});
}
