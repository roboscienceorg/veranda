#include "ui/mainwindow.h"
#include "ui_mainwindow.h"

#include <QMessageBox>
#include <QFileDialog>
#include <QDebug>
#include <QDate>
#include <QStandardItemModel>
#include <QThread>
#include <QListWidgetItem>
#include <stdexcept>
#include <string>

MainWindow::MainWindow(visualizerFactory factory, MapLoader_If *mapLoad, RobotLoader_If *robotLoad, QWidget *parent) :
    Simulator_Ui_If(parent),
    makeWidget(factory), mapLoader(mapLoad), robotLoader(robotLoad),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    speed = 1;
    play = false;

    //Initialize Widget Settings
    ui->buildToolsWidget->setVisible((false));
    ui->designerMenuWidget->setVisible(false);
    ui->playSimButton->setToolTip("Play Simulation");
    ui->speedSimButton->setToolTip("Speed x2");

    ui->simulatorButton->setEnabled(false);
    ui->modeLabel->setText("Map Mode");

    //Did Andrew do this? Setting Properties to display those of the map (?)
    visual = makeWidget();
    ui->worldViewLayout->addWidget(visual);
    ui->propertiesTableView->verticalHeader()->setVisible(false);
    connect(visual, SIGNAL(userSelectedObject(object_id)), this, SLOT(objectSelected(object_id)));
    connect(this, SIGNAL(objectIsSelected(object_id)), visual, SLOT(objectSelected(object_id)));

    //Main window button signals and slots
    connect(ui->showBuildObjectsButton, SIGNAL (released()), this, SLOT (showBuildObjectsButtonClick()));
    connect(ui->showMenuButton, SIGNAL (released()), this, SLOT (showMenuButtonClick()));
    connect(ui->simulatorButton, SIGNAL (released()), this, SLOT (simulatorButtonClick()));
    connect(ui->designerButton, SIGNAL (released()), this, SLOT (designerButtonClick()));

    //Simulation mode button signals and slots
    connect(ui->playSimButton, SIGNAL (released()), this, SLOT (playSimButtonClick()));
    connect(ui->speedSimButton, SIGNAL (released()), this, SLOT (speedSimButtonClick()));
    connect(ui->screenshotSimButton, SIGNAL (released()), this, SLOT (screenshotSimButtonClick()));

    //Simulation build tools widgets and slots
    connect(ui->importMapButton, SIGNAL (released()), this, SLOT (importMapButtonClick()));

    //Build tools list and world view slots
    propertiesModel = new QStandardItemModel(0,2,this); //12 Rows and 2 Columns
    propertiesModel->setHorizontalHeaderItem(0, new QStandardItem(QString("Property")));
    propertiesModel->setHorizontalHeaderItem(1, new QStandardItem(QString("Value")));
    ui->propertiesTableView->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    ui->propertiesTableView->setModel(propertiesModel);
    connect(ui->robotsWidget, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(robotItemClicked(QListWidgetItem*)));
    connect(visual, SIGNAL (userSelectedModel(model_id id)), this, SLOT (modelSelected(model_id id)));
}

MainWindow::~MainWindow()
{
    delete ui;
    delete mapLoader;
    delete robotLoader;
}

//Menu Mode Button Clicks
void MainWindow::simulatorButtonClick()
{
    ui->modeLabel->setText("Simulator");
    ui->propertiesLabel->setText("Simulation Object Properties");
    ui->buildToolsLabel->setText("Simulation Build Tools");

    ui->simulatorMenuWidget->setVisible(true);
    ui->designerMenuWidget->setVisible(false);

    //Enable/Disable Mode Buttons
    ui->designerButton->setEnabled(true);
    ui->simulatorButton->setEnabled(false);
}
void MainWindow::designerButtonClick()
{
    ui->modeLabel->setText("Designer");
    ui->propertiesLabel->setText("Designer Object Properties");
    ui->buildToolsLabel->setText("Designer Build Tools");

    ui->simulatorMenuWidget->setVisible(false);
    ui->designerMenuWidget->setVisible(true);

    //Enable/Disable Mode Buttons
    ui->simulatorButton->setEnabled(true);
    ui->designerButton->setEnabled(false);
}

void MainWindow::physicsStarted()
{
    play = true;
    ui->playSimButton->setToolTip("Stop Simulation");
    ui->playSimButton->setIcon(QIcon(":/sim/StopSimIcon"));
    ui->playSimButton->setIconSize(QSize(32,32));

    //disable options while simulation is running
    ui->saveSimButton->setEnabled(false);
    ui->designerButton->setEnabled(false);
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
    ui->designerButton->setEnabled(true);
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
          QString path = QFileDialog::getOpenFileName(this, tr("Open File"), "/home", tr("Json files (*.json);;Images (*.png *.jpg)"));

          if(path.length())
          {
              Map* map = mapLoader->loadMapFile(path);
              if(map)
              {
                  //Clear out simulation
                  while(worldObjects.size())
                      userRemoveWorldObjectFromSimulation(worldObjects.firstKey());

                  //Add map into new simulation
                  userAddWorldObjectToSimulation(map);
              }
          }

          break;
    }
        break;

      case QMessageBox::No:
          // Don't Save was clicked
          break;
      default:
          // should never be reached
          break;
    }
}

void MainWindow::nothingSelected()
{
    propertiesModel->setRowCount(0);
    disconnect(propertiesModel, &QStandardItemModel::dataChanged, 0, 0);
    nothingIsSelected();
}

void MainWindow::objectSelected(object_id id)
{
    if(worldObjects.contains(id))
    {
        nothingSelected();

        selected = id;
        WorldObjectProperties_If* obj = worldObjects[id];
        QStandardItemModel* model = propertiesModel;

        QMap<QString, PropertyView>& objProps = obj->getAllProperties();
        model->setRowCount(objProps.size());

        int i = 0;
        for(auto iter = objProps.begin(); iter != objProps.end(); iter++, i++)
        {
           QModelIndex ind;

           //Set key
           ind = model->index(i, 0);
           model->setData(ind, iter.key());

           //Init value
           ind = model->index(i, 1);
           model->setData(ind, iter.value().get());

           //Update value when it changes
           connect(&iter.value(), &PropertyView::valueSet, [i, model, ind](QVariant v)
           {
               model->setData(ind, v);
           });

           displayed_properties[i] = iter.key();
        }

        connect(model, &QStandardItemModel::dataChanged, [this, obj, model](QModelIndex tl, QModelIndex br)
        {
           for(int i = tl.row(); i <= br.row(); i++)
               obj->getAllProperties()[displayed_properties[i]].set(model->data(model->index(i, 1)));
        });

        objectIsSelected(id);
    }
}

void MainWindow::setWorldBounds(double xMin, double xMax, double yMin, double yMax)
{
    if(xMin > xMax) std::swap(xMin, xMax);
    if(yMin > yMax) std::swap(yMin, yMax);

    qDebug() << "Adjusting world size" << xMin << yMin << "->" << xMax << yMax;
    visual->setWorldBounds(xMin, xMax, yMin, yMax);

    qDebug() << "Populating default robots...";
    for(int i=0; i<3; i++)
    {
        Robot* r = robotLoader->loadRobotFile("");
        if(r)
        {
            r->setOrientation(rand() % int(xMax - xMin) + xMin, rand() % int(yMax - yMin) + yMin, rand()%360);
            r->getAllProperties()["Diff Drive/channels/input_velocities"].set("robot0/wheel_velocities");
            r->getAllProperties()["Diff Drive/axle_length"].set(1);
            r->getAllProperties()["Diff Drive/wheel_radius"].set(0.2);

            r->getAllProperties()["Touch Ring/channels/output_touches"].set("robot0/touch_ring");
            r->getAllProperties()["Touch Ring/sensor_count"].set(20);
            r->getAllProperties()["Touch Ring/angle_end"].set(360);
            r->getAllProperties()["Touch Ring/ring_radius"].set(0.5);
            userAddWorldObjectToSimulation(r);
            delete r;
        }
    }
}

//Add robot to the simulation world view
void MainWindow::worldObjectAddedToSimulation(WorldObjectProperties_If* object, object_id oId)
{
    if(worldObjects.contains(oId)) throw std::logic_error("world object " + std::to_string(oId) + " already exists in ui");

    worldObjects[oId] = object;

    visual->objectAddedToScreen(object->getModels(), oId);

    objectSelected(oId);
    drawActiveObjectsList();
}

void MainWindow::worldObjectRemovedFromSimulation(object_id oId)
{
    visual->objectRemovedFromScreen(oId);
    worldObjects.remove(oId);
    if(selected == oId)
        nothingSelected();
    drawActiveObjectsList();
}

void MainWindow::drawActiveObjectsList()
{
    ui->robotsWidget->clear();

    for(int i = 0; i < worldObjects.size(); i++)
    {
        QString str= QString::number(i);
        ui->robotsWidget->addItem(str);
    }
}

void MainWindow::robotItemClicked(QListWidgetItem* item)
{
    for(int kl = 1; kl < sizeof(worldObjects) - 1; kl++)
    {
        if (ui->robotsWidget->item(kl) == item)
        {
            objectSelected(kl);
        }
    }
}
void MainWindow::listBuildTools(int mode)
{
    //for each file in folder
        //put name of build object? can I add an icon for each build object? would be useful
}
