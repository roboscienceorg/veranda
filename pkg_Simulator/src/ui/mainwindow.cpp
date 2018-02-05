#include "ui/mainwindow.h"
#include "ui_mainwindow.h"

#include <QMessageBox>
#include <QFileDialog>
#include <QDebug>
#include <QDate>
#include <QStandardItemModel>
#include <QThread>
#include <QListWidgetItem>
#include <QWindow>
#include <stdexcept>
#include <string>

MainWindow::MainWindow(visualizerFactory factory, QMap<QString, WorldObjectComponent_Plugin_If *> components,
                       QVector<WorldObjectLoader_If*> loaders, QVector<WorldObjectSaver_If*> savers, QWidget *parent) :
    Simulator_Ui_If(parent),
    makeWidget(factory), componentPlugins(components),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->centralWidget->setLayout(ui->mainLayout);

    for(WorldObjectLoader_If* l : loaders)
        for(QString e : l->fileExts())
            objectLoaders[e] = l;

    for(WorldObjectSaver_If* s : savers)
        for(QString e : s->fileExts())
            objectSavers[e] = s;

    speed = 1;
    play = false;

    //Initialize Widget Settings
    ui->playSimButton->setToolTip("Play Simulation");
    ui->speedSimButton->setToolTip("Speed x2");
    ui->buildToolsWidget->setVisible((false));
    ui->designerMenuWidget->setVisible(false);
    ui->designerToolsMenu->setVisible(false);
    ui->simulatorButton->setEnabled(false);
    ui->designerToolsList->setVisible(false);
    ui->designerActiveWidget->setVisible(false);

    //Initiate a world view for Simulator and another for Designer
    visualSimulator = makeWidget();
    visualDesigner = makeWidget();
    ui->worldViewLayout->addWidget(visualSimulator);
    ui->worldViewLayout->addWidget(visualDesigner);
    visualDesigner->setVisible(false);

    ui->propertiesTableView->verticalHeader()->setVisible(false);
    connect(visualSimulator, SIGNAL(userSelectedObject(object_id)), this, SLOT(objectSelected(object_id)));
    //connect(visualDesigner, SIGNAL(userSelectedTool(tool_id)), this, SLOT(toolSelected(t_id)));
    connect(this, SIGNAL(objectIsSelected(object_id)), visualSimulator, SLOT(objectSelected(object_id)));
    connect(this, SIGNAL(nothingIsSelected()), visualSimulator, SLOT(nothingSelected()));

    //Main menu button signals and slots
    connect(ui->showBuildObjectsButton, SIGNAL (released()), this, SLOT (showBuildObjectsButtonClick()));
    connect(ui->showMenuButton, SIGNAL (released()), this, SLOT (showMenuButtonClick()));
    connect(ui->simulatorButton, SIGNAL (released()), this, SLOT (simulatorButtonClick()));
    connect(ui->designerButton, SIGNAL (released()), this, SLOT (designerButtonClick()));

    //Simulation mode button signals and slots
    connect(ui->playSimButton, SIGNAL (released()), this, SLOT (playSimButtonClick()));
    connect(ui->speedSimButton, SIGNAL (released()), this, SLOT (speedSimButtonClick()));
    connect(ui->importMapButton, SIGNAL (released()), this, SLOT (importMapButtonClick()));
    connect(ui->screenshotSimButton, SIGNAL (released()), this, SLOT (screenshotSimButtonClick()));
    connect(ui->joystickButton, SIGNAL (released()), this, SLOT (joystickButtonClick()));
    connect(ui->saveSimButton, SIGNAL (released()), this, SLOT (saveSimButtonClick()));
    connect(ui->restartSimButton, SIGNAL (released()), this, SLOT (restartSimButtonClick()));

    //Designer mode button signals and slots
    connect(ui->newObjectButton, SIGNAL (released()), this, SLOT (newObjectButtonClick()));
    connect(ui->loadObjectButton, SIGNAL (released()), this, SLOT (loadObjectButtonClick()));
    connect(ui->saveObjectButton, SIGNAL (released()), this, SLOT (saveObjectButtonClick()));

    //Simulation mode tool button signals and slots
    connect(ui->addObjectButton, SIGNAL (released()), this, SLOT (addObjectButtonClick()));
    connect(ui->deleteObjectButton, SIGNAL (released()), this, SLOT (deleteObjectButtonClick()));
    //connect(this, SIGNAL(addObjectToWorld()), visualSimulator, SLOT(addObjectButtonClick()));
    //connect(this, SIGNAL(removeObjectFromWorld()), visualDesigner, SLOT(deleteObjectButtonClick()));

    //Designer mode tool button signals and slots
    connect(ui->addToolButton, SIGNAL (released()), this, SLOT (addToolButtonClick()));
    connect(ui->deleteToolButton, SIGNAL (released()), this, SLOT (deleteToolButtonClick()));
    connect(ui->exportObjectButton, SIGNAL (released()), this, SLOT (exportObjectButtonClick()));
    //connect(this, SIGNAL(addToolToWorld()), visualDesigner, SLOT(addToolButtonClick()));
    //connect(this, SIGNAL(deleteToolFromWorld()), visualDesigner, SLOT(deleteToolButtonClick()));
    //connect(this, SIGNAL(exportObjectToWorld()), visualDesigner, SLOT(exportObjectButtonClick()));

    //World view signals and slots
    propertiesModel = new QStandardItemModel(0,2,this); //12 Rows and 2 Columns
    propertiesModel->setHorizontalHeaderItem(0, new QStandardItem(QString("Property")));
    propertiesModel->setHorizontalHeaderItem(1, new QStandardItem(QString("Value")));
    ui->propertiesTableView->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    ui->propertiesTableView->setModel(propertiesModel);
    connect(ui->simulatorActiveWidget, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(robotItemClicked(QListWidgetItem*)));
}

MainWindow::~MainWindow()
{
    delete ui;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Public signals and slots                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Add robot to the simulation world view
void MainWindow::worldObjectAddedToSimulation(WorldObjectProperties *object, object_id oId)
{
    if(worldObjects.contains(oId)) throw std::logic_error("world object " + std::to_string(oId) + " already exists in ui");

    worldObjects[oId] = object;

    visualSimulator->objectAddedToScreen(object->getModels(), oId);

    listItems[oId] = new QListWidgetItem();
    listItems[oId]->setData(Qt::DisplayRole, QString::number(oId));
    ui->simulatorActiveWidget->addItem(listItems[oId]);

    objectSelected(oId);
}

void MainWindow::worldObjectRemovedFromSimulation(object_id oId)
{
    visualSimulator->objectRemovedFromScreen(oId);
    worldObjects.remove(oId);

    ui->simulatorActiveWidget->removeItemWidget(listItems[oId]);
    delete listItems[oId];
    listItems.remove(oId);

    if(selected == oId)
        nothingSelected();
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
    ui->simulatorToolsList->setEnabled(false);
    ui->simulatorToolsMenu->setEnabled(false);
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
    ui->simulatorToolsList->setEnabled(true);
    ui->simulatorToolsMenu->setEnabled(true);
}

void MainWindow::setWorldBounds(double xMin, double xMax, double yMin, double yMax)
{
    if(xMin > xMax) std::swap(xMin, xMax);
    if(yMin > yMax) std::swap(yMin, yMax);

    qDebug() << "Adjusting world size" << xMin << yMin << "->" << xMax << yMax;
    visualSimulator->setWorldBounds(xMin, xMax, yMin, yMax);
    visualDesigner->setWorldBounds(xMin, xMax, yMin, yMax);

   // qDebug() << "Populating default robots...";

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main menu button signals and slots                                                                                        //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

void MainWindow::simulatorButtonClick()
{
    ui->modeLabel->setText("Simulator");
    ui->buildToolsLabel->setText("Simulation Build Tools");

    ui->simulatorMenuWidget->setVisible(true);
    ui->designerMenuWidget->setVisible(false);
    ui->simulatorToolsMenu->setVisible(true);
    ui->designerToolsMenu->setVisible(false);
    visualDesigner->setVisible(false);
    visualSimulator->setVisible(true);
    ui->designerToolsList->setVisible(false);
    ui->simulatorToolsList->setVisible(true);
    ui->simulatorActiveWidget->setVisible(true);
    ui->designerActiveWidget->setVisible(false);
    nothingSelected();

    //Enable/Disable Mode Buttons
    ui->designerButton->setEnabled(true);
    ui->simulatorButton->setEnabled(false);
}

void MainWindow::designerButtonClick()
{
    ui->modeLabel->setText("Designer");
    ui->buildToolsLabel->setText("Designer Build Tools");

    ui->simulatorMenuWidget->setVisible(false);
    ui->designerMenuWidget->setVisible(true);
    ui->simulatorToolsMenu->setVisible(false);
    ui->designerToolsMenu->setVisible(true);
    visualDesigner->setVisible(true);
    visualSimulator->setVisible(false);
    ui->designerToolsList->setVisible(true);
    ui->simulatorToolsList->setVisible(false);
    ui->simulatorActiveWidget->setVisible(false);
    ui->designerActiveWidget->setVisible(true);
    nothingSelected();

    //Enable/Disable Mode Buttons
    ui->simulatorButton->setEnabled(true);
    ui->designerButton->setEnabled(false);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Simulation mode button signals and slots                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
    visualSimulator->setStyleSheet("border: 2 solid red");
    QPixmap pixmap(visualSimulator->size());
    visualSimulator->render(&pixmap);
    qint64 current = QDateTime::currentMSecsSinceEpoch();
    QString fileName = QString::number(current);

    QFile file(fileName);
    file.open(QIODevice::WriteOnly);
    pixmap.save(&file, "PNG");
    visualSimulator->setStyleSheet("");
}

void MainWindow::importMapButtonClick()
{
    QMessageBox msgBox;
    msgBox.setText("WARNING: Changing the map will delete all world objects from this simulation.");
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
              /*Map* map = mapLoader->loadMapFile(path);
              if(map)
              {
                  //Clear out simulation
                  while(worldObjects.size())
                      userRemoveWorldObjectFromSimulation(worldObjects.firstKey());

                  //Add map into new simulation
                  userAddWorldObjectToSimulation(map);
                  delete map;
              }*/
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

void MainWindow::joystickButtonClick()
{
    //create window and joystick, link them
    QWindow *jWindow = new QWindow();
    JoystickPrototype *joystick = new JoystickPrototype(jWindow);

    //Joystick button signals and slots
    connect(joystick, SIGNAL(joystickMoved(double, double, double, QString)), visualSimulator, SLOT(joystickMoved(double, double, double, QString)));
    connect(joystick, SIGNAL(joystickButtonPress(int, QString)), visualSimulator, SLOT(joystickButtonPress(int, QString)));
    connect(joystick, SIGNAL(joystickButtonRelease(int, QString)), visualSimulator, SLOT(joystickButtonRelease(int, QString)));
    connect(joystick, &JoystickPrototype::joystickClosed, joystick, &JoystickPrototype::deleteLater);
    connect(joystick, &JoystickPrototype::destroyed, jWindow, &QWindow::deleteLater);

    //show this joystick
    joystick->show();
}

void MainWindow::saveSimButtonClick(){}
void MainWindow::restartSimButtonClick(){}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Designer mode button signals and slots                                                                                    //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::newObjectButtonClick(){}
void MainWindow::loadObjectButtonClick(){}
void MainWindow::saveObjectButtonClick(){}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Simulation mode tool button signals and slots                                                                             //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::addObjectButtonClick(){}
void MainWindow::deleteObjectButtonClick(){}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Designer mode tool button signals and slots                                                                               //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MainWindow::addToolButtonClick(){}
void MainWindow::deleteToolButtonClick(){}
void MainWindow::exportObjectButtonClick(){}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//World view signals and slots                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void listBuildTools(int mode);
void robotItemClicked(QListWidgetItem* item);
void updatePropertyInformation();

void MainWindow::objectSelected(object_id id)
{
    if(worldObjects.contains(id))
    {
        nothingSelected();

        selected = id;
        WorldObjectProperties* obj = worldObjects[id];
        QStandardItemModel* model = propertiesModel;

        QMap<QString, PropertyView>& objProps = obj->getProperties();
        QStringList propKeys = objProps.keys();
        model->setRowCount(objProps.size());

        int i = 0;
        for(QString k : propKeys)
        {
           QModelIndex ind;

           //Set key
           ind = model->index(i, 0);
           model->setData(ind, k);

           connect(&objProps[k], &PropertyView::valueSet, this, &MainWindow::updatePropertyInformation);

           displayed_properties[i] = k;
           i++;
        }

        connect(model, &QStandardItemModel::dataChanged, [this, obj, model](QModelIndex tl, QModelIndex br)
        {
           for(int i = tl.row(); i <= br.row(); i++)
               obj->getProperties()[displayed_properties[i]].set(model->data(model->index(i, 1)));
        });

        updatePropertyInformation();

        ui->simulatorActiveWidget->setCurrentItem(listItems[id]);

        objectIsSelected(id);
    }
}

void MainWindow::nothingSelected()
{
    propertiesModel->setRowCount(0);
    disconnect(propertiesModel, &QStandardItemModel::dataChanged, 0, 0);

    if(worldObjects.contains(selected))
    {
        for(auto iter : worldObjects[selected]->getProperties())
            disconnect(&iter, 0, this, 0);
    }
    selected = 0;

    nothingIsSelected();
}

void MainWindow::robotItemClicked(QListWidgetItem* item)
{
    objectSelected(item->data(Qt::DisplayRole).toInt());
}


void MainWindow::updatePropertyInformation()
{
    if(worldObjects.contains(selected))
    {
        QMap<QString, PropertyView>& ppts = worldObjects[selected]->getProperties();
        QStandardItemModel* model = propertiesModel;

        ui->propertiesTableView->setUpdatesEnabled(false);
        for(int i=0; i<ppts.size(); i++)
        {
            QString key = model->data(model->index(i, 0)).toString();
            model->setData(model->index(i, 1), ppts[key].get().toString(), Qt::DisplayRole);
        }
        ui->propertiesTableView->setUpdatesEnabled(true);
    }
}

void MainWindow::updateSimulatorBuildTools()
{
    //for each world object in folder
        //create new widget and display in
        //put name of build object? can I add an icon for each build object? would be useful
}

void MainWindow::updateDesignerBuildTools()
{
    //for(auto iter = robot->getProperties().begin(); iter != robot->getProperties().end(); iter++)
        //connect(&iter.value(), &PropertyView::valueSet, [](QVariant v){qDebug() << v;});
    //for each file in folder
        //how many tabs/categories?
        //widget using items from *components in basic viewer + mouse-over description
}
