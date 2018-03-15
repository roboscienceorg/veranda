/*#include "ui/mainwindow.h"
#include "ui_mainwindow.h"

#include <QMessageBox>
#include <QFileDialog>
#include <QDebug>
#include <QDate>
#include <QStandardItemModel>
#include <QThread>
#include <QListWidgetItem>
#include <QProgressBar>
#include <QtConcurrent/QtConcurrent>
#include <QWindow>

#include <stdexcept>
#include <string>
#include <limits>

MainWindow::MainWindow(visualizerFactory factory, QMap<QString, WorldObjectComponent_Plugin_If *> components,
                       QVector<WorldObjectLoader_If*> oloaders, QVector<WorldObjectSaver_If*> osavers,
                       QVector<WorldLoader_If *> wloaders, QVector<WorldSaver_If *> wsavers, WorldLoader_If* defaultLoader_, QWidget *parent) :
    Simulator_Ui_If(parent),
    makeWidget(factory), componentPlugins(components),
    ui(new Ui::MainWindow)
{
    qRegisterMetaType<QVector<object_id>>("QVector<object_id>");
    qRegisterMetaType<QVector<QSharedPointer<WorldObject>>>("QVector<QSharedPointer<WorldObject>>");

    defaultLoader = defaultLoader_;

    ui->setupUi(this);

    ui->centralWidget->setLayout(ui->mainLayout);

    for(WorldObjectLoader_If* l : oloaders)
        for(QString e : l->fileExts())
            objectLoaders[e] += l;

    for(WorldObjectSaver_If* s : osavers)
        for(QString e : s->fileExts())
            objectSavers[e] += s;

    for(WorldLoader_If* l : wloaders)
        for(QString e : l->fileExts())
            worldLoaders[e] += l;

    for(WorldSaver_If* s : wsavers)
        for(QString e : s->fileExts())
            worldSavers[e] += s;

    //If no world loader plugins exist,
    //don't present button to user
    if(wloaders.size() == 0)
        ui->importMapButton->setVisible(false);

    speed = 1;
    play = false;
    simulation = true;

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
    //connect(visualDesigner, SIGNAL(userSelectedObject(object_id)), this, SLOT(toolSelected(object_id)));

    //connect(visualDesigner, SIGNAL(userSelectedTool(tool_id)), this, SLOT(toolSelected(t_id)));
    connect(visualSimulator, SIGNAL(userDragMoveObject(object_id,double,double)), this, SLOT(simObjectMoveDragged(object_id,double,double)));
    connect(visualSimulator, SIGNAL(userDragRotateObject(object_id,double)), this, SLOT(simObjectRotateDragged(object_id,double)));
    connect(visualDesigner, SIGNAL(userDragMoveObject(object_id,double,double)), this, SLOT(buildObjectMoveDragged(object_id,double,double)));
    connect(visualDesigner, SIGNAL(userDragRotateObject(object_id,double)), this, SLOT(buildObjectRotateDragged(object_id,double)));

    connect(this, SIGNAL(objectIsSelected(object_id)), visualSimulator, SLOT(objectSelected(object_id)));
    connect(this, SIGNAL(nothingIsSelected()), visualSimulator, SLOT(nothingSelected()));
    //connect(this, SIGNAL(objectIsSelected(object_id)), visualDesigner, SLOT(toolSelected(object_id)));
    connect(this, SIGNAL(nothingIsSelected()), visualDesigner, SLOT(nothingSelected()));

    ui->designerToolsList->clear();
    ui->simulatorToolsList->clear();

    //add/delete tools
    //connect(this, SIGNAL(addObjectToSimulator(WorldObjectProperties*)), visualSimulator, SLOT(OBJECT_ADDED_TO_WORLD(WorldObjectProperties*)));
    //connect(this, SIGNAL(deleteObjectFromSimulator(WorldObjectProperties*)), visualSimulator, SLOT(OBJECT_REMOVED_FROM_WORLD(WorldObjectProperties*)));

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
    connect(ui->addObjectButton, SIGNAL (released()), this, SLOT (addObjectToSimButtonClick()));
    connect(ui->deleteObjectButton, SIGNAL (released()), this, SLOT (deleteObjectFromSimButtonClick()));
    connect(ui->loadObjectsButton, SIGNAL (released()), this, SLOT (loadObjectsForSimButtonClick()));

    //Designer mode tool button signals and slots
    connect(ui->addToolButton, SIGNAL (released()), this, SLOT (addToolButtonClick()));
    connect(ui->deleteToolButton, SIGNAL (released()), this, SLOT (deleteToolButtonClick()));
    connect(ui->exportObjectButton, SIGNAL (released()), this, SLOT (exportObjectButtonClick()));
    connect(ui->loadToolsButton, SIGNAL (released()), this, SLOT (loadToolsButtonClick()));

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
void MainWindow::worldObjectsAddedToSimulation(QVector<QPair<WorldObjectProperties *, object_id>> objs)
{
    for(auto& p : objs)
    {
        object_id& oId = p.second;
        WorldObjectProperties* object = p.first;

        if(worldObjects.contains(oId)) throw std::logic_error("world object " + std::to_string(oId) + " already exists in ui");

        worldObjects[oId] = object;

        visualSimulator->objectAddedToScreen(object->getModels(), oId);

        listItems[oId] = new QListWidgetItem();
        listItems[oId]->setData(Qt::DisplayRole, QString::number(oId));
        ui->simulatorActiveWidget->addItem(listItems[oId]);
    }
    if(objs.size())
        objectSelected(objs.last().second);
}

void MainWindow::worldObjectsRemovedFromSimulation(QVector<object_id> oIds)
{
    for(object_id oId : oIds)
    {
        visualSimulator->objectRemovedFromScreen(oId);
        worldObjects.remove(oId);

        ui->simulatorActiveWidget->removeItemWidget(listItems[oId]);
        delete listItems[oId];
        listItems.remove(oId);

        if(selected == oId)
            nothingSelected();
    }
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

    visualSimulator->setToolsEnabled(false);
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

    visualSimulator->setToolsEnabled(true);
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
    visualDesigner->setVisible(false);
    visualSimulator->setVisible(true);
    simulation = true;

    ui->modeLabel->setText("Simulator");
    ui->buildToolsLabel->setText("Simulation Build Tools");

    ui->simulatorMenuWidget->setVisible(true);
    ui->designerMenuWidget->setVisible(false);
    ui->simulatorToolsMenu->setVisible(true);
    ui->designerToolsMenu->setVisible(false);
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
    visualDesigner->setVisible(true);
    visualSimulator->setVisible(false);
    simulation = false;

    ui->modeLabel->setText("Designer");
    ui->buildToolsLabel->setText("Designer Build Tools");

    ui->simulatorMenuWidget->setVisible(false);
    ui->designerMenuWidget->setVisible(true);
    ui->simulatorToolsMenu->setVisible(false);
    ui->designerToolsMenu->setVisible(true);
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
    uint64_t next = (speed + 1) % SPEEDBUTTONS.size();
    userSetPhysicsTickMultiplier(SPEEDBUTTONS[next].first);
}

void MainWindow::physicsTickMultiplierChanged(double mult)
{
    double closest = std::numeric_limits<double>::max();
    for(int i=0; i<SPEEDBUTTONS.size(); i++)
    {
        double diff = std::abs(SPEEDBUTTONS[i].first - mult);
        if(diff < closest)
        {
            closest = diff;
            speed = i;
        }
    }

    ui->speedSimButton->setToolTip(SPEEDBUTTONS[speed].second.first);
    ui->speedSimButton->setIcon(QIcon(SPEEDBUTTONS[speed].second.second));
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
          QString types;
          qDebug() << "Able to load files:" << worldLoaders.keys();
          for(QString k : worldLoaders.keys())
              if(k.size())
                types += k + ";;";

          types = types.left(types.size()-2);

          // Save was clicked
          QString path = QFileDialog::getOpenFileName(this, tr("Open File"), "/home", types);

          if(path.length())
          {
              QString ext = QFileInfo(path).suffix();

              bool done = false;
              for(auto it = worldLoaders.begin(); it != worldLoaders.end() && !done; it++)
              {
                  if(it.key().contains(ext))
                      for(WorldLoader_If* wl : it.value())

                          //Find the first loader that can load this file
                          //and try to load. If it fails, we can't load it
                          if(!done && wl->canLoadFile(path))
                          {
                              //Get user options in main thread
                              wl->getUserOptions(path);

                              //Spin up side thread to actually load it
                              QtConcurrent::run([this, path, wl](){
                                  QVector<QSharedPointer<WorldObject>> sharedObjs;
                                  QVector<WorldObject*> loadedObjs;
                                  try
                                  {
                                      //Load file in separate thread
                                      qDebug() << "Load file";
                                      loadedObjs=wl->loadFile(path, componentPlugins);
                                  }catch(std::exception& ex){}

                                  if(loadedObjs.size())
                                  {
                                      qDebug() << "Clear world";
                                      userRemoveWorldObjectsFromSimulation(worldObjects.keys().toVector());

                                      qDebug() << "Build new world";
                                      for(WorldObject* w : loadedObjs)
                                        sharedObjs.push_back(QSharedPointer<WorldObject>(w));

                                      userAddWorldObjectsToSimulation(sharedObjs);

                                      //Add default robots
                                      if(defaultLoader && defaultLoader->canLoadFile(path))
                                      {
                                          loadedObjs.clear();
                                          sharedObjs.clear();
                                          try
                                          {
                                              loadedObjs=defaultLoader->loadFile(path, componentPlugins);
                                          }catch(std::exception& ex){}

                                          for(WorldObject* w : loadedObjs)
                                            sharedObjs.push_back(QSharedPointer<WorldObject>(w));

                                          userAddWorldObjectsToSimulation(sharedObjs);
                                      }
                                  }
                                  else
                                  {
                                    emit error("Unable to open \'" + path + "\' as a world file");
                                  }
                              });

                              //Stop looking for a file handler
                              //for this file
                              done = true;
                          }
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

void MainWindow::joystickButtonClick()
{
    //create window and joystick, link them
    QWindow *jWindow = new QWindow();
    JoystickPrototype *joystick = new JoystickPrototype(jWindow);

    //Joystick button signals and slots
    connect(joystick, SIGNAL(joystickMoved(double, double, double, QString)), this, SIGNAL(joystickMoved(double, double, double, QString)));
    connect(joystick, SIGNAL(joystickButtonPress(int, QString)), this, SIGNAL(joystickButtonPress(int, QString)));
    connect(joystick, SIGNAL(joystickButtonRelease(int, QString)), this, SIGNAL(joystickButtonRelease(int, QString)));

    connect(this, &MainWindow::windowClosed, joystick, &JoystickPrototype::deleteLater);
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
void MainWindow::addObjectToSimButtonClick()
{
    //ui->simulatorToolsList->currentWidget()->selectedItems();
}

void MainWindow::deleteObjectFromSimButtonClick()
{}

void MainWindow::loadObjectsForSimButtonClick()
{

    QString types;
    qDebug() << "Able to load files:" << objectLoaders.keys();
    for(QString k : objectLoaders.keys())
        if(k.size())
          types += k + ";;";

    types = types.left(types.size()-2);

    QFileDialog dialog(this);
    dialog.setDirectory(QDir::homePath());
    dialog.setFileMode(QFileDialog::ExistingFiles);
    //dialog.setNameFilter(trUtf8(types));


    qDebug() << "types:" << types;

    QStringList fileNames;
    if (dialog.exec())
        fileNames = dialog.selectedFiles();


    ui->simulatorToolsList->clear();

    for(int i = 0; i < fileNames.size(); i++)
    {
        QString path = fileNames[i];

        //TODO create WorldObjectProperties* object from file

        //newObject = makeWidget();
        //TODO add shapes to widget view (can probably do this in Designer_Widget.cpp)

        //TODO check properties for "type"
        //match existing type in designerTabs or simulatorTabs?
        //add to tab, else push new tab and add

        simulatorTabs[QString::number(i)] = new QListWidget();
        ui->simulatorToolsList->addTab(simulatorTabs[QString::number(i)], QString::number(i));
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Designer mode tool button signals and slots                                                                               //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MainWindow::addToolButtonClick()
{
    Designer_Widget* select = dynamic_cast<Designer_Widget*>(designerTabs[ui->designerToolsList->currentWidget()->accessibleName()]->selectedItems()[0]);
    WorldObjectProperties* object = new WorldObjectProperties(select->component->clone(), this);

    //add object to view
    visualDesigner->objectAddedToScreen(object->getModels(), designerId);

    //add object to UI model
    designerObjects[designerId] = object;
            designerId++;
}

void MainWindow::deleteToolButtonClick()
{
    //disable mode switch until request is fully complete
    ui->simulatorButton->setEnabled(false);

    //with only one "selected" item, do I need to differentiate designerObjects/worldObjects just for view?
    //deleteToolFromDesigner(designerObjects[selected]);

    //TODO: ultimately call this: worldObjectRemovedFromSimulation(WorldObjectProperties *object, object_id oId)
}

void MainWindow::exportObjectButtonClick()
{
    //if(propertyType)
    //simulatorTabs[QString::number(5)] = new QListWidget();
    //ui->simulatorToolsList->addTab(simulatorTabs[QString::number(5)], QString::number(5));

    //no save to file?
}

void MainWindow::loadToolsButtonClick()
{
    ui->designerToolsList->clear();

    for( auto it = componentPlugins.begin(); it != componentPlugins.end(); it++)
    {
        qDebug() << "lookin' at " << it.value();
        WorldObjectComponent* component = it.value()->createComponent();
        //manually set WorldObjectComponent to WorldObjectProperties

        WorldObjectProperties* properties = new WorldObjectProperties(component, this);

        //if tab does not exist, create it then add new designer widget
        if(designerTabs[properties->getType()] == nullptr)
        {
            designerTabs[properties->getType()] = new QListWidget();
            ui->designerToolsList->addTab(designerTabs[properties->getType()], properties->getType());
        }

        //add new designer widget to a tab
        Designer_Widget* tile = new Designer_Widget(component, properties, makeWidget, designerTabs[properties->getType()]);
        designerTabs[properties->getType()]->addItem(tile);
        designerTabs[properties->getType()]->update();
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//World view signals and slots                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MainWindow::objectSelected(object_id id)
{
    if(worldObjects.contains(id))
    {
        nothingSelected();

        selected = id;
        WorldObjectProperties* obj = worldObjects[id];
        QStandardItemModel* model = propertiesModel;

        selectedProps = obj->getProperties();
        QStringList propKeys = selectedProps.keys();
        model->setRowCount(selectedProps.size());

        int i = 0;
        for(QString k : propKeys)
        {
           QModelIndex ind;

           //Set key
           ind = model->index(i, 0);
           model->setData(ind, k);

           connect(selectedProps[k].data(), &PropertyView::valueSet, this, &MainWindow::updatePropertyInformation);

           displayed_properties[i] = k;
           i++;
        }

        connect(model, &QStandardItemModel::dataChanged, [this, obj, model](QModelIndex tl, QModelIndex br)
        {
           for(int i = tl.row(); i <= br.row(); i++)
               selectedProps[displayed_properties[i]]->set(model->data(model->index(i, 1)));
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
        for(auto iter : selectedProps)
            disconnect(iter.data(), 0, this, 0);
    }
    selectedProps.clear();
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
        QStandardItemModel* model = propertiesModel;

        ui->propertiesTableView->setUpdatesEnabled(false);
        for(int i=0; i<selectedProps.size(); i++)
        {
            QString key = model->data(model->index(i, 0)).toString();
            model->setData(model->index(i, 1), selectedProps[key]->get().toString(), Qt::DisplayRole);
        }
        ui->propertiesTableView->setUpdatesEnabled(true);
    }
}

void MainWindow::simObjectMoveDragged(object_id id, double dx, double dy)
{
   auto obj = worldObjects.find(id);
   if(obj != worldObjects.end())
       obj.value()->translate(dx, dy);
}

void MainWindow::simObjectRotateDragged(object_id id, double dt)
{
    auto obj = worldObjects.find(id);
    if(obj != worldObjects.end())
        obj.value()->rotate(dt);
}

//These two slots exist so that if build objects and simulation
//objects are stored separately, we can index into the correct list
void MainWindow::buildObjectMoveDragged(object_id id, double dx, double dy)
{
    auto obj = designerObjects.find(id);
    if(obj != designerObjects.end())
        obj.value()->translate(dx, dy);
}

void MainWindow::buildObjectRotateDragged(object_id id, double dt)
{
    auto obj = designerObjects.find(id);
    if(obj != designerObjects.end())
        obj.value()->rotate(dt);
}

void MainWindow::errorMessage(QString error)
{
    QMessageBox err;
    err.setText(error);
    err.exec();
}
//THIS COMPILES
*/

#include "ui/mainwindow.h"
#include "ui_mainwindow.h"

#include <QMessageBox>
#include <QFileDialog>
#include <QDebug>
#include <QDate>
#include <QStandardItemModel>
#include <QThread>
#include <QListWidgetItem>
#include <QProgressBar>
#include <QtConcurrent/QtConcurrent>
#include <QWindow>

#include <stdexcept>
#include <string>
#include <limits>

MainWindow::MainWindow(visualizerFactory factory, QMap<QString, WorldObjectComponent_Plugin_If *> components,
                       QVector<WorldObjectLoader_If*> oloaders, QVector<WorldObjectSaver_If*> osavers,
                       QVector<WorldLoader_If *> wloaders, QVector<WorldSaver_If *> wsavers, WorldLoader_If* defaultLoader_, QWidget *parent) :
    Simulator_Ui_If(parent),
    makeWidget(factory), componentPlugins(components),
    ui(new Ui::MainWindow)
{
    qRegisterMetaType<QVector<object_id>>("QVector<object_id>");
    qRegisterMetaType<QVector<QSharedPointer<WorldObject>>>("QVector<QSharedPointer<WorldObject>>");

    defaultLoader = defaultLoader_;

    ui->setupUi(this);

    ui->centralWidget->setLayout(ui->mainLayout);

    for(WorldObjectLoader_If* l : oloaders)
        for(QString e : l->fileExts())
            objectLoaders[e] += l;

    for(WorldObjectSaver_If* s : osavers)
        for(QString e : s->fileExts())
            objectSavers[e] += s;

    for(WorldLoader_If* l : wloaders)
        for(QString e : l->fileExts())
            worldLoaders[e] += l;

    for(WorldSaver_If* s : wsavers)
        for(QString e : s->fileExts())
            worldSavers[e] += s;

    //If no world loader plugins exist,
    //don't present button to user
    if(wloaders.size() == 0)
        ui->importMapButton->setVisible(false);

    speed = 1;
    play = false;

    //Initialize Widget Settings
    ui->playSimButton->setToolTip("Play Simulation");
    ui->speedSimButton->setToolTip("Speed x2");
    ui->buildToolsWidget->setVisible((false));
    ui->designerMenuWidget->setVisible(false);
    ui->designerToolsMenu->setVisible(false);
    ui->simulatorButton->setEnabled(false);
    ui->addObjectButton->setEnabled(false);
    ui->addToolButton->setEnabled(false);

    //Initiate a world view for Simulator and another for Designer
    simulator = new Mode_Controller(factory, ui->simulatorActiveWidget, ui->propertiesTableView, ui->simulatorToolsList, this);
    designer = new Mode_Controller(factory, ui->designerActiveWidget, ui->designerPropertiesTableView, ui->designerToolsList, this);

    simulator->simulator = true;
    ui->worldViewLayout->addWidget(simulator->visual);
    ui->worldViewLayout->addWidget(designer->visual);
    simulator->open();
    designer->close();

    //ui->propertiesTableView->verticalHeader()->setVisible(false);

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
    //connect(ui->addObjectButton, SIGNAL (released()), simulator, SLOT (addObjectToView()));
    //connect(ui->deleteObjectButton, SIGNAL (released()), simulator, SLOT (deleteObjectFromView()));
    //connect(ui->loadObjectsButton, SIGNAL (released()), this, SLOT (loadObjectsForSimButtonClick()));

    //Designer mode tool button signals and slots
    connect(ui->addToolButton, SIGNAL (released()), designer, SLOT (addObjectToView()));
    connect(ui->deleteToolButton, SIGNAL (released()), designer, SLOT (deleteObjectFromView()));
    connect(ui->exportObjectButton, SIGNAL (released()), this, SLOT (exportObjectButtonClick()));
    //connect(ui->loadToolsButton, SIGNAL (released()), this, SLOT (loadToolsButtonClick()));
    loadToolsButtonClick();
    connect(this, SIGNAL (objectsAddedToSimulation(QVector<QPair<WorldObjectProperties*, object_id>>)), simulator, SLOT (worldObjectsAddedToSimulation(QVector<QPair<WorldObjectProperties*, object_id>>)));
    connect(this, SIGNAL (objectsRemovedFromSimulation(QVector<object_id> oId)), simulator, SLOT (worldObjectsRemovedFromSimulation(QVector<object_id> oId)));

    connect(this, SIGNAL (objectsAddedToSimulation(QVector<QPair<WorldObjectProperties*, object_id>>)), designer, SLOT (worldObjectsAddedToSimulation(QVector<QPair<WorldObjectProperties*, object_id>>)));
    connect(this, SIGNAL (objectsRemovedFromSimulation(QVector<object_id> oId)), designer, SLOT (worldObjectsRemovedFromSimulation(QVector<object_id> oId)));
}

MainWindow::~MainWindow()
{
    delete ui;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Public signals and slots                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

    simulator->visual->setToolsEnabled(false);
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

    simulator->visual->setToolsEnabled(true);
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
    designer->close();
    simulator->open();

    ui->modeLabel->setText("Simulator");
    ui->buildToolsLabel->setText("Simulation Build Tools");

    ui->simulatorMenuWidget->setVisible(true);
    ui->designerMenuWidget->setVisible(false);
    ui->simulatorToolsMenu->setVisible(true);
    ui->designerToolsMenu->setVisible(false);

    //Enable/Disable Mode Buttons
    ui->designerButton->setEnabled(true);
    ui->simulatorButton->setEnabled(false);
}

void MainWindow::designerButtonClick()
{
    designer->open();
    simulator->close();

    ui->modeLabel->setText("Designer");
    ui->buildToolsLabel->setText("Designer Build Tools");

    ui->simulatorMenuWidget->setVisible(false);
    ui->designerMenuWidget->setVisible(true);
    ui->simulatorToolsMenu->setVisible(false);
    ui->designerToolsMenu->setVisible(true);

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
    uint64_t next = (speed + 1) % SPEEDBUTTONS.size();
    userSetPhysicsTickMultiplier(SPEEDBUTTONS[next].first);
}

void MainWindow::physicsTickMultiplierChanged(double mult)
{
    double closest = std::numeric_limits<double>::max();
    for(int i=0; i<SPEEDBUTTONS.size(); i++)
    {
        double diff = std::abs(SPEEDBUTTONS[i].first - mult);
        if(diff < closest)
        {
            closest = diff;
            speed = i;
        }
    }

    ui->speedSimButton->setToolTip(SPEEDBUTTONS[speed].second.first);
    ui->speedSimButton->setIcon(QIcon(SPEEDBUTTONS[speed].second.second));
    ui->speedSimButton->setIconSize(QSize(32,32));
}

void MainWindow::screenshotSimButtonClick()
{
    QPixmap pixmap(simulator->visual->size());
    simulator->visual->render(&pixmap);
    qint64 current = QDateTime::currentMSecsSinceEpoch();
    QString fileName = QString::number(current) + ".png";

    QFile file(fileName);
    file.open(QIODevice::WriteOnly);
    pixmap.save(&file, "PNG");
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
          QString types;
          qDebug() << "Able to load files:" << worldLoaders.keys();
          for(QString k : worldLoaders.keys())
              if(k.size())
                types += k + ";;";

          types = types.left(types.size()-2);

          // Save was clicked
          QString path = QFileDialog::getOpenFileName(this, tr("Open File"), "/home", types);

          if(path.length())
          {
              QString ext = QFileInfo(path).suffix();

              bool done = false;
              for(auto it = worldLoaders.begin(); it != worldLoaders.end() && !done; it++)
              {
                  if(it.key().contains(ext))
                      for(WorldLoader_If* wl : it.value())

                          //Find the first loader that can load this file
                          //and try to load. If it fails, we can't load it
                          if(!done && wl->canLoadFile(path))
                          {
                              //Get user options in main thread
                              wl->getUserOptions(path);

                              //Spin up side thread to actually load it
                              QtConcurrent::run([this, path, wl](){
                                  QVector<QSharedPointer<WorldObject>> sharedObjs;
                                  QVector<WorldObject*> loadedObjs;
                                  try
                                  {
                                      //Load file in separate thread
                                      qDebug() << "Load file";
                                      loadedObjs=wl->loadFile(path, componentPlugins);
                                  }catch(std::exception& ex){}

                                  if(loadedObjs.size())
                                  {
                                      qDebug() << "Clear world";
                                      userRemoveWorldObjectsFromSimulation(simulator->worldObjects.keys().toVector());

                                      qDebug() << "Build new world";
                                      for(WorldObject* w : loadedObjs)
                                        sharedObjs.push_back(QSharedPointer<WorldObject>(w));

                                      userAddWorldObjectsToSimulation(sharedObjs);

                                      //Add default robots
                                      if(defaultLoader && defaultLoader->canLoadFile(path))
                                      {
                                          loadedObjs.clear();
                                          sharedObjs.clear();
                                          try
                                          {
                                              loadedObjs=defaultLoader->loadFile(path, componentPlugins);
                                          }catch(std::exception& ex){}

                                          for(WorldObject* w : loadedObjs)
                                            sharedObjs.push_back(QSharedPointer<WorldObject>(w));

                                          userAddWorldObjectsToSimulation(sharedObjs);
                                      }
                                  }
                                  else
                                  {
                                    emit error("Unable to open \'" + path + "\' as a world file");
                                  }
                              });

                              //Stop looking for a file handler
                              //for this file
                              done = true;
                          }
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

void MainWindow::joystickButtonClick()
{
    //create window and joystick, link them
    QWindow *jWindow = new QWindow();
    JoystickPrototype *joystick = new JoystickPrototype(jWindow);

    //Joystick button signals and slots
    connect(joystick, SIGNAL(joystickMoved(double, double, double, QString)), this, SIGNAL(joystickMoved(double, double, double, QString)));
    connect(joystick, SIGNAL(joystickButtonPress(int, QString)), this, SIGNAL(joystickButtonPress(int, QString)));
    connect(joystick, SIGNAL(joystickButtonRelease(int, QString)), this, SIGNAL(joystickButtonRelease(int, QString)));

    connect(this, &MainWindow::windowClosed, joystick, &JoystickPrototype::deleteLater);
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
// RIGHT HAND MENU - Tool button signals and slots                                                                          //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MainWindow::loadObjectsForSimButtonClick()
{
    /*
    QString types;
    qDebug() << "Able to load files:" << objectLoaders.keys();
    for(QString k : objectLoaders.keys())
        if(k.size())
          types += k + ";;";

    types = types.left(types.size()-2);

    QFileDialog dialog(this);
    dialog.setDirectory(QDir::homePath());
    dialog.setFileMode(QFileDialog::ExistingFiles);
    //dialog.setNameFilter(trUtf8(types));


    qDebug() << "types:" << types;

    QStringList fileNames;
    if (dialog.exec())
        fileNames = dialog.selectedFiles();


    ui->simulatorToolsList->clear();

    for(int i = 0; i < fileNames.size(); i++)
    {
        QString path = fileNames[i];

        //TODO create WorldObjectProperties* object from file

        //newObject = makeWidget();
        //TODO add shapes to widget view (can probably do this in Designer_Widget.cpp)

        //TODO check properties for "type"
        //match existing type in designerTabs or simulatorTabs?
        //add to tab, else push new tab and add

        //simulatorTabs[QString::number(i)] = new QListWidget();
        //ui->simulatorToolsList->addTab(simulatorTabs[QString::number(i)], QString::number(i));
    }*/
}

void MainWindow::exportObjectButtonClick()
{
    //designer->getItemAsComponent();
    //if(propertyType)
    //simulatorTabs[QString::number(5)] = new QListWidget();
    //ui->simulatorToolsList->addTab(simulatorTabs[QString::number(5)], QString::number(5));

    //no save to file?
}

void MainWindow::loadToolsButtonClick()
{
    designer->tabs->clear();

    //tools can only be loaded from initial component plugins
    for( auto it = componentPlugins.begin(); it != componentPlugins.end(); it++)
    {
        qDebug() << "lookin' at " << it.value();
        WorldObjectComponent* component = it.value()->createComponent();
        designer->addObjectToTools(component);
        ui->addToolButton->setEnabled(true);
    }
}

void MainWindow::errorMessage(QString error)
{
    QMessageBox err;
    err.setText(error);
    err.exec();
}
