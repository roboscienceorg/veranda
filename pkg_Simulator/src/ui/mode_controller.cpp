#include "include/ui/mode_controller.h"

Mode_Controller::Mode_Controller(visualizerFactory factory, QToolButton *pModeButton, QWidget* pMenu, QWidget* pToolsMenu, QListWidget* pActive, QTableView* pProperties, QTabWidget* pTabs, QWidget *parent)
{
    //set mode objects, these will be used to display menus and lists of a specific mode
    makeWidget = factory;
    visual = factory();
    active = pActive;
    menu = pMenu;
    toolsMenu = pToolsMenu;
    modeButton = pModeButton;
    properties = pProperties;
    tabs = pTabs;
    tabs->clear();

    //designer world bounds will be redefined smaller in the main window because designer objects are small
    setWorldBounds(-100, 100, -100, 100);

    //connect slots to alter which mode is active and user interaction with the world view
    connect(active, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(robotItemClicked(QListWidgetItem*)));
    connect(this, SIGNAL(objectIsSelected(object_id)), visual, SLOT(objectSelected(object_id)));
    connect(this, SIGNAL(nothingIsSelected()), visual, SLOT(nothingSelected()));
    connect(visual, SIGNAL(userDragMoveObject(object_id,double,double)), this, SLOT(simObjectMoveDragged(object_id,double,double)));
    connect(visual, SIGNAL(userDragRotateObject(object_id,double)), this, SLOT(simObjectRotateDragged(object_id,double)));
    connect(visual, SIGNAL(userSelectedObject(object_id)), this, SLOT(objectSelected(object_id)));

    //keep tabs neutral and small enough to not enlarge the right side menu unnecessarily
    tabs->setAutoFillBackground( false );
    tabs->setMaximumWidth(200);
    setPropertiesTableView();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Functions for mode, menu, and ui component setup                                                                          //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//create columns for the properties display, properties are only displayed for the active object
void Mode_Controller::setPropertiesTableView()
{
    propertiesModel = new QStandardItemModel(0,2,this); //12 Rows and 2 Columns
    propertiesModel->setHorizontalHeaderItem(0, new QStandardItem(QString("Property")));
    propertiesModel->setHorizontalHeaderItem(1, new QStandardItem(QString("Value")));
    properties->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    properties->setModel(propertiesModel);

    nothingSelected();
}

//make this mode visible
void Mode_Controller::open()
{
    visual->setVisible(true);
    tabs->setVisible(true);
    active->setVisible(true);
    properties->setVisible(true);    
    modeButton->setEnabled(false);
    menu->setVisible(true);
    toolsMenu->setVisible(true);

    nothingSelected();
}

//make this mode invisible
void Mode_Controller::close()
{
    visual->setVisible(false);
    tabs->setVisible(false);
    active->setVisible(false);
    properties->setVisible(false);
    modeButton->setEnabled(true);
    menu->setVisible(false);
    toolsMenu->setVisible(false);

    nothingSelected();
}

//add robot to the simulation world view (will be auto-called from the backend in the simulator)
void Mode_Controller::worldObjectsAddedToSimulation(QVector<QPair<WorldObjectProperties *, object_id>> objs)
{
    for(auto& p : objs)
    {
        object_id& oId = p.second;
        WorldObjectProperties* object = p.first;
        qDebug() << oId;

        if(worldObjects.contains(oId)) throw std::logic_error("world object " + std::to_string(oId) + " already exists in ui");

        worldObjects[oId] = object;

        visual->objectAddedToScreen(object->getModels(), oId);

        listItems[oId] = new QListWidgetItem();
        listItems[oId]->setData(Qt::DisplayRole, QString::number(oId) + " " + object->getName());
        active->addItem(listItems[oId]);
    }
    if(objs.size())
        objectSelected(objs.last().second);
}

//remove robot from the simulation world view (will be auto-called from the backend in the simulator)
void Mode_Controller::worldObjectsRemovedFromSimulation(QVector<object_id> oIds)
{
    for(object_id oId : oIds)
    {
        visual->objectRemovedFromScreen(oId);
        worldObjects[oId]->deleteLater();
        worldObjects.remove(oId);

        active->removeItemWidget(listItems[oId]);
        delete listItems[oId];
        listItems.remove(oId);

        if(selected == oId)
            nothingSelected();
    }
}

//alter the reference size of the world view (zoom in/out)
void Mode_Controller::setWorldBounds(double xMin, double xMax, double yMin, double yMax)
{
    if(xMin > xMax) std::swap(xMin, xMax);
    if(yMin > yMax) std::swap(yMin, yMax);

    visual->setWorldBounds(xMin, xMax, yMin, yMax);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Interaction slots for button clicks in UI                                                                                 //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//used to create a new instance of whatever tool is selected in the right side menu
bool Mode_Controller::cloneSelectedTool()
{
    if(toolTabs.size() < 1)
        return false;

    QList<QListWidgetItem*> l = toolTabs[tabs->tabText(tabs->currentIndex())]->selectedItems();
    if ( ! l.empty()) {
            selectedTool = new WorldObjectProperties(dynamic_cast<Designer_Widget*>(l.at(0))->component->clone(), this);
            return true;
    }
    return false;
}

//used to name active objects
int Mode_Controller::getNextId()
{
    while(worldObjects.contains(idIncrementer))
        idIncrementer++;
    return idIncrementer;
}

//adds an object to the world view for designer or sends request to backend for simulator
void Mode_Controller::addObjectToView()
{
    if(cloneSelectedTool())
    {
        WorldObjectProperties* object = selectedTool;

        //add object to view if designer
        if(simulator)
        {
            QVector<WorldObject*> rVector;
            rVector.push_back(object->getObject());
            requestAddWorldObjects(rVector, true);
            delete object;
        }
        else
        {
            //add object to active list
            object_id oId = getNextId();

            worldObjectsAddedToSimulation({{object, oId}});
        }
    }
}

//removes an object from the world view for designer or sends request to backend for simulator
void Mode_Controller::deleteObjectFromView()
{
    object_id oId = selected;
    if(simulator)
    {
        QVector<object_id> rVector;
        rVector.push_back(oId);
        requestRemoveWorldObjects(rVector);
    }
    else
    {
        worldObjectsRemovedFromSimulation({oId});
    }
}

//removes all obects from world view and active objects list
void Mode_Controller::clear()
{
    for(auto e : worldObjects.keys())
    {
      objectSelected(e);
      deleteObjectFromView();
    }
}

//used by mainwindow to export a designer object to the simulator by returning all active objects
QVector<WorldObjectComponent*> Mode_Controller::getComponents()
{
    QVector<WorldObjectComponent*> rVector;

    for(auto e : worldObjects.keys())
    {
      WorldObjectProperties* object = worldObjects.value(e);
      rVector.push_back(object->getComponent()->clone());
    }

    return rVector;
}

//adds a world object to available tools and takes care of tab selection/new tab
void Mode_Controller::addObjectToTools(WorldObjectComponent* component)
{
    WorldObjectProperties* properties = new WorldObjectProperties(component, this);

    //if tab does not exist, create it then add new designer widget
    if(toolTabs[properties->getType()] == nullptr)
    {       
        toolTabs[properties->getType()] = new QListWidget();
        tabs->addTab(toolTabs[properties->getType()], properties->getType());
        toolTabs[properties->getType()]->setViewMode(QListWidget::IconMode);
        toolTabs[properties->getType()]->setResizeMode(QListWidget::Adjust);
        toolTabs[properties->getType()]->setMovement(QListView::Static);
        toolTabs[properties->getType()]->setIconSize(QSize(150, 150));
    }

    //add new designer widget to a tab (initialized with parent, so will automatically add to view)
    if(simulator)
        Designer_Widget* tile = new Designer_Widget(component, properties, makeWidget, toolTabs[properties->getType()], true);
    else
        Designer_Widget* tile = new Designer_Widget(component, properties, makeWidget, toolTabs[properties->getType()]);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//World view signals and slots                                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//backend creates green arrows and rotations to move the selected object, here the properties get displayed
void Mode_Controller::objectSelected(object_id id)
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

        properties->setUpdatesEnabled(false);
        int i = 0;
        for(QString k : propKeys)
        {
           QModelIndex ind;

           //Set key
           ind = model->index(i, 0);
           model->setData(ind, k);

           connect(selectedProps[k].data(), &PropertyView::valueSet, this, &Mode_Controller::updatePropertyInformation);

           displayed_properties[i] = k;
           i++;
        }
        properties->setUpdatesEnabled(true);

        connect(model, &QStandardItemModel::dataChanged, [this, obj, model](QModelIndex tl, QModelIndex br)
        {
           for(int i = tl.row(); i <= br.row(); i++)
               selectedProps[displayed_properties[i]]->set(model->data(model->index(i, 1)), !simulator);
        });

        updatePropertyInformation();

        properties->reset();

        QBrush disabled(QColor(230, 230, 230));
        for(auto it = displayed_properties.begin(); it != displayed_properties.end(); it++)
        {
            PropertyInfo inf = selectedProps[it.value()]->info();

            QStandardItem* itm = model->item(it.key(), 0);
            itm->setFlags(itm->flags() & ~Qt::ItemIsEditable);
            itm->setBackground(disabled);
            itm->setToolTip(inf.description);

            itm = model->item(it.key(), 1);
            itm->setToolTip(inf.description);

            if(inf.readOnly)
            {
                itm->setFlags(itm->flags() & ~Qt::ItemIsEditable);
                itm->setBackground(disabled);
            }
        }

        active->setCurrentItem(listItems[id]);

        objectIsSelected(id);
    }
}

//backend "releases" selected object and properties are cleared
void Mode_Controller::nothingSelected()
{
    propertiesModel->setRowCount(0);
    disconnect(propertiesModel, &QStandardItemModel::dataChanged, 0, 0);

    if(worldObjects.contains(selected))
    {
        for(auto iter : selectedProps)
            disconnect(iter.data(), 0, this, 0);
    }
    selectedProps.clear();
    selected = -1;

    nothingIsSelected();
}

//selects item via user clicking on active object list item
void Mode_Controller::robotItemClicked(QListWidgetItem* item)
{
    QString strName = item->data(Qt::DisplayRole).toString().section(QRegExp("\\s+"), 0, 0, QString::SectionSkipEmpty);
    objectSelected(strName.toInt());
}

//updates properties in backend and properties list when a property gets altered by user
void Mode_Controller::updatePropertyInformation()
{
    if(worldObjects.contains(selected))
    {
        QStandardItemModel* model = propertiesModel;

        properties->setUpdatesEnabled(false);
        for(int i=0; i<selectedProps.size(); i++)
        {
            QString key = model->data(model->index(i, 0)).toString();
            model->setData(model->index(i, 1), selectedProps[key]->get().toString(), Qt::DisplayRole);
        }

        if (!simulator)
            listItems[selected]->setData(Qt::DisplayRole, QString::number(selected) + " " + worldObjects[selected]->getName());
        properties->setUpdatesEnabled(true);
    }
}

//moves selected object up/down & left/right
void Mode_Controller::simObjectMoveDragged(object_id id, double dx, double dy)
{
    auto obj = worldObjects.find(id);
    if(obj != worldObjects.end())
    {
        obj.value()->translate(dx, dy);
    }
}

//rotates selected object about its' centerpoint
void Mode_Controller::simObjectRotateDragged(object_id id, double dt)
{
    auto obj = worldObjects.find(id);
    if(obj != worldObjects.end())
    {
        obj.value()->rotate(dt);
    }
}
