#ifndef MODE_CONTROLLER_H
#define MODE_CONTROLLER_H

#include <QStandardItem>
#include <QListWidgetItem>
#include <QHeaderView>
#include <QProgressBar>
#include <QVector>
#include <QPair>
#include <QTableView>
#include <QToolButton>
#include <QMap>

#include "interfaces/simulator_ui_if.h"
#include "interfaces/simulator_visual_if.h"
#include "interfaces/world_object_wrappers.h"
#include "ui/joystickprototype.h"
#include "ui/designer_widget.h"

#include <sdsmt_simulator/world_object_component_plugin.h>
#include <sdsmt_simulator/object_loader_if.h>
#include <sdsmt_simulator/object_saver_if.h>

class Mode_Controller: public QObject
{
    Q_OBJECT
    typedef std::function<Simulator_Visual_If*()> visualizerFactory;

public:
    Mode_Controller(visualizerFactory factory, QToolButton* pModeButton, QWidget* pMenu, QWidget* pToolsMenu, QListWidget* pActive, QTableView* pProperties, QTabWidget* pTabs, QWidget *parent = nullptr);
    void setWorldBounds(double xMin, double xMax, double yMin, double yMax);
    QMap<object_id, WorldObjectProperties*> worldObjects;
    Simulator_Visual_If* visual;
    bool simulator = false;

private:
    object_id idIncrementer = 1;
    QToolButton* modeButton;
    QWidget* menu;
    QWidget* toolsMenu;
    QListWidget* active;
    QTabWidget* tabs;
    QMap<QString, QListWidget*> toolTabs;
    visualizerFactory makeWidget;

    object_id selected;
    WorldObjectProperties* selectedTool = nullptr;

    QTableView* properties;
    QStandardItemModel* propertiesModel;
    QMap<object_id, QListWidgetItem*> listItems;
    QMap<QString, QSharedPointer<PropertyView>> selectedProps;
    QMap<uint64_t, QString> displayed_properties;
    bool cloneSelectedTool();

public slots:
    //Simulator core added something to the simulation
    //Do not delete the world object when it is removed; that will be handled elsewhere
    virtual void worldObjectsAddedToSimulation(QVector<QPair<WorldObjectProperties*, object_id>> objs);

    //Simulator core removed something from simulation
    virtual void worldObjectsRemovedFromSimulation(QVector<object_id> oIds);

    void setPropertiesTableView();
    void open();
    void close();
    int getNextId();
    void addObjectToView();
    QVector<WorldObjectComponent *> getComponents();
    void deleteObjectFromView();
    void addObjectToTools(WorldObjectComponent* component);

    void objectSelected(object_id id);
    void nothingSelected();
    void robotItemClicked(QListWidgetItem* item);
    void updatePropertyInformation();

    void simObjectMoveDragged(object_id id, double dx, double dy);
    void simObjectRotateDragged(object_id id, double dt);

signals:
    //selection slots for clicks on the world view
    void requestAddWorldObject(QVector<WorldObject> obj);
    void objectIsSelected(object_id id);
    void nothingIsSelected();
};

#endif // MODE_CONTROLLER_H
