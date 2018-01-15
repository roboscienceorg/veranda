#include "default_robot_loader.h"
#include <QDebug>

QVector<WorldObject*> DefaultRobotLoader::loadFile(QString filePath, QMap<QString, WorldObjectComponent_Plugin_If *> plugins)
{
    QVector<WorldObjectComponent_If*> components;

    qDebug() << "Building Default Robot";
    if(plugins.contains("org.sdsmt.sim.2d.worldObjectComponent.defaults.touchring"))
    {
        qDebug() << "Adding touch sensor ring";

        WorldObjectComponent_If* touchRing = plugins["org.sdsmt.sim.2d.worldObjectComponent.defaults.touchring"]->createComponent();
        QMap<QString, PropertyView> props = touchRing->getProperties();

        props["x_local"].set(0, true);
        props["y_local"].set(0, true);
        props["theta_local"].set(0, true);
        props["angle_start"].set(0, true);
        props["angle_end"].set(360, true);
        props["ring_radius"].set(2, true);
        props["sensor_count"].set(10, true);

        components.push_back(touchRing);
    }

    if(plugins.contains("org.sdsmt.sim.2d.worldObjectComponent.defaults.fixedwheel"))
    {
        qDebug() << "Adding differential wheel base";

        WorldObjectComponent_If* leftWheel = plugins["org.sdsmt.sim.2d.worldObjectComponent.defaults.fixedwheel"]->createComponent();
        WorldObjectComponent_If* rightWheel = plugins["org.sdsmt.sim.2d.worldObjectComponent.defaults.fixedwheel"]->createComponent();

        QMap<QString, PropertyView> propsl = leftWheel->getProperties();
        QMap<QString, PropertyView> propsr = rightWheel->getProperties();

        propsl["x_local"].set(-1.5, true);
        propsr["x_local"].set(1.5, true);

        propsl["y_local"].set(0, true);
        propsr["y_local"].set(0, true);

        propsl["theta_local"].set(90, true);
        propsr["theta_local"].set(90, true);

        propsl["wheel_radius"].set(0.75, true);
        propsr["wheel_radius"].set(0.75, true);

        propsl["wheel_width"].set(0.5, true);
        propsr["wheel_width"].set(0.5, true);

        propsl["is_driven"].set(true, true);
        propsr["is_driven"].set(true, true);

        propsl["channels/input_speed"].set("robot0/left_wheel", true);
        propsr["channels/input_speed"].set("robot0/right_wheel", true);

        components.push_back(leftWheel);
        components.push_back(rightWheel);
    }

    if(plugins.contains("org.sdsmt.sim.2d.worldObjectComponent.defaults.simpleshape"))
    {
        qDebug() << "Adding simple circle";

        WorldObjectComponent_If* simpleShape = plugins["org.sdsmt.sim.2d.worldObjectComponent.defaults.simpleshape"]->createComponent();
        QMap<QString, PropertyView> props = simpleShape->getProperties();

        props["x"].set(0, true);
        props["y"].set(0, true);
        props["radius"].set(2, true);

        components.push_back(simpleShape);
    }

    WorldObject* robot1 = new WorldObject(components);
    QMap<QString, PropertyView> props = robot1->getProperties();
    props["X"].set(5, true);
    props["Y"].set(0, true);
    props["Theta"].set(45, true);

    WorldObject* robot2 = new WorldObject(components);

    robot2->getProperties()["X"].set(0, true);
    robot2->getProperties()["Y"].set(0, true);
    robot2->getProperties()["Theta"].set(315, true);

    return QVector<WorldObject*>{robot1, robot2};
}
