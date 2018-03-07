#include "default_robot_loader.h"
#include <QDebug>

QVector<WorldObject*> DefaultRobotLoader::loadFile(QString filePath, QMap<QString, WorldObjectComponent_Plugin_If *> plugins)
{
    WorldObject* robot1 = makeDiffDriveBot(plugins);
    QMap<QString, QSharedPointer<PropertyView>> props = robot1->getProperties();
    props["X"]->set(20, true);
    props["Y"]->set(0, true);
    props["Theta"]->set(0, true);


    WorldObject* robot2 = makeAckermannBot(plugins);

    robot2->getProperties()["X"]->set(0, true);
    robot2->getProperties()["Y"]->set(0, true);
    robot2->getProperties()["Theta"]->set(0, true);

    return QVector<WorldObject*>{robot1, robot2};
}

WorldObject* DefaultRobotLoader::makeDiffDriveBot(QMap<QString, WorldObjectComponent_Plugin_If *> plugins)
{

    QVector<WorldObjectComponent*> components;

    if(plugins.contains("org.sdsmt.sim.2d.worldObjectComponent.defaults.touchring"))
    {
        WorldObjectComponent* touchRing = plugins["org.sdsmt.sim.2d.worldObjectComponent.defaults.touchring"]->createComponent();
        QMap<QString, QSharedPointer<PropertyView>> props = touchRing->getProperties();

        props["x_local"]->set(0, true);
        props["y_local"]->set(0, true);
        props["theta_local"]->set(0, true);
        props["angle_start"]->set(0, true);
        props["angle_end"]->set(360, true);
        props["ring_radius"]->set(2, true);
        props["sensor_count"]->set(10, true);

        components.push_back(touchRing);
    }

    if(plugins.contains("org.sdsmt.sim.2d.worldObjectComponent.defaults.fixedwheel"))
    {
        WorldObjectComponent* leftWheel = plugins["org.sdsmt.sim.2d.worldObjectComponent.defaults.fixedwheel"]->createComponent();
        WorldObjectComponent* rightWheel = plugins["org.sdsmt.sim.2d.worldObjectComponent.defaults.fixedwheel"]->createComponent();

        QMap<QString, QSharedPointer<PropertyView>> propsl = leftWheel->getProperties();
        QMap<QString, QSharedPointer<PropertyView>> propsr = rightWheel->getProperties();

        propsl["x_local"]->set(-1.5, true);
        propsr["x_local"]->set(1.5, true);

        propsl["y_local"]->set(0, true);
        propsr["y_local"]->set(0, true);

        propsl["theta_local"]->set(90, true);
        propsr["theta_local"]->set(90, true);

        propsl["wheel_radius"]->set(0.75, true);
        propsr["wheel_radius"]->set(0.75, true);

        propsl["wheel_width"]->set(0.5, true);
        propsr["wheel_width"]->set(0.5, true);

        propsl["density"]->set(20, true);
        propsr["density"]->set(20, true);

        propsl["is_driven"]->set(true, true);
        propsr["is_driven"]->set(true, true);

        propsl["channels/input_speed"]->set("robot0/left_wheel", true);
        propsr["channels/input_speed"]->set("robot0/right_wheel", true);

        components.push_back(leftWheel);
        components.push_back(rightWheel);
    }

    if(plugins.contains("org.sdsmt.sim.2d.worldObjectComponent.defaults.circle"))
    {
        WorldObjectComponent* circle = plugins["org.sdsmt.sim.2d.worldObjectComponent.defaults.circle"]->createComponent();
        QMap<QString, QSharedPointer<PropertyView>> props = circle->getProperties();

        props["x"]->set(0, true);
        props["y"]->set(0, true);
        props["radius"]->set(2, true);

        components.push_back(circle);
    }

    return new WorldObject(components);
}

WorldObject* DefaultRobotLoader::makeAckermannBot(QMap<QString, WorldObjectComponent_Plugin_If *> plugins)
{
    QVector<WorldObjectComponent*> components;

    if(plugins.contains("org.sdsmt.sim.2d.worldObjectComponent.defaults.lidar"))
    {
        WorldObjectComponent* lidar = plugins["org.sdsmt.sim.2d.worldObjectComponent.defaults.lidar"]->createComponent();
        QMap<QString, QSharedPointer<PropertyView>> props = lidar->getProperties();

        props["x_local"]->set(0, true);
        props["y_local"]->set(2, true);
        props["scan_radius"]->set(5, true);
        props["scan_range"]->set(90, true);
        props["scan_points"]->set(20, true);
        props["scan_rate"]->set(10, true);

        components.push_back(lidar);
    }

    if(plugins.contains("org.sdsmt.sim.2d.worldObjectComponent.defaults.rectangle"))
    {
        WorldObjectComponent* rectangle = plugins["org.sdsmt.sim.2d.worldObjectComponent.defaults.rectangle"]->createComponent();
        QMap<QString, QSharedPointer<PropertyView>> props = rectangle->getProperties();

        props["x_pos"]->set(0, true);
        props["y_pos"]->set(0, true);
        props["height"]->set(4, true);
        props["width"]->set(2, true);
        props["rotation"]->set(0, true);

        components.push_back(rectangle);
    }

    if(plugins.contains("org.sdsmt.sim.2d.worldObjectComponent.defaults.fixedwheel"))
    {
        WorldObjectComponent* leftWheel = plugins["org.sdsmt.sim.2d.worldObjectComponent.defaults.fixedwheel"]->createComponent();
        WorldObjectComponent* rightWheel = plugins["org.sdsmt.sim.2d.worldObjectComponent.defaults.fixedwheel"]->createComponent();

        QMap<QString, QSharedPointer<PropertyView>> propsl = leftWheel->getProperties();
        QMap<QString, QSharedPointer<PropertyView>> propsr = rightWheel->getProperties();

        propsl["x_local"]->set(-1.25, true);
        propsr["x_local"]->set(1.25, true);

        propsl["y_local"]->set(-2, true);
        propsr["y_local"]->set(-2, true);

        propsl["theta_local"]->set(90, true);
        propsr["theta_local"]->set(90, true);

        propsl["wheel_radius"]->set(0.75, true);
        propsr["wheel_radius"]->set(0.75, true);

        propsl["wheel_width"]->set(0.5, true);
        propsr["wheel_width"]->set(0.5, true);

        propsl["density"]->set(20, true);
        propsr["density"]->set(20, true);

        propsl["is_driven"]->set(true, true);
        propsr["is_driven"]->set(true, true);

        propsl["channels/input_speed"]->set("robot1/left_wheel", true);
        propsr["channels/input_speed"]->set("robot1/right_wheel", true);

        components.push_back(leftWheel);
        components.push_back(rightWheel);
    }

    if(plugins.contains("org.sdsmt.sim.2d.worldObjectComponent.defaults.ackermann"))
    {
        WorldObjectComponent* steer = plugins["org.sdsmt.sim.2d.worldObjectComponent.defaults.ackermann"]->createComponent();

        QMap<QString, QSharedPointer<PropertyView>> props = steer->getProperties();

        props["x_local"]->set(0, true);
        props["y_local"]->set(2, true);
        props["vehicle_length"]->set(4, true);
        props["axle_length"]->set(2.5, true);
        props["theta_local"]->set(0, true);
        props["wheel_radius"]->set(0.75, true);
        props["wheel_width"]->set(0.5, true);
        props["density"]->set(20, true);
        props["channels/input_angle"]->set("robot1/steer", true);

        components.push_back(steer);
    }

    return new WorldObject(components);
}
