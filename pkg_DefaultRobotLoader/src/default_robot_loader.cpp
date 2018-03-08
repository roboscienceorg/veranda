#include "default_robot_loader.h"
#include <QDebug>

QVector<QSharedPointer<WorldObject> > DefaultRobotLoader::loadFile(QString filePath, QMap<QString, WorldObjectComponent_Plugin_If *> plugins)
{
    qDebug() << "Building default robots......................................";
    QSharedPointer<WorldObject> robot1(makeDiffDriveBot(plugins));
    robot1->getProperties()["LocalPos/X"]->set(10, true);
    robot1->getProperties()["LocalPos/Y"]->set(0, true);
    robot1->getProperties()["LocalPos/Theta"]->set(0, true);

    QSharedPointer<WorldObject> robot2(makeAckermannBot(plugins));
    robot2->getProperties()["LocalPos/X"]->set(-3, true);
    robot2->getProperties()["LocalPos/Y"]->set(10, true);
    robot2->getProperties()["LocalPos/Theta"]->set(20, true);
    qDebug() << "Done building default robots.................................";

    return {robot1, robot2};
}

WorldObject* DefaultRobotLoader::makeDiffDriveBot(QMap<QString, WorldObjectComponent_Plugin_If *> plugins)
{
    QVector<WorldObjectComponent*> components;

    if(plugins.contains("org.sdsmt.sim.2d.worldObjectComponent.defaults.touchring"))
    {
        WorldObjectComponent* touchRing = plugins["org.sdsmt.sim.2d.worldObjectComponent.defaults.touchring"]->createComponent();
        QMap<QString, QSharedPointer<PropertyView>> props = touchRing->getProperties();

        props["LocalPos/X"]->set(0, true);
        props["LocalPos/Y"]->set(0, true);
        props["LocalPos/Theta"]->set(0, true);
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

        propsl["LocalPos/X"]->set(-1.5, true);
        propsr["LocalPos/X"]->set(1.5, true);

        propsl["LocalPos/Y"]->set(0, true);
        propsr["LocalPos/Y"]->set(0, true);

        propsl["LocalPos/Theta"]->set(90, true);
        propsr["LocalPos/Theta"]->set(90, true);

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

        props["LocalPos/X"]->set(0, true);
        props["LocalPos/Y"]->set(0, true);
        props["radius"]->set(2, true);

        components.push_back(circle);
    }

    if(plugins.contains("org.sdsmt.sim.2d.worldObjectComponent.defaults.lidar"))
    {
        WorldObjectComponent* lidar1 = plugins["org.sdsmt.sim.2d.worldObjectComponent.defaults.lidar"]->createComponent();
        WorldObjectComponent* lidar2 = plugins["org.sdsmt.sim.2d.worldObjectComponent.defaults.lidar"]->createComponent();
        QMap<QString, QSharedPointer<PropertyView>> props1 = lidar1->getProperties();
        QMap<QString, QSharedPointer<PropertyView>> props2 = lidar2->getProperties();

        props1["LocalPos/X"]->set(0, true);
        props1["LocalPos/Y"]->set(0, true);
        props1["LocalPos/Theta"]->set(0, true);
        props1["scan_radius"]->set(10, true);
        props1["scan_range"]->set(180, true);
        props1["scan_points"]->set(20, true);
        props1["scan_rate"]->set(10, true);

        props2["LocalPos/X"]->set(0, true);
        props2["LocalPos/Y"]->set(0, true);
        props1["LocalPos/Theta"]->set(180, true);
        props2["scan_radius"]->set(10, true);
        props2["scan_range"]->set(180, true);
        props2["scan_points"]->set(20, true);
        props2["scan_rate"]->set(10, true);

        components.push_back(lidar1);
        components.push_back(lidar2);
    }

    return new WorldObject(components, "Differential Turtle");
}

WorldObject* DefaultRobotLoader::makeAckermannBot(QMap<QString, WorldObjectComponent_Plugin_If *> plugins)
{
    QVector<WorldObjectComponent*> components;

    if(plugins.contains("org.sdsmt.sim.2d.worldObjectComponent.defaults.lidar"))
    {
        WorldObjectComponent* lidar = plugins["org.sdsmt.sim.2d.worldObjectComponent.defaults.lidar"]->createComponent();
        QMap<QString, QSharedPointer<PropertyView>> props = lidar->getProperties();

        props["LocalPos/X"]->set(0, true);
        props["LocalPos/Y"]->set(2, true);
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

        props["LocalPos/X"]->set(0, true);
        props["LocalPos/Y"]->set(0, true);
        props["height"]->set(4, true);
        props["width"]->set(2, true);

        components.push_back(rectangle);
    }

    if(plugins.contains("org.sdsmt.sim.2d.worldObjectComponent.defaults.fixedwheel"))
    {
        WorldObjectComponent* leftWheel = plugins["org.sdsmt.sim.2d.worldObjectComponent.defaults.fixedwheel"]->createComponent();
        WorldObjectComponent* rightWheel = plugins["org.sdsmt.sim.2d.worldObjectComponent.defaults.fixedwheel"]->createComponent();

        QMap<QString, QSharedPointer<PropertyView>> propsl = leftWheel->getProperties();
        QMap<QString, QSharedPointer<PropertyView>> propsr = rightWheel->getProperties();

        propsl["LocalPos/X"]->set(-1.25, true);
        propsr["LocalPos/X"]->set(1.25, true);

        propsl["LocalPos/Y"]->set(-2, true);
        propsr["LocalPos/Y"]->set(-2, true);

        propsl["LocalPos/Theta"]->set(90, true);
        propsr["LocalPos/Theta"]->set(90, true);

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

        props["LocalPos/X"]->set(0, true);
        props["LocalPos/Y"]->set(2, true);
        props["vehicle_length"]->set(4, true);
        props["axle_length"]->set(2.5, true);
        props["LocalPos/Theta"]->set(0, true);
        props["wheel_radius"]->set(0.75, true);
        props["wheel_width"]->set(0.5, true);
        props["density"]->set(20, true);
        props["channels/input_angle"]->set("robot1/steer", true);

        components.push_back(steer);
    }

    return new WorldObject(components, "Ackermann Box");
}
