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
        props["ring_radius"].set(1, true);
        props["sensor_count"].set(10, true);

        //components.push_back(touchRing);
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

        propsl["is_driven"].set(false, true);
        propsr["is_driven"].set(false, true);

        propsl["max_rps"].set(0, true);
        propsr["max_rps"].set(3, true);

        components.push_back(leftWheel);
        components.push_back(rightWheel);
    }

    return QVector<WorldObject*>{new WorldObject(components)};
}
