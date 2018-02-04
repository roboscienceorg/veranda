#ifndef POLYGONSCOMPONENT_H
#define POLYGONSCOMPONENT_H

#include <sdsmt_simulator/world_object_component_if.h>
#include <Box2D/Box2D.h>

class PolygonsComponent : public WorldObjectComponent_If
{
    Q_OBJECT

    Model* _polyModel;
    QVector<b2PolygonShape*> _polyShapes;
    QVector<b2Shape*> _shapePtrs;

    b2Body* _polyBody = nullptr;
    QVector<b2Fixture*> _polyFixtures;

    Property _numShapes = Property(PropertyInfo(true, false, PropertyInfo::INT, "Number of polygons in the shape"),
                                   QVariant(0));

    QMap<QString, PropertyView> _properties
    {
        {"polgyon_count", &_numShapes}
    };

    b2PolygonShape* copy(b2PolygonShape* orig);

public:
    PolygonsComponent(QVector<b2PolygonShape*> polys, QObject* parent=nullptr);
    ~PolygonsComponent();

    //Constructs copy of component
    WorldObjectComponent_If* clone(QObject* newParent=nullptr);

    //Interactions with ROS
    void setROSNode(std::shared_ptr<rclcpp::Node> node){}

    //Drawing Interactions
    QVector<Model*> getModels(){ return {_polyModel}; }

    //UI Interactions
    QMap<QString, PropertyView> getProperties(){ return _properties; }
    QString getPropertyGroup(){ return "PolyGroup"; }

    bool usesChannels(){return false;}

    void generateBodies(b2World* world, object_id oId, b2Body* anchor);
    void clearBodies(b2World* world);

public slots:
    void connectChannels(){}
    void disconnectChannels(){}
    void worldTicked(const b2World* w, const double t){}
    void setObjectMass(double mass){}

signals:
    void massChanged(WorldObjectComponent_If* component, double mass);
};

#endif
