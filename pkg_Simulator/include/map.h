#ifndef MAP_H
#define MAP_H

#include <QVector>
#include <Box2D/Box2D.h>
#include <QMap>

#include "interfaces/world_object_if.h"

class Map : public WorldObject_If
{
    Q_OBJECT

    Property xMin = Property(PropertyInfo(true, false, PropertyInfo::DOUBLE,
                                          "Coordinate value (meters) for left side of map"), 0);

    Property xMax = Property(PropertyInfo(true, false, PropertyInfo::DOUBLE,
                                          "Coordinate value (meters) for right side of map"), 0);

    Property yMin = Property(PropertyInfo(true, false, PropertyInfo::DOUBLE,
                                          "Coordinate value (meters) for bottom side of map"), 0);

    Property yMax = Property(PropertyInfo(true, false, PropertyInfo::DOUBLE,
                                          "Coordinate value (meters) for top side of map"), 0);

    Property xOrigin = Property(PropertyInfo(true, false, PropertyInfo::DOUBLE,
                                          "Coordinate value (meters) offset for origin x"), 0);

    Property yOrigin = Property(PropertyInfo(true, false, PropertyInfo::DOUBLE,
                                          "Coordinate value (meters) offset for origin y"), 0);

    QMap<QString, PropertyView> properties
    {
        {"xMin", &xMin},
        {"xMax", &xMax},
        {"yMin", &yMin},
        {"yMax", &yMax},
        {"xOrigin", &xOrigin},
        {"yOrigin", &yOrigin}
    };

    Model* model = nullptr;

    QVector<b2PolygonShape *> staticBodies;

    void _shiftObstaclesAroundOrigin(double oldX, double oldY, double newX, double newY);
    void _buildModel();
public:
    Map(QObject* parent = nullptr);
    ~Map();

    bool setXMin(double i);
    bool setXMax(double i);
    bool setYMin(double i);
    bool setYMax(double i);
    bool setXOrigin(double i);
    bool setYOrigin(double i);

    void getBounds(double& xMin_, double& yMin_, double& xMax_, double& yMax_);

    bool setStaticBodies(QVector<b2PolygonShape *> polygons);

    //Virtual clone
    virtual WorldObject_If* clone(QObject* newParent=nullptr);

    //Interfaces for UI to display properties
    virtual QMap<QString, PropertyView>& getAllProperties(){ return properties; }
    virtual QString propertyGroupName(){ return ""; }

    //Interface for world view to draw
    virtual QVector<Model*> getModels() { return {model}; }

    //Tells the physics engine how many bodies are needed
    virtual uint64_t staticBodiesRequired(){ return staticBodies.size(); }

    //Physics engine assigns bodies
    virtual void setStaticBodies(QVector<b2Body*>& bodies);
    virtual void clearStaticBodies(){}
};

#endif // MAP_H
