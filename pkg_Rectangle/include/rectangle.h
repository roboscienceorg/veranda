#ifndef RECTANGLE_H
#define RECTANGLE_H

#include "rclcpp/rclcpp.hpp"

#include <Box2D/Box2D.h>
#include <sdsmt_simulator/world_object_component_if.h>

#include <QVector>
#include <QString>
#include <QSet>
#include <QObject>

#include <memory>

class Rectangle : public WorldObjectComponent_If
{
    Q_OBJECT

    constexpr static double PI = 3.14159265359;
    constexpr static double RAD2DEG = (360.0/(2*PI));
    constexpr static double DEG2RAD = (1.0/RAD2DEG);

    bool _connected = false;

    Property x = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE,
                          "X position of the shape"), QVariant(0.0),
                          [](QVariant _old, QVariant _new)
                          {
                                bool valid;
                                int newVal = _new.toDouble(&valid);
                                if(valid)
                                    return _new;
                                return _old;
                          });

    Property y = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE,
                          "Y position of the shape"), QVariant(0.0),
                          [](QVariant _old, QVariant _new)
                          {
                                bool valid;
                                int newVal = _new.toDouble(&valid);
                                if(valid)
                                    return _new;
                                return _old;
                          });

    Property height = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE,
                               "Height of the rectangle"), QVariant(1.0),
                               [](QVariant _old, QVariant _new)
                               {
                                     bool valid;
                                     int newVal = _new.toDouble(&valid);
                                     if(valid && newVal >= 0)
                                         return _new;
                                     return _old;
                               });

    Property width = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE,
                               "Width of the rectangle"), QVariant(1.0),
                               [](QVariant _old, QVariant _new)
                               {
                                     bool valid;
                                     int newVal = _new.toDouble(&valid);
                                     if(valid && newVal >= 0)
                                         return _new;
                                     return _old;
                               });

    Property rotation = Property(PropertyInfo(false, false, PropertyInfo::DOUBLE,
                               "Degrees clockwise of north"), QVariant(1.0),
                               [](QVariant _old, QVariant _new)
                               {
                                     bool valid;
                                     int newVal = _new.toDouble(&valid);
                                     if(valid && newVal >= -360 && newVal < 360)
                                         return _new;
                                     return _old;
                               });

    QMap<QString, PropertyView> _properties{
        {"x_pos", &x},
        {"y_pos", &y},
        {"height", &height},
        {"width", &width},
        {"rotation", &rotation}
    };

    Model* shape_model = nullptr;

    b2Body* body = nullptr;
    b2WeldJoint* joint = nullptr;

public:
    Rectangle(QObject* parent=nullptr);

    WorldObjectComponent_If* clone(QObject *newParent);

    virtual QMap<QString, PropertyView> getProperties(){
        return _properties;
    }

    virtual QString getPropertyGroup(){
        return "Rectangle";
    }

    QVector<Model*> getModels(){
        return {shape_model};
    }

    bool usesChannels(){
        return true;
    }

    void generateBodies(b2World *world, object_id oId, b2Body *anchor);
    void clearBodies(b2World* world);
    void setROSNode(std::shared_ptr<rclcpp::Node>){}

private slots:
    void _channelChanged(QVariant);
    void _buildModels();

public slots:
    //Connects to all ROS topics
    virtual void connectChannels(){}

    //Disconnects all ROS topics
    virtual void disconnectChannels(){}

    virtual void worldTicked(const b2World*, const double);

    virtual void setObjectMass(double mass){}
};

#endif // FLOATER_DRIVETRAIN_H
