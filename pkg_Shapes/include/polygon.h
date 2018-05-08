//! \file
#pragma once

#include "rclcpp/rclcpp.hpp"

#include <Box2D/Box2D.h>
#include <sdsmt_simulator/world_object_component.h>

#include <QVector>
#include <QString>
#include <QSet>
#include <QObject>
#include <QVariantList>
#include <QVariant>

#include <memory>

#include "defines.h"

/*!
 * \brief WorldObjectComponent for creating a solid mass from a loop of points
 * The Polygon type can take any loop of points as an outer loop and some number
 * of of inner loops representing holes in the shape and create a soldi mass in the
 * physics engine. This is done by computing a triangulation of the loops and adding
 * each triangle as a separate box2d fixture.
 *
 * After a set of points is loaded, the polygon will straighten them, removing
 * successive points that have a low cross product. The resulting reduced point set
 * will be scaled and then triangulated. Triangulation is done using the OpenGL GLU Tesselator
 *
 * \todo Determine how to make the point lists editable by the user
 * \todo Switch Models to use QPolygonF instead of vector of b2Shape so visual is not limited by box2d polygon point limit; once this is done, allow the user to toggle between only loops and all triangles
 */
class Polygon : public WorldObjectComponent
{
    Q_OBJECT

    //! Model of all the triangle shapes
    Model* _triangleModel;

    //! List of all the triangles created
    QVector<b2Shape*> _triangleShapes;

    //! Physics body of the object
    b2Body* _polyBody = nullptr;

    //! List of triangle fixtures created
    QVector<b2Fixture*> _polyFixtures;

    //! Box2D world the object is loaded in
    b2World* _world = nullptr;

    /*!
     * \brief Converts a QVariant into list of lists conforming to the expected format.
     * This method expects the input QVariant to be a QVariantList with each element being
     * either a QPointF type or QVariantList containing doubles. In the latter case, each element
     * must contain 2 elements, an x, y pair
     * \param[in] v QVariant to convert
     * \return A QVariantList with each element being a QVariantList contining 2 double values
     */
    static QVariant toPoly(const QVariant& v)
    {
        QVariantList poly;
        QSequentialIterable iterable = v.value<QSequentialIterable>();
        for (const QVariant &vi : iterable)
        {
            if(vi.canConvert<QPointF>())
            {
                QPointF p = vi.value<QPointF>();
                poly.append(QVariant(QVariantList{p.x(), p.y()}));
            }
            else
            {
                QVariantList pnt;
                QSequentialIterable iterable2 = vi.value<QSequentialIterable>();
                for(const QVariant& v : iterable2)
                {
                    pnt.append(v.toDouble());
                    if(pnt.size() == 2) break;
                }
                poly.append(QVariant(pnt));
            }
        }
        return poly;
    }

    /*!
     * \brief Checks if a QVariant is a valid set of points
     * A valid set of points has 3 requirements:
     * * It must be a QVariantList
     * * Each element must be a QPointF or QVariantList containing 2 double values
     * * There must be at least 3 elements in the outer list
     *
     * \param[in] v The QVariant to check
     * \return True if v meets the three requirements, false otherwise
     */
    static bool isPoly(const QVariant& v)
    {
        if (!v.canConvert<QVariantList>()) return false;
        QSequentialIterable iterable = v.value<QSequentialIterable>();
        uint64_t count = 0;
        // Can use C++11 range-for:
        for (const QVariant &vi : iterable)
        {
            bool good = false;
            if(vi.canConvert<QPointF>()) good = true;
            else if(vi.canConvert<QVariantList>())
            {
                QSequentialIterable iterable2 = vi.value<QSequentialIterable>();
                if(iterable2.size() == 2)
                {
                    good = true;
                    for(const QVariant& vii : iterable2)
                        if(!vii.canConvert<double>())
                        {
                            good = false;
                            break;
                        }
                }
            }
            if(!good) return false;

            count++;
        }
        return count >= 3;
    }

    //! Property: The number of triangles generated
    Property _numShapes = Property(PropertyInfo(true, false, false, PropertyInfo::INT, "Number of polygons in the shape"),
                                   QVariant(0));

    //! The set of points which make up the outer loop of the shape
    Property _outerShape = Property(PropertyInfo(true, true, false, PropertyInfo::INT, "The outermost polygon of the shape"),
                                    QVariantList{QVariantList{0, 0}, QVariantList{1, 1}, QVariantList{1, 0}},
                                    [](const QVariant& _old, const QVariant& _new)
                                    {
                                        qDebug() << "Validate outer loop";
                                        if(isPoly(_new)) return toPoly(_new);
                                        qDebug() << "Failed";
                                        return _old;
                                    });

    //! A List of sets of points which make up holes in the outer shape
    Property _innerShapes = Property(PropertyInfo(true, true, false, PropertyInfo::INT, "Inner polygons defining holes"),
                                   QVariantList(),
                                    [](const QVariant& _old, const QVariant& _new)
                                    {
                                        qDebug() << "Validate inner loops";
                                        if (!_new.canConvert<QVariantList>()) return _old;
                                        qDebug() << "Is a list";
                                        QSequentialIterable iterable = _new.value<QSequentialIterable>();

                                        QVariantList polyList;
                                        // Can use C++11 range-for:
                                        for (const QVariant &vi : iterable)
                                        {
                                            if(!isPoly(vi))
                                            {
                                                qDebug() << "Failed";
                                                return _old;
                                            }
                                            polyList.append(toPoly(vi));
                                        }

                                        return QVariant(polyList);
                                    });

    //! Property: Flag for whether all triangles should be displayed or just the line loops
    Property _fullDraw = Property(PropertyInfo(true, true, false, PropertyInfo::BOOL, "Whether or not to draw all triangles"),
                                  QVariant(false), &Property::bool_validator);

    //! Property: Threshold under which cross product means the line is straight
    Property _straight = Property(PropertyInfo(false, true, false, PropertyInfo::DOUBLE, "How close to straight an edge has to be to have points removed"),
                                  QVariant(0), &Property::abs_double_validator);

    //! Property: Horizontal scaling factor
    Property _scalex = Property(PropertyInfo(false, true, false, PropertyInfo::DOUBLE, "Horizontal scaling factor"),
                                  QVariant(1), &Property::abs_double_validator);

    //! Property: Vertical scaling factor
    Property _scaley = Property(PropertyInfo(false, true, false, PropertyInfo::DOUBLE, "Vertical scaling factor"),
                                  QVariant(1), &Property::abs_double_validator);

#define prop(x) QSharedPointer<PropertyView>(new PropertyView(&(x)))
    //! Mapping of polygon properties by identifiers
    QMap<QString, QSharedPointer<PropertyView>> _properties
    {
        {"polgyon_count", prop(_numShapes)},
        {"outer_shape", prop(_outerShape)},
        {"inner_shapes", prop(_innerShapes)},
        //{"draw_triangles", prop(_fullDraw)},
        {"straightness", prop(_straight)},
        {"scale/horiz", prop(_scalex)},
        {"scale/vert", prop(_scaley)}
    };
#undef prop

    //! Straightens, scales, and triangulates the current line loops
    void makeTriangles();

    //! Creates box2d fixtures for all the triangles created in the last makeTriangles() call
    void makeFixtures();

protected:
    /*!
     * \brief Gets all the properties specific to this polygon component
     * \return A mapping of all the properties by their identifiers
     */
    QMap<QString, QSharedPointer<PropertyView>> _getProperties(){ return _properties; }

public:
    /*!
     * \brief Constructs a new Polygon component
     * \param[in] parent Parent QObject of the polygon
     */
    Polygon(QObject* parent=nullptr);

    //! Cleans up all the triangles in the Polygon
    ~Polygon();

    /*!
     * \brief Constructs a new empty Polygon component
     * \param[in] newParent Parent QObject of the copy
     * \return A newly constructed Polygon component
     */
    WorldObjectComponent *_clone(QObject* newParent=nullptr);

    /*!
     * \brief Create the physics body used by the component
     * For now, the body is always a static body; this component doe snot move
     * \todo Create some method by which Polygons can be either static or dynamic bodies
     * \param[in] world Box2d World to add the Polygon to
     * \param[in] oId object_id of the object the Polygon is part of
     * \param[in] anchor Anchor body the component should joint to
     */
    void generateBodies(b2World* world, object_id oId, b2Body* anchor);

    //! Clears the Polygon from the Box2D world
    void clearBodies();

    /*!
     * \brief Getter for the IID of the plugin that creates Polygons
     * \return "org.sdsmt.sim.2d.worldObjectComponent.defaults.polygon"
     */
    QString getPluginName(){ return POLYGON_IID; }
};
