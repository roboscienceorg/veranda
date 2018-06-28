//! \file
#pragma once

#include <QObject>
#include <QVector>
#include <QDebug>
#include <QColor>

#include <Box2D/Box2D.h>

#include "dllapi.h"

/*!
 * \brief Group of settings that can be added to models to hint at how they should be drawn
 */
struct veranda_API DrawHint
{
    //! Color hint for outline
    QColor outlineColor;

    //! Style hint for outline
    Qt::PenStyle outlineStyle;

    //! Color hint for fill
    QColor fillColor;

    //! Sytle hint for fill
    Qt::BrushStyle fillStyle;

    //! True hints that model should inherit all other hints from parent model
    bool inherit;

    /*!
     * \brief Not equal comparator for DrawHints
     * \param[in] other DrawHint to compare to
     * \return true of any of the DrawHint values don't match those of 'other'
     */
    bool operator != (const DrawHint& other) {
        return outlineColor != other.outlineColor || fillColor != other.fillColor ||
               outlineStyle != other.outlineStyle || fillStyle != other.fillStyle ||
               inherit != other.inherit;
    }
};

/*!
 * \brief Contains shapes to be drawn together
 *
 * The model is a method for grouping a set of
 * shapes together. They can contain shapes and children
 * models, all of which will be drawn together. Signals
 * from the model are emitted whenever it or one of its
 * chilren changes; this can be used to trigger redrawing
 * an image based on the model.
 *
 * Children and shapes added to the model are only references;
 * they DO NOT become 'owned' by the model, and the model
 * will not delete them at any time.
 *
 * This model does not directly access its children and shapes
 * after they are added; but observers of the model may, so the
 * Model owner should remove shapes and children model before
 * deleting them using removeChildren() and removeShapes()
 *
 * It is intended that the transform of a child model indicates
 * its location relative to its parent's location
 *
 * Similarly, the points and angles used used to locate the b2Shape*
 * objects held by the model are intended to be relative to the 'position'
 * of the model itself
 */
class veranda_API Model : public QObject
{
    Q_OBJECT

    //! List of children models
    QVector<Model*> _children;

    //! List of shapes in this model
    QVector<b2Shape*> _shapes;

    //! Parent model of this model
    Model* _parent = nullptr;

    //! Current x coordinate of model
    double _x=0;

    //! Current y coordinate of model
    double _y=0;

    //! Current rotation of model (Degrees)
    double _theta=0;

    //! Current drawhint for the model
    DrawHint _hint = {QColor(0, 0, 0), Qt::SolidLine, QColor(0, 0, 0), Qt::NoBrush, true};

    /*!
     * \brief Update the parent model of this mode; if drawhint is dependent on it, signal the hint changed
     * \param parent New model (or nullptr) to be parent
     */
    void _setParent(Model* parent)
    {
        _parent = parent;
        if(_hint.inherit) hintChanged(this);
    }

signals:
    /*!
     * \brief Indicates one of two things about the model
     *  * One or more of its shapes was changed, added, or removed
     *  * One or more of its children were added or removed (NOT Changed!)
     *
     * \param[in] mdl Pointer to the model that changed
     */
    void modelChanged(Model* mdl);

    /*!
     * \brief Indicates that the model's transform changed
     * \param[in] mdl Pointer to the model that changed
     * \param[in] dx Delta x
     * \param[in] dy Delta y
     * \param[in] dt Delta angle (degrees)
     */
    void transformChanged(Model* mdl, double dx, double dy, double dt);

    /*!
     * \brief Indicates that the model's drawhint was set
     * \param[in] mdl Pointer to the model that changed
     */
    void hintChanged(Model* mdl);

public:

    /*! Construct a new model and initialize its children models and shapes
     *
     * \param[in] children Children models of this model
     * \param[in] shapes Shapes represented in this model (Default empty list)
     * \param[in] parent QObject parent of the object (Default nullptr)
     */
    Model(QVector<Model*> children, QVector<b2Shape*> shapes = {}, QObject* parent = nullptr) : QObject(parent)
    {
        addChildren(children);
        addShapes(shapes);
    }

    /*! Construct a new model and initialize its shapes
     *
     * \param[in] shapes Shapes represented in this model (Default empty list)
     * \param[in] parent QObject parent of the object (Default nullptr)
     */
    Model(QVector<b2Shape*> shapes = {}, QObject* parent = nullptr) : QObject(parent)
    {
        addShapes(shapes);
    }

    //! Empty destructor
    ~Model(){}

    /*!
     * \brief Get the list of shapes held by the model
     * \return const QVector<b2Shape*>& - Shapes represented by the model
     */
    const QVector<b2Shape*>& shapes() const { return _shapes; }

    /*!
     * \brief Get the list of children models of this model
     * \return const QVector<Model*>& - Children of this model
     */
    const QVector<Model*>& children() const { return _children; }

    /*!
     * \brief Forces the modelChanged() signal to be emitted
     * Use this if you change the parameters of the b2Shape*'s
     * represented in the model, but don't add or remove any. (Like if
     * you change the center point of a b2CircleShape, or add more edges
     * on a b2PolygonShape)
     */
    void forceDraw()
    {
        modelChanged(this);
    }

    /*!
     * \brief Set the location of the model relative to its parent (or the world, if no parent)
     * \param[in] x The x coordinate of the model
     * \param[in] y The y coordinate of the model
     * \param[in] theta The rotation of the model in degrees
     */
    void setTransform(const double& x, const double& y, const double& theta)
    {
        double dx = x-_x, dy = y-_y, dt = theta-_theta;
        _x = x;
        _y = y;
        _theta = theta;

        //qDebug() << "Model adjusted by" << _x <<" " << _y << " " << _theta;

        //if(abs(dx) > 0.001 || abs(dy) > 0.001 || abs(dt) > 0.001)
            transformChanged(this, dx, dy, dt);
    }

    /*!
     * \brief Obtains the last transformation values set for the model
     * \param[out] x x coordinate
     * \param[out] y y coordinate
     * \param[out] theta angle (degrees)
     */
    void getTransform(double& x, double& y, double& theta)
    {
        x = _x;
        y = _y;
        theta = _theta;
    }

    /*!
     * \brief Adds new children models to the model
     * If a child is added multiple times (either in 1 call, or across
     * multiple) it will only be added 1 time
     * \param[in] newChildren A list of pointers to any number of models to add as children to this one
     */
    void addChildren(QVector<Model*> newChildren)
    {
        if(newChildren.size())
        {
            for(Model* s : newChildren)
                if(!_children.contains(s))
                {
                    _children.push_back(s);
                    s->setParent(this);
                }
            modelChanged(this);
        }
    }

    /*!
     * \brief Removes a set of children of this model
     * If a child to be removed is not actually a child of this model, it is
     * ignored
     * \param[in] oldChildren List of children to remove from this model
     */
    void removeChildren(QVector<Model*> oldChildren)
    {
        if(oldChildren.size())
        {
            for(Model* s : oldChildren)
            {
                _children.removeAll(s);
                disconnect(s, 0, this, 0);
                s->setParent(nullptr);
            }
            modelChanged(this);
        }
    }

    /*!
     * \brief Adds new shapes to the model
     * If a shape is added multiple times (either in 1 call, or across
     * multiple) it will only be added 1 time
     * \param[in] newShapes A list of pointers to any number of shapes to add
     */
    void addShapes(QVector<b2Shape*> newShapes)
    {
        if(newShapes.size())
        {
            for(b2Shape* s : newShapes)
                if(!_shapes.contains(s))
                    _shapes.push_back(s);
            modelChanged(this);
        }
    }

    /*!
     * \brief Removes a set of shapes from this model
     * If a shape to be removed is not actually a in this model, it is
     * ignored
     * \param[in] oldShapes List of shapes to remove from this model
     */
    void removeShapes(QVector<b2Shape*> oldShapes)
    {
        if(oldShapes.size())
        {
            for(b2Shape* s : oldShapes)
                _shapes.removeAll(s);

            modelChanged(this);
        }
    }

    /*!
     * \brief Sets the drawhint for the model and triggers the associated signal
     * \param[in] newHint The new drawhint
     */
    void setDrawHint(DrawHint newHint)
    {
        if(_hint != newHint)
        {
            _hint = newHint;
            hintChanged(this);
        }
    }

    /*!
     * \brief Gets the drawhint set for the model
     * \return DrawHint - The current hint for the model
     */
    DrawHint getDrawHint()
    {
        return _hint;
    }

    /*!
     * \brief Gets the parent model of this model (may be null)
     * \return Pointer to parent model or nullptr
     */
    Model* getParent()
    {
        return _parent;
    }
};
