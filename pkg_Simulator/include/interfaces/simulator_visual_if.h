//! \file
#pragma once

#include <QWidget>

#include <sdsmt_simulator/model.h>
#include <sdsmt_simulator/world_object.h>

/*!
 * \brief Interface for the widget which displays the simulation
 * This interface should be used to create widgets for the UI which
 * display the simulation. More generally, this interface should be
 * able to take a set of Models and display them, optionally allowing
 * the user to click and drag to move and rotate them
 */
class Simulator_Visual_If : public QWidget
{
    Q_OBJECT

public:
    //! Enum for different methods of drawing
    enum DrawLevel{Solid, Transparent, Off};

    /*!
     * \brief Constructs a new visualizer widget
     * \param[in] parent QObject parent
     */
    Simulator_Visual_If(QWidget* parent = nullptr) : QWidget(parent){}

    /*!
     * \brief Sets the bounds of the simulation shown in the viewport
     * \param[in] xMin Minimum x location to show
     * \param[in] xMax Maximum x location to show
     * \param[in] yMin Minimum y location to show
     * \param[in] yMax Maximum y location to show
     */
    virtual void setWorldBounds(double xMin, double xMax, double yMin, double yMax) = 0;
signals:
    /*!
     * \brief Signals that the user clicked on an object to select it
     * \param[in] id The id associated with the models clicked
     */
    void userSelectedObject(object_id id);

    /*!
     * \brief Signals that the user is dragging and moving an object
     * \param[in] oId Id associated with the models being dragged
     * \param[in] dx Distance to move in the x direction
     * \param[in] dy Distnace to move in the y direction
     */
    void userDragMoveObject(object_id oId, double dx, double dy);

    /*!
     * \brief Signals that the user is dragging and rotating an object
     * \param[in] oId Id associated with the models being dragged
     * \param[in] dt Delta angle in degrees
     */
    void userDragRotateObject(object_id oId, double dt);

public slots:
    /*!
     * \brief Slot adding an object to the view
     * Indicates that a set of Models should be drawn on the screen and associated
     * with some id. Models added should be drawn until they are removed with
     * objectRemovedFromScreen. The changed signals emitted by the Models should
     * be used to update the view when they change or move
     * \param[in] object Group of models that makes up the object
     * \param[in] oId Id to associate with the group of models
     */
    virtual void objectAddedToScreen(QVector<Model*> object, object_id oId) = 0;
    /*!
     * \brief Slot removing an object from the view
     * \param[in] id The identifier of the group of models that should be removed
     */
    virtual void objectRemovedFromScreen(object_id id) = 0;

    /*!
     * \brief Slot setting the draw mode for an object
     * \param[in] id Identifier of the group of models to set draw level for
     * \param[in] dl Draw level to set
     */
    virtual void objectDrawLevelSet(object_id id, DrawLevel dl) = 0;

    /*!
     * \brief Slot indicating that a group of models should be drawn as the 'selected' object
     * \param[in] id Id of the group of Models selected
     */
    virtual void objectSelected(object_id id) = 0;

    //! Slot indicating that no object should be drawn as 'selected'
    virtual void nothingSelected() = 0;

    /*!
     * \brief Slot disabling or enabling tools
     * The View may allow the user to drag objects to move and rotate them; this method
     * is used to enable or disable that functionality if it exists
     * \param[in] enabled If true, the view should allow the user to use move/rotate functionality
     */
    virtual void setToolsEnabled(bool enabled) = 0;
};
