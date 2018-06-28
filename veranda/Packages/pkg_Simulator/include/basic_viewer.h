//! \file
#pragma once

#include "interfaces/simulator_visual_if.h"

#include <QMap>
#include <QTimer>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsItem>
#include <QLayout>
#include <QColor>
#include <QDebug>
#include <QKeyEvent>

#include <Box2D/Box2D.h>
#include <veranda/model.h>

/*!
 * \brief Extension of QGraphicsView to capture events
 * Captures and publishes events for 'w','s','a','d','q', and 'e'
 * as navigation keys, as well as mouse clicks and mouse moves
 *
 * \todo Add 'follow' feature
 * \todo Add 'zoom to fill' feature to zoom in on a specific object
 */
class CustomGraphicsView : public QGraphicsView
{
    Q_OBJECT

public:
    /*!
     * \brief Constructs the CustomGraphicsView
     * \param[in] scene QGraphicsScene to draw a view of
     * \param[in] parent QWidget parent
     */
    CustomGraphicsView(QGraphicsScene* scene, QWidget* parent = nullptr) :
        QGraphicsView(scene, parent){}

signals:
    /*!
     * \brief Signals a mouse moved event
     * \param[in] event The mouse moved event
     */
    void mouseMoved(QMouseEvent* event);

    /*!
     * \brief Signals a mouse press event
     * \param[in] event The mouse pressed event
     */
    void mousePress(QMouseEvent* event);

    /*!
     * \brief Signals a mouse released event
     * \param[in] event The mouse released event
     */
    void mouseRelease(QMouseEvent* event);

    /*!
     * \brief Signals that one of the zoom in,out keys was pressed
     * \param[in] z Amount to zoom (-1 or 1)
     */
    void zoomTick(int z);

    /*!
     * \brief Signals that navigation keys were pressed
     * \param[in] x Amount to move horizontal (-1, 0, or 1)
     * \param[in] y Amount to move vertial (-1, 0, 1)
     */
    void screenShift(int x, int y);

private:
    /*!
     * \brief Capture and forward mouse move events
     * \param[in] event The mouse move event
     */
    void mouseMoveEvent(QMouseEvent* event)
    {
        mouseMoved(event);
    }

    /*!
     * \brief Capture and forward mouse press events
     * \param[in] event The mouse press event
     */
    void mousePressEvent(QMouseEvent* event)
    {
        mousePress(event);
    }

    /*!
     * \brief Capture and forward mouse release events
     * \param[in] event The mouse release event
     */
    void mouseReleaseEvent(QMouseEvent* event)
    {
        mouseRelease(event);
    }

    /*!
     * \brief Capture keypress events and check for navigation in the viewport
     * \param[in] event The keypress event
     */
    void keyPressEvent(QKeyEvent* event)
    {
        switch(event->key())
        {
            case Qt::Key_W: screenShift(0, -1); break;
            case Qt::Key_A: screenShift(-1, 0); break;
            case Qt::Key_S: screenShift(0, 1); break;
            case Qt::Key_D: screenShift(1, 0); break;
            case Qt::Key_Q: zoomTick(1); break;
            case Qt::Key_E: zoomTick(-1); break;
        }
    }
};

/*!
 * \brief Default viewing widget fulfilling Simulator_Visual_If
 * This is the default widget used to draw WorldObjectComponents
 * on a viewport. The widget uses the QGraphics Framework to draw
 * objects. This was chosen because it is optimized for 2D graphics and
 * because it allows creating a tree hierarchy of objects to place
 * elements relative to each other.
 *
 * Every object id is associated with a top-level QGraphicsItemGroup. That
 * group contains the drawn shapes for all of the models associated with that id.
 * Models are drawn by adding all shapes for the model to a QGraphicsItemGroup along
 * with the QGraphicsItemGroup used to drawn any children models. When any model on any
 * level is moved or modified, as few QGraphicsItems are updated as possible
 * to keep the view accurate
 */
class BasicViewer : public Simulator_Visual_If
{
    Q_OBJECT

    /*!
     * Color to use when drawing things that are selected
     */
    const QColor SELECTED_COLOR = QColor(50, 163, 103);

    /*!
     * Constant scaling factor between simulation and drawing.
     * This is not implemented as just a QGraphicsView::scale because
     * that reduces resolution when the objects drawn are very small
     */
    constexpr static int64_t WORLD_SCALE = 20;

    //! The models provided for each object id
    QMap<object_id, QVector<Model*>> _models;

    //! Top level QGraphicsItemGroups for each set of models
    QMap<object_id, QGraphicsItemGroup*> _topShapes;

    //! The QGraphicsItemGroup* for each model, whether its a submodel or not
    QMap<Model*, QGraphicsItemGroup*> _shapes;

    //! Record of the pens and brushes being used to draw models
    QMap<Model*, QPair<QPen, QBrush>> _modelPensBrushes;

    //! Mapping of QGraphicsItem to object id
    QMap<QGraphicsItem*, object_id> _shapeToObject;

    //! Mapping of Model to object_id
    QMap<Model*, object_id> _modelToObject;

    //! Record of which Models are children of which other Models
    QMap<Model*, QVector<Model*>> _modelChildren;

    //! Record of what drawlevels are set for each object
    QMap<object_id, DrawLevel> _drawLevels;

    //! Which object is currently selected; 0 represents none
    object_id _currSelection = 0;

    //! The graphics view showing the scene and capturing events
    CustomGraphicsView* _viewer;

    //! The QGraphicsScene holding all drawn objects
    QGraphicsScene* _scene;

    //! Layout to put the QGraphicsView in
    QLayout* _children;

    //! Group of shapes for the drag to move tool
    QGraphicsItem* _translater;

    //! Group of shapes for the drag to rotate tool
    QGraphicsItem* _rotater;

    //! Group of shapes for all of the click to drag tools
    QGraphicsItemGroup* _tools;

    /*!
     * \brief Converts a b2Shape pointer to a QGraphicsItem
     * \param[in] s The b2Shape to draw
     * \param[in] itemParent The parent item of the new QGraphicsItem
     * \return A QGraphicsItem representation of the b2Shape
     */
    QGraphicsItem* _drawb2Shape(b2Shape* s, QGraphicsItem* itemParent = nullptr);

    /*!
     * \brief Constructs a QGraphicsItemGroup for all of the shapes in a model (but not its children)
     * \param[in] m The model to draw
     * \return A QGraphicsItemGroup with all the shapes in the Model
     */
    QGraphicsItemGroup *_drawModel(Model* m);

    /*!
     * \brief Rescales the view based on the physical height and width (Like when the window size changes)
     */
    void _rescale();

    /*!
     * \brief Gets the alpha value that should be used to draw a model
     * \param[in] m The model to query for
     * \return 0-255
     */
    uint8_t _getAlpha(Model* m);

    /*!
     * \brief Recursively sets the pen and brush for a model and its children
     * \param[in] m The model to update
     */
    void _updateColoring(Model* m);

    //! Flag for if click drag tools are allowed
    bool _toolsEnabled = true;

    //! Flag for if the user is currently dragging to move something
    bool _draggingTranslate = false;

    //! Flag for if the user is currently dragging to rotate something
    bool _draggingRotate = false;

    //! Last location recorded for click to drag
    QPointF _dragStart;

    /*!
     * \brief Creates the drag-to-move tool
     * \return A QGraphicsGroup with all the shapes for the tool
     */
    QGraphicsItem* _makeTranslater();

    /*!
     * \brief Creates the drag-to-rotate tool
     * \return A QGraphicsGroup with all the shapes for the tool
     */
    QGraphicsItem* _makeRotater();

    /*!
     * \brief Creates arrow prongs for the click-drag tools
     *
     * \param[in] pointx X Location of the tip of the arrow
     * \param[in] pointy Y Location of the tip of the arrow
     * \param[in] angle Angle the arrow points at (degrees)
     * \param[in] p Pen to draw the arrow outline
     * \param[in] b Brush to draw the arrow interior
     *
     * \return A QGraphicsGroup with all the shapes for the arrow
     */
    QGraphicsItem* _makeArrow(double pointx, double pointy, double angle, QPen p, QBrush b);

    /*!
     * \brief Moves the tool pallatte to near the lower right corner of the selected object
     */
    void _placeTools();

public:
    /*!
     * \brief Constructs the view widget
     * \param[in] parent QWidget parent
     */
    BasicViewer(QWidget* parent = nullptr);

    /*!
     * \brief Sets the bounds of the world in the view
     * \param[in] xMin Min x coordinate shown
     * \param[in] xMax Max x coordinate shown
     * \param[in] yMin Min y coordinate shown
     * \param[in] yMax Max y coordinate shown
     */
    void setWorldBounds(double xMin, double xMax, double yMin, double yMax);

    /*!
     * \brief Sets the bounds of the world in the view
     * \param[in] rect Rectangle to bound view to
     */
    void setWorldBounds(QRectF rect);

public slots:
    void objectAddedToScreen(QVector<Model *> objects, object_id id) override;
    void objectRemovedFromScreen(object_id id) override;
    void objectDrawLevelSet(object_id id, DrawLevel level) override;
    void objectSelected(object_id id) override;
    void nothingSelected() override;
    void setToolsEnabled(bool enabled);

private slots:
    /*!
     * \brief Listener for models moving
     * \param[in] m Model that moved
     * \param[in] dx Delta movement in x direction
     * \param[in] dy Delta movement in y direction
     * \param[in] dt Delta rotation (degrees)
     */
    void modelMoved(Model* m, double dx, double dy, double dt);

    /*!
     * \brief Listener for models changing
     * \param[in] m The model that changed
     */
    void modelChanged(Model* m);

    /*!
     * \brief Listener for model draw hint changing
     * \param[in] m The model that had a draw hint change
     */
    void modelHinted(Model* m);

    /*!
     * \brief Undraws a model, stops tracking all its information, and removes the model's children
     * \param[in] m The model to remove
     */
    void removeModel(Model* m);

    /*!
     * \brief Adds a model, starts tracking its information, and adds all its children
     * \param[in] m The model to add
     * \param[in] id The id of the object the model is part of
     * \return The QGraphicsItem which contains all shapes for the model and its children
     */
    QGraphicsItem* addModel(Model* m, object_id id);

    /*!
     * \brief Handle mouse presses
     * \param[in] event The mouse press event
     */
    void viewMousePress(QMouseEvent *event);

    /*!
     * \brief Handle mouse releases
     * \param[in] event The mouse release event
     */
    void viewMouseRelease(QMouseEvent* event);

    /*!
     * \brief Handle mouse movements
     * \param[in] event The mouse move event
     */
    void viewMouseMove(QMouseEvent* event);

    /*!
     * \brief Handle widget resize events
     * \param[in] event The resize event
     */
    void resizeEvent(QResizeEvent* event);

    /*!
     * \brief Handle zoom in,out commands
     * \param[in] z The direction to zoom
     */
    void viewZoom(int z);

    /*!
     * \brief Handle viewport shift commands
     * \param[in] x Amount to shift horizontal
     * \param[in] y Amount to shift vertical
     */
    void viewShift(int x, int y);
};
