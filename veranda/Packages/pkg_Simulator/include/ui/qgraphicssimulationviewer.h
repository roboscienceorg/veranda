//! \file
#pragma once

#include "interfaces/simulator_visual_if.h"
#include "customgraphicsview.h"

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

namespace Ui {
class qgraphicssimulationviewer;
}

class QGraphicsSimulationViewer : public Simulator_Visual_If
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
    QGraphicsScene* _scene = nullptr;

    //! Group of shapes for the drag to move tool
    QGraphicsItem* _translater;

    //! Group of shapes for the drag to rotate tool
    QGraphicsItem* _rotater;

    //! Group of shapes for all of the click to drag tools
    QGraphicsItemGroup* _tools;

    //! Target viewport
    QRectF _targetView;

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

    /*!
     * \brief Resets the viewport and zoom level; makes the scene if it is null
     */
    void _resetScene();

    /*!
     * \brief Fits the viewer rect around the target view as well as possible by scaling the picture
     * \param targetView Rectangle that should be shown in view
     */
    void _fitInView(const QRectF& targetView);

public:
    /*!
     * \brief Constructs the view widget
     * \param[in] parent QWidget parent
     */
    explicit QGraphicsSimulationViewer(QWidget *parent = 0);

    /*!
     * \brief Destructs the view widget
     */
    ~QGraphicsSimulationViewer();

    void setNavigationEnabled(bool allowed);

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
    void modelMoved(Model *m, const double& dx, const double& dy, const double& dt);

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
    void viewZoom(const int& z);

    /*!
     * \brief Handle viewport shift commands
     * \param[in] x Amount to shift horizontal
     * \param[in] y Amount to shift vertical
     */
    void viewShift(const int& x, const int& y);

private:
    Ui::qgraphicssimulationviewer *ui;
};
