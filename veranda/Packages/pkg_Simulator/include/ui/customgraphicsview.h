//! \file
#pragma once

#include <QGraphicsView>
#include <QMouseEvent>
#include <QKeyEvent>

/*!
 * \brief Extension of QGraphicsView to capture events
 * Captures and publishes events for 'w','s','a','d','q', and 'e'
 * as navigation keys, as well as mouse clicks and mouse moves
 */
class CustomGraphicsView : public QGraphicsView
{
    Q_OBJECT

public:
    /*!
     * \brief Constructs the CustomGraphicsView
     * \param[in] parent QWidget parent
     */
    CustomGraphicsView(QWidget* parent = nullptr) :
        QGraphicsView(parent){}

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
