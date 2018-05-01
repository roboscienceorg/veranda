//! \file
#pragma once

#include <QWidget>
#include <QOpenGLWidget>
#include <QScrollBar>
#include <QDebug>
#include "ui/settingspopup.h"


namespace Ui {
class JoystickWidget;
class JoystickPrototype;
class RotationWidget;
}

/*!
 * \brief Widget for displaying a virtual joystick
 * The virtual joystick is rendered using Qt OpenGL, so this extends
 * QOpenGLWidget
 */
class JoystickWidget : public QOpenGLWidget /*Joystickprototype_If*/
{
    Q_OBJECT

public:
    /*!
     * \brief Makes a new JoystickWidget
     * \param[in] parent The QWidget parent
     */
    JoystickWidget(QWidget *parent);

    //! Flag for if the mouse is pressed
    bool m_MousePressed;

    //! Flag for if a key is moving the joystick up
    bool m_MouseUp;

    //! Flag for if a key is moving the joystick down
    bool m_MouseDown;

    //! Flag for if a key is moving the joystick to the right
    bool m_MouseEast;

    //! Flag for if a key is moving the joystick to the left
    bool m_MouseWest;

    //! Calculate the center point of the joystick
    void setCenter();

    //! Joystick x axis value
    double xVector;

    //! Joystick y axis value
    double yVector;

    //! Center of the x axis
    int xCenter;

    //! Center of the y axis
    double yCenter;

protected:
    /*!
     * \brief Mouse press handler
     * \param[in] event The mouse press event
     */
    void mousePressEvent(QMouseEvent * event);

    /*!
     * \brief Mouse release handler
     * \param[in] event The mouse release event
     */
    void mouseReleaseEvent(QMouseEvent * event);

    /*!
     * \brief Redraws the OpenGL canvas
     * \param[in] event The redraw event
     */
    void paintEvent(QPaintEvent *event);

    /*!
     * \brief Mouse move handler
     * \param[in] event The move event
     */
    void mouseMoveEvent(QMouseEvent *event);

private:
    //! Corner of the x axis
    int xCorner;

    //! Corner of the y axis
    int yCorner;

    //! Radius of joystick circle
    int radius;

signals:
    //! Signal that the joystick has moved
    void joystickMoved();
};

/*!
 * \brief Slider widget to represent the z axis of a joystick
 */
class RotationWidget : public QScrollBar
{
    Q_OBJECT

public:
    /*!
     * \brief Constructs a new slider bar
     * \param[in] parent The QWidget parent
     */
    RotationWidget(QWidget *parent);

    //! Flag for if the mouse is down
    bool m_MousePressed;

    //! Flag for if a key is forcing the slider to the left
    bool m_MouseLeft;

    //! Flag for if a key is forcing the slider to the right
    bool m_MouseRight;

    /*!
     * \brief Accumulates an amount on the current value
     * \param[in] amount The amount to add to the value
     */
    void addValue(int amount);

    //! Current value of the slider
    int value;

    //! Value published as z axis
    double zVector;

public slots:
    //! Resets the slider to 0 position
    void resetValue();
};

/*!
 * \brief Widget to display a virtual joystick
 * The joystick contains a draggable node for the x and y axes,
 * and a draggable slider for the z axis. It also contains a text
 * box to set the ROS 2 channel that should be published on and
 * an options menu to set directional hotkeys
 */
class JoystickPrototype : public QWidget
{
    Q_OBJECT

public:
    /*!
     * \brief Creates a new joystick widget
     * \param[in] parent The parent window
     */
    JoystickPrototype(QWindow *parent);

    //! The joystick x, y axes widget
    JoystickWidget *joystickWidget;

    //! The joystick z axis widget
    RotationWidget *rotationWidget;

    /*!
     * \brief Captures the event on close and forwards it as a signal
     * \param[in] e The close event
     */
    void closeEvent(QCloseEvent * e)
    {
        emit joystickClosed();
    }

private slots:
    //! Calculates the joystick x, y information when the user drags it
    void joystickMoved();

    /*!
     * \brief Recieves the new z axis value when it changes
     * \param[in] val The new z axis value
     */
    void rotationMoved(int val);

    //! Opens the hotkey binding menu
    void keysButtonClick();

    //! Enables the widget
    void makeUsable();

    /*!
     * \brief Binds new directional keys
     * \param[in] n Key bound to y axis forward
     * \param[in] e Key bound to x axis backward
     * \param[in] s Key bound to y axis backward
     * \param[in] w Key bound to x axis forward
     * \param[in] l Key bound to z axis backward
     * \param[in] r Key bound to z axis forward
     * \param[in] speed Weight of keypresses on direction
     */
    void settingsChanged(int n, int e, int s, int w, int l, int r, int speed);

protected:
    /*!
     * \brief Handles key release events
     * \param[in] event The key up event
     */
    virtual void keyPressEvent(QKeyEvent * event);

    /*!
     * \brief Handles key press events
     * \param[in] event The key down event
     */
    virtual void keyReleaseEvent(QKeyEvent * event);

    //! Key binding for y axis forward
    int up;

    //! Key binding for x axis forward
    int east;

    //! Key binding for x axis backward
    int west;

    //! Key binding for y axis backward
    int down;

    //! Key binding for z axis backward
    int left;

    //! Key binding for x axis forward
    int right;

    //! Weight of key presses on direction
    int speedButtons;

private:
    //! The autogenerated UI objects
    Ui::JoystickPrototype *joystickUi;

signals:
    /*!
     * \brief Signals that a joystick axis value changed
     * \param[in] x X axis value
     * \param[in] y Y axis value
     * \param[in] z Z axis value
     * \param[in] channel ROS 2 topic for joystick
     */
    void joystickMoved(double x, double y, double z, QString channel);

    /*!
     * \brief Signals that a joystick button was pressed
     * \param[in] key The button number
     * \param[in] channel ROS 2 topic for joystick
     */
    void joystickButtonPress(int key, QString channel);

    /*!
     * \brief Signals that a joystick button was released
     * \param[in] key The button number
     * \param[in] channel ROS 2 topic for joystick
     */
    void joystickButtonRelease(int key, QString channel);

    //! Signal that the joystick window was closed
    void joystickClosed();

};
