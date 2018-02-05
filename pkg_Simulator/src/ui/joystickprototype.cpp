#include "include/ui/joystickprototype.h"
#include "ui_joystickprototype.h"

#include <QDebug>
#include <QMouseEvent>
#include <QPainter>
#include <QScrollBar>
#include <QtMath>
#include <QString>

JoystickPrototype::JoystickPrototype(QWindow *parent) :
    joystickUi(new Ui::JoystickPrototype)
{
    joystickUi->setupUi(this);

    joystickWidget = new JoystickWidget(this);
    joystickUi->gridLayout->addWidget(joystickWidget);

    rotationWidget = new RotationWidget(this);
    joystickUi->verticalLayout->addWidget(rotationWidget);

    connect(joystickWidget, SIGNAL(joystickMoved()), this, SLOT(joystickMoved()));
    connect(rotationWidget, SIGNAL(valueChanged()), this, SLOT(joystickMoved()));

    //initialize joystick keyboard shortcuts, these can be altered by user
    up = Qt::Key_W;
    east = Qt::Key_D;
    west = Qt::Key_A;
    down = Qt::Key_S;
    left = Qt::Key_Z;
    right = Qt::Key_X;   

    //grabKeyboard();
}

void JoystickPrototype::joystickMoved()
{
    joystickMoved(joystickWidget->xVector, joystickWidget->yVector, rotationWidget->value - rotationWidget->maximum()/2, "temp channel");
}

void JoystickPrototype::keyPressEvent(QKeyEvent * event)
{
    if (event->type()==QEvent::KeyPress)
    {
        qDebug() << "key pressed " + event->text();
    }

    int speedButtons = 60;

    if(event->key() == up)
    {
        joystickWidget->yCenter -= speedButtons;
        joystickWidget->m_MouseUp = true;
    }
    else if(event->key() == east)
    {
        joystickWidget->xCenter += speedButtons;
        joystickWidget->m_MouseEast = true;
    }
    else if(event->key() == west)
    {
        joystickWidget->xCenter -= speedButtons;
        joystickWidget->m_MouseWest = true;
    }
    else if(event->key() == down)
    {
        joystickWidget->yCenter += speedButtons;
        joystickWidget->m_MouseDown = true;
    }
    else if(event->key() == left)
    {
        rotationWidget->m_MouseLeft = true;
        rotationWidget->addValue(-speedButtons);
    }
    else if(event->key() == right)
    {
        rotationWidget->m_MouseRight = true;
        rotationWidget->addValue(speedButtons);
    }
    else
        joystickButtonPress(event->key(), "mah channel");
    joystickWidget->update();
}

void JoystickPrototype::keyReleaseEvent(QKeyEvent * event)
{
    if(!event->isAutoRepeat())
    {
        if (event->type()==QEvent::KeyRelease)
        {
            qDebug() << "key released " + event->text();
        }

        if(event->key() == up)
            joystickWidget->m_MouseUp = false;
        else if(event->key() == east)
            joystickWidget->m_MouseEast = false;
        else if(event->key() == west)
            joystickWidget->m_MouseWest = false;
        else if(event->key() == down)
            joystickWidget->m_MouseDown = false;
        else if(event->key() == left)
            rotationWidget->m_MouseLeft = false;
        else if(event->key() == right)
            rotationWidget->m_MouseRight = false;
        else
            joystickButtonRelease(event->key(), "mah channel");
        joystickWidget->update();
        rotationWidget->resetValue();
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
RotationWidget::RotationWidget(QWidget *parent)
    : QScrollBar(Qt::Horizontal, parent)
{
    resetValue();
}

void RotationWidget::resetValue()
{
    if(!m_MouseLeft && !m_MouseRight)
        value = maximum()/2;
    addValue(0);
}

void RotationWidget::addValue(int amount)
{
    value = (value + amount);
    if(value > maximum())
        value = maximum();
    else if(value < 0)
        value = 0;
    qDebug() << QString::number(value);
    setValue(value);
}
void RotationWidget::mouseReleaseEvent(QMouseEvent * event)
{
    Q_UNUSED(event)
    m_MousePressed = false;
    resetValue();
}

/*
void RotationWidget::mousePressEvent(QMouseEvent* event)
{
    Q_UNUSED(event)
    m_MousePressed = true;
}

void RotationWidget::mouseMoveEvent(QMouseEvent *event)
{
    Q_UNUSED(event)
    if (event->type() == QEvent::MouseMove)
    {
        xCenter = event->pos().x();
        yCenter = event->pos().y();
        update();
    }
}*/

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
JoystickWidget::JoystickWidget(QWidget *parent)
    : QOpenGLWidget(parent)
{
}

void JoystickWidget::mousePressEvent(QMouseEvent* event)
{
    Q_UNUSED(event)
    m_MousePressed = true;
}

void JoystickWidget::mouseReleaseEvent(QMouseEvent *event)
{
    Q_UNUSED(event)
    update();
    m_MousePressed = false;
}

void JoystickWidget::mouseMoveEvent(QMouseEvent *event)
{
    Q_UNUSED(event)
    if (event->type() == QEvent::MouseMove)
    {
        xCenter = event->pos().x();
        yCenter = event->pos().y();
        update();
    }
}

void JoystickWidget::setCenter()
{
    xCenter = width()/2;
    yCenter = xCenter;
}

void JoystickWidget::paintEvent(QPaintEvent *event)
{
    if(!m_MousePressed && !m_MouseUp && !m_MouseDown && !m_MouseEast && !m_MouseWest)
    {
        setCenter();
    }

    Q_UNUSED(event)

    radius = width()/2;

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setRenderHint(QPainter::HighQualityAntialiasing);

    //draw background circle white
    QPen outerPen(Qt::black, 2);
    painter.setPen(outerPen);
    QBrush outerBrush(Qt::white);
    painter.setBrush(outerBrush);
    painter.drawEllipse(0, 0, width(), width());

    //draw square lines for referential ease of use
    painter.drawLine(width()/2, 0, width()/2, width()/2 - 60);
    painter.drawLine(width()/2, width()/2 + 60, width()/2, width());
    painter.drawLine(0, width()/2, width()/2 - 60, width()/2);
    painter.drawLine(width()/2 + 60, width()/2, width(), width()/2);

    //draw frontground circle
    qreal centerC = width()/2;
    qreal num = yCenter - centerC;
    qreal den = xCenter - centerC;

    //check inner circle for collision with edges of larger circle
    if(qPow(qPow(den, 2) + qPow(num, 2), .5) > radius/2)
    {
        qreal theta = qAtan(num/den);

        qreal newY = qSin(theta) * (radius/2);
        qreal newX = qCos(theta) * (radius/2);

        if(den > 0)
        {
            xCenter = centerC + newX;
            yCenter = centerC + newY;
        }
        else
        {
            xCenter = centerC - newX;
            yCenter = centerC - newY;
        }

        if(theta > -1.58 && theta < -1.57)
            yCenter = centerC - radius/2;
        if(theta > 1.57 && theta < 1.58)
            yCenter = centerC + radius/2;
    }

    //set vector coordinates and emit signal for movement
    xVector = xCenter - centerC;
    if(yCenter > centerC)
        yVector = 0 - yCenter + centerC;
    else
        yVector = centerC - yCenter;

    joystickMoved();

    //using corner to draw instead of center point
    xCorner = xCenter - radius/2;
    yCorner = yCenter - radius/2;


    painter.drawEllipse(xCorner, yCorner, radius, radius);
}
