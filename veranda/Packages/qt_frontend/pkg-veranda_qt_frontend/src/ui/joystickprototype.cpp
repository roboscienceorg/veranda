#include "ui/joystickprototype.h"
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
    connect(rotationWidget, SIGNAL(valueChanged(int)), this, SLOT(rotationMoved(int)));
    connect(joystickUi->setKeys, SIGNAL (released()), this, SLOT (keysButtonClick()));

    //initialize joystick keyboard shortcuts, these can be altered by user
    up = Qt::Key_W;
    east = Qt::Key_D;
    west = Qt::Key_A;
    down = Qt::Key_S;
    left = Qt::Key_Z;
    right = Qt::Key_X;
    speedButtons = 60;

    //initialize actions to false so joystick starts on center point
    joystickWidget->m_MousePressed = false;
    joystickWidget->m_MouseUp = false;
    joystickWidget->m_MouseEast = false;
    joystickWidget->m_MouseWest = false;
    joystickWidget->m_MouseDown = false;
    rotationWidget->m_MouseLeft = false;
    rotationWidget->m_MouseRight = false;


    setMinimumSize(size());
    setMaximumSize(size());
}

void JoystickPrototype::keysButtonClick()
{
    this->setEnabled(false);

    //create window and settings popup, link them
    QWindow *jWindow = new QWindow();
    settingspopup *settings = new settingspopup(jWindow);

    //Settings popup button signals and slots
    connect(settings, SIGNAL(settingsSaved(int, int, int, int, int, int, int)), this, SLOT(settingsChanged(int, int, int, int, int, int, int)));
    connect(settings, SIGNAL(settingsClosed()), this, SLOT(makeUsable()));

    connect(this, &JoystickPrototype::joystickClosed, settings, &settingspopup::deleteLater);
    connect(settings, &settingspopup::settingsClosed, settings, &settingspopup::deleteLater);
    connect(settings, &settingspopup::destroyed, jWindow, &QWindow::deleteLater);

    //show this popup
    settings->show();
}

void JoystickPrototype::makeUsable()
{
    this->setEnabled(true);
}

void JoystickPrototype::settingsChanged(int n, int e, int s, int w, int l, int r, int speed)
{
    up = n;
    east = e;
    west = w;
    down = s;
    left = l;
    right = r;

    speedButtons = speed;
    this->setEnabled(true);
}

void JoystickPrototype::rotationMoved(int val)
{
    joystickUi->setChannel->clearFocus();

    //qDebug() << "joystick moved " << joystickWidget->xVector << " " << joystickWidget->yVector << " " << val - rotationWidget->maximum()/2;

    joystickMoved(joystickWidget->xVector, joystickWidget->yVector,
                  double(val - rotationWidget->maximum()/2)/rotationWidget->maximum(),
                  joystickUi->setChannel->text());
}

void JoystickPrototype::joystickMoved()
{
    joystickUi->setChannel->clearFocus();

    //qDebug() << "joystick moved " << joystickWidget->xVector << " " << joystickWidget->yVector << " " << rotationWidget->value - rotationWidget->maximum()/2;

    joystickMoved(joystickWidget->xVector, joystickWidget->yVector,
                  rotationWidget->value - rotationWidget->maximum()/2,
                  joystickUi->setChannel->text());
}

void JoystickPrototype::keyPressEvent(QKeyEvent * event)
{
    //Don't trigger on keypresses in the text input
    if(joystickUi->setChannel->hasFocus())
    {
        if(event->key() == Qt::Key_Enter || event->key() == Qt::Key_Return)
            joystickUi->setChannel->clearFocus();
        else
            return;
    }

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
        joystickButtonPress(event->key(), joystickUi->setChannel->text());
    joystickWidget->update();
}

void JoystickPrototype::keyReleaseEvent(QKeyEvent * event)
{
    //Don't trigger on keypresses in the text input
    if(joystickUi->setChannel->hasFocus()) return;

    if(!event->isAutoRepeat())
    {
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
            joystickButtonRelease(event->key(), joystickUi->setChannel->text());
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
    connect(this, SIGNAL(sliderReleased()), this, SLOT(resetValue()));
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
    setValue(value);
}

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

    //Scale values to [-1, 1]
    xVector /= (radius*0.5);
    yVector /= (radius*0.5);

    joystickMoved();

    //using corner to draw instead of center point
    xCorner = xCenter - radius/2;
    yCorner = yCenter - radius/2;


    painter.drawEllipse(xCorner, yCorner, radius, radius);
}
