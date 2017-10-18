#include "ui/emptysimwindow.h"
#include "ui_emptysimwindow.h"

#include <QSizePolicy>

emptysimwindow::emptysimwindow(visualizerFactory factory, QWidget *parent) :
    Simulator_Ui_If(parent),
    ui(new Ui::emptysimwindow)
{
    ui->setupUi(this);

    _factory = factory;
    _visual = _factory();
    ui->simviewcontainer->addWidget(_visual);
}

emptysimwindow::~emptysimwindow()
{
    delete ui;
}

void emptysimwindow::showMainWindow()
{
    show();
}

void emptysimwindow::robotAddedToSimulation(Robot_Properties* robot)
{
    _visual->modelAddedToScreen(robot->createRobotBaseModel(), _robots++);
    _visual->modelAddedToScreen(robot->createRobotSensorsModel(), _robots++);
}
