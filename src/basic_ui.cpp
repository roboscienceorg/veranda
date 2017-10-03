#include "basic_ui.h"

BasicUi::BasicUi(visualizerFactory visualMaker, QObject *parent) : Simulator_Ui_If(parent), _visFactory(visualMaker)
{

}

void BasicUi::robotAddedToSimulation(Robot_Properties* robot, robot_id rId)
{

}

void BasicUi::robotRemovedFromSimulation(robot_id rId)
{

}

void BasicUi::robotSelected(robot_id rId)
{

}

void BasicUi::mapSetInSimulation()
{

}

void BasicUi::physicsTickChanged(double rate_hz, double duration_s)
{

}

void BasicUi::physicsStopped()
{

}

void BasicUi::physicsStarted()
{

}

void BasicUi::errorMessage(QString error)
{

}

void BasicUi::showMainWindow()
{

}
