#include "ui/settingspopup.h"
#include "ui_settingspopup.h"

settingspopup::settingspopup(QWindow *parent) :
    settingsUi(new Ui::settingspopup)
{
    settingsUi->setupUi(this);
    connect(settingsUi->saveButton, SIGNAL (released()), this, SLOT (saveButtonClick()));
}

void settingspopup::saveButtonClick()
{
    int north = QKeySequence::fromString(settingsUi->northLineEdit->text())[0];
    int east = QKeySequence::fromString(settingsUi->eastLineEdit->text())[0];
    int south = QKeySequence::fromString(settingsUi->southLineEdit->text())[0];
    int west = QKeySequence::fromString(settingsUi->westLineEdit->text())[0];
    int left = QKeySequence::fromString(settingsUi->leftLineEdit->text())[0];
    int right = QKeySequence::fromString(settingsUi->rightLineEdit->text())[0];
    int speed = settingsUi->speedLineEdit->text().toInt();

    settingsSaved(north, east, south, west, left, right, speed);
    settingsClosed();
}
