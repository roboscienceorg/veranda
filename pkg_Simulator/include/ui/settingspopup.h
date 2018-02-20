#ifndef SETTINGSPOPUP_H
#define SETTINGSPOPUP_H

#include <QWidget>
#include <QWindow>

namespace Ui {
class settingspopup;
}

class settingspopup : public QWidget
{
    Q_OBJECT

public:
    explicit settingspopup(QWindow *parent);
    void closeEvent(QCloseEvent *)
    {
        emit settingsClosed();
    }

private:
    Ui::settingspopup *settingsUi;

private slots:
    void saveButtonClick();

signals:
    void settingsClosed();
    void settingsSaved(int, int, int, int, int, int, int);
};

#endif // SETTINGSPOPUP_H
