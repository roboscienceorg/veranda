#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

namespace Ui {
class MainWindow;
class Settings;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    int speed;
    bool play;
    bool record;
private slots:
    void simModeButtonClick();
    void mapModeButtonClick();
    void robotModeButtonClick();
    void showBuildObjectsButtonClick();
    void showMenuButtonClick();
    void playSimButtonClick();
    void speedSimButtonClick();
    void recordSimButtonClick();
    void chooseMapButtonClick();
private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
