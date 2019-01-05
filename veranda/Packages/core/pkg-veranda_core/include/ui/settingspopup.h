//! \file
#pragma once

#include <QWidget>
#include <QWindow>

namespace Ui {
class settingspopup;
}

/*!
 * \brief Dialog for mapping joystick hotkeys
 */
class settingspopup : public QWidget
{
    Q_OBJECT

public:
    /*!
     * \brief Creates a new settings popup dialog
     * \param[in] parent Parent window of the dialog
     */
    explicit settingspopup(QWindow *parent);

    /*!
     * \brief Listens for the close event of the window and forwards it in a signal
     * \param[in] e The close event happening
     */
    void closeEvent(QCloseEvent *e)
    {
        emit settingsClosed();
    }

private:
    //! The auto-generated UI object
    Ui::settingspopup *settingsUi;

private slots:
    //! Handler for the save button
    void saveButtonClick();

signals:
    //! Signal that the dialog is closing
    void settingsClosed();

    /*!
     * \brief Signals that the options were saved
     * \param[in] n Key bound to y axis forward
     * \param[in] e Key bound to x axis backward
     * \param[in] s Key bound to y axis backward
     * \param[in] w Key bound to x axis forward
     * \param[in] l Key bound to z axis backward
     * \param[in] r Key bound to z axis forward
     * \param[in] speed Weight of keypresses on direction
     */
    void settingsSaved(int n, int e, int s, int w, int l, int r, int speed);
};
