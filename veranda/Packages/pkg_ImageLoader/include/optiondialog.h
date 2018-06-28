//! \file
#pragma once

#include <QDialog>
#include <QLineEdit>
#include <QFormLayout>
#include <QDialogButtonBox>
#include <QBoxLayout>
#include <QLabel>
#include <QDoubleValidator>
#include <QIntValidator>
#include <QPushButton>
#include <QHBoxLayout>
#include <QPalette>
#include <QColorDialog>

#include <limits>

/*!
 * \brief Options dialog for loading image files
 * When the user converts image files to obstacle maps, there are a number
 * of parameters they can set. This class creates an option dialog that can
 * be used to choose those options.
 * * Pixels/Meter horizontal
 * * Pixels/Meter vertical
 * * Black/White intensity threshold
 * * "Straightness" factor
 *
 * The user can enter either pixels/meter in each direction, or just
 * enter how many meters the image is in that direction, and px/m will be
 * calculated. Entering a value in either field will update the corresponding field as well.
 */
class ImageOptions : public QDialog
{
    Q_OBJECT

    //! Editor for px/m horizontal
    QLineEdit* _pxpmx;

    //! Editor for px/m vertical
    QLineEdit* _pxpmy;

    //! Editor for meters horizontal
    QLineEdit* _mx;

    //! Editor for meters vertical
    QLineEdit* _my;

    //! Editor for intensity threshold
    QLineEdit* _color;

    //! Editor for straightness value
    QLineEdit* _crossThresh;

    //! Button to open color editor
    QPushButton* _colorButton;

    //! Lable to display chosen color
    QLabel* _colorLabel;

    //! Width of the image in pixels
    uint64_t _width;

    //! Height of the image in pixels
    uint64_t _height;

    //! Currently selected color
    QColor _drawColor = QColor(0, 0, 0);

private slots:
    //! Uses the entered pixels per meter values to compute the width and height in meters
    void constrain1()
    {
        double pxmx = _pxpmx->text().toDouble();
        double pxmy = _pxpmy->text().toDouble();

        _mx->setText(QString::number(_width / pxmx));
        _my->setText(QString::number(_height / pxmy));
    }

    //! Uses the entered width and height in meters to compute the pixels/meter in both directions
    void constrain2()
    {
        double mx = _mx->text().toDouble();
        double my = _my->text().toDouble();

        _pxpmx->setText(QString::number(_width / mx));
        _pxpmy->setText(QString::number(_height / my));
    }

    /*!
     * \brief Displays the currently selected color using the color label
     */
    void updateColorLabel()
    {
        QPalette palette = _colorLabel->palette();
        palette.setColor(_colorLabel->backgroundRole(), _drawColor);
        _colorLabel->setAutoFillBackground(true);
        _colorLabel->setPalette(palette);
    }

    /*!
     * \brief Prompt the user to choose a color, and then update the label
     */
    void getNewColor()
    {
        _drawColor = QColorDialog::getColor(_drawColor, this, "Drawing Color");
        updateColorLabel();
    }
public:
    /*!
     * \brief Builds the dialog box for a specific size image
     * The meters and pixels line editors are bounded to positive double values.
     * The color threshold is bounded to [0, 255]. The straightness value is
     * bounded to positive integer values (All coordinates are integers, so all cross products will be as well)
     *
     * The width/height in meters lines are tied to the px/m lines on edit and vice versa so that
     * the values are accurate at all times.
     *
     * \param[in] imageWidth Width of the image in pixels
     * \param[in] imageHeight Height of the image in pixels
     * \param[in] parent QWidget parent of the dialog box
     */
    ImageOptions(uint64_t imageWidth=0, uint64_t imageHeight=0, QWidget* parent=nullptr) :
        QDialog(parent), _width(imageWidth), _height(imageHeight)
    {
        QFormLayout* form = new QFormLayout(this);
        setLayout(form);

        QString width = QString::number(imageWidth);
        QString height = QString::number(imageHeight);

        form->addRow(new QLabel("Color Options", this));
        form->addRow(QString("Black/White Threshold:"), _color = new QLineEdit("125", this));

        form->addRow(new QLabel("Parsing Options", this));
        form->addRow(QString("'Straight' threshold"), _crossThresh = new QLineEdit("0", this));

        form->addRow(new QLabel("Scaling Options", this));
        form->addRow(QString("Image Width (px):"), new QLabel(width, this));
        form->addRow(QString("Image Height (px):"), new QLabel(height, this));
        form->addRow(QString("Image Width (m):"), _mx = new QLineEdit(width, this));
        form->addRow(QString("Image Height (m):"), _my = new QLineEdit(height, this));
        form->addRow(QString("Pixels / Meter Horizontal:"), _pxpmx = new QLineEdit("1", this));
        form->addRow(QString("Pixels / Meter Vertical:"), _pxpmy = new QLineEdit("1", this));

        form->addRow(new QLabel("Drawing Options", this));

        QHBoxLayout* colorRow = new QHBoxLayout(this);
        colorRow->addWidget(_colorButton = new QPushButton("Color", this));
        colorRow->addWidget(_colorLabel = new QLabel(this));
        form->addRow(colorRow);

        updateColorLabel();
        connect(_colorButton, &QPushButton::pressed, this, &ImageOptions::getNewColor);

        QDialogButtonBox* bbox = new QDialogButtonBox(QDialogButtonBox::Ok, Qt::Horizontal, this);
        form->addRow(bbox);

        connect(bbox, &QDialogButtonBox::accepted, this, &QDialog::accept);

        _mx->setValidator(new QDoubleValidator(0, std::numeric_limits<double>::max(), 2, this));
        _my->setValidator(new QDoubleValidator(0, std::numeric_limits<double>::max(), 2, this));

        _pxpmx->setValidator(new QDoubleValidator(0, std::numeric_limits<double>::max(), 2, this));
        _pxpmy->setValidator(new QDoubleValidator(0, std::numeric_limits<double>::max(), 2, this));

        _color->setValidator(new QIntValidator(0, 255, this));

        _crossThresh->setValidator(new QIntValidator(0, std::numeric_limits<int>::max(), this));

        connect(_pxpmx, &QLineEdit::editingFinished, this, &ImageOptions::constrain1);
        connect(_pxpmy, &QLineEdit::editingFinished, this, &ImageOptions::constrain1);

        connect(_mx, &QLineEdit::editingFinished, this, &ImageOptions::constrain2);
        connect(_my, &QLineEdit::editingFinished, this, &ImageOptions::constrain2);
    }

    /*!
     * \brief Getter for chosen intensity threshold
     * \return Integer in the range [0, 255]
     */
    uint64_t getBlackWhiteThreshold()
    {
        return _color->text().toInt();
    }

    /*!
     * \brief Getter for straightness threshold
     * \return A positive integer value
     */
    uint64_t getCrossProductThreshold()
    {
        return _crossThresh->text().toInt();
    }

    /*!
     * \brief Getter for horizontal pixels per meter value
     * \return A positive double value
     */
    double getPxPerWidth()
    {
        return _pxpmx->text().toDouble();
    }

    /*!
     * \brief Getter for vertical pixels per meter value
     * \return A positive double value
     */
    double getPxPerHeight()
    {
        return _pxpmy->text().toDouble();
    }

    /*!
     * \brief Getter for the selected drawing color
     * \return An RGB Color
     */
    QColor getDrawColor()
    {
        return _drawColor;
    }

};
