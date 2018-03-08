#ifndef OPTIONDIALOG_H
#define OPTIONDIALOG_H

#include <QDialog>
#include <QLineEdit>
#include <QFormLayout>
#include <QDialogButtonBox>
#include <QBoxLayout>
#include <QLabel>
#include <QDoubleValidator>
#include <QIntValidator>

#include <limits>

class ImageOptions : public QDialog
{
    Q_OBJECT

    QLineEdit* _pxpmx;
    QLineEdit* _pxpmy;
    QLineEdit* _mx;
    QLineEdit* _my;
    QLineEdit* _color;
    QLineEdit* _crossThresh;

    uint64_t _width, _height;

private slots:
    void constrain1()
    {
        double pxmx = _pxpmx->text().toDouble();
        double pxmy = _pxpmy->text().toDouble();

        _mx->setText(QString::number(_width / pxmx));
        _my->setText(QString::number(_height / pxmy));
    }

    void constrain2()
    {
        double mx = _mx->text().toDouble();
        double my = _my->text().toDouble();

        _pxpmx->setText(QString::number(_width / mx));
        _pxpmy->setText(QString::number(_height / my));
    }

public:
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
        form->addRow(QString("'Straight' threshold"), _crossThresh = new QLineEdit("1", this));

        form->addRow(new QLabel("Scaling Options", this));
        form->addRow(QString("Image Width (px):"), new QLabel(width, this));
        form->addRow(QString("Image Height (px):"), new QLabel(height, this));
        form->addRow(QString("Image Width (m):"), _mx = new QLineEdit(width, this));
        form->addRow(QString("Image Height (m):"), _my = new QLineEdit(height, this));
        form->addRow(QString("Pixels / Meter Horizontal:"), _pxpmx = new QLineEdit("1", this));
        form->addRow(QString("Pixels / Meter Vertical:"), _pxpmy = new QLineEdit("1", this));

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

    uint64_t getBlackWhiteThreshold()
    {
        return _color->text().toInt();
    }

    uint64_t getCrossProductThreshold()
    {
        return _crossThresh->text().toInt();
    }

    double getPxPerWidth()
    {
        return _pxpmx->text().toDouble();
    }

    double getPxPerHeight()
    {
        return _pxpmy->text().toDouble();
    }
};

#endif // OPTIONDIALOG_H
