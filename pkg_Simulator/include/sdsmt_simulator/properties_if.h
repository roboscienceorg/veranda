#ifndef PROPERTIES_H
#define PROPERTIES_H

#include <QObject>
#include <QString>
#include <QVariant>
#include <QDebug>
#include <functional>

#include "model.h"

typedef uint64_t object_id;

class PropertyInfo
{
public:
    enum PropertyType {STRING, BOOL, INT, DOUBLE};

    QString description;
    bool readOnly;
    bool isList;
    PropertyType type;

    PropertyInfo(bool _readOnly=true, bool _isList=false, PropertyType _type=STRING, QString _description="") :
        description(_description), readOnly(_readOnly), isList(_isList), type(_type){}
};

class Property : public QObject
{
    Q_OBJECT

    friend class PropertyView;

    PropertyInfo _info;
    std::function<QVariant(QVariant, QVariant)> _validate;
    QVariant _value;

public:
    Property(PropertyInfo info = PropertyInfo(), QVariant defaultValue="",
            const std::function<QVariant(QVariant, QVariant)> validator = [](QVariant, QVariant _new){ return _new; },
            QObject* parent=nullptr) :
        QObject(parent), _info(info), _validate(validator), _value(defaultValue)
    {
        //qDebug() << "Created origin" << this;
    }

    Property(const Property& other) : _info(other._info), _validate(other._validate), _value(other._value)
    {

    }

    ~Property(){}

    const QVariant& get()
    { return _value; }

    const PropertyInfo& info()
    { return _info; }

    void set(const QVariant& v)
    {
        //qDebug() << "Setting property from origin" << this;
        _set(v);
    }

signals:
    //Signals that the value was changed or attempted to change
    void valueSet(QVariant);

private slots:
    void _set(QVariant newValue)
    {
        //qDebug() << "Validate value at origin" << this;
        _value = _validate(_value, newValue);

        //Push value to viewers created from this one
        _update();
    }

    void _update()
    {
        //Publish value to watching objects
        valueSet(_value);
    }
};

class PropertyView : public QObject
{
    Q_OBJECT

    Property* _origin = nullptr;

    void init()
    {
        if(_origin)
        {
            connect(_origin, &Property::valueSet, this, &PropertyView::valueSet);
            connect(_origin, &Property::destroyed, this, &PropertyView::_invalidate);
            connect(this, &PropertyView::requestValue, _origin, &Property::_set);
        }
    }

private slots:
    void _invalidate()
    {
        _origin = nullptr;
    }

public:
    PropertyView(Property* origin=nullptr) : _origin(origin)
    {
        init();
    }

    PropertyView(const PropertyView& other) : _origin(other._origin)
    {
        init();
    }

    PropertyView& operator =(const PropertyView& other)
    {
        _origin = other._origin;
        init();

        return *this;
    }

    void set(const QVariant& value)
    {
        //qDebug() << "Set " << _origin << "from" << this;
        if(_origin && !_origin->info().readOnly)
            requestValue(value);
    }

    const PropertyInfo info()
    { return _origin ? _origin->info() : PropertyInfo(); }

    const QVariant get()
    { return _origin ? _origin->get() : QVariant(); }

signals:
    void valueSet(QVariant);
    void requestValue(QVariant);
};

#endif // PROPERTIES_H
