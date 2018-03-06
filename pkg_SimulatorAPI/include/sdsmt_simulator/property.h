#ifndef PROPERTIES_H
#define PROPERTIES_H

#include <QObject>
#include <QString>
#include <QStringList>
#include <QVariant>
#include <QDebug>
#include <functional>

#include "model.h"
#include "dllapi.h"

typedef int32_t object_id;

class SDSMT_SIMULATOR_API PropertyInfo
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


class SDSMT_SIMULATOR_API Property : public QObject
{
    Q_OBJECT

    friend class PropertyView;

    PropertyInfo _info;
    std::function<QVariant(QVariant, QVariant)> _validate;
    QVariant _value;

public:
    static QVariant double_validator(QVariant _old, QVariant _new)
    {
        bool valid;
        _new.toDouble(&valid);
        if(valid)
            return _new;
        return _old;
    }

    static QVariant int_validator(QVariant _old, QVariant _new)
    {
        bool valid;
        _new.toInt(&valid);
        if(valid)
            return _new;
        return _old;
    }

    static QVariant bool_validator(QVariant _old, QVariant _new)
    {
        const static QStringList isTrue{"1", "true", "yes"};
        const static QStringList isFalse{"0", "false", "no"};

        if(isTrue.contains(_new.toString().toLower())) return true;
        if(isFalse.contains(_new.toString().toLower())) return false;
        return _old;
    }

    static QVariant abs_double_validator(QVariant _old, QVariant _new)
    {
        return std::abs(double_validator(_old, _new).toDouble());
    }

    static QVariant angle_validator(QVariant _old, QVariant _new)
    {
        bool isDouble;
        double asDouble = _new.toDouble(&isDouble);
        if(isDouble && asDouble >= 0 && asDouble <= 360)
            return _new;
        return _old;
    }

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

class SDSMT_SIMULATOR_API PropertyView : public QObject
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

    void set(const QVariant& value, bool force = false)
    {
        //qDebug() << "Set " << _origin << "from" << this;
        if(_origin && (!_origin->info().readOnly || force))
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
