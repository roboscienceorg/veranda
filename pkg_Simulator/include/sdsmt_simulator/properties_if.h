#ifndef PROPERTIES_H
#define PROPERTIES_H

#include <QObject>
#include <QString>
#include <QVariant>
#include <QDebug>
#include <functional>

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
    PropertyInfo _info;
    QVariant _value;

    void init()
    {
        if(_origin)
        {
            connect(_origin, &Property::valueSet, this, &PropertyView::update);
            connect(_origin, &Property::destroyed, [this](){_origin = nullptr;});
            connect(this, &PropertyView::requestUpdate, _origin, &Property::_update);
            connect(this, &PropertyView::requestValue, _origin, &Property::_set);

            requestUpdate();
        }
    }

public:
    PropertyView(Property* origin=nullptr) : _origin(origin)
    {
        if(origin)
            _info = origin->_info;
        //qDebug() << this << "of" << _origin;
        init();
    }

    PropertyView(const PropertyView& other) : _origin(other._origin)
    {
        _info = other._info;
        //qDebug() << this << "of" << _origin;
        init();
    }

    PropertyView& operator =(const PropertyView& other)
    {
        _info = other._info;
        _origin = other._origin;
        //qDebug() << this << "of" << _origin;

        return *this;
    }

    void set(const QVariant& value)
    {
        //qDebug() << "Set " << _origin << "from" << this;
        if(!_info.readOnly)
            requestValue(value);
    }

    const PropertyInfo& info()
    { return _info; }

    const QVariant& get()
    { return _value; }

signals:
    void valueSet(QVariant);
    void requestValue(QVariant);
    void requestUpdate();

private slots:
    void update(QVariant v)
    {
        _value = v;
    }
};

class PropertyObject_If : public QObject
{
    Q_OBJECT

public:
    PropertyObject_If(QObject* parent=nullptr) : QObject(parent){}

    virtual QMap<QString, PropertyView>& getAllProperties() = 0;
    virtual QString propertyGroupName() = 0;
};

#endif // PROPERTIES_H
