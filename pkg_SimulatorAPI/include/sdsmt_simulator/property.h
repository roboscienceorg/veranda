//! \file
#pragma once

#include <QObject>
#include <QString>
#include <QStringList>
#include <QVariant>
#include <QDebug>
#include <functional>

#include "dllapi.h"
#include "const.h"

//! Typedef for object identifier numbers
typedef int32_t object_id;

/*!
 * \brief Metadata container for info about a property
 */
class SDSMT_SIMULATOR_API PropertyInfo
{
public:
    //! Enum for types of properties
    enum PropertyType {STRING, BOOL, INT, DOUBLE};

    QString description; //!< Short description of the property that could be shown to a user
    bool readOnly; //!< True if the property should not be changed after a world object has been designed
    bool isList; //!< True if the property is an array of whatever its type is
    bool shouldSave; //!< True if the property is required to re-create the state of the component, and therefore should be saved and cloned
    PropertyType type; //!< Type of property

    //! Convenience constructor to initialize all values
    PropertyInfo(bool _readOnly=true, bool _shouldSave=false, bool _isList=false, PropertyType _type=STRING, QString _description="") :
        description(_description), readOnly(_readOnly), isList(_isList), shouldSave(_shouldSave), type(_type){}
};

/*!
 * \brief Data type used to expose internal variables to external objects
 * Properties are used to contain a configurable variable for an object. They
 * can be encapsulated in PropertyViews and given to other objects, which can
 * query the state of the variable and request that it change. When the property
 * is changed, both the owner and observers can recieve a signal in order to
 * react appropriately.
 *
 * Additionally, properties can have a validator function registered which will
 * automatically trigger each time the property value changes. The validator
 * can be used to ensure that invalid data is not stored in the property. A number
 * of useful validators are provided as static members of the Property class
 *
 * Validators should follow the prototype
 * QVariant (QVariant old, QVariant new);
 * When a value update is requested, the validator is called and given both
 * the previous value and the newly requested one; it must return a value which
 * will be the one stored (this could just be the old value). The default validator
 * accepts all values
 */
class SDSMT_SIMULATOR_API Property : public QObject
{
    Q_OBJECT

    friend class PropertyView;

    //! Metadata for the Property
    PropertyInfo _info;

    //! Validation function
    std::function<QVariant(QVariant, QVariant)> _validate;

    //! Current Property value
    QVariant _value;

public:
    /*!
     * \brief Validator to check that the stored value is a double
     * The validator rejects any non-numeric values. Values within EPSILON (see const.h)
     * of 0 are rounded to 0.
     *
     * \param[in] _old The old value stored
     * \param[in] _new The new value requested
     * \return The value that should be stored
     */
    static QVariant double_validator(QVariant _old, QVariant _new)
    {
        bool valid;
        _new.toDouble(&valid);
        if(valid)
            return std::abs(_new.toDouble()) < EPSILON ? 0.0 : _new;
        return _old;
    }

    /*!
     * \brief Validator to check that the stored value is an integer
     * The validator rejects any non-integer values.
     *
     * \param[in] _old The old value stored
     * \param[in] _new The new value requested
     * \return The value that should be stored
     */
    static QVariant int_validator(QVariant _old, QVariant _new)
    {
        bool valid;
        _new.toInt(&valid);
        if(valid)
            return _new;
        return _old;
    }

    /*!
     * \brief Validator to check that the stored value is boolen value.
     * All non-0 values, and non-empty strings, as well as the strings "1", "true", and "yes"
     * are treated as true. All empty strings, numeric 0's and the strings "0", "false", and "no"
     * are treated as false. String compares are case-insensitive.
     *
     * \param[in] _old The old value stored
     * \param[in] _new The new value requested
     * \return The value that should be stored
     */
    static QVariant bool_validator(const QVariant& _old, const QVariant& _new)
    {
        const static QStringList isTrue{"1", "true", "yes", "y"};
        const static QStringList isFalse{"0", "false", "no", "n"};

        bool k;

        //If expected indicator of true or false
        if(isTrue.contains(_new.toString().toLower())) return true;
        if(isFalse.contains(_new.toString().toLower())) return false;

        //If numeric value not 0, return true
        if(std::abs(_new.toDouble(&k)) > EPSILON && k) return true;
        //If numerick ~= 0, return false
        else if(k) return false;

        //If non-empty string, return true
        if(_new.toString().size()) return true;

        //All other cases false
        return false;
    }

    /*!
     * \brief Validator to check that the stored value is a double
     * The validator rejects any non-numeric values. Values within EPSILON (see const.h)
     * of 0 are rounded to 0. Any negative values are converted to their absolute values.
     *
     * \param[in] _old The old value stored
     * \param[in] _new The new value requested
     * \return The value that should be stored
     */
    static QVariant abs_double_validator(const QVariant& _old, const QVariant& _new)
    {
        return std::abs(double_validator(_old, _new).toDouble());
    }

    /*!
     * \brief Validator to check that the stored value is a double in the range [0, 1]
     * The validator rejects any non-numeric values. Values within EPSILON (see const.h)
     * of 0 are rounded to 0. Any negative values are converted to their absolute values.
     * Any values outside the allowed range are clamped to the range
     *
     * \param[in] _old The old value stored
     * \param[in] _new The new value requested
     * \return The value that should be stored
     */
    static QVariant probability_validator(const QVariant& _old, const QVariant& _new)
    {
        double absolute = double_validator(_old, _new).toDouble();
        return std::max(std::min(absolute, 1.0), 0.0);
    }


    /*!
     * \brief Validator to check that the stored value is an angle
     *  The validator requires that angle values be numeric between 0 and 360 inclusive.
     * \param[in] _old The old value stored
     * \param[in] _new The new value requested
     * \return The value that should be stored
     */
    static QVariant angle_validator(const QVariant& _old, const QVariant& _new)
    {
        bool isDouble;
        double asDouble = _new.toDouble(&isDouble);
        if(isDouble && asDouble >= 0 && asDouble <= 360)
            return asDouble < EPSILON ? 0.0 : _new;
        return _old;
    }

    /*! \brief Constructs a new property with metadata, a default value, and a validator
     *  See class description for details about validator behavior
     *
     * \param[in] info Metadata for the property
     * \param[in] defaultValue The value that the Property should contain at its creation
     * \param[in] validator The validator function to use
     * \param[in] parent QObject parent
     */
    Property(PropertyInfo info = PropertyInfo(), QVariant defaultValue="",
            const std::function<QVariant(const QVariant&, const QVariant&)> validator = [](const QVariant&, const QVariant& _new){ return _new; },
            QObject* parent=nullptr) :
        QObject(parent), _info(info), _validate(validator), _value(defaultValue){}

    /*!
     * \brief Copy constructor
     * \param[in] other Property to copy
     */
    Property(const Property& other) : QObject(other.parent()), _info(other._info), _validate(other._validate), _value(other._value) {}

    //! Empty destructor
    ~Property(){}

    /*!
     * \brief Getter for the stored value
     * \return The stored value
     */
    const QVariant& get()
    { return _value; }

    /*!
     * \brief Getter for the property metadata
     * \return The property metadata
     */
    const PropertyInfo& info()
    { return _info; }

    /*!
     * \brief Sets a new value using the validation function an signals listeners
     * \param[in] v The value to set
     * \param[in] notifyOwner If true, the valueRequested signal is emitted in addition to valueSet (Default false)
     */
    void set(const QVariant& v, bool notifyOwner = false)
    {
        //Update to value if valid
        _value = _validate(_value, v);

        //Notify watchers
        _update(notifyOwner);
    }

signals:
    /*!
     * \brief Signal that the value was set for the Property; this does not guarantee that it changed
     * \param[in] value The currently stored value
     */
    void valueSet(QVariant value);

    /*!
     * \brief Signal that the value was set for the Property by a request from an object other than the owner
     * \param[in] value The currently stored value
     */
    void valueRequested(QVariant value);

private slots:
    /*!
     * \brief Internal function used to change the value
     * \param[in] newValue The value to update to through the validator
     */
    void _request(QVariant newValue)
    {
        //qDebug() << "Validate value at origin" << this;
        _value = _validate(_value, newValue);

        //Push value to viewers created from this one
        _update(true);
    }

    /*!
     * \brief Notifies watchers that the value may have changed
     * \param[in] isRequest If true, then valueRequested is emitted in addition to the regualar valueSet
     */
    void _update(bool isRequest)
    {
        //Publish value to watching objects
        valueSet(_value);

        //Notify owner
        if(isRequest)
            valueRequested(_value);
    }
};

/*!
 * \brief Observer class that can be used to watch a property and request it to change
 */
class SDSMT_SIMULATOR_API PropertyView : public QObject
{
    Q_OBJECT

    //! The property being observed
    Property* _origin = nullptr;

    /*!
     * \brief Connects important signals between the view and the property
     */
    void init()
    {
        if(_origin)
        {
            connect(_origin, &Property::valueSet, this, &PropertyView::valueSet);
            connect(_origin, &Property::destroyed, this, &PropertyView::_invalidate);
            connect(this, &PropertyView::requestValue, _origin, &Property::_request);
        }
    }

private slots:
    /*!
     * \brief Invalidates the view and drops its reference to the property if the property is destroyed
     */
    void _invalidate()
    {
        _origin = nullptr;
    }

public:
    /*!
     * \brief Constructs a propertyView to watch a specific property
     * \param[in] origin The property to observe
     */
    PropertyView(Property* origin=nullptr) : _origin(origin)
    {
        init();
    }

    /*!
     * \brief Copy constructor to watch the same property as another PropertyView
     * \param[in] other PropertyView to copy
     */
    PropertyView(const PropertyView& other) : _origin(other._origin)
    {
        init();
    }

    /*!
     * \brief Assignment operator to copy other PropertyView
     * \param[in] other PropertyView to copy
     * \return Reference to self
     */
    PropertyView& operator =(const PropertyView& other)
    {
        _origin = other._origin;
        init();

        return *this;
    }

    /*!
     * \brief Request a change to the original property, optionally ignoreing the ReadOnly metadata
     * \param[in] value Value to request be set
     * \param[in] force If true, then the readOnly metadata is ignored (Default false)
     */
    void set(const QVariant& value, bool force = false)
    {
        //qDebug() << "Set " << _origin << "from" << this;
        if(_origin && (!_origin->info().readOnly || force))
            requestValue(value);
    }

    /*!
     * \brief Getter for the property metadata
     * \return The property metadata
     */
    const PropertyInfo info()
    { return _origin ? _origin->info() : PropertyInfo(); }

    /*!
     * \brief Getter for the property value
     * \return The property value
     */
    const QVariant get()
    { return _origin ? _origin->get() : QVariant(); }

signals:
    /*!
     * \brief Signals that the property's value may have changed
     * \param[in] value The current value of the property
     */
    void valueSet(QVariant value);

    /*!
     * \brief Signal requesting a new value to be set
     * This should not be called directly; use the set() method instead
     * \param[in] value The value requested
     */
    void requestValue(QVariant value);
};

