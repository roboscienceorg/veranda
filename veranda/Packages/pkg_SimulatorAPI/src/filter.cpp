#include "veranda/filter.h"

void Bindable::set_bound(const QVariant& v)
{
    _value = v;
    value_set(_value);
}

const QVariant& Bindable::operator()() const
{ return _value; }

void Bindable::operator = (QSharedPointer<PropertyView> prop)
{
    if(_binding)
        disconnect(_binding.data(), nullptr, this, nullptr);

    if(prop)
    {
        connect(prop.data(), &PropertyView::valueSet, this, &Bindable::set_bound);
        set_bound(prop->get());
    }
    _binding = prop;
}

void Bindable::operator = (const QVariant& newValue)
{
    if(_binding)
        disconnect(_binding.data(), nullptr, this, nullptr);
    _value = newValue;
    value_set(_value);
}
