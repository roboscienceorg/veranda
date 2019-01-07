#pragma once

#include "property.h"

#include <QObject>
#include <QSharedPointer>

#include <random>
#include <iostream>
#include "veranda_core/api/dllapi.h"

class VERANDA_CORE_API_DLL Bindable : public QObject
{
    Q_OBJECT

    QVariant _value;
    QSharedPointer<PropertyView> _binding;

    void set_bound(const QVariant& v);

public:
    const QVariant& operator()() const;

    void operator = (QSharedPointer<PropertyView> prop);
    void operator = (const QVariant& newValue);

signals:
    void value_set(const QVariant&);
};

template<class T>
class Filter
{
public:
    virtual ~Filter(){}
    virtual T apply(const T&) const = 0;
};

template<class reng = std::default_random_engine>
class NormalFilter : public Filter<double>
{
    Bindable _sigma;
    Bindable _mu;
    Bindable _nonNanChance;

    QSharedPointer<reng> _engine;
    static std::uniform_real_distribution<> _uniDist;
    mutable std::normal_distribution<> _normDist = std::normal_distribution<>();

    void _refresh()
    {
        _normDist = std::normal_distribution<>{_mu().toDouble(), _sigma().toDouble()};
    }

public:
    NormalFilter(QSharedPointer<reng> engine = QSharedPointer<reng>())
    {
        _sigma = 1.0;
        _mu = 0.0;
        _nonNanChance = 1.0;

        _engine = engine;
        _normDist = std::normal_distribution<>{_mu().toDouble(), _sigma().toDouble()};

        QObject::connect(&_sigma, &Bindable::value_set, [this](){_refresh();});
        QObject::connect(&_mu, &Bindable::value_set, [this](){_refresh();});

        if(!_engine)
            _engine = QSharedPointer<reng>(new reng());
    }

    template<class mu_type, class sigma_type, class nan_type>
    NormalFilter(mu_type mu, sigma_type sigma, nan_type nonNanChance, QSharedPointer<reng> engine = QSharedPointer<reng>())
    {
        _sigma = sigma;
        _mu = mu;
        _nonNanChance = nonNanChance;

        _engine = engine;
        _normDist = std::normal_distribution<>{_mu().toDouble(), _sigma().toDouble()};

        QObject::connect(&_sigma, &Bindable::value_set, [this](){_refresh();});
        QObject::connect(&_mu, &Bindable::value_set, [this](){_refresh();});

        if(!_engine)
            _engine = QSharedPointer<reng>(new reng());
    }

    double apply(const double& input = 1) const override
    {

        double beNan = _uniDist(*_engine);
        if(beNan >= _nonNanChance().toDouble()) return std::numeric_limits<double>::quiet_NaN();

        return input * _normDist(*_engine);
    }

    template<class sigma_type>
    void sigma(sigma_type sigma)
    {
        _sigma = sigma;
        _refresh();
    }

    template<class mu_type>
    void mu(mu_type mu)
    {
        _mu = mu;
        _refresh();
    }

    template<class nan_type>
    void nonNanChance(nan_type nonNanChance)
    {
        _nonNanChance = nonNanChance;
    }
};

template<class reng>
std::uniform_real_distribution<> NormalFilter<reng>::_uniDist = std::uniform_real_distribution<>(0.0, 1.0);
