#pragma once

#include "property.h"

#include <QObject>
#include <QSharedPointer>

#include <random>
#include <iostream>

class Bindable : public QObject
{
    Q_OBJECT

    QVariant _value;
    QSharedPointer<PropertyView> _binding;

    void set_bound(const QVariant& v);

public:
    const QVariant& operator()() const;

    void operator = (QSharedPointer<PropertyView> prop);
    void operator = (const QVariant& newValue);
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

public:
    NormalFilter(QSharedPointer<reng> engine = QSharedPointer<reng>())
    {
        _sigma = 1;
        _mu = 0;
        _nonNanChance = 1;

        _engine = engine;

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

        if(!_engine)
            _engine = QSharedPointer<reng>(new reng());
    }

    double apply(const double& input = 1) const override
    {
        auto normDist = std::normal_distribution<>{_mu().toDouble(), _sigma().toDouble()};

        double beNan = _uniDist(*_engine);
        if(beNan >= _nonNanChance().toDouble()) return std::numeric_limits<double>::quiet_NaN();

        return input * normDist(*_engine);
    }

    template<class sigma_type>
    void sigma(sigma_type sigma)
    {
        _sigma = sigma;
    }

    template<class mu_type>
    void mu(mu_type mu)
    {
        _mu = mu;
    }

    template<class nan_type>
    void nonNanChance(nan_type nonNanChance)
    {
        _nonNanChance = nonNanChance;
    }
};

template<class reng>
std::uniform_real_distribution<> NormalFilter<reng>::_uniDist{0.0, 1.0};
