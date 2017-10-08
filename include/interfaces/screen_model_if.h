#ifndef SCREEN_MODEL_H
#define SCREEN_MODEL_H

#include <QObject>
#include <QVector>

#include <Box2D/Box2D.h>

typedef uint64_t model_id;

class ScreenModel_If : public QObject
{
    Q_OBJECT

public:
    virtual QVector<b2Shape*> getModel() = 0;
    virtual void getTransform(double& x, double& y, double& theta) = 0;

    virtual void setModel(QVector<b2Shape*> newModel) = 0;
    virtual void setTransform(double x, double y, double theta) = 0;

signals:
    void modelChanged(ScreenModel_If*);
    void transformChanged(ScreenModel_If*);
};

#endif // SCREEN_MODEL_H
