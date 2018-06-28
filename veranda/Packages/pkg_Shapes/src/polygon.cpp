//This header order is important for MSVC
#include "triangulator.h"
#include "polygon.h"

Polygon::Polygon(QObject* parent) : WorldObjectComponent("Polygon", "Shapes", parent)
{
    _triangleModel = new Model({}, {}, this);
    _triangleModel->setDrawHint(DrawHint{QColor(0, 0, 0), Qt::SolidLine, QColor(0, 0, 0), Qt::SolidPattern, true});
    registerModel(_triangleModel);

    connect(&_outerShape, &Property::valueRequested, this, &Polygon::makeTriangles);
    connect(&_innerShapes, &Property::valueRequested, this, &Polygon::makeTriangles);
    connect(&_scalex, &Property::valueRequested, this, &Polygon::makeTriangles);
    connect(&_scaley, &Property::valueRequested, this, &Polygon::makeTriangles);
    connect(&_straight, &Property::valueRequested, this, &Polygon::makeTriangles);

    connect(&_drawTriangles, &Property::valueRequested, this, &Polygon::setDrawHint);
    connect(&_colorr, &Property::valueRequested, this, &Polygon::setDrawHint);
    connect(&_colorb, &Property::valueRequested, this, &Polygon::setDrawHint);
    connect(&_colorg, &Property::valueRequested, this, &Polygon::setDrawHint);

    makeTriangles();
}

Polygon::~Polygon()
{
    _triangleModel->removeShapes(_triangleModel->shapes());
    qDeleteAll(_triangleShapes);
    _triangleShapes.clear();
}

WorldObjectComponent* Polygon::_clone(QObject* newParent)
{
    Polygon* out = new Polygon(newParent);

    return out;
}

void Polygon::generateBodies(b2World* world, object_id oId, b2Body* anchor)
{
    clearBodies();
    _world = world;

    b2BodyDef bDef;
    bDef.type = b2_staticBody;
    _polyBody = world->CreateBody(&bDef);
    registerBody(_polyBody, {_triangleModel}, true);

    makeFixtures();
}

void Polygon::makeFixtures()
{
    if(_world)
    {
        //Remove old fixtures
        for(b2Fixture* f : _polyFixtures)
            _polyBody->DestroyFixture(f);
        _polyFixtures.clear();

        //Add all triangles available
        for(b2Shape* s : _triangleShapes)
            _polyFixtures.push_back(_polyBody->CreateFixture(s, 1));
    }
}

void Polygon::clearBodies()
{
    if(_world)
    {
        for(b2Fixture* f : _polyFixtures)
            _polyBody->DestroyFixture(f);
        _world->DestroyBody(_polyBody);
        unregisterBody(_polyBody);
        _polyFixtures.clear();
    }
    _world = nullptr;
}

void toPolygon(const QVariantList& points, QPolygonF& poly)
{
    poly.resize(points.size());
    auto iter = poly.begin();
    for(const QVariant& v : points)
    {
        QSequentialIterable iterable = v.value<QSequentialIterable>();

        *iter = QPointF(iterable.at(0).toDouble(), iterable.at(1).toDouble());
        iter++;
    }
}

void Polygon::makeTriangles()
{
    //Clear out old shapes
    _triangleModel->removeShapes(_triangleModel->shapes());

    qDeleteAll(_triangleShapes);
    _triangleShapes.clear();

    //Get point values out of QVariantLists and scale
    Shape sh;

    double scalex = _scalex.get().toDouble();
    double scaley = _scaley.get().toDouble();

    QVariantList tmp = _outerShape.get().value<QVariantList>();
    toPolygon(tmp, sh.outer);

    tmp = _innerShapes.get().value<QVariantList>();
    sh.inner.resize(tmp.size());
    auto iter = sh.inner.begin();
    for(const QVariant& v : tmp)
        toPolygon(v.value<QVariantList>(), *(iter++));

    //Clean up polygons
    simplify(sh, _straight.get().toULongLong());

    //Form triangle shapes
    QVector<QPolygonF> shapes = triangulate(sh);

    //Convert triangles to box2d and validate
    b2Vec2 triBuffer[3];
    for(QPolygonF& poly : shapes)
    {
        triBuffer[0].Set(poly[0].x()/scalex, poly[0].y()/scaley);
        triBuffer[1].Set(poly[1].x()/scalex, poly[1].y()/scaley);
        triBuffer[2].Set(poly[2].x()/scalex, poly[2].y()/scaley);

        //Remove 'triangles' that are just a line
        if(std::abs(b2Cross(triBuffer[1] - triBuffer[0], triBuffer[2] - triBuffer[0])) > 0.001)
        {
            b2PolygonShape* triangle = new b2PolygonShape();
            triangle->Set(triBuffer, 3);

            _triangleShapes.push_back(triangle);
        }
    }

    _triangleModel->addShapes(_triangleShapes);
    _numShapes.set(_triangleShapes.size());
    makeFixtures();
}

void Polygon::setDrawHint()
{
    DrawHint newHint;
    newHint.fillColor = QColor(_colorr.get().toInt(), _colorg.get().toInt(), _colorb.get().toInt());
    newHint.outlineColor = newHint.fillColor;
    newHint.outlineStyle = Qt::SolidLine;
    newHint.fillStyle = _drawTriangles.get().toBool() ? Qt::NoBrush : Qt::SolidPattern;

    _triangleModel->setDrawHint(newHint);
}
