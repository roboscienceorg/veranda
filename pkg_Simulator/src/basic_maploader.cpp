#include "basic_maploader.h"

//Must perform some error checking
Map* BasicMapLoader::loadMapFile(QString filename)
{
    QString val;
    QFile file;
    file.setFileName(filename);
    file.open(QIODevice::ReadOnly | QIODevice::Text);
    val = file.readAll();
    file.close();
    QJsonDocument d = QJsonDocument::fromJson(val.toUtf8());
    QJsonObject world = d.object();
    return loadMapObject(world);
}

Map* BasicMapLoader::loadMapObject(QJsonObject world)
{
    Map* m = new Map;

    m->setXMin(world["xMin"].toInt());
    m->setXMax(world["xMax"].toInt());
    m->setYMin(world["yMin"].toInt());
    m->setYMax(world["yMax"].toInt());
    m->setXOrigin(world["xOrigin"].toInt());
    m->setYOrigin(world["yOrigin"].toInt());
    QJsonValue sb = world["staticBodies"];
    QJsonArray sbArr = sb.toArray();

    QVector<b2PolygonShape *> polygons;

    foreach (const QJsonValue & s, sbArr)
    {
        b2PolygonShape* ps = new b2PolygonShape;
        QJsonObject sObj = s.toObject();
        QJsonValue p = sObj["points"];
        QJsonArray pArr = p.toArray();
        QVector<b2Vec2> vertices;

        foreach (const QJsonValue & p, pArr)
        {
            QJsonArray xy = p.toArray();
            vertices.push_back(b2Vec2(xy[0].toDouble(), xy[1].toDouble()));
        }

        ps->Set(vertices.data(), vertices.size());

        //Re-set centroid
        QJsonValue center = sObj["center"];
        QJsonArray cxy = center.toArray();
        ps->m_centroid = b2Vec2(cxy[0].toDouble(), cxy[1].toDouble());

        polygons.push_back(ps);
    }

    m->setStaticBodies(polygons);

    return m;
}
