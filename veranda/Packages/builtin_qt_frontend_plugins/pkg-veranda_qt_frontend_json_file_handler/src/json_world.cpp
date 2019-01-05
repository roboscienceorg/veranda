#include "json_world.h"
#include <QDebug>
#include <iostream>

QVector<WorldObject*> JsonWorldLoader::loadFile(QString filePath, QMap<QString, WorldObjectComponent_Plugin_If *> plugins)
{
    QFile loadFile(filePath);
    loadFile.open(QIODevice::ReadOnly);

    QByteArray saveData = loadFile.readAll();

    QJsonDocument loadDoc(QJsonDocument::fromJson(saveData));

    QJsonArray jsonArray = loadDoc.array();

    QVector<WorldObject*> robots;

    for (int i = 0; i < jsonArray.size(); i++)
        robots.append(jsonObjectToWorldObject(jsonArray[i].toObject(), plugins));

    return robots;
}

void JsonWorldSaver::saveFile(QString filePath, QVector<WorldObject*> objects)
{
    QFile saveFile(filePath);
    saveFile.open(QIODevice::WriteOnly  | QIODevice::Truncate);
    QJsonArray robotArray;

    for (int i = 0; i < objects.size(); i++)
        robotArray.append(worldObjectToJsonObject(objects[i]));

    QJsonDocument saveDoc(robotArray);
    saveFile.write(saveDoc.toJson());
}
