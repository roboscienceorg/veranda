#include "json_object.h"
#include <QDebug>

WorldObject* JsonObjectLoader::loadFile(QString filePath, QMap<QString, WorldObjectComponent_Plugin_If *> plugins)
{
    QFile loadFile(filePath);
    loadFile.open(QIODevice::ReadOnly);

    QByteArray saveData = loadFile.readAll();

    QJsonDocument loadDoc(QJsonDocument::fromJson(saveData));

    return jsonObjectToWorldObject(loadDoc.object(), plugins);
}

void JsonObjectSaver::saveFile(QString filePath, WorldObject* object)
{
    QFile saveFile(filePath);
    saveFile.open(QIODevice::ReadWrite);

    QJsonDocument saveDoc(worldObjectToJsonObject(object));
    saveFile.write(saveDoc.toJson());
}
