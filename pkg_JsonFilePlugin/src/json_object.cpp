#include "json_object.h"
#include <QDebug>

WorldObject* JsonObjectLoader::loadFile(QString filePath, QMap<QString, WorldObjectComponent_Plugin_If *> plugins)
{
    QFile loadFile(filePath);

    QByteArray saveData = loadFile.readAll();

    QJsonDocument loadDoc(QJsonDocument::fromJson(saveData));

    QJsonObject json = loadDoc.object();

    QVector<WorldObjectComponent*> components;

    if (json.contains("components") && json["components"].isArray())
    {
        QJsonArray componentArray = json["components"].toArray();
        for (int i = 0; i < componentArray.size(); i++)
        {
            QJsonObject componentObject = componentArray[i].toObject();
            if (componentObject.contains("pluginName") && componentObject["pluginName"].isObject() && plugins.contains(componentObject["pluginName"].toString()))
            {
                WorldObjectComponent* comp = plugins[componentObject["pluginName"].toString()]->createComponent();
                QMap<QString, QSharedPointer<PropertyView>> props = comp->getProperties();

                if (componentObject.contains("properties") && componentObject["properties"].isArray())
                {
                    QJsonArray propArray = componentObject["properties"].toArray();
                    for (int j = 0; j < propArray.size(); j++)
                    {
                        QJsonObject propertyObject = propArray[j].toObject();
                        if (propertyObject.contains("key") && componentObject["key"].isObject()
                         && propertyObject.contains("value") && componentObject["value"].isObject())
                        {
                            props[propertyObject["key"].toString()]->set(propertyObject["value"].toVariant(), true);
                        }
                    }
                }
                components.push_back(comp);
            }
        }
    }

    WorldObject* robot = new WorldObject(components, "Loaded from Json");

    if (json.contains("properties") && json["properties"].isArray())
    {
        QJsonArray propertyArray = json["properties"].toArray();
        for (int i = 0; i < propertyArray.size(); i++)
        {
            QJsonObject propertyObject = propertyArray[i].toObject();
            if (propertyObject.contains("key") && propertyObject["key"].isObject()
             && propertyObject.contains("value") && propertyObject["value"].isObject())
            {
                robot->getProperties()[propertyObject["key"].toString()]->set(propertyObject["value"].toVariant(), true);
            }
        }
    }

    return robot;
}

void JsonObjectSaver::saveFile(QString filePath, WorldObject* object)
{
    QFile saveFile(filePath);

    QJsonObject robotObject;

    QJsonArray compArray;
    foreach (WorldObjectComponent* comp, object->getComponents())
    {
        QJsonObject compObject;
        QJsonArray propArray;
        for (QMap<QString, QSharedPointer<PropertyView>>::iterator it = comp->getProperties().begin(); it != comp->getProperties().end(); it++)
        {
            QJsonObject propObj;
            propObj["key"] = it.key();
            propObj["value"] = QJsonValue::fromVariant(it.value()->get());
            propArray.append(propObj);
        }
        compObject["pluginName"] = comp->getPluginName();
        compObject["properties"] = propArray;
        compArray.append(compObject);
    }
    robotObject["components"] = compArray;

    QJsonArray propArray;
    for (QMap<QString, QSharedPointer<PropertyView>>::iterator it = object->getProperties().begin();it != object->getProperties().end(); it++)
    {
        QJsonObject propObj;
        propObj["key"] = it.key();
        propObj["value"] = QJsonValue::fromVariant(it.value()->get());
        propArray.append(propObj);
    }
    robotObject["properties"] = propArray;

    QJsonDocument saveDoc(robotObject);
    saveFile.write(saveDoc.toJson());
}
