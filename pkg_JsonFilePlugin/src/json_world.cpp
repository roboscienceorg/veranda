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
    {
        QJsonObject json = jsonArray[i].toObject();
        QVector<WorldObjectComponent*> components;

        if (json.contains("components") && json["components"].isArray())
        {
            QJsonArray componentArray = json["components"].toArray();
            for (int j = 0; j < componentArray.size(); j++)
            {
                QJsonObject componentObject = componentArray[j].toObject();
                if (componentObject.contains("pluginName") && plugins.contains(componentObject["pluginName"].toString()))
                {
                    WorldObjectComponent* comp = plugins[componentObject["pluginName"].toString()]->createComponent();
                    QMap<QString, QSharedPointer<PropertyView>> props = comp->getProperties();

                    if (componentObject.contains("properties") && componentObject["properties"].isArray())
                    {
                        QJsonArray propArray = componentObject["properties"].toArray();
                        for (int k = 0; k < propArray.size(); k++)
                        {
                            QJsonObject propertyObject = propArray[k].toObject();
                            if (propertyObject.contains("key") && propertyObject.contains("value"))
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
            for (int j = 0; j < propertyArray.size(); j++)
            {
                QJsonObject propertyObject = propertyArray[j].toObject();
                if (propertyObject.contains("key") && propertyObject.contains("value"))
                {
                    std::cout << propertyObject["key"].toString().toStdString() << std::endl;
                    robot->getProperties()[propertyObject["key"].toString()]->set(propertyObject["value"].toVariant(), true);
                }
            }
        }

        robots.append(robot);
    }

    return robots;
}

void JsonWorldSaver::saveFile(QString filePath, QVector<WorldObject*> objects)
{
    QFile saveFile(filePath);
    saveFile.open(QIODevice::WriteOnly);
    QJsonArray robotArray;

    for (int i = 0; i < objects.size(); i++)
    {
        QJsonObject robotObject;
        QJsonArray compArray;

        foreach (WorldObjectComponent* comp, objects[i]->getComponents())
        {
            QJsonObject compObject;
            QJsonArray propArray;
            QMap<QString, QSharedPointer<PropertyView>> compProps = comp->getProperties();
            QMap<QString, QSharedPointer<PropertyView>>::iterator it;
            for (it = compProps.begin(); it != compProps.end(); it++)
            {
                if (!it.key().contains("GlobalPos"))
                {
                    QJsonObject propObj;
                    propObj["key"] = it.key();
                    propObj["value"] = QJsonValue::fromVariant(it.value()->get());
                    propArray.append(propObj);
                }
            }
            compObject["pluginName"] = comp->getPluginName();
            compObject["properties"] = propArray;
            compArray.append(compObject);
        }
        robotObject["components"] = compArray;

        QJsonArray propArray;
        QMap<QString, QSharedPointer<PropertyView>> objProps = objects[i]->getProperties();
        QMap<QString, QSharedPointer<PropertyView>>::iterator it;
        for (it = objProps.begin();it != objProps.end(); it++)
        {
            if (it.key().contains("GlobalPos") && it.key().count('/') == 1)
            {
                QJsonObject propObj;
                propObj["key"] = it.key();
                propObj["value"] = QJsonValue::fromVariant(it.value()->get());
                propArray.append(propObj);
            }
        }

        robotObject["properties"] = propArray;
        robotArray.append(robotObject);
    }

    QJsonDocument saveDoc(robotArray);
    saveFile.write(saveDoc.toJson());
}
