//! \file
#pragma once

#include <QVector>
#include <QString>

#include <QJsonObject>
#include <QJsonArray>
#include <QDebug>

#include <veranda_qt_frontend/object_loader_if.h>
#include <veranda_qt_frontend/object_saver_if.h>

/*!
 * \brief Reads a JSON description of a WorldObject and builds that WorldObject
 * Any components that cannot be created because their plugin is not available will be skipped
 * \param[in] json The JSON Object to convert
 * \param[in] plugins The map of available component plugins
 * \return A new WorldObject
 */
inline WorldObject* jsonObjectToWorldObject(const QJsonObject &json, QMap<QString, WorldObjectComponent_Factory_If *> plugins)
{
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
                            QString target_key = propertyObject["key"].toString();
                            if(!props.value(target_key))
                                qWarning() << "Warning! Plugin object does not contain property '" << target_key << "'";
                            else
                                props[target_key]->set(propertyObject["value"].toVariant(), true);
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
                //qDebug() << propertyObject["key"].toString().toStdString();
                robot->getProperties()[propertyObject["key"].toString()]->set(propertyObject["value"].toVariant(), true);
            }
        }
    }
    return robot;
}

/*!
 * \brief Makes a JSON Object description of a WorldObject
 * \param[in] obj The WorldObject to make a description of
 * \return The resulting JsonObject description
 */
inline QJsonObject worldObjectToJsonObject(WorldObject* obj)
{
    QJsonObject robotObject;
    QJsonArray compArray;

    foreach (WorldObjectComponent* comp, obj->getComponents())
    {
        QJsonObject compObject;
        QJsonArray propArray;
        QMap<QString, QSharedPointer<PropertyView>> compProps = comp->getProperties();
        QMap<QString, QSharedPointer<PropertyView>>::iterator it;
        for (it = compProps.begin(); it != compProps.end(); it++)
        {
            if (it.value()->info().shouldSave)
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
    QMap<QString, QSharedPointer<PropertyView>> objProps = obj->getSelfProperties();
    QMap<QString, QSharedPointer<PropertyView>>::iterator it;
    for (it = objProps.begin();it != objProps.end(); it++)
    {
        if (it.value()->info().shouldSave)
        {
            QJsonObject propObj;
            propObj["key"] = it.key();
            propObj["value"] = QJsonValue::fromVariant(it.value()->get());
            propArray.append(propObj);
        }
    }

    robotObject["properties"] = propArray;
    return robotObject;
}
