/********************************************************************************
** Form generated from reading UI file 'emptysimwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_EMPTYSIMWINDOW_H
#define UI_EMPTYSIMWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_emptysimwindow
{
public:
    QWidget *centralwidget;
    QHBoxLayout *horizontalLayout;
    QVBoxLayout *simviewcontainer;

    void setupUi(QMainWindow *emptysimwindow)
    {
        if (emptysimwindow->objectName().isEmpty())
            emptysimwindow->setObjectName(QStringLiteral("emptysimwindow"));
        emptysimwindow->resize(800, 600);
        centralwidget = new QWidget(emptysimwindow);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        horizontalLayout = new QHBoxLayout(centralwidget);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        simviewcontainer = new QVBoxLayout();
        simviewcontainer->setObjectName(QStringLiteral("simviewcontainer"));

        horizontalLayout->addLayout(simviewcontainer);

        emptysimwindow->setCentralWidget(centralwidget);

        retranslateUi(emptysimwindow);

        QMetaObject::connectSlotsByName(emptysimwindow);
    } // setupUi

    void retranslateUi(QMainWindow *emptysimwindow)
    {
        emptysimwindow->setWindowTitle(QApplication::translate("emptysimwindow", "MainWindow", 0));
    } // retranslateUi

};

namespace Ui {
    class emptysimwindow: public Ui_emptysimwindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_EMPTYSIMWINDOW_H
