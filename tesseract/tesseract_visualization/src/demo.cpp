
#include <QGuiApplication>
#include <QtCore>
#include <QQuickView>
#include <QQuickItem>
#include <QQmlContext>
#include <QObject>
#include <tesseract_visualization/my_class.h>

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);
    QQuickView view(QUrl("qrc:/tesseract_visualization/JointManipulationWidget.qml"));

//    QQuickView view(QUrl("qrc:/tesseract_visualization/ConfigurationWidget.qml"));
//    QQuickItem *item = view.rootObject();
//    QQmlContext *context = view.rootContext();


//    QStringListModel list_model;
//    QStringList list = {"test", "test1", "test2"};
//    list_model.setStringList(list);
//    context->setContextProperty("fwdKinematicsComboBoxModel", &list_model);

//    MyClass myClass;

//    QObject::connect(item, SIGNAL(fwdKinematicsIndexChanged(int)),
//                     &myClass, SLOT(cppSlot(int)));

//    QObject::connect(item, SIGNAL(fwdKinematicsTextChanged(QString)),
//                     &myClass, SLOT(cppSlot2(QString)));

    view.show();
    return app.exec();
}
