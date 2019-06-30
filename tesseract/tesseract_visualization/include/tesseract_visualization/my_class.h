#ifndef MY_CLASS_H
#define MY_CLASS_H

#include <QtCore>
#include <QObject>

class MyClass : public QObject
{
    Q_OBJECT
public Q_SLOTS:
    void cppSlot(const int& index) {
        qDebug() << "Called the C++ slot with index:" << index;
    }

    void cppSlot2(const QString& text) {
        qDebug() << "Called the C++ slot with text:" << text;
    }
};
#endif // MY_CLASS_H
