#ifndef ANIMATION_H
#define ANIMATION_H

#include <QGraphicsLineItem>
#include <QObject>
#include <QGraphicsObject>
#include <QPropertyAnimation>
#include <QSequentialAnimationGroup>
#include <QParallelAnimationGroup>
#include <QGraphicsScene>
#include <QGraphicsItem>
#include <QtCore>
#include <QCoreApplication>
#include <QtGui>
#include <QGraphicsView>
#include <QDialog>
#include <QtGlobal>
#include <QPainter>
#include <QTimer>


#include "database.h"

//czas animacji
extern double _time1;

class CPoint: public QGraphicsObject
{
    Q_OBJECT

public:
    void paint(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget = 0);
    QRectF boundingRect() const;
};

class CLine: public QObject, public QGraphicsLineItem
{

    //zmiana typu z QLineF na Q_OBJECT
    Q_OBJECT
    Q_PROPERTY(QPointF p1 READ p1 WRITE setP1)
    Q_PROPERTY(QPointF p2 READ p2 WRITE setP2)

private:
    QLineF l;

public:

    using QGraphicsLineItem::QGraphicsLineItem;

    //utrworzenie punktow
    void setP1(const QPointF & p);
    void setP2(const QPointF & p);

    //zwrocenie punktow
    QPointF p1() const;
    QPointF p2() const;

};

class CArm
{

private:

    //linia typu QObject
    CLine *line;
    CPoint *circle1;
    CPoint *circle2;

    //pojedynczy ruch koncow czlonu
    QPropertyAnimation *point1;
    QPropertyAnimation *point1_1;
    QPropertyAnimation *point2;
    QPropertyAnimation *point2_2;

    //anmiacja ruchow koncow czlonu w kolejnych iteracjach
    QSequentialAnimationGroup *sequence1;
    QSequentialAnimationGroup *sequence1_1;
    QSequentialAnimationGroup *sequence2;
    QSequentialAnimationGroup *sequence2_2;

    //anmiacja ruchu czlonu w kolejnych iteracjach
    QParallelAnimationGroup *parrarel;

    //czas pojedynczego ruch konca czlonu
    double _time2;

public:

    //animacja czlonu
    CArm(QGraphicsScene *scene, Qt::GlobalColor color, SPoint array_point[]);

    //pojedynczy ruch jednego konca czlonu
    void create_animation(QPropertyAnimation *animation, double xp, double yp, double xk, double yk);
};

class CSupport
{
public:
    CSupport(QGraphicsScene *scene, Qt::GlobalColor color, int x, int y);
};

class CRobot
{

private:

    //czlony
    CArm arm0(QGraphicsScene,Qt::GlobalColor,SPoint[]);
    CArm arm1(QGraphicsScene,Qt::GlobalColor,SPoint[]);
    CArm arm2(QGraphicsScene,Qt::GlobalColor,SPoint[]);
    CArm arm3(QGraphicsScene,Qt::GlobalColor,SPoint[]);
    CArm arm4(QGraphicsScene,Qt::GlobalColor,SPoint[]);
    CArm arm5(QGraphicsScene,Qt::GlobalColor,SPoint[]);
    CArm arm6(QGraphicsScene,Qt::GlobalColor,SPoint[]);

public:

    //konstruktor robota
    CRobot (QGraphicsScene *scene, Arm array_arm[]);

};

class CRobot_animation
{

public:

    //rzuty
     QGraphicsScene *view_xy;
     QGraphicsScene *view_xz;
     QGraphicsScene *view_yz;

     //podparcie
     CSupport support0(QGraphicsScene,Qt::GlobalColor,int,int);
     CSupport support1(QGraphicsScene,Qt::GlobalColor,int,int);
     CSupport support2(QGraphicsScene,Qt::GlobalColor,int,int);

public:

     //konstruktor rzutu
     CRobot_animation(Robot *lista1, double _time);

     //tworzenie pojedycznego rzutu
     QGraphicsScene *create_view(View *array_view);
};


#endif // ANIMATION_H
