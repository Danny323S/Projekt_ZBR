#include "animation.h"

double _time1;

void CPoint::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(option)
    Q_UNUSED(widget)
    painter->setRenderHint(QPainter::Antialiasing);
    painter->setPen(QColor(Qt::black));
    painter->setBrush(QBrush(QColor(Qt::black)));
    QRectF rect = boundingRect();
    painter->drawEllipse(rect);
}
QRectF CPoint::boundingRect() const
{
    return QRectF(-4, -4, 8, 8);
}

QPointF CLine::p1() const
{
    return  line().p1();
}
void CLine::setP1(const QPointF &p)
{

    l = line();
    l.setP1(p);
    setLine(l);
}
QPointF CLine::p2() const
{
    return  line().p2();
}
void CLine::setP2(const QPointF &p)
{
    l = line();
    l.setP2(p);
    setLine(l);
}

CArm::CArm(QGraphicsScene *scene, Qt::GlobalColor color, SPoint array_point[])
{

        line=new CLine;
        QPen pen;
        pen.setWidth(50);
        pen.setColor(color);
        line->setPen(pen);

        circle1=new CPoint;
        circle2=new CPoint;

        sequence1= new QSequentialAnimationGroup;
        sequence1_1= new QSequentialAnimationGroup;
        sequence2=new QSequentialAnimationGroup;
        sequence2_2= new QSequentialAnimationGroup;

        parrarel= new QParallelAnimationGroup;

        _time2 = _time1/array_point[0].coordinate1.size()*1000;

        for(int i=0; i < array_point[0].coordinate1.size()-1;i++)
        {
            point1=new QPropertyAnimation (line,"p1");
            scene->addItem(line);
            create_animation(point1,array_point[0].coordinate1[i],array_point[0].coordinate2[i],array_point[0].coordinate1[i+1],array_point[0].coordinate2[i+1]);
            sequence1->addAnimation(point1);

            point1_1=new QPropertyAnimation(circle1,"pos");
            scene->addItem(circle1);
            create_animation(point1_1,array_point[0].coordinate1[i],array_point[0].coordinate2[i],array_point[0].coordinate1[i+1],array_point[0].coordinate2[i+1]);
            sequence1_1->addAnimation(point1_1);

            point2=new QPropertyAnimation (line,"p2");
            scene->addItem(line);
            create_animation(point2,array_point[1].coordinate1[i],array_point[1].coordinate2[i],array_point[1].coordinate1[i+1],array_point[1].coordinate2[i+1]);
            sequence2->addAnimation(point2);

            point2_2=new QPropertyAnimation(circle2,"pos");
            scene->addItem(circle2);
            create_animation(point2,array_point[1].coordinate1[i],array_point[1].coordinate2[i],array_point[1].coordinate1[i+1],array_point[1].coordinate2[i+1]);
            sequence2_2->addAnimation(point2_2);
        }




        parrarel->addAnimation(sequence1);

        parrarel->addAnimation(sequence2);


        parrarel->addAnimation(sequence1_1);

        parrarel->addAnimation(sequence2_2);

        parrarel->start();

}

void CArm::create_animation(QPropertyAnimation *animation, double xp, double yp, double xk, double yk)
{
    animation->setStartValue(QPointF(xp, yp));
    animation->setEndValue(QPointF(xk, yk));
    animation->setDuration(_time2);
}

CSupport::CSupport(QGraphicsScene *scene, Qt::GlobalColor color, int x, int y)
{
    QRectF rec(x,y,50,50);
    QPen pen;
    QBrush brush(color);
    scene->addRect(rec, pen, brush);
}

CRobot::CRobot(QGraphicsScene *scene, Arm array_arm[])
{
    CArm arm0(scene,Qt::green,array_arm[0].array_point);
    CArm arm1(scene,Qt::blue,array_arm[1].array_point);
    CArm arm2(scene,Qt::black,array_arm[2].array_point);
    CArm arm3(scene,Qt::magenta,array_arm[3].array_point);
    CArm arm4(scene,Qt::gray,array_arm[4].array_point);
    CArm arm5(scene,Qt::cyan,array_arm[5].array_point);
    CArm arm6(scene,Qt::darkGreen,array_arm[6].array_point);

}


CRobot_animation::CRobot_animation(Robot *lista1,double _time=60)
{
    _time1=_time;


        //---------------------------------------------------------zmienić wielkość okien

        view_xy=new QGraphicsScene;
        CSupport support0(view_xy,Qt::red,-25,-25);
        CRobot robot(view_xy,lista1->array_view[0].array_arm);
        //view_xy->setSceneRect(-182,-160,364,320);


        view_xz=new QGraphicsScene;
        CSupport support1(view_xz,Qt::red,-25,25);
        CRobot robot1(view_xz,lista1->array_view[1].array_arm);
        //view_xz->setSceneRect(-182,-485,364,320);



        view_yz=new QGraphicsScene;
        CSupport support2(view_yz,Qt::red,-25,25);
        CRobot robot2(view_yz,lista1->array_view[2].array_arm);
        //view_yz->setSceneRect(-182,-485,364,320);


}





