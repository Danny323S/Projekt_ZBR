#ifndef DATABASE_H
#define DATABASE_H

#include <QVector>
#include <QtMath>
#include "structures.h"

struct SPoint
{
    // dana wspolrzedna w kolejnych iteracjach
    QVector<double> coordinate1;
    QVector<double> coordinate2;
};

struct Arm
{
    // konce czlonu
    SPoint array_point[2];
};

struct View
{
    //czlony robota
    Arm array_arm[7];
};

struct Robot
{
    //rzuty
    View array_view[3];
};

struct Database
{
    double coordinate_max;
    QVector<MACHINE_COORDS> machine_coords;
    QVector<Point> vektor_tcp;
    //tranformacja listy z puntkami na liste do animacji
    Database(Iteration *lista, Robot *lista1);    
};


#endif // DATABASE_H


