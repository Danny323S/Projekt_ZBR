#ifndef STRUCTURES_H
#define STRUCTURES_H

#include <QVector>

struct Point{
    double x = 0;
    double y = 0;
    double z = 0;
};

struct VEKTOR{
    double x;
    double y;
    double z;
};

struct ROBOT_COORDS_SYSTEMS{
    Point coord0;
    Point coord1;
    Point coord1prim;
    Point coord2;
    Point coord2prim;
    //3th coordsystem
    Point coordR;
    //4th cooordsystem
    Point coordP;
    //5th coordsystem
    Point coordTCP;
};

//struktura zawierająca współrzędne maszynowe robota
struct MACHINE_COORDS {
    double fi1 = 0;
    double fi2 = 0;
    double fi3 = 0;
    double fi4 = 0;
    double fi5 = 0;
    double fi23 = 0;
    double fi234 = 0;
};

struct  ROBOT_PARAMETERS {
    //dlugości poszczególnych członów
    double l1 = 350;
    double l2 = 500;
    double l3 = 1350;
    double l4 = 150;
    double l5 = 300;
    double l6 = 120;

    //pierwsza długość odsadzenia
    double d = 220;

    //parametr potrzebny do obliczenia drugiego odsadzenia
    //drugie odsadzenie = d - e
    double e = 160;

    //kąty podejścia narzędnia
    double OAngle = 30;
    double YAngle = 45;

    //parametry rozwiązujące położenia ....
    double delta1 = -1;
    double delta2 = -1;
    double delta5 = -1;
};

struct Step
{
    Point array_point[8]; // ilosc punktow
};

struct Iteration{
    QVector<Step> vektor_step;
    QVector<MACHINE_COORDS> machine_coords;
};

#endif // STRUCTURES_H
