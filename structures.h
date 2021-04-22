#ifndef STRUCTURES_H
#define STRUCTURES_H

#include <QVector>

struct POINT{
    double x = 0;
    double y = 0;
    double z = 0;
};

struct ROBOT_COORDS_SYSTEMS{
    POINT coord0;
    POINT coord1;
    POINT coord1prim;
    POINT coord2;
    POINT coord2prim;
    //3th coordsystem
    POINT coordR;
    //4th cooordsystem
    POINT coordP;
    //5th coordsystem
    POINT coordTCP;
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

struct STEP
{
    POINT array_point[8]; // ilosc punktow
};

struct ITERATION{
    QVector<STEP> vektor_step;
};

#endif // STRUCTURES_H
