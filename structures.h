#ifndef STRUCTURES_H
#define STRUCTURES_H

struct POINT{
    double x = 0;
    double y = 0;
    double z = 0;
};

struct  ROBOT_PARAMETERS {
    //dlugości poszczególnych członów
    double arm1 = 350;
    double arm2 = 500;
    double arm3 = 1350;
    double arm4 = 150;
    double arm5 = 200;
    double arm6 = 220;

    //pierwsza długość odsadzenia
    double d = 220;

    //parametr potrzebny do obliczenia drugiego odsadzenia
    //drugie odsadzenie = d - e
    double e = 60;

    //kąty podejścia narzędnia
    double OAngle = 45;
    double YAngle = 30;

    //parametry rozwiązujące położenia ....
    int delta1 = -1;
    int delta2 = -1;
    int delta5 = -1;
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

#endif // STRUCTURES_H
