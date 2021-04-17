#include <cmath>
#include <iostream>
#include <QVector>
#include "coordinatesystem.h"
#include "calculations.h"

//funkcja na podstawie punktów x. y. z. wprowadzonych przez użytkownika tworzy vektor Punktów podporowych
//
QVector<POINT> trajectoryPoints(QVector<POINT> supportingPoints, POINT startPoint, POINT endPoint) {

    QVector<POINT> trajectoryPoints;

    if(!supportingPoints.empty())
    {
        trajectoryPoints.push_back(startPoint);

        for(int i = 0;  i < supportingPoints.size(); i++){
            trajectoryPoints.push_back(supportingPoints[i]);
        }

        trajectoryPoints.push_back(endPoint);
    }

    return trajectoryPoints;
}

POINT interpolation(int i, double step, POINT startPoint, POINT endPoint){

    struct VECTOR{
        double x = 0;
        double y = 0;
        double z = 0;
    };

    //if(!supportingPointsTab.empty());

    //wektor między punktem początkowym i końcowym TCP
    VECTOR SE;
    SE.x = endPoint.x - startPoint.x;
    SE.y = endPoint.y - startPoint.y;
    SE.z = endPoint.z - startPoint.z;

    VECTOR temp;
    temp.x = SE.x;
    temp.y = SE.y;
    temp.z = SE.z;

    POINT actTcpPoint;

    if(step == 0){
        actTcpPoint.x = startPoint.x;
        actTcpPoint.y = startPoint.y;
        actTcpPoint.z = startPoint.z;
    } else {
        temp.x = (double(i)/step)*temp.x;
        temp.y = (double(i)/step)*temp.y;
        temp.z = (double(i)/step)*temp.z;

        actTcpPoint.x = temp.x + startPoint.x;
        actTcpPoint.y = temp.y + startPoint.y;
        actTcpPoint.z = temp.z + startPoint.z;
    }

    std::cout << i <<". Wspolrzedne TCP:" << std::endl;
    std::cout << "actTCPPoint.x = " << actTcpPoint.x << std::endl;
    std::cout << "actTCPPoint.y = " << actTcpPoint.y << std::endl;
    std::cout << "actTCPPoint.z = " << actTcpPoint.z << std::endl << std::endl;

    return actTcpPoint;
}

POINT circularIntrpolation();

void checkCollision();

void chceckTrajectoryPoints();

void calculateMachineCoords(ROBOT_PARAMETERS parameter, POINT actTcpPoint, MACHINE_COORDS &actualCoords){

    //dlugości poszczególnych członów
    double arm1 = parameter.arm1;
    double arm2 = parameter.arm2;
    double arm3 = parameter.arm3;
    double arm4 = parameter.arm4;
    double arm5 = parameter.arm5;
    double arm6 = parameter.arm6;

    //pierwsza długość odsadzenia
    double d = parameter.d;

    //parametr potrzebny do obliczenia drugiego odsadzenia
    //drugie odsadzenie = d - e
    double e = parameter.e;

    //kąty podejścia narzędnia
    double OAngle = parameter.OAngle;
    double YAngle = parameter.YAngle;

    //parametry rozwiązujące położenia ....
    double delta1 = parameter.delta1;
    double delta2 = parameter.delta1;
    double delta5 = parameter.delta1;

    //zmienne do obliczeń
    double a = 0;
    double b = 0;

    //Układ bazowy O0
    coordinateSystem coord0(0, 0, 0);

    //deklaracja ukladu współrzędnych narzednia
    //pozycja bedzie później wprowadzana przez uzytkownika
    coordinateSystem TCPcoords(actTcpPoint.x,actTcpPoint.y,actTcpPoint.z);
    double sinO = sin(OAngle*M_PI / 180);
    double cosO = cos(OAngle*M_PI / 180);
    double sinY = sin(YAngle*M_PI / 180);
    double cosY = cos(YAngle*M_PI / 180);


    //Zdefiniowanie układu O5, Obliczenie xp, yp, zp
    coordinateSystem coord5(0, 0, 0);
    coord5.pos.x = TCPcoords.pos.x - ((arm6 + arm5) * cosO * cosY);
    coord5.pos.y = TCPcoords.pos.y - ((arm6 + arm5) * cosO * sinY);
    coord5.pos.z = TCPcoords.pos.z - ((arm6 + arm5) * sinO);

    //Obliczenie S1 i C1
    coordinateSystem coord1(0 ,0 ,0);
    coord1.S = (1/(pow(coord5.pos.x,2) + pow(coord5.pos.y,2))) * (e*coord5.pos.x + delta1*coord5.pos.y*sqrt(pow(coord5.pos.x,2) + pow(coord5.pos.y,2) - pow(e,2)));
    coord1.C = (1/(pow(coord5.pos.x,2) + pow(coord5.pos.y,2))) * (-e*coord5.pos.x + delta1*coord5.pos.x*sqrt(pow(coord5.pos.x,2) + pow(coord5.pos.y,2) - pow(e,2)));

    //Obliczenie S5 i C5
    coord5.S = cosO * (sinY*coord1.C - cosY*coord1.S);
    coord5.C = delta5*sqrt(1 - pow(coord5.S, 2));

    coordinateSystem coord234(0, 0, 0);
    coord234.S = sinO/coord5.C;
    coord234.C = (cosO/coord5.C) * (cosY*coord1.C + sinY*coord1.S);

    //Zdefiniowanie układu O3, Obliczenie xr, yr, zr
    coordinateSystem coord3(0, 0, 0);
    coord3.pos.x = coord5.pos.x - arm4*coord1.C*coord234.C;
    coord3.pos.y = coord5.pos.y - arm4*coord1.S*coord234.C;
    coord3.pos.z = coord5.pos.z - arm4*coord234.S;

    a = (-arm1 + delta1*sqrt(pow(coord3.pos.x,2) + pow(coord3.pos.y,2) - pow(e,2)));
    b = ((pow(a,2) + pow(coord3.pos.z,2) + pow(arm2,2) - pow(arm3, 2))/(2*arm2));

    //Zdefiniowanie układu O2, Obliczenie S2, C2
    coordinateSystem coord2(0,0,0);
    coord2.S = (coord3.pos.z*b + delta2*a*sqrt(pow(a,2) + pow(coord3.pos.z,2) - pow(b,2)))/(pow(a, 2) + pow(coord3.pos.z,2));
    coord2.C = (a*b - delta2*coord3.pos.z*sqrt(pow(a,2) + pow(coord3.pos.z,2) - pow(b,2)))/(pow(a, 2) + pow(coord3.pos.z,2));

    //Obliczenie S3, C3
    coord3.S = -(delta2/arm3) * sqrt(pow(a,2) + pow(coord3.pos.z,2) - pow(b,2));
    coord3.C = (pow(a,2) + pow(coord3.pos.z,2) - pow(arm1,2) - pow(arm3,2)/(2*arm2*arm3));

    //obliczenie sin(fi2 + fi3) cos(fi2 +fi3)
    coordinateSystem coord23(0,0,0);
    //coord23.S = (1/arm3) * (coord3.pos.z -arm2*coord2.S);
    //coord23.C = (1/arm3) * (a - arm2*coord2.C);
    coord23.S = (coord3.pos.z - (arm2*(coord3.pos.z*b + delta2*a*sqrt(pow(a,2) + pow(coord3.pos.z,2) - pow(b,2)))/(pow(a,2) + pow(coord3.pos.z, 2))))/arm3;
    coord23.C = (a - (arm2*(a*b - delta2*coord3.pos.z*sqrt(pow(a,2) + pow(coord3.pos.z,2) - pow(b,2)))/(pow(a,2) + pow(coord3.pos.z, 2))))/arm3;


    //Zdefiniowanie O4, Obliczenie S4, C4
    coordinateSystem coord4(0,0,0);
    coord4.S = coord234.S*coord23.C - coord234.C*coord23.S;
    coord4.C = coord234.C*coord23.C - coord234.S*coord23.S;

    //obliczenie pozycji 01
    //coord1.pos.x = arm1*coord1.C;
    //coord1.pos.y = arm1*coord1.S;
    //coord1.pos.z = 0;

    //obliczenie pozycji02
    //coord2.pos.x = coord1.pos.x + d*coord1.S + arm2*coord2.C*coord1.C - (d-e)*coord1.S;
    //coord2.pos.y = arm1*coord1.S -d*coord1.C + arm2*coord2.C*coord1.S + (d-e)*coord1.C;
    //coord2.pos.z = arm2*coord2.S;

    //MACHINE_COORDS actualCoords;

    actualCoords.fi1 = asin(coord1.S) * 180.0 / M_PI ;
    actualCoords.fi2 = asin(coord2.S) * 180.0 / M_PI;
    actualCoords.fi3 = asin(coord3.S) * 180.0 / M_PI;
    actualCoords.fi4 = asin(coord4.S) * 180.0 / M_PI;
    actualCoords.fi5 = asin(coord5.S) * 180.0 / M_PI;
    actualCoords.fi23 = asin(coord23.S) * 180.0 / M_PI;
    actualCoords.fi234 = asin(coord234.S) * 180.0 / M_PI;

    std::cout << "fi1 = " << actualCoords.fi1 << std::endl;
    std::cout << "fi2 = " << actualCoords.fi2 << std::endl;
    std::cout << "fi3 = " << actualCoords.fi3 << std::endl;
    std::cout << "fi4 = " << actualCoords.fi4 << std::endl;
    std::cout << "fi5 = " << actualCoords.fi5 << std::endl;


    //printMachineCoords(actualCoords);

    // actualCoords;
}

void printMachineCoords(MACHINE_COORDS machineCoords, int i) {
    std::cout << i <<". Wspolrzedne maszynowe: " << std::endl;
    std::cout << "fi1 = " << machineCoords.fi1 << std::endl;
    std::cout << "fi2 = " << machineCoords.fi2 << std::endl;
    std::cout << "fi3 = " << machineCoords.fi3 << std::endl;
    std::cout << "fi4 = " << machineCoords.fi4 << std::endl;
    std::cout << "fi5 = " << machineCoords.fi5 << std::endl;
    std::cout << "fi23 = " << machineCoords.fi23 << std::endl;
    std::cout << "fi234 = " << machineCoords.fi234 << std::endl << std::endl;
}
