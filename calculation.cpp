#include "calculation.h"

QVector<POINT> trajectoryPoints(QVector<POINT> supportingPoints, POINT startPoint, POINT endPoint){
    QVector<POINT> trajectoryPoints;

    if(!supportingPoints.empty())
    {
        trajectoryPoints.push_back(startPoint);

        for(int i = 0;  i < supportingPoints.size(); i++){
            trajectoryPoints.push_back(supportingPoints[i]);
        }

        trajectoryPoints.push_back(endPoint);
    } else {
        trajectoryPoints.push_back(startPoint);
        trajectoryPoints.push_back(endPoint);
    }

    return trajectoryPoints;
}

POINT linearInterpolation(int i, int j, double step, POINT startP, POINT endP){
    struct VECTOR{
        double x;
        double y;
        double z;
    };

    POINT actTcpPoint;
    VECTOR SE;
    SE.x = endP.x - startP.x;
    SE.y = endP.y - startP.y;
    SE.z = endP.z - startP.z;

    VECTOR SA = {0,0,0};

    if (i == 0){
        actTcpPoint = startP;
    } else {
        SA.x = double(i)*SE.x/step;
        SA.y = double(i)*SE.y/step;
        SA.z = double(i)*SE.z/step;

        actTcpPoint.x = SA.x + startP.x;
        actTcpPoint.y = SA.y + startP.y;
        actTcpPoint.z = SA.z + startP.z;
    }
   //Wypisanie aktualnego punktu TCP
//    std::cout << "Wspolrzedne TCP " << std::endl;
//    std::cout << "actTcpPoint x: " << actTcpPoint.x << std::endl;
//    std::cout << "actTcpPoint y: " << actTcpPoint.y << std::endl;
//    std::cout << "actTcpPoint z: " << actTcpPoint.z << std::endl;

    return  actTcpPoint;
}

void calculateMachineCoords(ROBOT_PARAMETERS param, ROBOT_COORDS_SYSTEMS &s, MACHINE_COORDS &actMachineCoords){
    //param, s , actMachineCoords

    double SO = sin(param.OAngle*M_PI / 180);
    double CO = cos(param.OAngle*M_PI / 180);
    double SY = sin(param.YAngle*M_PI / 180);
    double CY = cos(param.YAngle*M_PI / 180);

    //Obliczenie xp, yp, zp
    s.coordP.x = s.coordTCP.x - (param.l5 + param.l6)*CO*CY;
    s.coordP.y = s.coordTCP.y - (param.l5 + param.l6)*CO*SY;
    s.coordP.z = s.coordTCP.z - (param.l5 + param.l6)*SO;

//    std::cout << "Wspolrzedne ukladu P" << std::endl;
//    std::cout << "xP = " << s.coordP.x << std::endl;
//    std::cout << "yP = " << s.coordP.y << std::endl;
//    std::cout << "zP = " << s.coordP.z << std::endl;

    double S1 = ((param.e*s.coordP.x + param.delta1*s.coordP.y*sqrt(pow(s.coordP.x,2) + pow(s.coordP.y,2) - pow(param.e,2)))/(pow(s.coordP.x,2) + pow(s.coordP.y,2)));
    double C1 = ((-param.e*s.coordP.y + param.delta1*s.coordP.x*sqrt(pow(s.coordP.x,2) + pow(s.coordP.y,2) - pow(param.e,2)))/(pow(s.coordP.x,2) + pow(s.coordP.y,2)));

//    std::cout << "Wartosci S1 oraz C1" << std::endl;
//    std::cout << "S1 = " << S1 << std::endl;
//    std::cout << "C1 = " << C1 << std::endl;

    double S5 = CO*(SY*C1-CY*S1);
    double C5 = param.delta5*sqrt(1-pow(S5,2));

//    std::cout << "Wartosci S5 oraz C5" << std::endl;
//    std::cout << "S5 = " << S5 << std::endl;
//    std::cout << "C5 = " << C5 << std::endl;

    double S234 = SO/C5;
    double C234 = (CO/C5)*(CY*C1 + SY*S1);

//    std::cout << "Wartosci S234 oraz C234" << std::endl;
//    std::cout << "S5 = " << S234 << std::endl;
//    std::cout << "C5 = " << C234 << std::endl;

    s.coordR.x = s.coordP.x - param.l4*C1*C234;
    s.coordR.y = s.coordP.y - param.l4*S1*C234;
    s.coordR.z = s.coordP.z - param.l4*S234;

//    std::cout << "Wspolrzedne ukladu R" << std::endl;
//    std::cout << "xR = " << s.coordR.x << std::endl;
//    std::cout << "yR = " << s.coordR.y << std::endl;
//    std::cout << "zR = " << s.coordR.z << std::endl;

    double a = (-param.l1 + param.delta1*sqrt(pow(s.coordR.x,2) + pow(s.coordR.y,2) - pow(param.e,2)));
    double b = ((pow(a,2) + pow(s.coordR.z,2) + pow(param.l2,2) - pow(param.l3,2))/(2*param.l2));

//    std::cout << "Zmienna a i b" << std::endl;
//    std::cout << "a = " << a << std::endl;
//    std::cout << "b = " << b << std::endl;

    double S2 = (s.coordR.z*b + param.delta2*a*sqrt(pow(a,2) + pow(s.coordR.z,2) - pow(b,2)))/(pow(a,2) + pow(s.coordR.z,2));
    double C2 = (a*b - param.delta2*s.coordR.z*sqrt(pow(a,2) + pow(s.coordR.z,2) - pow(b,2)))/(pow(a,2) + pow(s.coordR.z,2));

//    std::cout << "Wartosci S2 oraz C2" << std::endl;
//    std::cout << "S2 = " << S2 << std::endl;
//    std::cout << "C2 = " << C2 << std::endl;

    double S3 = -(param.delta2*sqrt(pow(a,2) + pow(s.coordR.z,2) - pow(b,2)))/param.l3;
    double C3 = (b - param.l2)/param.l3;

//    std::cout << "Wartosci S3 oraz C3" << std::endl;
//    std::cout << "S3 = " << S3 << std::endl;
//    std::cout << "C3 = " << C3 << std::endl;

    double S23 = (s.coordR.z - param.l2*S2)/param.l3;
    double C23 = (a - param.l2*C2)/param.l3;

//    std::cout << "Wartosci S23 oraz C23" << std::endl;
//    std::cout << "S23 = " << S23 << std::endl;
//    std::cout << "C23 = " << C23 << std::endl;

    double S4 = S234*C23 - C234*S23;
    double C4 = C234*C23 + S234*S23;

    s.coord1.x = param.l1*C1;
    s.coord1.y = param.l1*S1;
    s.coord1.z = 0;

//    std::cout << "Wspolrzedne ukladu 01" << std::endl;
//    std::cout << "x01 = " << s.coord1.x << std::endl;
//    std::cout << "y01 = " << s.coord1.y << std::endl;
//    std::cout << "zR01 = " << s.coord1.z << std::endl;

    s.coord1prim.x = s.coord1.x + param.d*S1;
    s.coord1prim.y = s.coord1.y - param.d*C1;
    s.coord1prim.z = 0;

//    std::cout << "Wspolrzedne ukladu 01prim" << std::endl;
//    std::cout << "x01prim = " << s.coord1prim.x << std::endl;
//    std::cout << "y01prim = " << s.coord1prim.y << std::endl;
//    std::cout << "z01prim = " << s.coord1prim.z << std::endl;

    s.coord2prim.x = s.coord1prim.x + param.l2*C2*C1;
    s.coord2prim.y = s.coord1prim.y + param.l2*C2*S1;
    s.coord2prim.z = param.l2*S2;

//    std::cout << "Wspolrzedne ukladu 02prim" << std::endl;
//    std::cout << "x02prim = " << s.coord2prim.x << std::endl;
//    std::cout << "y02prim = " << s.coord2prim.y << std::endl;
//    std::cout << "z02prim = " << s.coord2prim.z << std::endl;

    s.coord2.x = s.coord2prim.x -(param.d - param.e)*S1;
    s.coord2.y = s.coord2.y +(param.d - param.e)*C1;
    s.coord2.z = s.coord2prim.z;

//    std::cout << "Wspolrzedne ukladu 02" << std::endl;
//    std::cout << "x02 = " << s.coord2.x << std::endl;
//    std::cout << "y02 = " << s.coord2.y << std::endl;
//    std::cout << "z02 = " << s.coord2.z << std::endl;


    actMachineCoords.fi1 = asin(S1) * 180.0 / M_PI;
    actMachineCoords.fi2 = asin(S2) * 180.0 / M_PI;
    actMachineCoords.fi3 = asin(S3) * 180.0 / M_PI;
    actMachineCoords.fi4 = asin(S4) * 180.0 / M_PI;
    actMachineCoords.fi5 = asin(S5) * 180.0 / M_PI;
    actMachineCoords.fi23 = asin(S23) * 180.0 / M_PI;
    actMachineCoords.fi234 = asin(S234) * 180.0 / M_PI;

//    std::cout << "Wspolrzedne maszynowe" << std::endl;
//    std::cout << "fi1 = " << actMachineCoords.fi1 << std::endl;
//    std::cout << "fi2 = " << actMachineCoords.fi2 << std::endl;
//    std::cout << "fi3 = " << actMachineCoords.fi3 << std::endl;
//    std::cout << "fi4 = " << actMachineCoords.fi4 << std::endl;
//    std::cout << "fi5 = " << actMachineCoords.fi5 << std::endl;


}

void printCoordSystemPosition(ROBOT_COORDS_SYSTEMS s, MACHINE_COORDS actMachineCoords, int licz){
    std::cout << licz <<". Krok " << std::endl;
    std::cout << "CS-0: " << "x: " << s.coord0.x << " y: " << s.coord0.y << " z: " << s.coord0.z << std::endl;
    std::cout << "CS-01: " << "x: " << s.coord1.x << " y: " << s.coord1.y << " z: " << s.coord1.z << std::endl;
    std::cout << "CS-01': " << "x: " << s.coord1prim.x << " y: " << s.coord1prim.y << " z: " << s.coord1prim.z << std::endl;
    std::cout << "CS-02': " << "x: " << s.coord2prim.x << " y: " << s.coord2prim.y << " z: " << s.coord2prim.z << std::endl;
    std::cout << "CS-02: " << "x: " << s.coord2.x << " y: " << s.coord2.y << " z: " << s.coord2prim.z << std::endl;
    std::cout << "CS-R: " << "x: " << s.coordR.x << " y: " << s.coordR.y << " z: " << s.coordR.z << std::endl;
    std::cout << "CS-P: " << "x: " << s.coordP.x << " y: " << s.coordP.y << " z: " << s.coordP.z << std::endl;
    std::cout << "CS-TCP: " << "x: " << s.coordTCP.x << " y: " << s.coordTCP.y << " z: " << s.coordTCP.z << std::endl;
    std::cout << "Wspolrzedne maszynowe: \n" ;
    std::cout << "fi1: " << actMachineCoords.fi1 << std::endl;
    std::cout << "fi2: " << actMachineCoords.fi2 << std::endl;
    std::cout << "fi3: " << actMachineCoords.fi3 << std::endl;
    std::cout << "fi4: " << actMachineCoords.fi4 << std::endl;
    std::cout << "fi5: " << actMachineCoords.fi5 << std::endl << std::endl;
}
