#include "calculation.h"
#include "QMessageBox"

QVector<Point> trajectoryPoints(QVector<Point> supportingPoints, Point startPoint, Point endPoint){
    QVector<Point> trajectoryPoints;

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

Point linearInterpolation(int i, int j, double step, Point startP, Point endP){
    struct VECTOR{
        double x;
        double y;
        double z;
    };

    Point actTcpPoint;
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

bool calculateMachineCoords(ROBOT_PARAMETERS param, ROBOT_COORDS_SYSTEMS &s, MACHINE_COORDS &actMachineCoords){
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

    double root1 = pow(s.coordP.x,2) + pow(s.coordP.y,2) - pow(param.e,2);

    bool checkRoot1 = true;

    if(root1 < 0)
        checkRoot1 = false;

    if(checkRoot1 == false)
        return false;

    double S1 = ((param.e*s.coordP.x + param.delta1*s.coordP.y*sqrt(root1))/(pow(s.coordP.x,2) + pow(s.coordP.y,2)));
    double C1 = ((-param.e*s.coordP.y + param.delta1*s.coordP.x*sqrt(root1))/(pow(s.coordP.x,2) + pow(s.coordP.y,2)));

//    std::cout << "Wartosci S1 oraz C1" << std::endl;
//    std::cout << "S1 = " << S1 << std::endl;
//    std::cout << "C1 = " << C1 << std::endl;

    double S5 = CO*(SY*C1-CY*S1);

    double root2 = 1-pow(S5,2);

    bool checkRoot2 = true;

    if(root2 < 0)
        checkRoot2 = false;

    if(checkRoot2 == false)
        return false;

    double C5 = param.delta5*sqrt(root2);

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

    double root5 = pow(s.coordR.x,2) + pow(s.coordR.y,2) - pow(param.e,2);

    bool checkRoot5 = true;

    if(root5 < 0)
        checkRoot5 = false;

    if(checkRoot5 == false)
        return false;

    double a = (-param.l1 + param.delta1*sqrt(root5));
    double b = ((pow(a,2) + pow(s.coordR.z,2) + pow(param.l2,2) - pow(param.l3,2))/(2*param.l2));

//    std::cout << "Zmienna a i b" << std::endl;
//    std::cout << "a = " << a << std::endl;
//    std::cout << "b = " << b << std::endl;

    double root3 = pow(a,2) + pow(s.coordR.z,2) - pow(b,2);

    bool checkRoot3 = true;

    if(root3 < 0)
        checkRoot3 = false;

    if(checkRoot3 == false)
        return false;

    double S2 = (s.coordR.z*b + param.delta2*a*sqrt(root3))/(pow(a,2) + pow(s.coordR.z,2));
    double C2 = (a*b - param.delta2*s.coordR.z*sqrt(root3))/(pow(a,2) + pow(s.coordR.z,2));

//    std::cout << "Wartosci S2 oraz C2" << std::endl;
//    std::cout << "S2 = " << S2 << std::endl;
//    std::cout << "C2 = " << C2 << std::endl;

    double root4 = pow(a,2) + pow(s.coordR.z,2) - pow(b,2);

    bool checkRoot4 = true;

    if(root4 < 0)
        checkRoot4 = false;

    if(checkRoot4 == false)
        return false;

    double S3 = -(param.delta2*sqrt(root4))/param.l3;
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

        return true;
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

bool checkPoint(ROBOT_PARAMETERS param, Point addedPoint){

    double reach = param.l1 + param.l2 + param.l3 + param.l4;

    double addedPointDistance = sqrt(pow(addedPoint.x,2) + pow(addedPoint.y,2) + pow(addedPoint.z,2));

    if(addedPointDistance > reach)
        return false;

    return true;
}

bool crossPoint(VEKTOR n1, VEKTOR n2, Point ws1, Point zs1, Point ws2, Point zs2){
    // obliczenie punktu przecięcia się lini przechodzących przez człony
    double y = (ws2.y + ws1.x - ws2.x)/(n1.y - n1.x);
    double x = n1.x*(y - ws1.y)/n1.y + ws1.x;
    double z = n1.z*(y - ws1.y)/n1.y + n1.z;

    bool checkX = false;
    bool checkY = false;
    bool checkZ = false;

    //Sprawdzenie czy punkt przecięcia leży na ramionach, jeżeli tak -> true
    if((ws1.x < x && x < zs1.x) && (ws2.x < x && x < zs2.x))
        checkX = true;
    if((ws1.y < y && y < zs1.y) && (ws2.y < y && y < zs2.y))
        checkY = true;
    if((ws1.z < z && z < zs1.z) && (ws2.z < z && z < zs2.z))
        checkZ = true;

    if(checkX == true && checkY == true && checkZ == true)
        return true; //Ramiona się przecinają :(
    else
        return false; // Ramiona nie przecinają się :)
}

bool checksafetyCondition(ROBOT_COORDS_SYSTEMS s){

    VEKTOR A = {s.coord1.x - s.coord0.x, s.coord1.y - s.coord0.y, s.coord1.z - s.coord0.z };
    VEKTOR B = {s.coord1prim.x - s.coord1.x, s.coord1prim.y - s.coord1.y, s.coord1prim.z - s.coord1.z };
    VEKTOR C = {s.coord2prim.x - s.coord1prim.x, s.coord2prim.y - s.coord1prim.y, s.coord2prim.z - s.coord1prim.z };
    VEKTOR D = {s.coord2.x - s.coord2prim.x, s.coord2.y - s.coord2prim.y, s.coord2.z - s.coord2prim.z };
    VEKTOR E = {s.coordR.x - s.coord2.x, s.coordR.y - s.coord2.y, s.coordR.z - s.coord2.z };
    VEKTOR F = {s.coordP.x - s.coordR.x, s.coordP.y - s.coordR.y, s.coordP.z - s.coordR.z };
    VEKTOR G = {s.coordTCP.x - s.coordP.x, s.coordTCP.y - s.coordP.y, s.coordTCP.z - s.coordP.z };

    //Jeżeli linie nie są skośne, sprawdzane jest czy punkt ich przecięcia leży w obrębie długości ramion
    if(skewLines(A, C, s.coord0, s.coord1prim) == false)
        if(crossPoint(A, C, s.coord0, s.coord1, s.coord1prim, s.coord2prim))
            return false;

    if(skewLines(A, D, s.coord0, s.coord1) == false)
        if(crossPoint(A, D, s.coord0, s.coord1, s.coord2prim, s.coord2))
            return false;

    if(skewLines(A, E, s.coord0, s.coord2) == false)
        if(crossPoint(A, E, s.coord0, s.coord1, s.coord2, s.coordR))
            return false;

    if(skewLines(A, F, s.coord0, s.coordR) == false)
        if(crossPoint(A, F, s.coord0, s.coord1, s.coordR, s.coordP))
            return false;

    if(skewLines(A, G, s.coord0, s.coordR) == false)
        if(crossPoint(A, G, s.coord0, s.coord1, s.coordP, s.coordTCP))
            return false;

    if(skewLines(B, E, s.coord1, s.coord2) == false)
        if(crossPoint(B, E, s.coord1, s.coord1prim, s.coord2, s.coordR))
            return false;

    if(skewLines(B, F, s.coord1, s.coordR) == false)
        if(crossPoint(B, F, s.coord1, s.coord1prim, s.coordR, s.coordP))
            return false;

    if(skewLines(B, G, s.coord1, s.coordP) == false)
        if(crossPoint(B, G, s.coord1, s.coord1prim, s.coordP, s.coordTCP))
            return false;

    if(skewLines(C, E, s.coord1prim, s.coord2) == false)
        if(crossPoint(C, E, s.coord1prim, s.coord2prim, s.coord2, s.coordR))
            return false;

    if(skewLines(C, F, s.coord1prim, s.coordR) == false)
        if(crossPoint(C, F, s.coord1prim, s.coord2prim, s.coordR, s.coordP))
            return false;

    if(skewLines(C, G, s.coord1prim, s.coordP) == false)
        if(crossPoint(C, G, s.coord1prim, s.coord2prim, s.coordP, s.coordTCP))
            return false;

    if(skewLines(D, F, s.coord2prim, s.coordR) == false)
        if(crossPoint(D, F, s.coord2prim, s.coord2, s.coordR, s.coordP))
            return false;

    if(skewLines(D, G, s.coord2prim, s.coordP) == false)
        if(crossPoint(D, G, s.coord2prim, s.coord2, s.coordP, s.coordTCP))
            return false;

    return true;
}

bool skewLines(VEKTOR n1, VEKTOR n2, Point p1, Point p2){
    // Jeżeli prsote są skośne to iloczyn mieszany (n1, n2, p1p2) != 0

    VEKTOR P1P2 = {p2.x-p1.x, p2.y-p1.y, p2.z-p1.z};

    //iloczyn miewszany
    double tripleProduct = n1.x*(n2.y*P1P2.z - n2.z*P1P2.y) - n1.y*(n2.x*P1P2.z - n2.z*P1P2.x) + n1.z*(n2.x*P1P2.y - n2.y*P1P2.x);

    if(tripleProduct == 0)
        return true;//Proste skośne == prawda
    else
        return false;//Proste przecinają się == false
}
