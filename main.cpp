#include "mainwindow.h"
#include <iostream>
#include <string>
#include <cmath>
#include <QVector>
#include "calculation.h"
#include "structures.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

//    //Tego fragmentu kodu tutaj nie będzie
//    QVector<POINT> supportingPoints;
//    POINT P1 = {700, 100, 400};
//    POINT P2 = {500, -300, 200};
//    POINT P3 = {100, -450, 200};
//    supportingPoints.push_back(P1);
//    supportingPoints.push_back(P2);
//    supportingPoints.push_back(P3);

//    ROBOT_PARAMETERS parameter; // Wprowadzane z GUI
//    POINT startP = {500, 600, 300}; // Wprowadzane z GUI
//    POINT endP = {-500, -600, 200}; // Wprowadzane z GUI
//    double step = 5; // Wprowadzane z GUI

//    MACHINE_COORDS machineC;
//    POINT actTcpPoint;

//    QVector<POINT> trajectoryP = trajectoryPoints(supportingPoints, startP, endP);

//    for(int j = 0; j < trajectoryP.size() - 1; j++)
//    {
//        for (int i = 0; i <= step; i++){
//            actTcpPoint = interpolation(i, step, trajectoryP[i], trajectoryP[i+1]);
//            calculateMachineCoords(parameter, actTcpPoint, machineC);
//            printMachineCoords(machineC, i);
//        }
//    }

    return a.exec();
}
