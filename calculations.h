#ifndef CALCULATIONS_H
#define CALCULATIONS_H
#include "structures.h"
#include <QVector>

//

QVector<POINT> trajectoryPoints(QVector<POINT> supportingPoints, POINT startPoint, POINT endPoint);

POINT interpolation(int i, double step, POINT startPoint, POINT endPoint);

POINT circularIntrpolation();

void checkCollision();

void chceckTrajectoryPoints();

void calculateMachineCoords(ROBOT_PARAMETERS parameter, POINT actTcpPoint, MACHINE_COORDS &actualCoords);

void printMachineCoords(MACHINE_COORDS machineCoords,  int i);



#endif // CALCULATIONS_H
