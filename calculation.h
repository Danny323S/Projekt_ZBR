#ifndef CALCULATION_H
#define CALCULATION_H
#include "structures.h"
#include <cmath>
#include <iostream>
#include <QVector>


QVector<POINT> trajectoryPoints(QVector<POINT> supportingPoints, POINT startPoint, POINT endPoint);

POINT linearInterpolation(int i, int j,double step, POINT startP, POINT endP);

void calculateMachineCoords(ROBOT_PARAMETERS param, ROBOT_COORDS_SYSTEMS &s, MACHINE_COORDS &actMachineCoords);

void printCoordSystemPosition(ROBOT_COORDS_SYSTEMS s, MACHINE_COORDS actMachineCoords, int licz);

#endif // CALCULATION_H

