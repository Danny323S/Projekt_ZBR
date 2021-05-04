#ifndef CALCULATION_H
#define CALCULATION_H
#include "structures.h"
#include <cmath>
#include <iostream>
#include <QVector>


QVector<Point> trajectoryPoints(QVector<Point> supportingPoints, Point startPoint, Point endPoint);

Point linearInterpolation(int i, int j,double step, Point startP, Point endP);

bool calculateMachineCoords(ROBOT_PARAMETERS param, ROBOT_COORDS_SYSTEMS &s, MACHINE_COORDS &actMachineCoords);

void printCoordSystemPosition(ROBOT_COORDS_SYSTEMS s, MACHINE_COORDS actMachineCoords, int licz);

bool checkPoint(ROBOT_PARAMETERS param, Point addedPoint);



//Warunke bezpiecznego przejścia
bool checksafetyCondition(ROBOT_COORDS_SYSTEMS s);

//Sprawdzenie czy proste są skośne
bool skewLines(VEKTOR n1, VEKTOR n2, Point p1, Point p2);

bool crossPoint(VEKTOR n1, VEKTOR n2, Point p1, Point p2);


#endif // CALCULATION_H

