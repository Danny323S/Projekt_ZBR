#ifndef COORDINATESYSTEM_H
#define COORDINATESYSTEM_H

class coordinateSystem {

public:

    struct position {
        double x;
        double y;
        double z;
    }pos;

    //współrzędna maszynowa
    double fi;
    //cos fi
    double C;
    //sin fi
    double S;

    coordinateSystem(double x, double y, double z);

};
#endif // COORDINATESYSTEM_H
