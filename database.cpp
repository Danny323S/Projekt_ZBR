#include "database.h"

#include <QFile>
#include <QTextStream>
#include <QTime>
#include <QThread>

Database::Database(Iteration *lista, Robot *lista1)
{


    coordinate_max=0;

    double coordinate;

    int view_max=3;

    int arm_max=7;


    int point_max=2;



    for(int i=0; i<lista->machine_coords.size(); i++)
    {
        this->machine_coords.push_back(lista->machine_coords[i]);

    }

    for(int i=0; i<lista->vektor_step.size(); i++)
    {
        this->vektor_tcp.push_back(lista->vektor_step[i].array_point[7]);

    }

 for(int view=0; view < view_max; view++)
        {
          for(int arm=0; arm < arm_max; arm++)
          {
             for(int point=0; point < point_max; point++)
             {
               for(int step=0; step < lista->vektor_step.size(); step++)
               {
                     if(view==0)
                     {
                        coordinate=lista->vektor_step[step].array_point[arm+point].x;
                        if(qFabs(coordinate)>coordinate_max) coordinate_max=qFabs(coordinate);
                        lista1->array_view[view].array_arm[arm].array_point[point].coordinate1.push_back(coordinate);

                        coordinate=lista->vektor_step[step].array_point[arm+point].y;
                        if(qFabs(coordinate)>coordinate_max) coordinate_max=qFabs(coordinate);
                        lista1->array_view[view].array_arm[arm].array_point[point].coordinate2.push_back(-coordinate);


                     }
                     if(view==1)
                     {
                         coordinate=lista->vektor_step[step].array_point[arm+point].x;
                         if(qFabs(coordinate)>coordinate_max) coordinate_max=qFabs(coordinate);
                         lista1->array_view[view].array_arm[arm].array_point[point].coordinate1.push_back(lista->vektor_step[step].array_point[arm+point].x);

                         coordinate=lista->vektor_step[step].array_point[arm+point].z;
                         if(qFabs(coordinate)>coordinate_max) coordinate_max=qFabs(coordinate);
                         lista1->array_view[view].array_arm[arm].array_point[point].coordinate2.push_back(-(lista->vektor_step[step].array_point[arm+point].z));
                     }
                     if(view==2)
                     {
                         coordinate=lista->vektor_step[step].array_point[arm+point].y;
                         if(qFabs(coordinate)>coordinate_max) coordinate_max=qFabs(coordinate);
                         lista1->array_view[view].array_arm[arm].array_point[point].coordinate1.push_back(lista->vektor_step[step].array_point[arm+point].y);

                         coordinate=lista->vektor_step[step].array_point[arm+point].z;
                         if(qFabs(coordinate)>coordinate_max) coordinate_max=qFabs(coordinate);
                         lista1->array_view[view].array_arm[arm].array_point[point].coordinate2.push_back(-(lista->vektor_step[step].array_point[arm+point].z));
                     }
                 }

             }
          }
 }
}

