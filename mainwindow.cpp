#include "mainwindow.h"
#include "ui_mainwindow.h"



MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //Dodawanie obrazka
    QPixmap icon(":/new/prefix1/img/icon.jpg");
    int width = ui->label_icon->width();
    int height = ui->label_icon->height();
    ui->label_icon->setPixmap(icon.scaled(width, height, Qt::KeepAspectRatio));

}

void MainWindow::animation()
{

    int positon_previous=11;

    ui->horizontalSlider->setValue(11);

    Robot lista1;

    database=new Database(lista,&lista1);

    double k = database->coordinate_max;



    double _time=(ui->spinBox_time->text()).toDouble();

    scene=new CRobot_animation(&lista1,_time);


    ui->graphicsView->setRenderHint(QPainter::Antialiasing);

    ui->graphicsView->setScene(scene->view_xy);

    //ui->graphicsView->fitInView(QRectF(-182,-160,364+k,320+k));
   ui->graphicsView->fitInView(QRectF(-182,-160,364+k,320+k));

    scene->view_xy->setSceneRect(-182,-160,364,320);


    ui->graphicsView_2->setRenderHint(QPainter::Antialiasing);

    ui->graphicsView_2->setScene(scene->view_xz);

    ui->graphicsView_2->fitInView(QRectF(-182,-160,364+k,320+k));

    scene->view_xz->setSceneRect(-182,-255-k/2,364,320);


    ui->graphicsView_3->setRenderHint(QPainter::Antialiasing);

    ui->graphicsView_3->setScene(scene->view_yz);

    ui->graphicsView_3->fitInView(QRectF(-182,-160,364+k,320+k));

    ui->graphicsView_3->setSceneRect(-182,-255-k/2,364,320);

    i=0;
    machine_coordinates_display();
    timer = new QTimer(this);
    timer->setInterval(_time/lista->machine_coords.size()*1000);
    connect(timer, SIGNAL(timeout()), this, SLOT(machine_coordinates_display()));
    timer->start();

    //delete database;


}

void MainWindow::machine_coordinates_display()
{

        ui->lineEdit_fi1->setText(QString::number(database->machine_coords[i].fi1));
        ui->lineEdit_fi2->setText(QString::number(database->machine_coords[i].fi2));
        ui->lineEdit_fi3->setText(QString::number(database->machine_coords[i].fi3));
        ui->lineEdit_fi4->setText(QString::number(database->machine_coords[i].fi4));
        ui->lineEdit_fi5->setText(QString::number(database->machine_coords[i].fi5));
        ui->lineEdit_startPointX_2->setText(QString::number(database->vektor_tcp[i].x));
        ui->lineEdit_startPointY_2->setText(QString::number(database->vektor_tcp[i].y));
        ui->lineEdit_startPointZ_2->setText(QString::number(database->vektor_tcp[i].z));
        i++;
        if(i >= lista->machine_coords.size()) timer->stop();
}

MainWindow::~MainWindow()
{
    delete ui;
}

//Przycisk uruchamiający całą symulację
void MainWindow::on_pushButtonStart_clicked()
{
    s.coord2.y=0;
    //Odczytywanie i zapisywanie parametrów robota wprowadzonych przez użytkownika
    robotParameters.l1 = ui->lineEdit_L1->text().toDouble();
    robotParameters.l2 = ui->lineEdit_L2->text().toDouble();
    robotParameters.l3 = ui->lineEdit_L3->text().toDouble();
    robotParameters.l4 = ui->lineEdit_L4->text().toDouble();
    robotParameters.l5 = ui->lineEdit_L5->text().toDouble();
    robotParameters.l6 = ui->lineEdit_L6->text().toDouble();

    robotParameters.d = ui->lineEdit_d->text().toDouble();
    robotParameters.e = ui->lineEdit_e->text().toDouble();

    robotParameters.OAngle = ui->lineEdit_OAngle->text().toDouble();
    robotParameters.YAngle = ui->lineEdit_YAngle->text().toDouble();

    robotParameters.delta1 = ui->comboBox_delta1->currentText().toDouble();
    robotParameters.delta2 = ui->comboBox_delta2->currentText().toDouble();
    robotParameters.delta5 = ui->comboBox_delta5->currentText().toDouble();

    //Odczytywanie punktu początkowego i końcowego wprowadzonego przez użytkownika
    Point startPoint;
    startPoint.x = ui->lineEdit_startPointX->text().toDouble();
    startPoint.y = ui->lineEdit_startPointY->text().toDouble();
    startPoint.z = ui->lineEdit_startPointZ->text().toDouble();

    if(!checkPoint(robotParameters, startPoint)){
        QMessageBox::warning(this, "Ostrzeżenie", "Wprowadzony punkt początkowy znajduje się po za polem roboczym");
        startPoint.x = 0;
        startPoint.y = 0;
        startPoint.z = 0;

        ui->lineEdit_startPointX->clear();
        ui->lineEdit_startPointY->clear();
        ui->lineEdit_startPointZ->clear();

    }

    Point endPoint;
    endPoint.x = ui->lineEdit_endPointX->text().toDouble();
    endPoint.y = ui->lineEdit_endPointY->text().toDouble();
    endPoint.z = ui->lineEdit_endPointZ->text().toDouble();

    if(!checkPoint(robotParameters, endPoint)){
        QMessageBox::warning(this, "Ostrzeżenie", "Wprowadzony punkt końcowy znajduje się po za polem roboczym");
        endPoint.x = 0;
        endPoint.y = 0;
        endPoint.z = 0;

        ui->lineEdit_endPointX->clear();
        ui->lineEdit_endPointY->clear();
        ui->lineEdit_endPointZ->clear();
    }

    //Wektor zawierającu Punkt początkowy, Punkt końcowy oraz punkty podporowe, jeżeli takie zostały dodane
    trajectoryP = trajectoryPoints(supportingP, startPoint, endPoint);

    //True jeżeli wyniki wszystkich obliczeń są poprawne
    // czyli wszystkie pierwiastki są dodatnie
    bool calculationResult = true;
    
    lista = new Iteration;

    for(int j = 0; j < trajectoryP.size() - 1; j++ ){
        for(int i = 0; i <= ui->spinBox_step->text().toDouble(); i++){

            //funkcja interpolacji liniowej zwraca następne położenia TCP między wprowadzoonymi
            //punktami. Interpolacja realizowana jest o zdany krok
            s.coordTCP = linearInterpolation(i, j, ui->spinBox_step->text().toDouble(), trajectoryP[j], trajectoryP[j+1]);

            //Pomijanie powtarzających się punktów (punktów początkowych oprócz pierwszego)
            //Pomijanie punktów początkowych, które w poprzedniej iteracji gunkcji "j" były punktami końcowimi
            if(j != 0 && i == 0)
                continue;

            //Obliczenie współrzędnych maszynowych, zwracanie przez referencje pozycji
            //układów współrzędnych oraz współrzędnych maszynowych.
            calculationResult = calculateMachineCoords(robotParameters, s, actMachineCoords);

            //Zapis zmiennych do animacji

            lista->machine_coords.push_back(actMachineCoords);

            Step temp;
            temp.array_point[0] = {0,0,0};
            temp.array_point[1] = {s.coord1.x,s.coord1.y,s.coord1.z};
            temp.array_point[2] = {s.coord1prim.x,s.coord1prim.y,s.coord1prim.z};
            temp.array_point[3] = {s.coord2.x,s.coord2.y,s.coord2.z};
            temp.array_point[4] = {s.coord2prim.x,s.coord2prim.y,s.coord2prim.z};
            temp.array_point[5] = {s.coordR.x,s.coordR.y,s.coordR.z};
            temp.array_point[6] = {s.coordP.x,s.coordP.y,s.coordP.z};
            temp.array_point[7] = {s.coordTCP.x,s.coordTCP.y,s.coordTCP.z};
            lista->vektor_step.push_back(temp);

            if(checksafetyCondition(s) == false){
                QMessageBox::warning(this, "Ostrzeżenie", "Warunke bezpiecznego przejścia nie spełniony\n Dodaj punkty podporowe");
                break;
            }

            if(calculationResult == false){
                QMessageBox::warning(this, "Ostrzeżenie", "Wproadzone dane są błędne");
                break;
            }
        }
        if(checksafetyCondition(s) == false){
            break;
        }

        if(calculationResult == false){
            break;
        }
    }


    if(calculationResult == true && checksafetyCondition(s) == true){
            animation();
    }

    //supportingP.clear();
    //trajectoryP.clear();


}

//Przycisk Dodaj Punkt. Dodaje punkt podporowy wprowadzony przez użytkownika
void MainWindow::on_pushButton_2_clicked()
{
    Point supportingPoint;
    supportingPoint.x = ui->lineEdit_supportingX->text().toDouble();
    supportingPoint.y = ui->lineEdit_supportingY->text().toDouble();
    supportingPoint.z = ui->lineEdit_supportingZ->text().toDouble();

    if(!checkPoint(robotParameters, supportingPoint)){
        QMessageBox::warning(this, "Ostrzeżenie", "Wprowadzony punkt podporowy znajduje się po za polem roboczym");
        supportingPoint.x = 0;
        supportingPoint.y = 0;
        supportingPoint.z = 0;

        ui->lineEdit_supportingX->clear();
        ui->lineEdit_supportingY->clear();
        ui->lineEdit_supportingZ->clear();
    } else {
        supportingP.push_back(supportingPoint);

        ui->tableWidget->insertRow(ui->tableWidget->rowCount());
        for(int i = 0; i < ui->tableWidget->columnCount(); i++){
            QTableWidgetItem *item = new QTableWidgetItem;

            if(i == 0)
                item->setText(QString::number(supportingP[supportingP.size()-1].x));
            if(i == 1)
                item->setText(QString::number(supportingP[supportingP.size()-1].y));
            if(i == 2)
                item->setText(QString::number(supportingP[supportingP.size()-1].z));

            ui->tableWidget->setItem(ui->tableWidget->rowCount() - 1, i, item);
        }


        ui->lineEdit_supportingX->clear();
        ui->lineEdit_supportingY->clear();
        ui->lineEdit_supportingZ->clear();
    }
}

//Usuwa ostatnio dodany punkt podporowy
void MainWindow::on_pushButton_usunPunkty_clicked()
{
    if(!(ui->tableWidget->rowCount() == 0)){
        supportingP.pop_back();
        ui->tableWidget->removeRow(ui->tableWidget->rowCount()-1);
    }
}

void MainWindow::on_horizontalSlider_sliderMoved(int position)
{

    if(position>positon_previous)
    {
        ui->graphicsView->scale(1.1,1.1);
        ui->graphicsView_2->scale(1.1,1.1);
        ui->graphicsView_3->scale(1.1,1.1);
    }
   if(position<positon_previous)
   {
       ui->graphicsView->scale(1/1.1,1/1.1);
       ui->graphicsView_2->scale(1/1.1,1/1.1);
       ui->graphicsView_3->scale(1/1.1,1/1.1);
   }
    positon_previous=position;

}



