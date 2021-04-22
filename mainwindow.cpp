#include "mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);    

}

MainWindow::~MainWindow()
{
    delete ui;
}

//Przycisk uruchamiający całą symulację
void MainWindow::on_pushButtonStart_clicked()
{
    //Odczytywanie i zapisywanie parametrów robota wprowadzonych przez użytkownika
    ROBOT_PARAMETERS robotParameters;
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
    POINT startPoint;
    startPoint.x = ui->lineEdit_startPoinX->text().toDouble();
    startPoint.y = ui->lineEdit_startPoinY->text().toDouble();
    startPoint.z = ui->lineEdit_startPoinZ->text().toDouble();


    POINT endPoint;
    endPoint.x = ui->lineEdit_endPointX->text().toDouble();
    endPoint.y = ui->lineEdit_endPointY->text().toDouble();
    endPoint.z = ui->lineEdit_endPointZ->text().toDouble();

    //Wektor zawierającu Punkt początkowy, Punkt końcowy oraz punkty podporowe, jeżeli takie zostały dodane
    trajectoryP = trajectoryPoints(supportingP, startPoint, endPoint);

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
            calculateMachineCoords(robotParameters, s, actMachineCoords);
            ui->lineEdit_fi1->setText(QString::number(actMachineCoords.fi1));
            ui->lineEdit_fi2->setText(QString::number(actMachineCoords.fi2));
            ui->lineEdit_fi3->setText(QString::number(actMachineCoords.fi3));
            ui->lineEdit_fi4->setText(QString::number(actMachineCoords.fi4));
            ui->lineEdit_fi5->setText(QString::number(actMachineCoords.fi5));

            //Zapis zmiennych do animacji
//            ITERATION  lista;
//            lista.machine_coords.push_back(actMachineCoords);

//            STEP temp;
//            temp.array_point[0] = {0,0,0};
//            temp.array_point[1] = s.coord1;
//            temp.array_point[2] = s.coord1prim;
//            temp.array_point[3] = s.coord2;
//            temp.array_point[4] = s.coord2prim;
//            temp.array_point[5] = s.coordR;
//            temp.array_point[6] = s.coordP;
//            temp.array_point[7] = s.coordTCP;
//            lista.vektor_step.push_back(temp);
        }
    }

}

//Przycisk Dodaj Punkt. Dodaje punkt podporowy wprowadzony przez użytkownika
void MainWindow::on_pushButton_2_clicked()
{
    POINT supportingPoint;
    supportingPoint.x = ui->lineEdit_supportingX->text().toDouble();
    supportingPoint.y = ui->lineEdit_supportingY->text().toDouble();
    supportingPoint.z = ui->lineEdit_supportingZ->text().toDouble();

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

//Usuwa ostatnio dodany punkt podporowy
void MainWindow::on_pushButton_usunPunkty_clicked()
{
    supportingP.pop_back();
    ui->tableWidget->removeRow(ui->tableWidget->rowCount()-1);
}


