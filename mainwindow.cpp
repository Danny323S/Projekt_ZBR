#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "structures.h"
#include <QMessageBox>

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


void MainWindow::on_pushButton_clicked()
{
    ROBOT_PARAMETERS robotParameters;
    robotParameters.arm1 = ui->lineEdit_L1->text().toDouble();
    robotParameters.arm2 = ui->lineEdit_L2->text().toDouble();
    robotParameters.arm3 = ui->lineEdit_L3->text().toDouble();
    robotParameters.arm4 = ui->lineEdit_L4->text().toDouble();
    robotParameters.arm5 = ui->lineEdit_L5->text().toDouble();
    robotParameters.arm6 = ui->lineEdit_L6->text().toDouble();

    robotParameters.d = ui->lineEdit_d->text().toDouble();
    robotParameters.e = ui->lineEdit_e->text().toDouble();

    robotParameters.OAngle = ui->lineEdit_OAngle->text().toDouble();
    robotParameters.YAngle = ui->lineEdit_YAngle->text().toDouble();

    //robotParameters.delta1 = ui->comboBox_delta1

}
