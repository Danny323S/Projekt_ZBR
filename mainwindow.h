#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "ui_mainwindow.h"
#include "structures.h"
#include "calculation.h"
#include <QMainWindow>
#include <QMessageBox>
#include <QVector>

#include <QTableWidget>
#include <QTableWidgetItem>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_pushButtonStart_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_usunPunkty_clicked();

    void on_pushButton_Start_clicked();

private:
    Ui::MainWindow *ui;

    QVector<POINT> supportingP;
    QVector<POINT> trajectoryP;

    ITERATION list;

    MACHINE_COORDS actMachineCoords;
    ROBOT_COORDS_SYSTEMS s;


};
#endif // MAINWINDOW_H
