#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "ui_mainwindow.h"
#include "structures.h"
#include "calculation.h"
#include <QMainWindow>
#include <QMessageBox>
#include <QVector>
#include "database.h"
#include "animation.h"
#include <QPixmap>

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
    void animation();
    ~MainWindow();

private slots:
    void on_pushButtonStart_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_usunPunkty_clicked();

    void on_horizontalSlider_sliderMoved(int position);

    void machine_coordinates_display();

private:
    Ui::MainWindow *ui;

    QVector<Point> supportingP;
    QVector<Point> trajectoryP;

    Iteration *lista;

    ROBOT_PARAMETERS robotParameters;
    MACHINE_COORDS actMachineCoords;
    ROBOT_COORDS_SYSTEMS s;

    //lista do animacji
    Robot lista1;

    //objekt transformuj�cy liste z punktami na liste do animacji
    Database *database;

    //animacja
    CRobot_animation *scene;

    //czas animacji
    double _time;

    int positon_previous;

    int i=0;
    QTimer *timer;


};
#endif // MAINWINDOW_H
