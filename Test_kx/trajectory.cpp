#include "trajectory.h"
#include "ui_trajectory.h"
#include <QtMath>

Trajectory::Trajectory(QWidget *parent) :
    QMainWindow(parent)
{
    ui = new Ui::Trajectory ();
    ui->setupUi(this);
    ui->widget->xAxis->setRange(-10, 10);
    ui->widget->yAxis->setRange(-10, 10);

    ui->widget-> setInteractions (QCP :: iRangeDrag | QCP :: iRangeZoom); // Перетаскиваемый + масштабирование колеса прокрутки

    ui->widget-> xAxis-> setLabel (QStringLiteral ("X_global")); // отображение текста по оси X
    ui->widget-> yAxis-> setLabel (QStringLiteral ("Y_global")); // отображение текста оси Y

    ui->widget->xAxis->grid()->setSubGridVisible(true); //пунктирные линии сетки
    ui->widget->yAxis->grid()->setSubGridVisible(true);

    ui->widget->xAxis->setUpperEnding(QCPLineEnding::esSpikeArrow);
    ui->widget->yAxis->setUpperEnding(QCPLineEnding::esSpikeArrow);
}

void Trajectory::draw_trajectory(int flag) {

    ui->widget->addGraph();
    ui->widget->graph (0) -> setPen (QPen (Qt :: red)); // Цвет кисти для рисования кривой 1 синий
    ui->widget->graph (0) -> setBrush(Qt::NoBrush);
    ui->widget->graph(0)->addData(X[22][0],X[23][0]); //перемещение аппарата 1

    ui->widget->addGraph();
    ui->widget->graph (1) -> setPen (QPen (Qt :: red)); // Цвет кисти для рисования кривой 2 красный
    ui->widget->graph(1)->addData(X[122][0],X[123][0]); //перемещение аппарата 2

    ui->widget->addGraph();
    ui->widget->graph (2) -> setPen (QPen (Qt :: red)); // Цвет кисти для рисования кривой 3 красный
    ui->widget->graph(2)->addData(X[222][0],X[223][0]); //перемещение аппарата 3

    ui->widget->addGraph();
    ui->widget->graph (3) -> setPen (QPen (Qt :: red)); // Цвет кисти для рисования кривой 4 красный
    ui->widget->graph(3)->addData(X[322][0],X[323][0]); //перемещение аппарата 4

    ui->widget->addGraph();
    ui->widget->graph (4) -> setPen (QPen (Qt :: red)); // Цвет кисти для рисования кривой 5 красный
    ui->widget->graph(4)->addData(X[422][0],X[423][0]); //перемещение аппарата 5

    if (flag == 1) {

        ui->widget->addGraph();
        ui->widget-> graph (5) -> setPen (QPen (Qt :: gray)); // Цвет кисти для рисования кривой 0
            for (float i = 0; i<2*M_PI; i+=0.01) {
            ui->widget->graph(5)->addData((X[1][0]+0.5*cos(i)),(X[2][0]+0.5*sin(i)));   //область нахождения заданной точки
            }
    }

    if (flag == 2) {
        ui->widget->addGraph();
        ui->widget-> graph (5) -> setPen (QPen (Qt :: gray)); // Цвет кисти для рисования кривой 0
        for (float i = 0; i<2*M_PI; i+=0.01) {
            ui->widget->graph(5)->addData((K[80]+0.5*cos(i)),(K[90]+0.5*sin(i)));   //область нахождения заданной точки
        }
        ui->widget->addGraph();
        ui->widget-> graph (6) -> setPen (QPen (Qt :: gray)); // Цвет кисти для рисования кривой 0
        for (float i = 0; i<2*M_PI; i+=0.01) {
            ui->widget->graph(6)->addData((K[81]+0.5*cos(i)),(K[91]+0.5*sin(i)));   //область нахождения заданной точки
        }
        ui->widget->addGraph();
        ui->widget-> graph (7) -> setPen (QPen (Qt :: gray)); // Цвет кисти для рисования кривой 0
        for (float i = 0; i<2*M_PI; i+=0.01) {
            ui->widget->graph(7)->addData((K[82]+0.5*cos(i)),(K[92]+0.5*sin(i)));   //область нахождения заданной точки
        }
        ui->widget->addGraph();
        ui->widget-> graph (8) -> setPen (QPen (Qt :: gray)); // Цвет кисти для рисования кривой 0
        for (float i = 0; i<2*M_PI; i+=0.01) {
            ui->widget->graph(8)->addData((K[83]+0.5*cos(i)),(K[93]+0.5*sin(i)));   //область нахождения заданной точки
        }
        ui->widget->addGraph();
        ui->widget-> graph (9) -> setPen (QPen (Qt :: gray)); // Цвет кисти для рисования кривой 0
        for (float i = 0; i<2*M_PI; i+=0.01) {
            ui->widget->graph(9)->addData((K[84]+0.5*cos(i)),(K[94]+0.5*sin(i)));   //область нахождения заданной точки
        }
    }

    ui->widget->replot();

}



Trajectory::~Trajectory()
{
    delete ui;
}
