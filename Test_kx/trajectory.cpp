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

void Trajectory::draw_trajectory() {
    ui->widget->addGraph();
    ui->widget-> graph (0) -> setPen (QPen (Qt :: gray)); // Цвет кисти для рисования кривой 0
    for (float i = 0; i<2*M_PI; i+=0.05) {
    ui->widget->graph(0)->addData((K[1]+0.5*cos(i)),(K[2]+0.5*sin(i)));   //область нахождения заданной точки
    }

    ui->widget->addGraph();
    ui->widget->graph (1) -> setPen (QPen (Qt :: blue)); // Цвет кисти для рисования кривой 1 синий
    ui->widget->graph (1) -> setBrush(Qt::NoBrush);
    ui->widget->graph(1)->addData(X[22][0],X[23][0]); //перемещение аппарата 1

    ui->widget->addGraph();
    ui->widget->graph (2) -> setBrush(Qt::NoBrush); // попытки удалить заливку, но она все равно осталась :(
    ui->widget->graph (2) -> setPen (QPen (Qt :: red)); // Цвет кисти для рисования кривой 3 красный
    ui->widget->graph(2)->addData(X[122][0],X[123][0]); //перемещение аппарата 2

    ui->widget->replot();
}

Trajectory::~Trajectory()
{
    delete ui;
}
