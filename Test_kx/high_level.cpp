#include "high_level.h"
#include <QtMath>

HIGH_LEVEL::HIGH_LEVEL(size_t rov_count, QObject *parent) : QObject(parent)
{
    for (size_t idx = 0; idx < rov_count; ++idx){
        cs_rov_arr.append(new CS_ROV([this](){xy.draw_trajectory();}, idx));
    }

    xy.show();

    if (movepointFlag) {
        connect(&timer, &QTimer::timeout, this, &HIGH_LEVEL::move_to_point);
        move_to_point();
    }
}

HIGH_LEVEL::~HIGH_LEVEL() {
    while(cs_rov_arr.count()) {
        delete cs_rov_arr.takeLast();
    }
}

void HIGH_LEVEL::move_to_point() {

    for (i = 0; i<2; i++) {
    x_goal = K[100*i+1];
    y_goal = K[100*i+2];
    if (((copysign(1.0, x_goal)*1.2*x_goal > copysign(1.0, X[100*i+22][0])*X[100*i+22][0]) and (copysign(1.0, X[100*i+22][100*i+22])*X[100*i+22][0] > copysign(1.0, x_goal)*0.8*x_goal))
        and ((copysign(1.0, y_goal)*1.2*y_goal > copysign(1.0, X[100*i+23][0])*X[100*i+23][0]) and (copysign(1.0, X[100*i+23][0])*X[100*i+23][0] > copysign(1.0, y_goal)*0.8*y_goal))) {
        cs_rov_arr[0]->closed_contour = false;
    } else {
        //случаи точек на осях   //copysign(1.0, y_goal) - функция определения знака
        if ((x_goal == 0) and (y_goal>0)) {
            X[100*i+31][0] = 90;
            if (X[100*i+21][0] < 0.99*X[100*i+31][0]) {
                X[100*i+32][0] = 0;
            } else X[100*i+32][0] = K[100*i+3];}
        if ((x_goal == 0) and (y_goal<0)) {
            X[100*i+31][0] = -90;
            if (X[100*i+21][0] > 0.99*X[100*i+31][0]) {
                X[100*i+32][0] = 0;
            } else X[100*i+32][0] = K[100*i+3];}
        if ((y_goal == 0) and (x_goal > 0)) {X[100*i+31][0] = 0; X[100*i+32][0] = K[100*i+3];}
        if ((y_goal == 0) and (x_goal < 0)) {X[100*i+31][0] = 0; X[100*i+32][0] = -K[100*i+3];}


        if (x_goal<0) {  //левая полуплоскость
            if (y_goal<0) {   // 3 четверть
                X[100*i+31][0] = -180 + qAtan((y_goal-X[i+23][0])/(x_goal-X[i+22][0]))*(180/M_PI); // расчет угла курса и его перевод в градусы
                X[100*i+32][0] = K[100*i+3];
            }
            if ((y_goal > 0)) {
            X[100*i+31][0] = 180 + qAtan((y_goal-X[i+23][0])/(x_goal-X[i+22][0]))*(180/M_PI); // 2 четверть
            X[100*i+32][0] = K[100*i+3];
            }
        }
        if (x_goal>0) {
            if (y_goal >0) { // правая полуплоскость
                X[100*i+31][0] = qAtan((y_goal-X[i+23][0])/(x_goal-X[i+22][0]))*(180/M_PI); // 1 четверть
                X[100*i+32][0] = K[100*i+3];
            }
            if (y_goal <0) {
                X[100*i+31][0] = qAtan((y_goal-X[i+23][0])/(x_goal-X[i+22][0]))*(180/M_PI); // 2 четверть
                X[100*i+32][0] = K[100*i+3];
            }
        }
}
        cs_rov_arr[i]->regulators();
        cs_rov_arr[i]->BFS_DRK(X[100*i+45][0], 0, 0, X[100*i+46][0], 0, 0);
        cs_rov_arr[i]->writeDataToModel();
        timer.start(20);
}
}
