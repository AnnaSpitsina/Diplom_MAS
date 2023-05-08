#include "high_level.h"
#include <QtMath>


HIGH_LEVEL::HIGH_LEVEL(std::function<void(void)> drawCall,  size_t rov_count, QObject *parent) : QObject(parent), i_max_ROV(rov_count), DrawCall(drawCall)
{
    for (size_t idx = 0; idx < i_max_ROV; ++idx){
        cs_rov_arr.append(new CS_ROV(idx));
    }

    if (movepointFlag) {
        connect(&timer, &QTimer::timeout, this, &HIGH_LEVEL::move_to_point);
        timer.start(20);
    }
}

HIGH_LEVEL::~HIGH_LEVEL() {
    while(cs_rov_arr.count()) {
        delete cs_rov_arr.takeLast();
    }
}

void HIGH_LEVEL::move_to_point() {

    double x_v_SK_sv_x_y_goal[i_max_ROV];
    double y_v_SK_sv_x_y_goal[i_max_ROV];

    for (size_t idx = 0; idx < i_max_ROV; ++idx) {
        x_goal = K[1];
        y_goal = K[2];

        x_v_SK_sv_x_y_goal[idx] = X[100*idx+22][0] - x_goal; //координаты аппаратов, в случае СК с началом координат в точке-цели
        y_v_SK_sv_x_y_goal[idx] = X[100*idx+23][0] - y_goal;

//        if ((x_v_SK_sv_x_y_goal[idx] == 0) and (y_v_SK_sv_x_y_goal[idx]<y_goal)) {
//            X[100*idx+31][0] = 90;
//            if (X[100*idx+21][0] < 0.99*X[100*idx+31][0]) {  //сначала  ПА поворачивается, как только повернется - задаем скорость
//                X[100*idx+32][0] = 0;
//            } else {
//                X[100*idx+32][0] = K[100*idx+3];}
//        }
//        if ((x_v_SK_sv_x_y_goal[idx] == 0) and (y_v_SK_sv_x_y_goal[idx]>y_goal)) {
//            X[100*idx+31][0] = -90;
//            if (X[100*idx+21][0] > 0.99*X[100*idx+31][0]) {
//                X[100*idx+32][0] = 0;
//            } else {
//                X[100*idx+32][0] = K[100*idx+3];}
//        }
//        if ((y_v_SK_sv_x_y_goal[idx] == 0) and (x_v_SK_sv_x_y_goal[idx] < 0)) {
//            X[100*idx+31][0] = 0; X[100*idx+32][0] = K[100*idx+3];
//        }
//        if ((y_v_SK_sv_x_y_goal[idx] == 0) and (x_v_SK_sv_x_y_goal[idx] > 0)) {
//            X[100*idx+31][0] = 0; X[100*idx+32][0] = -K[100*idx+3];
//        }

        if (qSqrt(qPow((X[100*idx+22][0] - x_goal), 2)+qPow((X[100*idx+23][0] - y_goal), 2)) < 1.5) {  // проверка расстояния до целевой точки, если < 2м, то размыкаю контура
            cs_rov_arr[idx]->closed_contour = false;

            qDebug()<< cs_rov_arr[1]->closed_contour;

        } else {
            if (x_v_SK_sv_x_y_goal[idx]<0) {  //левая полуплоскость относительно целевой точки
                if (y_v_SK_sv_x_y_goal[idx]<0) {   // 3 четверть
                    X[100*idx+31][0] = qAtan(y_v_SK_sv_x_y_goal[idx]/x_v_SK_sv_x_y_goal[idx])*(180/M_PI); // расчет угла курса и его перевод в градусы
                    X[100*idx+32][0] = K[100*idx+3];
                }
                if ((y_v_SK_sv_x_y_goal[idx] > 0)) {
                X[100*idx+31][0] = qAtan(y_v_SK_sv_x_y_goal[idx]/x_v_SK_sv_x_y_goal[idx])*(180/M_PI); // 2 четверть
                X[100*idx+32][0] = K[100*idx+3];
                }
            }
            if (x_v_SK_sv_x_y_goal[idx]>0) {
                if (y_v_SK_sv_x_y_goal[idx] >0) { // правая полуплоскость относительно целевой точки
                    X[100*idx+31][0] = -180 + qAtan(y_v_SK_sv_x_y_goal[idx]/x_v_SK_sv_x_y_goal[idx])*(180/M_PI); // 1 четверть
                    X[100*idx+32][0] = K[100*idx+3];
                }
                if (y_v_SK_sv_x_y_goal[idx] <0) {
                    X[100*idx+31][0] = 180 + qAtan(y_v_SK_sv_x_y_goal[idx]/x_v_SK_sv_x_y_goal[idx])*(180/M_PI); // 4 четверть
                    X[100*idx+32][0] = K[100*idx+3];
                }
            }

        }
        cs_rov_arr[idx]->regulators();
        cs_rov_arr[idx]->BFS_DRK(X[100*idx+45][0], 0, 0, X[100*idx+46][0], 0, 0);
        cs_rov_arr[idx]->writeDataToModel();
    }
    DrawCall();

}
