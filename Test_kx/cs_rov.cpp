#include "cs_rov.h"
#include "trajectory.h"

CS_ROV::CS_ROV(size_t idx, QObject *parent): QObject(parent), idx(idx),  model(&X[idx*100], K[100 * idx + 5], K[100 * idx + 6], K[100 * idx + 7])
{

}

void CS_ROV::regulators()
{
    if (closed_contour == true) {
        X[100*idx+41][0] = X[100*idx+31][0]-X[100*idx+21][0];
        X[100*idx+42][0] = K[100*idx+11]*X[100*idx+41][0];
        X[100*idx+43][0] = K[100*idx+13]*X[100*idx+31][0];
        X[100*idx+44][0] = K[100*idx+12]*X[100*idx+12][0];
        X[100*idx+45][0] = X[100*idx+42][0]+X[100*idx+43][0]-X[100*idx+44][0]; //Upsi

        X[100*idx+46][0] = K[100*idx+14]*X[100*idx+32][0]; //Ux
        } else {
            X[100*idx+45][0] = 0;
            X[100*idx+46][0] = 0;
    }
}

void CS_ROV::BFS_DRK(double Upsi, double Uteta, double Ugamma, double Ux, double Uy, double Uz)
{
    X[100*idx+50][0] = (K[100*idx+20]*Ux + K[100*idx+21]*Uy + K[100*idx+22]*Uz + K[100*idx+23]*Ugamma + K[100*idx+24]*Uteta + K[100*idx+25]*Upsi)*K[100*idx+26];//U1
    X[100*idx+60][0] = (K[100*idx+30]*Ux + K[100*idx+31]*Uy + K[100*idx+32]*Uz + K[100*idx+33]*Ugamma + K[100*idx+34]*Uteta + K[100*idx+35]*Upsi)*K[100*idx+36];//U2
    X[100*idx+70][0] = (K[100*idx+40]*Ux + K[100*idx+41]*Uy + K[100*idx+42]*Uz + K[100*idx+43]*Ugamma + K[100*idx+44]*Uteta + K[100*idx+45]*Upsi)*K[100*idx+46];//U3
    X[100*idx+80][0] = (K[100*idx+50]*Ux + K[100*idx+51]*Uy + K[100*idx+52]*Uz + K[100*idx+53]*Ugamma + K[100*idx+54]*Uteta + K[100*idx+55]*Upsi)*K[100*idx+56];//U4
    X[100*idx+90][0] = (K[100*idx+60]*Ux + K[100*idx+61]*Uy + K[100*idx+62]*Uz + K[100*idx+63]*Ugamma + K[100*idx+64]*Uteta + K[100*idx+65]*Upsi)*K[100*idx+66];//U5
    X[100*idx+100][0] = (K[100*idx+70]*Ux + K[100*idx+71]*Uy + K[100*idx+72]*Uz + K[100*idx+73]*Ugamma + K[100*idx+74]*Uteta + K[100*idx+75]*Upsi)*K[100*idx+76];//U6
}

void CS_ROV::writeDataToModel()
{

    model.tick(X[100*idx+50][0], X[100*idx+60][0], X[100*idx+70][0], X[100*idx+80][0], X[100*idx+90][0], X[100*idx+100][0], 0.01);
}

void CS_ROV::move_to_point(double x_goal, double y_goal) {

    double x_v_SK_sv_x_y_goal;
    double y_v_SK_sv_x_y_goal;

        x_v_SK_sv_x_y_goal = X[100*idx+22][0] - x_goal; //координаты аппаратов, в случае СК с началом координат в точке-цели
        y_v_SK_sv_x_y_goal = X[100*idx+23][0] - y_goal;

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

       if (qSqrt(qPow((X[100*idx+22][0] - x_goal), 2)+qPow((X[100*idx+23][0] - y_goal), 2)) < 2) {  // проверка расстояния до целевой точки, если < 2м, то размыкаю контура
            closed_contour = false;
            point_reach = true;

        } else {
           if (x_v_SK_sv_x_y_goal < 0) {  //левая полуплоскость относительно целевой точки
                if (y_v_SK_sv_x_y_goal <= 0) {   // 3 четверть
                    X[100*idx+31][0] = qAtan(y_v_SK_sv_x_y_goal/x_v_SK_sv_x_y_goal)*(180/M_PI); // расчет угла курса и его перевод в градусы
                    if (X[100*idx+21][0] < 0.95*X[100*idx+31][0]) {
                       X[100*idx+32][0] = 0;
                    } else {X[100*idx+32][0] = K[100*idx+3];}
                    }
                if ((y_v_SK_sv_x_y_goal > 0)) {
                    X[100*idx+31][0] = qAtan(y_v_SK_sv_x_y_goal/x_v_SK_sv_x_y_goal)*(180/M_PI); // 2 четверть
                    if (X[100*idx+21][0] > 0.95*X[100*idx+31][0]) {
                       X[100*idx+32][0] = 0;
                    } else {X[100*idx+32][0] = K[100*idx+3];}
                    }
             }
           if (x_v_SK_sv_x_y_goal >= 0) {
               if (y_v_SK_sv_x_y_goal > 0) { // правая полуплоскость относительно целевой точки
                   X[100*idx+31][0] = -180 + qAtan(y_v_SK_sv_x_y_goal/x_v_SK_sv_x_y_goal)*(180/M_PI); // 1 четверть
                   if (X[100*idx+21][0] > 0.99*X[100*idx+31][0]) {
                      X[100*idx+32][0] = 0;
                   } else {X[100*idx+32][0] = K[100*idx+3];}
               }
               if (y_v_SK_sv_x_y_goal <= 0) {
                   X[100*idx+31][0] = 180 + qAtan(y_v_SK_sv_x_y_goal/x_v_SK_sv_x_y_goal)*(180/M_PI); // 4 четверть
                   if (X[100*idx+21][0] < 0.99*X[100*idx+31][0]) {
                      X[100*idx+32][0] = 0;
                   } else {X[100*idx+32][0] = K[100*idx+3];}
               }
           }


/*
            if (x_v_SK_sv_x_y_goal < 0) {  //левая полуплоскость относительно целевой точки
                if (y_v_SK_sv_x_y_goal <= 0) {   // 3 четверть
                    X[100*idx+31][0] = qAtan(y_v_SK_sv_x_y_goal/x_v_SK_sv_x_y_goal)*(180/M_PI); // расчет угла курса и его перевод в градусы
                    X[100*idx+32][0] = K[100*idx+3];
                }
                if ((y_v_SK_sv_x_y_goal > 0)) {
                X[100*idx+31][0] = qAtan(y_v_SK_sv_x_y_goal/x_v_SK_sv_x_y_goal)*(180/M_PI); // 2 четверть
                X[100*idx+32][0] = K[100*idx+3];
                }
            }
            if (x_v_SK_sv_x_y_goal >= 0) {
                if (y_v_SK_sv_x_y_goal > 0) { // правая полуплоскость относительно целевой точки
                    X[100*idx+31][0] = -180 + qAtan(y_v_SK_sv_x_y_goal/x_v_SK_sv_x_y_goal)*(180/M_PI); // 1 четверть
                    X[100*idx+32][0] = K[100*idx+3];
                }
                if (y_v_SK_sv_x_y_goal <= 0) {
                    X[100*idx+31][0] = 180 + qAtan(y_v_SK_sv_x_y_goal/x_v_SK_sv_x_y_goal)*(180/M_PI); // 4 четверть
                    X[100*idx+32][0] = K[100*idx+3];
                }
            }    */

        }
        regulators();
        BFS_DRK(X[100*idx+45][0], 0, 0, X[100*idx+46][0], 0, 0);
        writeDataToModel();
    }


