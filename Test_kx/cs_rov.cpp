#include "cs_rov.h"
#include "trajectory.h"

CS_ROV::CS_ROV(std::function<void(void)> drawCall,  size_t idx, QObject *parent): QObject(parent), idx(idx),  model(&X[idx*100]), DrawCall(drawCall)
{
//    connect(&timer, &QTimer::timeout, this, &CS_ROV::tick);
//    timer.start(10);
}

/*void CS_ROV::tick()
{
    readDataFromPult();
    regulators();
    BFS_DRK(X[100*i+45][0], 0, 0, X[100*i+46][0], 0, 0);
    writeDataToModel();
}

void CS_ROV::readDataFromPult()
{
    X[100*i+31][0] = K[100*i+1]; //курс (в градусах)
    X[100*i+32][0] = K[100*i+2]; //скорость по маршу
}       */

void CS_ROV::regulators()
{
    if (closed_contour == true) {
        X[100*idx+41][0] = X[100*idx+31][0]-X[100*idx+21][0];
        X[100*idx+42][0] = K[100*idx+11]*X[100*idx+41][0];
        X[100*idx+43][0] = K[100*idx+13]*X[100*idx+31][0];
        X[100*idx+44][0] = K[100*idx+12]*X[100*idx+12][0];
        X[100*idx+45][0] = X[100*idx+42][0]+X[100*idx+43][0]-X[100*idx+44][0]; //Upsi

/*      X[100*i+42][0] = K[100*i+11]*X[100*i+31][0];
        X[100*i+44][0] = K[100*i+12]*X[100*i+12][0];
        X[100*i+45][0] = X[100*i+42][0]-X[100*i+44][0];
        qDebug()<< X[100*i+44][0];   */    //для настройки контуров

//      X[100*i+46][0] = K[100*i+14]*X[100*i+13][0]; //Ux
//?      X[100*i+46][0] = K[100*i+14]*(X[100*i+32][0]-X[100*i+13][0])*X[32][0]; надо подгонять, но работает, но точность так себе

        X[100*idx+46][0] = K[100*idx+14]*X[100*idx+32][0]; //Ux   как раньше работало
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
    DrawCall();
    model.tick(X[100*idx+50][0], X[100*idx+60][0], X[100*idx+70][0], X[100*idx+80][0], X[100*idx+90][0], X[100*idx+100][0], 0.01);
}
