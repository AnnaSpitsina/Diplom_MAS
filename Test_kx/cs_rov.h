#ifndef CS_ROV_H_
#define CS_ROV_H_

#include "rov_model.h"
#include "math.h"
#include <qmath.h>
#include <QTime>
#include <QDebug>

#include <functional>

const QString ConfigFile = "protocols.conf";
const QString XI = "x";
const QString KI = "k";


class CS_ROV : public QObject
{
    Q_OBJECT
public:
    CS_ROV(size_t idx, QObject * parent = nullptr);
    size_t idx;
    void regulators();
    void BFS_DRK(double Upsi, double Uteta, double Ugamma, double Ux, double Uy, double Uz);
    void writeDataToModel();
    void move_to_point(double x_goal, double y_goal);
    ROV_Model model;
    bool closed_contour = true;
    bool point_reach = false;
    QTimer timer;


};
#endif // CS_ROV_H_
