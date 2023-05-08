#ifndef HIGH_LEVEL_H
#define HIGH_LEVEL_H

#include "cs_rov.h"
#include "trajectory.h"

extern double X[2000][2];
extern QVector<double> K;

class HIGH_LEVEL : public QObject
{
    Q_OBJECT
public:
    HIGH_LEVEL(std::function<void(void)> drawCall, size_t rov_count, QObject * parent = nullptr);
    ~HIGH_LEVEL();
    size_t i_max_ROV;
    void raschet_goal_point(); //рассчитать целевую точку
    //поведения
    void move_to_point();  //выйти в точку
    void follow_the_leader(); //следовать за лидером

    bool movepointFlag = true;  // флаг выполнения определенного поведения

    QVector<CS_ROV*> cs_rov_arr;
    double x_goal;
    double y_goal;
    QTimer timer;

    std::function<void(void)> DrawCall;
};

#endif // HIGH_LEVEL_H
