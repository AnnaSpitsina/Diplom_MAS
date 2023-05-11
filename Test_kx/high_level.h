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

    // Структура для представления точки
    struct Point
    {
        double x, y;
    };

    //вспомогательные функции
    Point raschet_goal_point(const QVector<Point>& points); //рассчитать целевую точку
    double distance(const Point& p1, const Point& p2); //найти расстояние между точками

    //поведения
    void all_ANPA_move_to_point();  //выйти в точку
    void follow_the_leader(); //следовать за лидером

    size_t i_max_ROV;
    bool movepointFlag = true;  // флаг выполнения поведения выхода в точку
    QVector<CS_ROV*> cs_rov_arr;
    QTimer timer;

    double x_goal;
    double y_goal;


    std::function<void(void)> DrawCall;
};

#endif // HIGH_LEVEL_H
