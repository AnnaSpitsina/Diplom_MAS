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
    HIGH_LEVEL(std::function<void(int)> drawCall, size_t rov_count, QObject * parent = nullptr);
    ~HIGH_LEVEL();

    // Структура для представления точки
    struct Point
    {
        double x, y;
    };

    //вспомогательные функции
    Point raschet_goal_point(const QVector<Point>& points); //рассчитать целевую точку
    double distance(const Point& p1, const Point& p2); //найти расстояние между точками
    void all_ANPA_distribute_point(); //расчет ближайших к точкам аппаратов

    //поведения
    void all_ANPA_move_to_point();  //выйти в точку
    void all_ANPA_move_to_different_point() ; //выйти к ближайшим точкам
    void follow_the_leader(); //следовать за лидером

    size_t i_max_ROV;
    size_t cnt_goal_point; //количество целевых точек
    QVector<double> arr_goal_point_x; //вектор координат х всех целевых точек
    QVector<double> arr_goal_point_y; //вектор координат у всех целевых точек
    QVector < QVector <double> > ANPA_and_point;
    bool movepointFlag = false;  // флаг выполнения поведения выхода в точку
    bool distribute_pointFlag = true;  //флаг выполнения поведения распределения целевых точек
    QVector<CS_ROV*> cs_rov_arr;
    QTimer timer_1;
    QTimer timer_2;

    double x_goal;
    double y_goal;


    std::function<void(int)> DrawCall;
};

#endif // HIGH_LEVEL_H
