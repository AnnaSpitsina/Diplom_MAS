#include "high_level.h"
#include <QtMath>

using namespace std;


HIGH_LEVEL::HIGH_LEVEL(std::function<void(void)> drawCall, size_t rov_count, QObject *parent) : QObject(parent), i_max_ROV(rov_count), DrawCall(drawCall)
{
    for (size_t idx = 0; idx < i_max_ROV; ++idx){
        cs_rov_arr.append(new CS_ROV(idx));}

    connect(&timer, &QTimer::timeout, this, &HIGH_LEVEL::all_ANPA_move_to_point);

    QVector<Point> points = {
        { K[5], K[6] },
        { K[105], K[106] },
        { K[205], K[206] },
        { K[305], K[306] },
        { K[405], K[406] },
    };

    Point gm = raschet_goal_point(points);
    x_goal = X[1][0] = gm.x;
    y_goal = X[2][0] = gm.y;


    if (movepointFlag) {
        all_ANPA_move_to_point();
    }
}

HIGH_LEVEL::~HIGH_LEVEL() {
    while(cs_rov_arr.count()) {
        delete cs_rov_arr.takeLast();
    }
}

double HIGH_LEVEL::distance(const Point& p1, const Point& p2)
{
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return sqrt(dx * dx + dy * dy);
}

HIGH_LEVEL::Point HIGH_LEVEL::raschet_goal_point(const QVector<Point>& points) {
    Point median = {0, 0};
    for (const auto& point : points) {
        median.x += point.x;
        median.y += point.y;
    }
    median.x /= points.size();
    median.y /= points.size();

    const int maxIterations = 100;
    const double epsilon = 1e-6;
    for (int i = 0; i < maxIterations; i++) {
        Point numerator = {0, 0};
        double denominator = 0;
        for (const auto& point : points) {
            double d = distance(median, point);
            if (d < epsilon) {
                d = epsilon;
            }
            numerator.x += point.x / d;
            numerator.y += point.y / d;
            denominator += 1 / d;
        }
        Point newMedian = {numerator.x / denominator, numerator.y / denominator};
        if (distance(median, newMedian) < epsilon) {
            break;
        }
        median = newMedian;
    }

    return median;

}


void HIGH_LEVEL::all_ANPA_move_to_point() {

    bool all_done = true;
    for (size_t i = 0; i<i_max_ROV; i++) {
        cs_rov_arr[i]->move_to_point(x_goal, y_goal);
        all_done &= cs_rov_arr[i]->point_reach;
    }
    if (all_done) {
        movepointFlag = false;
    }
    DrawCall();

    timer.start(20);
}

