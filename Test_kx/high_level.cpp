#include "high_level.h"
#include <QtMath>

using namespace std;


HIGH_LEVEL::HIGH_LEVEL(std::function<void(int)> drawCall, size_t rov_count, QObject *parent) : QObject(parent), i_max_ROV(rov_count), DrawCall(drawCall)
{
    for (size_t idx = 0; idx < i_max_ROV; ++idx){
        cs_rov_arr.append(new CS_ROV(idx));}

    connect(&timer_1, &QTimer::timeout, this, &HIGH_LEVEL::all_ANPA_move_to_point);
    connect(&timer_2, &QTimer::timeout, this, &HIGH_LEVEL::all_ANPA_move_to_different_point);

    if (movepointFlag) {
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
        all_ANPA_move_to_point();
    }

    if (distribute_pointFlag) {
        cnt_goal_point = K[0];
        for (size_t l=0;  l<cnt_goal_point; l++) {
            arr_goal_point_x.append(K[80+l]);
            arr_goal_point_y.append(K[90+l]);
        }
        all_ANPA_distribute_point();
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
        //timer.stop();
    }
    DrawCall(1);

    timer_1.start(10);
}

void HIGH_LEVEL::all_ANPA_distribute_point(){

    QVector<QVector<double>> arr_dist; //двумерный вектор для расстояний от па до целевых точек
    for (size_t a=0; a<i_max_ROV; a++ ){
        QVector <double> v;
        for (size_t b=0; b<cnt_goal_point; b++) {
            v.push_back(qSqrt(qPow((K[100*a+5] - arr_goal_point_x[b]), 2)+qPow((K[100*a+6] - arr_goal_point_y[b]), 2)));  //передала через K начальную координату, хотя по идее лучше через текущее положение
        }
        arr_dist.push_back(v);
    }

    QVector<bool> arr_free_ANPA;    //массив для отслеживания аппаратов, которые уже выбрали точку
    for (size_t c=0; c<i_max_ROV; c++) {
        arr_free_ANPA.push_back({true});
    }

    for (size_t i=0; i<cnt_goal_point; ++i) {
        int minIdx = -1;
        double minDist = -1;
        for (size_t j=0; j<i_max_ROV; ++j) {
            if (arr_free_ANPA[j]) {
                if ((minIdx == -1) || (arr_dist[j][i]<minDist)) {
                    minDist = arr_dist[j][i];
                    minIdx = j;
                }
            }
        }
        arr_free_ANPA[minIdx] = false;
        ANPA_and_point.push_back({(double)minIdx, K[80+i], K[90+i]});
    }
        timer_2.start(10);

//    all_ANPA_move_to_different_point();
//    for (int n=0; n<ANPA_and_point.size(); n++) {
//        for (int m=0; m<ANPA_and_point[n].size(); m++) {
//            qDebug()<<ANPA_and_point[n][m];
//            qDebug()<< minDist;
//            qDebug()<< minIdx;
//        }
//    }
}


//    for (size_t i = 0; i<i_max_ROV; i++) {
//        cs_rov_arr[i]->move_to_point(?, ?);}

//}
//}

void HIGH_LEVEL::all_ANPA_move_to_different_point() {

    bool all_done = true;
    for (int i = 0; i<ANPA_and_point.size(); i++) { //ANPA_and_point.size() - количество строк вектора векторов, т е кол-во ПА
   //     qDebug()<< ANPA_and_point[i][0] << ANPA_and_point[i][1] << ANPA_and_point[i][2];
        cs_rov_arr[ANPA_and_point[i][0]]->move_to_point(ANPA_and_point[i][1], ANPA_and_point[i][2]);
        all_done &= cs_rov_arr[ANPA_and_point[i][0]]->point_reach;
    }
    if (all_done) {
        distribute_pointFlag = false;
        //timer.stop();
    }
    DrawCall(2);
}
