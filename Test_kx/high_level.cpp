#include "high_level.h"
#include <QtMath>


HIGH_LEVEL::HIGH_LEVEL(std::function<void(void)> drawCall, size_t rov_count, QObject *parent) : QObject(parent), i_max_ROV(rov_count), DrawCall(drawCall)
{
    for (size_t idx = 0; idx < i_max_ROV; ++idx){
        cs_rov_arr.append(new CS_ROV(idx));
        connect(&timer, &QTimer::timeout, this, &HIGH_LEVEL::all_ANPA_move_to_point);
        timer.start(20);
    }

    if (movepointFlag) {
        all_ANPA_move_to_point();
    }
}

HIGH_LEVEL::~HIGH_LEVEL() {
    while(cs_rov_arr.count()) {
        delete cs_rov_arr.takeLast();
    }
}

void HIGH_LEVEL::all_ANPA_move_to_point() {

    bool all_done = true;
    for (size_t i = 0; i<i_max_ROV; i++) {
        cs_rov_arr[i]->move_to_point(K[1], K[2]);
        all_done &= cs_rov_arr[i]->point_reach;
    }
    if (all_done) {
        movepointFlag = false;
    }
    DrawCall();
}

