#ifndef LOCALIZE_COMMUNICATION_H
#define LOCALIZE_COMMUNICATION_H

#include <vector>
#include <geometry_msgs/Point.h>

namespace silver_fundamentals {

enum class PlanSuccessState {
    NONE,        // initial/unset state
    PLAN_DONE,
    PLAN_FAILED
};


extern PlanSuccessState success_state;

extern std::vector<geometry_msgs::Point> com_waypoints;

extern bool plan_exits;

} // namespace silver_fundamentals

#endif // LOCALIZE_COMMUNICATION_H
