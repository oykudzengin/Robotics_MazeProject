#include <ros/ros.h>
#include <silver_fundamentals/ExecutePlan.h>
#include "silver_fundamentals/Laser.h"
#include <feedback_drive.h>
#include <config.h>
#include <vector>
#include <cstddef>   // for std::size_t
// Example usage
#include <iostream>

// typedef struct  movement
// {
//     direction: dir;
//     union {
//         double radius;
//         double distance;
//     } dist;
// } movement;

//movement.dist.radius;
//movement.dist.distance;
std::vector<int> compressPlan(const int plan[], std::size_t length) {

    // 1. Build an globalPlan with a leading '1'
    std::vector<int> globalPlan;
    globalPlan.reserve(length + 1);
    globalPlan.push_back(1);  // initial heading for conversion
    for (std::size_t i = 0; i < length; ++i) {
        globalPlan.push_back(plan[i]);
    }

    // 2. Compute local instructions
    std::vector<int> localPlan;
    localPlan.reserve(globalPlan.size() - 1);
    for (std::size_t i = 0; i + 1 < globalPlan.size(); ++i) {
        int diff = (globalPlan[i] - globalPlan[i+1] + 4) % 4;
        localPlan.push_back(diff);
    }

    return localPlan;
}


bool executePlan(silver_fundamentals::ExecutePlan::Request &req,
                  silver_fundamentals::ExecutePlan::Response &res) 
{

    ROS_INFO("Received a plan of %d steps", req.plan.size());
    auto driver = FeedbackDrive(3.25, 26.203, 2.0);

    std::vector<int> plan = req.plan;

    for (int i=0; i<plan.size(); i++) {
        switch (plan[i]) {
            case 0: { //right
                driver.turn_n_degrees(90, right);
                driver.drive_n_cm(40);
                break;
            }
            case 1: { //up
                driver.drive_n_cm(40);
                break;
            }
            case 2: { //left
                driver.turn__n_degrees(90,left);
                driver.drive_n_cm(10);
                break;
            }
            case 3: { //down
                driver.turn_n_degrees(180, right);
                driver.drive_n_cm(40);
                break;
            }
            default:
                ROS_ERROR("Unknown action %d", action);
        }
    }

    return true;
}


int main(int argc, char **argv) 
{
    ros::init(argc, argv, "execute_plan_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("execute_plan", executePlan);
    ROS_INFO("Ready to execute plan.");
    ros::spin();

    // int plan[] = {3, 0, 3, 2, 2, 1};
    // auto local = compressPlan(plan, sizeof(plan) / sizeof(plan[0]));

    // std::cout << "Local plan: ";
    // for (int step : local) {
    //     std::cout << step << ' ';
    // }
    // std::cout << std::endl;

    return 0;
}