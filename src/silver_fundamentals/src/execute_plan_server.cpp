#include <ros/ros.h>
#include <silver_fundamentals/ExecutePlan.h>
#include "silver_fundamentals/Laser.h"
#include <feedback_drive.h>
#include <config.h>
#include <vector>
#include <cstddef>   // for std::size_t
// Example usage
#include <iostream>


std::vector<int> compressPlan(const std::vector<int32_t>& globalPlan) {
    // 1. Build an extPlan with a leading '1'
    std::vector<int> extPlan;
    extPlan.reserve(globalPlan.size() + 1);
    extPlan.push_back(1);  // initial heading for conversion
    for (auto dir : globalPlan) {
        extPlan.push_back(static_cast<int>(dir));
    }

    // 2. Compute local instructions
    std::vector<int> localPlan;
    localPlan.reserve(extPlan.size() - 1);
    for (size_t i = 0; i + 1 < extPlan.size(); ++i) {
        int diff = (extPlan[i] - extPlan[i+1] + 4) % 4;
        localPlan.push_back(diff);
    }

    return localPlan;
}


bool executePlan(silver_fundamentals::ExecutePlan::Request &req,
                  silver_fundamentals::ExecutePlan::Response &res) 
{

    ROS_INFO("Received a plan of %d steps", req.plan.size());
    auto driver = FeedbackDrive(3.25, 26.203, 2.0);

    std::vector<int> plan = req.plan; //compressPlan will work here
    ros::Rate loop_rate(10);

    for (int i=0; i < plan.size(); i++) {
        locDirection cur_dir = static_cast<locDirection>(plan[i]); // Convert to locDirection enum
        switch (cur_dir) {
            case l_RIGHT: { //right
                ROS_INFO("Turning right");
                driver.turn_n_degrees(90, right);
                driver.drive_n_cm(40);
                break;
            }
            case l_UP: { //up
                ROS_INFO("Moving up");
                driver.drive_n_cm(40);
                break;
            }
            case l_LEFT: { //left
                ROS_INFO("Turning left");
                driver.turn_n_degrees(90,left);
                driver.drive_n_cm(40);
                break;
            }
            case l_DOWN: { //down
                ROS_INFO("Moving down");
                driver.turn_n_degrees(180, right);
                driver.drive_n_cm(40);
                break;
            }
            default:
                ROS_ERROR("Unknown action %d", plan[i]);
        }
        loop_rate.sleep();
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

    return 0;
}