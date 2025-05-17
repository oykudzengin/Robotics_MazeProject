#include <ros/ros.h>
#include <silver_fundamentals/ExecutePlan.h>
#include "silver_fundamentals/Laser.h"
#include <feedback_drive.h>
#include <config.h>
#include <vector>
#include <cstddef>   // for std::size_t
// Example usage
#include <iostream>
#include <geometry_msgs/Point.h>


std::vector<geometry_msgs::Point>
convertMovesToWaypoints(const std::vector<int>& moves)
{
    const double step_length = 0.8;  // meters per cell (80 cm)
    std::vector<geometry_msgs::Point> waypoints;
    waypoints.reserve(moves.size());

    // Starting point
    double x = 0.0;
    double y = 0.0;

    for (int dir : moves)
    {
        switch (dir)
        {
            case 1: // up
                y += step_length;
                break;
            case 0: // right
                x += step_length;
                break;
            case 3: // down
                y -= step_length;
                break;
            case 2: // left
                x -= step_length;
                break;
            default:
                ROS_WARN("Unknown direction code: %d", dir);
                continue;  // skip invalid codes
        }

        geometry_msgs::Point pt;
        pt.x = (double) x;
        pt.y = (double) y;
        pt.z = 0.0;  // assume planar (z=0)
        waypoints.push_back(pt);
    }

    return waypoints;
}

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

    ROS_INFO("Received a plan of %ld steps", req.plan.size());
    auto driver = FeedbackDrive(3.25, 26.203, 2.0);
    driver.reset_encoders();

    std::vector<int> plan = req.plan; //compressPlan will work here
    auto waypoints = convertMovesToWaypoints(plan);
    ROS_INFO("Waypoints: [");
    for (size_t i = 0; i < waypoints.size(); ++i) {
        ROS_INFO("%ld", waypoints[i]);
    }
    ROS_INFO("]");
    driver.potential_field_drive(waypoints, 10.0, 0.01, 0.3, 0.08);
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