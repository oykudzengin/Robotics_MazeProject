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
    double last_dir = 10.0;


    for (int i = 0; i < moves.size(); i++) {
        int dir = moves[i];
        switch (dir) {
            case 1: y += step_length; break; // up
            case 2: x += step_length; break; // left
            case 3: y -= step_length; break; // down
            case 0: x -= step_length; break; //right
            default:
                ROS_WARN("Unknown direction code in run: %d", dir);
                continue;
        }
#ifdef DOTASK2
        if (last_dir == dir) {
            waypoints[waypoints.size() - 1].x = x;
            waypoints[waypoints.size() - 1].y = y;
        } else if (std::abs(last_dir - dir) == 2) {
            waypoints[waypoints.size() - 1].z = 0.15;
            geometry_msgs::Point pt;
            pt.x = x;
            pt.y = y;
            pt.z = 0.4;
            waypoints.push_back(pt);
        } else {
            geometry_msgs::Point pt;
            pt.x = x;
            pt.y = y;
            pt.z = 0.4;
            waypoints.push_back(pt);
        }
#else
        geometry_msgs::Point pt;
        pt.x = x;
        pt.y = y;
        pt.z = 0.4;
        waypoints.push_back(pt);
#endif
    last_dir = dir;

    }
    waypoints[waypoints.size() - 1].z = 0.15;
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
    auto driver = FeedbackDrive(3.25, 26.5, 2.0);
    driver.reset_encoders();

    std::vector<int> plan = req.plan; //compressPlan will work here
    std::vector<geometry_msgs::Point> waypoints = convertMovesToWaypoints(plan);
    ROS_ERROR("Waypoints: [");
    for (size_t i = 0; i < waypoints.size(); ++i) {
        ROS_ERROR("%f %f %f", waypoints[i].x, waypoints[i].y, waypoints[i].z);
    }
    ROS_ERROR("]");
    int ret = driver.potential_field_drive(waypoints, 1000.0, 0.01, 0.2, 0.08);
    if (ret != 0)
        ROS_ERROR("Potential field drive failed");
    res.success = ret == 0?true:false;
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