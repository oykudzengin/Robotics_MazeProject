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

    size_t i = 0;
    while (i < moves.size()) {
        int dir = moves[i];
        size_t j = i + 1;
        // count consecutive runs of the same direction
        while (j < moves.size() && moves[j] == dir) {
            ++j;
        }
        size_t runLength = j - i;
        // apply the total displacement for the entire run
        switch (dir) {
            case 1: y += step_length * runLength; break; // up
            case 2: x += step_length * runLength; break; // left
            case 3: y -= step_length * runLength; break; // down
            case 0: x -= step_length * runLength; break; //right
            default:
                ROS_WARN("Unknown direction code in run: %d", dir);
                i = j;
                continue;
        }
        geometry_msgs::Point pt;
        pt.x = x;
        pt.y = y;
        // use 0.15 for the final waypoint, 0.4 otherwise
        pt.z = (j == moves.size()) ? 0.15 : 0.4;
        waypoints.push_back(pt);
        i = j;
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
    auto driver = FeedbackDrive(3.25, 26.5, 2.0);
    driver.reset_encoders();

    std::vector<int> plan = req.plan; //compressPlan will work here
    std::vector<geometry_msgs::Point> waypoints = convertMovesToWaypoints(plan);
    ROS_ERROR("Waypoints: [");
    for (size_t i = 0; i < waypoints.size(); ++i) {
        ROS_ERROR("%f %f %f", waypoints[i].x, waypoints[i].y, waypoints[i].z);
    }
    ROS_ERROR("]");
    int ret = driver.potential_field_drive(waypoints, 10.0, 0.01, 0.2, 0.07);
    if (ret != 0)
        ROS_ERROR("Potential field drive failed");
    return ret == 0?true:false;
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