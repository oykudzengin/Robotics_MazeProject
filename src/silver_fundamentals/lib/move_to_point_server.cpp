#include <ros/ros.h>
#include <silver_fundamentals/Com.h>
#include "silver_fundamentals/MoveToPosition.h"
#include <vector>
#include <cstdint>
#include <LocalizeCommunication.h>

// localaise communication to execute plan server 
namespace silver_fundamentals {
    PlanSuccessState success_state = PlanSuccessState::NONE;
    std::vector<geometry_msgs::Point> com_waypoints;
    bool plan_exits = false;
}


// Service callback: handles incoming /move_to_position requests
bool moveToPositionCallback(
    silver_fundamentals::MoveToPosition::Request &req,
    silver_fundamentals::MoveToPosition::Response &res)
{
    ROS_INFO("move_to_position called: row=%d, column=%d", req.row, req.column);
    // TODO: insert your position handling logic here.
    res.success = true;
    ros::NodeHandle nh_comm;
    // Use a static client to avoid recreating each call
    static ros::ServiceClient comm_client =
    nh_comm.serviceClient<silver_fundamentals::Com>("comm");

    // Build the request
    silver_fundamentals::Com get_srv;
    get_srv.request.operation     = silver_fundamentals::Com::Request::SET_DATA;
    get_srv.request.success_state = static_cast<uint8_t>(silver_fundamentals::PlanSuccessState::NONE);
    get_srv.request.goal_exists   = true;
    get_srv.request.goal = std::vector<uint8_t>{ 
        static_cast<uint8_t>(req.row), 
        static_cast<uint8_t>(req.column) 
    };

    // Call the service
    if (!comm_client.call(get_srv)) {
    ROS_ERROR("execute_plan_server: failed to call comm service for SET_DATA");
    }
    //return true;

    silver_fundamentals::PlanSuccessState status; // <— and the status variable
    // Poll until comm_node reports PLAN_DONE or PLAN_FAILED
    do {
    if (!comm_client.call(get_srv)) {
        ROS_ERROR("execute_plan_server: failed to call comm service for GET_DATA");
        return false;
    }
    status = static_cast<silver_fundamentals::PlanSuccessState>(
        get_srv.response.success_state);
    ros::Duration(0.1).sleep();  // avoid tight loop
    ROS_INFO("Waiting for answer from localizsaiton ");
    } while (ros::ok() && status == silver_fundamentals::PlanSuccessState::NONE);

    // Interpret result
    if (status == silver_fundamentals::PlanSuccessState::PLAN_DONE) {
        // goal reached
        res.success = true;
        return true;
    } else if (status == silver_fundamentals::PlanSuccessState::PLAN_FAILED) {
        // goal not reached
        res.success = false;
        return true;
    } else {
        return false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_to_point_server");
    ros::NodeHandle nh;

    // Advertise the service under the name '/move_to_position31'
    ros::ServiceServer srv = nh.advertiseService(
        "move_to_position",
        moveToPositionCallback
    );
    ROS_INFO("Service '/move_to_position' ready.");

    ros::spin();
    return 0;
}
