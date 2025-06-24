#include <ros/ros.h>
#include <silver_fundamentals/Com.h>
#include "silver_fundamentals/MoveToPosition.h"
#include <vector>
#include <cstdint>
#include <LocalizeCommunication.h>

#include "playsong.h"

// localaise communication to execute plan server 
namespace silver_fundamentals {
    PlanSuccessState success_state = PlanSuccessState::NONE;
    std::vector<geometry_msgs::Point> com_waypoints;
    bool plan_exits = false;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "full_run_server");
    ros::NodeHandle nh;

    // TODO: read in files and create vectors
    std::vector<std::vector<uint8_t>> goals;


    const ros::ServiceClient comm_client = nh.serviceClient<silver_fundamentals::Com>("comm");
    silver_fundamentals::Com comm_srv;

    for (const auto &goal: goals) {

        bool success = false;
        // do this as long as we (might) fail
        while (!success) {
            // set request
            comm_srv.request.operation = silver_fundamentals::Com::Request::SET_DATA;
            comm_srv.request.success_state = static_cast<uint8_t>(silver_fundamentals::PlanSuccessState::NONE);
            comm_srv.request.goal_exists   = true;
            comm_srv.request.goal = goal;

            //send request
            while (!comm_client.call(comm_srv))
                ROS_ERROR("execute_plan_server: failed to call comm service for SET_DATA");

            // wait for execution
            comm_srv.request.operation = silver_fundamentals::Com::Request::GET_DATA;
            while (ros::ok() && (!comm_client.call(comm_srv) || comm_srv.response.success_state == silver_fundamentals::PlanSuccessState::NONE)) {
                ROS_INFO("waiting for client to execute a plan...");
                ros::Duration(0.1).sleep();
            }

            // check success and maybe redo
            success = (comm_srv.response.success_state == silver_fundamentals::PlanSuccessState::PLAN_DONE);
        }

        // play song
        silver_fundamentals::playSong3(nh);
        ros::Duration(5).sleep();

    }


    return 0;
}
