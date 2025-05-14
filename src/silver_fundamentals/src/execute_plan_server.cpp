#include <ros/ros.h>
#include <silver_fundamentals/ExecutePlan.h>
#include "silver_fundamentals/Laser.h"
#include <feedback_drive.h>

bool executePlan(silver_fundamentals::ExecutePlan::Request &req,
                  silver_fundamentals::ExecutePlan::Response &res) 
{

    ROS_INFO("Received a plan of %d steps", req.plan.size());
    auto driver = FeedbackDrive(3.25, 26.203, 2.0);

    std::vector<int> plan = req.plan;
    ros::Rate loop_rate(10);

    for (int i=0; i < plan.size(); i++) {
        switch (plan[i]) {
            case 0: { //right
                ROS_INFO("Turning right");
                driver.turn_n_degrees(90, right);
                driver.drive_n_cm(40);
                break;
            }
            case 1: { //up
                ROS_INFO("Moving up");
                driver.drive_n_cm(40);
                break;
            }
            case 2: { //left
                ROS_INFO("Turning left");
                driver.turn_n_degrees(90,left);
                driver.drive_n_cm(40);
                break;
            }
            case 3: { //down
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