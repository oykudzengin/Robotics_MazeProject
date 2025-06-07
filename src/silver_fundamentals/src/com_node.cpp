#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <LocalizeCommunication.h>
#include <silver_fundamentals/Com.h>

namespace silver_fundamentals {
  PlanSuccessState success_state = PlanSuccessState::NONE;
  std::vector<geometry_msgs::Point> com_waypoints;
  bool plan_exits = false;
}

bool commCb(silver_fundamentals::Com::Request &req,
            silver_fundamentals::Com::Response &res)
{
  if (req.operation == silver_fundamentals::Com::Request::GET_DATA) {
    res.success_state = static_cast<uint8_t>(silver_fundamentals::success_state);
    res.waypoints     = silver_fundamentals::com_waypoints;
    res.plan_exists   = silver_fundamentals::plan_exits;
    res.success       = true;
  }
  else if (req.operation == silver_fundamentals::Com::Request::SET_DATA) {
    silver_fundamentals::success_state = 
       static_cast<silver_fundamentals::PlanSuccessState>(req.success_state);
    silver_fundamentals::com_waypoints = req.waypoints;
    silver_fundamentals::plan_exits    = req.plan_exists;
    res.success = true;
  }
  else {
    res.success = false;
  }
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "comm_node");
  ros::NodeHandle nh;

  ros::ServiceServer srv = nh.advertiseService("comm", commCb);
  ROS_INFO("comm_node ready");
  ros::spin();
  return 0;
}