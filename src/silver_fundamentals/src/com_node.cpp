#include <ros/ros.h>
#include <LocalizeCommunication.h>
#include <silver_fundamentals/Com.h>
#include <vector>
#include <cstdint>

namespace silver_fundamentals {
  PlanSuccessState success_state = PlanSuccessState::NONE;
  std::vector<uint8_t> goal;
  bool goal_exists = false;
}

bool commCb(silver_fundamentals::Com::Request &req,
            silver_fundamentals::Com::Response &res)
{
  if (req.operation == silver_fundamentals::Com::Request::GET_DATA) {
    res.success_state = static_cast<uint8_t>(silver_fundamentals::success_state);
    res.goal     = silver_fundamentals::goal;
    res.goal_exists   = silver_fundamentals::goal_exists;
    res.success       = true;
  }
  else if (req.operation == silver_fundamentals::Com::Request::SET_DATA) {
    silver_fundamentals::success_state = 
       static_cast<silver_fundamentals::PlanSuccessState>(req.success_state);
    silver_fundamentals::goal = req.goal;
    silver_fundamentals::goal_exists = req.goal_exists;
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