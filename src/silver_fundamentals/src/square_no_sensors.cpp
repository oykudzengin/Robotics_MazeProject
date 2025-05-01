#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"

void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
  ROS_INFO("left encoder: %f, right encoder: %f", msg->encoderLeft, msg->encoderRight);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "square_no_sensors");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("sensor_packet", 1, sensorCallback);
  ros::ServiceClient diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
  
  create_fundamentals::DiffDrive srv;

  double forward_speed = 10.0;
  double forward_time = 3.125; // seconds, adjust as needed
  double turn_speed = 1.0;
  double turn_time = 1.571; // seconds, adjust as needed

  for (int i = 0; i < 4; ++i)
  {
    // Move forward
    ROS_INFO("Moving forward...");
    srv.request.left = forward_speed;
    srv.request.right = forward_speed;
    diffDrive.call(srv);
    ros::Duration(forward_time).sleep();

    // Stop
    srv.request.left = 0;
    srv.request.right = 0;
    diffDrive.call(srv);
    ros::Duration(0.5).sleep();

    // Turn 90 degrees
    ROS_INFO("Turning...");
    srv.request.left = -turn_speed;
    srv.request.right = turn_speed;
    diffDrive.call(srv);
    ros::Duration(turn_time).sleep();

    // Stop again
    srv.request.left = 0;
    srv.request.right = 0;
    diffDrive.call(srv);
    ros::Duration(0.5).sleep();
  }

  ROS_INFO("Finished square path.");

  ros::spin();
  return 0;
}
