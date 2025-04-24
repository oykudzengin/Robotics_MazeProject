#include "ros/ros.h"
#include <cstdlib>
#include "sensor_msgs/LaserScan.h"
#include "create_fundamentals/DiffDrive.h"
#include <cmath>
#include <config.h>




// Store the closest obstacle distance
float min_distance = std::numeric_limits<float>::infinity();

// Callback to update min_distance from laser scan
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  // Use the laser scan's center ray for straight-ahead distance
  size_t center_index = msg->ranges.size() / 2;
  float center_distance = msg->ranges[center_index];
  min_distance = center_distance;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "wanderer");
  ros::NodeHandle nh;

  // Subscribe to the laser scan topic
  ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>(
    "/scan_filtered", 100, laserCallback);

  // Service client for diff_drive
  ros::ServiceClient drive_client =
    nh.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
  create_fundamentals::DiffDrive srv;

  float speed = 10.0;
  float n = 80.0;
  float time = n / (WHEEL_RADIUS * speed);
  srv.request.left = speed;
  srv.request.right = speed;

  drive_client.call(srv);
  ros::Duration(time).sleep();

  // ros::Rate rate(10);  // 10 Hz loop

  // while (ros::ok()) {
  //   ros::spinOnce();

  //   srv.request.left = 2;
  //   srv.request.right = 2;
    
  //   drive_client.call(srv);
  //   ros::Duration(11,42).sleep();

  //   srv.request.left = 0;
  //   srv.request.right = 0;
    
  //   drive_client.call(srv);

  //   return 0;



  //   // // Basic obstacle avoidance: if too close, turn; else drive forward
  //   // ROS_INFO("%f", min_distance);
  //   // if (min_distance < 0.25 || min_distance == std::numeric_limits<float>::infinity()) {  // threshold in meters
  //   // //   ROS_INFO("%s", "turning ");
  //   // //   srv.request.left  =  5.0;  // turn in place
  //   // //   srv.request.right = -5.0;
  //   // //   ros::Duration(0.5).sleep();
  //   //   srv.request.left  =  0;  // turn in place
  //   //   srv.request.right = 0;

  //   // } else {
  //   //     ROS_INFO("%s", "driving ");
  //   //   srv.request.left  = 10.0;   // move straight
  //   //   srv.request.right = 10.0;
  //   // }

  //   // if (!drive_client.call(srv)) {
  //   //   ROS_ERROR("Failed to call diff_drive service");
  //   // }

  //   rate.sleep();
  // }

  return 0;
}