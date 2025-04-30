#include "ros/ros.h"
#include <cstdlib>
#include "sensor_msgs/LaserScan.h"
#include "create_fundamentals/DiffDrive.h"
#include "silver_fundamentals/Laser.h"
#include <cmath>
#include <sstream>
#include <vector>
#include <config.h>
#include <laser_distance_map.h>


// Store the closest obstacle distance
// float min_distance = std::numeric_limits<float>::infinity();

enum direction {
  none,
  left,
  right,
};

direction compute_turning_direction(std::vector<double> *ranges) {
  int ranges_size = ranges->size();
  int threshold_table_size = threshold_table.size();
  int starting_offset = (threshold_table_size-ranges_size)/2;
  for (int i = 0; i < ranges_size; i++) {
    if (ranges->at(i) < threshold_table.at(i+starting_offset)) {
      return left;
    } else if (ranges->at(ranges_size-1-i) < threshold_table.at(threshold_table_size-starting_offset-1-i)) {
      return right;
    }
  }
  return none;
}

void wander() {
  ros::NodeHandle n;

  ros::ServiceClient laser_client = n.serviceClient<silver_fundamentals::Laser>("laserAngleRange");
  silver_fundamentals::Laser laser_srv;

  ros::ServiceClient drive_client = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
  create_fundamentals::DiffDrive drive_srv;

  ros::Rate rate(2);
  double speed = 10.0;


  while (ros::ok()) {
  	//drive_srv.request.left = speed;
    //drive_srv.request.right = speed;
          
    //drive_client.call(drive_srv);
    laser_srv.request.start = -90;
    laser_srv.request.end = 90;

   	if (laser_client.call(laser_srv)) {
      switch(laser_srv.response.values) {
        case none: ROS_INFO("Do not turn"); break;
        case left: ROS_INFO("Turn   left"); break;
        case right: ROS_INFO("Turn  right"); break;
      }
   	}
   	rate.sleep();
	}


}

int main(int argc, char** argv) {
  ros::init(argc, argv, "wanderer");


  wander();

  ros::NodeHandle nh;

  // Subscribe to the laser scan topic
  //ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan_filtered", 100, laserCallback);

  ros::ServiceClient client = nh.serviceClient<silver_fundamentals::Laser>("laserAngleRange");
  silver_fundamentals::Laser srv;
  std::stringstream ss;
  for (const double range : threshold_table)
  {
    ss << range << " ";
  }
  ROS_INFO("Threshold values, %ld values: %s ", threshold_table.size(), ss.str().c_str());

  ros::Rate rate(1);
  srv.request.start = 90;
  srv.request.end = 120;
  while (ros::ok()) {
    if (client.call(srv)) {
      std::stringstream ss;
      for (const double range : srv.response.values)
      {
        ss << range << " ";
      }
      ROS_INFO("Wanderer success, %d values: %s ", srv.response.size, ss.str().c_str());
    }
    rate.sleep();
  }


  // Service client for diff_drive
  ros::ServiceClient drive_client =
    nh.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
  create_fundamentals::DiffDrive srv_drive;

  float speed = 10.0;
  float n = (float) atof(argv[2]);
  float time = n / ( (float)(atof(argv[1])) * speed);
  ROS_INFO("Time is %f s", time);
  srv_drive.request.left = -speed;
  srv_drive.request.right = speed;

  drive_client.call(srv_drive);
  ros::Duration(time).sleep();

  srv_drive.request.left = 0;
  srv_drive.request.right = 0;
  drive_client.call(srv_drive);


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