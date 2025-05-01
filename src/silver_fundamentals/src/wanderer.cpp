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



direction compute_turning_direction(std::vector<double> *ranges) {
  int ranges_size = ranges->size();
  int threshold_table_size = threshold_table.size();
  int starting_offset = (threshold_table_size-ranges_size)/2;

  //ROS_INFO("ranges size is %d, threshold table size is %d, offset is %d", ranges_size, threshold_table_size, starting_offset);

  for (int i = 0; i < ranges_size; i++) {
    if (ranges->at(i) < threshold_table.at(i+starting_offset)) {
      ROS_INFO("turning left, index is %d", i);
      return left;
    } else if (ranges->at(ranges_size-1-i) < threshold_table.at(threshold_table_size-starting_offset-1-i)) {
      ROS_INFO("turning right, index is %d", i);
      return right;
    }
  }
  return none;
}

bool threshold_is_clear(std::vector<double> *ranges) {
  return compute_turning_direction(ranges) == none;
}

void wander() {
  ros::NodeHandle n;

  ros::ServiceClient laser_client = n.serviceClient<silver_fundamentals::Laser>("laserAngleRange");
  silver_fundamentals::Laser laser_srv;

  ros::ServiceClient drive_client = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
  create_fundamentals::DiffDrive drive_srv;

  ros::Rate rate(10);
  ros::Rate turning_update_rate(100);
  double speed = 10.0;


  while (ros::ok()) {

    laser_srv.request.start = -120;
    laser_srv.request.end = 120;

    if (laser_client.call(laser_srv)) {
      switch(compute_turning_direction(&laser_srv.response.values)) {
        case none: {
          drive_srv.request.left = speed;
          drive_srv.request.right = speed;
          break;
        }
        case left: {
          drive_srv.request.left = -4;
          drive_srv.request.right = 4;
          break;
         }
        case right: {
          drive_srv.request.left = 4;
          drive_srv.request.right = -4;
          break;
        }
      }
    }
    drive_client.call(drive_srv);
    while (laser_client.call(laser_srv) && !threshold_is_clear(&laser_srv.response.values)) {
      //turning_update_rate.sleep();
    }

    drive_srv.request.left = speed;
    drive_srv.request.right = speed;

    drive_client.call(drive_srv);

    rate.sleep();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "wanderer");

  wander();

  return 0;
}