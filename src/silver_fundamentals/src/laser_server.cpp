#include "ros/ros.h"
#include "silver_fundamentals/laser_server.h"

#include <cstring>

static float scan_data[726];

bool complete_laser_data(silver_fundamentals::Laser::Request  &req,
          silver_fundamentals::Laser::Response &res) {

 }

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    memcpy(scan_data, msg->data, sizeof(scan_data));
    ROS_INFO("%16f %ld", msg->ranges[msg->ranges.size()/2], msg.ranges.size());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_server");
    ros::NodeHandle n;

    ros::
    ros::ServiceServer service = n.advertiseService("complete_laser_data", complete_laser_data);
    ros::Subscriber sub = n.subscribe("scan_filtered", 1, laserCallback);

    ROS_INFO("Ready to serve");
    ros::spin();


    return 0;
}
