#include "ros/ros.h"
#include "silver_fundamentals/Laser.h"
#include "sensor_msgs/LaserScan.h"
#include <cstdlib>

#include <cstring>
#include <bits/streambuf_iterator.h>

static struct laser_data
{
    float ranges[726];
    float angle_min;
    float angle_max;
    float angle_increment;
} laser_data;

static int count = 0;

bool complete_laser_data(silver_fundamentals::Laser::Request  &req,
          silver_fundamentals::Laser::Response &res) {
            return false;
 }

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    std::copy(msg->ranges.begin(), msg->ranges.end(), laser_data.ranges);
    laser_data.angle_min = msg->angle_min;
    laser_data.angle_max = msg->angle_max;
    laser_data.angle_increment = msg->angle_increment;
    //memcpy(scan_data, &(msg->ranges), sizeof(scan_data));
    count++;
    if (count % 10 == 0) {
        ROS_INFO("%10f  %10f %10f %10f %10f %10f %ld", msg->ranges[544], msg->ranges[msg->ranges.size()/2], msg->ranges[182],msg->angle_min, msg->angle_max, msg->angle_increment, msg->ranges.size());
        count = 0;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("complete_laser_data", complete_laser_data);
    ros::Subscriber sub = n.subscribe("scan_filtered", 1, laserCallback);

    ROS_INFO("Ready to serve");
    ros::spin();


    return 0;
}
