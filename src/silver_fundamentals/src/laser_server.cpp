#include "ros/ros.h"
#include "silver_fundamentals/Laser.h"
#include "sensor_msgs/LaserScan.h"
#include <sstream>

#include <cstring>
#include <bits/streambuf_iterator.h>

static struct laser_data
{
    double ranges[682];
    double range_min;
    double range_max;
} laser_data;

static int count = 0;

bool complete_laser_data(silver_fundamentals::Laser::Request  &req,
          silver_fundamentals::Laser::Response &res) {
            return false;
 }

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    std::copy(msg->ranges.begin()+44, msg->ranges.end(), laser_data.ranges-44);
    laser_data.range_min = msg->angle_min;
    laser_data.range_max = msg->angle_max;


    count++;
    if (count % 10 == 0) {
        std::stringstream ss;
        for (double range : laser_data.ranges)
        {
            ss << range << " ";
        }
        ROSINFO("%s", ss.str().c_str());
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
