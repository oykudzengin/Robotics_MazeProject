#include "ros/ros.h"
#include "silver_fundamentals/Laser.h"
#include "sensor_msgs/LaserScan.h"
#include <sstream>
#include <array>
#include <vector>
#include <cmath>
#include <bits/streambuf_iterator.h>
#include <config.h>

#define LASER_ARAY_SIZE 682.0
#define LASER_ANGLE_RANGE 240.0

static struct laser_data
{
    std::array<double, static_cast<int>(LASER_ARAY_SIZE)> ranges;
    double angle_range = 4.178563637658954*180/PI;
    double angle_min = -2.086213869974017*180/PI;
    double angle_increment = 0.006135923322290182*180/PI;
} laser_data;

static int count = 0;

bool laser_angle_range(silver_fundamentals::Laser::Request  &req, silver_fundamentals::Laser::Response &res) {
    double min_angle = req.start;
    double max_angle = req.end;
    int min_index = static_cast<int>(std::round(
        std::max(min_angle + LASER_ANGLE_RANGE / 2.0, 0.0) * LASER_ARAY_SIZE / LASER_ANGLE_RANGE));
    int max_index = static_cast<int>(std::round(
        std::min(max_angle + LASER_ANGLE_RANGE / 2.0, 240.0) * LASER_ARAY_SIZE / LASER_ANGLE_RANGE));
    ROS_INFO("Creating array between %d and %d", min_index, max_index);

    res.values = std::vector<double>(laser_data.ranges.begin() + min_index, laser_data.ranges.begin() + std::min(max_index+1, static_cast<int>(LASER_ARAY_SIZE)));
    res.size = max_index - min_index + 1;
    return true;
 }

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    std::copy(msg->ranges.begin()+44, msg->ranges.end(), laser_data.ranges.begin());

    count++;
    if (count % 100 == 0) {
        std::stringstream ss;
        for (const double range : laser_data.ranges)
        {
            ss << range << " ";
        }
        ROS_INFO("%s", ss.str().c_str());
        count = 0;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("laserAngleRange", laser_angle_range);
    ros::Subscriber sub = n.subscribe("scan_filtered", 1, laserCallback);

    ROS_INFO("Ready to serve");
    ros::spin();


    return 0;
}
