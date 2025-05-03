#include <array>
#include <bits/streambuf_iterator.h>
#include <cmath>
#include <config.h>
#include <sstream>
#include <vector>
#include <geometry_msgs/Point.h>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "silver_fundamentals/Laser.h"

#define LASER_ARAY_SIZE 682.0
#define LASER_ANGLE_RANGE 240.0

static struct laser_data
{
    std::array<double, static_cast<int>(LASER_ARAY_SIZE)> ranges{};
    double angle_range = 4.178563637658954 * 180 / PI;
    double angle_min = -2.086213869974017 * 180 / PI;
    double angle_increment = 0.006135923322290182 * 180 / PI;
    double range_min = 0.019999999552965164;
} laser_data;

static int count = 0;

int minimum_discarding(std::vector<double>* ranges)
{
    int count = 0;
    for (double & range : *ranges)
    {
        if (range < laser_data.range_min)
        {
            range = std::numeric_limits<double>::signaling_NaN();
            count++;
        }
    }
    return count;
}


bool laser_angle_range(silver_fundamentals::Laser::Request& req, silver_fundamentals::Laser::Response& res)
{
    double min_angle = req.start;
    double max_angle = req.end;
    int min_index = static_cast<int>(
        std::round(std::max(min_angle + LASER_ANGLE_RANGE / 2.0, 0.0) * LASER_ARAY_SIZE / LASER_ANGLE_RANGE));
    int max_index = static_cast<int>(
        std::round(std::min(max_angle + LASER_ANGLE_RANGE / 2.0, 240.0) * LASER_ARAY_SIZE / LASER_ANGLE_RANGE));

    res.values =
        std::vector<double>(laser_data.ranges.begin() + min_index,
                            laser_data.ranges.begin() + std::min(max_index + 1, static_cast<int>(LASER_ARAY_SIZE)));
    minimum_discarding(&res.values);
    res.size = max_index - min_index + 1;
    return true;
}

bool laser_angle_range_cartesian(silver_fundamentals::LaserCartesian::Request& req, silver_fundamentals::LaserCartesian::Response& res) {
    double min_angle = req.start;
    double max_angle = req.end;
    int min_index = static_cast<int>(
        std::round(std::max(min_angle + LASER_ANGLE_RANGE / 2.0, 0.0) * LASER_ARAY_SIZE / LASER_ANGLE_RANGE));
    int max_index = static_cast<int>(
        std::round(std::min(max_angle + LASER_ANGLE_RANGE / 2.0, 240.0) * LASER_ARAY_SIZE / LASER_ANGLE_RANGE));

    /*
    res.values =
        std::vector<double>(laser_data.ranges.begin() + min_index,
                            laser_data.ranges.begin() + std::min(max_index + 1, static_cast<int>(LASER_ARAY_SIZE)));
    */
    std::vector<geometry_msgs::Point> cart_points;
    cart_points.reserve(std::min(max_index + 1, static_cast<int>(laser_data.ranges.size())) - min_index);

    for (int i = min_index; i <= max_index && i < static_cast<int>(laser_data.ranges.size()); i++) {
        double dist = laser_data.ranges[i];
        if (std::isnan(dist) || dist < laser_data.range_min) {
            continue;
        }
        double angle = laser_data.angle_min + i*laser_data.angle_increment;
        double angle_rad = angle*PI/180.0f;

        geometry_msgs::Point point;
        point.x = std::cos(angle_rad) * dist;
        point.y = std::sin(angle_rad) * dist;
        point.z = angle;

        cart_points.push_back(point);
    }

    res.values = cart_points;
    res.size = static_cast<int>(cart_points.size());
    return true;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    std::copy(msg->ranges.begin() + 44, msg->ranges.end(), laser_data.ranges.begin());

    count++;
    if (count % 100 == 0)
    {
        std::stringstream ss;
        for (const double range : laser_data.ranges)
        {
            ss << range << " ";
        }
        ROS_INFO("%s", ss.str().c_str());
        count = 0;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("laserAngleRange", laser_angle_range);
    ros::ServiceServer service2 = n.advertiseService("laserAngleRangeCart", laser_angle_range_cartesian);
    ros::Subscriber sub = n.subscribe("scan_filtered", 1, laserCallback);

    ROS_INFO("Ready to serve");
    ros::spin();


    return 0;
}
