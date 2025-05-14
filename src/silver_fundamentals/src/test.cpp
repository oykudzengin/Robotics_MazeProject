
#include "ros/ros.h"
#include <feedback_drive.h>
#include "silver_fundamentals/LaserCartesian.h"
#include "silver_fundamentals/Laser.h"
#include <config.h>
#include <random>
#include <sstream>
#include <string>
#include <geometry_msgs/Point.h>
#include <ransac.h>

// wheel diameter 3.25
// wheelbase 26.203

int turn(int argc, char **argv) {
    double speed;
    direction dir;
    if (argc <= 4) {
        dir = right;
    } else {
        dir = atoi(argv[4]) == -1 ? right: (atoi(argv[4]) == 1 ? left : none);
    }
    if (argc <= 3) {
        speed = 4.0;
    } else {
        speed = atof(argv[3]);
    }
    double angle = atof(argv[1]);
    double radius = atof(argv[2]);
    ros::init(argc, argv, "test");
    // larger wheelbase -> higher turning angle
    // larger wheelradius -> going further
    auto driver = FeedbackDrive(3.25, 23.5, 10.0);
    driver.reset_encoders();
    driver.turn(angle, dir, radius, speed);
    return 0;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "test");
    auto driver = FeedbackDrive(3.25, 23.5, 10.0);
    driver.drive_along_wall(40, 40, 10, nullptr);
    return 0;
    ros::NodeHandle n;

    ros::ServiceClient laser_cart_client = n.serviceClient<silver_fundamentals::LaserCartesian>("laserAngleRangeCartesian");
    silver_fundamentals::LaserCartesian laser_cart_srv;

    laser_cart_srv.request.start = -120.0;
    laser_cart_srv.request.end = 120.0;
    laser_cart_srv.request.max_dist = 100.0;

    while (ros::ok()) {
        int amount;
        double dist;

        while (!laser_cart_client.call(laser_cart_srv)) {
            ROS_ERROR("Failed to call laser cartesian service");
        }
        std::vector<geometry_msgs::Point> pts = laser_cart_srv.response.values;

        double angle = ransac_with_dist(pts, 0.08, 7000, &amount, &dist);
        ROS_INFO("Angle: %4f, points used %3d, distance %5f", angle, amount, dist);
    }

    return 0;
}