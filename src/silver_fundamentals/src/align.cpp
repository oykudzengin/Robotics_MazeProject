#include "ros/ros.h"
#include "silver_fundamentals/LaserCartesian.h"
#include "silver_fundamentals/Laser.h"
#include <config.h>
#include <random>
#include <sstream>
#include <string>
#include <geometry_msgs/Point.h>
#include <feedback_drive.h>
#include <ransac.h>

double minIgnoringNaN(const std::vector<double> &v) {
    double best = std::numeric_limits<double>::infinity();
    bool gotOne = false;

    for (double x: v) {
        if (std::isnan(x)) continue;
        gotOne = true;
        best = std::min(best, x);
    }
    return gotOne
               ? best
               : std::numeric_limits<double>::quiet_NaN();
}

int align() {
    int ransac_iterations = 7000;
    double ransac_distance = 0.8;
    int ransac_threshold_points = 120;
    double ransac_min_angle = -80.0;
    double ransac_max_angle = 80.0;
    ros::NodeHandle n;

    ros::ServiceClient laser_cart_client = n.serviceClient<silver_fundamentals::LaserCartesian>("laserAngleRangeCartesian");
    silver_fundamentals::LaserCartesian laser_cart_srv;

    ros::ServiceClient laser_pol_client = n.serviceClient<silver_fundamentals::Laser>("laserAngleRange");
    silver_fundamentals::Laser laser_pol_srv;

    auto driver = FeedbackDrive(3.25, 26.203, 2.0);

    ros::Rate rate(1);

    laser_cart_srv.request.start = -60.0;
    laser_cart_srv.request.end = 60.0;
    laser_cart_srv.request.max_dist = 90.0;

    int values_used = 0;

    double angle_to_closest_wall;


    int tries = 0;
    while (true) {
        rate.sleep();
        rate.sleep();
        // get laser data
        while (!laser_cart_client.call(laser_cart_srv)) {
            ROS_ERROR("Failed to call laser cartesian service");
        }
        std::vector<geometry_msgs::Point> pts = laser_cart_srv.response.values;

        // get ransac angle
        angle_to_closest_wall = ransac(pts, ransac_distance, ransac_iterations, &values_used);

        // TODO: better values then 150?
        if (values_used > 130)
            break;

        driver.turn_n_degrees(30, right);
        tries++;
        if (tries % 12 == 0) {
            driver.distance_to_wall(35);
        }


    }
    ROS_INFO("Angle: %f first wall", angle_to_closest_wall);

    // turn to wall
    driver.turn_n_degrees(std::abs(angle_to_closest_wall), angle_to_closest_wall > 0 ? left : right);

    // drive to wall
    /* TODO: drive to wall 40cm */
    driver.distance_to_wall(26);
    ROS_INFO("aligned to first wall");

    // check for wall right and left
    laser_pol_srv.request.start = -120;
    laser_pol_srv.request.end = -90;
    while (!laser_pol_client.call(laser_pol_srv)) {
        ROS_ERROR("Failed to call laser polar service");
    }
    bool something_to_the_right = !std::isnan(minIgnoringNaN(laser_pol_srv.response.values));

    laser_pol_srv.request.start = 90;
    laser_pol_srv.request.end = 120;
    while (!laser_pol_client.call(laser_pol_srv)) {
        ROS_ERROR("Failed to call laser polar service");
    }
    bool something_to_the_left = !std::isnan(minIgnoringNaN(laser_pol_srv.response.values));

    ROS_INFO("Left is %d, Right is %d", something_to_the_left, something_to_the_right);


    direction align_direction = none;
    double align_angle = std::numeric_limits<double>::infinity();

    if (something_to_the_right) {

        // turn to wall
        driver.turn_n_degrees(90.0, right);
        rate.sleep();
        rate.sleep();

        // setup laser_call
        laser_cart_srv.request.start = ransac_min_angle;
        laser_cart_srv.request.end = ransac_max_angle;
        laser_cart_srv.request.max_dist = 90.0;

        values_used = 0;

        // get laser data
        while (!laser_cart_client.call(laser_cart_srv)) {
            ROS_ERROR("Failed to call laser cartesian service");
        }
        std::vector<geometry_msgs::Point> pts = laser_cart_srv.response.values;

        // to ransac until wall in front is found
        ROS_INFO("RANSAC right wall");
        do {
            angle_to_closest_wall = ransac(pts, ransac_distance, ransac_iterations, &values_used);
            ROS_INFO("Found angle of %f with %d points.", angle_to_closest_wall, values_used);
        } while (std::abs(angle_to_closest_wall) > 25.0 && values_used > ransac_threshold_points);

        if (values_used <= ransac_threshold_points) {
            // turning right might be fine but ransac found nothing
            align_direction = right;
        } else {
            // ransac found stuff;
            align_angle = std::abs(angle_to_closest_wall);
            align_direction = angle_to_closest_wall<0?right:left;
        }
    }
    if (align_angle == std::numeric_limits<double>::infinity() && something_to_the_left) {

        // turn depending on if turned right before
        if (something_to_the_right)
            driver.turn_n_degrees(180.0, right);
        else
            driver.turn_n_degrees(90.0, left);

        rate.sleep();
        rate.sleep();

        // setup laser_call
        laser_cart_srv.request.start = ransac_min_angle;
        laser_cart_srv.request.end = ransac_max_angle;
        laser_cart_srv.request.max_dist = 90.0;

        values_used = 0;

        // get laser data
        while (!laser_cart_client.call(laser_cart_srv)) {
            ROS_ERROR("Failed to call laser cartesian service");
        }
        std::vector<geometry_msgs::Point> pts = laser_cart_srv.response.values;

        // to ransac until wall in front is found
        ROS_INFO("RANSAC left wall");
        do {
            angle_to_closest_wall = ransac(pts, ransac_distance, ransac_iterations, &values_used);
            ROS_INFO("Found angle of %f with %d points.", angle_to_closest_wall, values_used);
        } while (std::abs(angle_to_closest_wall) > 25.0 && values_used > ransac_threshold_points);

        if (values_used <= ransac_threshold_points) {
            align_direction = left;
        } else {
            align_angle = std::abs(angle_to_closest_wall);
            align_direction = angle_to_closest_wall<0?right:left;
        }
    }
    if (align_angle == std::numeric_limits<double>::infinity()) {
        if (!something_to_the_left && !something_to_the_right) {
            align_angle = 90.0;
        } else {
            align_angle = 0.0;
        }

    }
    if (align_direction == none) {
        align_direction = left;
    }

    ROS_INFO("Turning %f now", align_angle);

    driver.turn_n_degrees(align_angle, align_direction);

    /* TODO: drive to 40cm next to wall*/
    driver.distance_to_wall(26);
    ROS_INFO("DONE??????");
    // DONE :)

    return 0;

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "align");

    align();
    return 0;

    ros::NodeHandle n;

    ros::ServiceClient laser_cart_client = n.serviceClient<silver_fundamentals::LaserCartesian>(
        "laserAngleRangeCartesian");
    silver_fundamentals::LaserCartesian laser_cart_srv;

    ros::Rate rate(10);


    auto driver = FeedbackDrive(3.25, 26.203, 2.0);

    while (ros::ok()) {
        laser_cart_srv.request.start = -120;
        laser_cart_srv.request.end = 120;
        laser_cart_srv.request.max_dist = 90.0;

        if (!laser_cart_client.call(laser_cart_srv)) {
            ROS_ERROR("Failed to call laser_cart_client.call()");
            continue;
        }

        int amount;
        double wall_direction = ransac(laser_cart_srv.response.values, 0.08, 15000, &amount);
        ROS_INFO("Wall direction: %f %d", wall_direction, amount);

        driver.turn_n_degrees(std::abs(wall_direction), wall_direction <0?right:left);
        return 0;

    }

    return 0;
}
