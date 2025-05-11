#include <feedback_drive.h>
#include <cmath>
#include "silver_fundamentals/Laser.h"
#include <laser_distance_map.h>
#include "ros/ros.h"
#include <vector>
#include <sstream>

#include "ransac.h"

#define ROBOT_RADIUS 13.3


inline double threshold_for_angle(double theta,
                                  double half_width,
                                  double half_length) {
    const double cx = std::cos(theta);
    const double sy = std::sin(theta);
    const double eps = 1e-8;  // guard against division by zero

    // distance to x-slab (|x| = half_length)
    double dx = (std::fabs(cx) > eps)
              ? half_length / std::fabs(cx)
              : std::numeric_limits<double>::infinity();

    // distance to y-slab (|y| = half_width)
    double dy = (std::fabs(sy) > eps)
              ? half_width  / std::fabs(sy)
              : std::numeric_limits<double>::infinity();
    double d = std::min(dx, dy);
    return std::isfinite(d) ? d : 0.0;
}

static int points_on_angle_range(const double angle) {
    return angle * PI / 180 / ANGLE_STEP;
}

FeedbackDrive::FeedbackDrive(double wr, double wb, double s) {
    wheel_radius = wr;
    wheel_base = wb;
    speed = s;

    drive_data_client = n.serviceClient<silver_fundamentals::DriveData>("encoder_data");
    drive_client = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    reset_encoders_client = n.serviceClient<silver_fundamentals::ResetEncoders>("wrap_reset_encoders");
    laser_cart_client = n.serviceClient<silver_fundamentals::Laser>("laserAngleRangeCartesian");

}


void FeedbackDrive::drive_n_cm(double n) {
    ROS_INFO("Drive n CM %f", n);
    drive_srv.request.left = speed;
    drive_srv.request.right = speed;
    double distance_in_rad = n/wheel_radius;

    // reset_encoders_client.call(reset_encoders_srv);
    drive_data_client.call(drive_data_srv);
    double base_line_left = drive_data_srv.response.left_encoder;
    double base_line_right = drive_data_srv.response.right_encoder;


    drive_client.call(drive_srv);
    ROS_INFO("%d, %d", ros::ok(), drive_data_client.call(drive_data_srv));
    while (ros::ok() && drive_data_client.call(drive_data_srv)) {
        double left_delta = drive_data_srv.response.left_encoder - base_line_left;
        double right_delta = drive_data_srv.response.right_encoder - base_line_right;
        ROS_INFO("%f %f", left_delta, right_delta);
        double current_rad_distance = (left_delta + right_delta)/2.0;
        if (current_rad_distance > distance_in_rad)
            break;

        rate.sleep();
    }
    ROS_INFO("Drive done");

    drive_srv.request.left = 0;
    drive_srv.request.right = 0;
    drive_client.call(drive_srv);
}


void FeedbackDrive::turn_n_degrees(double n, direction d) {
    switch (d) {
        case none: {
            ROS_INFO("direction is none");
            drive_srv.request.left = speed;
            drive_srv.request.right = speed;
            break;
        }
        case left: {
            ROS_INFO("direction is left");
            drive_srv.request.left = -2;
            drive_srv.request.right = 2;
            break;
        }
        case right: {
            ROS_INFO("direction is right");
            drive_srv.request.left = 2;
            drive_srv.request.right = -2;
            break;
        }
    }
    double distance_in_rad = (n*wheel_base*PI)/(360.0*wheel_radius);

    //reset_encoders_client.call(reset_encoders_srv);
    drive_data_client.call(drive_data_srv);
    double base_line_left = drive_data_srv.response.left_encoder;
    double base_line_right = drive_data_srv.response.right_encoder;

    drive_client.call(drive_srv);
    while (ros::ok() && drive_data_client.call(drive_data_srv)) {
        double left_delta = drive_data_srv.response.left_encoder - base_line_left;
        double right_delta = drive_data_srv.response.right_encoder - base_line_right;
        double current_rad_distance = (std::abs(left_delta) + std::abs(right_delta))/2.0;
        if (current_rad_distance > distance_in_rad)
            break;

        rate.sleep();
    }

    drive_srv.request.left = 0;
    drive_srv.request.right = 0;
    drive_client.call(drive_srv);
}

void FeedbackDrive::reset_encoders(void) {
    reset_encoders_client.call(reset_encoders_srv);
}

void FeedbackDrive::distance_to_wall(double should_distance) {
    ros::NodeHandle n;
    double real_distance;
    double offset = 0.01 * should_distance;
    double speed = 2.0;

    ros::ServiceClient laser_client = n.serviceClient<silver_fundamentals::Laser>("laserAngleRange");
    silver_fundamentals::Laser laser_srv;

    ros::ServiceClient drive_client = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    create_fundamentals::DiffDrive drive_srv;


    ros::Rate rate(100);


    compute_hitbox(ROBOT_RADIUS, should_distance);


    laser_srv.request.start = -90.0;
    laser_srv.request.end = 90.0;

    while(!laser_client.call(laser_srv)) {
        ROS_ERROR("Failed to call laser polar service");
    }
    std::vector<double> ranges = laser_srv.response.values;

    if (window_intersects_box(ranges, -90.0, 90.0)) {
        ROS_INFO("Drive backward!");
        // drive back until window_intersect says drive forward
        drive_srv.request.left = -speed;
        drive_srv.request.right = -speed;
        while(ros::ok() && window_intersects_box(ranges, -90.0, 90.0)) {
            drive_client.call(drive_srv);
            //rate.sleep();
            laser_srv.request.start = -90;
            laser_srv.request.end = 90;

            while(!laser_client.call(laser_srv)) {
                ROS_ERROR("Failed to call laser polar service");
            }
            ranges = laser_srv.response.values;
        }
        // Stop because window_intersect now says drive forward
        ROS_INFO("Stop Driving backward!");
        drive_srv.request.left = 0;
        drive_srv.request.right = 0;
        drive_client.call(drive_srv);
    } else {
        ROS_INFO("Drive forward!");
        // drive forward until window_intersect says drive backwards
        drive_srv.request.left = speed;
        drive_srv.request.right = speed;
        while(ros::ok() && !window_intersects_box(ranges, -90.0, 90.0)) {
            drive_client.call(drive_srv);
            //rate.sleep();
            laser_srv.request.start = -90;
            laser_srv.request.end = 90;

            while(!laser_client.call(laser_srv)) {
                ROS_ERROR("Failed to call laser polar service");
            }
            ranges = laser_srv.response.values;
        }
        // Stop because window_intersect now says drive backward
        ROS_INFO("Stop Driving forward!");
        drive_srv.request.left = 0;
        drive_srv.request.right = 0;
        drive_client.call(drive_srv);
    }
}

void FeedbackDrive::compute_hitbox(double width, double distance) {
    width /= 100.0;
    distance /= 100.0;

    for (unsigned int i = 0; i < LIDAR_POINTS; ++i) {
        double angle_deg = ANGLE_MIN + i * ANGLE_STEP;
        double angle_rad = angle_deg * PI / 180.0;
        // ROS_INFO("arngel rad: %f", angle_rad);
        // ROS_INFO("arngel deg: %f", angle_deg);
        hitbox[i] = threshold_for_angle(angle_rad, width, distance);
    }
}

bool FeedbackDrive::window_intersects_box(const std::vector<double> &ranges, const double min_angle, const double max_angle) {
    if (min_angle >= max_angle) return false;

    int idx_start = static_cast<int>(
        std::round(std::max(min_angle + ANGLE_SPAN / 2.0, 0.0) * ((double) LIDAR_POINTS) / ANGLE_SPAN));
    int idx_end = static_cast<int>(
        std::round(std::min(max_angle + ANGLE_SPAN / 2.0, 240.0) * ((double) LIDAR_POINTS) / ANGLE_SPAN));


    for (int i = idx_start; i <= idx_end; ++i) {
        const double r = ranges[i-idx_start];
        const double thr = hitbox[i];
        if (r <= thr) {
            return true;
        }
    }

    return false;



}

int FeedbackDrive::turn(const double angle, direction dir, const double radius, const double speed) {
    const double angle_rad = angle * PI / (180.0 * wheel_radius);
    const double dist_from_inner_wheel = radius - WHEEL_BASE/2;
    const double dist_from_outer_wheel = radius + WHEEL_BASE/2;
    const double inner_driving_dist = angle_rad * dist_from_inner_wheel;
    const double outer_driving_dist = angle_rad * dist_from_outer_wheel;

    ROS_INFO("angle is %f, meaning inner_dist of %f, outer dist of %f", angle_rad, inner_driving_dist, outer_driving_dist);

    volatile double inner_driving_speed = speed * (radius - WHEEL_BASE/2) / radius;
    volatile double outer_driving_speed = speed * (radius + WHEEL_BASE/2) / radius;

    switch (dir) {
        case none: {
            ROS_ERROR("Direction must be set.");
            return 1;
        }
        case left: {
            ROS_INFO("Turning left.");
            drive_srv.request.left = inner_driving_speed;
            drive_srv.request.right = outer_driving_speed;
            break;
        }
        case right: {
            ROS_INFO("Turning right.");
            drive_srv.request.left = outer_driving_speed;
            drive_srv.request.right = inner_driving_speed;
            break;
        }
    }

    drive_data_client.call(drive_data_srv);
    double base_line_inner = dir == left ? drive_data_srv.response.left_encoder:drive_data_srv.response.right_encoder;
    double base_line_outer = dir == left ? drive_data_srv.response.right_encoder:drive_data_srv.response.left_encoder;

    drive_client.call(drive_srv);
    while (ros::ok() && drive_data_client.call(drive_data_srv)) {
        double inner_delta = (dir == left ? drive_data_srv.response.left_encoder: drive_data_srv.response.right_encoder) - base_line_inner;
        double outer_delta = (dir == left ? drive_data_srv.response.right_encoder: drive_data_srv.response.left_encoder) - base_line_outer;

        if (abs(inner_delta) > abs(inner_driving_dist) || abs(outer_delta) > abs(outer_driving_dist))
            break;

        rate.sleep();
    }

    drive_srv.request.left = 0;
    drive_srv.request.right = 0;
    drive_client.call(drive_srv);
    return 0;
}

int FeedbackDrive::drive_along_wall(const double right_wall_dist, const double left_wall_dist, const double speed, bool (*cond)()) {

    silver_fundamentals::Laser right_laser_srv;
    silver_fundamentals::Laser left_laser_srv;
    silver_fundamentals::Laser front_laser_srv;

    int ransac_iterations = 7000;
    double ransac_distance = 0.8;

    right_laser_srv.request.start = -110;
    right_laser_srv.request.stop = -70;
    right_laser_srv.request.max_dist = 100;
    left_laser_srv.request.start = 70;
    left_laser_srv.request.stop = 110;
    left_laser_srv.request.max_dist = 100;
    front_laser_srv.request.start = -25;
    front_laser_srv.request.stop = 25;
    front_laser_srv.request.max_dist = 100;
    int side_laser_points = points_on_angle_range(40);
    int front_laser_points = points_on_angle_range(50);


    drive_srv.request.left = speed;
    drive_srv.request.right = speed;

    bool success = cond();
    while (ros::ok() && !success) {

        laser_cart_client.call(right_laser_srv);
        laser_cart_client.call(left_laser_srv);
        int right_values_used, left_values_used;

        std::vector<geometry_msgs::Point> right_pts = right_laser_srv.response.values;
        std::vector<geometry_msgs::Point> left_pts = left_laser_srv.response.values;

        // get ransac angle
        double right_wall_angle = ransac(right_pts, ransac_distance, ransac_iterations, &right_values_used);
        double left_wall_angle = ransac(left_pts, ransac_distance, ransac_iterations, &left_values_used);



        drive_client.call(drive_srv);


        rate.sleep();
    }
}


