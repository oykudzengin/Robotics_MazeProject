#include <feedback_drive.h>
#include <cmath>
#include "silver_fundamentals/Laser.h"
#include <laser_distance_map.h>
#include "ros/ros.h"


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

FeedbackDrive::FeedbackDrive(double wr, double wb, double s) {
    wheel_radius = wr;
    wheel_base = wb;
    speed = s;

    drive_data_client = n.serviceClient<silver_fundamentals::DriveData>("encoder_data");
    drive_client = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    reset_encoders_client = n.serviceClient<silver_fundamentals::ResetEncoders>("wrap_reset_encoders");

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
            drive_srv.request.left = -4;
            drive_srv.request.right = 4;
            break;
        }
        case right: {
            ROS_INFO("direction is right");
            drive_srv.request.left = 4;
            drive_srv.request.right = -4;
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

    ros::ServiceClient laser_client = n.serviceClient<silver_fundamentals::Laser>("laserAngleRange");
    silver_fundamentals::Laser laser_srv;

    ros::Rate rate(100);

    compute_hitbox(17.0, should_distance);


    if (window_intersects_box()) {
        // drive  back until does not
    } else {
        // drive forward until it does
    }


}

void FeedbackDrive::compute_hitbox(double width, double distance) {
    width /= 100.0;
    distance /= 100.0;

    for (unsigned int i = 0; i < LIDAR_POINTS; ++i) {
        double angle_deg = ANGLE_MIN + i * ANGLE_STEP;
        double angle_rad = angle_deg * PI / 180.0;
        hitbox[i] = threshold_for_angle(angle_rad, width, distance);
    }
}

bool FeedbackDrive::window_intersects_box(const std::vector<double> &ranges, const double min_angle, const double max_angle) {
    if (min_angle < max_angle) return false;

    int idx_start = static_cast<int>(
        std::round(std::max(min_angle + ANGLE_SPAN / 2.0, 0.0) * ((double) LIDAR_POINTS) / ANGLE_SPAN));
    int idx_end = static_cast<int>(
        std::round(std::min(max_angle + ANGLE_SPAN / 2.0, 240.0) * ((double) LIDAR_POINTS) / ANGLE_SPAN));

    for (int i = idx_start; i <= idx_end; ++i) {
        const double r = ranges[i];
        const double thr = hitbox[i];
        if (r > 0.0 && r <= thr) {
            return true;
        }
    }
    return false;
}

