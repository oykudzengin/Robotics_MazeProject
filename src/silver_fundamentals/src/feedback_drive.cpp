#include <feedback_drive.h>
#include <cmath>
#include "silver_fundamentals/Laser.h"
#include <laser_distance_map.h>
#include "ros/ros.h"

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


void FeedbackDrive::distance_to_wall(int should_distance) {
    ros::NodeHandle n;
    double real_distance;

    ros::ServiceClient laser_client = n.serviceClient<silver_fundamentals::Laser>("laserAngleRange");
    silver_fundamentals::Laser laser_srv;

    ros::Rate rate(10);
    do {
        // get real distance 
        laser_srv.request.start = 0;
        laser_srv.request.end = 0;

        if (laser_client.call(laser_srv)) {
            real_distance = laser_srv.response.values[0];
        }

        if (real_distance > should_distance) {
            // drive at wal (forward)
            drive_srv.request.left = 5;
            drive_srv.request.right = 5;
            drive_client.call(drive_srv);

        } else if (real_distance < should_distance) {
            // drive backwards (from wall)
            drive_srv.request.left = -5;
            drive_srv.request.right = -5;
            drive_client.call(drive_srv);

        } else {
            // at distance stop
            drive_srv.request.left = 0;
            drive_srv.request.right = 0;
            drive_client.call(drive_srv);
            break;
        }



    } while(ros::ok() && real_distance != should_distance);






}

