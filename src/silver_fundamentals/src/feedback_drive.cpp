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


void FeedbackDrive::distance_to_wall(double should_distance) {
    ros::NodeHandle n;
    double real_distance;
    double real_distance_side_r;
    double real_distance_side_l;
    double offset = 0.01 * should_distance;
    double side_offset;
    

    ros::ServiceClient laser_client = n.serviceClient<silver_fundamentals::Laser>("laserAngleRange");
    silver_fundamentals::Laser laser_srv;

    ros::Rate rate(100);

    // should a 
    double a = 20; // robot radius + x
    // should b 
    double b = 0.4;


    double alpha = atan(b/a);

    double should_distance_side = sqrt(pow(a, 2) + pow(b, 2));
    side_offset = 0.01 * should_distance_side;


    do {
        // get real distance 
        laser_srv.request.start = 0;
        laser_srv.request.end = 0;

        if (laser_client.call(laser_srv)) {
            real_distance = laser_srv.response.values[0];
        }

        // get real distance 
        laser_srv.request.start = alpha;
        laser_srv.request.end = alpha;

        if (laser_client.call(laser_srv)) {
            real_distance_side_r = laser_srv.response.values[0];
        }

        // get real distance 
        laser_srv.request.start = -alpha;
        laser_srv.request.end = -alpha;

        if (laser_client.call(laser_srv)) {
            real_distance_side_l = laser_srv.response.values[0];
        }

        ROS_INFO("should_side: %f , real_side_l:%f , real_side_r: %f, alpha: %f", should_distance_side, real_distance_side_l, real_distance_side_r, alpha);

        
        if ((real_distance - should_distance < offset && should_distance - real_distance < offset) && (real_distance_side_r - should_distance_side < side_offset && should_distance_side - real_distance_side_r < side_offset) && (real_distance_side_l - should_distance_side < side_offset && should_distance_side - real_distance_side_l < side_offset)) {
            // in between
            // at distance stop
            drive_srv.request.left = 0;
            drive_srv.request.right = 0;
            drive_client.call(drive_srv);
            break;
        } else if (real_distance > should_distance && real_distance_side_r > should_distance_side && real_distance_side_l > should_distance_side) {
            // drive at wal (forward)
            drive_srv.request.left = 1;
            drive_srv.request.right = 1;
            drive_client.call(drive_srv);

        } else if (real_distance < should_distance && real_distance_side_r < should_distance_side && real_distance_side_l < should_distance_side) {
            // drive backwards (from wall)
            drive_srv.request.left = -1;
            drive_srv.request.right = -1;
            drive_client.call(drive_srv);

        } else {
            // stop for nan.
            drive_srv.request.left = 0;
            drive_srv.request.right = 0;
            drive_client.call(drive_srv);
            break;
        }



    } while(ros::ok() && real_distance != should_distance);


}

