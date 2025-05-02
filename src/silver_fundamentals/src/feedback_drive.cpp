#include <feedback_drive.h>

FeedbackDrive::FeedbackDrive(double wr, double wb, double s) {
    wheel_radius = wr;
    wheel_base = wb;
    speed = s;
}


void FeedbackDrive::drive_n_cm(double n) {
    ROS_INFO("Drive n CM %f", n);
    drive_srv.request.left = speed;
    drive_srv.request.right = speed;
    double distance_in_rad = n/wheel_radius;

    reset_encoders_client.call(reset_encoders_srv);
    ROS_INFO("%d, %d", ros::ok(), drive_data_client.call(drive_data_srv));
    while (ros::ok() && drive_data_client.call(drive_data_srv)) {
        double current_rad_distance = (drive_data_srv.response.left_encoder + drive_data_srv.response.right_encoder)/2.0;
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
            drive_srv.request.left = speed;
            drive_srv.request.right = speed;
            break;
        }
        case left: {
            drive_srv.request.left = -4;
            drive_srv.request.right = 4;
            break;
        }
        case right: {
            drive_srv.request.left = 4;
            drive_srv.request.right = -4;
            break;
        }
    }
    double distance_in_rad = (n*WHEEL_BASE*PI)/360.0;

    reset_encoders_client.call(reset_encoders_srv);
    while (ros::ok() && drive_data_client.call(drive_data_srv)) {
        double current_rad_distance = (drive_data_srv.response.left_encoder + drive_data_srv.response.right_encoder)/2.0;
        if (current_rad_distance > distance_in_rad)
            break;

        rate.sleep();
    }

    drive_srv.request.left = 0;
    drive_srv.request.right = 0;
    drive_client.call(drive_srv);
}

