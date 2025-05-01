#include <feedback_drive.h>

FeedbackDrive::FeedbackDrive(double wr, double wb, double s) {
    wheel_radius = wr;
    wheel_base = wb;
    speed = s;
}

void FeedbackDrive::drive_n_cm(double n) {
    reset_encoders_client.call(reset_encoders_srv);
    drive_srv.left = speed;
    drive_srv.right = speed;

    while (ros::ok() && drive_data.call(drive_data_srv);) {
        double current_rad_distance = (drive_data_srv.response.left_encoder + drive_data_srv.response.right_encoder)/2.0;
        if (c) {}
    }
}