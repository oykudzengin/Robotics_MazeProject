#include "ros/ros.h"
#include <feedback_drive.h>


int main(int argc, char **argv) {
    ros::init(argc, argv, "square_with_encoders");
    FeedbackDrive driver = FeedbackDrive(3.2, 24.8, 10.0);
    driver.reset_encoders();
    for (int i = 0; i < 4; i++) {
        driver.drive_n_cm(100);
        driver.turn_n_degrees(90.0, right);
    }

}
