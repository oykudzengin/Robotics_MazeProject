#include "ros/ros.h"
#include <feedback_drive.h>
#include <string.h>


int main(int argc, char **argv) {
    double wheel_radius = std::atof(argv[1]);
    double wheel_base = std::atof(argv[2]);
    ros::init(argc, argv, "square_with_encoders");
    // larger wheelbase -> higher turning angle
    // larger wheelradius -> going less far
    FeedbackDrive driver = FeedbackDrive(wheel_radius, wheel_base, 10.0);
    driver.reset_encoders();
    for (int i = 0; i < 4; i++) {
        driver.drive_n_cm(100);
        driver.turn_n_degrees(90.0, right);
    }

}
