#include "ros/ros.h"
#include <feedback_drive.h>
#include <string.h>

// wheel diameter 3.25
// wheelbase 26.203

int main(int argc, char **argv) {
    double wheel_radius = std::atof(argv[1]);
    double wheel_base = std::atof(argv[2]);
    int amount = std::atoi(argv[3]);
    ros::init(argc, argv, "square_with_encoders");
    // larger wheelbase -> higher turning angle
    // larger wheelradius -> going further
    FeedbackDrive driver = FeedbackDrive(26.203, 3.25, 10.0);
    //driver.reset_encoders();
    driver.distance_to_wall(40.0);

}
