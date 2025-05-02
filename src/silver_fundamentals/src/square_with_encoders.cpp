#include "ros/ros.h"
#include <feedback_drive.h>


int main(int argc, char **argv) {
    ros::init(argc, argv, "square_with_encoders");
    FeedbackDrive driver = FeedbackDrive(3.2, 25.0, 10.0);
    driver.turn_n_degrees(90, right);
}
