
#include "ros/ros.h"
#include <feedback_drive.h>
#include <string.h>

// wheel diameter 3.25
// wheelbase 26.203

int main(int argc, char **argv) {
    double speed;
    direction dir;
    if (argc <= 4) {
        dir = right;
    } else {
        dir = atoi(argv[4]) == -1 ? right: (atoi(argv[4]) == 1 ? left : none);
    }
    if (argc <= 3) {
        speed = 4.0;
    } else {
        speed = atof(argv[3]);
    }
    double angle = atof(argv[1]);
    double radius = atof(argv[2]);
    ros::init(argc, argv, "test");
    // larger wheelbase -> higher turning angle
    // larger wheelradius -> going further
    auto driver = FeedbackDrive(3.25, 23.5, 10.0);
    driver.reset_encoders();
    driver.turn(angle, dir, radius, speed);

}