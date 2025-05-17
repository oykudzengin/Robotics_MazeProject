
#include "ros/ros.h"
#include <feedback_drive.h>
#include "silver_fundamentals/LaserCartesian.h"
#include "silver_fundamentals/Laser.h"
#include <config.h>
#include <random>
#include <sstream>
#include <string>
#include <geometry_msgs/Point.h>
#include <ransac.h>

// wheel diameter 3.25
// wheelbase 26.203

int turn(int argc, char **argv) {
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
    auto driver = FeedbackDrive(3.25, 27.5, 10.0);
    driver.reset_encoders();
    driver.turn(angle, dir, radius, speed);
    return 0;
}

bool stud() {return false;}

int main(int argc, char **argv) {
    ros::init(argc, argv, "test");
    auto driver = FeedbackDrive(3.25, 26.5, 10.0);

    geometry_msgs::Point goal;
    goal.x = atof(argv[1]);
    goal.y = atof(argv[2]);

    driver.reset_encoders();
    // driver.potential_field_drive(goal, atof(argv[3]), atof(argv[4]), atof(argv[5]), atof(argv[6]));
}