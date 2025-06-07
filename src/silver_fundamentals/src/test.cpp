#include <iostream>
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
#include <parse_map_file.h>

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
    ros::NodeHandle n;
    printf("start\n");
    // auto driver = FeedbackDrive(3.25, 26.5, 10.0);

    // geometry_msgs::Point goal;
    // goal.x = atof(argv[1]);
    // goal.y = atof(argv[2]);

    // driver.reset_encoders();
    // driver.potential_field_drive(goal, atof(argv[3]), atof(argv[4]), atof(argv[5]), atof(argv[6]));

    const std::string pkg_path = "src/silver_fundamentals";
    std::string mapfile = pkg_path + "/maps/map.txt";
    const auto lhf = LikelihoodField(n, mapfile, 5.0);

    geometry_msgs::Pose2D start_point;
    geometry_msgs::Point end_point;
    start_point.x = -0.5;
    start_point.y = -0.7;

    end_point.x = -0.9;
    end_point.y = -0.3;
    end_point.z = -PI/4;

    printf("dist %f\n", lhf.get_ray_wall_dist(start_point, end_point));

    ros::spin();
}
