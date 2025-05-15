#ifndef FEEDBACK_DRIVE_H
#define FEEDBACK_DRIVE_H

#include <ros/ros.h>
#include "silver_fundamentals/DriveData.h"
#include "create_fundamentals/DiffDrive.h"
#include "silver_fundamentals/ResetEncoders.h"
#include <config.h>
#include <geometry_msgs/Point.h>

class FeedbackDrive {
    public:
        FeedbackDrive(double, double, double);
        ~FeedbackDrive() = default;

        void drive_n_cm(double n);
        void turn_n_degrees(double n, direction d);
        void reset_encoders(void);
        void distance_to_wall(double should_distance);
        int turn(double, direction, double, double);
        int drive_along_wall(double, double, double, bool (*)());

        int potentialFieldDrive(geometry_msgs::Point goal, double k_att, double k_rep, double r);


    private:
        double wheel_radius;
        double wheel_base;
        double speed;


        std::array<double, LIDAR_POINTS> hitbox;

        ros::NodeHandle n;

        ros::ServiceClient drive_data_client;
        silver_fundamentals::DriveData drive_data_srv;

        ros::ServiceClient drive_client;
        create_fundamentals::DiffDrive drive_srv;

        ros::ServiceClient reset_encoders_client;
        silver_fundamentals::ResetEncoders reset_encoders_srv;

        ros::ServiceClient laser_cart_client;


        ros::Rate rate{1000};

        void compute_hitbox(double width, double distance);
        bool window_intersects_box(const std::vector<double> &ranges, double min_angle, double max_angle) const;

        geometry_msgs::Point get_potentials(geometry_msgs::Point current_pos, geometry_msgs::Point goal, double k_att, double k_rep, double r);
        geometry_msgs::Point position_update(geometry_msgs::Point current_pos, double delta_right, double delta_left);


};



#endif //FEEDBACK_DRIVE_H
