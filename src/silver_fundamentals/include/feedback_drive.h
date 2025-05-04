#ifndef FEEDBACK_DRIVE_H
#define FEEDBACK_DRIVE_H

#include <ros/ros.h>
#include "silver_fundamentals/DriveData.h"
#include "create_fundamentals/DiffDrive.h"
#include "silver_fundamentals/ResetEncoders.h"
#include <config.h>

class FeedbackDrive {
    public:
        FeedbackDrive(double, double, double);
        ~FeedbackDrive() = default;

        void drive_n_cm(double n);
        void turn_n_degrees(double n, direction d);
        void reset_encoders(void);
        void distance_to_wall(double should_distance);


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

        ros::Rate rate{1000};

        void compute_hitbox(double width, double distance);
        bool window_intersects_box(const std::vector<double> &ranges, double min_angle, double max_angle);
};



#endif //FEEDBACK_DRIVE_H
