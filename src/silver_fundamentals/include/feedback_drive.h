#ifndef FEEDBACK_DRIVE_H
#define FEEDBACK_DRIVE_H

#include <ros/ros.h>
#include <silver_fundamentals/DriveData.h>
#include <create_fundamentals/DiffDrive.h>
#include <create_fundamentals/ResetEncoders.h>
#include <config.h>

class FeedbackDrive {
    public:
        FeedbackDrive();
        ~FeedbackDrive() = default;

        void drive_n_cm(double n);
        void turn_n_degrees(double n, direction d);

    private:
        double wheel_radius;
        double wheel_base;
        double speed;

        ros::NodeHandle n;

        ros::ServiceClient drive_data_client;
        silver_fundamentals::DriveData drive_data_srv;

        ros::ServiceClient drive_client;
        create_fundamentals::DiffDrive drive_srv;

        ros::ServiceClient reset_encoders_client;
        create_fundamentals::ResetEncoders reset_encoders_srv;

        ros::Rate rate{100};
};



#endif //FEEDBACK_DRIVE_H
