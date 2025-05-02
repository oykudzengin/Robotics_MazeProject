#include "ros/ros.h"
#include <create_fundamentals/SensorPacket.h>
#include "silver_fundamentals/DriveData.h"

static float left_encoder;
static float right_encoder;

void driveCallback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
  left_encoder = msg->encoderLeft;
  right_encoder = msg->encoderRight;
}



bool get_encoder_data(silver_fundamentals::DriveData::Request  &req, silver_fundamentals::DriveData::Response &res) {
    res.encoderLeft = left_encoder;
    res.encoderRight = right_encoder;
    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "drive_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("encoder_data", get_encoder_data);
    ros::Subscriber sub = n.subscribe("sensor_packet", 1, driveCallback);

    ROS_INFO("Ready to serve");
    ros::spin();

    return 0;
}