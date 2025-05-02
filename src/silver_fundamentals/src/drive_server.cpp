#include "ros/ros.h"
#include <create_fundamentals/SensorPacket.h>
#include "silver_fundamentals/DriveData.h"

static float left_encoder;
static float right_encoder;
static int i;

void driveCallback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
  left_encoder = msg->encoderLeft;
  right_encoder = msg->encoderRight;
  i++;
  if (i % 10 == 0) {
      i = 0;
      ROS_INFO("Encoder Left: %f, Encoder Right: %f", left_encoder, right_encoder);
  }

}



bool get_encoder_data(silver_fundamentals::DriveData::Request  &req, silver_fundamentals::DriveData::Response &res) {
    ROS_INFO("Received request");
    res.left_encoder = left_encoder;
    res.right_encoder = right_encoder;
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