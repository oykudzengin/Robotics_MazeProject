#include "ros/ros.h"
#include "silver_fundamentals/Pose.h"

void poseCallback(const silver_fundamentals::Pose::ConstPtr& msg) {
  ROS_INFO("Received pose: row=%d, column=%d, orientation=%d",
           msg->row, msg->column, msg->orientation);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pose_server");
  ros::NodeHandle nh;

  // Publisher for Pose messages on 'pose_topic'
  ros::Publisher pose_pub = nh.advertise<silver_fundamentals::Pose>("pose_topic", 10);

  // Subscriber to the same topic
  ros::Subscriber pose_sub = nh.subscribe("pose_topic", 10, poseCallback);

  ros::Rate loop_rate(1);  // 1 Hz publish rate
  while (ros::ok()) {
    silver_fundamentals::Pose msg;
    // Example values; adjust as needed
    msg.row = 0;
    msg.column = 0;
    msg.orientation = silver_fundamentals::Pose::RIGHT;

    pose_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}