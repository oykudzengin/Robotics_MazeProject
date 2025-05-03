#include "ros/ros.h"
#include "silver_fundamentals/LaserCartesian.h"
#include <config.h>
#include <sstream>
#include <string>

int main(int argc, char** argv) {
    ros::init(argc, argv, "align");
    ros::NodeHandle n;

    ros::ServiceClient laser_cart_client = n.serviceClient<silver_fundamentals::LaserCartesian>("laserAngleRangeCartesian");
    silver_fundamentals::LaserCartesian laser_cart_srv;

    ros::Rate rate(10);

    while (ros::ok()) {
        laser_cart_srv.request.start = -120;
        laser_cart_srv.request.end = 120;

        if (!laser_cart_client.call(laser_cart_srv)) {
            ROS_ERROR("Failed to call laser_cart_client.call()");
            continue;
        }

        // print out the points
        const auto& pts = laser_cart_srv.response.values;
        std::ostringstream oss;
        oss << "Points: ";
        for (size_t i = 0; i < pts.size(); ++i) {
            const auto& p = pts[i];
            oss << "("
                << std::fixed << std::setprecision(3)
                << p.x << "," << p.y
                << ")";
            if (i + 1 < pts.size()) oss << ", ";
        }

        // Print them all in one line:
        ROS_INFO("%s", oss.str().c_str());
        ROS_INFO("Total points returned: %d", laser_cart_srv.response.size);



        rate.sleep();
    }




    return 0;
}
