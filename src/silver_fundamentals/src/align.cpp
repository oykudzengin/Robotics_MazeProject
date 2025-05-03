#include "ros/ros.h"
#include "silver_fundamentals/LaserCartesian.h"
#include <config.h>
#include <random>
#include <sstream>
#include <string>
#include <geometry_msgs/Point.h>



double ransac(std::vector<geometry_msgs::Point>& pts, double max_offset, int max_iterations) {
    if (pts.size() < 2)
        return std::numeric_limits<double>::infinity();

    std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<unsigned int> uni(0, pts.size() - 1);

    unsigned int best_count = 0;
    double best_a = 0, best_b = 0, best_c = 0;
    geometry_msgs::Point best_pt;

    for (int iter = 0; iter < max_iterations; iter++) {
        const unsigned int i = uni(rng);
        const unsigned int j = uni(rng);

        // point are the same
        if (i == j) {
            iter--;
            continue;
        }

        auto &P = pts[i], &Q = pts[j];
        double dx = (P.x - Q.x);
        double dy = (P.y - Q.y);

        double norm = std::hypot(dx, dy);

        if (norm < 0.1) {
            iter--;
            continue;
        }

        double a = dy / norm;
        double b = dx / norm;
        double c = dx*P.y - dy*P.x;


        unsigned int inliners = 0;

        geometry_msgs::Point best_current_pt;
        double best_current_dist = std::numeric_limits<double>::infinity();

        for (const auto &pt : pts) {
            const double dist_to_line = std::abs(a*pt.x + b*pt.y + c);
            if (dist_to_line <= max_offset)
                inliners++;
                const double dist_to_origin = std::hypot(pt.x, pt.y);
                if (dist_to_origin < best_current_dist) {
                    best_current_dist = dist_to_origin;
                    best_current_pt   = pt;
                }
        }

        if (inliners > best_count) {
            best_count = inliners;
            best_a = a;
            best_b = b;
            best_c = c;
            best_pt = best_current_pt;
        }
    }


    if (best_count < 2)
        std::numeric_limits<double>::infinity();


    return best_pt.z;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "align");
    ros::NodeHandle n;

    ros::ServiceClient laser_cart_client = n.serviceClient<silver_fundamentals::LaserCartesian>("laserAngleRangeCartesian");
    silver_fundamentals::LaserCartesian laser_cart_srv;

    ros::Rate rate(10);

    /*
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
    */
    while (ros::ok()) {
        laser_cart_srv.request.start = -120;
        laser_cart_srv.request.end = 120;

        if (!laser_cart_client.call(laser_cart_srv)) {
            ROS_ERROR("Failed to call laser_cart_client.call()");
            continue;
        }

        double wall_direction = ransac(laser_cart_srv.response.values, 0.05, 10000);
        ROS_INFO("Wall direction: %f", wall_direction);
    }




    return 0;
}
