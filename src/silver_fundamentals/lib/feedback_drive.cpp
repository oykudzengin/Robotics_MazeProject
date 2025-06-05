#include <feedback_drive.h>
#include <cmath>
#include "silver_fundamentals/Laser.h"
#include "silver_fundamentals/LaserCartesian.h"
#include <laser_distance_map.h>
#include "ros/ros.h"
#include <vector>
#include <geometry_msgs/Point.h>
#include <sstream>
#include <config.h>
#include <ransac.h>
#include <coordinate_conversion.h>

#define ROBOT_RADIUS 13.3f
#define LIDAR_SENSOR_OFFSET 14.0f

#define WIDTH_THRESHOLD_RADIUS 4f
#define ANGLE_THRESHOLD 5f


inline double threshold_for_angle(double theta,
                                  double half_width,
                                  double half_length) {
    const double cx = std::cos(theta);
    const double sy = std::sin(theta);
    const double eps = 1e-8;  // guard against division by zero

    // distance to x-slab (|x| = half_length)
    double dx = (std::fabs(cx) > eps)
              ? half_length / std::fabs(cx)
              : std::numeric_limits<double>::infinity();

    // distance to y-slab (|y| = half_width)
    double dy = (std::fabs(sy) > eps)
              ? half_width  / std::fabs(sy)
              : std::numeric_limits<double>::infinity();
    double d = std::min(dx, dy);
    return std::isfinite(d) ? d : 0.0;
}

static int points_on_angle_range(const double angle) {
    return static_cast<int>(angle / ANGLE_STEP);
}

FeedbackDrive::FeedbackDrive(double wr, double wb, double s) {
    wheel_radius = wr;
    wheel_base = wb;
    speed = s;

    drive_data_client = n.serviceClient<silver_fundamentals::DriveData>("encoder_data");
    drive_client = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    reset_encoders_client = n.serviceClient<silver_fundamentals::ResetEncoders>("wrap_reset_encoders");
    laser_cart_client = n.serviceClient<silver_fundamentals::LaserCartesian>("laserAngleRangeCartesian");
    laser_cart_offset_client = n.serviceClient<silver_fundamentals::LaserCartesian>("laserAngleRangeCartesianOffset");

}


void FeedbackDrive::drive_n_cm(double n) {
    ROS_INFO("Drive n CM %f", n);
    drive_srv.request.left = speed;
    drive_srv.request.right = speed;
    double distance_in_rad = n/wheel_radius;

    // reset_encoders_client.call(reset_encoders_srv);
    drive_data_client.call(drive_data_srv);
    double base_line_left = drive_data_srv.response.left_encoder;
    double base_line_right = drive_data_srv.response.right_encoder;


    drive_client.call(drive_srv);
    ROS_INFO("%d, %d", ros::ok(), drive_data_client.call(drive_data_srv));
    while (ros::ok() && drive_data_client.call(drive_data_srv)) {
        double left_delta = drive_data_srv.response.left_encoder - base_line_left;
        double right_delta = drive_data_srv.response.right_encoder - base_line_right;
        ROS_INFO("%f %f", left_delta, right_delta);
        double current_rad_distance = (left_delta + right_delta)/2.0;
        if (current_rad_distance > distance_in_rad)
            break;

        rate.sleep();
    }
    ROS_INFO("Drive done");

    drive_srv.request.left = 0;
    drive_srv.request.right = 0;
    drive_client.call(drive_srv);
}

void FeedbackDrive::turn_n_degrees(double n, direction d) {
    switch (d) {
        case none: {
            ROS_INFO("direction is none");
            drive_srv.request.left = speed;
            drive_srv.request.right = speed;
            break;
        }
        case left: {
            ROS_INFO("direction is left");
            drive_srv.request.left = -2;
            drive_srv.request.right = 2;
            break;
        }
        case right: {
            ROS_INFO("direction is right");
            drive_srv.request.left = 2;
            drive_srv.request.right = -2;
            break;
        }
    }
    double distance_in_rad = (n*wheel_base*PI)/(360.0*wheel_radius);

    //reset_encoders_client.call(reset_encoders_srv);
    drive_data_client.call(drive_data_srv);
    double base_line_left = drive_data_srv.response.left_encoder;
    double base_line_right = drive_data_srv.response.right_encoder;

    drive_client.call(drive_srv);
    while (ros::ok() && drive_data_client.call(drive_data_srv)) {
        double left_delta = drive_data_srv.response.left_encoder - base_line_left;
        double right_delta = drive_data_srv.response.right_encoder - base_line_right;
        double current_rad_distance = (std::abs(left_delta) + std::abs(right_delta))/2.0;
        if (current_rad_distance > distance_in_rad)
            break;

        rate.sleep();
    }

    drive_srv.request.left = 0;
    drive_srv.request.right = 0;
    drive_client.call(drive_srv);
}

void FeedbackDrive::reset_encoders(void) {
    reset_encoders_client.call(reset_encoders_srv);
}

void FeedbackDrive::distance_to_wall(double should_distance) {
    ros::NodeHandle n;
    double real_distance;
    double offset = 0.01 * should_distance;
    double speed = 2.0;

    ros::ServiceClient laser_client = n.serviceClient<silver_fundamentals::Laser>("laserAngleRange");
    silver_fundamentals::Laser laser_srv;

    ros::ServiceClient drive_client = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    create_fundamentals::DiffDrive drive_srv;


    ros::Rate rate(100);


    compute_hitbox(ROBOT_RADIUS, should_distance);


    laser_srv.request.start = -90.0;
    laser_srv.request.end = 90.0;

    while(!laser_client.call(laser_srv)) {
        ROS_ERROR("Failed to call laser polar service");
    }
    std::vector<double> ranges = laser_srv.response.values;

    if (window_intersects_box(ranges, -90.0, 90.0)) {
        ROS_INFO("Drive backward!");
        // drive back until window_intersect says drive forward
        drive_srv.request.left = -speed;
        drive_srv.request.right = -speed;
        while(ros::ok() && window_intersects_box(ranges, -90.0, 90.0)) {
            drive_client.call(drive_srv);
            //rate.sleep();
            laser_srv.request.start = -90;
            laser_srv.request.end = 90;

            while(!laser_client.call(laser_srv)) {
                ROS_ERROR("Failed to call laser polar service");
            }
            ranges = laser_srv.response.values;
        }
        // Stop because window_intersect now says drive forward
        ROS_INFO("Stop Driving backward!");
        drive_srv.request.left = 0;
        drive_srv.request.right = 0;
        drive_client.call(drive_srv);
    } else {
        ROS_INFO("Drive forward!");
        // drive forward until window_intersect says drive backwards
        drive_srv.request.left = speed;
        drive_srv.request.right = speed;
        while(ros::ok() && !window_intersects_box(ranges, -90.0, 90.0)) {
            drive_client.call(drive_srv);
            //rate.sleep();
            laser_srv.request.start = -90;
            laser_srv.request.end = 90;

            while(!laser_client.call(laser_srv)) {
                ROS_ERROR("Failed to call laser polar service");
            }
            ranges = laser_srv.response.values;
        }
        // Stop because window_intersect now says drive backward
        ROS_INFO("Stop Driving forward!");
        drive_srv.request.left = 0;
        drive_srv.request.right = 0;
        drive_client.call(drive_srv);
    }
}

void FeedbackDrive::compute_hitbox(double width, double distance) {
    width /= 100.0;
    distance /= 100.0;

    for (unsigned int i = 0; i < LIDAR_POINTS; ++i) {
        double angle_deg = ANGLE_MIN + i * ANGLE_STEP;
        double angle_rad = angle_deg * PI / 180.0;
        // ROS_INFO("arngel rad: %f", angle_rad);
        // ROS_INFO("arngel deg: %f", angle_deg);
        hitbox[i] = threshold_for_angle(angle_rad, width, distance);
    }
}

bool FeedbackDrive::window_intersects_box(const std::vector<double> &ranges, const double min_angle, const double max_angle) const {
    if (min_angle >= max_angle) return false;

    int idx_start = static_cast<int>(
        std::round(std::max(min_angle + ANGLE_SPAN / 2.0, 0.0) * ((double) LIDAR_POINTS) / ANGLE_SPAN));
    int idx_end = static_cast<int>(
        std::round(std::min(max_angle + ANGLE_SPAN / 2.0, 240.0) * ((double) LIDAR_POINTS) / ANGLE_SPAN));


    for (int i = idx_start; i <= idx_end; ++i) {
        const double r = ranges[i-idx_start];
        const double thr = hitbox[i];
        if (r <= thr) {
            return true;
        }
    }

    return false;



}

int FeedbackDrive::turn(const double angle, direction dir, const double radius, const double speed) {
    const double angle_rad = angle * PI / (180.0 * wheel_radius);
    const double dist_from_inner_wheel = radius - WHEEL_BASE/2;
    const double dist_from_outer_wheel = radius + WHEEL_BASE/2;
    const double inner_driving_dist = angle_rad * dist_from_inner_wheel;
    const double outer_driving_dist = angle_rad * dist_from_outer_wheel;

    ROS_INFO("angle is %f, meaning inner_dist of %f, outer dist of %f", angle_rad, inner_driving_dist, outer_driving_dist);

    volatile double inner_driving_speed = speed * (radius - WHEEL_BASE/2) / radius;
    volatile double outer_driving_speed = speed * (radius + WHEEL_BASE/2) / radius;

    switch (dir) {
        case none: {
            ROS_ERROR("Direction must be set.");
            return 1;
        }
        case left: {
            ROS_INFO("Turning left.");
            drive_srv.request.left = inner_driving_speed;
            drive_srv.request.right = outer_driving_speed;
            break;
        }
        case right: {
            ROS_INFO("Turning right.");
            drive_srv.request.left = outer_driving_speed;
            drive_srv.request.right = inner_driving_speed;
            break;
        }
    }

    drive_data_client.call(drive_data_srv);
    double base_line_inner = dir == left ? drive_data_srv.response.left_encoder:drive_data_srv.response.right_encoder;
    double base_line_outer = dir == left ? drive_data_srv.response.right_encoder:drive_data_srv.response.left_encoder;

    drive_client.call(drive_srv);
    while (ros::ok() && drive_data_client.call(drive_data_srv)) {
        double inner_delta = (dir == left ? drive_data_srv.response.left_encoder: drive_data_srv.response.right_encoder) - base_line_inner;
        double outer_delta = (dir == left ? drive_data_srv.response.right_encoder: drive_data_srv.response.left_encoder) - base_line_outer;

        if (abs(inner_delta) > abs(inner_driving_dist) || abs(outer_delta) > abs(outer_driving_dist))
            break;

        rate.sleep();
    }

    drive_srv.request.left = 0;
    drive_srv.request.right = 0;
    drive_client.call(drive_srv);
    return 0;
}

// x is right-left
geometry_msgs::Point FeedbackDrive::position_update(geometry_msgs::Point current_pos, double delta_right, double delta_left) {
    double delta_right_m = delta_right*wheel_radius/100;
    double delta_left_m = delta_left*wheel_radius/100;
    double delta_trans = (delta_right_m + delta_left_m)/2.0;
    double delta_rot = (delta_right_m - delta_left_m)/(2.0*wheel_base/100);
    current_pos.x += delta_trans * std::sin(current_pos.z+delta_rot);
    current_pos.y += delta_trans * std::cos(current_pos.z+delta_rot);
    current_pos.z += 2.0*delta_rot;

    return current_pos;
}



geometry_msgs::Point FeedbackDrive::get_potentials(geometry_msgs::Point current_pos, geometry_msgs::Point goal, double k_att, double k_rep, double r) {
    // init and call laser srv
    silver_fundamentals::LaserCartesian laser_srv;
    laser_srv.request.start = -120.0;
    laser_srv.request.end = 120.0;
    laser_srv.request.max_dist = 100.0;
    laser_srv.request.lidar_sensor_offset = LIDAR_SENSOR_OFFSET/100.0;
    laser_srv.request.wall_thickness = (ROBOT_RADIUS+1)/100.0;

    while (!laser_cart_offset_client.call(laser_srv))
        ROS_ERROR("Failed to call laser cart service, retrying...");


    // world view distances
	geometry_msgs::Point local_goal_pos = global_to_local(current_pos, goal);

    double local_dx = local_goal_pos.x;
    double local_dy = local_goal_pos.y;

    // length of force vector
    double force_vector_dist = local_goal_pos.z;
	double x_att = 1e-6;
    double y_att = 1e-6;

    if (force_vector_dist > 1e-6) {
        x_att = k_att*local_dx/force_vector_dist;
        y_att = k_att*local_dy/force_vector_dist;
    }

    // ROS_INFO("x_force: %f current_x: %f goal_x: %f", x_force, current_pos.x, goal.x);
	double x_rep = 0.0;
    double y_rep = 0.0;

    for (auto &pt : laser_srv.response.values) {
        double real_x = pt.y;
        double real_y = pt.x;
        double d_zero = std::sqrt(real_x * real_x + real_y * real_y);
        if (d_zero > r)
            continue;
        if (d_zero < 1e-6) {
            y_rep += -1e9;
            x_rep += 0;
        } else {
        	x_rep += k_rep * (1/d_zero - 1/r) * -real_x / (d_zero * d_zero * d_zero * 2.0);
        	y_rep += k_rep * (1/d_zero - 1/r) * -real_y / (d_zero * d_zero * d_zero * 2.0);
        }
    }


    double dot = x_att*x_rep + y_att*y_rep;
    double m_att = std::hypot(x_att, y_att);
    double m_rep = std::hypot(x_rep, y_rep);

    double angle_between = 0.0;
    if (m_rep > 1e10 && m_att > 1e-6 && m_rep)
        angle_between = std::acos(std::max(-1.0, std::min(dot / (m_att * m_rep), 1.0 )));

    geometry_msgs::Point result;
    result.x = x_att+x_rep;
    result.y = y_att+y_rep;
    result.z = angle_between;

    ROS_WARN("attx: %f, atty: %f, repx: %f, repy: %f, angle: %f %f", x_att, y_att, x_rep, y_rep, angle_between, current_pos.z);

    return result;
}

int FeedbackDrive::potential_field_drive(std::vector<geometry_msgs::Point> goals, double k_att, double k_rep, double r, double rot_rate) {
    double min_turning_angle = 6.0/180.0*PI;
    double max_turning_angle = 120.0/180.0*PI;


	auto sleep_rate = ros::Rate(100);
    silver_fundamentals::DriveData encoder_srv;

    drive_data_client.call(encoder_srv);
    double base_line_left = encoder_srv.response.left_encoder;
    double base_line_right = encoder_srv.response.right_encoder;

    double current_encoder_right = base_line_right;
    double current_encoder_left = base_line_left;
    double current_turning_angle = 0.0;

    const double base_speed = 6.0;


    geometry_msgs::Point current_pos;
    current_pos.x = 0.0;
    current_pos.y = 0.0;
    current_pos.z = 0.0;

    for (int goal_idx = 0; goal_idx < goals.size(); goal_idx++) {
        auto &goal = goals[goal_idx];
    	do {

            // get potential field forces
    	    const geometry_msgs::Point field_vector = get_potentials(current_pos, goal, k_att, k_rep, r);
   		    double angle = std::atan2(field_vector.x, field_vector.y);

            // check exit condition
            if (std::abs(field_vector.z)*180.0/PI > 170.0) {
                drive_srv.request.left = 0;
    			drive_srv.request.right = 0;
    			drive_client.call(drive_srv);
                return 1;
            }

        	// clip angle to always be smaller max_turning_angle and ignore it if smaller min_turning_angle
        	if (std::abs(angle) < min_turning_angle)
            	angle = 0.0;

        	if (angle > 170/180.0*PI)
            	angle -= 2.0*PI;

        	//ROS_INFO("Field vector x is %f, y is %f, angle is %f Current pos is %f %f %f", field_vector.x, field_vector.y, angle/PI*180.0, current_pos.x, current_pos.y, current_pos.z/PI*180.);

            // compute turning radius
        	double rotation_rate = angle * rot_rate * base_speed;
        	drive_srv.request.left = base_speed - wheel_base/2 * rotation_rate - (base_speed * std::min(-1.0, std::max(1.0, angle/170.0)));
        	drive_srv.request.right = base_speed + wheel_base/2 * rotation_rate - (base_speed * std::min(-1.0, std::max(1.0, angle/170.0)));
        	drive_client.call(drive_srv);

        	sleep_rate.sleep();

        	base_line_left = current_encoder_left;
        	base_line_right = current_encoder_right;
        	drive_data_client.call(encoder_srv);
        	current_encoder_left = encoder_srv.response.left_encoder;
        	current_encoder_right = encoder_srv.response.right_encoder;

            // update position
        	current_pos = position_update(current_pos, current_encoder_right-base_line_right, current_encoder_left-base_line_left);
        	//ROS_INFO("Current pos is %f %f %f", current_pos.x, current_pos.y, current_pos.z/PI*180.0);

    	} while (ros::ok() && std::sqrt((current_pos.x-goal.x) * (current_pos.x-goal.x) + (current_pos.y-goal.y) * (current_pos.y-goal.y)) > goal.z);

        //ros::Duration(0.5).sleep();
#ifndef DOTASK2
        drive_srv.request.left = 0;
        drive_srv.request.right = 0;
        drive_client.call(drive_srv);

        ros::Rate(4.0).sleep();
        ros::Rate(4.0).sleep();
#endif
    }

    drive_srv.request.left = 0;
    drive_srv.request.right = 0;
    drive_client.call(drive_srv);

    return 0;
}

