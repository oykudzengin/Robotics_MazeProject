#include <iostream>
#include "ros/ros.h"
#include <feedback_drive.h>
#include "silver_fundamentals/LaserCartesian.h"
#include "silver_fundamentals/Laser.h"
#include "silver_fundamentals/DriveData.h"
#include "create_fundamentals/DiffDrive.h"
#include <config.h>
#include <random>
#include <sstream>
#include <string>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <tf2/LinearMath/Quaternion.h>
#include <ransac.h>
#include <parse_map_file.h>
#include <cmath>

#include <coordinate_conversion.h>
#include <geometry_msgs/Pose.h>

#define SIGMA 5.0
#define AMOUNT_OF_RAYS 48
#define AMOUNT_OF_PARTICLES 1000
#define AMOUNT_RANDOM_INJECTIONS 50
#define PROBABILITY_RANDOM_INJECTIONS 0.05
#define ALPHA1 0.1 //rotation noise
#define ALPHA2 0.1 //rotation noise related to translation
#define ALPHA3 0.05 //translation noise
#define ALPHA4 0.05 //translation noise related to rotation


static std::random_device rd;
static std::mt19937 gen(rd());

struct Particle {
    geometry_msgs::Pose2D position;
    double weight;

    static Particle random(const LikelihoodField &lhf) {
        static std::uniform_real_distribution<double> ux(0, lhf.get_col_count() * lhf.get_cell_size() / 100.0);
        static std::uniform_real_distribution<double> uy(0, lhf.get_row_count() * lhf.get_cell_size() / 100.0);
        static std::uniform_real_distribution<double> utheta(-PI / 2, PI / 2);

        Particle p;
        p.position.x = -ux(gen);
        p.position.y = -uy(gen);
        p.position.theta = utheta(gen);
        p.weight = 1.0;

        return p;
    }
};


std::vector<geometry_msgs::Point> get_laser_rays(ros::ServiceClient &laser_pol_client) {
    silver_fundamentals::Laser laser_pol_srv;

    std::vector<geometry_msgs::Point> laser_rays;
    double step_size = ANGLE_SPAN / static_cast<double>(AMOUNT_OF_RAYS);

    for (int i = 0; i < AMOUNT_OF_RAYS; i++) {
        laser_pol_srv.request.start = ANGLE_MIN + i * step_size + step_size / 2;
        laser_pol_srv.request.end = ANGLE_MIN + i * step_size + step_size / 2;

        double current_rad_angle = (ANGLE_MIN + i * step_size + step_size / 2) / 180.0 * PI;

        while (!laser_pol_client.call(laser_pol_srv));

        if (laser_pol_srv.response.values[0] > 1)
            continue;

        geometry_msgs::Point ray;
        ray.x = sin(current_rad_angle) * laser_pol_srv.response.values[0];
        ray.y = cos(current_rad_angle) * laser_pol_srv.response.values[0];
        ray.z = current_rad_angle;
        laser_rays.push_back(ray);
    }
    return laser_rays;
}

void compute_weights(const LikelihoodField &lhf, std::array<Particle, AMOUNT_OF_PARTICLES> &particles,
                     std::vector<geometry_msgs::Point> &measurements) {
    for (auto &particle: particles) {
        double weight = 1;
        for (auto &measurement: measurements) {
            geometry_msgs::Point global_ray_ending = local_to_global(particle.position, measurement);
            const double ray_weight = lhf.get_prob_field_value(global_ray_ending);
            weight *= ray_weight;
        }
        particle.weight = weight;
    }
}

double sample_normal(double std_dev) {
    std::normal_distribution<double> dist(0.0, std_dev);
    return dist(gen);
}

void particle_odometry_update(std::array<Particle, AMOUNT_OF_PARTICLES> &particles, const double delta_right,
                              const double delta_left) {
    const double delta_right_m = delta_right * WHEEL_RADIUS / 100;
    const double delta_left_m = delta_left * WHEEL_RADIUS / 100;

    const double delta_trans = (delta_right_m + delta_left_m) / 2.0;
    const double delta_rot = (delta_right_m - delta_left_m) / (WHEEL_BASE / 100);

    const double local_dx = delta_trans * std::sin(delta_rot / 2.0);
    const double local_dy = delta_trans * std::cos(delta_rot / 2.0);

    const double delta_rot1 = std::atan2(local_dx, local_dy);
    const double delta_rot2 = delta_rot - delta_rot1;

    for (auto &p: particles) {
        // Add noise to the odometry values
        const double delta_rot1_hat = delta_rot1 + sample_normal(
                                          ALPHA1 * pow(delta_rot1, 2) + ALPHA2 * delta_trans * delta_trans);
        const double delta_trans_hat = delta_trans + sample_normal(
                                           ALPHA3 * pow(delta_trans, 2) + ALPHA4 * (
                                               pow(delta_rot1, 2) + pow(delta_rot2, 2)));
        const double delta_rot2_hat = delta_rot2 + sample_normal(
                                          ALPHA1 * pow(delta_rot2, 2) + ALPHA2 * pow(delta_trans, 2));


        p.position.x += delta_trans_hat * std::sin(p.position.theta + delta_rot1_hat);
        p.position.y += delta_trans_hat * std::cos(p.position.theta + delta_rot1_hat);
        p.position.theta += delta_rot1_hat + delta_rot2_hat;
    }
}

void resample(const LikelihoodField &lhf, std::array<Particle, AMOUNT_OF_PARTICLES> &particles) {
    constexpr double injection_probability = AMOUNT_RANDOM_INJECTIONS * PROBABILITY_RANDOM_INJECTIONS; // probably wrong

    std::vector<double> masses;
    masses.reserve(AMOUNT_OF_PARTICLES + 1);
    for (const auto &particle: particles)
        masses.push_back(particle.weight);
    masses.push_back(injection_probability);
    std::discrete_distribution<int> sampler(masses.begin(), masses.end());

    std::array<Particle, AMOUNT_OF_PARTICLES> new_particles;
    for (int i = 0; i < AMOUNT_OF_PARTICLES; i++) {
        const int idx = sampler(gen);
        if (idx == AMOUNT_OF_PARTICLES)
            new_particles[i] = Particle::random(lhf);
        else
            new_particles[i] = particles[idx];
    }
    particles = new_particles;
}

void viszualize_particles(ros::Publisher &posearray_pub, const std::array<Particle, AMOUNT_OF_PARTICLES> &particles) {
    geometry_msgs::PoseArray msg;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    msg.poses.reserve(AMOUNT_OF_PARTICLES);

    for (const auto &p: particles) {
        geometry_msgs::Pose pose_msg;

        // --- POSITION ---
        // Internal: p.position.x = forward, p.position.y = left
        // ROS map: x = forward, y = left
        pose_msg.position.x = -p.position.y; // left → ROS y (but placed into x field, because we swapped)
        pose_msg.position.y = -p.position.x; // forward → ROS x (but placed into y field)
        pose_msg.position.z = 0.0;

        // --- ORIENTATION ---
        // Internal θ = 0 means facing forward (ROS +X). So yaw_ros = θ_internal.
        double yaw_ros = p.position.theta;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw_ros);
        q.normalize();
        pose_msg.orientation.x = q.x();
        pose_msg.orientation.y = q.y();
        pose_msg.orientation.z = q.z();
        pose_msg.orientation.w = q.w();

        msg.poses.push_back(pose_msg);
    }

    posearray_pub.publish(msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "localize");
    ros::NodeHandle n;
    static ros::Publisher posearray_pub =
           n.advertise<geometry_msgs::PoseArray>("particle_poses", 1, true);
    auto rate = ros::Rate(100);
    const std::string pkg_path = "src/silver_fundamentals";
    std::string mapfile = pkg_path + "/maps/map.txt";

    auto driver = FeedbackDrive(3.25, 26.5, 2.0);
    driver.reset_encoders();
    const auto lhf = LikelihoodField(n, mapfile, SIGMA);

    ros::ServiceClient laser_pol_client = n.serviceClient<silver_fundamentals::Laser>("laserAngleRange");
    ros::ServiceClient drive_data_client = n.serviceClient<silver_fundamentals::DriveData>("encoder_data");
    silver_fundamentals::DriveData encoder_srv;


    // init particles array
    std::array<Particle, AMOUNT_OF_PARTICLES> particles;
    for (int i = 0; i < AMOUNT_OF_PARTICLES; i++)
        particles[i] = Particle::random(lhf);

    while (ros::ok()) {
        // do laser measurement
        std::vector<geometry_msgs::Point> reference_measurements = get_laser_rays(laser_pol_client);
        compute_weights(lhf, particles, reference_measurements);
        // do sampling
        resample(lhf, particles);
        // do drive init
        while (!drive_data_client.call(encoder_srv))
            ROS_ERROR("encoder service call failed");
        double curr_right_encoder = encoder_srv.response.right_encoder;
        double curr_left_encoder = encoder_srv.response.left_encoder;
        // do driving

        // do sleep
        rate.sleep();
        // do odometry adjustment
        while (!drive_data_client.call(encoder_srv))
            ROS_ERROR("encoder service call failed");
        double right_encoder_delta = encoder_srv.response.right_encoder - curr_right_encoder;
        double left_encoder_delta = encoder_srv.response.left_encoder - curr_left_encoder;
        particle_odometry_update(particles, right_encoder_delta, left_encoder_delta);

        viszualize_particles(posearray_pub, particles);
    }
    return 0;
}
