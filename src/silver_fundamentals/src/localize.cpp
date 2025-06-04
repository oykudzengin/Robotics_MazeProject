#include <iostream>
#include "ros/ros.h"
#include <feedback_drive.h>
#include "silver_fundamentals/LaserCartesian.h"
#include "silver_fundamentals/Laser.h"
#include <config.h>
#include <random>
#include <sstream>
#include <string>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <ransac.h>
#include <parse_map_file.h>
#include <cmath>

#include <coordinate_conversion.h>

#define SIGMA 0
#define AMOUNT_OF_RAYS 0
#define AMOUNT_OF_PARTICLES 1000
#define AMOUNT_RANDOM_INJECTIONS 0
#define PROBABILITY_RANDOM_INJECTIONS 0
#define ALPHA1 0.1 //rotation noise
#define ALPHA2 0.1 //rotation noise related to translation
#define ALPHA3 0.05 //translation noise
#define ALPHA4 0.05 //translation noise related to rotation


static std::random_device rd;
static std::mt19937 gen(rd());

struct Particle {
    geometry_msgs::Point position;
    double weight;

    static Particle random(const LikelihoodField &lhf) {
        static std::uniform_real_distribution<double> ux(0, lhf.get_col_count()*lhf.get_cell_size()/100.0);
        static std::uniform_real_distribution<double> uy(0, lhf.get_row_count()*lhf.get_cell_size()/100.0);
        static std::uniform_real_distribution<double> utheta(-PI/2, PI/2);

        Particle p;
        p.position.x = ux(gen);
        p.position.y = uy(gen);
        p.position.z = utheta(gen);
        p.weight = 1.0;

        return p;
    }
};




std::vector<geometry_msgs::Point> get_laser_rays(ros::ServiceClient& laser_pol_client) {
    silver_fundamentals::Laser laser_pol_srv;
    laser_pol_srv.request.max_dist = 100.0;


    std::vector<geometry_msgs::Point> laser_rays;
    double step_size = ANGLE_SPAN / static_cast<double>(AMOUNT_OF_RAYS);

    for (int i = 0; i < AMOUNT_OF_RAYS; i++) {
        laser_pol_srv.request.start = ANGLE_MIN + i * step_size + step_size/2;
        laser_pol_srv.request.end = ANGLE_MIN + i * step_size + step_size/2;

        double current_rad_angle = (ANGLE_MIN + i * step_size + step_size/2)/180.0*PI;

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

void compute_weights(const LikelihoodField &lhf, std::array<Particle, AMOUNT_OF_PARTICLES> &particles, std::array<geometry_msgs::Point, AMOUNT_OF_RAYS> &measurements) {
    for (auto &particle : particles) {
        double weight = 1;
        for (auto &measurement : measurements) {
            geometry_msgs::Point global_ray_ending = local_to_global(particle.position, measurement);
            const double ray_weight = lhf.get_prob_field_value(global_ray_ending);
            weight *= ray_weight;
        }
        particle.weight = weight;
    }
}

geometry_msgs::Point rob_position_update(geometry_msgs::Point current_pos, double delta_right, double delta_left) {
    double delta_right_m = delta_right*wheel_radius/100;
    double delta_left_m = delta_left*wheel_radius/100;
    double additional_encoder_distance = (delta_right_m + delta_left_m)/2.0;
    double average_encoder_distance = (delta_right_m - delta_left_m)/(2.0*wheel_base/100);
    current_pos.x += additional_encoder_distance * std::sin(current_pos.z+average_encoder_distance);
    current_pos.y += additional_encoder_distance * std::cos(current_pos.z+average_encoder_distance);
    current_pos.z += 2.0*average_encoder_distance;

    return current_pos;
}

double sample_normal(double std_dev) {
    std::normal_distribution<double> dist(0.0, std_dev);
    return dist(gen);
}

void update_particles_with_odometry(std::array<Particle, AMOUNT_OF_PARTICLES>& particles,
                                    const geometry_msgs::Pose2D& prev_odom,
                                    const geometry_msgs::Pose2D& curr_odom) {

    geometry_msgs::Pose2D prev_odom = {0.0, 0.0, 0.0}; // initial odom                                    
    // Compute odometry delta
    double dx = curr_odom.x - prev_odom.x;
    double dy = curr_odom.y - prev_odom.y;
    double dtheta = curr_odom.theta - prev_odom.theta;

    double delta_trans = std::sqrt(dx*dx + dy*dy);
    double delta_rot1 = std::atan2(dy, dx) - prev_odom.theta;
    double delta_rot2 = dtheta - delta_rot1;

    for (auto &p : particles) {
        // Add noise to the odometry values
        double delta_rot1_hat = delta_rot1 + sample_normal(ALPHA1 * pow(delta_rot1, 2) + ALPHA2 * delta_trans * delta_trans);
        double delta_trans_hat = delta_trans + sample_normal(ALPHA3 * pow(delta_trans, 2) + ALPHA4 * (pow(delta_rot1, 2) + pow(delta_rot2, 2)));
        double delta_rot2_hat = delta_rot2 + sample_normal(ALPHA1 * pow(delta_rot2, 2) + ALPHA2 * pow(delta_trans, 2));

        // Update particle position
        p.position.x += delta_trans * std::cos(p.position.z + delta_rot1_hat);
        p.position.y += delta_trans * std::sin(p.position.z + delta_rot1_hat);
        p.position.z += delta_rot1_hat + delta_rot2_hat;
    }
}

void resample(const LikelihoodField &lhf, std::array<Particle, AMOUNT_OF_PARTICLES> &particles) {
    constexpr double injection_probability = AMOUNT_RANDOM_INJECTIONS * PROBABILITY_RANDOM_INJECTIONS; // probably wrong

    std::vector<double> masses;
    masses.reserve(AMOUNT_OF_PARTICLES + 1);
    for (const auto &particle : particles)
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

int main(int argc, char **argv) {
    ros::init(argc, argv, "localize");
    ros::NodeHandle n;
    const std::string pkg_path = "src/silver_fundamentals";
    std::string mapfile = pkg_path + "/maps/map.txt";

    auto driver = FeedbackDrive(3.25, 26.5, 2.0);
    driver.reset_encoders();
    const auto lhf = LikelihoodField(mapfile, SIGMA);


    ros::ServiceClient laser_pol_client = n.serviceClient<silver_fundamentals::Laser>("laserAngleRange");


    // init particles array
    std::array<Particle, AMOUNT_OF_PARTICLES> particles;


    for (int i = 0 ; i < AMOUNT_OF_PARTICLES ; i++) {
        particles[i] = Particle::random(lhf);
    }

    while (ros::ok()) {
        // do laser measurement
        std::array<geometry_msgs::Point, AMOUNT_OF_PARTICLES> reference_measurements = get_laser_rays(laser_pol_client);
        compute_weights(lhf, particles, reference_measurements);
        // do sampling
        resample(lhf, particles);
        // do driving

        // do sleep

        // do odometry adjustment


    }

    return 0;
}
