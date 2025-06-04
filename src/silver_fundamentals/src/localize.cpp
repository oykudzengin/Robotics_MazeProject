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
#include <ransac.h>
#include <parse_map_file.h>

#include <coordinate_conversion.h>

#define SIGMA 0
#define AMOUNT_OF_RAYS 0
#define AMOUNT_OF_PARTICLES 1000
#define AMOUNT_RANDOM_INJECTIONS 0
#define PROBABILITY_RANDOM_INJECTIONS 0

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
