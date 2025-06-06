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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define SIGMA 40.0
#define AMOUNT_OF_RAYS 10
#define AMOUNT_OF_PARTICLES 500
#define AMOUNT_RANDOM_INJECTIONS 25
#define PROBABILITY_RANDOM_INJECTIONS 0.05
#define ALPHA1 0.05 //rotation noise
#define ALPHA2 0.05 //rotation noise related to translation
#define ALPHA3 0.05 //translation noise
#define ALPHA4 0.05 //translation noise related to rotation

#define K_ATT 10.0
#define K_REP 0.01
#define NO_EFFECTION_POT_FIELDS 0.35
#define ROT_RATE 0.03
#define BASE_SPEED 4.0


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
        p.weight = 2.0;

        return p;
    }
    static Particle zero() {
        Particle p;
        p.position.x = 0.0;
        p.position.y = 0.0;
        p.position.theta = 0.0;
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

        const double current_rad_angle = (ANGLE_MIN + i * step_size + step_size / 2) / 180.0 * PI;

        while (!laser_pol_client.call(laser_pol_srv))
            ROS_ERROR("laser_pol_client.call failed");

        if (laser_pol_srv.response.values[0] > 1)
            continue;

        geometry_msgs::Point ray;
        ray.x = sin(current_rad_angle) * laser_pol_srv.response.values[0];
        ray.y = cos(current_rad_angle) * laser_pol_srv.response.values[0] + LIDAR_SENSOR_OFFSET / 100.0;
        ray.z = std::atan2(ray.x,  ray.y);
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

    const double delta_rot1 = /* std::atan2(local_dx, local_dy); */ 0.5 * delta_rot;
    const double delta_rot2 = delta_rot - delta_rot1;

    // printf("dtrans %f, drot %f, (y, x) (%f, %f), rot1 %f rot2 %f, dy dx dtheta become %f %f %f\n", delta_trans, delta_rot, local_dy, local_dx, delta_rot1, delta_rot2, delta_trans * std::cos(delta_rot1), delta_trans * std::sin(delta_rot1), delta_rot1 + delta_rot2);

    for (auto &p: particles) {
        const double var_rot1 = ALPHA1 * pow(delta_rot1, 2) + ALPHA2 * delta_trans * delta_trans;
        const double var_trans = ALPHA3 * pow(delta_trans, 2) + ALPHA4 * (
                                               pow(delta_rot1, 2) + pow(delta_rot2, 2));
        const double var_rot2 = ALPHA1 * pow(delta_rot2, 2) + ALPHA2 * pow(delta_trans, 2);       // Add noise to the odometry values
        const double delta_rot1_hat = delta_rot1 + sample_normal(std::sqrt(var_rot1));
        const double delta_trans_hat = delta_trans + sample_normal(std::sqrt(var_trans));
        const double delta_rot2_hat = delta_rot2 + sample_normal(std::sqrt(var_rot2));


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

void visualize_reference_rays(const ros::Publisher ray_pub,
    const std::vector<geometry_msgs::Point>& reference_measurements)
{

    visualization_msgs::Marker m;
    m.header.frame_id = "map";
    m.header.stamp    = ros::Time::now();
    m.ns             = "laser_rays";
    m.id             = 0;
    m.type           = visualization_msgs::Marker::LINE_LIST;
    m.action         = visualization_msgs::Marker::ADD;

    // Each line segment is defined by two consecutive points in m.points:
    //   [start0, end0, start1, end1, start2, end2, ...]
    m.scale.x = 0.02;  // line thickness (meters)

    // Color the rays blue (or pick any color you prefer)
    m.color.r = 0.0f;
    m.color.g = 0.0f;
    m.color.b = 1.0f;
    m.color.a = 1.0f;

    m.points.clear();
    m.points.reserve(reference_measurements.size() * 2);

    // For each measurement, draw a line from (0,0,0) to (px,py,0)
    geometry_msgs::Point start, end;
    start.x = 0.0;
    start.y = 0.0;
    start.z = 0.0;

    for (const auto& meas : reference_measurements) {
        end.x = -meas.y;
        end.y = -meas.x;
        end.z = 0;  // usually 0, but copy whatever z came in

        m.points.push_back(start);
        m.points.push_back(end);
    }

    m.lifetime = ros::Duration(0.0);  // latched until overwritten
    ray_pub.publish(m);
}
void viszualize_particles(
    const ros::Publisher &marker_pub,
    const std::array<Particle, AMOUNT_OF_PARTICLES> &particles)
{
    visualization_msgs::MarkerArray markers;
    markers.markers.reserve(AMOUNT_OF_PARTICLES);

    for (size_t i = 0; i < particles.size(); ++i) {
        const auto &p = particles[i];

        visualization_msgs::Marker m;
        m.header.frame_id = "map";
        m.header.stamp    = ros::Time::now();
        m.ns              = "particles";
        m.id              = static_cast<int>(i);
        m.type            = visualization_msgs::Marker::ARROW;
        m.action          = visualization_msgs::Marker::ADD;

        // --- POSITION ---
        // Internal: p.position.x = forward, p.position.y = left
        // We want ROS: x = forward, y = left → no axis swap
        m.pose.position.x = -p.position.y;
        m.pose.position.y = -p.position.x;
        m.pose.position.z = 0.0;

        // --- ORIENTATION ---
        // Internal θ=0 means “facing +Y” (forward). ROS yaw=0 means +X, so rotate by +π/2.
        double yaw_ros = p.position.theta + PI;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw_ros);
        q.normalize();
        m.pose.orientation.x = q.x();
        m.pose.orientation.y = q.y();
        m.pose.orientation.z = q.z();
        m.pose.orientation.w = q.w();

        // --- SCALE (arrow length) ---
        // Make each arrow a quarter as long: 0.25 m
        m.scale.x = 0.25;   // arrow length in meters
        m.scale.y = 0.05;   // shaft diameter
        m.scale.z = 0.05;   // head diameter

        // --- COLOR (by weight) ---
        if (p.weight <= 0.0) {
            // red
            m.color.r = 1.0f;
            m.color.g = 0.0f;
            m.color.b = 0.0f;
            m.color.a = 1.0f;
        }
        else if (p.weight == 1.0) {
            // green
            m.color.r = 0.0f;
            m.color.g = 1.0f;
            m.color.b = 0.0f;
            m.color.a = 1.0f;
        }
	else if (p.weight > 1.0) {
	    m.color.r = 0.0f;
	    m.color.g = 0.0f;
	    m.color.b = 1.0f;
	    m.color.a = 1.0f;
	}
        else {
            // yellow
            m.color.r = 1.0f;
            m.color.g = 1.0f;
            m.color.b = 0.0f;
            m.color.a = 1.0f;
        }

        m.lifetime = ros::Duration(0.0);
        markers.markers.push_back(m);
    }

    marker_pub.publish(markers);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "localize");
    ros::NodeHandle n;
    static ros::Publisher posearray_pub =
           n.advertise<visualization_msgs::MarkerArray>("particle_poses", 1, true);
    static ros::Publisher ray_pub =
        n.advertise<visualization_msgs::Marker>("reference_rays", 1, true);
    auto rate = ros::Rate(10);
    const std::string pkg_path = "src/silver_fundamentals";
    std::string mapfile = pkg_path + "/maps/map.txt";

    auto driver = FeedbackDrive(3.25, 26.5, 2.0);
    driver.reset_encoders();
    const auto lhf = LikelihoodField(n, mapfile, SIGMA);

    ros::ServiceClient laser_pol_client = n.serviceClient<silver_fundamentals::Laser>("laserAngleRange");
    ros::ServiceClient drive_data_client = n.serviceClient<silver_fundamentals::DriveData>("encoder_data");
    ros::ServiceClient drive_client =n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    silver_fundamentals::DriveData encoder_srv;
    create_fundamentals::DiffDrive drive_srv;


    // init particles array
    std::array<Particle, AMOUNT_OF_PARTICLES> particles;
    for (int i = 0; i < AMOUNT_OF_PARTICLES; i++)
        particles[i] = Particle::random(lhf); /* particles[i] = Particle::zero(); */

    double curr_right_encoder = encoder_srv.response.right_encoder;
    double curr_left_encoder = encoder_srv.response.left_encoder;
    while (ros::ok()) {
        // do laser measurement
        std::vector<geometry_msgs::Point> reference_measurements = get_laser_rays(laser_pol_client);
        compute_weights(lhf, particles, reference_measurements);
        visualize_reference_rays(ray_pub, reference_measurements);
        // do sampling
        resample(lhf, particles);
        // do drive init
        while (!drive_data_client.call(encoder_srv))
            ROS_ERROR("encoder service call failed");

        // do driving
        geometry_msgs::Point current, goal;
        current.x = 0; current.y = 0; current.z = 0;
        goal.x = 0; goal.y = 1; goal.z = 0;
        const geometry_msgs::Point field_vector = driver.get_potentials(current, goal, K_ATT, K_REP, NO_EFFECTION_POT_FIELDS);
        double angle = std::atan2(field_vector.x, field_vector.y);
        double rotation_rate = angle * ROT_RATE * BASE_SPEED;
        drive_srv.request.left = BASE_SPEED - WHEEL_BASE/2 * rotation_rate;
        drive_srv.request.right = BASE_SPEED + WHEEL_BASE/2 * rotation_rate;
        drive_client.call(drive_srv);

        // do sleep
        rate.sleep();
        // do odometry adjustment
        while (!drive_data_client.call(encoder_srv))
            ROS_ERROR("encoder service call failed");

        double right_encoder_delta = encoder_srv.response.right_encoder - curr_right_encoder;
        double left_encoder_delta = encoder_srv.response.left_encoder - curr_left_encoder;
        curr_right_encoder = encoder_srv.response.right_encoder;
        curr_left_encoder = encoder_srv.response.left_encoder;

        particle_odometry_update(particles, right_encoder_delta, left_encoder_delta);

        viszualize_particles(posearray_pub, particles);
        printf("Particle at %f %f heading %f %f\n", particles[0].position.x, particles[0].position.y, particles[0].position.theta*180.0/PI, particles[0].weight);
    }
    return 0;
}
