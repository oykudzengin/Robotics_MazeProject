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

#include <signal_handler.h>
#include <coordinate_conversion.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <LocalizeCommunication.h>
#include "silver_fundamentals/Pose.h"

#include <silver_fundamentals/Com.h>
#include <playsong.h>

#define SIGMA 11.0
#define GAMMA 1.00
#define AMOUNT_OF_RAYS 40
#define AMOUNT_OF_PARTICLES 600
#define AMOUNT_RANDOM_INJECTIONS 30
#define PROBABILITY_RANDOM_INJECTIONS 0.005
#define MIN_PARTICLE_PROB 0.01
#define ALPHA1 0.01 //rotation noise
#define ALPHA2 0.01 //rotation noise related to translation
#define ALPHA3 0.01 //translation noise
#define ALPHA4 0.01 //translation noise related to rotation

#define K_ATT 10.0
#define K_REP 0.01
#define NO_EFFECTION_POT_FIELDS 0.35
#define ROT_RATE 0.03
#define BASE_SPEED 4.0

#define MINIMAL_WALL_DIST 2
#define LOCALIZE_VAR_LOWER 0.06
#define LOCALIZE_VAR_UPPER 0.20
#define LOCALIZE_COUNT_THRESHOLD 50
#define UNLOCALIZE_COUNT_THRESHOLD 12
#define CELL_SIZE_CM 80.0


static std::random_device rd;
static std::mt19937 gen(rd());

struct Particle {
    geometry_msgs::Pose2D position;
    double weight;

    static Particle random(const LikelihoodField &lhf) {
        static std::uniform_real_distribution<double> ux(MINIMAL_WALL_DIST/100.0, (lhf.get_col_count() * lhf.get_cell_size() + MINIMAL_WALL_DIST) / 100.0);
        static std::uniform_real_distribution<double> uy(MINIMAL_WALL_DIST/100.0, (lhf.get_row_count() * lhf.get_cell_size() + MINIMAL_WALL_DIST) / 100.0);
        static std::uniform_real_distribution<double> utheta(-PI / 2, PI / 2);

        Particle p;
        p.position.x = -ux(gen);
        p.position.y = -uy(gen);
        p.position.theta = utheta(gen);
        p.weight = 10000;

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

// localaise communication to execute plan server
namespace silver_fundamentals {
    PlanSuccessState success_state = PlanSuccessState::NONE;
    std::vector<geometry_msgs::Point> com_waypoints;
    bool plan_exits = false;
}

// --- Localization state and flags ---
static bool localized_flag = false; // true once localisation is achieved
static bool alignment_state = false; // true once alignment step is complete

enum class LocalizeState {
    LOCALISING, // initial scanning/localising
    ALIGNING_ANGLE, // adjusting orientation
    ALIGNING_DRIVE, // driving into alignment position
    ALIGNING_ANGLE_ORIENT, // final fine-angle orient
    WAIT_FOR_PLAN, // waiting for plan input (localise without drive)
    EXECUTING_PLAN, // carrying out received plan
    EXECUTED_PLAN_FAIL,
    EXECUTED_PLAN_SUC
};



std::vector<geometry_msgs::Point> get_laser_rays(ros::ServiceClient &laser_pol_client) {
    silver_fundamentals::Laser laser_pol_srv;

    std::vector<geometry_msgs::Point> laser_rays;
    double step_size = ANGLE_SPAN / static_cast<double>(AMOUNT_OF_RAYS);

    for (int i = 0; i < AMOUNT_OF_RAYS; i++) {
        laser_pol_srv.request.start = ANGLE_MIN + i * step_size + step_size / 2;
        laser_pol_srv.request.end = ANGLE_MIN + i * step_size + step_size / 2;

        const double current_rad_angle = (ANGLE_MIN + i * step_size + step_size / 2) / 180.0 * PI;

        while (!laser_pol_client.call(laser_pol_srv)  && ros::ok())
            ROS_ERROR("laser_pol_client.call failed");

        if (laser_pol_srv.response.values[0] > 1)
            continue;

        geometry_msgs::Point ray;
        ray.x = sin(current_rad_angle) * laser_pol_srv.response.values[0];
        ray.y = cos(current_rad_angle) * laser_pol_srv.response.values[0] + LIDAR_SENSOR_OFFSET / 100.0;
        ray.z = std::atan2(ray.x, ray.y);
        laser_rays.push_back(ray);
    }
    return laser_rays;
}

void compute_weights(const LikelihoodField &lhf, std::array<Particle, AMOUNT_OF_PARTICLES> &particles,
                     std::vector<geometry_msgs::Point> &measurements) {
    for (auto &particle: particles) {
        if (particle.weight < 0) {
            particle.weight = 0;
            continue;
        }
        double weight = 0;
        for (auto &measurement: measurements) {
            geometry_msgs::Point global_ray_ending = local_to_global(particle.position, measurement);
            const double ray_wall_dist = lhf.get_ray_wall_dist(particle.position, global_ray_ending);
            const double delta_dist = std::abs(std::hypot(measurement.x, measurement.y) - ray_wall_dist);
            // const double delta_dist = lhf.get_field_value(global_ray_ending);
            const double ray_weight = -delta_dist * delta_dist / (2*lhf.sigma_value*lhf.sigma_value/(100*100.0));
            weight += ray_weight;
        }
        particle.weight = std::pow(std::max(std::pow(MIN_PARTICLE_PROB, AMOUNT_OF_RAYS), std::exp(weight)), GAMMA);
	// particle.weight = weight;
    }
}

double sample_normal(double std_dev) {
    std::normal_distribution<double> dist(0.0, std_dev);
    return dist(gen);
}

void particle_odometry_update(const LikelihoodField &lhf, std::array<Particle, AMOUNT_OF_PARTICLES> &particles,
                              const double delta_right,
                              const double delta_left) {
    const double delta_right_m = delta_right * WHEEL_RADIUS / 100;
    const double delta_left_m = delta_left * WHEEL_RADIUS / 100;

    const double delta_trans = (delta_right_m + delta_left_m) / 2.0;
    const double delta_rot = (delta_right_m - delta_left_m) / (WHEEL_BASE / 100);

    const double local_dx = delta_trans * std::sin(delta_rot / 2.0) * 1000.0;
    const double local_dy = delta_trans * std::cos(delta_rot / 2.0) * 1000.0;

    const double delta_rot1 = std::atan2(local_dx, local_dy); /*  0.5 * delta_rot; */
    const double delta_rot2 = delta_rot - delta_rot1;

    // printf("dtrans %f, drot %f, (y, x) (%f, %f), rot1 %f rot2 %f, dy dx dtheta become %f %f %f\n", delta_trans, delta_rot, local_dy, local_dx, delta_rot1, delta_rot2, delta_trans * std::cos(delta_rot1), delta_trans * std::sin(delta_rot1), delta_rot1 + delta_rot2);

    for (auto &p: particles) {
        const double var_rot1 = ALPHA1 * pow(delta_rot1, 2) + ALPHA2 * delta_trans * delta_trans;
        const double var_trans = ALPHA3 * pow(delta_trans, 2) + ALPHA4 * (
                                     pow(delta_rot1, 2) + pow(delta_rot2, 2));
        const double var_rot2 = ALPHA1 * pow(delta_rot2, 2) + ALPHA2 * pow(delta_trans, 2);
        // Add noise to the odometry values

        const double delta_rot1_hat = delta_rot1 + sample_normal(std::sqrt(var_rot1));
        const double delta_trans_hat = delta_trans + sample_normal(std::sqrt(var_trans));
        const double delta_rot2_hat =  delta_rot2 + sample_normal(std::sqrt(var_rot2));


        p.position.x += delta_trans_hat * std::sin(p.position.theta + delta_rot1_hat);
        p.position.y += delta_trans_hat * std::cos(p.position.theta + delta_rot1_hat);
        p.position.theta += delta_rot1_hat + delta_rot2_hat;

        geometry_msgs::Point temp;
        temp.x = p.position.x;
        temp.y = p.position.y;
        temp.z = 0;

        if (lhf.get_field_value(temp) < MINIMAL_WALL_DIST)
            p.weight = -1;
	else if (-p.position.x < 0 || -p.position.y < 0)
	    p.weight = -1;
	else if (-p.position.x * 100 > lhf.get_cell_size() * lhf.get_col_count())
	    p.weight = -1;
	else if (-p.position.y * 100 > lhf.get_cell_size() * lhf.get_row_count())
	    p.weight = -1;
    }
}

void resample(const LikelihoodField &lhf, std::array<Particle, AMOUNT_OF_PARTICLES> &particles) {
    constexpr double injection_probability = AMOUNT_RANDOM_INJECTIONS * PROBABILITY_RANDOM_INJECTIONS; // probably wrong

    std::vector<double> masses;
    masses.reserve(AMOUNT_OF_PARTICLES+1);
    for (const auto &particle: particles)
        masses.push_back(particle.weight);
    masses.push_back(injection_probability);
    std::discrete_distribution<int> sampler(masses.begin(), masses.end());
    std::array<Particle, AMOUNT_OF_PARTICLES> new_particles;
    for (int i = 0; i < AMOUNT_OF_PARTICLES; i++) {
	/* if (i < (double) AMOUNT_OF_PARTICLES * injection_probability) {
	    new_particles[i] = Particle::random(lhf);
            continue;
	}*/
        const int idx = sampler(gen);
        if (idx == AMOUNT_OF_PARTICLES)
            new_particles[i] = Particle::random(lhf);
        else
            new_particles[i] = particles[idx];
    }
    particles = new_particles;
}

void visualize_reference_rays(const ros::Publisher ray_pub,
                              const std::vector<geometry_msgs::Point> &reference_measurements, const geometry_msgs::Point &ref) {
    visualization_msgs::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = ros::Time::now();
    m.ns = "laser_rays";
    m.id = 0;
    m.type = visualization_msgs::Marker::LINE_LIST;
    m.action = visualization_msgs::Marker::ADD;

    // Each line segment is defined by two consecutive points in m.points:
    //   [start0, end0, start1, end1, start2, end2, ...]
    m.scale.x = 0.02; // line thickness (meters)

    // Color the rays blue (or pick any color you prefer)
    m.color.r = 0.0f;
    m.color.g = 0.0f;
    m.color.b = 1.0f;
    m.color.a = 1.0f;

    m.points.clear();
    m.points.reserve(reference_measurements.size() * 2);

    // For each measurement, draw a line from (0,0,0) to (px,py,0)
    geometry_msgs::Point start, end;
    start.x = -ref.y;
    start.y = -ref.x;
    start.z = 0.0;

    for (const auto &meas: reference_measurements) {
        end.x = -meas.y;
        end.y = -meas.x;
        end.z = 0; // usually 0, but copy whatever z came in

        m.points.push_back(start);
        m.points.push_back(end);
    }

    m.lifetime = ros::Duration(0.0); // latched until overwritten
    ray_pub.publish(m);
}

void viszualize_particles(
    const ros::Publisher &marker_pub,
    const std::array<Particle, AMOUNT_OF_PARTICLES> &particles) {
    visualization_msgs::MarkerArray markers;
    markers.markers.reserve(AMOUNT_OF_PARTICLES);

    for (size_t i = 0; i < particles.size(); ++i) {
        const auto &p = particles[i];

        visualization_msgs::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = ros::Time::now();
        m.ns = "particles";
        m.id = static_cast<int>(i);
        m.type = visualization_msgs::Marker::ARROW;
        m.action = visualization_msgs::Marker::ADD;

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
        m.scale.x = 0.05;   // arrow length in meters
        m.scale.y = 0.02;   // shaft diameter
        m.scale.z = 0.02;   // head diameter

        // --- COLOR (by weight) ---
        if (p.weight <= 0.0) {
            // red
            m.color.r = 1.0f;
            m.color.g = 0.0f;
            m.color.b = 0.0f;
            m.color.a = 1.0f;
        } else if (p.weight == 10000) {
            // green
            m.color.r = 0.0f;
            m.color.g = 1.0f;
            m.color.b = 0.0f;
            m.color.a = 1.0f;
        } else if (p.weight > 6000) {
            m.color.r = 0.0f;
            m.color.g = 0.0f;
            m.color.b = 1.0f;
            m.color.a = 1.0f;
        } else {
            // yellow
            m.color.r = 1.0f-p.weight;
            m.color.g = p.weight;
            m.color.b = 0.0f;
            m.color.a = 1.0f;
        }

        m.lifetime = ros::Duration(0.0);
        markers.markers.push_back(m);
    }

    marker_pub.publish(markers);
}


std::vector<int> approx_current_pos(double x, double y, double angle) {
    // Convert meters to centimeters and reflect

    // Convert x and y to int (row col)
    int col = static_cast<int>(-x * 100) / 80;
    int row = static_cast<int>(-y * 100) / 80;

    angle += PI;

    // Normalize angle to range [-PI, PI]
    while (angle > PI && ros::ok()) angle -= 2 * PI;
    while (angle <= -PI && ros::ok()) angle += 2 * PI;
    int final_orient = 4;

    if (angle > -PI / 4 && angle <= PI / 4) {
        final_orient = 0; // RIGHT
    } else if (angle > PI / 4 && angle <= 3 * PI / 4) {
        final_orient = 1; // UP
    } else if (angle > -3 * PI / 4 && angle <= -PI / 4) {
        final_orient = 3; // DOWN
    } else {
        final_orient = 2; // LEFT
    }

    return {col, row, final_orient};
}


geometry_msgs::Point get_particle_variance(const std::array<Particle, AMOUNT_OF_PARTICLES> &particles, geometry_msgs::Point &averages) {
    double sum_sin = 0;
    double sum_cos = 0;

    averages.x = 0.0;
    averages.y = 0.0;
    averages.z = 0.0;

    for (int i = 0; i < particles.size(); ++i) {
        const auto &p = particles[i];
        averages.x += p.position.x;
        averages.y += p.position.y;

        sum_cos += std::cos(p.position.theta);
        sum_sin += std::sin(p.position.theta);
    }
    averages.x /= particles.size();
    averages.y /= particles.size();
    averages.z = std::atan2(sum_cos, sum_sin);

    geometry_msgs::Point variance;
    variance.x = 0.0;
    variance.y = 0.0;
    variance.z = 0.0;

    for (int i = 0; i < particles.size(); ++i) {
        const auto &p = particles[i];
        variance.x += (p.position.x - averages.x) * (p.position.x - averages.x);
        variance.y += (p.position.y - averages.y) * (p.position.y - averages.y);
    }

    variance.x /= particles.size();
    variance.y /= particles.size();
    variance.z = 1 - std::sqrt(sum_sin * sum_sin + sum_cos * sum_cos) / particles.size();

    return variance;
}

int main(int argc, char **argv) {
    // ros::init(argc, argv, "localize");
    ros::init(argc, argv, "localize",
        ros::init_options::NoSigintHandler);
    initSignalHandler();
    ros::NodeHandle n;
    static ros::Publisher posearray_pub =
            n.advertise<visualization_msgs::MarkerArray>("particle_poses", 1, true);
    static ros::Publisher ray_pub =
            n.advertise<visualization_msgs::Marker>("reference_rays", 1, true);

    // Publisher for Pose messages
    static ros::Publisher pose_pub =
            n.advertise<silver_fundamentals::Pose>("pose", 10);

    auto rate = ros::Rate(10);
    const std::string pkg_path = "src/silver_fundamentals";
    std::string mapfile = pkg_path + "/maps/map.txt";

    auto driver = FeedbackDrive(3.25, 26.5, 2.0);
    driver.reset_encoders();
    const auto lhf = LikelihoodField(n, mapfile, SIGMA);

    ros::ServiceClient laser_pol_client = n.serviceClient<silver_fundamentals::Laser>("laserAngleRange");
    ros::ServiceClient drive_data_client = n.serviceClient<silver_fundamentals::DriveData>("encoder_data");
    ros::ServiceClient drive_client = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
    silver_fundamentals::DriveData encoder_srv;
    create_fundamentals::DiffDrive drive_srv;


    std::vector<geometry_msgs::Point> waypoints;
    int current_waypoint = 0;
    // init particles array
    std::array<Particle, AMOUNT_OF_PARTICLES> particles;
    for (int i = 0; i < AMOUNT_OF_PARTICLES; i++) {
        particles[i] = Particle::random(lhf); /* particles[i] = Particle::zero(); */
	// particles[i].position.y = -2.0;
	// particles[i].position.x = -0.4;
	// particles[i].position.theta = -PI/2.0;
    }
    double curr_right_encoder = encoder_srv.response.right_encoder;
    double curr_left_encoder = encoder_srv.response.left_encoder;
    unsigned int count = 0;

    // Prepare comm service client and request
    ros::ServiceClient comm_client = n.serviceClient<silver_fundamentals::Com>("comm");
    silver_fundamentals::Com comm_srv;
    comm_srv.request.operation = silver_fundamentals::Com::Request::GET_DATA;

    struct {
        double angle;
        direction dir;
        double dist;
        double orientation;
        direction orientation_dir;
        geometry_msgs::Point cell_center;
    } alignment;
    geometry_msgs::Pose2D current_position;



     auto localize_state = LocalizeState::LOCALISING;

    int localize_count = 0;
    int unlocalize_count = 0;
    bool first_execution = false;

    // main loop
    while (ros::ok()) {

        if (isShutdownRequested()) {
            drive_srv.request.left = 0;
            drive_srv.request.right = 0;
            drive_client.call(drive_srv);
            ROS_INFO("SIGINT: Stopped robot motors.");
            ROS_INFO("SIGINT: Shutting Down ...");
            ros::shutdown();
            break;
        }

        geometry_msgs::Point averages;
        geometry_msgs::Point variance = get_particle_variance(particles, averages);

        double max_weight = 0.0;
        Particle &best_particle = particles[0];

        for (int i = 0; i < particles.size(); ++i) {
            const auto &p = particles[i];
            if (p.weight > max_weight && p.weight < 2.0) {
                max_weight = p.weight;
                best_particle = p;
            }

        }

        current_position.x = averages.x;
        current_position.y = averages.y;
        current_position.theta = best_particle.position.theta;


        if (variance.x < LOCALIZE_VAR_LOWER && variance.y < LOCALIZE_VAR_LOWER && localize_state == LocalizeState::LOCALISING) {
            printf("within threshold, count is %d\n", localize_count);
            if (localize_count >= LOCALIZE_COUNT_THRESHOLD) {
                localize_state = LocalizeState::ALIGNING_ANGLE;
                // compute alignment
                geometry_msgs::Point new_pos;

                new_pos.x = -static_cast<double>(static_cast<int>(-current_position.x * 100) / 80 * 80)/100.0-0.4;
                new_pos.y = -static_cast<double>(static_cast<int>(-current_position.y * 100) / 80 * 80)/100.0-0.4;

                alignment.cell_center = new_pos;

                geometry_msgs::Point curr_as_point;
                curr_as_point.x = current_position.x;
                curr_as_point.y = current_position.y;
                curr_as_point.z = current_position.theta;
                //curr_as_point.z = particles[0].position.theta;
                auto goal = global_to_local(curr_as_point, new_pos);



                double angle = atan2(goal.x, goal.y);

                alignment.angle = std::abs(angle);
                alignment.dir = angle>0?left:right;
                alignment.dist = goal.z;
                alignment.orientation = std::abs(-current_position.theta-angle);
                alignment.orientation_dir = -current_position.theta-angle>0?left:right;

                driver.reset_encoder_base_lines();
                drive_srv.request.left = 0;
                drive_srv.request.right = 0;
                drive_client.call(drive_srv);

                printf("now localized at %f %f %f, aligning to %f %f with %f %f\n", current_position.x, current_position.y, current_position.theta, goal.x, goal.y, alignment.angle, alignment.dist);
            } else
                localize_count++;


        } else if (localize_state == LocalizeState::LOCALISING) {
            localize_count = 0;
        }
        if (localize_state != LocalizeState::LOCALISING && (variance.x > LOCALIZE_VAR_UPPER || variance.y > LOCALIZE_VAR_UPPER)) {
            if (unlocalize_count >= UNLOCALIZE_COUNT_THRESHOLD) {
                if (localize_state == LocalizeState::EXECUTING_PLAN)
                    localize_state = LocalizeState::EXECUTED_PLAN_FAIL;
                else
                    localize_state = LocalizeState::LOCALISING;
            } else
                unlocalize_count++;

        } else if (localize_state != LocalizeState::LOCALISING) {
            unlocalize_count = 0;
        }


        // do laser measurement
        std::vector<geometry_msgs::Point> reference_measurements = get_laser_rays(laser_pol_client);
        compute_weights(lhf, particles, reference_measurements);
        // visualize_reference_rays(ray_pub, reference_measurements);
	std::vector<geometry_msgs::Point> applied_measurements;
	    geometry_msgs::Point p;
	    p.x = averages.x;
	    p.y = averages.y;
	    p.z = current_position.theta;
	for (int i = 0; i < reference_measurements.size(); i++) {
		applied_measurements.push_back(local_to_global(p, reference_measurements[i]));
	}
	visualize_reference_rays(ray_pub, applied_measurements, p);
        viszualize_particles(posearray_pub, particles);
	

	double sum = 0;
	for (auto &particle: particles)
		sum += particle.weight;
	// ROS_INFO("Average confidence is %f", sum/ (double) AMOUNT_OF_PARTICLES);
        // do sampling
        //if (localize_state != LocalizeState::WAIT_FOR_PLAN)
            resample(lhf, particles);
        // do drive init
        while (!drive_data_client.call(encoder_srv) && ros::ok())
            ROS_ERROR("encoder service call failed");

        switch (localize_state) {
            case LocalizeState::LOCALISING: {
                // wandering

                while (!drive_data_client.call(encoder_srv) && ros::ok())
                    ROS_ERROR("encoder service call failed");

                if (count % 4 == 0) {
                    geometry_msgs::Point current, goal;
                    current.x = 0;
                    current.y = 0;
                    current.z = 0;
                    goal.x = 0;
                    goal.y = 1;
                    goal.z = 0;
                    const geometry_msgs::Point field_vector = driver.get_potentials(
                        current, goal, K_ATT, K_REP, NO_EFFECTION_POT_FIELDS);
                    double angle = std::atan2(field_vector.x, field_vector.y);
                    double rotation_rate = angle * ROT_RATE * BASE_SPEED;
                    drive_srv.request.left = BASE_SPEED - WHEEL_BASE / 2 * rotation_rate;
                    drive_srv.request.right = BASE_SPEED + WHEEL_BASE / 2 * rotation_rate;
                    drive_client.call(drive_srv);
                }
                break;
            }
            case LocalizeState::ALIGNING_ANGLE: {
                if (driver.turn_n_degrees_async(alignment.angle, alignment.dir) == true) {
                    localize_state = LocalizeState::ALIGNING_DRIVE;
                    driver.reset_encoder_base_lines();
                }
                break;
            }
            case LocalizeState::ALIGNING_DRIVE: {
                if (driver.drive_n_cm_async(alignment.dist) == true) {
                    localize_state = LocalizeState::WAIT_FOR_PLAN;
                    driver.reset_encoder_base_lines();
                    std::vector<int> pos = approx_current_pos(current_position.x, current_position.y, current_position.theta);
                    silver_fundamentals::playSong1(n); ROS_INFO("Published pose: row=%d, column=%d, orientation=%d",
                         pos[1], pos[0], pos[2]);
                }

                break;
            }
            case LocalizeState::ALIGNING_ANGLE_ORIENT: {
                if (driver.turn_n_degrees_async(alignment.orientation, alignment.orientation_dir) == true) {
                    localize_state = LocalizeState::WAIT_FOR_PLAN;
                }
                break;
            }
            case LocalizeState::WAIT_FOR_PLAN: {

                // convert current_pos to pose representation
                std::vector<int> pos = approx_current_pos(current_position.x, current_position.y, current_position.theta);

                // create and publish Pose message
                silver_fundamentals::Pose pose_msg;
                pose_msg.column = pos[0];
                pose_msg.row = pos[1];
                pose_msg.orientation = pos[2];
                pose_pub.publish(pose_msg);


                // wait for execute plan call
                comm_srv.request.operation = silver_fundamentals::Com::Request::GET_DATA;
                bool plan_exists_flag = false;
                while (!comm_client.call(comm_srv) && ros::ok())
                    ROS_ERROR("localize: failed to call comm service for GET_DATA");

                plan_exists_flag = comm_srv.response.plan_exists;
                if (plan_exists_flag == true) {
                    waypoints = comm_srv.response.waypoints;
                    for (auto &waypoint : waypoints) {
                        waypoint.x += alignment.cell_center.x;
                        waypoint.y += alignment.cell_center.y;
                    }
                    comm_srv.request.operation = silver_fundamentals::Com::Request::SET_DATA;
                    comm_srv.request.plan_exists = false;
                    comm_srv.request.success_state = static_cast<uint8_t>(silver_fundamentals::PlanSuccessState::NONE);

                    while (!comm_client.call(comm_srv) && ros::ok())
                        ROS_ERROR("localize: failed to call comm service for GET_DATA");

                    localize_state = LocalizeState::EXECUTING_PLAN;
                    first_execution = true;
                }
                break;
            }
            case LocalizeState::EXECUTING_PLAN: {
                while (!drive_data_client.call(encoder_srv) && ros::ok())
                    ROS_ERROR("encoder service call failed");
                if (count % 4 == 0) {
                    geometry_msgs::Point current, goal;
                    current.x = current_position.x;
                    current.y = current_position.y;
                    current.z = current_position.theta;
                    goal.x = waypoints[current_waypoint].x;
                    goal.y = waypoints[current_waypoint].y;
                    goal.z = waypoints[current_waypoint].z;
                    const geometry_msgs::Point field_vector = driver.get_potentials(
                        current, goal, K_ATT, K_REP, NO_EFFECTION_POT_FIELDS);
                    double angle = std::atan2(field_vector.x, field_vector.y);
                    double rotation_rate = angle * ROT_RATE * BASE_SPEED;
                    drive_srv.request.left = BASE_SPEED - WHEEL_BASE / 2 * rotation_rate;
                    drive_srv.request.right = BASE_SPEED + WHEEL_BASE / 2 * rotation_rate;
                    drive_client.call(drive_srv);
                    if (std::sqrt(
                            (current_position.x - goal.x) * (current_position.x - goal.x) + (
                                current_position.y - goal.y) * (current_position.y - goal.y)) <= goal.z) {
                        current_waypoint++;
                        if (waypoints.size() == current_waypoint) {
                            localize_state = LocalizeState::EXECUTED_PLAN_SUC;
                            current_waypoint = 0;
                        }
                    }
                    if (first_execution == true) {
                        first_execution = false;
                        ros::Duration(0.1).sleep();
                    }
                }

                std::vector<int> pos = approx_current_pos(current_position.x, current_position.y, current_position.theta);

                // create and publish Pose message
                silver_fundamentals::Pose pose_msg;
                pose_msg.column = pos[0];
                pose_msg.row = pos[1];
                pose_msg.orientation = pos[2];
                pose_pub.publish(pose_msg);
                ROS_INFO("Published pose: row=%d, column=%d, orientation=%d",
                         pose_msg.row, pose_msg.column, pose_msg.orientation);
                break;
            }
            case LocalizeState::EXECUTED_PLAN_FAIL: {
                // play failed sound
                silver_fundamentals::playSong4(n);
                // create empty waypoints vector;
                std::vector<geometry_msgs::Point> empty_waypoints = {};
                comm_srv.request.operation = silver_fundamentals::Com::Request::SET_DATA;
                comm_srv.request.success_state = static_cast<uint8_t>(
                    silver_fundamentals::PlanSuccessState::PLAN_FAILED);
                comm_srv.request.plan_exists = false;
                comm_srv.request.waypoints = empty_waypoints;

                // Call the service
                if (!comm_client.call(comm_srv)) {
                    ROS_ERROR("execute_plan_server: failed to call comm service for SET_DATA");
                }

                localize_state = LocalizeState::LOCALISING;
                break;
            }
            case LocalizeState::EXECUTED_PLAN_SUC: {
                std::vector<geometry_msgs::Point> empty_waypoints = {};
                comm_srv.request.operation = silver_fundamentals::Com::Request::SET_DATA;
                comm_srv.request.success_state = static_cast<uint8_t>(
                    silver_fundamentals::PlanSuccessState::PLAN_DONE);
                comm_srv.request.plan_exists = false;
                comm_srv.request.waypoints = empty_waypoints;

                // Call the service
                if (!comm_client.call(comm_srv)) {
                    ROS_ERROR("execute_plan_server: failed to call comm service for SET_DATA");
                }
                // TODO:
                localize_state = LocalizeState::LOCALISING;
		        localize_count = 0;
		        unlocalize_count = 0;
                current_waypoint = 0;
                break;
            }
            default:
                break;
        }
        // do sleep
        rate.sleep();
        // do odometry adjustment
        while (!drive_data_client.call(encoder_srv) && ros::ok())
            ROS_ERROR("encoder service call failed");

        double right_encoder_delta = encoder_srv.response.right_encoder - curr_right_encoder;
        double left_encoder_delta = encoder_srv.response.left_encoder - curr_left_encoder;
        curr_right_encoder = encoder_srv.response.right_encoder;
        curr_left_encoder = encoder_srv.response.left_encoder;

        particle_odometry_update(lhf, particles, right_encoder_delta, left_encoder_delta);


    // printf("Particle at %f %f heading %f %f\n", particles[50].position.x, particles[50].position.y, particles[50].position.theta*180.0/PI, particles[50].weight);
	count++;
    }
    drive_srv.request.left = 0;
    drive_srv.request.right = 0;
    drive_client.call(drive_srv);


    return 0;
}
