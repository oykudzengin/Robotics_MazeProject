#include <ransac.h>
#include "ros/ros.h"
#include "silver_fundamentals/LaserCartesian.h"
#include "silver_fundamentals/Laser.h"
#include <config.h>
#include <random>
#include <ransac.h>



double ransac_with_dist(std::vector<geometry_msgs::Point> &pts, const double max_offset, const int max_iterations, int *amount, double *dist) {
    if (pts.size() < 2) {
        *amount = 0;
        *dist   = std::numeric_limits<double>::infinity();
        return std::numeric_limits<double>::infinity();
    }


    std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<unsigned int> uni(0, pts.size() - 1);

    unsigned int best_count = 0;
    double best_a = 0, best_b = 0, best_c = 0;

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
        if (norm == 0.0) { --iter; continue; }

        double a = dy / norm;
        double b = dx / norm;
        double c = (dx * P.y - dy * P.x) / norm;


        unsigned int inliners = 0;

        for (const auto &pt: pts) {
            const double dist_to_line = std::fabs(a * pt.x + b * pt.y + c);
            if (dist_to_line <= max_offset)
                inliners++;
        }

        if (inliners > best_count) {
            best_count = inliners;
            best_a = a;
            best_b = b;
            best_c = c;
        }
    }

    if (best_count < 2) {
        *amount = 0;
        *dist   = std::numeric_limits<double>::infinity();
        return std::numeric_limits<double>::infinity();
    }


    pts.erase(std::remove_if(pts.begin(), pts.end(), [&](const geometry_msgs::Point &pt) {
        return std::fabs(best_a * pt.x + best_b * pt.y + best_c) <= max_offset;
    }), pts.end());


    // compute line angle
    double x0 = -best_a * best_c;
    double y0 = -best_b * best_c;

    double angle_rad = std::atan2(y0, x0);
    double angle_deg = angle_rad * 180.0 / M_PI;

    double d0 = std::fabs(best_c);

    *amount = best_count;
    *dist = d0;
    return -angle_deg;
}



double new_ransac(std::vector<geometry_msgs::Point> &pts, const double max_offset, const int max_iterations, int *inliers_used, double *dist) {
    if (pts.size() < 2) {
        *inliers_used = 0;
        *dist   = std::numeric_limits<double>::infinity();
        return std::numeric_limits<double>::infinity();
    }

    std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<unsigned int> uni(0, pts.size() - 1);

    unsigned int best_count = 0;
    double best_a = 0, best_b = 0, best_c = 0;

    for (int i = 0; i < max_iterations; i++) {
        const unsigned int first_idx = uni(rng);
        const unsigned int second_idx = uni(rng);

        if (first_idx == second_idx) {
            continue;
        }

        auto &P = pts[first_idx], &Q = pts[second_idx];
        double a = Q.y - P.y;
        double b = P.x - Q.x;
        double c = -a*P.x + b*P.y;

        unsigned int inliers = 0;
        for (const auto &pt: pts) {
            const double distance_to_line = std::fabs((a * pt.x + b * pt.y + c)) / (std::sqrt(a * a + b * b));
            if (distance_to_line <= max_offset) {
                inliers++;
            }
        }
        if (inliers > best_count) {
            best_count = inliers;
            best_a = a;
            best_b = b;
            best_c = c;
        }
    }

    if (best_count < 2) {
        *inliers_used = 0;
        *dist   = std::numeric_limits<double>::infinity();
        return std::numeric_limits<double>::infinity();
    }

    pts.erase(std::remove_if(pts.begin(), pts.end(), [&](const geometry_msgs::Point &pt) {
        return std::fabs((best_a * pt.x + best_b * pt.y + best_c)) / std::sqrt(best_a * best_a + best_b * best_b) <= max_offset;
    }), pts.end());

    double theta_rad = std::atan2(best_a < 0? -best_a:best_a, best_a < 0? best_b : -best_b);
    double theta_deg = theta_rad * 180.0 / PI;

    *dist = std::fabs(best_c/std::sqrt(best_a * best_a + best_b * best_b));

    return theta_deg;

}

double ransac(std::vector<geometry_msgs::Point> &pts, const double max_offset, const int max_iterations, int *amount) {
    double unused;
    return ransac_with_dist(pts, max_offset, max_iterations, amount, &unused);
}

