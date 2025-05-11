#include <ransac.h>
#include "ros/ros.h"
#include "silver_fundamentals/LaserCartesian.h"
#include "silver_fundamentals/Laser.h"
#include <config.h>
#include <random>


double minIgnoringNaN(const std::vector<double> &v) {
    double best = std::numeric_limits<double>::infinity();
    bool gotOne = false;

    for (double x: v) {
        if (std::isnan(x)) continue;
        gotOne = true;
        best = std::min(best, x);
    }
    return gotOne
               ? best
               : std::numeric_limits<double>::quiet_NaN();
}

double ransac(std::vector<geometry_msgs::Point> &pts, const double max_offset, const int max_iterations, int *amount) {
    if (pts.size() < 2) {
        *amount = 0;
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

        double a = dy / norm;
        double b = dx / norm;
        double c = dx * P.y - dy * P.x;


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
        std::numeric_limits<double>::infinity();
    }


    pts.erase(std::remove_if(pts.begin(), pts.end(), [&](const geometry_msgs::Point &pt) {
        return std::fabs(best_a * pt.x + best_b * pt.y + best_c) <= max_offset;
    }), pts.end());


    // compute line angle
    double x0 = -best_a * best_c;
    double y0 = -best_b * best_c;

    double angle_rad = std::atan2(y0, x0);
    double angle_deg = angle_rad * 180.0 / M_PI;

    *amount = best_count;
    return -angle_deg;
}