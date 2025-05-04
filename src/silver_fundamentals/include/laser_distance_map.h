#ifndef LASER_DISTANCE_MAP_H
#define LASER_DISTANCE_MAP_H
#include <array>
#include <cmath>
#include <config.h>

#define FRONT_THRESHOLD 0.140f
#define SIDE_THRESHOLD 0.27f
#define SIDE_ANGLE 120



inline constexpr double threshold_at_angle(double angle) {
  return (angle >= -SIDE_ANGLE && angle <= SIDE_ANGLE)? FRONT_THRESHOLD + (SIDE_THRESHOLD - FRONT_THRESHOLD)*(angle/SIDE_ANGLE)*(angle/SIDE_ANGLE) : 0.0f;
}

constexpr double threshold_at_index(unsigned int index) {
  return threshold_at_angle(ANGLE_MIN + index*ANGLE_STEP);
}

extern const std::array<double, LIDAR_POINTS> threshold_table;

#endif
