#ifndef LASER_DISTANCE_MAP_H
#define LASER_DISTANCE_MAP_H
#include <array>
#include <cmath>
#include <config.h>

#define FRONT_THRESHOLD 0.5f
#define SIDE_THRESHOLD 0.5f
#define SIDE_ANGLE 90

#define N_LIDAR 682
#define ANGLE_SPAN (4.178563637658954*180/PI)
#define ANGLE_MIN (-2.086213869974017*180/PI)
#define ANGLE_MAX (ANGLE_MIN + ANGLE_SPAN)
#define ANGLE_STEP (0.006135923322290182*180/PI)

inline constexpr double threshold_at_angle(unsigned int angle) {
  return (angle >= -SIDE_ANGLE && angle <= SIDE_ANGLE)? FRONT_THRESHOLD + (SIDE_THRESHOLD - FRONT_THRESHOLD)*(angle/SIDE_ANGLE)*(angle/SIDE_ANGLE) : 0.0f;
}

constexpr double threshold_at_index(unsigned int index) {
  return threshold_at_angle(ANGLE_MIN + index*ANGLE_STEP);
}

extern const std::array<double, N_LIDAR> threshold_table;

#endif
