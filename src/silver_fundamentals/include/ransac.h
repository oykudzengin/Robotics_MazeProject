#ifndef RANSAC_H
#define RANSAC_H
#include <vector>
#include <geometry_msgs/Point.h>

double ransac(std::vector<geometry_msgs::Point> &pts, const double max_offset, const int max_iterations, int *amount);

#endif