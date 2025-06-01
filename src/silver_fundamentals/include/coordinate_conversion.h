#ifndef ROBITICSFUNDAMENTALSSILVER_COORDINATE_CONVERSION_H
#define ROBITICSFUNDAMENTALSSILVER_COORDINATE_CONVERSION_H
#include <geometry_msgs/Point.h>

// these both assume to be x to be right to left, y to be forward-backward and theta the angle from y axis, right being positive
geometry_msgs::Point global_to_local(geometry_msgs::Point &relative_center, geometry_msgs::Point &to_convert);
geometry_msgs::Point local_to_global(geometry_msgs::Point &relative_center, geometry_msgs::Point &to_convert);

#endif //ROBITICSFUNDAMENTALSSILVER_COORDINATE_CONVERSION_H
