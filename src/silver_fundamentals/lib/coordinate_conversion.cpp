#include <coordinate_conversion.h>
#include <cmath>

geometry_msgs::Point global_to_local(geometry_msgs::Point &relative_center, geometry_msgs::Point &to_convert) {
    double wx = to_convert.x - relative_center.x;
    double wy = to_convert.y - relative_center.y;

    // convert to local frame
    double yaw = relative_center.z;
    double c = std::cos(yaw);
    double s = std::sin(yaw);

    geometry_msgs::Point local_pos;

    local_pos.x = c*wx - s*wy;
    local_pos.y = s*wx + c*wy;
    local_pos.z = std::hypot(local_pos.x, local_pos.y);

    return local_pos;
}



geometry_msgs::Point local_to_global(geometry_msgs::Point &relative_center, geometry_msgs::Point &to_convert) {
    double lx = to_convert.x;
    double ly = to_convert.y;

    double yaw = relative_center.z;
    double c = std::cos(yaw);
    double s = std::sin(yaw);

    double wx = c*lx + s*ly;
    double wy = -s*lx + c*ly;

    geometry_msgs::Point global_pos;
    global_pos.x = wx + relative_center.x;
    global_pos.y = wy + relative_center.y;
    global_pos.z = 0;

    return global_pos;
}

geometry_msgs::Point local_to_global(geometry_msgs::Pose2D &relative_center, geometry_msgs::Point &to_convert) {
    geometry_msgs::Point relative_center_interally;
    relative_center_interally.x = relative_center.x;
    relative_center_interally.y = relative_center.y;
    relative_center_interally.z = relative_center.theta;
    return local_to_global(relative_center_interally, to_convert);
}
