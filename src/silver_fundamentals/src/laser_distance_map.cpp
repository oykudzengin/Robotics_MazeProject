#include <laser_distance_map.h>

const std::array<double, LIDAR_POINTS> threshold_table = [](){
    std::array<double, LIDAR_POINTS> tbl{};
    for (auto i = 0; i < LIDAR_POINTS; i++) {
      tbl[i] = threshold_at_index(i);
    }
    return tbl;
}();