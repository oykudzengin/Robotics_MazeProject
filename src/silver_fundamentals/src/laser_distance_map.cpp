#include <laser_distance_map.h>

const std::array<double, N_LIDAR> threshold_table = [](){
    std::array<double, N_LIDAR> tbl{};
    for (auto i = 0; i < N_LIDAR; i++) {
      tbl[i] = threshold_at_index(i);
    }
    return tbl;
}();