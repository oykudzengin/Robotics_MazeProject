#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <geometry_msgs/Point.h>
#include <config.h>

#include <parse_map_file.h>

#define POW2(x) ((x)*(x))

LikelihoodField::LikelihoodField(std::string filename, double sigma) {
    sigma_value = sigma;
    std::vector<std::vector<unsigned int>> initial_map;
    parse_file_lowres(filename, initial_map);
    build_lookup_map(initial_map);
}

bool LikelihoodField::parse_file_lowres(const std::string &filename, std::vector <std::vector<unsigned int>> &map) {
    std::ifstream in(filename);
    if (!in.is_open()) {
        std::cerr << "Could not open file. " << filename << "\n";
        return false;
    }
    std::stringstream buffer;
    buffer << in.rdbuf();
    std::string content = buffer.str();
    in.close();

    map.clear();

    int depth = 0;
    std::vector<unsigned int> current_row;
    unsigned int current_cell_mask;

    for (unsigned int i = 0; i < content.size(); i++) {
        char c = content[i];

        if (c == '[') {
            depth++;
            if (depth == 2)
                current_row.clear();
            else if (depth == 3)
                current_cell_mask = 0;
        } else if (c == ']') {
            if (depth == 3)
                current_row.push_back(current_cell_mask);
            else if (depth == 2)
                map.push_back(current_row);
            depth--;
        } else if (depth == 3) {
            switch (c) {
                case 'T':
                    current_cell_mask |= TOP;
                    break;
                case 'R':
                    current_cell_mask |= RIGHT;
                    break;
                case 'B':
                    current_cell_mask |= BOTTOM;
                    break;
                case 'L':
                    current_cell_mask |= LEFT;
                    break;
                default:
                    __builtin_unreachable();
                    break;
            }
        }
    }
    return true;
}

#pragma GCC optimize ("O3")
void LikelihoodField::build_lookup_map(const std::vector <std::vector<unsigned int>> &map) {

    row_count = map.size();
    col_count = map[0].size();

    const int lookup_grid_row_count = row_count * cell_size + 2 * buffer_size;
    const int lookup_grid_col_count = col_count * cell_size + 2 * buffer_size;

    field.assign(lookup_grid_row_count, std::vector<double>(lookup_grid_col_count, 0));

    ROS_INFO("got %d rows, %d cols, translating to highres %d rows %d cols\n", row_count, col_count, lookup_grid_row_count, lookup_grid_col_count);

    for (int current_row = 0; current_row < lookup_grid_row_count; current_row++) {
        for (int current_col = 0; current_col < lookup_grid_col_count; current_col++) {
            double closest_wall_dist = std::numeric_limits<double>::infinity();
            if (current_row < buffer_size && current_col < buffer_size) {
                /* upper left corner */
                closest_wall_dist = std::sqrt(POW2(buffer_size - current_row) + POW2(buffer_size - current_col));
            }
            else if (current_row < buffer_size && current_col + buffer_size >= lookup_grid_col_count)
                /* upper right corner */
                closest_wall_dist = std::sqrt(
                        POW2(buffer_size - current_row) + POW2(lookup_grid_col_count - 1 - current_col));
            else if (current_row < buffer_size)
                /* top side */
                closest_wall_dist = buffer_size - current_row;
            else if (current_row + buffer_size >= lookup_grid_row_count && current_col < buffer_size)
                /* bottom left corner */
                closest_wall_dist = std::sqrt(
                        POW2(lookup_grid_row_count - 1 - current_row) + POW2(buffer_size - current_col));
            else if (current_row + buffer_size >= lookup_grid_row_count &&
                     current_col + buffer_size >= lookup_grid_col_count)
                /* bottom right corner */
                closest_wall_dist = std::sqrt(
                        POW2(lookup_grid_row_count - 1 - current_row) + POW2(lookup_grid_col_count - 1 - current_col));
            else if (current_row + buffer_size >= lookup_grid_row_count)
                /* bottom side */
                closest_wall_dist = lookup_grid_row_count - 1 - current_row;
            else if (current_col < buffer_size)
                /* left side */
                closest_wall_dist = buffer_size - current_col;
            else if (current_col + buffer_size >= lookup_grid_col_count)
                /* right side */
                closest_wall_dist = lookup_grid_col_count - 1 - current_col;
            else {
                /* inside some cell */
                const int low_res_row = (current_row - buffer_size) / cell_size;
                const int low_res_col = (current_col - buffer_size) / cell_size;

                double l_row = (current_row - buffer_size) % cell_size;
                double l_col = (current_col - buffer_size) % cell_size;

                const int current_mask = map[low_res_row][low_res_col];

                // printf("%d, %d has coords %d %d, %f %f, mask %d\n", current_row, current_col, low_res_row, low_res_col, l_row, l_col, current_mask);

                closest_wall_dist = std::numeric_limits<double>::infinity();

                /* cell has at least one wall */
                if (current_mask & TOP)
                    closest_wall_dist = std::min(closest_wall_dist, l_row);
                if (current_mask & RIGHT)
                    closest_wall_dist = std::min(closest_wall_dist, cell_size - 1 - l_col);
                if (current_mask & BOTTOM)
                    closest_wall_dist = std::min(closest_wall_dist, cell_size - 1 - l_row);
                if (current_mask & LEFT)
                    closest_wall_dist = std::min(closest_wall_dist, l_col);

                /* cell might have adjacent corners */
                if (low_res_row == 0 || low_res_col == 0 ||
                    (map[low_res_row - 1][low_res_col - 1] & BOTTOM) != 0 ||
                    (map[low_res_row - 1][low_res_col - 1] & RIGHT) != 0)
                    /* top left corner exists */
                    closest_wall_dist = std::min(closest_wall_dist, std::sqrt(POW2(l_row) + POW2(l_col)));
                if (low_res_row == 0 || low_res_col == col_count - 1 ||
                    (map[low_res_row - 1][low_res_col + 1] & BOTTOM) != 0 ||
                    (map[low_res_row - 1][low_res_col + 1] & LEFT) != 0)
                    /* top right corner exists */
                    closest_wall_dist = std::min(closest_wall_dist,
                                                 std::sqrt(POW2(l_row) + POW2(cell_size - 1 - l_col)));
                if (low_res_row == row_count - 1 || low_res_col == 0 ||
                    (map[low_res_row + 1][low_res_col - 1] & TOP) != 0 ||
                    (map[low_res_row + 1][low_res_col - 1] & RIGHT) != 0)
                    /* bottom left corner exists */
                    closest_wall_dist = std::min(closest_wall_dist,
                                                 std::sqrt(POW2(cell_size - 1 - l_row) + POW2(l_col)));
                if (low_res_row == row_count - 1 || low_res_col == col_count - 1 ||
                    (map[low_res_row + 1][low_res_col + 1] & TOP) != 0 ||
                    (map[low_res_row + 1][low_res_col + 1] & LEFT) != 0)
                    /* bottom right corner exists */
                    closest_wall_dist = std::min(closest_wall_dist, std::sqrt(
                            POW2(cell_size - 1 - l_row) + POW2(cell_size - 1 - l_col)));
            }

            /* closest_wall_dist is now either set if wall were there or infinity if not */
            if (closest_wall_dist > 100)
                continue;
            field[current_row][current_col] = closest_wall_dist;

        }
    }
}

double LikelihoodField::get_field_value(const geometry_msgs::Point &global_space_point) const {
    const int real_y = -(global_space_point.y*100) + buffer_size;
    const int real_x = -(global_space_point.x*100) + buffer_size;
    // printf("%f %f gets looked up at %d %d\n", global_space_point.y, global_space_point.x,real_y, real_x);
    if (real_x < 0 || real_y < 0 || real_x > cell_size*col_count+2*buffer_size || real_y > cell_size*row_count+2*buffer_size)
        return std::numeric_limits<double>::infinity();
    return field[real_y][real_x];
}
double LikelihoodField::get_prob_field_value(const geometry_msgs::Point &global_space_point) const {
    const double dist2 = get_field_value(global_space_point)*get_field_value(global_space_point);
    return std::exp(-dist2/(2*sigma_value*sigma_value));
}
