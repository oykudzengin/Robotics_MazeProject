#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <config.h>

#include <parse_map_file.h>

#define POW2(x) ((x)*(x))

void LikelihoodField::publish_low_res_walls(ros::NodeHandle &nh,
                                               const ros::Publisher &lowres_wall_pub,
                                               const std::vector<std::vector<unsigned int> > &lowres_map) const {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map"; // same frame you used for your OccupancyGrids
    marker.header.stamp = ros::Time::now();
    marker.ns = "lowres_walls";
    marker.id = 100; // any unique ID
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    // Set the thickness of each line (in meters). You can tweak this.
    marker.scale.x = 0.05; // 5 cm thick lines

    // Color: solid black (for example)
    std_msgs::ColorRGBA black;
    black.r = 0.0;
    black.g = 0.0;
    black.b = 0.0;
    black.a = 1.0;
    marker.color = black;

    // Now iterate over every low‐res cell (r, c). If a bit is set, draw the corresponding edge.
    //
    // Convention:
    //   Cell (c, r) covers world‐coords:
    //     x ∈ [ c * LOWRES_RESOLUTION,  (c+1) * LOWRES_RESOLUTION ]
    //     y ∈ [ r * LOWRES_RESOLUTION,  (r+1) * LOWRES_RESOLUTION ]
    //
    //   “TOP” edge is the horizontal segment at y = (r+1)*LOWRES_RESOLUTION,
    //   from x = c*LOWRES_RESOLUTION to x = (c+1)*LOWRES_RESOLUTION.
    //
    //   “RIGHT” edge is the vertical segment at x = (c+1)*LOWRES_RESOLUTION,
    //   from y = r*LOWRES_RESOLUTION to y = (r+1)*LOWRES_RESOLUTION.
    //
    //   “BOTTOM” edge is at y = r * LOWRES_RESOLUTION,
    //   from x = c*LOWRES_RESOLUTION to x = (c+1)*LOWRES_RESOLUTION.
    //
    //   “LEFT” edge is at x = c * LOWRES_RESOLUTION,
    //   from y = r*LOWRES_RESOLUTION to y = (r+1)*LOWRES_RESOLUTION.

    for (int r = 0; r < row_count; ++r) {
        for (int c = 0; c < col_count; ++c) {
            const unsigned int mask = lowres_map[r][c];
            if (mask == 0) continue; // no wall in this cell

            // Precompute the four “corners” of this cell in WORLD coordinates:
            const double x0 = c * 0.8;
            const double y0 = r * 0.8;
            const double x1 = (c + 1) * 0.8;
            const double y1 = (r + 1) * 0.8;

            geometry_msgs::Point p_start, p_end;

            // TOP edge?
            if (mask & BOTTOM) {
                // from (x0, y1) to (x1, y1)
                p_start.y = x0;
                p_start.x = y1;
                p_start.z = 0.0;
                p_end.y = x1;
                p_end.x = y1;
                p_end.z = 0.0;
                marker.points.push_back(p_start);
                marker.points.push_back(p_end);
            }

            // RIGHT edge?
            if (mask & RIGHT) {
                // from (x1, y0) to (x1, y1)
                p_start.y = x1;
                p_start.x = y0;
                p_start.z = 0.0;
                p_end.y = x1;
                p_end.x = y1;
                p_end.z = 0.0;
                marker.points.push_back(p_start);
                marker.points.push_back(p_end);
            }

            // BOTTOM edge?
            if (mask & TOP) {
                // from (x0, y0) to (x1, y0)
                p_start.y = x0;
                p_start.x = y0;
                p_start.z = 0.0;
                p_end.y = x1;
                p_end.x = y0;
                p_end.z = 0.0;
                marker.points.push_back(p_start);
                marker.points.push_back(p_end);
            }

            // LEFT edge?
            if (mask & LEFT) {
                // from (x0, y0) to (x0, y1)
                p_start.y = x0;
                p_start.x = y0;
                p_start.z = 0.0;
                p_end.y = x0;
                p_end.x = y1;
                p_end.z = 0.0;
                marker.points.push_back(p_start);
                marker.points.push_back(p_end);
            }
        }
    }

    // Now publish that single Marker. RViz will draw one line per pair of points.
    lowres_wall_pub.publish(marker);
}

void LikelihoodField::publish_high_res_walls(ros::NodeHandle &nh) const {
    // 1) Create the high‐res OccupancyGrid message
    nav_msgs::OccupancyGrid highres_grid;
    highres_grid.header.frame_id = "map";
    highres_grid.header.stamp    = ros::Time::now();

    // 2) Compute high‐res dimensions
    const int h_rows = row_count * cell_size + 2 * buffer_size;
    const int h_cols = col_count * cell_size + 2 * buffer_size;

    // 3) Set resolution so that 'cell_size' high‐res cells = 0.8 m (one low‐res cell)
    highres_grid.info.resolution = 0.8 / static_cast<double>(cell_size);
    highres_grid.info.width  = h_cols;
    highres_grid.info.height = h_rows;

    // 4) Place high‐res (buffer_size, buffer_size) at world (0,0)
    double resH = highres_grid.info.resolution;
    highres_grid.info.origin.position.x = -static_cast<double>(buffer_size) * resH;
    highres_grid.info.origin.position.y = -static_cast<double>(buffer_size) * resH;
    highres_grid.info.origin.position.z = 0.0;
    highres_grid.info.origin.orientation.w = 1.0;

    // 5) Resize data array
    highres_grid.data.resize(h_rows * h_cols);

    for (int r = 0; r < h_rows; ++r) {
        for (int c = 0; c < h_cols; ++c) {
            geometry_msgs::Point p;
            p.y = (double) (-c+buffer_size) / 100;
            p.x = (double) (-r+buffer_size) / 100;
            highres_grid.data[r * h_cols + c] = (int)(get_prob_field_value(p) * 100.0);
	    // printf("%d, coords %d %d  has value %d %f\n", r * h_cols + c, r, c, highres_grid.data[r * h_cols + c], get_prob_field_value(p));
        }
    }

    highres_pub.publish(highres_grid);
}


LikelihoodField::LikelihoodField(ros::NodeHandle &nh, const std::string &filename, const double sigma)
    : sigma_value(sigma) {
    lowres_pub = nh.advertise<visualization_msgs::Marker>("lowres_map", 1, true);
    highres_pub = nh.advertise<nav_msgs::OccupancyGrid>("highres_map", 1, true);



    std::vector<std::vector<unsigned int>> initial_map;
    parse_file_lowres(filename, initial_map);
    build_lookup_map(initial_map);

    publish_high_res_walls(nh);
    ros::Duration(0.5).sleep();
}
#pragma GCC optimize ("O0")
bool LikelihoodField::parse_file_lowres(const std::string &filename, std::vector<std::vector<unsigned int> > &map) {

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
void LikelihoodField::build_lookup_map(const std::vector<std::vector<unsigned int> > &map) {
    row_count = map.size();
    col_count = map[0].size();

    const int lookup_grid_row_count = row_count * cell_size + 2 * buffer_size;
    const int lookup_grid_col_count = col_count * cell_size + 2 * buffer_size;

    field.assign(lookup_grid_row_count, std::vector<double>(lookup_grid_col_count, 0));
    dist_field.assign(lookup_grid_row_count, std::vector<double>(lookup_grid_col_count, 0));

    ROS_INFO("got %d rows, %d cols, translating to highres %d rows %d cols\n", row_count, col_count,
             lookup_grid_row_count, lookup_grid_col_count);

    for (int current_row = 0; current_row < lookup_grid_row_count; current_row++) {
        for (int current_col = 0; current_col < lookup_grid_col_count; current_col++) {
            double closest_wall_dist = std::numeric_limits<double>::infinity();
            if (current_row < buffer_size && current_col < buffer_size) {
                /* upper left corner */
                closest_wall_dist = std::sqrt(POW2(buffer_size - current_row) + POW2(buffer_size - current_col));
            } else if (current_row < buffer_size && current_col + buffer_size >= lookup_grid_col_count)
                /* upper right corner */
                closest_wall_dist = std::sqrt(
                    POW2(buffer_size - current_row) + POW2(buffer_size - (lookup_grid_col_count-1-current_col)));
            else if (current_row < buffer_size)
                /* top side */
                closest_wall_dist = buffer_size - current_row;
            else if (current_row + buffer_size >= lookup_grid_row_count && current_col < buffer_size)
                /* bottom left corner */
                closest_wall_dist = std::sqrt(
                    POW2(buffer_size - (lookup_grid_row_count - 1 - current_row)) + POW2(buffer_size - current_col));
            else if (current_row + buffer_size >= lookup_grid_row_count &&
                     current_col + buffer_size >= lookup_grid_col_count)
                /* bottom right corner */
                closest_wall_dist = std::sqrt(
                    POW2(buffer_size - (lookup_grid_row_count - 1 - current_row)) + POW2(buffer_size - (lookup_grid_col_count - 1 - current_col)));
            else if (current_row + buffer_size >= lookup_grid_row_count)
                /* bottom side */
                closest_wall_dist = buffer_size - (lookup_grid_row_count - 1 - current_row);
            else if (current_col < buffer_size)
                /* left side */
                closest_wall_dist = buffer_size - current_col;
            else if (current_col + buffer_size >= lookup_grid_col_count)
            /* right side */
                closest_wall_dist = buffer_size - (lookup_grid_col_count - 1 - current_col);
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
            const double dist2 = closest_wall_dist * closest_wall_dist;
            //printf("%f %f has %f\n", global_space_point.y, global_space_point.x, dist2);
            field[current_row][current_col] = std::max(0.1,std::exp(-dist2 / (2 * sigma_value * sigma_value)));
            dist_field[current_row][current_col] = closest_wall_dist;
        }
    }
}

double LikelihoodField::get_field_value(const geometry_msgs::Point &global_space_point) const {
    const int real_y = -(global_space_point.y * 100) + buffer_size;
    const int real_x = -(global_space_point.x * 100) + buffer_size;
    // printf("%f %f gets looked up at %d %d\n", global_space_point.y, global_space_point.x,real_y, real_x);
    if (real_x < 0 || real_y < 0 || real_x >= cell_size * col_count + 2 * buffer_size || real_y >= cell_size *
        row_count + 2 * buffer_size)
        return std::numeric_limits<double>::infinity();
    return dist_field[real_y][real_x];
}

double LikelihoodField::get_prob_field_value(const geometry_msgs::Point &global_space_point) const {
    /*
     const double dist2 = get_field_value(global_space_point) * get_field_value(global_space_point);
    //printf("%f %f has %f\n", global_space_point.y, global_space_point.x, dist2);
    return std::exp(-dist2 / (2 * sigma_value * sigma_value));
    */
    const int real_y = -(global_space_point.y * 100) + buffer_size;
    const int real_x = -(global_space_point.x * 100) + buffer_size;
    if (real_x < 0 || real_y < 0 || real_x >= cell_size * col_count + 2 * buffer_size || real_y >= cell_size * row_count + 2 * buffer_size)
        return 0.0;
    return field[real_y][real_x];
}
