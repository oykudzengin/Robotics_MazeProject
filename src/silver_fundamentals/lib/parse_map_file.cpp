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
#include <geometry_msgs/Pose2D.h>

// Navigation Graph & Lookup Map
// -----------------------

#include <queue>
#include <map>
#include <vector>
#include <utility>

// Alias for a grid cell (row, col)
using Cell = std::pair<int, int>;

// Cell wall bit definitions (must match LikelihoodField private enum)
static constexpr unsigned int CW_TOP    = 1u << 0;
static constexpr unsigned int CW_RIGHT  = 1u << 1;
static constexpr unsigned int CW_BOTTOM = 1u << 2;
static constexpr unsigned int CW_LEFT   = 1u << 3;

// Directions in the low-res map: bit mask, row delta, col delta
struct Dir { unsigned int bit; int dr, dc; };
static const Dir DIRS_LOWRES[] = {
    { CW_TOP,    -1,  0 },
    { CW_BOTTOM,  1,  0 },
    { CW_LEFT,     0, -1 },
    { CW_RIGHT,    0,  1 },
};

// Forward declaration for lookup map builder
std::map<Cell, std::map<Cell, std::vector<Cell>>> buildLookupMapGrid(const std::vector<std::vector<unsigned int>>& map);

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
    build_wall_tables(initial_map);

    publish_high_res_walls(nh);
    publish_low_res_walls(nh, lowres_pub, initial_map);
    ros::Duration(2).sleep();
}

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
                case ' ':
                    [[fallthrough]];
                case ',':
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
    // Tabelle der kürzesten Pfade aufbauen
    lookup_map_ = buildLookupMapGrid(map);
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

// Liefert kürzesten Pfad als Points (x=row, y=col, z=0.4)
std::vector<geometry_msgs::Point>
LikelihoodField::getPath(int start_r, int start_c, int target_r, int target_c) const
{
    using Cell = std::pair<int,int>;
    Cell start{start_r, start_c}, target{target_r, target_c};
    std::vector<geometry_msgs::Point> result;
    auto it_start = lookup_map_.find(start);
    if (it_start == lookup_map_.end()) return result;
    auto it_target = it_start->second.find(target);
    if (it_target == it_start->second.end()) return result;
    for (const Cell &cell : it_target->second) {
        geometry_msgs::Point p;
        p.x = cell.first;   // row
        p.y = cell.second;  // col
        p.z = 0.4;          // feste Radius-Angabe
        result.push_back(p);
    }
    return result;
}

void LikelihoodField::build_wall_tables(const std::vector<std::vector<unsigned int>> &map) {
    horizontal_walls.clear();
    vertical_walls.clear();

    const double L = cell_size / 100.0;

    //
    // 1) Collect every single‐cell segment
    //
    for (int r = 0; r < row_count; ++r) {
        double y0 = r   * L;
        double y1 = (r+1)* L;
        for (int c = 0; c < col_count; ++c) {
            double x0 =  c   * L;
            double x1 = (c+1)* L;
            unsigned m = map[r][c];
            if (m & TOP)    horizontal_walls .push_back({ y0, x0, x1 });
            if (m & BOTTOM) horizontal_walls .push_back({ y1, x0, x1 });
            if (m & LEFT)   vertical_walls   .push_back({ x0, y0, y1 });
            if (m & RIGHT)  vertical_walls   .push_back({ x1, y0, y1 });
        }
    }

    //
    // 2) Merge any contiguous, collinear segments
    //
    auto merge_in_place = [&](std::vector<CellWallLine> &segs) {
        if (segs.empty()) return;
        // Sort by the constant coord, then by start
        std::sort(segs.begin(), segs.end(),
            [](auto &A, auto &B){
                if (A.c != B.c) return A.c < B.c;
                return A.start < B.start;
            });

        std::vector<CellWallLine> merged;
        merged.reserve(segs.size());

        CellWallLine cur = segs[0];
        for (size_t i = 1; i < segs.size(); ++i) {
            const auto &s = segs[i];
            // same line and overlapping/touching?
            if (std::abs(s.c - cur.c) < 1e-9 && s.start <= cur.end + 1e-9) {
                // extend
                cur.end = std::max(cur.end, s.end);
            }
            else {
                // push & start a new one
                merged.push_back(cur);
                cur = s;
            }
        }
        merged.push_back(cur);
        segs.swap(merged);
    };

    merge_in_place(horizontal_walls);
    merge_in_place(vertical_walls);

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

double LikelihoodField::get_ray_wall_dist(const geometry_msgs::Point &start_point, const geometry_msgs::Point &end_point) const {
    const double ox = -start_point.x;
    const double oy = -start_point.y;

    const double vx = -end_point.x - ox;
    const double vy = -end_point.y - oy;

    const double len = std::hypot(vx, vy);
    const double dx = vx / len;
    const double dy = vy / len;

    const double max_range = 1;

    double best_t = max_range;

    if (std::abs(dx) > 1e-6) {
        for (const auto &w : vertical_walls) {
            double t = (w.c - ox) / dx;
            if (t <= 0.0 || t >= best_t) continue;
            double y_hit = oy + t * dy;
            if (y_hit >= w.start && y_hit <= w.end) {
                best_t = t;
            }
        }
    }

    if (std::abs(dy) > 1e-6) {
        for (const auto &w : horizontal_walls) {
            double t = (w.c - oy) / dy;
            if (t <= 0.0 || t >= best_t) continue;
            double x_hit = ox + t * dx;
            if (x_hit >= w.start && x_hit <= w.end) {
                best_t = t;
            }
        }
    }

    return best_t;

}
double LikelihoodField::get_ray_wall_dist(const geometry_msgs::Pose2D &start_point, const geometry_msgs::Point &end_point) const {
    const double ox = -start_point.x;
    const double oy = -start_point.y;

    const double vx = -end_point.x - ox;
    const double vy = -end_point.y - oy;

    const double len = std::hypot(vx, vy);
    const double dx = vx / len;
    const double dy = vy / len;

    const double max_range = 1;

    double best_t = max_range;

    if (std::abs(dx) > 1e-6) {
        for (const auto &w : vertical_walls) {
            double t = (w.c - ox) / dx;
            if (t <= 0.0 || t >= best_t) continue;
            double y_hit = oy + t * dy;
            if (y_hit >= w.start && y_hit <= w.end) {
                best_t = t;
            }
        }
    }

    if (std::abs(dy) > 1e-6) {
        for (const auto &w : horizontal_walls) {
            double t = (w.c - oy) / dy;
            if (t <= 0.0 || t >= best_t) continue;
            double x_hit = ox + t * dx;
            if (x_hit >= w.start && x_hit <= w.end) {
                best_t = t;
            }
        }
    }

    return best_t;

}



/**
 * @brief Build an adjacency list graph from a low-res map.
 * @param map Vector of rows×cols cell masks.
 * @return map from each Cell to its list of neighbor Cells.
 */
std::map<Cell, std::vector<Cell>> buildGridGraph(const std::vector<std::vector<unsigned int>>& map) {
    std::map<Cell, std::vector<Cell>> graph;
    int rows = map.size();
    int cols = map.empty() ? 0 : map[0].size();
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            Cell node{r, c};
            std::vector<Cell> neighbors;
            for (const auto& d : DIRS_LOWRES) {
                int nr = r + d.dr;
                int nc = c + d.dc;
                // Check bounds
                if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue;
                // If there's no wall in this direction, add neighbor
                if ((map[r][c] & d.bit) == 0) {
                    neighbors.emplace_back(nr, nc);
                }
            }
            graph[node] = std::move(neighbors);
        }
    }
    return graph;
}

/**
 * @brief Perform BFS from a start cell to compute shortest paths to all reachable cells.
 * @param graph Adjacency list of the grid graph.
 * @param start Starting cell.
 * @return map from each reachable Cell to the vector of Cells representing the shortest path.
 */
std::map<Cell, std::vector<Cell>> bfsAllPaths(
    const std::map<Cell, std::vector<Cell>>& graph,
    const Cell& start)
{
    std::map<Cell, std::vector<Cell>> paths;
    std::queue<Cell> queue;
    paths[start] = { start };
    queue.push(start);

    while (!queue.empty()) {
        Cell current = queue.front();
        queue.pop();
        for (const Cell& nbr : graph.at(current)) {
            if (paths.find(nbr) == paths.end()) {
                // Extend path
                std::vector<Cell> newPath = paths[current];
                newPath.push_back(nbr);
                paths[nbr] = std::move(newPath);
                queue.push(nbr);
            }
        }
    }
    return paths;
}

/**
 * @brief Build a full lookup map of shortest paths between all pairs of cells.
 * @param map Vector of rows×cols cell masks.
 * @return Nested map: lookup[start][target] = shortest-path vector of Cells.
 */
std::map<Cell, std::map<Cell, std::vector<Cell>>> buildLookupMapGrid(
    const std::vector<std::vector<unsigned int>>& map)
{
    auto graph = buildGridGraph(map);
    std::map<Cell, std::map<Cell, std::vector<Cell>>> lookup;
    for (const auto& kv : graph) {
        lookup[kv.first] = bfsAllPaths(graph, kv.first);
    }
    return lookup;
}

// Example usage (append after parsing low-res map):
//
// std::vector<std::vector<unsigned int>> lowres_map;
// parse_file_lowres(filename, lowres_map);
// auto lookup = buildLookupMapGrid(lowres_map);
// Now lookup[{r1,c1}][{r2,c2}] gives the shortest path.