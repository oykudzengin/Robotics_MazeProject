#ifndef ROBITICSFUNDAMENTALSSILVER_PARSE_MAP_FILE_H
#define ROBITICSFUNDAMENTALSSILVER_PARSE_MAP_FILE_H


#include <vector>
#include <string>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <map>
#include <utility>

class LikelihoodField {
public:
    // Typen für Pfad-Lookup
    using Cell = std::pair<int,int>;
    using LookupMap = std::map<Cell, std::map<Cell, std::vector<Cell>>>;
    ~LikelihoodField() = default;

    void publish_low_res_walls(ros::NodeHandle &nh, const ros::Publisher &lowres_wall_pub,
                               const std::vector<std::vector<unsigned int>> &lowres_map) const;

    void publish_high_res_walls(ros::NodeHandle &nh) const;

    LikelihoodField(ros::NodeHandle &nh, const std::string &filename, double sigma);

    static bool parse_file_lowres(const std::string &filename, std::vector<std::vector<unsigned int> > &map);
    void build_wall_tables(const std::vector<std::vector<unsigned int>> &map);
    void build_lookup_map(const std::vector <std::vector<unsigned int>> &map);
    /**
     * Liefert kürzesten Pfad als Liste von Points (x=row, y=col, z=Radius).
     */
    std::vector<geometry_msgs::Point> getPath(int start_r, int start_c, int target_r, int target_c) const;
    double get_field_value(const geometry_msgs::Point &global_space_point) const;
    double get_prob_field_value(const geometry_msgs::Point &global_space_point) const;

    double get_ray_wall_dist(const geometry_msgs::Pose2D &start_point, const geometry_msgs::Point &end_point) const;
    double get_ray_wall_dist(const geometry_msgs::Point &start_point, const geometry_msgs::Point &end_point) const;

    int get_row_count() const {return row_count;};
    int get_col_count() const {return col_count;};
    int get_cell_size() const {return cell_size;};

    double sigma_value;
private:
    // All-Pairs-Pfad-Lookup
    LookupMap lookup_map_;
    enum CellWall {
        TOP = 1 << 0, RIGHT = 1 << 1, BOTTOM = 1 << 2, LEFT = 1 << 3
    };
    struct CellWallLine {
        double c;
        double start, end;
    };

    std::vector<CellWallLine> horizontal_walls;
    std::vector<CellWallLine> vertical_walls;

    std::vector<std::vector<double>> field;
    std::vector<std::vector<double>> dist_field;

    int row_count;
    int col_count;

    const int cell_size = 80;
    const int buffer_size = 80;

    ros::Publisher lowres_pub;
    ros::Publisher highres_pub;

};
#endif //ROBITICSFUNDAMENTALSSILVER_PARSE_MAP_FILE_H
