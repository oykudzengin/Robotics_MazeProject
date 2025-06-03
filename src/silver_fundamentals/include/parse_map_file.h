#ifndef ROBITICSFUNDAMENTALSSILVER_PARSE_MAP_FILE_H
#define ROBITICSFUNDAMENTALSSILVER_PARSE_MAP_FILE_H


#include <vector>
#include <string>
#include <geometry_msgs/Point.h>
class LikelihoodField {
public:
    ~LikelihoodField() = default;

    void publish_low_res_walls(ros::NodeHandle &nh, const ros::Publisher &lowres_wall_pub,
                               const std::vector<std::vector<unsigned int>> &lowres_map) const;

    void publish_high_res_walls(ros::NodeHandle &nh) const;

    LikelihoodField(ros::NodeHandle &nh, const std::string &filename, double sigma);

    static bool parse_file_lowres(const std::string &filename, std::vector <std::vector<unsigned int>> &map);
    void build_lookup_map(const std::vector <std::vector<unsigned int>> &map);
    double get_field_value(const geometry_msgs::Point &global_space_point) const;
    double get_prob_field_value(const geometry_msgs::Point &global_space_point) const;
    int get_row_count() const {return row_count;};
    int get_col_count() const {return col_count;};
    int get_cell_size() const {return cell_size;};
private:
    enum CellWall {
        TOP = 1 << 0, RIGHT = 1 << 1, BOTTOM = 1 << 2, LEFT = 1 << 3
    };

    std::vector<std::vector<double>> field;
    double sigma_value;
    int row_count;
    int col_count;

    const int cell_size = 80;
    const int buffer_size = 80;

    ros::Publisher lowres_pub;
    ros::Publisher highres_pub;

};
#endif //ROBITICSFUNDAMENTALSSILVER_PARSE_MAP_FILE_H
