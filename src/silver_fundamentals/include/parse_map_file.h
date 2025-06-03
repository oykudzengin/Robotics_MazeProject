#ifndef ROBITICSFUNDAMENTALSSILVER_PARSE_MAP_FILE_H
#define ROBITICSFUNDAMENTALSSILVER_PARSE_MAP_FILE_H

#include <vector>
#include <string>
#include <geometry_msgs/Point.h>
class LikelihoodField {
public:
    LikelihoodField(std::string filename, double sigma);
    ~LikelihoodField() = default;

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

};
#endif //ROBITICSFUNDAMENTALSSILVER_PARSE_MAP_FILE_H
