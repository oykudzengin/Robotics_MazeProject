#ifndef SILVER_FUNDAMENTALS_MAZE_GRAPH_H
#define SILVER_FUNDAMENTALS_MAZE_GRAPH_H
#include <parse_map_file.h>
class mazeGraph {
public:
    mazeGraph();
    ~mazeGraph() = default;
private:
   LikelihoodField lhf;
}
#endif
