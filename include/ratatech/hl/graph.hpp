#pragma once

#include "ratatech/hl/maze.hpp"
#include <cstddef>
#include <vector>

namespace ratatech::hl {

class Graph {
  public:
    struct Node {
        size_t i;
        size_t j;
    };

    explicit Graph(const Maze &maze);

    std::vector<std::pair<Node, int>> neighbors(const Node &node) const;

    std::vector<Node> shortest_path(const Node &start,
                                    const std::vector<Node> &end) const;

  private:
    const Maze &maze_;
};

} // namespace ratatech::hl
