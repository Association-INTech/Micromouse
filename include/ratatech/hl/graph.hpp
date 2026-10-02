#pragma once

#include "ratatech/hl/maze.hpp"
#include <cstddef>
#include <unordered_set>
#include <vector>

namespace ratatech::hl {

class Graph {
  public:
    struct Node {
        size_t i;
        size_t j;

        bool operator==(const Node &other) const = default;
    };

    explicit Graph(const Maze &maze);

    std::vector<std::pair<Node, int>> neighbors(const Node &node) const;

    std::vector<Node> shortest_path(const Node &start,
                                    const std::unordered_set<Node> &end) const;

  private:
    const Maze &maze_;
};

} // namespace ratatech::hl

namespace std {

template <> struct hash<ratatech::hl::Graph::Node> {
    size_t operator()(const ratatech::hl::Graph::Node &node) const noexcept {
        return node.i * ratatech::hl::Maze::maze_size + node.j;
    }
};

} // namespace std
