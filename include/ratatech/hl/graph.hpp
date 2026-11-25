#pragma once

#include "ratatech/hl/graph_utils.hpp"
#include "ratatech/hl/maze.hpp"
#include <cstddef>
#include <cstdint>
#include <unordered_set>
#include <vector>

namespace ratatech::hl {

template <int Rank> class Graph {
  private:
    const Maze &maze_;

  public:
    explicit Graph(const Maze &maze);

    std::vector<Edge<Rank>> neighbors(const Node<Rank> &node) const;

    std::optional<std::vector<Edge<Rank>>>
    shortest_path(const Node<Rank> &start,
                  const std::unordered_set<Node<Rank>> &end) const;
};

} // namespace ratatech::hl

// Necessary stuff for use in unordered_map
namespace std {

template <int Rank> struct hash<ratatech::hl::Node<Rank>> {
    size_t operator()(const ratatech::hl::Node<Rank> &node) const noexcept;
};

} // namespace std

#include "ratatech/hl/graph.tpp"
