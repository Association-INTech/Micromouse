#pragma once

#include "ratatech/hl/graph_utils.hpp"
#include "ratatech/hl/maze.hpp"
#include <cstddef>
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

namespace std {

template <int Rank> struct hash<ratatech::hl::Node<Rank>> {
    size_t operator()(const ratatech::hl::Node<Rank> &node) const noexcept {
        return node.i * ratatech::hl::maze_size + node.j;
    }
};

template <int Rank> struct formatter<ratatech::hl::Node<Rank>> {
    constexpr auto parse(format_parse_context &context) {
        return context.begin();
    }

    template <class FormatContext>
    auto format(const ratatech::hl::Node<Rank> &node,
                FormatContext &ctx) const {
        return std::format_to(ctx.out(), "Node({}, {})", node.i, node.j);
    }
};

} // namespace std
