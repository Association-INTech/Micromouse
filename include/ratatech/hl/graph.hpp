#pragma once

#include "ratatech/hl/maze.hpp"
#include <cstddef>
#include <format>
#include <optional>
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

    std::optional<std::vector<Node>>
    shortest_path(const Node &start, const std::unordered_set<Node> &end) const;

  private:
    const Maze &maze_;
};

} // namespace ratatech::hl

using namespace ratatech::hl;

namespace std {

template <> struct hash<Graph::Node> {
    size_t operator()(const Graph::Node &node) const noexcept {
        return node.i * Maze::maze_size + node.j;
    }
};

template <> struct formatter<Graph::Node> {
    constexpr auto parse(format_parse_context &context) {
        return context.begin();
    }

    template <class FormatContext>
    auto format(const Graph::Node &node, FormatContext &ctx) const {
        return std::format_to(ctx.out(), "Node({}, {})", node.i, node.j);
    }
};

} // namespace std
