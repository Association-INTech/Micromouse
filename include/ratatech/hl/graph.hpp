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
    size_t operator()(const ratatech::hl::Node<Rank> &node) const noexcept {
        constexpr uint32_t FNV_offset_basis{2166136261};
        constexpr uint32_t FNV_prime{16777619};

        uint32_t hash{FNV_offset_basis};

        auto add_bytes{[&](size_t value) {
            for (int i{0}; i < sizeof(value); ++i) {
                auto byte{static_cast<uint8_t>((value >> (8 * i)) & 0xff)};
                hash ^= byte;
                hash *= FNV_prime;
            }
        }};

        for (auto &coord : node.coordinates) {
            add_bytes(coord.i);
            add_bytes(coord.j);
        }

        return hash;
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

#include "ratatech/hl/graph.tpp"
