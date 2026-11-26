#pragma once

#include "ratatech/hl/graph.hpp"
#include "ratatech/hl/graph_utils.hpp"
#include "ratatech/hl/maze.hpp"
#include <algorithm>
#include <optional>
#include <queue>
#include <unordered_map>
#include <utility>

namespace ratatech::hl {

template <int Rank> Graph<Rank>::Graph(const Maze &maze) : maze_{maze} {}

template <int Rank>
std::vector<Edge<Rank>> Graph<Rank>::neighbors(const Node<Rank> &node) const {
    std::vector<Edge<Rank>> neighbors;

    for (auto dir : all_directions) {
        auto [delta_i, delta_j] = direction_to_delta(dir);
        auto last_coord{node.coordinates.back()};

        if (!maze_.get_wall(last_coord.i, last_coord.j, dir)) {
            last_coord.i += delta_i;
            last_coord.j += delta_j;

            neighbors.emplace_back(node, last_coord, 1);
        }
    }

    return neighbors;
}

template <int Rank>
std::optional<std::vector<Edge<Rank>>>
Graph<Rank>::shortest_path(const Node<Rank> &start,
                           const std::unordered_set<Node<Rank>> &end) const {
    using Entry = std::pair<Node<Rank>, int>;
    struct Compare {
        bool operator()(const Entry &a, const Entry &b) {
            return a.second > b.second;
        }
    };

    std::priority_queue<Entry, std::vector<Entry>, Compare> to_process;
    std::unordered_map<Node<Rank>, Node<Rank>> predecessors;
    std::unordered_map<Node<Rank>, int> distances;
    distances[start] = 0;

    // will be used to create the result path from successor to predecessor
    auto successor{Node<Rank>::invalid_node()};
    std::vector<Edge<Rank>> result;

    for (to_process.push({start, 0}); !to_process.empty(); to_process.pop()) {
        auto [node, distance]{to_process.top()};

        if (end.contains(node)) {
            successor = node;
            break;
        }

        for (auto edge : neighbors(node)) {
            Node child{edge.destination()};
            int new_distance = distance + edge.cost;
            // (child, prev_distance) if child is found or end()
            auto prev_item{distances.find(child)};

            if (prev_item == distances.end() ||
                new_distance < prev_item->second) {
                predecessors.insert_or_assign(child, node);
                distances.insert_or_assign(child, new_distance);
                to_process.push({child, new_distance});
            }
        }
    }

    // make sure we did find a path
    if (successor == Node<Rank>::invalid_node()) {
        return std::nullopt;
    }

    // build back the path
    // (it can be empty)
    while (successor != start) {
        Node predecessor{predecessors.at(successor)};
        result.emplace_back(predecessor, successor, 1);
        successor = predecessor;
    }

    std::reverse(result.begin(), result.end());
    return result;
}

} // namespace ratatech::hl

// Necessary stuff for use in unordered_map
template <int Rank>
size_t std::hash<ratatech::hl::Node<Rank>>::operator()(
    const ratatech::hl::Node<Rank> &node) const noexcept {
    constexpr uint32_t FNV_offset_basis{2166136261};
    constexpr uint32_t FNV_prime{16777619};

    uint32_t hash{FNV_offset_basis};

    auto add_bytes{[&](size_t value) {
        for (size_t i{0}; i < sizeof(value); ++i) {
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
