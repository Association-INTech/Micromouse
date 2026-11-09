#include "ratatech/hl/graph.hpp"
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

        if (!maze_.get_wall(node.i, node.j, dir)) {
            Edge edge{.coordinates = node.coordinates, .cost = 1};

            // add the last coordinate to the edge
            edge.coordinates.back() = {.i = last_coord.i + delta_i,
                                       .j = last_coord.j + delta_j};

            neighbors.push_back(edge);
        }
    }

    return neighbors;
}

template <int Rank>
std::optional<std::vector<Edge<Rank>>>
Graph<Rank>::shortest_path(const Node<Rank> &start,
                           const std::unordered_set<Node<Rank>> &end) const {
    using Entry = std::pair<Node<Rank>, int>;
    std::priority_queue<Entry, std::vector<Entry>,
                        decltype([](const Entry &a, const Entry &b) {
                            return a.second > b.second;
                        })>
        to_process;
    to_process.push({start, 0});

    std::unordered_map<Node<Rank>, Node<Rank>> predecessors;
    std::unordered_map<Node<Rank>, int> distances;
    distances[start] = 0;

    std::vector<Node<Rank>> result;

    for (; !to_process.empty(); to_process.pop()) {
        auto [node, distance]{to_process.top()};

        if (end.contains(node)) {
            result.push_back(node);
            break;
        }

        for (auto [child, extra_distance] : neighbors(node)) {
            int new_distance = distance + extra_distance;

            if (new_distance < distances[child]) {
                predecessors[child] = node;
                distances[child] = new_distance;
                to_process.push({child, new_distance});
            }
        }
    }

    if (result.empty()) {
        return std::nullopt;
    }

    // build back the path
    while (result.back() != start)
        result.push_back(predecessors[result.back()]);

    std::reverse(result.begin(), result.end());
    return result;
}

} // namespace ratatech::hl
