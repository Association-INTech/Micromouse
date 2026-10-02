#include "ratatech/hl/graph.hpp"
#include "ratatech/hl/maze.hpp"
#include <algorithm>
#include <limits>
#include <queue>
#include <unordered_map>
#include <utility>

namespace ratatech::hl {

Graph::Graph(const Maze &maze) : maze_{maze} {}

std::vector<std::pair<Graph::Node, int>>
Graph::neighbors(const Node &node) const {
    std::vector<std::pair<Graph::Node, int>> neighbors{};

    for (auto dir : all_directions) {
        auto [delta_i, delta_j] = direction_to_delta(dir);

        if (maze_.get_wall(node.i, node.j, dir))
            neighbors.push_back({{node.i + delta_i, node.j + delta_j}, 1});
    }

    return neighbors;
}

std::vector<Graph::Node>
Graph::shortest_path(const Node &start,
                     const std::unordered_set<Node> &end) const {
    using Entry = std::pair<Node, int>;
    std::priority_queue<Entry, std::vector<Entry>,
                        decltype([](const Entry &a, const Entry &b) {
                            return a.second > b.second;
                        })>
        to_process;
    to_process.push({start, 0});

    std::unordered_map<Node, int> distances;
    for (size_t i{0}; i < Maze::maze_size; ++i)
        for (size_t j{0}; j < Maze::maze_size; ++j)
            distances[Node(i, j)] = std::numeric_limits<int>::max();
    distances[start] = 0;

    std::unordered_map<Node, Node> predecessors;

    std::vector<Node> result;

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

    // build back the path
    for (auto node{result.back()}; node != start; node = predecessors[node])
        result.push_back(node);

    std::reverse(result.begin(), result.end());
    return result;
}

} // namespace ratatech::hl
