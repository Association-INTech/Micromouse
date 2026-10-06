#include "ratatech/hl/trajectory.hpp"
#include "ratatech/hl/curve.hpp"
#include "ratatech/hl/graph.hpp"
#include <ranges>
#include <variant>

// some helper functions
namespace {
auto start_time_visit(auto &curve) {
    return std::visit([](auto &&curve) { return curve.start_time(); }, curve);
}
auto end_time_visit(auto &curve) {
    return std::visit([](auto &&curve) { return curve.end_time(); }, curve);
}
auto state_visit(auto &curve, float time) {
    return std::visit([time](auto &&curve) { return curve.state(time); },
                      curve);
}
} // namespace

namespace ratatech::hl {

Trajectory::Trajectory(const std::vector<Graph::Node> &path) {
    if (path.empty())
        return;

    if (path.size() == 1) {
        curves.push_back(CurveStraight(0, path.back(), path.back()));
        return;
    }

    Graph::Node old_node{path.front()};
    float old_end_time{0};

    for (const Graph::Node &new_node : path | std::views::drop(1)) {
        curves.push_back(CurveStraight(old_end_time, old_node, new_node));

        old_node = new_node;
        old_end_time = end_time_visit(curves.back());
    }
}

StateVector Trajectory::state(float time) {
    auto curve{curves.begin() + last_access_idx};

    float start_time{start_time_visit(*curve)};
    float end_time{end_time_visit(*curve)};

    // change last_access_idx if necessary

    while (time < start_time) {
        --last_access_idx;
        curve = curves.begin() + last_access_idx;
        start_time = start_time_visit(*curve);
    }

    while (time > end_time) {
        ++last_access_idx;
        curve = curves.begin() + last_access_idx;
        start_time = start_time_visit(*curve);
    }

    return state_visit(*curve, time);
}

} // namespace ratatech::hl
