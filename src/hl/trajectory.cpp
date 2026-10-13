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

Trajectory::Trajectory(const std::vector<Graph::Node> &path)
    : last_access_idx{0} {
    if (path.empty())
        return;

    Graph::Node old_node{path.front()};
    float old_end_time{0};

    curves.push_back(CurveStart(old_end_time, old_node));

    for (const Graph::Node &new_node : path | std::views::drop(1)) {
        curves.push_back(CurveStraight(old_end_time, old_node, new_node));

        old_node = new_node;
        old_end_time = end_time_visit(curves.back());
    }

    curves.push_back(CurveEnd(old_end_time, old_node));
}

StateVector Trajectory::state(float time) {
    // change last_access_idx if necessary
    while (time < start_time_visit(curves[last_access_idx])) {
        --last_access_idx;
    }
    while (time > end_time_visit(curves[last_access_idx])) {
        ++last_access_idx;
    }

    return state_visit(curves[last_access_idx], time);
}

} // namespace ratatech::hl
