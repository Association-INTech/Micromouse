#include "ratatech/hl/trajectory.hpp"
#include "ratatech/hl/graph_utils.hpp"
#include <optional>
#include <vector>

namespace ratatech::hl {

template <>
Trajectory<1>::Trajectory(const std::vector<Edge<1>> &path)
    : last_access_idx{0} {
    float previous_end_time = 0.0f;
    for (const Edge<1> &edge : path) {
        curves.push_back(CurveStraight(edge, previous_end_time));
        previous_end_time = curves.back().end_time();
    }
}

template <int Rank>
std::optional<StateVector> Trajectory<Rank>::state(float time) {
    // change last_access_idx if necessary
    while (time < curves[last_access_idx].start_time()) {
        --last_access_idx;
    }
    while (time > end_time_visit(curves[last_access_idx])) {
        ++last_access_idx;
    }

    if (0 <= last_access_idx && last_access_idx <= curves.size())
        return state_visit(curves[last_access_idx], time);
    else
        return std::nullopt;
}

} // namespace ratatech::hl
