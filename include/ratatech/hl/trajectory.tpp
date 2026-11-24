#pragma once

#include "ratatech/hl/graph_utils.hpp"
#include "ratatech/hl/trajectory.hpp"
#include <optional>
#include <vector>

namespace ratatech::hl {

template <> Trajectory<1>::Trajectory(const std::vector<Edge<1>> &path) {
    float previous_end_time = 0.0f;
    for (const Edge<1> &edge : path) {
        curves.push_back(CurveStraight(edge, previous_end_time));
        previous_end_time = curves.back().end_time();
    }
}

template <int Rank>
std::optional<StateVector> Trajectory<Rank>::state(float time) const {
    if (curves.empty() || time < curves[0].start_time())
        return std::nullopt;

    size_t idx;
    for (idx = 0; idx < curves.size() && time > curves[idx].end_time(); ++idx)
        ;

    if (idx < curves.size())
        return curves[idx].state(time);
    else
        return std::nullopt;
}

} // namespace ratatech::hl
