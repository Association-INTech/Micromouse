#pragma once

#include "ratatech/hl/curve.hpp"
#include "ratatech/hl/graph.hpp"
#include <cstddef>
#include <variant>
#include <vector>

namespace ratatech::hl {

class Trajectory {
  public:
    explicit Trajectory(const std::vector<Graph::Node> &path);

    StateVector state(float time);

  private:
    std::vector<std::variant<CurveStraight>> curves;
    size_t last_access_idx;
};

} // namespace ratatech::hl
