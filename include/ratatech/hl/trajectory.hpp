#pragma once

#include "ratatech/hl/curve.hpp"
#include <cstddef>
#include <optional>
#include <vector>

namespace ratatech::hl {

template <int Rank> class Trajectory {
  public:
    explicit Trajectory(const std::vector<Edge<Rank>> &path);

    std::optional<StateVector> state(float time);

  private:
    std::vector<Curve<Rank>> curves;
    size_t last_access_idx;
};

} // namespace ratatech::hl
