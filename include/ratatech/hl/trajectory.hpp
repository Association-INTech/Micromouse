#pragma once

#include "ratatech/hl/curve.hpp"
#include <optional>
#include <vector>

namespace ratatech::hl {

template <int Rank> class Trajectory {
  public:
    explicit Trajectory(const std::vector<Edge<Rank>> &path);

    std::optional<StateVector> state(float time) const;

  private:
    std::vector<Curve<Rank>> curves;
};

} // namespace ratatech::hl

#include "ratatech/hl/trajectory.tpp"
