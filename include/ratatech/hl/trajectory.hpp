#pragma once

#include "ratatech/hl/graph.hpp"
#include <cstdint>
#include <vector>

namespace ratatech::hl {

using millis_t = uint32_t;

struct StateVector {
    float x;
    float y;
    float theta;
    float vx;
    float vy;
    float omega;
};

class Curve {
  public:
    millis_t start() const;
    millis_t end() const;
    virtual StateVector state(millis_t time) const;

  private:
    millis_t start_;
    millis_t end_;
};

class Trajectory {
  public:
    explicit Trajectory(const std::vector<Graph::Node> &path);

    StateVector state(millis_t time) const;

  private:
    std::vector<Curve> curves;
};

} // namespace ratatech::hl
