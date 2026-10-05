#pragma once

#include "ratatech/hl/graph.hpp"

namespace ratatech::hl {

inline constexpr float cell_size{0.16f};

struct StateVector {
    float x;     // m
    float y;     // m
    float theta; // rad
    float speed; // m/s
    float omega; // rad/s
};

class CurveStraight {
  public:
    explicit CurveStraight(float start_time, const Graph::Node &start,
                           const Graph::Node &end);

    float start_time() const { return start_time_; }
    float end_time() const { return end_time_; }

    StateVector state(float time) const;

  private:
    static constexpr float speed{1000};
    float start_time_, end_time_;
    float start_x_, start_y_;
    float end_x_, end_y_;
    float theta_;
};

} // namespace ratatech::hl
