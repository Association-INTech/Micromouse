#pragma once

#include "ratatech/hl/graph_utils.hpp"
#include "ratatech/hl/state_vector.hpp"

namespace ratatech::hl {

class CurveStraight {
  public:
    explicit CurveStraight(const Edge<1> &edge, float start_time);

    float start_time() const { return start_time_; }
    float end_time() const { return end_time_; }

    StateVector state(float time) const;

  private:
    static constexpr float speed{1.0f};
    static constexpr float stop_duration{1.0f};
    float start_time_, end_time_;
    float start_x_, start_y_;
    float end_x_, end_y_;
    float theta_;
};

} // namespace ratatech::hl
