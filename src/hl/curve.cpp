#include "ratatech/hl/curve.hpp"
#include <cmath>

namespace ratatech::hl {

CurveStraight::CurveStraight(float start_time, const Graph::Node &start,
                             const Graph::Node &end)
    : start_time_{start_time},
      end_time_{
          start_time +
              std::hypotf(end.i - start.i, end.j - start.j) * cell_size / speed,
      },
      start_x_{start.i * cell_size}, start_y_{start.j * cell_size},
      end_x_{end.i * cell_size}, end_y_{end.j * cell_size},
      theta_{std::atan2f(end.j - start.j, end.i - start.i)} {}

StateVector CurveStraight::state(float time) const {
    float progress = (time - start_time()) / (end_time() - start_time());

    return {.x = std::lerp(start_x_, end_x_, progress),
            .y = std::lerp(start_y_, end_y_, progress),
            .theta = theta_,
            .speed = speed,
            .omega = 0.0f};
};

} // namespace ratatech::hl
