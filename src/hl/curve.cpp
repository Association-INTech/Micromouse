#include "ratatech/hl/curve.hpp"
#include <algorithm>
#include <cmath>

namespace ratatech::hl {

CurveStraight::CurveStraight(float start_time, const Graph::Node &start,
                             const Graph::Node &end)
    : start_time_{start_time},
      end_time_{
          start_time + stop_duration +
              std::hypotf(
                  static_cast<float>(end.i) - static_cast<float>(start.i),
                  static_cast<float>(end.j) - static_cast<float>(start.j)) *
                  cell_size / speed,
      },
      start_x_{start.i * cell_size}, start_y_{start.j * cell_size},
      end_x_{end.i * cell_size}, end_y_{end.j * cell_size},
      theta_{std::atan2f(
          static_cast<float>(end.j) - static_cast<float>(start.j),
          static_cast<float>(end.i) - static_cast<float>(start.i))} {}

StateVector CurveStraight::state(float time) const {
    float progress = (time - start_time() - stop_duration) /
                     (end_time() - start_time() - stop_duration);

    progress = std::clamp(progress, 0.0f, 1.0f);

    return {.x = std::lerp(start_x_, end_x_, progress),
            .y = std::lerp(start_y_, end_y_, progress),
            .theta = theta_,
            .speed = speed,
            .omega = 0.0f};
};

CurveStart::CurveStart(float end_time, const Graph::Node &node)
    : end_time_{end_time}, x_{node.i * cell_size}, y_{node.j * cell_size} {}

StateVector CurveStart::state([[maybe_unused]] float time) const {
    return {.x = x_, .y = y_, .theta = 0, .speed = 0, .omega = 0};
};

CurveEnd::CurveEnd(float start_time, const Graph::Node &node)
    : start_time_{start_time}, x_{node.i * cell_size}, y_{node.j * cell_size} {}

StateVector CurveEnd::state([[maybe_unused]] float time) const {
    return {.x = x_, .y = y_, .theta = 0, .speed = 0, .omega = 0};
};

} // namespace ratatech::hl
