#include "ratatech/hl/curve_rank1.hpp"
#include <algorithm>
#include <cmath>

namespace ratatech::hl {

CurveStraight::CurveStraight(const Edge<1> &edge, float start_time)
    : start_time_{start_time},
      end_time_{
          start_time + stop_duration +
              std::hypotf(static_cast<float>(edge.coordinates[0].i) -
                              static_cast<float>(edge.coordinates[1].i),
                          static_cast<float>(edge.coordinates[0].j) -
                              static_cast<float>(edge.coordinates[1].j)) *
                  cell_size / speed,
      },
      start_x_{edge.coordinates[1].i * cell_size},
      start_y_{edge.coordinates[1].j * cell_size},
      end_x_{edge.coordinates[0].i * cell_size},
      end_y_{edge.coordinates[0].j * cell_size},
      theta_{std::atan2f(static_cast<float>(edge.coordinates[0].j) -
                             static_cast<float>(edge.coordinates[1].j),
                         static_cast<float>(edge.coordinates[0].i) -
                             static_cast<float>(edge.coordinates[1].i))} {}

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

} // namespace ratatech::hl
