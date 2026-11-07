#pragma once

namespace ratatech::hl {

inline constexpr float cell_size{0.16f};

struct StateVector {
    float x;     // m
    float y;     // m
    float theta; // rad
    float speed; // m/s
    float omega; // rad/s
};

} // namespace ratatech::hl
