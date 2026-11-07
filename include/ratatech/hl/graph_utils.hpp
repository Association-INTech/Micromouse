#pragma once

#include <array>
#include <cstddef>
namespace ratatech::hl {

template <int Rank> struct Coord {
    size_t i, j;

    bool operator==(const Coord &other) const = default;
};

template <int Rank> struct Node {
    std::array<Coord<Rank>, Rank> coordinates;

    bool operator==(const Node &other) const = default;
};

template <int Rank> struct Edge {
    std::array<Coord<Rank>, Rank + 1> coordinates;
    int cost;
};

} // namespace ratatech::hl
