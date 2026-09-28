#pragma once

#include <array>
#include <cstddef>

namespace ratatech::hl {

enum class Direction { Left, Right, Bottom, Up };

class Maze {
  public:
    static constexpr size_t size{16};

    explicit Maze();

    void add_wall(size_t i, size_t j, Direction direction);
    bool get_wall(size_t i, size_t j, Direction direction) const;

  private:
    struct Cell {
        bool righ_wall;
        bool bottom_wall;
    };

    std::array<std::array<Cell, size>, size> data_;
};

} // namespace ratatech::hl
