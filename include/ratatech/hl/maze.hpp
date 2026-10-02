#pragma once

#include <array>
#include <cstddef>
#include <cstdio>

namespace ratatech::hl {

enum class Direction { Left = 0, Right, Down, Up };

constexpr std::array<Direction, 4> all_directions{
    Direction::Left, Direction::Right, Direction::Down, Direction::Up};

std::pair<ssize_t, ssize_t> direction_to_delta(Direction direction);

class Maze {
  public:
    static constexpr size_t maze_size{16};

    explicit Maze();

    void add_wall(size_t i, size_t j, Direction direction);
    bool get_wall(size_t i, size_t j, Direction direction) const;

  private:
    struct Cell {
        bool right_wall;
        bool bottom_wall;
    };

    std::array<std::array<Cell, maze_size>, maze_size> data_;
};

} // namespace ratatech::hl
