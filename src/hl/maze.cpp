#include "ratatech/hl/maze.hpp"
#include <array>

namespace ratatech::hl {

std::pair<ssize_t, ssize_t> direction_to_delta(Direction direction) {
    constexpr std::array<std::pair<ssize_t, ssize_t>, 4> deltas{{
        {0, -1},
        {0, +1},
        {-1, 0},
        {+1, 0},
    }};

    return deltas[static_cast<size_t>(direction)];
}

Maze::Maze() : data_{} {
    for (size_t i{0}; i < maze_size; ++i) {
        data_[i][maze_size - 1].right_wall = true;
        data_[maze_size - 1][i].bottom_wall = true;
    }
}

void Maze::add_wall(size_t i, size_t j, Direction direction) {
    switch (direction) {
    case Direction::Right:
        data_[i][j].right_wall = true;
    case Direction::Down:
        data_[i][j].bottom_wall = true;
    case Direction::Left:
        if (j != 0)
            data_[i][j - 1].right_wall = true;
    case Direction::Up:
        if (i != 0)
            data_[i - 1][j].bottom_wall = true;
    }
}

bool Maze::get_wall(size_t i, size_t j, Direction direction) const {
    switch (direction) {
    case Direction::Right:
        return data_[i][j].right_wall;
    case Direction::Down:
        return data_[i][j].bottom_wall;
    case Direction::Left:
        if (j != 0)
            return data_[i][j - 1].right_wall;
    case Direction::Up:
        if (i != 0)
            return data_[i - 1][j].bottom_wall;
    }

    return true;
}

} // namespace ratatech::hl
