#pragma once

#include <array>
#include <cstddef>
#include <cstdio>
#include <format>

namespace ratatech::hl {

enum class Direction { Right = 0, Down, Left, Up };

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

using namespace ratatech::hl;

namespace std {

template <> struct formatter<Maze> {
    constexpr auto parse(format_parse_context &context) {
        return context.begin();
    }

    template <class FormatContext>
    auto format(const Maze &maze, FormatContext &ctx) const {
        auto out = ctx.out();

        for (size_t j{0}; j < Maze::maze_size; ++j)
            out = std::format_to(out, "+---");
        out = std::format_to(out, "+");

        for (size_t i{0}; i < Maze::maze_size; ++i) {
            out = std::format_to(out, "\n|");

            for (size_t j{0}; j < Maze::maze_size; ++j)
                out = std::format_to(
                    out, "   {}",
                    maze.get_wall(i, j, Direction::Right) ? '|' : ' ');

            out = std::format_to(out, "\n+");

            for (size_t j{0}; j < Maze::maze_size; ++j)
                out = std::format_to(
                    out, "{}+",
                    maze.get_wall(i, j, Direction::Down) ? "---" : "   ");
        }

        return out;
    }
};

} // namespace std
