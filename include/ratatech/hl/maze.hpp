#pragma once

#include <array>
#include <cstddef>
#include <cstdio>
#include <format>

namespace ratatech::hl {

enum class Direction { Right = 0, Down, Left, Up };

inline constexpr std::array<Direction, 4> all_directions{
    Direction::Left, Direction::Right, Direction::Down, Direction::Up};

std::pair<ssize_t, ssize_t> direction_to_delta(Direction direction);

inline constexpr size_t maze_size{16};

class Maze {
  public:
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

namespace std {

template <> struct formatter<ratatech::hl::Maze> {
    constexpr auto parse(format_parse_context &context) {
        return context.begin();
    }

    template <class FormatContext>
    auto format(const ratatech::hl::Maze &maze, FormatContext &ctx) const {
        auto out = ctx.out();

        for (size_t j{0}; j < ratatech::hl::maze_size; ++j)
            out = std::format_to(out, "+---");
        out = std::format_to(out, "+");

        for (size_t i{0}; i < ratatech::hl::maze_size; ++i) {
            out = std::format_to(out, "\n|");

            for (size_t j{0}; j < ratatech::hl::maze_size; ++j)
                out = std::format_to(
                    out, "   {}",
                    maze.get_wall(i, j, ratatech::hl::Direction::Right) ? '|'
                                                                        : ' ');

            out = std::format_to(out, "\n+");

            for (size_t j{0}; j < ratatech::hl::maze_size; ++j)
                out = std::format_to(
                    out, "{}+",
                    maze.get_wall(i, j, ratatech::hl::Direction::Down) ? "---"
                                                                       : "   ");
        }

        return out;
    }
};

} // namespace std
