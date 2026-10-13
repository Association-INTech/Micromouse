#include "ratatech/hl/maze.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <utility>

using namespace ratatech::hl;

namespace {

// Direction convention (from the existing tests):
//   Right -> j + 1, Left -> j - 1, Down -> i + 1, Up -> i - 1
Direction opposite(Direction d) {
    switch (d) {
    case Direction::Left:
        return Direction::Right;
    case Direction::Right:
        return Direction::Left;
    case Direction::Up:
        return Direction::Down;
    case Direction::Down:
        return Direction::Up;
    }
    return d; // unreachable
}

// True if the neighbor of (i, j) in direction d is inside the maze.
bool has_neighbor(size_t i, size_t j, Direction d) {
    switch (d) {
    case Direction::Left:
        return j > 0;
    case Direction::Right:
        return j < maze_size - 1;
    case Direction::Up:
        return i > 0;
    case Direction::Down:
        return i < maze_size - 1;
    }
    return false;
}

std::pair<size_t, size_t> neighbor(size_t i, size_t j, Direction d) {
    switch (d) {
    case Direction::Left:
        return {i, j - 1};
    case Direction::Right:
        return {i, j + 1};
    case Direction::Up:
        return {i - 1, j};
    case Direction::Down:
        return {i + 1, j};
    }
    return {i, j};
}

size_t count_walls(const Maze &maze, size_t i, size_t j) {
    size_t n = 0;
    for (Direction d : all_directions)
        n += maze.get_wall(i, j, d) ? 1 : 0;
    return n;
}

} // namespace

// ============================================================================
// Default state
// ============================================================================

// Verify default initialization (boundary walls present, interior empty)
TEST(Maze, DefaultState) {
    Maze maze;

    // Outer boundary walls at the corners
    EXPECT_TRUE(maze.get_wall(0, 0, Direction::Left));
    EXPECT_TRUE(maze.get_wall(0, 0, Direction::Up));
    EXPECT_TRUE(maze.get_wall(maze_size - 1, maze_size - 1, Direction::Right));
    EXPECT_TRUE(maze.get_wall(maze_size - 1, maze_size - 1, Direction::Down));

    // Interior walls are open by default
    EXPECT_FALSE(maze.get_wall(0, 0, Direction::Right));
    EXPECT_FALSE(maze.get_wall(0, 0, Direction::Down));
    EXPECT_FALSE(maze.get_wall(5, 5, Direction::Up));
    EXPECT_FALSE(maze.get_wall(5, 5, Direction::Left));
}

// Every cell on the border has the wall(s) facing outward, on all four sides.
TEST(Maze, FullPerimeterIsWalled) {
    Maze maze;
    for (size_t k = 0; k < maze_size; ++k) {
        EXPECT_TRUE(maze.get_wall(0, k, Direction::Up)) << "top, col " << k;
        EXPECT_TRUE(maze.get_wall(maze_size - 1, k, Direction::Down))
            << "bottom, col " << k;
        EXPECT_TRUE(maze.get_wall(k, 0, Direction::Left)) << "left, row " << k;
        EXPECT_TRUE(maze.get_wall(k, maze_size - 1, Direction::Right))
            << "right, row " << k;
    }
}

// Every wall between two cells is open by default.
TEST(Maze, AllInteriorEdgesOpenByDefault) {
    Maze maze;
    for (size_t i = 0; i < maze_size; ++i)
        for (size_t j = 0; j < maze_size; ++j)
            for (Direction d : all_directions)
                if (has_neighbor(i, j, d))
                    EXPECT_FALSE(maze.get_wall(i, j, d))
                        << "cell (" << i << "," << j << ")";
}

TEST(Maze, CornerCellsHaveExactlyTwoWallsByDefault) {
    Maze maze;
    EXPECT_EQ(count_walls(maze, 0, 0), 2u);
    EXPECT_EQ(count_walls(maze, 0, maze_size - 1), 2u);
    EXPECT_EQ(count_walls(maze, maze_size - 1, 0), 2u);
    EXPECT_EQ(count_walls(maze, maze_size - 1, maze_size - 1), 2u);
}

TEST(Maze, EdgeCellsHaveExactlyOneWallByDefault) {
    Maze maze;
    for (size_t k = 1; k < maze_size - 1; ++k) {
        EXPECT_EQ(count_walls(maze, 0, k), 1u);
        EXPECT_EQ(count_walls(maze, maze_size - 1, k), 1u);
        EXPECT_EQ(count_walls(maze, k, 0), 1u);
        EXPECT_EQ(count_walls(maze, k, maze_size - 1), 1u);
    }
}

TEST(Maze, InteriorCellsHaveNoWallsByDefault) {
    Maze maze;
    for (size_t i = 1; i < maze_size - 1; ++i)
        for (size_t j = 1; j < maze_size - 1; ++j)
            EXPECT_EQ(count_walls(maze, i, j), 0u);
}

// ============================================================================
// Wall reciprocity
// ============================================================================

// Wall reciprocity: a wall added on one cell must be visible from its neighbor
TEST(Maze, WallReciprocity) {
    Maze maze;
    size_t i = 4, j = 4;

    // Right wall of (4, 4) is Left wall of (4, 5)
    maze.add_wall(i, j, Direction::Right);
    EXPECT_TRUE(maze.get_wall(i, j, Direction::Right));
    EXPECT_TRUE(maze.get_wall(i, j + 1, Direction::Left));

    // Down wall of (4, 4) is Up wall of (5, 4)
    maze.add_wall(i, j, Direction::Down);
    EXPECT_TRUE(maze.get_wall(i, j, Direction::Down));
    EXPECT_TRUE(maze.get_wall(i + 1, j, Direction::Up));
}

// Reverse direction insertion: Left wall of (4, 5) sets Right wall of (4, 4)
TEST(Maze, WallReciprocityReverse) {
    Maze maze;
    size_t i = 4, j = 4;

    maze.add_wall(i, j + 1, Direction::Left);
    EXPECT_TRUE(maze.get_wall(i, j, Direction::Right));
    EXPECT_TRUE(maze.get_wall(i, j + 1, Direction::Left));
}

TEST(Maze, WallReciprocityUpAndLeft) {
    Maze maze;

    maze.add_wall(4, 4, Direction::Up);
    EXPECT_TRUE(maze.get_wall(3, 4, Direction::Down));

    maze.add_wall(4, 4, Direction::Left);
    EXPECT_TRUE(maze.get_wall(4, 3, Direction::Right));
}

// Exhaustive: for every cell and every direction with a neighbor, adding the
// wall from one side makes it visible from the other side.
TEST(Maze, ReciprocityHoldsForEveryEdge) {
    for (size_t i = 0; i < maze_size; ++i) {
        for (size_t j = 0; j < maze_size; ++j) {
            for (Direction d : all_directions) {
                if (!has_neighbor(i, j, d))
                    continue;
                Maze maze;
                maze.add_wall(i, j, d);
                auto [ni, nj] = neighbor(i, j, d);
                EXPECT_TRUE(maze.get_wall(i, j, d))
                    << "(" << i << "," << j << ")";
                EXPECT_TRUE(maze.get_wall(ni, nj, opposite(d)))
                    << "(" << i << "," << j << ")";
            }
        }
    }
}

// Reciprocity right at the index limits (off-by-one hunting).
TEST(Maze, ReciprocityAtUpperIndexLimits) {
    Maze maze;

    maze.add_wall(maze_size - 1, maze_size - 1 - 1, Direction::Right);
    EXPECT_TRUE(maze.get_wall(maze_size - 1, maze_size - 1, Direction::Left));

    maze.add_wall(maze_size - 1 - 1, maze_size - 1, Direction::Down);
    EXPECT_TRUE(maze.get_wall(maze_size - 1, maze_size - 1, Direction::Up));
}

TEST(Maze, ReciprocityAtLowerIndexLimits) {
    Maze maze;

    maze.add_wall(0, 1, Direction::Left);
    EXPECT_TRUE(maze.get_wall(0, 0, Direction::Right));

    maze.add_wall(1, 0, Direction::Up);
    EXPECT_TRUE(maze.get_wall(0, 0, Direction::Down));
}

// ============================================================================
// Isolation: add_wall touches exactly the intended edge
// ============================================================================

// Adding one wall changes that edge and nothing else.
TEST(Maze, AddWallOnlyAffectsOneEdge) {
    for (Direction added : all_directions) {
        Maze maze;
        maze.add_wall(8, 8, added);

        for (Direction d : all_directions)
            EXPECT_EQ(maze.get_wall(8, 8, d), d == added);
    }
}

TEST(Maze, AddWallDoesNotAffectOtherCells) {
    Maze maze;
    maze.add_wall(4, 4, Direction::Right);

    // The two cells sharing the edge changed; their other sides did not.
    EXPECT_EQ(count_walls(maze, 4, 4), 1u);
    EXPECT_EQ(count_walls(maze, 4, 5), 1u);

    // Neighbors of the cells involved are untouched.
    EXPECT_EQ(count_walls(maze, 3, 4), 0u);
    EXPECT_EQ(count_walls(maze, 5, 4), 0u);
    EXPECT_EQ(count_walls(maze, 4, 3), 0u);
    EXPECT_EQ(count_walls(maze, 3, 5), 0u);
    EXPECT_EQ(count_walls(maze, 5, 5), 0u);
    EXPECT_EQ(count_walls(maze, 4, 6), 0u);
}

// Walls on different edges of the same cell are independent.
TEST(Maze, MultipleWallsOnOneCell) {
    Maze maze;
    maze.add_wall(6, 6, Direction::Left);
    maze.add_wall(6, 6, Direction::Up);

    EXPECT_TRUE(maze.get_wall(6, 6, Direction::Left));
    EXPECT_TRUE(maze.get_wall(6, 6, Direction::Up));
    EXPECT_FALSE(maze.get_wall(6, 6, Direction::Right));
    EXPECT_FALSE(maze.get_wall(6, 6, Direction::Down));
    EXPECT_EQ(count_walls(maze, 6, 6), 2u);
}

// A cell can be fully enclosed, and the walls are shared with all 4 neighbors.
TEST(Maze, FullyEnclosedCell) {
    Maze maze;
    for (Direction d : all_directions)
        maze.add_wall(3, 3, d);

    EXPECT_EQ(count_walls(maze, 3, 3), 4u);
    EXPECT_TRUE(maze.get_wall(3, 2, Direction::Right));
    EXPECT_TRUE(maze.get_wall(3, 4, Direction::Left));
    EXPECT_TRUE(maze.get_wall(2, 3, Direction::Down));
    EXPECT_TRUE(maze.get_wall(4, 3, Direction::Up));
}

// Adjacent cells share the edge between them but not the others.
TEST(Maze, AdjacentCellsShareOnlyTheCommonEdge) {
    Maze maze;
    maze.add_wall(5, 5, Direction::Right);
    maze.add_wall(5, 6, Direction::Down);

    EXPECT_TRUE(maze.get_wall(5, 5, Direction::Right));
    EXPECT_TRUE(maze.get_wall(5, 6, Direction::Left));
    EXPECT_TRUE(maze.get_wall(5, 6, Direction::Down));
    EXPECT_FALSE(maze.get_wall(5, 5, Direction::Down));
}

// ============================================================================
// Perimeter / idempotence
// ============================================================================

// Re-adding perimeter walls must be a safe, idempotent operation
TEST(Maze, OuterBoundarySafety) {
    Maze maze;

    maze.add_wall(0, 0, Direction::Up);
    maze.add_wall(0, 0, Direction::Left);
    maze.add_wall(maze_size - 1, maze_size - 1, Direction::Down);
    maze.add_wall(maze_size - 1, maze_size - 1, Direction::Right);

    EXPECT_TRUE(maze.get_wall(0, 0, Direction::Up));
    EXPECT_TRUE(maze.get_wall(0, 0, Direction::Left));
    EXPECT_TRUE(maze.get_wall(maze_size - 1, maze_size - 1, Direction::Down));
    EXPECT_TRUE(maze.get_wall(maze_size - 1, maze_size - 1, Direction::Right));
}

// Re-adding the whole perimeter must not create or remove any other wall.
TEST(Maze, ReAddingPerimeterChangesNothingElse) {
    Maze maze;
    for (size_t k = 0; k < maze_size; ++k) {
        maze.add_wall(0, k, Direction::Up);
        maze.add_wall(maze_size - 1, k, Direction::Down);
        maze.add_wall(k, 0, Direction::Left);
        maze.add_wall(k, maze_size - 1, Direction::Right);
    }

    Maze reference;
    for (size_t i = 0; i < maze_size; ++i)
        for (size_t j = 0; j < maze_size; ++j)
            for (Direction d : all_directions)
                EXPECT_EQ(maze.get_wall(i, j, d), reference.get_wall(i, j, d));
}

TEST(Maze, AddWallTwiceIsIdempotent) {
    Maze maze;
    maze.add_wall(4, 4, Direction::Right);
    maze.add_wall(4, 4, Direction::Right);

    EXPECT_TRUE(maze.get_wall(4, 4, Direction::Right));
    EXPECT_TRUE(maze.get_wall(4, 5, Direction::Left));
    EXPECT_EQ(count_walls(maze, 4, 4), 1u);
    EXPECT_EQ(count_walls(maze, 4, 5), 1u);
}

// Adding the same shared wall from both sides is also idempotent.
TEST(Maze, AddWallFromBothSidesIsIdempotent) {
    Maze maze;
    maze.add_wall(4, 4, Direction::Right);
    maze.add_wall(4, 5, Direction::Left);

    EXPECT_TRUE(maze.get_wall(4, 4, Direction::Right));
    EXPECT_TRUE(maze.get_wall(4, 5, Direction::Left));
    EXPECT_EQ(count_walls(maze, 4, 4), 1u);
    EXPECT_EQ(count_walls(maze, 4, 5), 1u);
}

// ============================================================================
// Instances
// ============================================================================

TEST(Maze, InstancesAreIndependent) {
    Maze a;
    Maze b;
    a.add_wall(4, 4, Direction::Right);

    EXPECT_TRUE(a.get_wall(4, 4, Direction::Right));
    EXPECT_FALSE(b.get_wall(4, 4, Direction::Right));
}

TEST(Maze, CopyKeepsWallsAndIsIndependent) {
    Maze original;
    original.add_wall(4, 4, Direction::Right);

    Maze copy = original;
    EXPECT_TRUE(copy.get_wall(4, 4, Direction::Right));
    EXPECT_TRUE(copy.get_wall(4, 5, Direction::Left));

    copy.add_wall(7, 7, Direction::Down);
    EXPECT_FALSE(original.get_wall(7, 7, Direction::Down));
}

// Building a checkerboard of walls and reading it all back.
TEST(Maze, ManyWallsReadBackCorrectly) {
    Maze maze;
    for (size_t i = 0; i + 1 < maze_size; ++i)
        for (size_t j = 0; j + 1 < maze_size; ++j)
            if ((i + j) % 2 == 0)
                maze.add_wall(i, j, Direction::Right);

    for (size_t i = 0; i + 1 < maze_size; ++i) {
        for (size_t j = 0; j + 1 < maze_size; ++j) {
            bool expected = ((i + j) % 2 == 0);
            EXPECT_EQ(maze.get_wall(i, j, Direction::Right), expected)
                << "(" << i << "," << j << ")";
            EXPECT_EQ(maze.get_wall(i, j + 1, Direction::Left), expected)
                << "(" << i << "," << j + 1 << ")";
        }
    }
}
