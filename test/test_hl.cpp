#include "ratatech/hl/graph.hpp"
#include "ratatech/hl/maze.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <iostream>
#include <optional>
#include <print>
#include <unordered_set>
#include <vector>

using namespace ratatech::hl;

// ============================================================================
// Maze Edge Cases
// ============================================================================

// Verify default initialization (boundary walls present, interior empty)
TEST(Maze, DefaultState) {
    Maze maze;
    constexpr size_t last = Maze::maze_size - 1;

    // Outer boundary walls at the corners
    EXPECT_TRUE(maze.get_wall(0, 0, Direction::Left));
    EXPECT_TRUE(maze.get_wall(0, 0, Direction::Up));
    EXPECT_TRUE(maze.get_wall(last, last, Direction::Right));
    EXPECT_TRUE(maze.get_wall(last, last, Direction::Down));

    // Interior walls are open by default
    EXPECT_FALSE(maze.get_wall(0, 0, Direction::Right));
    EXPECT_FALSE(maze.get_wall(0, 0, Direction::Down));
    EXPECT_FALSE(maze.get_wall(5, 5, Direction::Up));
    EXPECT_FALSE(maze.get_wall(5, 5, Direction::Left));
}

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

// Re-adding perimeter walls must be a safe, idempotent operation
TEST(Maze, OuterBoundarySafety) {
    Maze maze;
    constexpr size_t last = Maze::maze_size - 1;

    maze.add_wall(0, 0, Direction::Up);
    maze.add_wall(0, 0, Direction::Left);
    maze.add_wall(last, last, Direction::Down);
    maze.add_wall(last, last, Direction::Right);

    EXPECT_TRUE(maze.get_wall(0, 0, Direction::Up));
    EXPECT_TRUE(maze.get_wall(0, 0, Direction::Left));
}

// ============================================================================
// Graph Edge Cases: Neighbors
// ============================================================================

// Corner cells on an open maze have exactly 2 valid neighbors
TEST(GraphNeighbors, CornerCells) {
    Maze maze;
    Graph graph(maze);
    constexpr size_t last = Maze::maze_size - 1;

    auto top_left = graph.neighbors({0, 0});
    EXPECT_EQ(top_left.size(), 2u);

    auto bottom_right = graph.neighbors({last, last});
    EXPECT_EQ(bottom_right.size(), 2u);
}

// Fully walled-in isolated cell has 0 neighbors
TEST(GraphNeighbors, IsolatedCell) {
    Maze maze;
    size_t i = 3, j = 3;

    maze.add_wall(i, j, Direction::Left);
    maze.add_wall(i, j, Direction::Right);
    maze.add_wall(i, j, Direction::Up);
    maze.add_wall(i, j, Direction::Down);

    Graph graph(maze);
    auto nbrs = graph.neighbors({i, j});

    EXPECT_TRUE(nbrs.empty());
}

// ============================================================================
// Graph Edge Cases: Shortest Path
// ============================================================================

// Start node is already in the destination set
TEST(ShortestPath, StartIsGoal) {
    Maze maze;
    Graph graph(maze);

    Graph::Node start{2, 3};
    std::unordered_set<Graph::Node> targets{{2, 3}, {4, 5}};

    auto path = graph.shortest_path(start, targets).value();
    ASSERT_EQ(path.size(), 1u);
    EXPECT_EQ(path.front(), start);
}

// Destination set is completely empty
TEST(ShortestPath, EmptyTargets) {
    Maze maze;
    Graph graph(maze);

    Graph::Node start{0, 0};
    std::unordered_set<Graph::Node> targets{};

    auto path = graph.shortest_path(start, targets);
    EXPECT_EQ(path, std::nullopt);
}

// Destination is physically unreachable (enclosed by walls)
TEST(ShortestPath, UnreachableDestination) {
    Maze maze;
    // Box in cell (1, 1) completely
    maze.add_wall(1, 1, Direction::Up);
    maze.add_wall(1, 1, Direction::Down);
    maze.add_wall(1, 1, Direction::Left);
    maze.add_wall(1, 1, Direction::Right);

    Graph graph(maze);
    Graph::Node start{0, 0};
    std::unordered_set<Graph::Node> targets{{1, 1}};

    auto path = graph.shortest_path(start, targets);
    EXPECT_EQ(path, std::nullopt);
}

// Multi-target selection: picks the strictly closest target
TEST(ShortestPath, SelectsClosestTarget) {
    Maze maze;
    Graph graph(maze);

    Graph::Node start{0, 0};
    Graph::Node near_target{0, 2}; // Manhattan dist = 2
    Graph::Node far_target{5, 5};  // Manhattan dist = 10

    std::unordered_set<Graph::Node> targets{near_target, far_target};

    auto path = graph.shortest_path(start, targets).value();
    ASSERT_FALSE(path.empty());
    EXPECT_EQ(path.back(), near_target);
    EXPECT_EQ(path.size(), 3u); // (0,0) -> (0,1) -> (0,2)
}

// Detour routing around a barrier
TEST(ShortestPath, DetourAroundWall) {
    Maze maze;
    // Wall between (0,0) and (0,1)
    maze.add_wall(0, 0, Direction::Right);

    Graph graph(maze);
    Graph::Node start{0, 0};
    std::unordered_set<Graph::Node> targets{{0, 1}};

    auto path = graph.shortest_path(start, targets).value();

    // Must divert downwards: (0,0) -> (1,0) -> (1,1) -> (0,1)
    ASSERT_EQ(path.size(), 4u); // ASSERT: the lines below index into path
    EXPECT_EQ(path[1], (Graph::Node{1, 0}));
    EXPECT_EQ(path[2], (Graph::Node{1, 1}));
    EXPECT_EQ(path[3], (Graph::Node{0, 1}));
}
