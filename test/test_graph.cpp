#include "ratatech/hl/graph.hpp"
#include "ratatech/hl/maze.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdlib>
#include <optional>
#include <unordered_set>
#include <vector>

using namespace ratatech::hl;

namespace {

constexpr size_t last = maze_size - 1;
static_assert(maze_size >= 8, "tests assume a maze of at least 8x8 cells");

// Direction convention: Right -> j + 1, Left -> j - 1, Down -> i + 1,
// Up -> i - 1 (same as in the maze tests).
bool has_neighbor(size_t i, size_t j, Direction d) {
    switch (d) {
    case Direction::Left:
        return j > 0;
    case Direction::Right:
        return j < last;
    case Direction::Up:
        return i > 0;
    case Direction::Down:
        return i < last;
    }
    return false;
}

Graph::Node neighbor_of(size_t i, size_t j, Direction d) {
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

// Nodes reachable in one step according to the maze (reference oracle).
std::unordered_set<Graph::Node> expected_neighbors(const Maze &maze, size_t i,
                                                   size_t j) {
    std::unordered_set<Graph::Node> result;
    for (Direction d : all_directions)
        if (has_neighbor(i, j, d) && !maze.get_wall(i, j, d))
            result.insert(neighbor_of(i, j, d));
    return result;
}

template <typename Range>
std::unordered_set<Graph::Node> to_set(const Range &r) {
    return {std::begin(r), std::end(r)};
}

template <typename Range> bool contains(const Range &r, const Graph::Node &n) {
    return std::find(std::begin(r), std::end(r), n) != std::end(r);
}

// neighbors() returns (node, cost) pairs; most tests only care about nodes.
std::vector<Graph::Node> neighbor_nodes(const Graph &graph,
                                        const Graph::Node &node) {
    std::vector<Graph::Node> out;
    for (const auto &[n, cost] : graph.neighbors(node))
        out.push_back(n);
    return out;
}

// Wall between column j and column j + 1 on every row except gap_row.
void add_vertical_barrier(Maze &maze, size_t j, size_t gap_row) {
    for (size_t i = 0; i <= last; ++i)
        if (i != gap_row)
            maze.add_wall(i, j, Direction::Right);
}

// Deterministic "random-looking" maze for exhaustive checks.
Maze patterned_maze() {
    Maze maze;
    for (size_t i = 0; i <= last; ++i) {
        for (size_t j = 0; j <= last; ++j) {
            if (j < last && (i * 7 + j * 3) % 5 == 0)
                maze.add_wall(i, j, Direction::Right);
            if (i < last && (i * 5 + j * 11) % 7 == 0)
                maze.add_wall(i, j, Direction::Down);
        }
    }
    return maze;
}

// A single step between two nodes: adjacent and not blocked by a wall.
bool valid_step(const Maze &maze, const Graph::Node &a, const Graph::Node &b) {
    long di = static_cast<long>(b.i) - static_cast<long>(a.i);
    long dj = static_cast<long>(b.j) - static_cast<long>(a.j);
    if (std::labs(di) + std::labs(dj) != 1)
        return false;

    Direction d = di == 1    ? Direction::Down
                  : di == -1 ? Direction::Up
                  : dj == 1  ? Direction::Right
                             : Direction::Left;
    return !maze.get_wall(a.i, a.j, d);
}

// Path starts at `start`, ends in `targets`, every step is legal, and no node
// is visited twice.
void expect_valid_path(const Maze &maze, const std::vector<Graph::Node> &path,
                       const Graph::Node &start,
                       const std::unordered_set<Graph::Node> &targets) {
    ASSERT_FALSE(path.empty());
    EXPECT_EQ(path.front(), start);
    EXPECT_TRUE(targets.contains(path.back()));

    for (size_t k = 1; k < path.size(); ++k)
        EXPECT_TRUE(valid_step(maze, path[k - 1], path[k]))
            << "illegal step at index " << k;

    std::unordered_set<Graph::Node> seen(path.begin(), path.end());
    EXPECT_EQ(seen.size(), path.size()) << "path revisits a node";
}

size_t manhattan(const Graph::Node &a, const Graph::Node &b) {
    auto d = [](size_t x, size_t y) { return x > y ? x - y : y - x; };
    return d(a.i, b.i) + d(a.j, b.j);
}

} // namespace

// ============================================================================
// Graph Edge Cases: Neighbors
// ============================================================================

// Corner cells on an open maze have exactly 2 valid neighbors
TEST(GraphNeighbors, CornerCells) {
    Maze maze;
    Graph graph(maze);

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

// Which neighbors, not just how many.
TEST(GraphNeighbors, CornerCellNeighborIdentity) {
    Maze maze;
    Graph graph(maze);

    EXPECT_EQ(to_set(neighbor_nodes(graph, {0, 0})),
              (std::unordered_set<Graph::Node>{{0, 1}, {1, 0}}));
    EXPECT_EQ(to_set(neighbor_nodes(graph, {0, last})),
              (std::unordered_set<Graph::Node>{{0, last - 1}, {1, last}}));
    EXPECT_EQ(to_set(neighbor_nodes(graph, {last, 0})),
              (std::unordered_set<Graph::Node>{{last - 1, 0}, {last, 1}}));
    EXPECT_EQ(
        to_set(neighbor_nodes(graph, {last, last})),
        (std::unordered_set<Graph::Node>{{last - 1, last}, {last, last - 1}}));
}

TEST(GraphNeighbors, EdgeCellsHaveThreeNeighbors) {
    Maze maze;
    Graph graph(maze);

    for (size_t k = 1; k < last; ++k) {
        EXPECT_EQ(graph.neighbors({0, k}).size(), 3u);
        EXPECT_EQ(graph.neighbors({last, k}).size(), 3u);
        EXPECT_EQ(graph.neighbors({k, 0}).size(), 3u);
        EXPECT_EQ(graph.neighbors({k, last}).size(), 3u);
    }
}

TEST(GraphNeighbors, InteriorCellHasFourNeighbors) {
    Maze maze;
    Graph graph(maze);

    auto nbrs = neighbor_nodes(graph, {5, 5});
    EXPECT_EQ(to_set(nbrs), (std::unordered_set<Graph::Node>{
                                {4, 5}, {6, 5}, {5, 4}, {5, 6}}));
}

TEST(GraphNeighbors, SingleWallRemovesOnlyThatNeighbor) {
    for (Direction blocked : all_directions) {
        Maze maze;
        maze.add_wall(5, 5, blocked);
        Graph graph(maze);

        auto nbrs = neighbor_nodes(graph, {5, 5});
        EXPECT_EQ(nbrs.size(), 3u);
        EXPECT_FALSE(contains(nbrs, neighbor_of(5, 5, blocked)));
    }
}

// A wall blocks movement in both directions.
TEST(GraphNeighbors, WallBlocksBothDirections) {
    Maze maze;
    maze.add_wall(4, 4, Direction::Right);
    Graph graph(maze);

    EXPECT_FALSE(contains(neighbor_nodes(graph, {4, 4}), Graph::Node{4, 5}));
    EXPECT_FALSE(contains(neighbor_nodes(graph, {4, 5}), Graph::Node{4, 4}));
}

TEST(GraphNeighbors, EnclosedCellIsNotANeighborOfAnyone) {
    Maze maze;
    for (Direction d : all_directions)
        maze.add_wall(3, 3, d);
    Graph graph(maze);

    for (Direction d : all_directions) {
        auto nbrs = neighbor_nodes(graph, neighbor_of(3, 3, d));
        EXPECT_FALSE(contains(nbrs, Graph::Node{3, 3}));
    }
}

TEST(GraphNeighbors, NeverContainsSelf) {
    Maze maze;
    Graph graph(maze);
    for (size_t i = 0; i <= last; ++i)
        for (size_t j = 0; j <= last; ++j)
            EXPECT_FALSE(
                contains(neighbor_nodes(graph, {i, j}), Graph::Node{i, j}));
}

// Exhaustive check against the maze walls on a non-trivial maze.
TEST(GraphNeighbors, MatchesMazeWallsEverywhere) {
    Maze maze = patterned_maze();
    Graph graph(maze);

    for (size_t i = 0; i <= last; ++i) {
        for (size_t j = 0; j <= last; ++j) {
            auto nbrs = neighbor_nodes(graph, {i, j});
            EXPECT_EQ(to_set(nbrs), expected_neighbors(maze, i, j))
                << "cell (" << i << "," << j << ")";
            // no duplicates
            EXPECT_EQ(to_set(nbrs).size(), nbrs.size())
                << "cell (" << i << "," << j << ")";
        }
    }
}

// If b is a neighbor of a, a is a neighbor of b.
TEST(GraphNeighbors, RelationIsSymmetric) {
    Maze maze = patterned_maze();
    Graph graph(maze);

    for (size_t i = 0; i <= last; ++i) {
        for (size_t j = 0; j <= last; ++j) {
            for (const Graph::Node &n : neighbor_nodes(graph, {i, j}))
                EXPECT_TRUE(
                    contains(neighbor_nodes(graph, n), Graph::Node{i, j}))
                    << "(" << i << "," << j << ") -> (" << n.i << "," << n.j
                    << ")";
        }
    }
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

// --- length on an open maze -------------------------------------------------

TEST(ShortestPath, AdjacentTargetGivesTwoNodePath) {
    Maze maze;
    Graph graph(maze);

    Graph::Node start{4, 4};
    std::unordered_set<Graph::Node> targets{{4, 5}};
    auto path = graph.shortest_path(start, targets).value();

    ASSERT_EQ(path.size(), 2u);
    EXPECT_EQ(path[0], start);
    EXPECT_EQ(path[1], (Graph::Node{4, 5}));
}

TEST(ShortestPath, CornerToCornerLength) {
    Maze maze;
    Graph graph(maze);

    Graph::Node start{0, 0};
    std::unordered_set<Graph::Node> targets{{last, last}};
    auto path = graph.shortest_path(start, targets).value();

    EXPECT_EQ(path.size(), 2 * last + 1);
    expect_valid_path(maze, path, start, targets);
}

TEST(ShortestPath, OpenMazeLengthIsManhattanDistance) {
    Maze maze;
    Graph graph(maze);

    const std::array<std::pair<Graph::Node, Graph::Node>, 5> cases{{
        {{0, 0}, {3, 4}},
        {{7, 2}, {1, 6}},
        {{5, 5}, {5, 0}},
        {{last, 0}, {0, last}},
        {{2, 6}, {2, 6}},
    }};

    for (const auto &[a, b] : cases) {
        std::unordered_set<Graph::Node> targets{b};
        auto path = graph.shortest_path(a, targets).value();
        EXPECT_EQ(path.size(), manhattan(a, b) + 1)
            << "(" << a.i << "," << a.j << ") -> (" << b.i << "," << b.j << ")";
        expect_valid_path(maze, path, a, targets);
    }
}

TEST(ShortestPath, LengthIsSymmetricBetweenEndpoints) {
    Maze maze = patterned_maze();
    Graph graph(maze);

    Graph::Node a{1, 1};
    Graph::Node b{last - 1, last - 2};

    auto ab = graph.shortest_path(a, {b});
    auto ba = graph.shortest_path(b, {a});
    ASSERT_EQ(ab.has_value(), ba.has_value());
    if (ab)
        EXPECT_EQ(ab->size(), ba->size());
}

// --- validity ---------------------------------------------------------------

TEST(ShortestPath, PathNeverCrossesWalls) {
    Maze maze = patterned_maze();
    Graph graph(maze);

    Graph::Node start{0, 0};
    std::unordered_set<Graph::Node> targets{{last, last}};
    auto path = graph.shortest_path(start, targets);

    // The patterned maze may or may not connect the corners; if it does, the
    // path must be legal.
    if (path)
        expect_valid_path(maze, *path, start, targets);
}

// --- multiple targets -------------------------------------------------------

TEST(ShortestPath, EquidistantTargetsReturnsOneOfThem) {
    Maze maze;
    Graph graph(maze);

    Graph::Node start{5, 5};
    std::unordered_set<Graph::Node> targets{{5, 8}, {5, 2}, {2, 5}, {8, 5}};
    auto path = graph.shortest_path(start, targets).value();

    EXPECT_EQ(path.size(), 4u);
    expect_valid_path(maze, path, start, targets);
}

// The closest target by Manhattan distance is walled off: the target that is
// closest *by path length* must be chosen instead.
TEST(ShortestPath, ClosestTargetIsByPathNotByManhattanDistance) {
    Maze maze;
    // Barrier between columns 3 and 4, only passable on row 6.
    add_vertical_barrier(maze, 3, 6);
    Graph graph(maze);

    Graph::Node start{0, 2};
    Graph::Node across_barrier{0, 5}; // Manhattan 3, but ~15 steps by path
    Graph::Node same_side{5, 0};      // Manhattan 7, exactly 7 steps by path
    std::unordered_set<Graph::Node> targets{across_barrier, same_side};

    auto path = graph.shortest_path(start, targets).value();
    EXPECT_EQ(path.back(), same_side);
    EXPECT_EQ(path.size(), 8u);
    expect_valid_path(maze, path, start, targets);
}

TEST(ShortestPath, ReachableTargetChosenOverUnreachableOne) {
    Maze maze;
    for (Direction d : all_directions)
        maze.add_wall(1, 1, d); // (1,1) unreachable
    Graph graph(maze);

    Graph::Node start{0, 0};
    Graph::Node reachable{6, 6};
    std::unordered_set<Graph::Node> targets{{1, 1}, reachable};

    auto path = graph.shortest_path(start, targets).value();
    EXPECT_EQ(path.back(), reachable);
    expect_valid_path(maze, path, start, targets);
}

TEST(ShortestPath, AllTargetsUnreachable) {
    Maze maze;
    for (Direction d : all_directions) {
        maze.add_wall(1, 1, d);
        maze.add_wall(6, 6, d);
    }
    Graph graph(maze);

    std::unordered_set<Graph::Node> targets{{1, 1}, {6, 6}};
    EXPECT_EQ(graph.shortest_path({0, 0}, targets), std::nullopt);
}

// --- walls and barriers -----------------------------------------------------

// Barrier with a single gap: the path must go through the gap.
TEST(ShortestPath, PathGoesThroughTheOnlyGap) {
    Maze maze;
    const size_t gap_row = 2;
    add_vertical_barrier(maze, 3, gap_row);
    Graph graph(maze);

    Graph::Node start{0, 0};
    std::unordered_set<Graph::Node> targets{{0, last}};
    auto path = graph.shortest_path(start, targets).value();

    // Direct distance is `last`; the detour down to the gap and back adds
    // 2 * gap_row steps.
    EXPECT_EQ(path.size(), last + 2 * gap_row + 1);
    expect_valid_path(maze, path, start, targets);

    // It crosses between columns 3 and 4 exactly on the gap row.
    bool crossed_on_gap = false;
    for (size_t k = 1; k < path.size(); ++k) {
        if (path[k - 1].j == 3 && path[k].j == 4) {
            EXPECT_EQ(path[k].i, gap_row);
            crossed_on_gap = true;
        }
    }
    EXPECT_TRUE(crossed_on_gap);
}

// Barrier with no gap at all splits the maze into two disconnected halves.
TEST(ShortestPath, ClosedBarrierMakesTargetUnreachable) {
    Maze maze;
    for (size_t i = 0; i <= last; ++i)
        maze.add_wall(i, 3, Direction::Right);
    Graph graph(maze);

    std::unordered_set<Graph::Node> targets{{0, last}};
    EXPECT_EQ(graph.shortest_path({0, 0}, targets), std::nullopt);

    // The other half is still fully reachable from within.
    std::unordered_set<Graph::Node> same_side{{last, 3}};
    EXPECT_TRUE(graph.shortest_path({0, 0}, same_side).has_value());
}

// The start cell is enclosed, so no target can be reached...
TEST(ShortestPath, StartEnclosedWithOutsideTarget) {
    Maze maze;
    for (Direction d : all_directions)
        maze.add_wall(3, 3, d);
    Graph graph(maze);

    std::unordered_set<Graph::Node> targets{{5, 5}};
    EXPECT_EQ(graph.shortest_path({3, 3}, targets), std::nullopt);
}

// ...unless the start is itself a target.
TEST(ShortestPath, StartEnclosedButIsTarget) {
    Maze maze;
    for (Direction d : all_directions)
        maze.add_wall(3, 3, d);
    Graph graph(maze);

    std::unordered_set<Graph::Node> targets{{3, 3}};
    auto path = graph.shortest_path({3, 3}, targets).value();
    ASSERT_EQ(path.size(), 1u);
    EXPECT_EQ(path.front(), (Graph::Node{3, 3}));
}

// Winding corridor: a snake that forces a long path.
TEST(ShortestPath, SnakeForcesLongPath) {
    Maze maze;
    // Horizontal barriers on rows 0..5 (between row r and r+1), each with one
    // gap alternating at the right and left ends -> serpentine.
    const size_t rows = 6;
    for (size_t r = 0; r < rows; ++r) {
        size_t gap = (r % 2 == 0) ? last : 0;
        for (size_t j = 0; j <= last; ++j)
            if (j != gap)
                maze.add_wall(r, j, Direction::Down);
    }
    Graph graph(maze);

    Graph::Node start{0, 0};
    std::unordered_set<Graph::Node> targets{{rows, last}};
    auto path = graph.shortest_path(start, targets).value();

    // Each of the `rows` corridors has to be traversed end to end.
    EXPECT_GE(path.size(), rows * last);
    expect_valid_path(maze, path, start, targets);
}

// --- reuse ------------------------------------------------------------------

TEST(ShortestPath, RepeatedQueriesGiveSameLength) {
    Maze maze = patterned_maze();
    Graph graph(maze);

    std::unordered_set<Graph::Node> targets{{last, 0}};
    auto first = graph.shortest_path({0, 0}, targets);
    auto second = graph.shortest_path({0, 0}, targets);

    ASSERT_EQ(first.has_value(), second.has_value());
    if (first)
        EXPECT_EQ(first->size(), second->size());
}

TEST(ShortestPath, GraphIsReusableAcrossDifferentStarts) {
    Maze maze;
    Graph graph(maze);
    std::unordered_set<Graph::Node> targets{{last, last}};

    EXPECT_EQ(graph.shortest_path({0, 0}, targets)->size(), 2 * last + 1);
    EXPECT_EQ(graph.shortest_path({last, 0}, targets)->size(), last + 1);
    EXPECT_EQ(graph.shortest_path({last, last}, targets)->size(), 1u);
}

// ============================================================================
// Costs and Graph/Maze lifetime
// ============================================================================

// Every edge currently costs 1.
TEST(GraphNeighbors, AllEdgeCostsAreOne) {
    Maze maze = patterned_maze();
    Graph graph(maze);

    for (size_t i = 0; i <= last; ++i)
        for (size_t j = 0; j <= last; ++j)
            for (const auto &[n, cost] : graph.neighbors({i, j}))
                EXPECT_EQ(cost, 1);
}

// Graph keeps a *reference* to the Maze, so walls added after construction
// are seen by the graph. This documents the current behaviour.
TEST(Graph, SeesWallsAddedAfterConstruction) {
    Maze maze;
    Graph graph(maze);
    EXPECT_EQ(graph.neighbors({4, 4}).size(), 4u);

    maze.add_wall(4, 4, Direction::Right);
    EXPECT_EQ(graph.neighbors({4, 4}).size(), 3u);

    std::unordered_set<Graph::Node> targets{{4, 5}};
    auto path = graph.shortest_path({4, 4}, targets).value();
    EXPECT_GT(path.size(), 2u); // direct step is now blocked
}
