#include "ratatech/hl/curve.hpp"
#include "ratatech/hl/graph.hpp"
#include "ratatech/hl/trajectory.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>
#include <vector>

using namespace ratatech::hl;

namespace {

constexpr float kTol = 1e-5f;
constexpr float kInf = std::numeric_limits<float>::infinity();
constexpr float kStop = 1.0f;  // CurveStraight::stop_duration
constexpr float kSpeed = 1.0f; // CurveStraight::speed

void expect_state(const StateVector &s, float x, float y, float theta,
                  float speed, float omega) {
    EXPECT_NEAR(s.x, x, kTol);
    EXPECT_NEAR(s.y, y, kTol);
    EXPECT_NEAR(s.theta, theta, kTol);
    EXPECT_NEAR(s.speed, speed, kTol);
    EXPECT_NEAR(s.omega, omega, kTol);
}

void expect_position(const StateVector &s, float x, float y) {
    EXPECT_NEAR(s.x, x, kTol);
    EXPECT_NEAR(s.y, y, kTol);
}

} // namespace

// ============================================================================
// Trajectory: construction edge cases
// ============================================================================

TEST(Trajectory, SingleNodePathStaysAtNode) {
    Trajectory traj({{2, 3}});
    auto s = traj.state(0.0f);
    expect_position(s, 2 * cell_size, 3 * cell_size);
}

TEST(Trajectory, SingleNodePathWithinStopDuration) {
    Trajectory traj({{2, 3}});
    auto s = traj.state(0.5f);
    expect_position(s, 2 * cell_size, 3 * cell_size);
    EXPECT_NEAR(s.omega, 0.0f, kTol);
}

// Path of length 1 builds a single CurveStraight of duration stop_duration;
// querying past it must not run off the end of the curve list.
TEST(Trajectory, SingleNodePathQueriedAfterItsEnd) {
    Trajectory traj({{2, 3}});
    auto s = traj.state(10.0f);
    expect_position(s, 2 * cell_size, 3 * cell_size);
}

// ============================================================================
// Trajectory: two-node path  (Start -> Straight -> End)
// ============================================================================

class TrajectoryTwoNodes : public ::testing::Test {
  protected:
    // 3 cells along j.
    std::vector<Graph::Node> path{{0, 0}, {0, 3}};
    float t_end_straight = kStop + 3 * cell_size / kSpeed;
};

TEST_F(TrajectoryTwoNodes, AtTimeZeroIsAtStart) {
    Trajectory traj(path);
    expect_position(traj.state(0.0f), 0.0f, 0.0f);
}

TEST_F(TrajectoryTwoNodes, DuringMotionIsBetweenEndpoints) {
    Trajectory traj(path);
    float t = kStop + 0.5f * 3 * cell_size / kSpeed;
    auto s = traj.state(t);
    EXPECT_NEAR(s.x, 0.0f, kTol);
    EXPECT_NEAR(s.y, 1.5f * cell_size, kTol);
    EXPECT_NEAR(s.speed, kSpeed, kTol);
}

TEST_F(TrajectoryTwoNodes, WellAfterEndIsAtGoalAndStationary) {
    Trajectory traj(path);
    auto s = traj.state(t_end_straight + 100.0f);
    expect_position(s, 0.0f, 3 * cell_size);
    EXPECT_NEAR(s.speed, 0.0f, kTol);
    EXPECT_NEAR(s.omega, 0.0f, kTol);
}

TEST_F(TrajectoryTwoNodes, ExactlyAtEndTimeIsAtGoal) {
    Trajectory traj(path);
    expect_position(traj.state(t_end_straight), 0.0f, 3 * cell_size);
}

TEST_F(TrajectoryTwoNodes, NegativeTimeIsAtStartAndStationary) {
    Trajectory traj(path);
    auto s = traj.state(-5.0f);
    expect_position(s, 0.0f, 0.0f);
    EXPECT_NEAR(s.speed, 0.0f, kTol);
}

// ============================================================================
// Trajectory: multi-node path
// ============================================================================

class TrajectoryMultiNode : public ::testing::Test {
  protected:
    // L-shape: (0,0) -> (0,2) -> (2,2) -> (2,0)
    std::vector<Graph::Node> path{{0, 0}, {0, 2}, {2, 2}, {2, 0}};
    float leg = kStop + 2 * cell_size / kSpeed; // duration of each segment
};

TEST_F(TrajectoryMultiNode, StartPosition) {
    Trajectory traj(path);
    expect_position(traj.state(0.0f), 0.0f, 0.0f);
}

TEST_F(TrajectoryMultiNode, SegmentEndpointsAreVisitedInOrder) {
    Trajectory traj(path);
    expect_position(traj.state(1 * leg), 0.0f, 2 * cell_size);
    expect_position(traj.state(2 * leg), 2 * cell_size, 2 * cell_size);
    expect_position(traj.state(3 * leg), 2 * cell_size, 0.0f);
}

TEST_F(TrajectoryMultiNode, MidSegmentHeadingsFollowEachLeg) {
    Trajectory traj(path);
    float mid = kStop + cell_size / kSpeed; // halfway through moving phase
    EXPECT_NEAR(traj.state(0 * leg + mid).theta, std::numbers::pi_v<float> / 2,
                kTol);
    EXPECT_NEAR(traj.state(1 * leg + mid).theta, 0.0f, kTol);
    EXPECT_NEAR(traj.state(2 * leg + mid).theta, -std::numbers::pi_v<float> / 2,
                kTol);
}

TEST_F(TrajectoryMultiNode, AfterEndStaysAtGoal) {
    Trajectory traj(path);
    expect_position(traj.state(3 * leg + 50.0f), 2 * cell_size, 0.0f);
}

// Sequentially increasing queries, like a real control loop.
TEST_F(TrajectoryMultiNode, ForwardSweepIsContinuous) {
    Trajectory traj(path);
    float max_step = 2 * cell_size; // generous bound per dt
    StateVector prev = traj.state(0.0f);
    for (float t = 0.01f; t <= 3 * leg + 1.0f; t += 0.01f) {
        StateVector s = traj.state(t);
        EXPECT_LE(std::hypot(s.x - prev.x, s.y - prev.y), max_step) << t;
        prev = s;
    }
}

// Seeking backwards must walk the cached index back down.
TEST_F(TrajectoryMultiNode, BackwardQueryAfterForward) {
    Trajectory traj(path);
    expect_position(traj.state(3 * leg + 10.0f), 2 * cell_size, 0.0f);
    expect_position(traj.state(0.0f), 0.0f, 0.0f);
}

TEST_F(TrajectoryMultiNode, JumpingForwardAndBackwardRepeatedly) {
    Trajectory traj(path);
    for (int k = 0; k < 3; ++k) {
        expect_position(traj.state(2 * leg), 2 * cell_size, 2 * cell_size);
        expect_position(traj.state(1 * leg), 0.0f, 2 * cell_size);
        expect_position(traj.state(3 * leg), 2 * cell_size, 0.0f);
        expect_position(traj.state(0.0f), 0.0f, 0.0f);
    }
}

TEST_F(TrajectoryMultiNode, RepeatedQuerySameTimeIsStable) {
    Trajectory traj(path);
    float t = 1.5f * leg;
    auto a = traj.state(t);
    auto b = traj.state(t);
    EXPECT_FLOAT_EQ(a.x, b.x);
    EXPECT_FLOAT_EQ(a.y, b.y);
    EXPECT_FLOAT_EQ(a.theta, b.theta);
}

TEST_F(TrajectoryMultiNode, TwoTrajectoriesAreIndependent) {
    Trajectory a(path);
    Trajectory b(path);
    (void)a.state(3 * leg + 10.0f);
    expect_position(b.state(0.0f), 0.0f, 0.0f);
}

// ============================================================================
// Trajectory: path with a repeated node (zero-length segment)
// ============================================================================

TEST(Trajectory, RepeatedNodeInPath) {
    Trajectory traj({{0, 0}, {0, 1}, {0, 1}, {0, 2}});
    expect_position(traj.state(1000.0f), 0.0f, 2 * cell_size);
}

// ============================================================================
// Trajectory: empty path
// ============================================================================

// Empty path produces no curves, so state() has nothing to evaluate. The
// expected behaviour is a design decision (throw / default state); this
// documents that construction itself is safe.
TEST(Trajectory, EmptyPathConstructs) {
    EXPECT_NO_THROW(Trajectory traj(std::vector<Graph::Node>{}));
}
