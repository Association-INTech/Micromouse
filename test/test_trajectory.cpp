#include "ratatech/hl/trajectory.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <optional>
#include <vector>

using namespace ratatech::hl;

namespace {

constexpr float kTol = 1e-5f;
constexpr float kStop = 1.0f;  // CurveStraight::stop_duration
constexpr float kSpeed = 1.0f; // CurveStraight::speed

void expect_position(const StateVector &s, float x, float y) {
    EXPECT_NEAR(s.x, x, kTol);
    EXPECT_NEAR(s.y, y, kTol);
}

} // namespace

// ============================================================================
// Trajectory: construction edge cases
// ============================================================================

TEST(Trajectory, SingleZeroLengthEdgeStaysAtNode) {
    Trajectory<1> traj({Edge<1>({{2, 3}}, {{2, 3}}, 1)});
    auto s = traj.state(0.0f).value();
    expect_position(s, 2 * cell_size, 3 * cell_size);
}

TEST(Trajectory, SingleZeroLengthEdgeWithinStopDuration) {
    Trajectory<1> traj({Edge<1>({{2, 3}}, {{2, 3}}, 1)});
    auto s = traj.state(0.5f).value();
    expect_position(s, 2 * cell_size, 3 * cell_size);
    EXPECT_NEAR(s.omega, 0.0f, kTol);
}

TEST(Trajectory, EmptyTrajectoryIsNullopt) {
    Trajectory<1> traj({});
    auto s = traj.state(0.0f);
    EXPECT_EQ(s, std::nullopt);
}

// ============================================================================
// Trajectory: two-node path  (Start -> Straight -> End)
// ============================================================================

class TrajectoryTwoNodes : public ::testing::Test {
  protected:
    // 3 cells along j.
    std::vector<Edge<1>> path{Edge<1>({{0, 0}}, {{0, 3}}, 1)};
    float t_end_straight = kStop + 3 * cell_size / kSpeed;
};

TEST_F(TrajectoryTwoNodes, AtTimeZeroIsAtStart) {
    Trajectory<1> traj(path);
    expect_position(traj.state(0.0f).value(), 0.0f, 0.0f);
}

TEST_F(TrajectoryTwoNodes, DuringMotionIsBetweenEndpoints) {
    Trajectory<1> traj(path);
    float t = kStop + 0.5f * 3 * cell_size / kSpeed;
    auto s = traj.state(t).value();
    EXPECT_NEAR(s.x, 0.0f, kTol);
    EXPECT_NEAR(s.y, 1.5f * cell_size, kTol);
    EXPECT_NEAR(s.speed, kSpeed, kTol);
}

TEST_F(TrajectoryTwoNodes, WellAfterEndTime) {
    Trajectory<1> traj(path);
    auto s = traj.state(t_end_straight + 100.0f);
    EXPECT_EQ(s, std::nullopt);
}

TEST_F(TrajectoryTwoNodes, ExactlyAtEndTimeIsAtGoal) {
    Trajectory<1> traj(path);
    expect_position(traj.state(t_end_straight).value(), 0.0f, 3 * cell_size);
}

TEST_F(TrajectoryTwoNodes, NegativeTimeIsNullopt) {
    Trajectory<1> traj(path);
    auto s = traj.state(-5.0f);
    EXPECT_EQ(s, std::nullopt);
}

// ============================================================================
// Trajectory: multi-node path
// ============================================================================

class TrajectoryMultiNode : public ::testing::Test {
  protected:
    // L-shape: (0,0) -> (0,2) -> (2,2) -> (2,0)
    std::vector<Edge<1>> path{Edge<1>({{0, 0}}, {{0, 2}}, 1),
                              Edge<1>({{0, 2}}, {{2, 2}}, 1),
                              Edge<1>({{2, 2}}, {{2, 0}}, 1)};
    float leg = kStop + 2 * cell_size / kSpeed; // duration of each segment
};

TEST_F(TrajectoryMultiNode, StartPosition) {
    Trajectory<1> traj(path);
    expect_position(traj.state(0.0f).value(), 0.0f, 0.0f);
}

TEST_F(TrajectoryMultiNode, SegmentEndpointsAreVisitedInOrder) {
    Trajectory<1> traj(path);
    expect_position(traj.state(1 * leg).value(), 0.0f, 2 * cell_size);
    expect_position(traj.state(2 * leg).value(), 2 * cell_size, 2 * cell_size);
    expect_position(traj.state(3 * leg).value(), 2 * cell_size, 0.0f);
}

TEST_F(TrajectoryMultiNode, MidSegmentHeadingsFollowEachLeg) {
    Trajectory<1> traj(path);
    float mid = kStop + cell_size / kSpeed; // halfway through moving phase
    EXPECT_NEAR(traj.state(0 * leg + mid).value().theta,
                std::numbers::pi_v<float> / 2, kTol);
    EXPECT_NEAR(traj.state(1 * leg + mid).value().theta, 0.0f, kTol);
    EXPECT_NEAR(traj.state(2 * leg + mid).value().theta,
                -std::numbers::pi_v<float> / 2, kTol);
}

// Sequentially increasing queries, like a real control loop.
TEST_F(TrajectoryMultiNode, ForwardSweepIsContinuous) {
    Trajectory<1> traj(path);
    float max_step = 2 * cell_size; // generous bound per dt
    StateVector prev = traj.state(0.0f).value();
    for (float t = 0.01f;; t += 0.01f) {
        auto result = traj.state(t);
        if (result == std::nullopt)
            break;
        auto s = result.value();
        EXPECT_LE(std::hypot(s.x - prev.x, s.y - prev.y), max_step) << t;
        prev = s;
    }
}

TEST_F(TrajectoryMultiNode, JumpingForwardAndBackwardRepeatedly) {
    Trajectory<1> traj(path);
    for (int k = 0; k < 3; ++k) {
        expect_position(traj.state(2 * leg).value(), 2 * cell_size,
                        2 * cell_size);
        expect_position(traj.state(1 * leg).value(), 0.0f, 2 * cell_size);
        expect_position(traj.state(3 * leg).value(), 2 * cell_size, 0.0f);
        expect_position(traj.state(0.0f).value(), 0.0f, 0.0f);
    }
}

TEST_F(TrajectoryMultiNode, RepeatedQuerySameTimeIsStable) {
    Trajectory<1> traj(path);
    float t = 1.5f * leg;
    auto a = traj.state(t).value();
    auto b = traj.state(t).value();
    EXPECT_FLOAT_EQ(a.x, b.x);
    EXPECT_FLOAT_EQ(a.y, b.y);
    EXPECT_FLOAT_EQ(a.theta, b.theta);
}

TEST_F(TrajectoryMultiNode, TwoTrajectoriesAreIndependent) {
    Trajectory<1> a(path);
    Trajectory<1> b(path);
    (void)a.state(3 * leg).value();
    expect_position(b.state(0.0f).value(), 0.0f, 0.0f);
}
