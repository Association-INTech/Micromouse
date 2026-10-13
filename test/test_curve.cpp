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
// CurveStraight: timing
// ============================================================================

TEST(CurveStraight, StartTimeIsStored) {
    CurveStraight c(2.5f, {0, 0}, {0, 1});
    EXPECT_FLOAT_EQ(c.start_time(), 2.5f);
}

TEST(CurveStraight, EndTimeAxisAligned) {
    // 3 cells along j: stop + 3 * cell_size / speed
    CurveStraight c(2.0f, {0, 0}, {0, 3});
    EXPECT_NEAR(c.end_time(), 2.0f + kStop + 3 * cell_size / kSpeed, kTol);
}

TEST(CurveStraight, EndTimeDiagonalUsesEuclideanDistance) {
    // 3-4-5 triangle
    CurveStraight c(0.0f, {0, 0}, {3, 4});
    EXPECT_NEAR(c.end_time(), kStop + 5 * cell_size / kSpeed, kTol);
}

TEST(CurveStraight, ZeroLengthSegmentLastsOnlyStopDuration) {
    CurveStraight c(1.0f, {2, 2}, {2, 2});
    EXPECT_NEAR(c.end_time(), 1.0f + kStop, kTol);
}

TEST(CurveStraight, EndTimeAfterStartTime) {
    CurveStraight c(0.0f, {1, 1}, {1, 2});
    EXPECT_GT(c.end_time(), c.start_time());
}

// ============================================================================
// CurveStraight: heading
// ============================================================================

TEST(CurveStraight, ThetaIncreasingI) {
    CurveStraight c(0.0f, {0, 0}, {3, 0});
    EXPECT_NEAR(c.state(0.5f).theta, 0.0f, kTol);
}

TEST(CurveStraight, ThetaIncreasingJ) {
    CurveStraight c(0.0f, {0, 0}, {0, 3});
    EXPECT_NEAR(c.state(0.5f).theta, std::numbers::pi_v<float> / 2, kTol);
}

TEST(CurveStraight, ThetaDiagonal) {
    CurveStraight c(0.0f, {0, 0}, {2, 2});
    EXPECT_NEAR(c.state(0.5f).theta, std::numbers::pi_v<float> / 4, kTol);
}

// Moving toward smaller indices. If Graph::Node::i/j are unsigned,
// `end.i - start.i` wraps around and these fail (heading and duration).
TEST(CurveStraight, ThetaDecreasingI) {
    CurveStraight c(0.0f, {3, 0}, {0, 0});
    EXPECT_NEAR(std::fabs(c.state(0.5f).theta), std::numbers::pi_v<float>,
                kTol);
}

TEST(CurveStraight, ThetaDecreasingJ) {
    CurveStraight c(0.0f, {0, 3}, {0, 0});
    EXPECT_NEAR(c.state(0.5f).theta, -std::numbers::pi_v<float> / 2, kTol);
}

TEST(CurveStraight, EndTimeDecreasingDirectionMatchesIncreasing) {
    CurveStraight fwd(0.0f, {0, 0}, {0, 3});
    CurveStraight bwd(0.0f, {0, 3}, {0, 0});
    EXPECT_NEAR(fwd.end_time(), bwd.end_time(), kTol);
}

TEST(CurveStraight, ThetaConstantOverTime) {
    CurveStraight c(0.0f, {0, 0}, {0, 3});
    float theta = c.state(c.start_time()).theta;
    for (float t = 0.0f; t <= c.end_time(); t += 0.1f)
        EXPECT_NEAR(c.state(t).theta, theta, kTol);
}

// ============================================================================
// CurveStraight: position along the segment
// ============================================================================

TEST(CurveStraight, PositionBeforeStartIsStartPoint) {
    CurveStraight c(5.0f, {1, 2}, {1, 5});
    expect_position(c.state(-10.0f), 1 * cell_size, 2 * cell_size);
    expect_position(c.state(4.0f), 1 * cell_size, 2 * cell_size);
}

TEST(CurveStraight, PositionDuringStopIsStartPoint) {
    CurveStraight c(5.0f, {1, 2}, {1, 5});
    expect_position(c.state(5.0f), 1 * cell_size, 2 * cell_size);
    expect_position(c.state(5.0f + kStop), 1 * cell_size, 2 * cell_size);
}

TEST(CurveStraight, PositionAfterEndIsEndPoint) {
    CurveStraight c(0.0f, {1, 2}, {1, 5});
    expect_position(c.state(c.end_time() + 10.0f), 1 * cell_size,
                    5 * cell_size);
}

// The segment must reach its end point exactly when end_time is reached.
TEST(CurveStraight, PositionAtEndTimeIsEndPoint) {
    CurveStraight c(0.0f, {0, 0}, {0, 4});
    expect_position(c.state(c.end_time()), 0.0f, 4 * cell_size);
}

// Halfway through the moving phase the position is the segment midpoint.
TEST(CurveStraight, PositionAtMovingMidpoint) {
    CurveStraight c(0.0f, {0, 0}, {0, 4});
    float travel = 4 * cell_size / kSpeed;
    expect_position(c.state(kStop + travel / 2), 0.0f, 2 * cell_size);
}

TEST(CurveStraight, PositionDiagonalMidpoint) {
    CurveStraight c(0.0f, {0, 0}, {2, 2});
    float travel = std::hypot(2.0f, 2.0f) * cell_size / kSpeed;
    expect_position(c.state(kStop + travel / 2), 1 * cell_size, 1 * cell_size);
}

TEST(CurveStraight, PositionIsMonotonicAlongAxis) {
    CurveStraight c(0.0f, {0, 0}, {0, 5});
    float prev = c.state(c.start_time()).y;
    for (float t = 0.0f; t <= c.end_time() + 1.0f; t += 0.01f) {
        float y = c.state(t).y;
        EXPECT_GE(y, prev - kTol);
        prev = y;
    }
}

TEST(CurveStraight, PositionStaysWithinSegmentBounds) {
    CurveStraight c(0.0f, {1, 1}, {4, 1});
    for (float t = -1.0f; t <= c.end_time() + 1.0f; t += 0.05f) {
        auto s = c.state(t);
        EXPECT_GE(s.x, 1 * cell_size - kTol);
        EXPECT_LE(s.x, 4 * cell_size + kTol);
        EXPECT_NEAR(s.y, 1 * cell_size, kTol);
    }
}

TEST(CurveStraight, ZeroLengthSegmentStaysPut) {
    CurveStraight c(0.0f, {3, 3}, {3, 3});
    for (float t : {-1.0f, 0.0f, 0.5f, 1.0f, 2.0f})
        expect_position(c.state(t), 3 * cell_size, 3 * cell_size);
}

TEST(CurveStraight, SpeedAndOmega) {
    CurveStraight c(0.0f, {0, 0}, {0, 3});
    // Mid-travel the robot moves at the nominal speed with no rotation.
    auto s = c.state(kStop + 0.5f * 3 * cell_size / kSpeed);
    EXPECT_NEAR(s.speed, kSpeed, kTol);
    EXPECT_NEAR(s.omega, 0.0f, kTol);
}

TEST(CurveStraight, OmegaAlwaysZero) {
    CurveStraight c(0.0f, {0, 0}, {2, 5});
    for (float t = -1.0f; t <= c.end_time() + 1.0f; t += 0.1f)
        EXPECT_FLOAT_EQ(c.state(t).omega, 0.0f);
}

TEST(CurveStraight, StateIsConst) {
    const CurveStraight c(0.0f, {0, 0}, {0, 1});
    EXPECT_NO_THROW((void)c.state(0.5f));
}

// ============================================================================
// CurveStart
// ============================================================================

TEST(CurveStart, StartTimeIsNegativeInfinity) {
    CurveStart c(3.0f, {1, 1});
    EXPECT_EQ(c.start_time(), -kInf);
}

TEST(CurveStart, EndTimeIsStored) {
    CurveStart c(3.0f, {1, 1});
    EXPECT_FLOAT_EQ(c.end_time(), 3.0f);
}

TEST(CurveStart, StateIsStationaryAtNode) {
    CurveStart c(3.0f, {2, 5});
    expect_state(c.state(0.0f), 2 * cell_size, 5 * cell_size, 0, 0, 0);
}

TEST(CurveStart, StateIndependentOfTime) {
    CurveStart c(3.0f, {2, 5});
    for (float t : {-1000.0f, -1.0f, 0.0f, 2.9f, 3.0f, 100.0f}) {
        expect_state(c.state(t), 2 * cell_size, 5 * cell_size, 0, 0, 0);
    }
}

TEST(CurveStart, OriginNode) {
    CurveStart c(0.0f, {0, 0});
    expect_state(c.state(0.0f), 0, 0, 0, 0, 0);
}

// ============================================================================
// CurveEnd
// ============================================================================

TEST(CurveEnd, StartTimeIsStored) {
    CurveEnd c(4.0f, {1, 1});
    EXPECT_FLOAT_EQ(c.start_time(), 4.0f);
}

TEST(CurveEnd, EndTimeIsPositiveInfinity) {
    CurveEnd c(4.0f, {1, 1});
    EXPECT_EQ(c.end_time(), kInf);
}

TEST(CurveEnd, StateIsStationaryAtNode) {
    CurveEnd c(4.0f, {6, 7});
    expect_state(c.state(5.0f), 6 * cell_size, 7 * cell_size, 0, 0, 0);
}

TEST(CurveEnd, StateIndependentOfTime) {
    CurveEnd c(4.0f, {6, 7});
    for (float t : {-1.0f, 4.0f, 5.0f, 1e6f}) {
        expect_state(c.state(t), 6 * cell_size, 7 * cell_size, 0, 0, 0);
    }
}
