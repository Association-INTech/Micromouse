#include "ratatech/hl/curve.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

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
// CurveStraight: timing
// ============================================================================

TEST(CurveStraight, StartTimeIsStored) {
    CurveStraight c(Edge<1>({{0, 0}}, {{0, 1}}, 1), 2.5f);
    EXPECT_FLOAT_EQ(c.start_time(), 2.5f);
}

TEST(CurveStraight, EndTimeAxisAligned) {
    // 3 cells along j: stop + 3 * cell_size / speed
    CurveStraight c(Edge<1>({{0, 0}}, {{0, 3}}, 1), 2.0f);
    EXPECT_NEAR(c.end_time(), 2.0f + kStop + 3 * cell_size / kSpeed, kTol);
}

TEST(CurveStraight, EndTimeDiagonalUsesEuclideanDistance) {
    // 3-4-5 triangle
    CurveStraight c(Edge<1>({{0, 0}}, {{3, 4}}, 1), 0.0f);
    EXPECT_NEAR(c.end_time(), kStop + 5 * cell_size / kSpeed, kTol);
}

TEST(CurveStraight, ZeroLengthSegmentLastsOnlyStopDuration) {
    CurveStraight c(Edge<1>({{2, 2}}, {{2, 2}}, 1), 1.0f);
    EXPECT_NEAR(c.end_time(), 1.0f + kStop, kTol);
}

TEST(CurveStraight, EndTimeAfterStartTime) {
    CurveStraight c(Edge<1>({{1, 1}}, {{1, 2}}, 1), 0.0f);
    EXPECT_GT(c.end_time(), c.start_time());
}

// ============================================================================
// CurveStraight: heading
// ============================================================================

TEST(CurveStraight, ThetaIncreasingI) {
    CurveStraight c(Edge<1>({{0, 0}}, {{3, 0}}, 1), 0.0f);
    EXPECT_NEAR(c.state(0.5f).theta, 0.0f, kTol);
}

TEST(CurveStraight, ThetaIncreasingJ) {
    CurveStraight c(Edge<1>({{0, 0}}, {{0, 3}}, 1), 0.0f);
    EXPECT_NEAR(c.state(0.5f).theta, std::numbers::pi_v<float> / 2, kTol);
}

TEST(CurveStraight, ThetaDiagonal) {
    CurveStraight c(Edge<1>({{0, 0}}, {{2, 2}}, 1), 0.0f);
    EXPECT_NEAR(c.state(0.5f).theta, std::numbers::pi_v<float> / 4, kTol);
}

// Moving toward smaller indices checks that direction and duration use the
// difference between Edge<1>(oordinates without unsigned underflow.
TEST(CurveStraight, ThetaDecreasingI) {
    CurveStraight c(Edge<1>({{3, 0}}, {{0, 0}}, 1), 0.0f);
    EXPECT_NEAR(std::fabs(c.state(0.5f).theta), std::numbers::pi_v<float>,
                kTol);
}

TEST(CurveStraight, ThetaDecreasingJ) {
    CurveStraight c(Edge<1>({{0, 3}}, {{0, 0}}, 1), 0.0f);
    EXPECT_NEAR(c.state(0.5f).theta, -std::numbers::pi_v<float> / 2, kTol);
}

TEST(CurveStraight, EndTimeDecreasingDirectionMatchesIncreasing) {
    CurveStraight fwd(Edge<1>({{0, 0}}, {{0, 3}}, 1), 0.0f);
    CurveStraight bwd(Edge<1>({{0, 3}}, {{0, 0}}, 1), 0.0f);
    EXPECT_NEAR(fwd.end_time(), bwd.end_time(), kTol);
}

TEST(CurveStraight, ThetaConstantOverTime) {
    CurveStraight c(Edge<1>({{0, 0}}, {{0, 3}}, 1), 0.0f);
    float theta = c.state(c.start_time()).theta;
    for (float t = 0.0f; t <= c.end_time(); t += 0.1f)
        EXPECT_NEAR(c.state(t).theta, theta, kTol);
}

// ============================================================================
// CurveStraight: position along the segment
// ============================================================================

TEST(CurveStraight, PositionBeforeStartIsStartPoint) {
    CurveStraight c(Edge<1>({{1, 2}}, {{1, 5}}, 1), 5.0f);
    expect_position(c.state(-10.0f), 1 * cell_size, 2 * cell_size);
    expect_position(c.state(4.0f), 1 * cell_size, 2 * cell_size);
}

TEST(CurveStraight, PositionDuringStopIsStartPoint) {
    CurveStraight c(Edge<1>({{1, 2}}, {{1, 5}}, 1), 5.0f);
    expect_position(c.state(5.0f), 1 * cell_size, 2 * cell_size);
    expect_position(c.state(5.0f + kStop), 1 * cell_size, 2 * cell_size);
}

TEST(CurveStraight, PositionAfterEndIsEndPoint) {
    CurveStraight c(Edge<1>({{1, 2}}, {{1, 5}}, 1), 0.0f);
    expect_position(c.state(c.end_time() + 10.0f), 1 * cell_size,
                    5 * cell_size);
}

// The segment must reach its end point exactly when end_time is reached.
TEST(CurveStraight, PositionAtEndTimeIsEndPoint) {
    CurveStraight c(Edge<1>({{0, 0}}, {{0, 4}}, 1), 0.0f);
    expect_position(c.state(c.end_time()), 0.0f, 4 * cell_size);
}

// Halfway through the moving phase the position is the segment midpoint.
TEST(CurveStraight, PositionAtMovingMidpoint) {
    CurveStraight c(Edge<1>({{0, 0}}, {{0, 4}}, 1), 0.0f);
    float travel = 4 * cell_size / kSpeed;
    expect_position(c.state(kStop + travel / 2), 0.0f, 2 * cell_size);
}

TEST(CurveStraight, PositionDiagonalMidpoint) {
    CurveStraight c(Edge<1>({{0, 0}}, {{2, 2}}, 1), 0.0f);
    float travel = std::hypot(2.0f, 2.0f) * cell_size / kSpeed;
    expect_position(c.state(kStop + travel / 2), 1 * cell_size, 1 * cell_size);
}

TEST(CurveStraight, PositionIsMonotonicAlongAxis) {
    CurveStraight c(Edge<1>({{0, 0}}, {{0, 5}}, 1), 0.0f);
    float prev = c.state(c.start_time()).y;
    for (float t = 0.0f; t <= c.end_time() + 1.0f; t += 0.01f) {
        float y = c.state(t).y;
        EXPECT_GE(y, prev - kTol);
        prev = y;
    }
}

TEST(CurveStraight, PositionStaysWithinSegmentBounds) {
    CurveStraight c(Edge<1>({{1, 1}}, {{4, 1}}, 1), 0.0f);
    for (float t = -1.0f; t <= c.end_time() + 1.0f; t += 0.05f) {
        auto s = c.state(t);
        EXPECT_GE(s.x, 1 * cell_size - kTol);
        EXPECT_LE(s.x, 4 * cell_size + kTol);
        EXPECT_NEAR(s.y, 1 * cell_size, kTol);
    }
}

TEST(CurveStraight, ZeroLengthSegmentStaysPut) {
    CurveStraight c(Edge<1>({{3, 3}}, {{3, 3}}, 1), 0.0f);
    for (float t : {-1.0f, 0.0f, 0.5f, 1.0f, 2.0f})
        expect_position(c.state(t), 3 * cell_size, 3 * cell_size);
}

TEST(CurveStraight, SpeedAndOmega) {
    CurveStraight c(Edge<1>({{0, 0}}, {{0, 3}}, 1), 0.0f);
    // Mid-travel the robot moves at the nominal speed with no rotation.
    auto s = c.state(kStop + 0.5f * 3 * cell_size / kSpeed);
    EXPECT_NEAR(s.speed, kSpeed, kTol);
    EXPECT_NEAR(s.omega, 0.0f, kTol);
}

TEST(CurveStraight, OmegaAlwaysZero) {
    CurveStraight c(Edge<1>({{0, 0}}, {{2, 5}}, 1), 0.0f);
    for (float t = -1.0f; t <= c.end_time() + 1.0f; t += 0.1f)
        EXPECT_FLOAT_EQ(c.state(t).omega, 0.0f);
}

TEST(CurveStraight, StateIsConst) {
    const CurveStraight c(Edge<1>({{0, 0}}, {{0, 1}}, 1), 0.0f);
    EXPECT_NO_THROW((void)c.state(0.5f));
}

// ============================================================================
// Curve<1>: current variant wrapper
// ============================================================================

TEST(Curve, ForwardsTimingAndStateToStraightAlternative) {
    Curve<1> curve{CurveStraight(Edge<1>({{0, 0}}, {{0, 2}}, 1), 3.0f)};
    EXPECT_FLOAT_EQ(curve.start_time(), 3.0f);
    EXPECT_NEAR(curve.end_time(), 3.0f + kStop + 2 * cell_size / kSpeed, kTol);
    expect_position(curve.state(curve.end_time()), 0.0f, 2 * cell_size);
}
