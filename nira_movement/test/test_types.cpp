#include <gtest/gtest.h>

#include <numbers>
#include <type_traits>

#include "nira_utils/body_types.hpp"

using namespace nira_geometry::units;

static_assert(!std::is_constructible_v<nira_geometry::CPosition, Angle, Angle, Angle>);
static_assert(!std::is_constructible_v<nira_geometry::COrientation, Length, Length, Length>);
static_assert(!std::is_constructible_v<nira_geometry::CLegAngles, Length, Length, Length>);
static_assert(!std::is_assignable_v<decltype(nira_geometry::CPosition::x)&, double>);
static_assert(!std::is_assignable_v<decltype(nira_geometry::CLegAngles::torso_coxa)&, double>);
static_assert(!std::is_assignable_v<decltype(nira_geometry::CLegAngles::torso_coxa)&, Length>);
static_assert(!std::is_assignable_v<decltype(nira_geometry::CTorsoCenterOffset::psi)&, Length>);

TEST(MovementTypesTest, MembersAcceptCompatibleUnits) {
    nira_geometry::CPosition position;
    position.x = 125.0 * mm;
    EXPECT_DOUBLE_EQ(position.x.numerical_value_in(m), 0.125);

    nira_geometry::CLegAngles angles;
    angles.torso_coxa = std::numbers::pi / 2.0 * rad;
    angles.coxa_femur = 45.0 * deg;
    const auto halfway = angles.linearInterpolate(nira_geometry::CLegAngles(), 0.5);
    EXPECT_NEAR(halfway.torso_coxa.numerical_value_in(deg), 45.0, 1e-12);
    EXPECT_DOUBLE_EQ(halfway.coxa_femur.numerical_value_in(deg), 22.5);

    nira_geometry::CTorsoCenterOffset offset;
    offset.x = 50.0 * mm;
    offset.psi = std::numbers::pi * rad;
    EXPECT_DOUBLE_EQ(offset.x.numerical_value_in(m), 0.050);
    EXPECT_NEAR(offset.psi.numerical_value_in(deg), 180.0, 1e-12);
}

TEST(MovementTypesTest, PositionToleranceHasLengthUnits) {
    const nira_geometry::CPosition origin;
    const nira_geometry::CPosition near(0.5 * mm, 0.0 * m, 0.0 * m);
    EXPECT_TRUE(nira_geometry::CPosition::almostEqual(origin, near, 1.0 * mm));
    EXPECT_FALSE(nira_geometry::CPosition::almostEqual(origin, near, 0.1 * mm));
}

TEST(MovementTypesTest, PositionAcceptsMixedLengthUnits) {
    const nira_geometry::CPosition position(50.0 * mm, -0.1 * m, 25.0 * mm);
    EXPECT_DOUBLE_EQ(position.x.numerical_value_in(nira_geometry::units::m), 0.050);
    EXPECT_DOUBLE_EQ(position.y.numerical_value_in(nira_geometry::units::m), -0.1);
    EXPECT_DOUBLE_EQ(position.z.numerical_value_in(nira_geometry::units::m), 0.025);

    const auto halfway =
        position.linearInterpolate(nira_geometry::CPosition(150.0 * mm, 0.1 * m, -25.0 * mm), 0.5);
    EXPECT_NEAR(halfway.x.numerical_value_in(nira_geometry::units::m), 0.1, 1e-12);
    EXPECT_DOUBLE_EQ(halfway.y.numerical_value_in(nira_geometry::units::m), 0.0);
    EXPECT_DOUBLE_EQ(halfway.z.numerical_value_in(nira_geometry::units::m), 0.0);
}

TEST(MovementTypesTest, OrientationAndLegAnglesAcceptMixedAngleUnits) {
    const nira_geometry::COrientation orientation(std::numbers::pi / 2.0 * rad, -45.0 * deg, 0.0 * rad);
    EXPECT_NEAR(orientation.roll.numerical_value_in(nira_geometry::units::deg), 90.0, 1e-12);
    EXPECT_DOUBLE_EQ(orientation.pitch.numerical_value_in(nira_geometry::units::deg), -45.0);
    EXPECT_DOUBLE_EQ(orientation.yaw.numerical_value_in(nira_geometry::units::deg), 0.0);

    const nira_geometry::CLegAngles angles(-std::numbers::pi / 2.0 * rad, 45.0 * deg,
                                              std::numbers::pi * rad);
    EXPECT_NEAR(angles.torso_coxa.numerical_value_in(nira_geometry::units::deg), -90.0, 1e-12);
    EXPECT_DOUBLE_EQ(angles.coxa_femur.numerical_value_in(nira_geometry::units::deg), 45.0);
    EXPECT_NEAR(angles.femur_tibia.numerical_value_in(nira_geometry::units::deg), 180.0, 1e-12);
}

TEST(MovementTypesTest, PoseConvertsLengthsAndAngles) {
    const nira_geometry::CPose pose(100.0 * mm, 0.2 * m, -50.0 * mm, 0.0 * rad, 90.0 * deg,
                                       -std::numbers::pi * rad);
    EXPECT_DOUBLE_EQ(pose.position.x.numerical_value_in(nira_geometry::units::m), 0.1);
    EXPECT_DOUBLE_EQ(pose.position.y.numerical_value_in(nira_geometry::units::m), 0.2);
    EXPECT_DOUBLE_EQ(pose.position.z.numerical_value_in(nira_geometry::units::m), -0.05);
    EXPECT_DOUBLE_EQ(pose.orientation.roll.numerical_value_in(nira_geometry::units::deg), 0.0);
    EXPECT_DOUBLE_EQ(pose.orientation.pitch.numerical_value_in(nira_geometry::units::deg), 90.0);
    EXPECT_NEAR(pose.orientation.yaw.numerical_value_in(nira_geometry::units::deg), -180.0, 1e-12);
}

TEST(MovementTypesTest, LegNamesUseLowercaseSnakeCase) {
    EXPECT_EQ(nira_geometry::legNameToIndex("right_front"), nira_geometry::ELegIndex::RightFront);
    EXPECT_EQ(nira_geometry::legNameToIndex("left_mid"), nira_geometry::ELegIndex::LeftMid);
    for (auto index : magic_enum::enum_values<nira_geometry::ELegIndex>()) {
        EXPECT_EQ(nira_geometry::legNameToIndex(nira_geometry::legIndexToName(index)), index);
    }
    EXPECT_FALSE(nira_geometry::parseLegIndex("unknown_leg").has_value());
}
