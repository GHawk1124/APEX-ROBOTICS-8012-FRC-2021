#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/trajectory/constraint/DifferentialDriveKinematicsConstraint.h>
#include <units/acceleration.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <wpi/math>

#pragma once

namespace AutoConstants {
constexpr auto kMaxSpeed = 1_mps;
constexpr auto kMaxAcceleration = 1_mps_sq;

// Reasonable baseline values for a RAMSETE follower in units of meters and
// seconds
constexpr double kRamseteB = 2;
constexpr double kRamseteZeta = 0.7;
} // namespace AutoConstants

namespace DriveConstants {
namespace Ports {
constexpr uint8_t kLeftFrontPort = 2;
constexpr uint8_t kLeftBackPort = 3;
constexpr uint8_t kRightFrontPort = 4;
constexpr uint8_t kRightBackPort = 1;

constexpr uint8_t kIntakePort = 5;
constexpr uint8_t kIndexPort = 6;
constexpr uint8_t kShooterPort = 7;
} // namespace Ports

constexpr double kTurnSensitivityCutoff = 0.9;
constexpr double kTurnSensitivity = 0.5;

constexpr auto kTrackwidth = 0.59_m;
extern const frc::DifferentialDriveKinematics kDriveKinematics;

constexpr int kEncoderCPR = 2048;
constexpr double kWheelDiameterInches = 6.25;
constexpr double kExternalGearRatio = 10.7142857143;
constexpr double kEncoderDistancePerPulse =
    // Assumes the encoders are directly mounted on the wheel shafts
    (kWheelDiameterInches * wpi::math::pi) /
    (static_cast<double>(kEncoderCPR) * kExternalGearRatio);

constexpr auto ks = 0.22_V;
// Volts * Seconds / meter - Velocity Constant
constexpr auto kv = 1.98 * 1_V * 1_s / 1_m;
// Volts * Seconds^2 / meter - Acceleration Constant
constexpr auto ka = 0.2 * 1_V * 1_s * 1_s / 1_m;

// Example value only - as above, this must be tuned for your drive!
constexpr double kPDriveVel = 0.1;
} // namespace DriveConstants

namespace OIConstants {
constexpr double kDeadzone = 0.05;
constexpr uint8_t kIntakeButton = 2;
constexpr uint8_t kIndexButton = 3;
constexpr uint8_t kShooterButton = 1;
constexpr uint8_t kDriverControllerPort = 0;
constexpr uint8_t kDriverController2Port = 1;

constexpr double kIndexSpeed = 1;
constexpr double kIntakeSpeed = 1;
constexpr double kShooterSpeed = 0.75;
} // namespace OIConstants
