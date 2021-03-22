#pragma once

namespace AutonConstants {
constexpr double kDrivekP = 0;
constexpr double kDrivekI = 0;
constexpr double kDrivekD = 0;
} // namespace AutonConstants

namespace DriveConstants {
constexpr uint8_t kLeftFrontPort = 1;
constexpr uint8_t kLeftBackPort = 1;
constexpr uint8_t kRightFrontPort = 1;
constexpr uint8_t kRightBackPort = 1;
} // namespace DriveConstants

namespace OIConstants {
constexpr double kDeadzone = 0.05;
constexpr uint8_t kIntakeButton = 2;
constexpr uint8_t kIndexButton = 3;
constexpr uint8_t kShooterButton = 1;
constexpr uint8_t kDriverControllerPort = 1;
} // namespace OIConstants
