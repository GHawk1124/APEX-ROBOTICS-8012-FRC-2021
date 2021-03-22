#include "commands/DriveDistance.h"

using namespace AutonConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
DriveDistance::DriveDistance()
    : CommandHelper(
          frc2::PIDController(kDrivekP, kDrivekI, kDrivekD),
          // This should return the measurement
          [] { return 0; },
          // This should return the setpoint (can also be a constant)
          [] { return 0; },
          // This uses the output
          [](double output) {
            // Use the output here
          }) {}

// Returns true when the command should end.
bool DriveDistance::IsFinished() {
  return false;
}
