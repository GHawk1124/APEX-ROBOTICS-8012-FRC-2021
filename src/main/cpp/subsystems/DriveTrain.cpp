#include "subsystems/DriveTrain.h"
#include "Constants.h"

DriveTrain::DriveTrain() = default;

void DriveTrain::Periodic() {}

void DriveTrain::Drive(double forwardAxis, double turnAxis) {
  double z = (forwardAxis > DEADZONE || forwardAxis < -DEADZONE)
                 ? (forwardAxis > 0 ? forwardAxis * forwardAxis
                                    : -1 * forwardAxis * forwardAxis)
                 : 0;
  double y =
      (turnAxis > DEADZONE || turnAxis < -DEADZONE)
          ? (turnAxis > 0 ? turnAxis * turnAxis : -1 * turnAxis * turnAxis)
          : 0;
  LF.Set(ControlMode::PercentOutput, z - y);
  LB.Set(ControlMode::PercentOutput, z - y);
  RF.Set(ControlMode::PercentOutput, z + y);
  RB.Set(ControlMode::PercentOutput, z + y);
}
