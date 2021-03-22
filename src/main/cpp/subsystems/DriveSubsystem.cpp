#include "subsystems/DriveSubsystem.h"

using namespace DriveConstants;

DriveSubsystem::DriveSubsystem()
    : m_LF{kLeftFrontPort}, m_LB{kLeftBackPort}, m_RF{kRightFrontPort},
      m_RB{kRightBackPort} {}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void DriveSubsystem::ArcadeDrive(double fwd, double rot) {
  m_LF.Set(ControlMode::PercentOutput, fwd - rot);
  m_LB.Set(ControlMode::PercentOutput, fwd - rot);
  m_RF.Set(ControlMode::PercentOutput, fwd + rot);
  m_RB.Set(ControlMode::PercentOutput, fwd + rot);
}

void DriveSubsystem::ResetEncoders() {
  // TODO: Reset Encoders
}
