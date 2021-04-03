#include "subsystems/DriveSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>

using namespace DriveConstants;
using namespace Ports;

DriveSubsystem::DriveSubsystem()
    : m_LF{kLeftFrontPort}, m_LB{kLeftBackPort}, m_RF{kRightFrontPort},
      m_RB{kRightBackPort}, m_odometry{m_gyro.GetRotation2d()} {
  m_LF.ConfigFactoryDefault();
  m_LB.ConfigFactoryDefault();
  m_RF.ConfigFactoryDefault();
  m_RB.ConfigFactoryDefault();

  m_LF.SetNeutralMode(NeutralMode::Coast);
  m_LB.SetNeutralMode(NeutralMode::Coast);
  m_RF.SetNeutralMode(NeutralMode::Coast);
  m_RB.SetNeutralMode(NeutralMode::Coast);

  ResetEncoders();
}

void DriveSubsystem::Periodic() {
  auto leftSideEncoders =
      (m_LF.GetSelectedSensorPosition() + m_LB.GetSelectedSensorPosition()) / 2;
  auto rightSideEncoders =
      (m_RF.GetSelectedSensorPosition() + m_RB.GetSelectedSensorPosition()) / 2;
  leftSideEncoders = leftSideEncoders*kEncoderDistancePerPulse;
  rightSideEncoders = leftSideEncoders*kEncoderDistancePerPulse;
  m_odometry.Update(m_gyro.GetRotation2d(), units::meter_t(leftSideEncoders),
                    units::meter_t(rightSideEncoders));
}

// * DONE: Add Deadzone
// TODO: Add Traction Control
// * DONE: Change Drive Scaling
void DriveSubsystem::ArcadeDrive(double fwd, double rot) {
  rot = std::fabs(rot) > kTurnSensitivityCutoff ? rot : rot * kTurnSensitivity;
  rot = std::fabs(rot) > OIConstants::kDeadzone ? rot : 0;
  fwd = std::fabs(fwd) > OIConstants::kDeadzone ? fwd : 0;
  m_drive.ArcadeDrive(-1 * fwd, rot);
}

void DriveSubsystem::ResetEncoders() {
  m_LF.SetSelectedSensorPosition(0);
  m_LB.SetSelectedSensorPosition(0);
  m_RF.SetSelectedSensorPosition(0);
  m_RB.SetSelectedSensorPosition(0);
}

frc::Pose2d DriveSubsystem::GetPose() { return m_odometry.GetPose(); }

frc::DifferentialDriveWheelSpeeds DriveSubsystem::GetWheelSpeeds() {
  auto leftRate =
      (m_LF.GetSelectedSensorVelocity() + m_LB.GetSelectedSensorVelocity()) / 2;
  auto rightRate =
      (m_RF.GetSelectedSensorVelocity() + m_RB.GetSelectedSensorVelocity()) / 2;
  leftRate = leftRate*kEncoderDistancePerPulse*10;
  rightRate = rightRate*kEncoderDistancePerPulse*10;
  return {units::meters_per_second_t(leftRate),
          units::meters_per_second_t(rightRate)};
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  ResetEncoders();
  //m_gyro.Reset();
  m_odometry.ResetPosition(pose, m_gyro.GetRotation2d());
}

void DriveSubsystem::TankDriveVolts(units::volt_t left, units::volt_t right) {
  m_leftMotors.SetVoltage(-left);
  m_rightMotors.SetVoltage(right);
  m_drive.Feed();
}

void DriveSubsystem::calibrateGyro() { m_gyro.Calibrate(); }