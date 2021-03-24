#include "subsystems/DriveSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>

using namespace DriveConstants;

DriveSubsystem::DriveSubsystem()
    : m_LF{kLeftFrontPort}, m_LB{kLeftBackPort}, m_RF{kRightFrontPort},
      m_RB{kRightBackPort}, m_odometry{m_gyro.GetRotation2d()} {
  /*  m_LF.ConfigPulseWidthPeriod_EdgesPerRot(kEncoderDistancePerPulse);
        m_LB.ConfigPulseWidthPeriod_EdgesPerRot(kEncoderDistancePerPulse);
        m_RF.ConfigPulseWidthPeriod_EdgesPerRot(kEncoderDistancePerPulse);
        m_RB.ConfigPulseWidthPeriod_EdgesPerRot(kEncoderDistancePerPulse); */

  ResetEncoders();
}

void DriveSubsystem::Periodic() {
  auto leftSideEncoders =
      (m_LF.GetSelectedSensorPosition() + m_LB.GetSelectedSensorPosition()) / 2;
  auto rightSideEncoders =
      (m_RF.GetSelectedSensorPosition() + m_RB.GetSelectedSensorPosition()) / 2;
  m_odometry.Update(m_gyro.GetRotation2d(), units::meter_t(leftSideEncoders),
                    units::meter_t(rightSideEncoders));
}

void DriveSubsystem::ArcadeDrive(double fwd, double rot) {
  m_LF.Set(ControlMode::PercentOutput, fwd - rot);
  m_LB.Set(ControlMode::PercentOutput, fwd - rot);
  m_RF.Set(ControlMode::PercentOutput, fwd + rot);
  m_RB.Set(ControlMode::PercentOutput, fwd + rot);
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
  return {units::meters_per_second_t(leftRate),
          units::meters_per_second_t(rightRate)};
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  ResetEncoders();
  m_odometry.ResetPosition(pose, m_gyro.GetRotation2d());
}

void DriveSubsystem::TankDriveVolts(units::volt_t left, units::volt_t right) {
  m_leftMotors.SetVoltage(left);
  m_rightMotors.SetVoltage(-right);
  m_drive.Feed();
}