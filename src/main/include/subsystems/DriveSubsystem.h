// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/ADXRS450_Gyro.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc2/command/SubsystemBase.h>

#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/Path.h>
#include <wpi/SmallString.h>

#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>

#include "ctre/Phoenix.h"

#include "Constants.h"

class DriveSubsystem : public frc2::SubsystemBase {
public:
  DriveSubsystem();

  void Periodic() override;

  void ArcadeDrive(double fwd, double rot);

  void ResetEncoders();

  frc::Pose2d GetPose();

  frc::DifferentialDriveWheelSpeeds GetWheelSpeeds();

  void ResetOdometry(frc::Pose2d pose);

  void TankDriveVolts(units::volt_t left, units::volt_t right);

  void calibrateGyro();

  void reverseMotors(bool current);

  void ResetOdometry1();
  void ResetOdometry2();
  void ResetOdometry3();
  void ResetOdometry4();

private:
  WPI_TalonFX m_LF;
  WPI_TalonFX m_LB;
  WPI_TalonFX m_RF;
  WPI_TalonFX m_RB;

  // The motors on the left side of the drive
  frc::SpeedControllerGroup m_leftMotors{m_LF, m_LB};

  // The motors on the right side of the drive
  frc::SpeedControllerGroup m_rightMotors{m_RF, m_RB};

  // The robot's drive
  frc::DifferentialDrive m_drive{m_leftMotors, m_rightMotors};

  // The gyro sensor
  frc::ADXRS450_Gyro m_gyro;

  // Odometry class for tracking robot pose
  frc::DifferentialDriveOdometry m_odometry;

  double leftWheelAccel = 0;
  double rightWheelAccel = 0;
  double prevleftWheelSpeed = 0;
  double prevrightWheelSpeed = 0;

  double ldownScale = 1;
  double rdownScale = 1;

  bool driveBackwards = false;

public:
  frc::Trajectory Trajectory1;
  frc::Trajectory Trajectory2;
  frc::Trajectory Trajectory3;
  frc::Trajectory Trajectory4;
};
