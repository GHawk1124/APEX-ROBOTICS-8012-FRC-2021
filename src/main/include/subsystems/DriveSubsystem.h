// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include "ctre/Phoenix.h"

#include "Constants.h"

class DriveSubsystem : public frc2::SubsystemBase {
public:
  DriveSubsystem();

  void Periodic() override;

  void ArcadeDrive(double fwd, double rot);

  void ResetEncoders();

private:
  TalonFX m_LF;
  TalonFX m_LB;
  TalonFX m_RF;
  TalonFX m_RB;
};
