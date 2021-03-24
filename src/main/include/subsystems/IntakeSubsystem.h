// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include "ctre/Phoenix.h"

class IntakeSubsystem : public frc2::SubsystemBase {
public:
  IntakeSubsystem();

  void Periodic() override;

  void ActivateIntake();

  void outtake();

  void stopMotors();

private:
  WPI_VictorSPX m_intake;
};
