// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include "ctre/Phoenix.h"

class ShooterSubsystem : public frc2::SubsystemBase {
public:
  ShooterSubsystem();

  void Periodic() override;

  void spinUp(double throttle);

  void shoot();

  void stopShooter();

  void stopIndex();

private:
  WPI_VictorSPX m_index;
  WPI_VictorSPX m_shooter;
};
