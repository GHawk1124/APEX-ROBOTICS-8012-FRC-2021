// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/DriveSubsystem.h"

class DefaultDrive
    : public frc2::CommandHelper<frc2::CommandBase, DefaultDrive> {
public:
  DefaultDrive(DriveSubsystem *subsystem, std::function<double()> forward,
               std::function<double()> rotation);

  void Execute() override;

private:
  DriveSubsystem *m_drive;
  std::function<double()> m_forward;
  std::function<double()> m_rotation;
};
