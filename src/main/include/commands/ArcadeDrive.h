#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/DriveTrain.h"

class ArcadeDrive
    : public frc2::CommandHelper<frc2::CommandBase, DriveTrain> {
public:
  ArcadeDrive(DriveTrain *subsystem, std::function<double()> forward,
              std::function<double()> rotation);

  void Execute() override;

private:
  DriveTrain *m_drive;
  std::function<double()> m_forward;
  std::function<double()> m_rotation;
};