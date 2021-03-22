#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/PIDCommand.h>

#include "Constants.h"

class DriveDistance
    : public frc2::CommandHelper<frc2::PIDCommand, DriveDistance> {
 public:
  DriveDistance();

  bool IsFinished() override;
};
