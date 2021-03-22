#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "subsystems/DriveSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"

class DefaultAuto
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 DefaultAuto> {
 public:
  DefaultAuto(DriveSubsystem* drive, IntakeSubsystem* intake, ShooterSubsystem* shooter);
};
