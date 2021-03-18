#pragma once

#include "Constants.h"

#include <frc2/command/Command.h>

#include "commands/Auton.h"

#include "subsystems/DriveTrain.h"
#include "subsystems/Intake.h"
#include "subsystems/Shooter.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
public:
  RobotContainer();

  frc2::Command *GetAutonomousCommand();

private:
  // The robot's subsystems and commands are defined here...
  DriveTrain m_DriveTrain_subsystem;
  Intake m_Intake_subsystem;
  Shooter m_Shooter_subsystem;

  Auton m_autonomousCommand;

  void ConfigureButtonBindings();
};
