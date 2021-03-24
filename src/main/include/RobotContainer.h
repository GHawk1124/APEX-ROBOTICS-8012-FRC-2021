#pragma once

#include <frc/Joystick.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>

#include "Constants.h"

#include "subsystems/DriveSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"

class RobotContainer {
public:
  RobotContainer();

  frc2::Command *GetAutonomousCommand();

private:
  // The robot's subsystems and commands are defined here...

  // The robot's subsystems
  DriveSubsystem m_drive;
  IntakeSubsystem m_intake;
  ShooterSubsystem m_shooter;

  // The autonomous routines
  /* DriveDistance m_simpleAuto{AutoConstants::kAutoDriveDistanceInches,
                             AutoConstants::kAutoDriveSpeed, &m_drive};
  ComplexAuto m_complexAuto{&m_drive, &m_hatch}; */
  //DefaultAuto m_defaultAuto{&m_drive, &m_intake, &m_shooter};

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command *> m_chooser;

  // The driver's controller
  frc::Joystick m_driverController{OIConstants::kDriverControllerPort};

  void ConfigureButtonBindings();
};
