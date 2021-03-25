#pragma once

#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>

#include "Constants.h"

#include "subsystems/DriveSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"


// TODO: Add custom frequency for checking controller inputs
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
  // DefaultAuto m_defaultAuto{&m_drive, &m_intake, &m_shooter};

  frc2::InstantCommand m_spinUp{[this] { m_shooter.spinUp(); }, {&m_shooter}};
  frc2::InstantCommand m_indexBall{[this] { m_shooter.shoot(); }, {&m_shooter}};
  frc2::InstantCommand m_intakeBall{[this] { m_intake.ActivateIntake(); },
                                    {&m_intake}};

  frc2::InstantCommand m_stopShooter{[this] { m_shooter.stopShooter(); },
                                     {&m_shooter}};
  frc2::InstantCommand m_stopIndex{[this] { m_shooter.stopIndex(); },
                                   {&m_shooter}};
  frc2::InstantCommand m_stopIntake{[this] { m_intake.stopMotors(); },
                                    {&m_intake}};

  // TODO: Add Commands for Drive Scaling

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command *> m_chooser;

  // The driver's controller
  frc::Joystick m_driverController{OIConstants::kDriverControllerPort};
  frc::XboxController m_driverController2{OIConstants::kDriverController2Port};

  void ConfigureButtonBindings();
};
