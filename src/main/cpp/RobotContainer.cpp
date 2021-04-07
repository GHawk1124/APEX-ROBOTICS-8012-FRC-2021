#include "RobotContainer.h"

#include <utility>

#include <frc/controller/PIDController.h>
#include <frc/controller/RamseteController.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RamseteCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/button/JoystickButton.h>

#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/Path.h>
#include <wpi/SmallString.h>

#include "Constants.h"
#include "commands/DefaultDrive.h"

using namespace DriveConstants;

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Add commands to the autonomous command chooser
  // m_chooser.SetDefaultOption("defaultAuto", &m_defaultAuto);
  // m_chooser.AddOption("Complex Auto", &m_complexAuto);

  // Put the chooser on the dashboard
  frc::Shuffleboard::GetTab("Autonomous").Add(m_chooser);

  // Calibrate Gyro
  wpi::outs() << "Calibrating Gyro"
              << "\n";
  m_drive.calibrateGyro();
  wpi::outs() << "Calibrated Gyro"
              << "\n";

  // Configure the button bindings
  ConfigureButtonBindings();

  // Set up default drive command
  m_drive.SetDefaultCommand(DefaultDrive(
      &m_drive, [this] { return m_driverController.GetZ(); },
      [this] { return m_driverController.GetY(); }));
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here

  frc2::JoystickButton(&m_driverController, OIConstants::kShooterButton)
      .WhenHeld(&m_spinUp, 1)
      .WhenReleased(&m_stopShooter);
  frc2::JoystickButton(&m_driverController, OIConstants::kIntakeButton)
      .WhenHeld(&m_intakeBall)
      .WhenReleased(&m_stopIntake);
  frc2::JoystickButton(&m_driverController, OIConstants::kIndexButton)
      .WhenHeld(&m_indexBall)
      .WhenReleased(&m_stopIndex);
}

frc2::Command *RobotContainer::GetAutonomousCommand() {
  /*   // Create a voltage constraint to ensure we don't accelerate too fast
    frc::DifferentialDriveVoltageConstraint autoVoltageConstraint(
        frc::SimpleMotorFeedforward<units::meters>(
            DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
        DriveConstants::kDriveKinematics, DriveConstants::kMaxDriveVoltage); */

  /*   // Set up config for trajectory
    frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                                 AutoConstants::kMaxAcceleration);
    // Add kinematics to ensure max speed is actually obeyed
    config.SetKinematics(DriveConstants::kDriveKinematics);
    // Apply the voltage constraint
    config.AddConstraint(autoVoltageConstraint); */

  /*      // An example trajectory to follow.  All units in meters.
      auto Trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
          frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
          {frc::Translation2d(1_m, 1_m), frc::Translation2d(2_m, -1_m)},
          frc::Pose2d(3_m, 0_m, frc::Rotation2d(0_deg)), config);  */

  frc2::RamseteCommand ramseteCommand1(
      m_drive.Trajectory1, [this]() { return m_drive.GetPose(); },
      frc::RamseteController(AutoConstants::kRamseteB,
                             AutoConstants::kRamseteZeta),
      frc::SimpleMotorFeedforward<units::meters>(ks, kv, ka),
      DriveConstants::kDriveKinematics,
      [this] { return m_drive.GetWheelSpeeds(); },
      frc2::PIDController(kPDriveVel, kIDriveVel, kDDriveVel),
      frc2::PIDController(kPDriveVel, kIDriveVel, kDDriveVel),
      [this](auto left, auto right) { m_drive.TankDriveVolts(-left, -right); },
      {&m_drive});

  frc2::RamseteCommand ramseteCommand2(
      m_drive.Trajectory2, [this]() { return m_drive.GetPose(); },
      frc::RamseteController(AutoConstants::kRamseteB,
                             AutoConstants::kRamseteZeta),
      frc::SimpleMotorFeedforward<units::meters>(ks, kv, ka),
      DriveConstants::kDriveKinematics,
      [this] { return m_drive.GetWheelSpeeds(); },
      frc2::PIDController(kPDriveVel, kIDriveVel, kDDriveVel),
      frc2::PIDController(kPDriveVel, kIDriveVel, kDDriveVel),
      [this](auto left, auto right) { m_drive.TankDriveVolts(-right, -left); },
      {&m_drive});

  frc2::RamseteCommand ramseteCommand3(
      m_drive.Trajectory3, [this]() { return m_drive.GetPose(); },
      frc::RamseteController(AutoConstants::kRamseteB,
                             AutoConstants::kRamseteZeta),
      frc::SimpleMotorFeedforward<units::meters>(ks, kv, ka),
      DriveConstants::kDriveKinematics,
      [this] { return m_drive.GetWheelSpeeds(); },
      frc2::PIDController(kPDriveVel, kIDriveVel, kDDriveVel),
      frc2::PIDController(kPDriveVel, kIDriveVel, kDDriveVel),
      [this](auto left, auto right) { m_drive.TankDriveVolts(-left, -right); },
      {&m_drive});

  frc2::RamseteCommand ramseteCommand4(
      m_drive.Trajectory4, [this]() { return m_drive.GetPose(); },
      frc::RamseteController(AutoConstants::kRamseteB,
                             AutoConstants::kRamseteZeta),
      frc::SimpleMotorFeedforward<units::meters>(ks, kv, ka),
      DriveConstants::kDriveKinematics,
      [this] { return m_drive.GetWheelSpeeds(); },
      frc2::PIDController(kPDriveVel, kIDriveVel, kDDriveVel),
      frc2::PIDController(kPDriveVel, kIDriveVel, kDDriveVel),
      [this](auto left, auto right) { m_drive.TankDriveVolts(-right, -left); },
      {&m_drive});

  return new frc2::SequentialCommandGroup(
      frc2::InstantCommand([this] { m_drive.ResetOdometry1(); }, {}),
      std::move(ramseteCommand1),
      frc2::InstantCommand([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {}),
      frc2::InstantCommand([this] { m_drive.ResetOdometry2(); }, {}),
      frc2::InstantCommand([this] { m_drive.reverseMotors(true); }, {}),
      std::move(ramseteCommand2),
      frc2::InstantCommand([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {}),
      frc2::InstantCommand([this] { m_drive.ResetOdometry3(); }, {}),
      frc2::InstantCommand([this] { m_drive.reverseMotors(false); }, {}),
      std::move(ramseteCommand3),
      frc2::InstantCommand([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {}),
      frc2::InstantCommand([this] { m_drive.ResetOdometry4(); }, {}),
      frc2::InstantCommand([this] { m_drive.reverseMotors(true); }, {}),
      std::move(ramseteCommand4),
      frc2::InstantCommand([this] { m_drive.reverseMotors(false); }, {}),
      frc2::InstantCommand([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {}));
}
