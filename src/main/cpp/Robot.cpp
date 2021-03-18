#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>

void Robot::RobotInit() {
  imu.Calibrate();

  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

// ! Called EVERY robot packet, regardless of mode.
void Robot::RobotPeriodic() { frc2::CommandScheduler::GetInstance().Run(); }

void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  LF.Set(ControlMode::PercentOutput, m_stick->GetZ() - m_stick->GetY());
  LB.Set(ControlMode::PercentOutput, m_stick->GetZ() - m_stick->GetY());
  RF.Set(ControlMode::PercentOutput, m_stick->GetZ() + m_stick->GetY());
  RB.Set(ControlMode::PercentOutput, m_stick->GetZ() + m_stick->GetY());

  intake.Set(m_stick->GetRawButton(2));
  index.Set(m_stick->GetRawButton(3));
  shooter.Set(m_stick->GetRawButton(1));
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
