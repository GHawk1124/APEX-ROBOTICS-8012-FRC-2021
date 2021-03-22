#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>

void Robot::RobotInit() { /* imu.Calibrate(); */ }

// ! Called EVERY robot packet, regardless of mode.
void Robot::RobotPeriodic() { frc2::CommandScheduler::GetInstance().Run(); }

void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Cancel();
    m_autonomousCommand = nullptr;
  }
}

void Robot::TeleopPeriodic() {
/*   LF.Set(ControlMode::PercentOutput, m_stick->GetZ() - m_stick->GetY());
  LB.Set(ControlMode::PercentOutput, m_stick->GetZ() - m_stick->GetY());
  RF.Set(ControlMode::PercentOutput, m_stick->GetZ() + m_stick->GetY());
  RB.Set(ControlMode::PercentOutput, m_stick->GetZ() + m_stick->GetY());

  intake.Set(m_stick->GetRawButton(2));
  index.Set(m_stick->GetRawButton(3));
  shooter.Set(m_stick->GetRawButton(1)); */
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
