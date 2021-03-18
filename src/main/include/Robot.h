#pragma once

#include <string>

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <adi/ADIS16448_IMU.h>

#include "ctre/Phoenix.h"

#include "RobotContainer.h"

class Robot : public frc::TimedRobot {
public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;

private:
  WPI_VictorSPX intake{5};
  WPI_VictorSPX index{6};
  WPI_VictorSPX shooter{7};
  TalonFX LB{3};
  TalonFX LF{2};
  TalonFX RB{1};
  TalonFX RF{4};

  frc::ADIS16448_IMU imu{};

  frc::Joystick *m_stick = new frc::Joystick{0};
  frc::Timer timer{};

  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  frc2::Command* m_autonomousCommand = nullptr;

  RobotContainer m_container;
};
