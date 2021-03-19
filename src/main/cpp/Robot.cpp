#include <adi/ADIS16448_IMU.h>
#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc2/command/PIDCommand.h>

#include "ctre/Phoenix.h"

#include "constants.h"

class Robot : public frc::TimedRobot {
  WPI_VictorSPX intake{5};
  WPI_VictorSPX index{6};
  WPI_VictorSPX shooter{7};
  TalonFX LB{3};
  TalonFX LF{2};
  TalonFX RB{1};
  TalonFX RF{4};

  frc::ADIS16448_IMU imu{};
  frc::Joystick m_stick{0};

  double drivekP = 0.0;
  double drivekI = 0.0;
  double drivekD = 0.0;

  frc2::PIDController drivePID{drivekP, drivekI, drivekD};

public:
  void RobotInit() override { imu.Calibrate(); }

  void AutonomousInit() override {
    LB.Config_kP(0, drivekP);
    LB.Config_kI(0, drivekI);
    LB.Config_kD(0, drivekD);

    LF.Config_kP(0, drivekP);
    LF.Config_kI(0, drivekI);
    LF.Config_kD(0, drivekD);

    RB.Config_kP(0, drivekP);
    RB.Config_kI(0, drivekI);
    RB.Config_kD(0, drivekD);

    RF.Config_kP(0, drivekP);
    RF.Config_kI(0, drivekI);
    RF.Config_kD(0, drivekD);

    LB.SetNeutralMode(NeutralMode::Coast);
    LF.SetNeutralMode(NeutralMode::Coast);
    RB.SetNeutralMode(NeutralMode::Coast);
    RF.SetNeutralMode(NeutralMode::Coast);

    LB.SetSensorPhase(true);
    LF.SetSensorPhase(true);
    RB.SetSensorPhase(true);
    RB.SetSensorPhase(true);

    //int test[10] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
  }

  void AutonomousPeriodic() override {
    LB.Set(ControlMode::Position, 100);
    LF.Set(ControlMode::Position, 100);
    RB.Set(ControlMode::Position, 100);
    RF.Set(ControlMode::Position, 100);
  }

  void TeleopPeriodic() override {
    wpi::outs() << LB.GetSensorCollection().GetIntegratedSensorPosition() << '\n';
    wpi::outs() << LF.GetSensorCollection().GetIntegratedSensorPosition() << '\n';
    wpi::outs() << RB.GetSensorCollection().GetIntegratedSensorPosition() << '\n';
    wpi::outs() << RF.GetSensorCollection().GetIntegratedSensorPosition() << '\n';
    double z = (m_stick.GetZ() > DEADZONE || m_stick.GetZ() < -DEADZONE)
                   ? (m_stick.GetZ() > 0 ? m_stick.GetZ() * m_stick.GetZ()
                                         : -1 * m_stick.GetZ() * m_stick.GetZ())
                   : 0;
    double y = (m_stick.GetY() > DEADZONE || m_stick.GetY() < -DEADZONE)
                   ? (m_stick.GetY() > 0 ? m_stick.GetY() * m_stick.GetY()
                                         : -1 * m_stick.GetY() * m_stick.GetY())
                   : 0;
    LF.Set(ControlMode::PercentOutput, z - y);
    LB.Set(ControlMode::PercentOutput, z - y);
    RF.Set(ControlMode::PercentOutput, z + y);
    RB.Set(ControlMode::PercentOutput, z + y);

    intake.Set(m_stick.GetRawButton(2) + m_stick.GetRawButton(3));
    index.Set(m_stick.GetRawButton(3));
    shooter.Set(m_stick.GetRawButton(1));
  }
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif