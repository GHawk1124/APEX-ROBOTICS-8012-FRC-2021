//#pragma once

#include <frc/Joystick.h>
#include <frc2/command/SubsystemBase.h>

#include "ctre/Phoenix.h"

class DriveTrain : public frc2::SubsystemBase {
public:
  DriveTrain();

  void Periodic() override;

  void Drive(double forwardAxis, double turnAxis);

private:
  TalonFX LB{3};
  TalonFX LF{0};
  TalonFX RB{1};
  TalonFX RF{2};
};
