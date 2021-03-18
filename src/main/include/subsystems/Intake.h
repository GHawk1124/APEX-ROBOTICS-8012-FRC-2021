#pragma once

#include <frc/Joystick.h>
#include <frc2/command/SubsystemBase.h>

#include "ctre/Phoenix.h"

class Intake : public frc2::SubsystemBase {
public:
  Intake();

  void Periodic() override;

  void pickUp(double val);

  void outtake(double val);

private:
  WPI_VictorSPX intake{5};
};
