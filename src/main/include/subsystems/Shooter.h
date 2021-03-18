#pragma once

#include <frc/Joystick.h>
#include <frc2/command/SubsystemBase.h>

#include "ctre/Phoenix.h"

class Shooter : public frc2::SubsystemBase {
public:
  Shooter();

  void Periodic() override;

private:
  WPI_VictorSPX index{6};
  WPI_VictorSPX shooter{7};
};
