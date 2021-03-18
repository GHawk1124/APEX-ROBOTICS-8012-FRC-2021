#include "commands/ArcadeDrive.h"

ArcadeDrive::ArcadeDrive(DriveTrain* subsystem,
                           std::function<double()> forward,
                           std::function<double()> rotation)
    : m_drive{subsystem}, m_forward{forward}, m_rotation{rotation} {
  AddRequirements({subsystem});
}

void ArcadeDrive::Execute() {
  m_drive->Drive(m_forward(), m_rotation());
}
