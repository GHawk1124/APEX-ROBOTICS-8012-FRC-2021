#include "subsystems/IntakeSubsystem.h"

#include "Constants.h"

IntakeSubsystem::IntakeSubsystem()
    : m_intake{DriveConstants::Ports::kIntakePort} {}

void IntakeSubsystem::Periodic() {}

void IntakeSubsystem::ActivateIntake() {
  m_intake.Set(OIConstants::kIntakeSpeed);
}

void IntakeSubsystem::outtake() {
  m_intake.Set(-1 * OIConstants::kIntakeSpeed);
}

void IntakeSubsystem::stopMotors() { m_intake.Set(0); }
