#include "subsystems/ShooterSubsystem.h"

#include "Constants.h"

ShooterSubsystem::ShooterSubsystem()
    : m_index{DriveConstants::Ports::kIndexPort},
      m_shooter{DriveConstants::Ports::kShooterPort} {}

void ShooterSubsystem::Periodic() {}

void ShooterSubsystem::spinUp() { m_shooter.Set(OIConstants::kShooterSpeed); }

void ShooterSubsystem::shoot() { m_index.Set(OIConstants::kIndexSpeed); }

void ShooterSubsystem::stopShooter() { m_shooter.Set(0); }

void ShooterSubsystem::stopIndex() { m_index.Set(0); }
