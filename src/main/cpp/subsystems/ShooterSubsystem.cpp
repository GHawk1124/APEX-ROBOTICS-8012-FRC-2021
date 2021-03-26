#include "subsystems/ShooterSubsystem.h"
#include "commands/DefaultDrive.h"

#include "Constants.h"

ShooterSubsystem::ShooterSubsystem()
    : m_index{DriveConstants::Ports::kIndexPort},
      m_shooter{DriveConstants::Ports::kShooterPort} {}

void ShooterSubsystem::Periodic() {}

void ShooterSubsystem::spinUp(double throttle) { m_shooter.Set(OIConstants::kShooterSpeed * throttle); }

void ShooterSubsystem::shoot() { m_index.Set(OIConstants::kIndexSpeed); }

void ShooterSubsystem::stopShooter() { m_shooter.Set(0); }

void ShooterSubsystem::stopIndex() { m_index.Set(0); }
