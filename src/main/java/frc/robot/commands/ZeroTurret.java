// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Concept credit: FRC team 2910, Jack in the Bot
package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotPreferences.prefTurret;
import frc.robot.subsystems.Turret;

public class ZeroTurret extends Command {
  Turret subTurret;

  double zeroingTimestamp;

  public ZeroTurret(Turret subTurret) {
    this.subTurret = subTurret;

    addRequirements(subTurret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subTurret.setTurretSoftwareLimits(false, true);

    subTurret.setTurretVoltage(0);
    zeroingTimestamp = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subTurret.setTurretVoltage(prefTurret.turretZeroingVoltage.getValue());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subTurret.setTurretSoftwareLimits(true, true);

    // Stop all movement
    subTurret.setTurretVoltage(0);

    // Reset to the current position if this command was not interrupted
    if (!interrupted) {
      subTurret.setTurretSensorAngle(prefTurret.turretSensorAngle.getValue());
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // If the current velocity is low enough to be considered as zeroed
    if (Math.abs(subTurret.getTurretVelocity()) <= Math.abs(prefTurret.turretZeroedVelocity.getValue())) {
      // And this is the first loop it has happened, begin the timer
      if (zeroingTimestamp == 0) {
        zeroingTimestamp = Timer.getFPGATimestamp();
        return false;
      }

      // If this isn't the first loop, return if it has been below the threshold for
      // long enough
      return (Timer.getFPGATimestamp() - zeroingTimestamp) >= prefTurret.turretZeroedTime.getValue();
    }

    // If the above wasn't true, we have gained too much velocity, so we aren't at 0
    // & need to restart the timer
    zeroingTimestamp = 0;
    return false;
  }
}
