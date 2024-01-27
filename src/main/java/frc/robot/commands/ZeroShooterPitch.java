// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Concept credit: FRC team 2910, Jack in the Bot
package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotPreferences.prefShooter;
import frc.robot.subsystems.Shooter;

public class ZeroShooterPitch extends Command {
  Shooter subShooter;

  double zeroingTimestamp;

  public ZeroShooterPitch(Shooter subShooter) {
    this.subShooter = subShooter;

    addRequirements(subShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subShooter.setPitchVoltage(0);
    zeroingTimestamp = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subShooter.setPitchVoltage(prefShooter.pitchZeroingVoltage.getValue());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop all movement
    subShooter.setPitchVoltage(0);

    // Reset to the current position if this command was not interrupted
    if (!interrupted) {
      subShooter.setPitchSensorAngle(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // If the current velocity is low enough to be considered as zeroed
    if (Math.abs(subShooter.getPitchVelocity()) <= Math.abs(prefShooter.pitchZeroedVelocity.getValue())) {
      // And this is the first loop it has happened, begin the timer
      if (zeroingTimestamp == 0) {
        zeroingTimestamp = Timer.getFPGATimestamp();
        return false;
      }

      // If this isn't the first loop, return if it has been below the threshold for
      // long enough
      return (Timer.getFPGATimestamp() - zeroingTimestamp) >= prefShooter.pitchZeroedTime.getValue();
    }

    // If the above wasn't true, we have gained too much velocity so we need to
    // restart the timer
    zeroingTimestamp = 0;
    return false;
  }
}
