// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotPreferences.prefShooter;
import frc.robot.subsystems.Shooter;

public class ZeroShooterPitch extends Command {
  Shooter subShooter;

  boolean isZeroed;

  public ZeroShooterPitch(Shooter subShooter) {
    this.subShooter = subShooter;

    addRequirements(subShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subShooter.setPitchVoltage(0);
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
      subShooter.setPitchAngle(subShooter.getPitchAngle());
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // The hood is zeroed when a. Enough velocity is being supplied to it b. It has
    // been long enough
    if (subShooter.getPitchVelocity() < prefShooter.pitchZeroedVelocity.getValue()) {
      return true; // TODO: ADD TIME
    }
    return false;
  }
}
