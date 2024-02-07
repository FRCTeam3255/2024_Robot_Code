// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotPreferences.prefShooter;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
  Shooter subShooter;

  public Shoot(Shooter subShooter) {
    this.subShooter = subShooter;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subShooter.setShootingVelocities(
        prefShooter.leftShooterVelocity.getValue(),
        prefShooter.leftShooterFeedForward.getValue(),
        prefShooter.rightShooterVelocity.getValue(),
        prefShooter.rightShooterFeedForward.getValue());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subShooter.setShootingNeutralOutput();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
