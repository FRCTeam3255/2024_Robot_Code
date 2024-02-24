// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotPreferences.prefShooter;
import frc.robot.RobotPreferences.prefTransfer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;

public class IntakeFromShooter extends Command {
  Shooter globalShooter;
  Transfer globalTransfer;

  /** Creates a new ShooterIntake. */
  public IntakeFromShooter(Shooter subShooter, Transfer subTransfer) {
    globalShooter = subShooter;
    globalTransfer = subTransfer;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    globalShooter.setShootingVelocities(
        prefShooter.leftShooterIntakeVelocity.getValue(),
        prefShooter.leftShooterIntakeFeedForward.getValue(),
        prefShooter.rightShooterIntakeVelocity.getValue(),
        prefShooter.rightShooterIntakeFeedForward.getValue());
    globalTransfer.setFeederMotorSpeed(prefTransfer.feederIntakeMotorSpeed.getValue());
    globalTransfer.setTransferMotorSpeed(prefTransfer.transferIntakeMotorSpeed.getValue());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    globalTransfer.setFeederNeutralOutput();
    globalTransfer.setTransferNeutralOutput();
    globalShooter.setShootingNeutralOutput();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
