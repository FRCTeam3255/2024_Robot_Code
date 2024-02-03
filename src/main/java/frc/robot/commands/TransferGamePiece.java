// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotPreferences.prefTransfer;
import frc.robot.subsystems.Transfer;

public class TransferGamePiece extends Command {
  /** Creates a new TransferGamePiece. */
  Transfer globalFeeder;

  public TransferGamePiece(Transfer givenTransfer) {
    // Use addRequirements() here to declare subsystem dependencies.
    globalFeeder = givenTransfer;

    // globalTransfer = givenTransfer;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    globalFeeder.setFeederMotorSpeed(prefTransfer.feederMotorSpeed.getValue());
    globalFeeder.setTransferMotorSpeed(prefTransfer.transferMotorSpeed.getValue());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    globalFeeder.setFeederNeutralOutput();
    globalFeeder.setTransferNeutralOutput();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
