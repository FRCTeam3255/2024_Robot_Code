// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotPreferences.prefShooter;
import frc.robot.RobotPreferences.prefTransfer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RepositionGamePiece extends SequentialCommandGroup {
  Transfer subTransfer;
  Shooter subShooter;

  /** Creates a new RepositionGamePiece. */
  public RepositionGamePiece(Transfer subTransfer, Shooter subShooter) {
    this.subTransfer = subTransfer;
    this.subShooter = subShooter;

    addRequirements(subTransfer, subShooter);

    addCommands(
        Commands.runOnce(() -> subTransfer.setTransferMotorSpeed(prefTransfer.transferRepositionSpeed.getValue())),
        Commands.waitSeconds(prefTransfer.transferRepositionTime.getValue()),
        Commands.runOnce(() -> subTransfer.setTransferMotorSpeed(-prefTransfer.transferRepositionSpeed.getValue())),
        Commands.waitSeconds(prefTransfer.transferRepositionTime.getValue() / 2),
        Commands.runOnce(() -> subTransfer.setTransferNeutralOutput()),
        Commands.runOnce(() -> subShooter.setDesiredVelocities(prefShooter.leftShooterSubVelocity.getValue(),
            prefShooter.rightShooterSubVelocity.getValue())),
        Commands.runOnce(() -> subShooter.getUpToSpeed()));

  }
}
