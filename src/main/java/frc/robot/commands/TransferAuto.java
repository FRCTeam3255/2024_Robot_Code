// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotPreferences.prefShooter;
import frc.robot.RobotPreferences.prefTransfer;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Turret;

public class TransferAuto extends Command {
  /** Creates a new TransferGamePiece. */
  Transfer subTransfer;
  Turret subTurret;
  Shooter subShooter;
  Pitch subPitch;

  public TransferAuto(Shooter subShooter, Turret subTurret,
      Transfer subTransfer, Pitch subPitch) {
    this.subTransfer = subTransfer;
    this.subShooter = subShooter;
    this.subPitch = subPitch;
    this.subTurret = subTurret;

    addRequirements(subTransfer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subShooter.setDesiredVelocities(prefShooter.leftShooterSpeakerVelocity.getValue(),
        prefShooter.rightShooterSpeakerVelocity.getValue());
    subShooter.getUpToSpeed();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (subShooter.areBothShootersUpToSpeed()) {
      subTransfer.setFeederMotorSpeed(prefTransfer.feederShootSpeed.getValue());
      subTransfer.setTransferMotorSpeed(prefTransfer.transferShootSpeed.getValue());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return subShooter.areBothShootersUpToSpeed();
  }
}
