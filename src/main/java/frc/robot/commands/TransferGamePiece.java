// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotPreferences.prefTransfer;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Turret;

public class TransferGamePiece extends Command {
  /** Creates a new TransferGamePiece. */
  Transfer subTransfer;
  Turret subTurret;
  Shooter subShooter;
  Hood subHood;

  public TransferGamePiece(Shooter subShooter, Turret subTurret,
      Transfer subTransfer, Hood subHood) {
    this.subTransfer = subTransfer;
    this.subShooter = subShooter;
    this.subHood = subHood;
    this.subTurret = subTurret;

    addRequirements(subTransfer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (subShooter.areBothShootersUpToSpeed()
        && subHood.isHoodAtGoalAngle()
        && subTurret.isTurretAtGoalAngle()) {

      subTransfer.setGamePieceCollected(false);
      subTransfer.setFeederMotorSpeed(prefTransfer.feederShootSpeed.getValue());
      subTransfer.setTransferMotorSpeed(prefTransfer.transferShootSpeed.getValue());
      return;
    } else {
      subTransfer.setFeederMotorSpeed(0);
      subTransfer.setTransferMotorSpeed(0);
      return;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subTransfer.setFeederMotorSpeed(0);
    subTransfer.setTransferMotorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
