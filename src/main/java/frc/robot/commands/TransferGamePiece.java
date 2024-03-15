// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.LockedLocation;
import frc.robot.RobotPreferences.prefIntake;
import frc.robot.RobotPreferences.prefTransfer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Turret;

public class TransferGamePiece extends Command {
  /** Creates a new TransferGamePiece. */
  Transfer subTransfer;
  Turret subTurret;
  Shooter subShooter;
  Pitch subPitch;
  Intake subIntake;

  public TransferGamePiece(Shooter subShooter, Turret subTurret,
      Transfer subTransfer, Pitch subPitch, Intake subIntake) {
    this.subTransfer = subTransfer;
    this.subShooter = subShooter;
    this.subPitch = subPitch;
    this.subTurret = subTurret;
    this.subIntake = subIntake;

    addRequirements(subTransfer, subIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.getLockedLocation() != LockedLocation.AMP) {
      if (subShooter.areBothShootersUpToSpeed()
          && subPitch.isPitchAtGoalAngle()
          && subTurret.isTurretAtGoalAngle()) {

        subTransfer.setGamePieceCollected(false);
        subTransfer.setFeederMotorSpeed(prefTransfer.feederShootSpeed.getValue());
        subTransfer.setTransferMotorSpeed(prefTransfer.transferShootSpeed.getValue());
        return;
      }
    } else {
      // TODO: Do we want to check if we are actually at the desired intake pivot
      // position before amping?
      subIntake.setIntakeRollerSpeed(prefIntake.rollerPlaceAmpSpeed.getValue());
    }
    subTransfer.setFeederMotorSpeed(0);
    subTransfer.setTransferMotorSpeed(0);
    return;

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
