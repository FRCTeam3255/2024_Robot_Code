// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.LockedLocation;
import frc.robot.RobotPreferences.prefIntake;
import frc.robot.RobotPreferences.prefTransfer;
import frc.robot.subsystems.Climber;
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
  Climber subClimber;

  public TransferGamePiece(Shooter subShooter, Turret subTurret,
      Transfer subTransfer, Pitch subPitch, Intake subIntake, Climber subClimber) {
    this.subTransfer = subTransfer;
    this.subShooter = subShooter;
    this.subPitch = subPitch;
    this.subTurret = subTurret;
    this.subIntake = subIntake;
    this.subClimber = subClimber;

    addRequirements(subTransfer, subIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subTransfer.setTransferSensorAngle(0);
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
      // TRAP
      if (subClimber.getPosition() > 0.3) {
        subIntake.setPivotAngle(prefIntake.pivotPlaceTrapAngle.getValue());
        if (subIntake.isPivotAtAngle(prefIntake.pivotPlaceTrapAngle.getValue())) {
          subTransfer.setGamePieceCollected(false);
          subIntake.setIntakeRollerSpeed(prefIntake.rollerPlaceTrapSpeed.getValue());
        }
      } else {
        subIntake.setPivotAngle(prefIntake.pivotPlaceAmpAngle.getValue());

        if (subIntake.isPivotAtAngle(prefIntake.pivotPlaceAmpAngle.getValue())) {
          subTransfer.setGamePieceCollected(false);
          subIntake.setIntakeRollerSpeed(prefIntake.rollerPlaceAmpSpeed.getValue());
        }
      }
    }
    subTransfer.setTransferSensorAngle(0);
    subTransfer.setFeederMotorSpeed(0);
    subTransfer.setTransferMotorSpeed(0);
    return;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!RobotState.isAutonomous()) {
      subTransfer.setFeederMotorSpeed(0);
      subTransfer.setTransferMotorSpeed(0);
      subIntake.setIntakeRollerSpeed(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
