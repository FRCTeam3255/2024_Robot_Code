// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constLEDs;
import frc.robot.RobotPreferences.prefShooter;
import frc.robot.RobotPreferences.prefTransfer;
import frc.robot.subsystems.LEDs;
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
  LEDs subLEDs;

  public TransferGamePiece(Shooter subShooter, LEDs subLEDs, Turret subTurret,
      Transfer subTransfer, Pitch subPitch) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subTransfer = subTransfer;
    this.subShooter = subShooter;
    this.subLEDs = subLEDs;
    this.subPitch = subPitch;
    this.subTurret = subTurret;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subTransfer.setGamePieceCollected(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (subShooter.areBothShootersUpToSpeed()
        && subPitch.isPitchAtGoalAngle()
        && subTurret.isTurretAtGoalAngle()) {
      subLEDs.setLEDs(constLEDs.SHOOTER_UP_TO_SPEED_COLOR);
      subTransfer.setFeederMotorSpeed(
          prefTransfer.feederShootMotorSpeed.getValue());
      subTransfer.setTransferMotorSpeed(prefTransfer.transferMotorSpeed.getValue());

    } else {
      subLEDs.setLEDsToAnimation(constLEDs.SHOOTER_ANIMATION);
      subTransfer.setFeederNeutralOutput();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subTransfer.setFeederNeutralOutput();
    subTransfer.setTransferNeutralOutput();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
