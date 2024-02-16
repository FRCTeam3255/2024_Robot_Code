// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.LockedLocation;
import frc.robot.Constants.constLEDs;
import frc.robot.RobotPreferences.prefIntake;
import frc.robot.RobotPreferences.prefTransfer;
import frc.robot.RobotPreferences.prefTurret;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Turret;

public class IntakeGamePiece extends Command {
  Intake subIntake;
  Transfer subTransfer;
  Turret subTurret;
  LEDs subLEDs;

  public IntakeGamePiece(Intake subIntake, Transfer subTransfer, Turret subTurret, LEDs subLEDs) {
    this.subIntake = subIntake;
    this.subTransfer = subTransfer;
    this.subTurret = subTurret;
    this.subLEDs = subLEDs;
    addRequirements(subIntake, subTransfer, subTurret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    subTurret.setTurretAngle(prefTurret.turretIntakePos.getValue());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // subIntake.setPivotMotorAngle(prefIntake.intakeIntakingPosition.getValue());

    subIntake.setIntakeMotorsSpeed(prefIntake.intakeRollerSpeed.getValue(),
        prefIntake.intakeCenteringSpeed.getValue());
    subTransfer.setTransferMotorSpeed(prefTransfer.transferMotorSpeed.getValue());

    subTransfer.setFeederMotorSpeed(prefTransfer.feederMotorSpeed.getValue());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // subIntake.setPivotMotorAngle(prefIntake.intakeStowPosition.getValue());
    subIntake.setNeutralMode();
    subTransfer.setTransferNeutralOutput();
    subTransfer.setFeederNeutralOutput();
    subLEDs.setLEDs(constLEDs.INTAKE_GAME_PIECE_COLLECTED);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return subTransfer.isGamePieceCollected();

  }
}
