// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constLEDs;
import frc.robot.RobotPreferences.prefIntake;
import frc.robot.RobotPreferences.prefTransfer;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Transfer;

public class SpitGamePiece extends Command {
  /** Creates a new SpitGamePiece. */
  Intake globalIntake;
  Transfer globalTransfer;
  LEDs globalLEDs;
  Pitch subPitch;
  Climber subClimber;

  double lastDesiredPitch;

  public SpitGamePiece(Intake subIntake, Transfer subTransfer, LEDs subLEDs, Pitch subPitch, Climber subClimber) {
    // Use addRequirements() here to declare subsystem dependencies.
    globalIntake = subIntake;
    globalTransfer = subTransfer;
    globalLEDs = subLEDs;
    this.subPitch = subPitch;
    this.subClimber = subClimber;

    addRequirements(globalIntake, globalTransfer, subPitch);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastDesiredPitch = subPitch.getPitchAngle();
    globalTransfer.setGamePieceCollected(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    globalIntake.setIntakeMotorsSpeed(prefIntake.intakeSpitOutSpeed.getValue());

    globalTransfer.setTransferMotorSpeed(prefTransfer.transferSpitOutSpeed.getValue());
    globalTransfer.setFeederMotorSpeed(prefTransfer.feederSpitOutSpeed.getValue());

    subPitch.setPitchAngle(0, subClimber.collidesWithPitch());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    globalIntake.setNeutralMode();
    globalTransfer.setTransferNeutralOutput();
    globalTransfer.setFeederNeutralOutput();
    subPitch.setPitchAngle(lastDesiredPitch, subClimber.collidesWithPitch());

    if (!interrupted) {

      globalLEDs.setLEDs(constLEDs.SPIT_OUT_GAME_PIECE);
    }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
}
