// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.constLEDs;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Turret;

public class SetLEDS extends Command {
  LEDs subLEDs;
  Shooter subShooter;
  Turret subTurret;
  Pitch subPitch;
  Transfer subTransfer;
  Trigger amplify;
  Trigger defense;

  public SetLEDS(LEDs subLEDs, Shooter subShooter, Turret subTurret, Pitch subPitch, Transfer subTransfer,
      Trigger amplify, Trigger defense) {
    this.subLEDs = subLEDs;
    this.subShooter = subShooter;
    this.subTurret = subTurret;
    this.subPitch = subPitch;
    this.subTransfer = subTransfer;
    this.amplify = amplify;
    this.defense = defense;

    addRequirements(subLEDs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (amplify.getAsBoolean()) {
      subLEDs.setLEDsToAnimation(constLEDs.AMPLIFY_ANIMATION);
      return;
    }

    if (defense.getAsBoolean()) {
      subLEDs.setLEDsToAnimation(constLEDs.DEFENSE_MODE_ANIMATION);
      return;
    }

    // If we have a game piece, set to game piece colors
    if (subTransfer.hasGamePiece) {
      // Set LEDs when we are ready to shoot
      if (subShooter.areBothShootersUpToSpeed()
          && subPitch.isPitchAtGoalAngle()
          && subTurret.isTurretAtGoalAngle()) {

        subLEDs.setLEDs(constLEDs.GREEN_COLOR);
        return;
      }
      subLEDs.setLEDs(constLEDs.INTAKE_GAME_PIECE_COLLECTED);
      return;
    }
    subLEDs.clearAnimation();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
