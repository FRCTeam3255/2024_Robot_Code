// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotPreferences.prefIntake;
import frc.robot.RobotPreferences.prefPitch;
import frc.robot.RobotPreferences.prefTransfer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Transfer;

public class SpitGamePiece extends Command {
  /** Creates a new SpitGamePiece. */
  Intake globalIntake;
  Transfer globalTransfer;
  Pitch subPitch;

  Measure<Angle> lastDesiredPitch;

  public SpitGamePiece(Intake subIntake, Transfer subTransfer, Pitch subPitch) {
    globalIntake = subIntake;
    globalTransfer = subTransfer;
    this.subPitch = subPitch;
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
    globalIntake.setIntakeRollerSpeed(prefIntake.rollerSpitSpeed.getValue(Units.Value));

    globalTransfer.setTransferMotorSpeed(prefTransfer.transferSpitOutSpeed.getValue(Units.Value));
    globalTransfer.setFeederMotorSpeed(prefTransfer.feederSpitOutSpeed.getValue(Units.Value));

    subPitch.setPitchAngle(prefPitch.pitchReverseLimit.getMeasure());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    globalIntake.setRollerNeutralOutput();
    globalTransfer.setTransferNeutralOutput();
    globalTransfer.setFeederNeutralOutput();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
