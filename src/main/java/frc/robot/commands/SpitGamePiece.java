// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
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

  double lastDesiredPitch;

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
    globalIntake.setIntakeRollerSpeed(prefIntake.intakeSpitOutSpeed.getValue());

    globalTransfer.setTransferMotorSpeed(prefTransfer.transferSpitOutSpeed.getValue());
    globalTransfer.setFeederMotorSpeed(prefTransfer.feederSpitOutSpeed.getValue());

    subPitch.setPitchAngle(Units.rotationsToDegrees(prefPitch.pitchReverseLimit.getValue()));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    globalIntake.setNeutralOutput();
    globalTransfer.setTransferNeutralOutput();
    globalTransfer.setFeederNeutralOutput();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
