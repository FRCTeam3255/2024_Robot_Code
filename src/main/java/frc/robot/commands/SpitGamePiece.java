// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotPreferences.prefHood;
import frc.robot.RobotPreferences.prefIntake;
import frc.robot.RobotPreferences.prefTransfer;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Transfer;

public class SpitGamePiece extends Command {
  /** Creates a new SpitGamePiece. */
  Intake globalIntake;
  Transfer globalTransfer;
  Hood subHood;
  Climber subClimber;

  double lastDesiredHood;

  public SpitGamePiece(Intake subIntake, Transfer subTransfer, Hood subHood, Climber subClimber) {
    globalIntake = subIntake;
    globalTransfer = subTransfer;
    this.subHood = subHood;
    this.subClimber = subClimber;

    addRequirements(globalIntake, globalTransfer, subHood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastDesiredHood = subHood.getHoodAngle();
    globalTransfer.setGamePieceCollected(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    globalIntake.setIntakeMotorsSpeed(prefIntake.intakeSpitOutSpeed.getValue());

    globalTransfer.setTransferMotorSpeed(prefTransfer.transferSpitOutSpeed.getValue());
    globalTransfer.setFeederMotorSpeed(prefTransfer.feederSpitOutSpeed.getValue());

    subHood.setHoodAngle(prefHood.hoodReverseLimit.getValue(), subClimber.collidesWithHood());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    globalIntake.setNeutralMode();
    globalTransfer.setTransferNeutralOutput();
    globalTransfer.setFeederNeutralOutput();
    subHood.setHoodAngle(lastDesiredHood, subClimber.collidesWithHood());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
