// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotPreferences.prefIntake;
import frc.robot.RobotPreferences.prefTransfer;
import frc.robot.RobotPreferences.prefTurret;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Turret;

public class IntakeGroundGamePiece extends Command {
  Intake subIntake;
  Transfer subTransfer;
  Turret subTurret;
  Pitch subPitch;

  double lastDesiredPitch;
  double lastDesiredTurret;

  public IntakeGroundGamePiece(Intake subIntake, Transfer subTransfer, Turret subTurret,
      Pitch subPitch) {
    this.subIntake = subIntake;
    this.subTransfer = subTransfer;
    this.subTurret = subTurret;
    this.subPitch = subPitch;
    addRequirements(subIntake, subTransfer, subTurret, subPitch);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastDesiredTurret = subTurret.getAngle();
    lastDesiredPitch = subPitch.getPitchAngle();
    // moved climber pivot to init since its pid
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    subIntake.setIntakeMotorsSpeed(prefIntake.intakeRollerSpeed.getValue());

    subTransfer.setTransferMotorSpeed(prefTransfer.transferIntakeGroundSpeed.getValue());
    subTransfer.setFeederMotorSpeed(prefTransfer.feederIntakeGroundSpeed.getValue());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!RobotState.isAutonomous()) {
      subIntake.setNeutralMode();
    }
    subTransfer.setTransferNeutralOutput();
    subTransfer.setFeederNeutralOutput();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return subTransfer.calcGamePieceCollected();
  }
}
