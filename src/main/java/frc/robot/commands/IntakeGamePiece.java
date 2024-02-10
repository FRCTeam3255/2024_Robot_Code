// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotPreferences.prefIntake;
import frc.robot.RobotPreferences.prefTransfer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transfer;

public class IntakeGamePiece extends Command {
  Intake subIntake;
  Transfer subTransfer;

  public IntakeGamePiece(Intake subIntake, Transfer subTransfer) {
    this.subIntake = subIntake;
    this.subTransfer = subTransfer;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (subTransfer.isGamePieceCollected() == true) {
      subTransfer.setTransferNeutralOutput();
      subIntake.setNeutralMode();
    } else {
      subIntake.setIntakeMotorsSpeed(prefIntake.intakeRollerSpeed.getValue(),
          prefIntake.intakeCenteringSpeed.getValue());
      subTransfer.setTransferMotorSpeed(prefTransfer.transferMotorSpeed.getValue());
    }
    subTransfer.setFeederMotorSpeed(0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subIntake.setPivotMotorAngle(prefIntake.intakeStowPosition.getValue());
    subIntake.setNeutralMode();
    subTransfer.setTransferNeutralOutput();
    subTransfer.setFeederNeutralOutput();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
