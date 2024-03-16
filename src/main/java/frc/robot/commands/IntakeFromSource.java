// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.RobotPreferences.prefPitch;
import frc.robot.RobotPreferences.prefShooter;
import frc.robot.RobotPreferences.prefTransfer;
import frc.robot.RobotPreferences.prefTurret;
import frc.robot.subsystems.Climber;

import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Turret;

public class IntakeFromSource extends Command {
  Shooter subShooter;
  Transfer subTransfer;
  Pitch subPitch;
  Turret subTurret;
  Climber subClimber;

  double lastDesiredPitch = prefPitch.pitchReverseLimit.getValue();
  double lastDesiredAngle;
  double lastDesiredTurret;

  /** Creates a new ShooterIntake. */
  public IntakeFromSource(Shooter subShooter, Transfer subTransfer, Pitch subPitch, Turret subTurret,
      Climber subClimber) {
    this.subShooter = subShooter;
    this.subTransfer = subTransfer;
    this.subPitch = subPitch;
    this.subTurret = subTurret;
    this.subClimber = subClimber;

    addRequirements(subShooter, subPitch, subTurret, subTransfer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastDesiredPitch = subPitch.getPitchAngle();

    subShooter.setVoltage(prefShooter.leftShooterIntakeVoltage.getValue(),
        prefShooter.rightShooterIntakeVoltage.getValue());

    subPitch.setPitchAngle(prefPitch.pitchSourceAngle.getValue(), false);

    subTurret.setTurretAngle(prefTurret.turretIntakePos.getValue(), subClimber.collidesWithTurret());
    subShooter.setDesiredVelocities(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subTransfer.setFeederMotorSpeed(prefTransfer.feederIntakeSourceSpeed.getValue());
    subTransfer.setTransferMotorSpeed(prefTransfer.transferIntakeSourceSpeed.getValue());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!RobotState.isAutonomous()) {
      subTransfer.setNeutralMode();
    }
    if (!interrupted) {
      subTransfer.repositionGamePiece();

      subShooter.setDesiredVelocities(prefShooter.leftShooterSubVelocity.getValue(),
          prefShooter.rightShooterSubVelocity.getValue());
    } else {
      subTransfer.setTransferNeutralOutput();
      subShooter.setDesiredVelocities(0, 0);
      subShooter.setVoltage(0, 0);
    }

    subShooter.getUpToSpeed();
    subTransfer.setFeederNeutralOutput();
    subPitch.setPitchAngle(lastDesiredPitch, false);
    subTurret.setTurretAngle(lastDesiredTurret, subClimber.collidesWithTurret());
    subClimber.setNeutralOutput();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return subTransfer.intakeSourceGamePieceDetection();
  }
}
