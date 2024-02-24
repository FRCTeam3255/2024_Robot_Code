// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotPreferences.prefPitch;
import frc.robot.RobotPreferences.prefShooter;
import frc.robot.RobotPreferences.prefTransfer;
import frc.robot.RobotPreferences.prefTurret;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Turret;

public class IntakeFromSource extends Command {
  Shooter subShooter;
  Transfer subTransfer;
  Pitch subPitch;
  Turret subTurret;

  double lastDesiredSpeedLeft;
  double lastDesiredSpeedRight;
  double lastDesiredPitch;
  double lastDesiredAngle;

  /** Creates a new ShooterIntake. */
  public IntakeFromSource(Shooter subShooter, Transfer subTransfer, Pitch subPitch, Turret subTurret) {
    this.subShooter = subShooter;
    this.subTransfer = subTransfer;
    this.subPitch = subPitch;
    this.subTurret = subTurret;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastDesiredSpeedLeft = subShooter.getLeftShooterVelocity();
    lastDesiredSpeedRight = subShooter.getRightShooterVelocity();

    lastDesiredPitch = subPitch.getPitchAngle();

    subShooter.setDesiredVelocities(prefShooter.leftShooterIntakeVelocity.getValue(),
        prefShooter.rightShooterIntakeVelocity.getValue());
    subShooter.getUpToSpeed();

    subTransfer.setFeederMotorSpeed(prefTransfer.feederIntakeMotorSpeed.getValue());
    subTransfer.setTransferMotorSpeed(prefTransfer.transferIntakeMotorSpeed.getValue());

    subPitch.setPitchAngle(prefPitch.pitchSourceAngle.getValue());

    subTurret.setTurretAngle(prefTurret.turretIntakePos.getValue());

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subTransfer.setFeederNeutralOutput();
    subTransfer.setTransferNeutralOutput();
    subShooter.setDesiredVelocities(lastDesiredSpeedLeft, lastDesiredSpeedRight);
    subPitch.setPitchAngle(lastDesiredPitch);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}