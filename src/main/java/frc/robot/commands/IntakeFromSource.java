// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotPreferences.prefIntake;
import frc.robot.RobotPreferences.prefPitch;
import frc.robot.RobotPreferences.prefShooter;
import frc.robot.RobotPreferences.prefTransfer;
import frc.robot.RobotPreferences.prefTurret;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
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
  Intake subIntake;
  double intakeCurrent;
  double lastDesiredSpeedLeft;
  double lastDesiredSpeedRight;
  double lastDesiredPitch;
  double lastDesiredAngle;

  /** Creates a new ShooterIntake. */
  public IntakeFromSource(Shooter subShooter, Transfer subTransfer, Pitch subPitch, Turret subTurret,
      Climber subClimber, Intake subIntake) {
    this.subShooter = subShooter;
    this.subTransfer = subTransfer;
    this.subPitch = subPitch;
    this.subTurret = subTurret;
    this.subClimber = subClimber;
    this.subIntake = subIntake;
    addRequirements(subShooter, subPitch, subTurret, subTransfer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastDesiredSpeedLeft = subShooter.getLeftShooterVelocity();
    lastDesiredSpeedRight = subShooter.getRightShooterVelocity();

    lastDesiredPitch = subPitch.getPitchAngle();
    subIntake.setIntakeMotorsSpeed(prefIntake.intakeRollerSpeed.getValue());
    subShooter.setDesiredVelocities(prefShooter.leftShooterIntakeVelocity.getValue(),
        prefShooter.rightShooterIntakeVelocity.getValue());
    subShooter.getUpToSpeed();

    subTransfer.setFeederMotorSpeed(prefTransfer.feederIntakeSourceSpeed.getValue());
    subTransfer.setTransferMotorSpeed(prefTransfer.transferIntakeSourceSpeed.getValue());

    subPitch.setPitchAngle(prefPitch.pitchSourceAngle.getValue(), subClimber.collidesWithPitch());

    subTurret.setTurretAngle(prefTurret.turretIntakePos.getValue(), subClimber.collidesWithTurret());

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (subIntake.intakeSourceGamePieceDetection()) {
      subTransfer.setTransferNeutralOutput();
      subIntake.setIntakeNeutralOutput();
      subTransfer.setFeederNeutralOutput();
      subShooter.setShootingNeutralOutput();
    }

    else {
      subTransfer.setFeederMotorSpeed(prefTransfer.feederIntakeSourceSpeed.getValue());
      subTransfer.setTransferMotorSpeed(prefTransfer.transferIntakeSourceSpeed.getValue());
      subIntake.setIntakeMotorsSpeed(prefIntake.intakeRollerSpeed.getValue());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subTransfer.setFeederNeutralOutput();
    subTransfer.setTransferNeutralOutput();
    subShooter.setDesiredVelocities(lastDesiredSpeedLeft, lastDesiredSpeedRight);
    subPitch.setPitchAngle(lastDesiredPitch, subClimber.collidesWithPitch());
    subIntake.setIntakeNeutralOutput();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return subIntake.intakeSourceGamePieceDetection();
  }
}
