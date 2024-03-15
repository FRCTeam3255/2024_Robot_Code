// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotPreferences.prefIntake;
import frc.robot.RobotPreferences.prefPitch;
import frc.robot.RobotPreferences.prefShooter;
import frc.robot.RobotPreferences.prefTransfer;
import frc.robot.RobotPreferences.prefTurret;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Turret;

public class IntakeGroundGamePiece extends Command {
  Intake subIntake;
  Transfer subTransfer;
  Turret subTurret;
  Pitch subPitch;
  Shooter subShooter;

  double lastDesiredPitch;
  double lastDesiredTurret;

  public IntakeGroundGamePiece(Intake subIntake, Transfer subTransfer, Turret subTurret,
      Pitch subPitch, Shooter subShooter) {
    this.subIntake = subIntake;
    this.subTransfer = subTransfer;
    this.subTurret = subTurret;
    this.subPitch = subPitch;
    this.subShooter = subShooter;

    addRequirements(subIntake, subTransfer, subTurret, subPitch, subShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastDesiredTurret = subTurret.getAngle();
    lastDesiredPitch = subPitch.getPitchAngle();
    subTurret.setTurretAngle(prefTurret.turretIntakePos.getValue());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    subIntake.setIntakeRollerSpeed(prefIntake.intakeRollerSpeed.getValue());

    subTransfer.setTransferMotorSpeed(prefTransfer.transferIntakeGroundSpeed.getValue());
    subTransfer.setFeederMotorSpeed(prefTransfer.feederIntakeGroundSpeed.getValue());

    subPitch.setPitchAngle(Units.rotationsToDegrees(prefPitch.pitchReverseLimit.getValue()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!RobotState.isAutonomous()) {
      subIntake.setNeutralOutput();
    }
    if (!interrupted) {
      subTransfer.repositionGamePiece();

      subShooter.setDesiredVelocities(prefShooter.leftShooterSubVelocity.getValue(),
          prefShooter.rightShooterSubVelocity.getValue());
      subShooter.getUpToSpeed();
    } else {
      subTransfer.setTransferNeutralOutput();
    }
    subTransfer.setFeederNeutralOutput();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return subTransfer.calcGamePieceCollected();
  }
}
