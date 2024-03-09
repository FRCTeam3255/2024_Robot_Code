// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotPreferences.prefHood;
import frc.robot.RobotPreferences.prefShooter;
import frc.robot.RobotPreferences.prefTransfer;
import frc.robot.RobotPreferences.prefTurret;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Turret;

public class IntakeFromSource extends Command {
  Shooter subShooter;
  Transfer subTransfer;
  Hood subHood;
  Turret subTurret;
  Climber subClimber;

  double lastDesiredSpeedLeft;
  double lastDesiredSpeedRight;
  double lastDesiredHood;
  double lastDesiredAngle;

  /** Creates a new ShooterIntake. */
  public IntakeFromSource(Shooter subShooter, Transfer subTransfer, Hood subHood, Turret subTurret,
      Climber subClimber) {
    this.subShooter = subShooter;
    this.subTransfer = subTransfer;
    this.subHood = subHood;
    this.subTurret = subTurret;
    this.subClimber = subClimber;

    addRequirements(subShooter, subHood, subTurret, subTransfer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastDesiredSpeedLeft = subShooter.getLeftShooterVelocity();
    lastDesiredSpeedRight = subShooter.getRightShooterVelocity();

    lastDesiredHood = subHood.getHoodAngle();

    subShooter.setDesiredVelocities(prefShooter.leftShooterIntakeVelocity.getValue(),
        prefShooter.rightShooterIntakeVelocity.getValue());
    subShooter.getUpToSpeed();

    subTransfer.setFeederMotorSpeed(prefTransfer.feederIntakeSourceSpeed.getValue());
    subTransfer.setTransferMotorSpeed(prefTransfer.transferIntakeSourceSpeed.getValue());

    subHood.setHoodAngle(prefHood.hoodSourceAngle.getValue(), subClimber.collidesWithHood());

    subTurret.setTurretAngle(prefTurret.turretIntakePos.getValue(), subClimber.collidesWithTurret());

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
    subHood.setHoodAngle(lastDesiredHood, subClimber.collidesWithHood());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
