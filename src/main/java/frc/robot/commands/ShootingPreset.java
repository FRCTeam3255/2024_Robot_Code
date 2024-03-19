// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotPreferences.prefPitch;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class ShootingPreset extends Command {
  Shooter subShooter;
  Turret subTurret;
  Pitch subPitch;

  double desiredLeftVelocity;
  double desiredRightVelocity;
  double desiredTurretAngle;
  double desiredPitchAngle;
  boolean ignoreFlywheelSpeed;

  /** Creates a new ShootingPreset. */
  public ShootingPreset(Shooter subShooter, Turret subTurret, Pitch subPitch, double desiredLeftVelocity,
      double desiredRightVelocity, double desiredTurretAngle, double desiredPitchAngle, boolean ignoreFlywheelSpeed) {
    this.subShooter = subShooter;
    this.subTurret = subTurret;
    this.subPitch = subPitch;

    this.desiredLeftVelocity = desiredLeftVelocity;
    this.desiredRightVelocity = desiredRightVelocity;
    this.desiredTurretAngle = desiredTurretAngle;
    this.desiredPitchAngle = desiredPitchAngle;
    this.ignoreFlywheelSpeed = ignoreFlywheelSpeed;

    addRequirements(subShooter, subTurret, subPitch);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subShooter.setDesiredVelocities(desiredLeftVelocity, desiredRightVelocity);
    subTurret.setTurretAngle(desiredTurretAngle);
    subPitch.setPitchAngle(desiredPitchAngle);
    subShooter.setIgnoreFlywheelSpeed(ignoreFlywheelSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subShooter.getUpToSpeed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subTurret.setTurretAngle(0);
    subPitch.setPitchAngle(prefPitch.pitchReverseLimit.getValue());
    subShooter.setIgnoreFlywheelSpeed(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
