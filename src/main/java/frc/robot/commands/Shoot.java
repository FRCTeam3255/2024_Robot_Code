// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constLEDs;
import frc.robot.RobotPreferences.prefShooter;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
  Shooter subShooter;
  LEDs subLEDs;
  SN_DoublePreference leftShooterVelocity;
  SN_DoublePreference leftShooterFeedForward;
  SN_DoublePreference rightShooterVelocity;
  SN_DoublePreference rightShooterFeedForward;
  SN_DoublePreference ShooterPitchAngle;

  public Shoot(Shooter subShooter, LEDs subLEDs, SN_DoublePreference givenLeftShooterVelocity,
      SN_DoublePreference givenLeftShooterFeedForward, SN_DoublePreference givenRightShooterVelocity,
      SN_DoublePreference givenRightShooterFeedForward, SN_DoublePreference givenShooterPitchAngle) {
    this.subShooter = subShooter;
    this.subLEDs = subLEDs;
    this.leftShooterVelocity = givenLeftShooterVelocity;
    this.leftShooterFeedForward = givenLeftShooterFeedForward;
    this.rightShooterVelocity = givenRightShooterVelocity;
    this.rightShooterFeedForward = givenRightShooterFeedForward;
    this.ShooterPitchAngle = givenShooterPitchAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subShooter.setShootingVelocities(
        prefShooter.leftShooterVelocity.getValue(),
        prefShooter.leftShooterFeedForward.getValue(),
        prefShooter.rightShooterVelocity.getValue(),
        prefShooter.rightShooterFeedForward.getValue());

    // Set LEDs when shooters are up to speed
    if (subShooter.isLeftShooterAtVelocity(leftShooterVelocity.getValue(),
        prefShooter.shooterUpToSpeedTolerance.getValue())
        && subShooter.isRightShooterAtVelocity(rightShooterVelocity.getValue(),
            prefShooter.shooterUpToSpeedTolerance.getValue())) {
      subLEDs.setLEDs(constLEDs.SHOOTER_UP_TO_SPEED_COLOR);
    } else {
      subLEDs.clearAnimation();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subShooter.setShootingNeutralOutput();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
