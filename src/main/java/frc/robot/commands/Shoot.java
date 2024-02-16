// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.frcteam3255.preferences.SN_DoublePreference;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  public Shoot(Shooter subShooter, LEDs subLEDs, SN_DoublePreference givenLeftShooterVelocity,
      SN_DoublePreference givenLeftShooterFeedForward, SN_DoublePreference givenRightShooterVelocity,
      SN_DoublePreference givenRightShooterFeedForward) {
    this.subShooter = subShooter;
    this.subLEDs = subLEDs;
    this.leftShooterVelocity = givenLeftShooterVelocity;
    this.leftShooterFeedForward = givenLeftShooterFeedForward;
    this.rightShooterVelocity = givenRightShooterVelocity;
    this.rightShooterFeedForward = givenRightShooterFeedForward;
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
        this.leftShooterVelocity.getValue(),
        this.leftShooterFeedForward.getValue(),
        this.rightShooterVelocity.getValue(),
        this.rightShooterFeedForward.getValue());

    // Set LEDs when shooters are up to speed
    if (subShooter.isLeftShooterAtVelocity(leftShooterVelocity.getValue(),
        prefShooter.shooterUpToSpeedTolerance.getValue())
        && subShooter.isRightShooterAtVelocity(rightShooterVelocity.getValue(),
            prefShooter.shooterUpToSpeedTolerance.getValue())) {
      SmartDashboard.putBoolean("At velocity", true);
      subLEDs.setLEDs(constLEDs.SHOOTER_UP_TO_SPEED_COLOR);
    } else {
      subLEDs.clearAnimation();
      SmartDashboard.putBoolean("At velocity", false);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subShooter.setShootingNeutralOutput();
    subLEDs.clearAnimation();
    SmartDashboard.putBoolean("At velocity", false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
