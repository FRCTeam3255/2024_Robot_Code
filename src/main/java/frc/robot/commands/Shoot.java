// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.frcteam3255.utils.SN_Math;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constLEDs;
import frc.robot.RobotPreferences.prefShooter;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
  Shooter subShooter;
  LEDs subLEDs;

  public Shoot(Shooter subShooter, LEDs subLEDs) {
    this.subShooter = subShooter;
    this.subLEDs = subLEDs;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subShooter.setShootingVelocities(SN_Math.metersToRotations(prefShooter.leftShooterVelocity.getValue(), 1, 0),
        prefShooter.leftShooterFeedForward.getValue(),
        SN_Math.metersToRotations(prefShooter.rightShooterVelocity.getValue(), 1, 0),
        prefShooter.rightShooterFeedForward.getValue());

    // Set LEDs when shooters are up to speed
    if (subShooter.getLeftShooterVelocity() > (prefShooter.leftShooterVelocity.getValue()
        - prefShooter.shooterUpToSpeedTolerance.getValue())
        && subShooter.getRightShooterVelocity() > (prefShooter.rightShooterVelocity.getValue()
            - prefShooter.shooterUpToSpeedTolerance.getValue())) {
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
