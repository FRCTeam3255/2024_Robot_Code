// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constLEDs;
import frc.robot.RobotPreferences.prefShooter;

import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Turret;

public class Shoot extends Command {
  Shooter subShooter;
  LEDs subLEDs;

  public Shoot(Shooter subShooter, LEDs subLEDs) {
    this.subShooter = subShooter;
    this.subLEDs = subLEDs;

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
    subShooter.getUpToSpeed();

    // Set LEDs when shooters are up to speed
    if (subShooter.areBothShootersUpToSpeed()) {
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
