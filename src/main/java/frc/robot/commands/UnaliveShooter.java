// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.LockedLocation;
import frc.robot.RobotPreferences.prefPitch;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class UnaliveShooter extends InstantCommand {
  Shooter subShooter;
  Turret subTurret;
  Pitch subPitch;
  LEDs subLEDs;

  public UnaliveShooter(Shooter subShooter, Turret subTurret, Pitch subPitch, LEDs subLEDs) {
    this.subShooter = subShooter;
    this.subTurret = subTurret;
    this.subPitch = subPitch;
    this.subLEDs = subLEDs;

    addRequirements(subShooter, subTurret, subPitch, subLEDs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.setLockedLocation(LockedLocation.NONE);
    subShooter.setDesiredVelocities(0, 0);
    subTurret.setTurretAngle(Units.Degree.zero());
    subPitch.setPitchAngle(prefPitch.pitchReverseLimit.getMeasure());
    subLEDs.clearAnimation();
    subShooter.setIgnoreFlywheelSpeed(false);
  }
}
