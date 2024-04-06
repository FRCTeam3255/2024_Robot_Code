// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.LockedLocation;
import frc.robot.Constants.constLEDs;
import frc.robot.RobotContainer;
import frc.robot.RobotPreferences.prefIntake;
import frc.robot.RobotPreferences.prefPitch;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class SmileyDefense extends InstantCommand {
  Climber subClimber;
  Intake subIntake;
  Turret subTurret;
  Pitch subPitch;
  Shooter subShooter;
  LEDs subLEDs;

  /**
   * Defensive preset inspired by FRC 9452 - LHS Steel Stingers
   */
  public SmileyDefense(Climber subClimber, Intake subIntake, Turret subTurret, Pitch subPitch, Shooter subShooter,
      LEDs subLEDs) {
    this.subClimber = subClimber;
    this.subIntake = subIntake;
    this.subTurret = subTurret;
    this.subPitch = subPitch;
    this.subShooter = subShooter;
    this.subLEDs = subLEDs;

    addRequirements(subClimber, subIntake, subTurret, subPitch, subShooter, subLEDs);
  }

  @Override
  public void initialize() {
    RobotContainer.setLockedLocation(LockedLocation.NONE);
    subShooter.setDesiredVelocities(0, 0);
    subTurret.setTurretAngle(0);
    subPitch.setPitchAngle(Units.rotationsToDegrees(prefPitch.pitchReverseLimit.getValue()));
    subLEDs.setLEDsToAnimation(constLEDs.PANIC_ANIMATION);
    subShooter.setIgnoreFlywheelSpeed(false);

    subClimber.setPosition(1.9);
    subIntake.setPivotAngle(prefIntake.pivotMaxPos.getValue() - 1);
  }
}
