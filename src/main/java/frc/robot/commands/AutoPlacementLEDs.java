// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constLEDs;
import frc.robot.RobotPreferences.prefVision;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LEDs;

public class AutoPlacementLEDs extends Command {
  LEDs subLEDs;
  Drivetrain subDrivetrain;

  double desiredStartingPositionX; // TODO: uh find a way to get the actual value
  double desiredStartingPositionY;
  double desiredStartingRotation;
  double translationalAutoPlacementTolerance;
  double rotationalAutoPlacementTolerance;

  public AutoPlacementLEDs(LEDs subLEDs, Drivetrain subDrivetrain, double desiredStartingPositionX,
      double desiredStartingPositionY, double desiredStartingRotation) {
    this.subLEDs = subLEDs;
    this.subDrivetrain = subDrivetrain;
    this.desiredStartingPositionX = desiredStartingPositionX;
    this.desiredStartingPositionY = desiredStartingPositionY;
    this.desiredStartingRotation = desiredStartingRotation;

    translationalAutoPlacementTolerance = prefVision.translationalAutoPlacementTolerance.getValue();
    rotationalAutoPlacementTolerance = prefVision.rotationalAutoPlacementTolerance.getValue();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // checking rotation
    if (Math.abs(desiredStartingRotation
        - subDrivetrain.getPose().getRotation().getDegrees()) <= rotationalAutoPlacementTolerance) {
      subLEDs.setIndividualLED(constLEDs.GREEN_COLOR, 0);
      subLEDs.setIndividualLED(constLEDs.GREEN_COLOR, 3);
      subLEDs.setIndividualLED(constLEDs.GREEN_COLOR, 4);
      subLEDs.setIndividualLED(constLEDs.GREEN_COLOR, 7);
    }

    // checking translational position
    if (Math.abs(desiredStartingPositionX - subDrivetrain.getPose().getX()) <= translationalAutoPlacementTolerance &&
        Math.abs(desiredStartingPositionY - subDrivetrain.getPose().getY()) <= translationalAutoPlacementTolerance) {
      subLEDs.setIndividualLED(constLEDs.GREEN_COLOR, 1);
      subLEDs.setIndividualLED(constLEDs.GREEN_COLOR, 2);
      subLEDs.setIndividualLED(constLEDs.GREEN_COLOR, 5);
      subLEDs.setIndividualLED(constLEDs.GREEN_COLOR, 6);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
