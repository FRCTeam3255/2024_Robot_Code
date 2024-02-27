// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FieldConstants;
import frc.robot.Constants.constDrivetrain;
import frc.robot.RobotPreferences.prefDrivetrain;
import frc.robot.subsystems.Drivetrain;

public class Drive extends Command {
  Drivetrain subDrivetrain;
  DoubleSupplier xAxis, yAxis, rotationAxis;
  boolean isPracticeBot, isOpenLoop;
  Trigger slowMode, northTrigger, eastTrigger, southTrigger, westTrigger, sourceTrigger, trapTrigger;

  double driveSpeed, xVelocity, yVelocity, rVelocity, slowMultiplier, robotY;
  Translation2d translationVelocity;
  Rotation2d sourceAngle, leftStageAngle, rightStageAngle, centerStageAngle;
  Pose3d[] fieldPositions;

  public Drive(
      Drivetrain subDrivetrain,
      DoubleSupplier xAxis,
      DoubleSupplier yAxis,
      DoubleSupplier rotationAxis,
      Trigger slowMode,
      Trigger northTrigger,
      Trigger eastTrigger,
      Trigger southTrigger,
      Trigger westTrigger,
      Trigger sourceTrigger,
      Trigger trapTrigger,
      boolean isPracticeBot) {

    this.subDrivetrain = subDrivetrain;
    this.xAxis = xAxis;
    this.yAxis = yAxis;
    this.rotationAxis = rotationAxis;
    this.slowMode = slowMode;
    this.isPracticeBot = isPracticeBot;

    this.northTrigger = northTrigger;
    this.eastTrigger = eastTrigger;
    this.southTrigger = southTrigger;
    this.westTrigger = westTrigger;
    this.sourceTrigger = sourceTrigger;
    this.trapTrigger = trapTrigger;

    isOpenLoop = false;

    addRequirements(this.subDrivetrain);
  }

  @Override
  public void initialize() {
    driveSpeed = (isPracticeBot) ? constDrivetrain.pracBot.DRIVE_SPEED : constDrivetrain.DRIVE_SPEED;
    fieldPositions = FieldConstants.GET_FIELD_POSITIONS();

    sourceAngle = fieldPositions[2].getRotation().toRotation2d();
    leftStageAngle = fieldPositions[3].getRotation().toRotation2d();
    centerStageAngle = fieldPositions[4].getRotation().toRotation2d();
    rightStageAngle = fieldPositions[5].getRotation().toRotation2d();
  }

  @Override
  public void execute() {
    // Get inputs
    slowMultiplier = (slowMode.getAsBoolean()) ? prefDrivetrain.slowModeMultiplier.getValue() : 1;
    xVelocity = xAxis.getAsDouble() * driveSpeed;
    yVelocity = -yAxis.getAsDouble() * driveSpeed;
    rVelocity = -rotationAxis.getAsDouble() * Units.degreesToRadians(prefDrivetrain.turnSpeed.getValue());

    translationVelocity = new Translation2d(xVelocity, yVelocity).times(slowMultiplier);

    // If the Driver is providing a rotation manually, don't snap
    if (rVelocity != 0.0) {
    } else {
      // Otherwise, check if snapping is being requested
      if (northTrigger.getAsBoolean()) {
        rVelocity = subDrivetrain.getVelocityToSnap(Rotation2d.fromDegrees(180));
      } else if (eastTrigger.getAsBoolean()) {
        rVelocity = subDrivetrain.getVelocityToSnap(Rotation2d.fromDegrees(90));
      } else if (southTrigger.getAsBoolean()) {
        rVelocity = subDrivetrain.getVelocityToSnap(Rotation2d.fromDegrees(0));
      } else if (westTrigger.getAsBoolean()) {
        rVelocity = subDrivetrain.getVelocityToSnap(Rotation2d.fromDegrees(-90));
      } else if (sourceTrigger.getAsBoolean()) {
        rVelocity = subDrivetrain.getVelocityToSnap(sourceAngle);
      } else if (trapTrigger.getAsBoolean()) {
        String debugString = "Closest Chain: NONE";
        // TODO: MAKE THIS USE MATH INSTEAD OF SCUFFING IT
        // https://www.desmos.com/calculator/l1ntqvcuv3

        // Current implementation: This will snap to the left or right chain. If we want
        // to snap to the center chain, just use cardinal directons
        robotY = subDrivetrain.getPose().getY();
        if (robotY > 4.114171028137207) { // TODO: MAKE CONSTANT (AFTER PHR)
          rVelocity = subDrivetrain.getVelocityToSnap(leftStageAngle);
          debugString = "Closest Chain: LEFT";

        } else {
          rVelocity = subDrivetrain.getVelocityToSnap(rightStageAngle);
          debugString = "Closest Chain: RIGHT";

        }

        System.out.println(debugString);
      }
    }

    subDrivetrain.drive(translationVelocity, rVelocity, isOpenLoop);
  }

  @Override
  public void end(boolean interrupted) {
    subDrivetrain.neutralDriveOutputs();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
