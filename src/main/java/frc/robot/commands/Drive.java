// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
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
  double driveSpeed, xVelocity, yVelocity, rVelocity, slowMultiplier, robotY, robotX;
  Translation2d translationVelocity;
  Pose3d[] fieldPositions;
  Pose2d sourcePose, leftStage, rightStage, centerStage;

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
    fieldPositions = FieldConstants.GET_FIELD_POSITIONS().get();

    sourcePose = fieldPositions[2].toPose2d();
    leftStage = fieldPositions[3].toPose2d();
    centerStage = fieldPositions[4].toPose2d();
    rightStage = fieldPositions[5].toPose2d();
  }

  @Override
  public void execute() {
    if (slowMode.getAsBoolean()) {
      slowMultiplier = prefDrivetrain.slowModeMultiplier.getValue(Units.Value);
    } else {
      slowMultiplier = 1;
    }

    xVelocity = (xAxis.getAsDouble() * driveSpeed) * slowMultiplier;
    yVelocity = (-yAxis.getAsDouble() * driveSpeed) * slowMultiplier;
    rVelocity = -rotationAxis.getAsDouble() * prefDrivetrain.turnSpeed.getValue(Units.RadiansPerSecond);

    translationVelocity = new Translation2d(xVelocity, yVelocity).times(slowMultiplier);

    // If the Driver is providing a rotation manually, don't snap
    if (rVelocity != 0.0) {
    } else {
      // Otherwise, check if snapping is being requested
      if (southTrigger.getAsBoolean()) {
        rVelocity = subDrivetrain.getVelocityToSnap(Rotation2d.fromDegrees(180));

      } else if (westTrigger.getAsBoolean()) {
        rVelocity = subDrivetrain.getVelocityToSnap(Rotation2d.fromDegrees(90));

      } else if (northTrigger.getAsBoolean()) {
        if (getDrivetrainRotation().getAsDouble() > 180) {
          rVelocity = subDrivetrain.getVelocityToSnap(Rotation2d.fromDegrees(355));
        } else {
          rVelocity = subDrivetrain.getVelocityToSnap(Rotation2d.fromDegrees(5));
        }

      } else if (eastTrigger.getAsBoolean()) {
        rVelocity = subDrivetrain.getVelocityToSnap(Rotation2d.fromDegrees(270));

      } else if (sourceTrigger.getAsBoolean()) {
        rVelocity = subDrivetrain.getVelocityToSnap(sourcePose.getRotation());

      } else if (trapTrigger.getAsBoolean()) {
        Rotation2d desiredRotation = subDrivetrain.getDesiredRotForChain(rightStage, leftStage, centerStage);

        if (desiredRotation.getDegrees() == 0) {
          if (getDrivetrainRotation().getAsDouble() > 180) {
            rVelocity = subDrivetrain.getVelocityToSnap(Rotation2d.fromDegrees(355));
          } else {
            rVelocity = subDrivetrain.getVelocityToSnap(Rotation2d.fromDegrees(5));
          }
        } else {
          rVelocity = subDrivetrain.getVelocityToSnap(
              subDrivetrain.getDesiredRotForChain(rightStage, leftStage,
                  centerStage));
        }
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

  public DoubleSupplier getDrivetrainRotation() {
    return () -> subDrivetrain.getRotation().getDegrees();
  }
}
