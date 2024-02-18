// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constDrivetrain;
import frc.robot.RobotPreferences.prefDrivetrain;
import frc.robot.subsystems.Drivetrain;

public class Drive extends Command {
  Drivetrain subDrivetrain;
  DoubleSupplier xAxis, yAxis, rotationAxis;
  boolean isOpenLoop;

  boolean isPracticeBot;
  double driveSpeed;

  public Drive(Drivetrain subDrivetrain, DoubleSupplier xAxis, DoubleSupplier yAxis,
      DoubleSupplier rotationAxis, boolean isPracticeBot) {
    this.subDrivetrain = subDrivetrain;
    this.xAxis = xAxis;
    this.yAxis = yAxis;
    this.rotationAxis = rotationAxis;
    this.isPracticeBot = isPracticeBot;

    isOpenLoop = false;

    addRequirements(this.subDrivetrain);
  }

  @Override
  public void initialize() {
    driveSpeed = (isPracticeBot) ? constDrivetrain.pracBot.DRIVE_SPEED : constDrivetrain.DRIVE_SPEED;
  }

  @Override
  public void execute() {
    // Get Joystick inputs
    double xVelocity = xAxis.getAsDouble() * driveSpeed;
    double yVelocity = -yAxis.getAsDouble() * driveSpeed;
    double rVelocity = -rotationAxis.getAsDouble() * Units.degreesToRadians(prefDrivetrain.turnSpeed.getValue());

    subDrivetrain.drive(new Translation2d(xVelocity, yVelocity), rVelocity, isOpenLoop);
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
