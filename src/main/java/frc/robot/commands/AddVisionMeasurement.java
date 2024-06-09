// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class AddVisionMeasurement extends Command {
  Drivetrain subDrivetrain;
  Limelight subLimelight;

  PoseEstimate estimatedPose;
  double drivetrainRotation = 0;

  public AddVisionMeasurement(Drivetrain subDrivetrain, Limelight subLimelight) {
    this.subDrivetrain = subDrivetrain;
    this.subLimelight = subLimelight;

    addRequirements(subLimelight);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    LimelightHelpers.SetRobotOrientation("limelight",
        subDrivetrain.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);

    estimatedPose = subLimelight.getPoseEstimate();

    if (!subLimelight.rejectUpdate(estimatedPose, subDrivetrain.getGyroRate())) {
      subDrivetrain.addVisionMeasurement(estimatedPose.pose, estimatedPose.timestampSeconds);
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
