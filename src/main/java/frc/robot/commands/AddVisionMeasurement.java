// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotPreferences.prefVision;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class AddVisionMeasurement extends Command {

  Drivetrain subDrivetrain;
  Vision subVision;
  Pose2d estimatedPose;
  double timestamp;
  Optional<EstimatedRobotPose> ARresult = Optional.empty();
  Optional<EstimatedRobotPose> OVresult = Optional.empty();

  EstimatedRobotPose arResult;
  EstimatedRobotPose ovResult;

  Rotation3d arRotation = new Rotation3d();
  Rotation3d ovRotation = new Rotation3d();
  double drivetrainRotation = 0;

  double arRotRelativeToBot = 0;
  double ovRotRelativeToBot = 0;

  Vector<N3> multiTagSTDevs = VecBuilder.fill(
      Units.feetToMeters(prefVision.multiTagStdDevsPosition.getValue()),
      Units.feetToMeters(prefVision.multiTagStdDevsPosition.getValue()),
      Units.degreesToRadians(prefVision.multiTagStdDevsHeading.getValue()));

  // Not used because we reject all single-tag estimates for now
  Vector<N3> singleTagSTDevs = VecBuilder.fill(
      Units.feetToMeters(prefVision.singleTagStdDevsPosition.getValue()),
      Units.feetToMeters(prefVision.singleTagStdDevsPosition.getValue()),
      Units.degreesToRadians(prefVision.singleTagStdDevsHeading.getValue()));

  public AddVisionMeasurement(Drivetrain subDrivetrain, Vision subVision) {
    this.subDrivetrain = subDrivetrain;
    this.subVision = subVision;
    addRequirements(subVision);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    ARresult = subVision.getPoseFromARCamera();
    OVresult = subVision.getPoseFromOVCamera();

    // Don't collect Vision results in auto
    if (!RobotState.isAutonomous() && !RobotState.isDisabled()) {

      // If both results are present, pick the closest of the 2 poses
      if (ARresult.isPresent() && OVresult.isPresent()) {
        arResult = ARresult.get();
        ovResult = OVresult.get();

        // If both are viable...
        if (isEstimateViable(arResult) && isEstimateViable(ovResult)) {
          // Do the one thats closest in rotation
          arRotation = arResult.estimatedPose.getRotation();
          ovRotation = ovResult.estimatedPose.getRotation();
          drivetrainRotation = subDrivetrain.getRotation().getDegrees();

          // Calculate angular distance considering wrapping around 360 degrees
          arRotRelativeToBot = Math.min(Math.abs(drivetrainRotation - arRotation.getZ()),
              360 - Math.abs(drivetrainRotation - arRotation.getZ()));

          // Calculate angular distance considering wrapping around 360 degrees
          ovRotRelativeToBot = Math.min(Math.abs(drivetrainRotation - ovRotation.getZ()),
              360 - Math.abs(drivetrainRotation - ovRotation.getZ()));

          if (arRotRelativeToBot < ovRotRelativeToBot) {
            // log ar rotation
            estimatedPose = arResult.estimatedPose.toPose2d();
            subDrivetrain.addVisionMeasurement(estimatedPose, timestamp, multiTagSTDevs);
            subVision.arPose = estimatedPose;

          } else {
            // log ov rotation
            estimatedPose = ovResult.estimatedPose.toPose2d();
            subDrivetrain.addVisionMeasurement(estimatedPose, timestamp, multiTagSTDevs);
            subVision.ovPose = estimatedPose;

          }

        }

        // Otherwise, only one or neither are viable so check accordingly
        else if (isEstimateViable(arResult)) {
          estimatedPose = arResult.estimatedPose.toPose2d();
          subDrivetrain.addVisionMeasurement(estimatedPose, timestamp, multiTagSTDevs);
          subVision.arPose = estimatedPose;
        }

        else if (isEstimateViable(ovResult)) {
          estimatedPose = ovResult.estimatedPose.toPose2d();
          subDrivetrain.addVisionMeasurement(estimatedPose, timestamp, multiTagSTDevs);
          subVision.ovPose = estimatedPose;
        }
      }

      // Otherwise, check if each one is present and add it if its viable
      else if (ARresult.isPresent()) {
        arResult = ARresult.get();
        estimatedPose = ARresult.get().estimatedPose.toPose2d();
        timestamp = ARresult.get().timestampSeconds;

        if (isEstimateViable(arResult)) {
          subDrivetrain.addVisionMeasurement(estimatedPose, timestamp, multiTagSTDevs);
          subVision.arPose = estimatedPose;
        }
      }

      else if (OVresult.isPresent()) {
        ovResult = OVresult.get();
        estimatedPose = ovResult.estimatedPose.toPose2d();
        timestamp = ovResult.timestampSeconds;

        if (isEstimateViable(ovResult)) {
          subDrivetrain.addVisionMeasurement(estimatedPose, timestamp, multiTagSTDevs);
          subVision.ovPose = estimatedPose;
        }
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public boolean isEstimateViable(EstimatedRobotPose estimate) {
    // Single Tag poses are rejected - Only use Multi-tag
    if (estimate.targetsUsed.size() != 1) {
      estimatedPose = estimate.estimatedPose.toPose2d();

      // Don't collect pose estimates from outside of the wings
      if (estimatedPose.getX() < 5.847097873687744 || estimatedPose.getX() > 10.699200630187988) {
        return true;
      }
    }
    return false;
  }
}
