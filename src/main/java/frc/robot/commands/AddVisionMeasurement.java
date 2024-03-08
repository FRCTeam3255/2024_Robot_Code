// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotPreferences.prefDrivetrain;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class AddVisionMeasurement extends Command {

  Drivetrain subDrivetrain;
  Vision subVision;
  Pose2d estimatedPose;
  double timestamp;
  Optional<EstimatedRobotPose> ARresult = Optional.empty();
  Optional<EstimatedRobotPose> OVresult = Optional.empty();

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

    if (ARresult.isPresent() && !RobotState.isAutonomous()) {
      estimatedPose = ARresult.get().estimatedPose.toPose2d();
      timestamp = ARresult.get().timestampSeconds;
      if (estimatedPose.getX() < 5.847097873687744 || estimatedPose.getX() > 10.699200630187988) {
        subDrivetrain.addVisionMeasurement(estimatedPose, timestamp);
        subVision.arPose = estimatedPose;
      }
    }

    if (OVresult.isPresent() && !RobotState.isAutonomous()) {
      estimatedPose = OVresult.get().estimatedPose.toPose2d();
      timestamp = OVresult.get().timestampSeconds;
      if (estimatedPose.getX() < 5.847097873687744 || estimatedPose.getX() > 10.699200630187988) {
        subDrivetrain.addVisionMeasurement(estimatedPose, timestamp);
        subVision.ovPose = estimatedPose;
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
}
