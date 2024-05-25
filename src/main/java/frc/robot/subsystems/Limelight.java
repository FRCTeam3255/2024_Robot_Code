// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Limelight extends SubsystemBase {

  AprilTagFieldLayout aprilTagFieldLayout;

  public Limelight() {
    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (Exception e) {
      System.out.println("Could not load AprilTagFieldLayout!" + e);
    }

  }

  public PoseEstimate getPoseEstimate() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
  }

  // From Limelight Example Code
  // {
  // LimelightHelpers.SetRobotOrientation("limelight",
  // m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0,
  // 0, 0);
  // LimelightHelpers.PoseEstimate mt2 =
  // LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
  // if(Math.abs(m_gyro.getRate()) > 720) // if our angular velocity is greater
  // than 720 degrees per second, ignore vision updates
  // {
  // doRejectUpdate = true;
  // }
  // if(mt2.tagCount == 0)
  // {
  // doRejectUpdate = true;
  // }
  // if(!doRejectUpdate)
  // {
  // m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
  // m_poseEstimator.addVisionMeasurement(
  // mt2.pose,
  // mt2.timestampSeconds);
  // }
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
