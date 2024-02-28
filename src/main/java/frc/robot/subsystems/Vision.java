// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constVision;

public class Vision extends SubsystemBase {
  PhotonPoseEstimator ARCameraPoseEstimator;
  PhotonPoseEstimator OVCameraPoseEstimator;
  AprilTagFieldLayout aprilTagFieldLayout;
  PhotonCamera ARCamera;
  PhotonCamera OVCamera;

  public Vision() {
    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (Exception e) {
      System.out.println("Could not load AprilTagFieldLayout!" + e);
    }

    try {
      ARCamera = new PhotonCamera(constVision.AR_NAME);
    } catch (Exception e) {
      System.out.println("AR Camera not found!");
    }

    try {
      OVCamera = new PhotonCamera(constVision.OV_NAME);
    } catch (Exception e) {
      System.out.println("OV Camera not found!");
    }

    ARCameraPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        ARCamera,
        constVision.ROBOT_TO_AR);

    ARCameraPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    OVCameraPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        OVCamera,
        constVision.ROBOT_TO_OV);

    OVCameraPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

  }

  public Optional<EstimatedRobotPose> getPoseFromARCamera() {
    try {
      return ARCameraPoseEstimator.update();
    } catch (Exception e) {
      return Optional.empty();
    }
  }

  public Optional<EstimatedRobotPose> getPoseFromOVCamera() {
    try {
      return OVCameraPoseEstimator.update();
    } catch (Exception e) {
      return Optional.empty();
    }
  }

  @Override
  public void periodic() {
  }
}
