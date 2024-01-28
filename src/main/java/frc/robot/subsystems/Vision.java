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
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constVision;

public class Vision extends SubsystemBase {
  PhotonPoseEstimator ARCameraPoseEstimator;
  PhotonPoseEstimator OVCameraPoseEstimator;
  AprilTagFieldLayout aprilTagFieldLayout;

  public Vision() {
    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (Exception e) {
      System.out.println("Could not load AprilTagFieldLayout!" + e);
    }

    PhotonCamera ARCamera = new PhotonCamera(constVision.AR_NAME);
    PhotonCamera OVCamera = new PhotonCamera(constVision.OV_NAME);

    // TODO: Must configure the AprilTagFieldLayout properly in the UI, please see
    // here
    // (https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/multitag.html#multitag-localization)
    // for more information.
    ARCameraPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        ARCamera,
        constVision.ROBOT_TO_AR);

    OVCameraPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        OVCamera,
        constVision.ROBOT_TO_OV);
  }

  public Optional<EstimatedRobotPose> getPoseFromARCamera() {
    try {
      return ARCameraPoseEstimator.update();
    } catch (Exception e) {
      return null;
    }
  }

  public Optional<EstimatedRobotPose> getPoseFromOVCamera() {
    try {
      return OVCameraPoseEstimator.update();
    } catch (Exception e) {
      return null;
    }
  }

  @Override
  public void periodic() {
  }
}
