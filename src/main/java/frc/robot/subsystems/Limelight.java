// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;

public class Limelight extends SubsystemBase {

  /**
   * <p>
   * Maximum rate of rotation before we begin rejecting pose updates
   * </p>
   * <b> Units: </b> Rotations per second
   */
  final double maxAngularVelocity = 720;

  /**
   * The area that one tag (if its the only tag in the update) needs to exceed
   * before being accepted
   */
  final double areaThreshold = 0.1;

  public PoseEstimate getPoseEstimate() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
  }

  /**
   * Determines if a given pose estimate should be rejected.
   * 
   * @param poseEstimate The pose estimate to check
   * @param gyroRate     The current rate of rotation observed by our gyro. <b>
   *                     Units: </b> Rotations per second
   * @return True if the estimate should be rejected
   */
  public boolean rejectUpdate(PoseEstimate poseEstimate, double gyroRate) {
    int[] blueTags = { 6, 7, 8, 9, 10, 14, 15, 16 };
    int[] redTags = { 1, 2, 3, 4, 5, 11, 12, 13 };

    if (FieldConstants.isRedAlliance()) {
      // reject blue tags
      for (int tag : blueTags) {
        for (RawFiducial id : poseEstimate.rawFiducials) {
          if (id.id == tag) {
            return true;
          }
        }
      }
    } else {
      // reject red tags
      for (int tag : redTags) {
        for (RawFiducial id : poseEstimate.rawFiducials) {
          if (id.id == tag) {
            return true;
          }
        }
      }
    }

    // Angular velocity is too high to have accurate vision
    // if (Math.abs(gyroRate) > maxAngularVelocity) {
    // return true;
    // }
    // No tags :[
    if (poseEstimate.tagCount == 0) {
      return true;
    }
    // 1 Tag with a large area
    if (poseEstimate.tagCount == 1 && LimelightHelpers.getTA("limelight") > areaThreshold) {
      return false;
      // 2 tags
    } else if (poseEstimate.tagCount > 1) {
      return false;
    }

    return true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
