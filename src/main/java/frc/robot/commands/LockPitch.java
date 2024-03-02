// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.Constants.LockedLocation;
import frc.robot.RobotPreferences.prefPitch;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pitch;

public class LockPitch extends Command {
  Pitch subPitch;
  Drivetrain subDrivetrain;
  Climber subClimber;

  LockedLocation lockedLocation = LockedLocation.NONE;

  Rotation2d desiredAngle = new Rotation2d();

  Optional<Alliance> alliance = DriverStation.getAlliance();

  Pose3d[] fieldPoses;
  Pose3d speakerPose;
  Pose3d ampPose;
  Pose2d robotPose = new Pose2d();

  public LockPitch(Pitch subPitch, Drivetrain subDrivetrain, Climber subClimber) {
    this.subPitch = subPitch;
    this.subDrivetrain = subDrivetrain;
    this.subClimber = subClimber;

    addRequirements(subPitch);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    desiredAngle = Rotation2d.fromDegrees(subPitch.getPitchAngle());

    fieldPoses = FieldConstants.GET_FIELD_POSITIONS();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    robotPose = subDrivetrain.getPose();

    Optional<Rotation2d> calculatedAngle = subPitch.getDesiredAngleToLock(robotPose, fieldPoses,
        RobotContainer.getLockedLocation());

    if (calculatedAngle.isPresent()) {
      desiredAngle = Rotation2d.fromRotations(
          MathUtil.clamp(
              calculatedAngle.get().getRotations(),
              prefPitch.pitchReverseLimit.getValue(),
              prefPitch.pitchForwardLimit.getValue()));

      subPitch.desiredLockingPitch = desiredAngle.getDegrees();

      subPitch.setPitchAngle(desiredAngle.getDegrees(), subClimber.collidesWithPitch());
    }
    SmartDashboard.putNumber("Pitch/Locking Desired Angle", desiredAngle.getDegrees());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
