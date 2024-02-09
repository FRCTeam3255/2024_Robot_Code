// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

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
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;

public class LockTurret extends Command {
  Turret subTurret;
  Drivetrain subDrivetrain;

  LockedLocation lockedLocation = LockedLocation.NONE;

  Rotation2d desiredAngle = new Rotation2d();

  Optional<Alliance> alliance = DriverStation.getAlliance();

  Pose3d[] fieldPoses;
  Pose3d speakerPose;
  Pose3d ampPose;
  Pose2d robotPose = new Pose2d();

  public LockTurret(Turret subTurret, Drivetrain subDrivetrain) {
    this.subTurret = subTurret;
    this.subDrivetrain = subDrivetrain;

    addRequirements(subTurret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    desiredAngle = Rotation2d.fromDegrees(subTurret.getAngle());

    fieldPoses = FieldConstants.GET_FIELD_POSITIONS();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    robotPose = subDrivetrain.getPose();

    Optional<Rotation2d> calculatedAngle = subTurret.getDesiredAngleToLock(robotPose, fieldPoses,
        RobotContainer.getLockedLocation());

    if (calculatedAngle.isPresent()) {
      desiredAngle = calculatedAngle.get();
    }
    // Otherwise, try to go to the last calculated angle

    SmartDashboard.putNumber("DEBUG - TURRET DESIRED ANGLE", desiredAngle.getDegrees());
    SmartDashboard.putString("DEBUG - LOCKED LOCATION", RobotContainer.getLockedLocation().toString());
    if (subTurret.isAnglePossible(desiredAngle.getDegrees())) {
      subTurret.setTurretAngle(desiredAngle.getDegrees());
    }

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
