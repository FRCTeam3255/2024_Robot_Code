// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.Constants.constTurret.LockedLocation;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;

public class LockTurret extends Command {
  Turret subTurret;
  Drivetrain subDrivetrain;

  LockedLocation lockedLocation = LockedLocation.NONE;

  Rotation2d desiredAngle = new Rotation2d();

  Optional<Alliance> alliance = DriverStation.getAlliance();

  Pose2d[] fieldPoses;
  Pose2d speakerPose;
  Pose2d ampPose;
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
    speakerPose = fieldPoses[0];
    ampPose = fieldPoses[1];
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    robotPose = subDrivetrain.getPose();
    lockedLocation = subTurret.getLockedLocation();
    // lockedLocation = lockedLocation.SPEAKER;

    if (lockedLocation.equals(LockedLocation.SPEAKER)) {
      double distX = robotPose.getX() - speakerPose.getX();
      double distY = robotPose.getY() - speakerPose.getY();

      desiredAngle = Rotation2d.fromDegrees((-Units.radiansToDegrees(Math.atan2(distX, distY))) + 90);
      desiredAngle = desiredAngle.rotateBy(robotPose.getRotation().unaryMinus());

      // This is an "else if" so that if both are equal somehow, we lock onto the
      // speaker rather than flipping between the two
    } else if (lockedLocation.equals(LockedLocation.AMP)) {
      double distX = robotPose.getX() - ampPose.getX();
      double distY = robotPose.getY() - ampPose.getY();

      desiredAngle = Rotation2d.fromDegrees((-Units.radiansToDegrees(Math.atan2(distX, distY))) + 90);
      desiredAngle = desiredAngle.rotateBy(robotPose.getRotation().unaryMinus());
    }

    if (subTurret.isAnglePossible(desiredAngle.getDegrees())) {
      subTurret.setTurretAngle(desiredAngle.getDegrees());
    }

    SmartDashboard.putNumber("DEBUG - TURRET DESIRED ANGLE", desiredAngle.getDegrees()); // TODO: REMOVE
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
