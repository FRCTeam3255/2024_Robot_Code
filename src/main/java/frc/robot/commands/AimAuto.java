// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LockedLocation;
import frc.robot.RobotPreferences.prefPitch;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Turret;

public class AimAuto extends Command {
  Pitch subPitch;
  Turret subTurret;

  Drivetrain subDrivetrain;

  LockedLocation lockedLocation = LockedLocation.NONE;

  Measure<Angle> desiredPitchAngle;
  Measure<Angle> desiredTurretAngle;

  Optional<Alliance> alliance = DriverStation.getAlliance();

  Pose3d[] fieldPoses;
  Pose3d speakerPose;
  Pose3d ampPose;
  Pose2d robotPose = new Pose2d();

  /** Creates a new AimAuto. */
  public AimAuto(Pitch subPitch, Turret subTurret, Drivetrain subDrivetrain) {
    this.subPitch = subPitch;
    this.subTurret = subTurret;
    this.subDrivetrain = subDrivetrain;

    addRequirements(subPitch, subTurret);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    desiredPitchAngle = subPitch.getPitchAngle();
    desiredTurretAngle = subTurret.getAngle();

    fieldPoses = FieldConstants.GET_FIELD_POSITIONS().get();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    fieldPoses = FieldConstants.GET_FIELD_POSITIONS().get();
    SmartDashboard.putString("SPEAKER according to AUTO AIM!!", fieldPoses[0].toString());

    // AIM PITCH
    robotPose = subDrivetrain.getPose();

    Optional<Measure<Angle>> calculatedPitchAngle = subPitch.getDesiredAngleToLock(robotPose, fieldPoses,
        RobotContainer.getLockedLocation());

    if (calculatedPitchAngle.isPresent()) {
      desiredPitchAngle = Units.Rotations.of(MathUtil.clamp(
          calculatedPitchAngle.get().in(Units.Rotations),
          prefPitch.pitchReverseLimit.getValue(Units.Rotations),
          prefPitch.pitchForwardLimit.getValue(Units.Rotations)));
    }

    subPitch.setPitchAngle(desiredPitchAngle);

    robotPose = subDrivetrain.getPose();

    Optional<Measure<Angle>> calculatedTurretAngle = subTurret.getDesiredAngleToLock(robotPose, fieldPoses,
        RobotContainer.getLockedLocation());

    if (calculatedTurretAngle.isPresent()) {
      desiredTurretAngle = calculatedTurretAngle.get();
      if (subTurret.isAnglePossible(desiredTurretAngle)) {
        subTurret.setTurretAngle(desiredTurretAngle);
      }

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return subPitch.isPitchLocked() && subTurret.isTurretLocked();
  }
}
