// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.LockedLocation;
import frc.robot.RobotPreferences.prefShooter;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.IntakeGroundGamePiece;
import frc.robot.commands.TransferGamePiece;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Turret;

public class PreloadOnly extends SequentialCommandGroup implements AutoInterface {
  Drivetrain subDrivetrain;
  Intake subIntake;
  LEDs subLEDs;
  Pitch subPitch;
  Shooter subShooter;
  Transfer subTransfer;
  Turret subTurret;
  Climber subClimber;

  double FIELD_LENGTH = FieldConstants.FIELD_LENGTH;

  int startingPosition = 0;
  // BLUE
  Pose2d S1B = new Pose2d(0.602, 6.747, Rotation2d.fromRadians(-2.094));
  Pose2d S2B = new Pose2d(1.360, 5.563, Rotation2d.fromRadians(3.142));
  Pose2d S3B = new Pose2d(0.602, 4.348, Rotation2d.fromRadians(2.094395));
  Pose2d S4B = new Pose2d(1.438, 3.359, Rotation2d.fromRadians(3.142));
  Pose2d S5B = new Pose2d(1.438, 2.059, Rotation2d.fromRadians(3.142));
  Pose2d[] startingPositionsBlue = { S1B, S2B, S3B, S4B, S5B };

  // RED
  Pose2d S1R = new Pose2d(FIELD_LENGTH - 0.602, 6.747, Rotation2d.fromRadians(-2.68780728));
  Pose2d S2R = new Pose2d(FIELD_LENGTH - 1.360, 5.563, Rotation2d.fromRadians(0));
  Pose2d S3R = new Pose2d(FIELD_LENGTH - 0.602, 4.348, Rotation2d.fromRadians(2.68780728));
  Pose2d S4R = new Pose2d(FIELD_LENGTH - 1.438, 3.359, Rotation2d.fromRadians(0));
  Pose2d S5R = new Pose2d(FIELD_LENGTH - 1.438, 2.059, Rotation2d.fromRadians(0));
  Pose2d[] startingPositionsRed = { S1R, S2R, S3R, S4R, S5R };

  boolean shoots = true;

  /*
   * @param startingPosition Your desired starting position. 0 -> 4. Please refer
   * to Choreo for this
   */
  public PreloadOnly(Drivetrain subDrivetrain, Intake subIntake, LEDs subLEDs, Pitch subPitch, Shooter subShooter,
      Transfer subTransfer, Turret subTurret, Climber subClimber, int startingPosition, boolean shoots) {
    this.subDrivetrain = subDrivetrain;
    this.subIntake = subIntake;
    this.subLEDs = subLEDs;
    this.subPitch = subPitch;
    this.subShooter = subShooter;
    this.subTransfer = subTransfer;
    this.subTurret = subTurret;
    this.subClimber = subClimber;
    this.startingPosition = startingPosition;
    this.shoots = shoots;

    addCommands(
        Commands.runOnce(
            () -> subDrivetrain.resetPoseToPose(getInitialPose().get())),

        Commands.sequence(
            Commands.runOnce(() -> subShooter.setDesiredVelocities(prefShooter.leftShooterSpeakerVelocity.getValue(),
                prefShooter.rightShooterSpeakerVelocity.getValue())),
            Commands.runOnce(() -> subShooter.getUpToSpeed()),
            Commands.runOnce(() -> RobotContainer.setLockedLocation(LockedLocation.SPEAKER)),
            Commands.runOnce(() -> subTransfer.setTransferSensorAngle(0)),
            Commands.runOnce(() -> subShooter.setIgnoreFlywheelSpeed(false)),

            // PRELOAD
            // Aim
            Commands.parallel(
                Commands.run(() -> subTurret.setTurretAngle(getTurretInitAngle()))
                    .until(() -> subTurret.isTurretAtAngle(getTurretInitAngle())),
                Commands.run(() -> subPitch.setPitchAngle(getPitchInitAngle()))
                    .until(() -> subPitch.isPitchAtAngle(getPitchInitAngle()))),

            Commands.runOnce(() -> subShooter.getUpToSpeed()),

            // Shoot
            new TransferGamePiece(subShooter, subTurret, subTransfer, subPitch, subIntake, subClimber)
                .until(() -> subTransfer.calcGPShotAuto()),
            Commands.runOnce(() -> subIntake.setIntakeRollerSpeed(0))).unless(() -> !shoots)

    );
  }

  public Supplier<Pose2d> getInitialPose() {
    return () -> (FieldConstants.isRedAlliance())
        ? startingPositionsRed[startingPosition]
        : startingPositionsBlue[startingPosition];
  }

  public double getTurretInitAngle() {
    double isRed = (FieldConstants.isRedAlliance()) ? -1 : 1;
    switch (startingPosition) {
      case 0: // S1
        return 0 * isRed;
      case 1: // S2
        return 0.0 * isRed;
      case 2: // S3
        return 0 * isRed;
      case 3: // S4
        return -61.712 * isRed;
      case 4: // S5 (We've NEVER run this one)
        return -3255 * isRed;
      default:
        return -3255 * isRed;

    }
  }

  public double getPitchInitAngle() {
    switch (startingPosition) {
      case 0: // S1
        return 55;
      case 1: // S2
        return 55;
      case 2: // S3
        return 55;
      case 3: // S4
        return 38.453;
      case 4: // S5 (We've NEVER run this one)
        return -3255;
      default:
        return -3255;
    }
  }

  public Command getAutonomousCommand() {
    return this;
  }
}
