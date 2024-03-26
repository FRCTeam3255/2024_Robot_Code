// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.util.function.Supplier;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.LockedLocation;
import frc.robot.RobotPreferences.prefIntake;
import frc.robot.RobotPreferences.prefShooter;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.IntakeGroundGamePiece;
import frc.robot.commands.TransferGamePiece;
import frc.robot.commands.UnaliveShooter;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Turret;

public class PreloadTaxi extends SequentialCommandGroup implements AutoInterface {
  Drivetrain subDrivetrain;
  Intake subIntake;
  LEDs subLEDs;
  Pitch subPitch;
  Shooter subShooter;
  Transfer subTransfer;
  Turret subTurret;
  Climber subClimber;

  double FIELD_LENGTH = FieldConstants.FIELD_LENGTH;

  // BLUE
  Pose2d S4B = new Pose2d(1.438, 3.359, Rotation2d.fromRadians(3.142));
  Pose2d S5B = new Pose2d(1.438, 2.059, Rotation2d.fromRadians(3.142));

  // RED
  Pose2d S4R = new Pose2d(FIELD_LENGTH - 1.438, 3.359, Rotation2d.fromRadians(0));
  Pose2d S5R = new Pose2d(FIELD_LENGTH - 1.438, 2.059, Rotation2d.fromRadians(0));

  boolean shoots = false;

  public PreloadTaxi(Drivetrain subDrivetrain, Intake subIntake, LEDs subLEDs, Pitch subPitch, Shooter subShooter,
      Transfer subTransfer, Turret subTurret, Climber subClimber, boolean shoots) {
    this.subDrivetrain = subDrivetrain;
    this.subIntake = subIntake;
    this.subLEDs = subLEDs;
    this.subPitch = subPitch;
    this.subShooter = subShooter;
    this.subTransfer = subTransfer;
    this.subTurret = subTurret;
    this.subClimber = subClimber;
    this.shoots = shoots;

    addCommands(
        Commands.runOnce(
            () -> subDrivetrain.resetPoseToPose(getInitialPose().get())),
        Commands.runOnce(() -> subDrivetrain.resetYaw(
            getInitialPose().get().getRotation().getDegrees())),

        Commands.sequence(
            Commands.runOnce(() -> subShooter.setDesiredVelocities(prefShooter.leftShooterSpeakerVelocity.getValue(),
                prefShooter.rightShooterSpeakerVelocity.getValue())),
            Commands.runOnce(() -> subShooter.getUpToSpeed()),
            Commands.runOnce(() -> RobotContainer.setLockedLocation(LockedLocation.SPEAKER)),
            Commands.runOnce(() -> subTransfer.setTransferSensorAngle(0)),
            Commands.runOnce(() -> subShooter.setIgnoreFlywheelSpeed(false)),

            // throw out that intake
            // Intake until we have the game piece
            new IntakeGroundGamePiece(subIntake, subTransfer, subTurret, subPitch, subShooter, subClimber),

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
            Commands.runOnce(() -> subIntake.setIntakeRollerSpeed(0)),

            new UnaliveShooter(subShooter, subTurret, subPitch, subLEDs),
            Commands.runOnce(() -> subIntake.setPivotAngle(prefIntake.pivotStowAngle.getValue())))
            .unless(() -> !shoots),

        new PathPlannerAuto(determinePathName()));
  }

  public Supplier<Pose2d> getInitialPose() {
    return () -> (FieldConstants.isRedAlliance())
        ? ((shoots) ? S4R : S5R)
        : ((shoots) ? S4B : S5B);
  }

  public double getTurretInitAngle() {
    double isRed = (FieldConstants.isRedAlliance()) ? -1 : 1;
    return -61.712 * isRed;
  }

  public double getPitchInitAngle() {
    return 38.453;
  }

  public String determinePathName() {
    return (shoots) ? "S4Taxi" : "S5Taxi";
  }

  public Command getAutonomousCommand() {
    return this;
  }
}
