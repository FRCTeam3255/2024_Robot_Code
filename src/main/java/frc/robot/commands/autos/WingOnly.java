// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.LockedLocation;
import frc.robot.RobotPreferences.prefShooter;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.IntakeGroundGamePiece;
import frc.robot.commands.LockPitch;
import frc.robot.commands.LockTurret;
import frc.robot.commands.Shoot;
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

public class WingOnly extends SequentialCommandGroup implements AutoInterface {
  Drivetrain subDrivetrain;
  Intake subIntake;
  LEDs subLEDs;
  Pitch subPitch;
  Shooter subShooter;
  Transfer subTransfer;
  Turret subTurret;
  Climber subClimber;

  double FIELD_LENGTH = FieldConstants.FIELD_LENGTH;

  boolean goesDown = false;

  /**
   * @param goesDown If your path goes up or down
   */
  public WingOnly(Drivetrain subDrivetrain, Intake subIntake, LEDs subLEDs, Pitch subPitch, Shooter subShooter,
      Transfer subTransfer, Turret subTurret, Climber subClimber, boolean goesDown) {
    this.subDrivetrain = subDrivetrain;
    this.subIntake = subIntake;
    this.subLEDs = subLEDs;
    this.subPitch = subPitch;
    this.subShooter = subShooter;
    this.subTransfer = subTransfer;
    this.subTurret = subTurret;
    this.subClimber = subClimber;
    this.goesDown = goesDown;

    addCommands(
        Commands.runOnce(
            () -> subDrivetrain.resetPoseToPose(getInitialPose().get())),
        Commands.runOnce(() -> subDrivetrain.resetYaw(
            getInitialPose().get().getRotation().getDegrees())),

        // throw out that intake
        // Intake until we have the game piece
        new IntakeGroundGamePiece(subIntake, subTransfer, subTurret, subPitch, subShooter, subClimber).withTimeout(2),
        // TODO: REMOVE TIME OUT
        // Redundant call (makes it easier for sim testing)
        Commands.runOnce(() -> subTransfer.setGamePieceCollected(true)),

        Commands.race(
            new Shoot(subShooter, subLEDs).repeatedly(),
            new LockTurret(subTurret, subDrivetrain).repeatedly(),
            new LockPitch(subPitch, subDrivetrain).repeatedly(),

            // Shooting the game piece
            Commands.sequence(
                // Aim
                Commands.parallel(
                    Commands.runOnce(() -> RobotContainer.setLockedLocation(LockedLocation.SPEAKER)),
                    Commands
                        .runOnce(
                            () -> subShooter.setDesiredVelocities(prefShooter.leftShooterSpeakerVelocity.getValue(),
                                prefShooter.rightShooterSpeakerVelocity.getValue())),
                    Commands.runOnce(() -> subShooter.setIgnoreFlywheelSpeed(false))),

                // Shoot
                new TransferGamePiece(subShooter, subTurret, subTransfer, subPitch, subIntake)
                    .until(() -> !subTransfer.hasGamePiece).withTimeout(2),
                Commands.parallel(
                    Commands.runOnce(() -> subTransfer.setFeederNeutralOutput()),
                    Commands.runOnce(() -> subTransfer.setTransferNeutralOutput())))),

        // Go get W1
        new UnaliveShooter(subShooter, subTurret, subPitch, subLEDs),
        AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(determinePathName() + ".1")),

        // Shoot W1
        Commands.race(
            new Shoot(subShooter, subLEDs).repeatedly(),
            new LockTurret(subTurret, subDrivetrain).repeatedly(),
            new LockPitch(subPitch, subDrivetrain).repeatedly(),

            // Shooting the game piece
            Commands.sequence(
                // Aim
                Commands.parallel(
                    Commands.runOnce(() -> RobotContainer.setLockedLocation(LockedLocation.SPEAKER)),
                    Commands
                        .runOnce(
                            () -> subShooter.setDesiredVelocities(prefShooter.leftShooterSpeakerVelocity.getValue(),
                                prefShooter.rightShooterSpeakerVelocity.getValue())),
                    Commands.runOnce(() -> subShooter.setIgnoreFlywheelSpeed(false))),

                // Shoot
                new TransferGamePiece(subShooter, subTurret, subTransfer, subPitch, subIntake)
                    .until(() -> !subTransfer.hasGamePiece).withTimeout(2),
                Commands.parallel(
                    Commands.runOnce(() -> subTransfer.setFeederNeutralOutput()),
                    Commands.runOnce(() -> subTransfer.setTransferNeutralOutput())))),

        // Go get W2
        new UnaliveShooter(subShooter, subTurret, subPitch, subLEDs),
        AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(determinePathName() + ".2")),

        // Shoot W2
        Commands.race(
            new Shoot(subShooter, subLEDs).repeatedly(),
            new LockTurret(subTurret, subDrivetrain).repeatedly(),
            new LockPitch(subPitch, subDrivetrain).repeatedly(),

            // Shooting the game piece
            Commands.sequence(
                // Aim
                Commands.parallel(
                    Commands.runOnce(() -> RobotContainer.setLockedLocation(LockedLocation.SPEAKER)),
                    Commands
                        .runOnce(
                            () -> subShooter.setDesiredVelocities(prefShooter.leftShooterSpeakerVelocity.getValue(),
                                prefShooter.rightShooterSpeakerVelocity.getValue())),
                    Commands.runOnce(() -> subShooter.setIgnoreFlywheelSpeed(false))),

                // Shoot
                new TransferGamePiece(subShooter, subTurret, subTransfer, subPitch, subIntake)
                    .until(() -> !subTransfer.hasGamePiece).withTimeout(2),
                Commands.parallel(
                    Commands.runOnce(() -> subTransfer.setFeederNeutralOutput()),
                    Commands.runOnce(() -> subTransfer.setTransferNeutralOutput())))),

        // Go get W3
        new UnaliveShooter(subShooter, subTurret, subPitch, subLEDs),
        AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(determinePathName() + ".3")),

        // Shoot W3
        Commands.race(
            new Shoot(subShooter, subLEDs).repeatedly(),
            new LockTurret(subTurret, subDrivetrain).repeatedly(),
            new LockPitch(subPitch, subDrivetrain).repeatedly(),

            // Shooting the game piece
            Commands.sequence(
                // Aim
                Commands.parallel(
                    Commands.runOnce(() -> RobotContainer.setLockedLocation(LockedLocation.SPEAKER)),
                    Commands
                        .runOnce(
                            () -> subShooter.setDesiredVelocities(prefShooter.leftShooterSpeakerVelocity.getValue(),
                                prefShooter.rightShooterSpeakerVelocity.getValue())),
                    Commands.runOnce(() -> subShooter.setIgnoreFlywheelSpeed(false))),

                // Shoot
                new TransferGamePiece(subShooter, subTurret, subTransfer, subPitch, subIntake)
                    .until(() -> !subTransfer.hasGamePiece).withTimeout(2),
                Commands.parallel(
                    Commands.runOnce(() -> subTransfer.setFeederNeutralOutput()),
                    Commands.runOnce(() -> subTransfer.setTransferNeutralOutput()))))

    );
  }

  public Supplier<Pose2d> getInitialPose() {
    return () -> (!FieldConstants.isRedAlliance())
        ? PathPlannerAuto.getStaringPoseFromAutoFile(determinePathName())
        : PathPlannerPath.fromChoreoTrajectory(determinePathName()).flipPath().getPreviewStartingHolonomicPose();
  }

  public String determinePathName() {
    return (goesDown) ? "PsW1sW2sW3s" : "PsW3sW2sW1s";
  }

  public Command getAutonomousCommand() {
    return this;
  }
}
