// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
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

public class Centerline extends SequentialCommandGroup implements AutoInterface {
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
  public Centerline(Drivetrain subDrivetrain, Intake subIntake, LEDs subLEDs, Pitch subPitch, Shooter subShooter,
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

        // Shoot Preload
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

        // Go get C5/1
        new UnaliveShooter(subShooter, subTurret, subPitch, subLEDs),
        AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(determineInitPathName() + ".1")),

        // We are now at C5 or C1
        Commands.either(
            Commands.sequence(
                // Shoot C5 or C1
                AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(determineScorePathName() + ".1")),
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
                                    () -> subShooter.setDesiredVelocities(
                                        prefShooter.leftShooterSpeakerVelocity.getValue(),
                                        prefShooter.rightShooterSpeakerVelocity.getValue())),
                            Commands.runOnce(() -> subShooter.setIgnoreFlywheelSpeed(false))),

                        // Shoot
                        new TransferGamePiece(subShooter, subTurret, subTransfer, subPitch, subIntake)
                            .until(() -> !subTransfer.hasGamePiece).withTimeout(2),
                        Commands.parallel(
                            Commands.runOnce(() -> subTransfer.setFeederNeutralOutput()),
                            Commands.runOnce(() -> subTransfer.setTransferNeutralOutput())))),
                // Return to centerline (C4 or C2)
                new UnaliveShooter(subShooter, subTurret, subPitch, subLEDs),
                AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(determineScorePathName() + ".2"))),

            // Hop to C4 or C2
            AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(determineHopPathName() + ".1")),
            () -> !subTransfer.hasGamePiece),

        // Either way, we just tried to get C4 or C2
        Commands.either(
            Commands.sequence(
                // Shoot C4 or C2
                AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(determineScorePathName() + ".3")),
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
                                    () -> subShooter.setDesiredVelocities(
                                        prefShooter.leftShooterSpeakerVelocity.getValue(),
                                        prefShooter.rightShooterSpeakerVelocity.getValue())),
                            Commands.runOnce(() -> subShooter.setIgnoreFlywheelSpeed(false))),

                        // Shoot
                        new TransferGamePiece(subShooter, subTurret, subTransfer, subPitch, subIntake)
                            .until(() -> !subTransfer.hasGamePiece).withTimeout(2),
                        Commands.parallel(
                            Commands.runOnce(() -> subTransfer.setFeederNeutralOutput()),
                            Commands.runOnce(() -> subTransfer.setTransferNeutralOutput())))),

                // Return to centerline (C3)
                new UnaliveShooter(subShooter, subTurret, subPitch, subLEDs),
                AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(determineScorePathName() + ".4"))),

            // Hop to C3
            AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(determineHopPathName() + ".2")),
            () -> !subTransfer.hasGamePiece),

        // Either way, we just tried to get C3
        Commands.either(
            Commands.sequence(
                // Shoot C3
                AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(determineScorePathName() + ".5")),
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
                                    () -> subShooter.setDesiredVelocities(
                                        prefShooter.leftShooterSpeakerVelocity.getValue(),
                                        prefShooter.rightShooterSpeakerVelocity.getValue())),
                            Commands.runOnce(() -> subShooter.setIgnoreFlywheelSpeed(false))),

                        // Shoot
                        new TransferGamePiece(subShooter, subTurret, subTransfer, subPitch, subIntake)
                            .until(() -> !subTransfer.hasGamePiece).withTimeout(2),
                        Commands.parallel(
                            Commands.runOnce(() -> subTransfer.setFeederNeutralOutput()),
                            Commands.runOnce(() -> subTransfer.setTransferNeutralOutput())))),

                // Return to centerline (C2 or C4)
                new UnaliveShooter(subShooter, subTurret, subPitch, subLEDs),
                AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(determineScorePathName() + ".6"))),

            // Hop to C2 or C4
            AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(determineHopPathName() + ".3")),
            () -> !subTransfer.hasGamePiece),

        // Either way, we just tried to get C2 or C4
        Commands.either(
            Commands.sequence(
                // Shoot C2 or C4
                AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(determineScorePathName() + ".7")),
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
                                    () -> subShooter.setDesiredVelocities(
                                        prefShooter.leftShooterSpeakerVelocity.getValue(),
                                        prefShooter.rightShooterSpeakerVelocity.getValue())),
                            Commands.runOnce(() -> subShooter.setIgnoreFlywheelSpeed(false))),

                        // Shoot
                        new TransferGamePiece(subShooter, subTurret, subTransfer, subPitch, subIntake)
                            .until(() -> !subTransfer.hasGamePiece).withTimeout(2),
                        Commands.parallel(
                            Commands.runOnce(() -> subTransfer.setFeederNeutralOutput()),
                            Commands.runOnce(() -> subTransfer.setTransferNeutralOutput())))),

                // Return to centerline (C1 or C5)
                new UnaliveShooter(subShooter, subTurret, subPitch, subLEDs),
                AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(determineScorePathName() + ".8"))),

            // Hop to C1 or C5
            AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(determineHopPathName() + ".4")),
            () -> !subTransfer.hasGamePiece),

        // Either way, we just tried to get C1 or C5. Just shoot it
        // Shoot C1 or C5
        AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(determineScorePathName() + ".9")),
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
                            () -> subShooter.setDesiredVelocities(
                                prefShooter.leftShooterSpeakerVelocity.getValue(),
                                prefShooter.rightShooterSpeakerVelocity.getValue())),
                    Commands.runOnce(() -> subShooter.setIgnoreFlywheelSpeed(false))),

                // Shoot
                new TransferGamePiece(subShooter, subTurret, subTransfer, subPitch, subIntake)
                    .until(() -> !subTransfer.hasGamePiece).withTimeout(2),
                Commands.parallel(
                    Commands.runOnce(() -> subTransfer.setFeederNeutralOutput()))))

    );
  }

  public Supplier<Pose2d> getInitialPose() {
    return () -> (!FieldConstants.isRedAlliance())
        ? PathPlannerAuto.getStaringPoseFromAutoFile(determineInitPathName())
        : PathPlannerPath.fromChoreoTrajectory(determineInitPathName()).flipPath().getPreviewStartingHolonomicPose();
  }

  public String determineInitPathName() {
    return (goesDown) ? "D PsC1" : "U PsC5";
  }

  public String determineScorePathName() {
    return (goesDown) ? "D C1sUntilC5s" : "U C5sUntilC1s";
  }

  public String determineHopPathName() {
    return (goesDown) ? "D C1UntilC5" : "U C5UntilC1";
  }

  public Command getAutonomousCommand() {
    return this;
  }
}
