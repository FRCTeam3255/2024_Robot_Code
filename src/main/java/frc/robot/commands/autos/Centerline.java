// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.util.function.DoubleSupplier;
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
import frc.robot.commands.AimAuto;
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

    // AutoBuilder.buildAuto(determineInitPathName());
    // AutoBuilder.buildAuto(determineScorePathName() + ".1");
    // AutoBuilder.buildAuto(determineScorePathName() + ".2");
    // AutoBuilder.buildAuto(determineScorePathName() + ".3");
    // AutoBuilder.buildAuto(determineScorePathName() + ".4");

    // AutoBuilder.buildAuto(determineHopPathName() + ".1");
    // AutoBuilder.buildAuto(determineHopPathName() + ".2");

    addCommands(
        Commands.runOnce(
            () -> subDrivetrain.resetPoseToPose(getInitialPose().get())),
        Commands.runOnce(() -> subDrivetrain.resetYaw(
            getInitialPose().get().getRotation().getDegrees())),

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
            Commands.run(() -> subTurret.setTurretAngle(getTurretInitAngle().getAsDouble()))
                .until(() -> subTurret.isTurretAtAngle(getTurretInitAngle().getAsDouble())),
            Commands.run(() -> subPitch.setPitchAngle(getPitchInitAngle().getAsDouble()))
                .until(() -> subPitch.isPitchAtAngle(getPitchInitAngle().getAsDouble()))),

        Commands.runOnce(() -> subShooter.getUpToSpeed()),

        Commands.runOnce(() -> subShooter.getUpToSpeed()),

        // Shoot
        new TransferGamePiece(subShooter, subTurret, subTransfer, subPitch, subIntake, subClimber)
            .until(() -> subTransfer.calcGPShotAuto()),
        Commands.runOnce(() -> subIntake.setIntakeRollerSpeed(0)),

        // Go get C5/1
        new PathPlannerAuto(determineInitPathName()),
        new IntakeGroundGamePiece(subIntake, subTransfer, subTurret, subPitch, subShooter, subClimber)
            .withTimeout(0.25),
        Commands.waitSeconds(1),

        // We are now at C5 or C1
        Commands.either(
            Commands.sequence(
                // Drive to shoot
                new PathPlannerAuto(determineScorePathName() + ".1"),
                // SHOOT C5
                // Aim
                new AimAuto(subPitch, subTurret, subDrivetrain),
                Commands.runOnce(() -> subShooter.getUpToSpeed()),

                // Shoot
                new TransferGamePiece(subShooter, subTurret, subTransfer, subPitch, subIntake, subClimber)
                    .until(() -> subTransfer.calcGPShotAuto()),
                Commands.runOnce(() -> subIntake.setIntakeRollerSpeed(0)),

                // Return to centerline (C4 or C2)
                new PathPlannerAuto(determineScorePathName() + ".2")),

            // Hop to C4 or C2
            new PathPlannerAuto(determineHopPathName() + ".1"),
            () -> subTransfer.hasGamePiece),

        // Either way, we just tried to get C4 or C2
        new IntakeGroundGamePiece(subIntake, subTransfer, subTurret, subPitch, subShooter, subClimber)
            .withTimeout(0.25),
        Commands.waitSeconds(1),

        Commands.either(
            Commands.sequence(
                // Drive to shoot
                new PathPlannerAuto(determineScorePathName() + ".3"),
                // SHOOT C4/2
                // Aim
                new AimAuto(subPitch, subTurret, subDrivetrain),
                Commands.runOnce(() -> subShooter.getUpToSpeed()),

                // Shoot
                new TransferGamePiece(subShooter, subTurret, subTransfer, subPitch, subIntake, subClimber)
                    .until(() -> subTransfer.calcGPShotAuto()),
                Commands.runOnce(() -> subIntake.setIntakeRollerSpeed(0)),

                // Return to centerline (C3)
                new PathPlannerAuto(determineScorePathName() + ".4")),

            // Hop to C3
            new PathPlannerAuto(determineHopPathName() + ".2"),
            () -> subTransfer.hasGamePiece)

    );
  }

  public Supplier<Pose2d> getInitialPose() {
    return () -> (!FieldConstants.isRedAlliance())
        ? PathPlannerAuto.getStaringPoseFromAutoFile(determineInitPathName())
        : PathPlannerPath.fromPathFile(determineInitPathName()).flipPath().getPreviewStartingHolonomicPose();
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

  public DoubleSupplier getTurretInitAngle() {
    // return () -> (goesDown) ? ((FieldConstants.isRedAlliance()) ? -30.613 :
    // 30.613)
    // : ((FieldConstants.isRedAlliance()) ? 0 : 0);
    return () -> 0;
  }

  public DoubleSupplier getPitchInitAngle() {
    // return () -> (goesDown) ? (46.349) : (55);
    return () -> 55;
  }

  public Command getAutonomousCommand() {
    return this;
  }
}
