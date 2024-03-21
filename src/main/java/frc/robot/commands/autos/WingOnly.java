// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.util.function.Supplier;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.LockedLocation;
import frc.robot.RobotPreferences.prefShooter;
import frc.robot.RobotPreferences.prefTransfer;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.IntakeGroundGamePiece;
import frc.robot.commands.LockPitch;
import frc.robot.commands.LockTurret;
import frc.robot.commands.TransferGamePiece;
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

        Commands.runOnce(() -> subShooter.setDesiredVelocities(prefShooter.leftShooterSpeakerVelocity.getValue(),
            prefShooter.rightShooterSpeakerVelocity.getValue())),
        Commands.runOnce(() -> subShooter.getUpToSpeed()),
        Commands.runOnce(() -> RobotContainer.setLockedLocation(LockedLocation.SPEAKER)),
        Commands.runOnce(() -> subTurret.setTurretGoalAngle(-3255)),
        Commands.runOnce(() -> subPitch.setPitchGoalAngle(-3255)),
        Commands.runOnce(() -> subTransfer.setTransferSensorAngle(0)),
        Commands.runOnce(() -> subShooter.setIgnoreFlywheelSpeed(false)),

        // throw out that intake
        // Intake until we have the game piece
        new IntakeGroundGamePiece(subIntake, subTransfer, subTurret, subPitch, subShooter, subClimber),

        // PRELOAD
        Commands.race(
            // Shooting the game piece
            Commands.sequence(
                // Aim
                Commands.parallel(
                    new LockTurret(subTurret, subDrivetrain).repeatedly().until(() -> subTurret.isTurretLocked()),
                    new LockPitch(subPitch, subDrivetrain).repeatedly().until(() -> subPitch.isPitchLocked())),
                Commands.runOnce(() -> subShooter.getUpToSpeed()),

                // Shoot
                new TransferGamePiece(subShooter, subTurret, subTransfer, subPitch, subIntake)
                    .until(() -> subTransfer.calcGPShotAuto()),
                Commands.runOnce(() -> subIntake.setIntakeRollerSpeed(0)))),

        new PathPlannerAuto(determinePathName() + ".1"),
        new IntakeGroundGamePiece(subIntake, subTransfer, subTurret, subPitch, subShooter, subClimber),
        Commands.waitUntil(() -> subTransfer.hasRepositioned == true),

        Commands.runOnce(() -> subTurret.setTurretGoalAngle(-3255)),
        Commands.runOnce(() -> subPitch.setPitchGoalAngle(-3255)),
        Commands.runOnce(() -> subTransfer.setTransferSensorAngle(0)),

        // SHOOT W3
        Commands.race(
            // Shooting the game piece
            Commands.sequence(
                // Aim
                Commands.parallel(
                    new LockTurret(subTurret, subDrivetrain).repeatedly().until(() -> subTurret.isTurretLocked()),
                    new LockPitch(subPitch, subDrivetrain).repeatedly().until(() -> subPitch.isPitchLocked())),
                Commands.runOnce(() -> subShooter.getUpToSpeed()),

                // Shoot
                new TransferGamePiece(subShooter, subTurret, subTransfer, subPitch, subIntake)
                    .until(() -> subTransfer.calcGPShotAuto()),
                Commands.runOnce(() -> subIntake.setIntakeRollerSpeed(0)))),

        new PathPlannerAuto(determinePathName() + ".2"),
        new IntakeGroundGamePiece(subIntake, subTransfer, subTurret, subPitch, subShooter, subClimber),
        Commands.waitUntil(() -> subTransfer.hasRepositioned == true),

        Commands.runOnce(() -> subTurret.setTurretGoalAngle(-3255)),
        Commands.runOnce(() -> subPitch.setPitchGoalAngle(-3255)),
        Commands.runOnce(() -> subTransfer.setTransferSensorAngle(0)),

        // SHOOT W2
        Commands.race(
            // Shooting the game piece
            Commands.sequence(
                // Aim
                Commands.parallel(
                    new LockTurret(subTurret, subDrivetrain).repeatedly().until(() -> subTurret.isTurretLocked()),
                    new LockPitch(subPitch, subDrivetrain).repeatedly().until(() -> subPitch.isPitchLocked())),
                Commands.runOnce(() -> subShooter.getUpToSpeed()),

                // Shoot
                new TransferGamePiece(subShooter, subTurret, subTransfer, subPitch, subIntake)
                    .until(() -> subTransfer.calcGPShotAuto()),
                Commands.runOnce(() -> subIntake.setIntakeRollerSpeed(0)))),
        new PathPlannerAuto(determinePathName() + ".3"),
        new IntakeGroundGamePiece(subIntake, subTransfer, subTurret, subPitch,
            subShooter, subClimber),
        Commands.waitUntil(() -> subTransfer.hasRepositioned == true),

        Commands.runOnce(() -> subTurret.setTurretGoalAngle(-3255)),
        Commands.runOnce(() -> subPitch.setPitchGoalAngle(-3255)),
        Commands.runOnce(() -> subTransfer.setTransferSensorAngle(0)),

        // SHOOT W3
        Commands.race(
            // Shooting the game piece
            Commands.sequence(
                // Aim
                Commands.parallel(
                    new LockTurret(subTurret, subDrivetrain).repeatedly().until(() -> subTurret.isTurretLocked()),
                    new LockPitch(subPitch, subDrivetrain).repeatedly().until(() -> subPitch.isPitchLocked())),
                Commands.runOnce(() -> subShooter.getUpToSpeed()),

                // Shoot
                new TransferGamePiece(subShooter, subTurret, subTransfer, subPitch, subIntake)
                    .until(() -> subTransfer.calcGPShotAuto()),
                Commands.runOnce(() -> subIntake.setIntakeRollerSpeed(0))))

    );
  }

  public Supplier<Pose2d> getInitialPose() {
    return () -> (!FieldConstants.isRedAlliance())
        ? PathPlannerAuto.getStaringPoseFromAutoFile(determinePathName())
        : PathPlannerPath.fromPathFile(determinePathName()).flipPath().getPreviewStartingHolonomicPose();
  }

  public String determinePathName() {
    // return (goesDown) ? "PsW1sW2sW3s" : "PsW3sW2sW1s";
    return "PsW1sW2sW3s";
  }

  public Command getAutonomousCommand() {
    return this;
  }
}
