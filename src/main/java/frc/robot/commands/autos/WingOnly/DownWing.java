// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.WingOnly;

import java.util.function.Supplier;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.LockedLocation;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.IntakeGroundGamePiece;
import frc.robot.commands.LockPitch;
import frc.robot.commands.LockTurret;
import frc.robot.commands.TransferAuto;
import frc.robot.commands.autos.AutoInterface;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Turret;

public class DownWing extends SequentialCommandGroup implements AutoInterface {
  Drivetrain subDrivetrain;
  Intake subIntake;
  LEDs subLEDs;
  Pitch subPitch;
  Shooter subShooter;
  Transfer subTransfer;
  Turret subTurret;
  Climber subClimber;

  PathPlannerAuto PsW1sW2sW3s1 = new PathPlannerAuto("PsW1sW2sW3s.1");

  public DownWing(Drivetrain subDrivetrain, Intake subIntake, LEDs subLEDs, Pitch subPitch, Shooter subShooter,
      Transfer subTransfer, Turret subTurret, Climber subClimber) {
    this.subDrivetrain = subDrivetrain;
    this.subIntake = subIntake;
    this.subLEDs = subLEDs;
    this.subPitch = subPitch;
    this.subShooter = subShooter;
    this.subTransfer = subTransfer;
    this.subTurret = subTurret;
    this.subClimber = subClimber;

    addCommands(
        Commands.runOnce(
            () -> subDrivetrain.resetPoseToPose(PathPlannerAuto.getStaringPoseFromAutoFile("PsW1sW2sW3s.1"))),
        Commands.runOnce(() -> subDrivetrain.resetYaw(
            PathPlannerAuto.getStaringPoseFromAutoFile("PsW1sW2sW3s.1").getRotation().getDegrees())),

        // GET PRELOAD AND SHOOT IT
        Commands.runOnce(() -> RobotContainer.setLockedLocation(LockedLocation.SPEAKER)),
        new IntakeGroundGamePiece(subIntake, subTransfer, subTurret, subClimber, subPitch),
        Commands.runOnce(() -> subTransfer.setGamePieceCollected(true)),

        // SHOOTING
        Commands.runOnce(
            () -> subDrivetrain.resetPoseToPose(getInitialPose().get())),

        Commands.waitUntil(() -> !subClimber.collidesWithPitch()),
        new LockPitch(subPitch, subDrivetrain, subClimber).until(() -> subPitch.isPitchLocked()),
        Commands.waitUntil(() -> !subClimber.collidesWithTurret()),
        new LockTurret(subTurret, subDrivetrain, subClimber).until(() -> subTurret.isTurretLocked()),
        Commands.waitSeconds(0.5),
        new TransferAuto(subShooter, subTurret, subTransfer, subPitch),
        Commands.runOnce(() -> subTransfer.setGamePieceCollected(false)),
        Commands.waitSeconds(0.5),

        new PathPlannerAuto("PsW1sW2sW3s.1"),

        // W1 shoot
        // GET PRELOAD AND SHOOT IT
        Commands.runOnce(() -> RobotContainer.setLockedLocation(LockedLocation.SPEAKER)),
        new IntakeGroundGamePiece(subIntake, subTransfer, subTurret, subClimber, subPitch),
        Commands.runOnce(() -> subTransfer.setGamePieceCollected(true)),

        // SHOOTING
        Commands.runOnce(
            () -> subDrivetrain.resetPoseToPose(getInitialPose().get())),

        Commands.waitUntil(() -> !subClimber.collidesWithPitch()),
        new LockPitch(subPitch, subDrivetrain, subClimber).until(() -> subPitch.isPitchLocked()),
        Commands.waitUntil(() -> !subClimber.collidesWithTurret()),
        new LockTurret(subTurret, subDrivetrain, subClimber).until(() -> subTurret.isTurretLocked()),
        Commands.waitSeconds(0.5),
        new TransferAuto(subShooter, subTurret, subTransfer, subPitch),
        Commands.runOnce(() -> subTransfer.setGamePieceCollected(false)),
        Commands.waitSeconds(0.5),

        new PathPlannerAuto("PsW1sW2sW3s.2"),

        // W2
        // GET PRELOAD AND SHOOT IT
        Commands.runOnce(() -> RobotContainer.setLockedLocation(LockedLocation.SPEAKER)),
        new IntakeGroundGamePiece(subIntake, subTransfer, subTurret, subClimber, subPitch),
        Commands.runOnce(() -> subTransfer.setGamePieceCollected(true)),

        // SHOOTING
        Commands.runOnce(
            () -> subDrivetrain.resetPoseToPose(getInitialPose().get())),

        Commands.waitUntil(() -> !subClimber.collidesWithPitch()),
        new LockPitch(subPitch, subDrivetrain, subClimber).until(() -> subPitch.isPitchLocked()),
        Commands.waitUntil(() -> !subClimber.collidesWithTurret()),
        new LockTurret(subTurret, subDrivetrain, subClimber).until(() -> subTurret.isTurretLocked()),
        Commands.waitSeconds(0.5),
        new TransferAuto(subShooter, subTurret, subTransfer, subPitch),
        Commands.runOnce(() -> subTransfer.setGamePieceCollected(false)),
        Commands.waitSeconds(0.5),

        new PathPlannerAuto("PsW1sW2sW3s.3"),

        // W3
        // GET PRELOAD AND SHOOT IT
        Commands.runOnce(() -> RobotContainer.setLockedLocation(LockedLocation.SPEAKER)),
        new IntakeGroundGamePiece(subIntake, subTransfer, subTurret, subClimber, subPitch),
        Commands.runOnce(() -> subTransfer.setGamePieceCollected(true)),

        // SHOOTING
        Commands.runOnce(
            () -> subDrivetrain.resetPoseToPose(getInitialPose().get())),

        Commands.waitUntil(() -> !subClimber.collidesWithPitch()),
        new LockPitch(subPitch, subDrivetrain, subClimber).until(() -> subPitch.isPitchLocked()),
        Commands.waitUntil(() -> !subClimber.collidesWithTurret()),
        new LockTurret(subTurret, subDrivetrain, subClimber).until(() -> subTurret.isTurretLocked()),
        Commands.waitSeconds(0.5),
        new TransferAuto(subShooter, subTurret, subTransfer, subPitch),
        Commands.runOnce(() -> subTransfer.setGamePieceCollected(false)),
        Commands.waitSeconds(0.5));

  }

  public Supplier<Pose2d> getInitialPose() {
    return () -> (!FieldConstants.isRedAlliance())
        ? PathPlannerAuto.getStaringPoseFromAutoFile("PsW1sW2sW3s.1")
        : PathPlannerPath.fromPathFile("PsW1sW2sW3s.1").flipPath().getPreviewStartingHolonomicPose();
  }

  public Command getAutonomousCommand() {
    return this;
  }
}
