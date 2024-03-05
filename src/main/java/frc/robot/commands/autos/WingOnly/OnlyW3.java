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

public class OnlyW3 extends SequentialCommandGroup implements AutoInterface {
  Drivetrain subDrivetrain;
  Intake subIntake;
  LEDs subLEDs;
  Pitch subPitch;
  Shooter subShooter;
  Transfer subTransfer;
  Turret subTurret;
  Climber subClimber;

  public OnlyW3(Drivetrain subDrivetrain, Intake subIntake, LEDs subLEDs, Pitch subPitch, Shooter subShooter,
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
            () -> subDrivetrain.resetPoseToPose(PathPlannerAuto.getStaringPoseFromAutoFile("OnlyW3"))),
        Commands.runOnce(() -> subDrivetrain.resetYaw(
            PathPlannerAuto.getStaringPoseFromAutoFile("OnlyW3").getRotation().getDegrees())),

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

        new PathPlannerAuto("OnlyW2"),

        // W2 shoot
        new LockPitch(subPitch, subDrivetrain, subClimber).until(() -> subPitch.isPitchLocked()),
        new LockTurret(subTurret, subDrivetrain, subClimber).until(() -> subTurret.isTurretLocked()),
        new TransferAuto(subShooter, subTurret, subTransfer, subPitch),
        Commands.runOnce(() -> subTransfer.setGamePieceCollected(false)),
        Commands.waitSeconds(0.5));

  }

  public Supplier<Pose2d> getInitialPose() {
    return () -> (!FieldConstants.isRedAlliance())
        ? PathPlannerAuto.getStaringPoseFromAutoFile("OnlyW3")
        : PathPlannerPath.fromPathFile("OnlyW3").flipPath().getPreviewStartingHolonomicPose();
  }

  public Command getAutonomousCommand() {
    return this;
  }
}
