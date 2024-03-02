// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.WingOnly;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.LockedLocation;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.Shoot;
import frc.robot.commands.TransferGamePiece;
import frc.robot.commands.autos.AutoInterface;
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

  PathPlannerAuto PsW1sW2sW3s1 = new PathPlannerAuto("PsW1sW2sW3s.1");

  public DownWing(Drivetrain subDrivetrain, Intake subIntake, LEDs subLEDs, Pitch subPitch, Shooter subShooter,
      Transfer subTransfer, Turret subTurret) {
    this.subDrivetrain = subDrivetrain;
    this.subIntake = subIntake;
    this.subLEDs = subLEDs;
    this.subPitch = subPitch;
    this.subShooter = subShooter;
    this.subTransfer = subTransfer;
    this.subTurret = subTurret;

    addCommands(
        Commands.parallel(
            RobotContainer.zeroPitch(),
            Commands.runOnce(() -> subDrivetrain.resetPoseToPose(getInitialPose().get())),
            Commands.runOnce(() -> subDrivetrain.resetYaw(getInitialPose().get().getRotation().getDegrees()))),

        Commands.parallel(new Shoot(subShooter, subLEDs).repeatedly(),
            // SHOOT PRELOAD
            Commands.sequence(
                // get preload
                Commands.runOnce(() -> RobotContainer.setLockedLocation(LockedLocation.SPEAKER)),
                Commands.runOnce(() -> subTransfer.setGamePieceCollected(true)),

                // shoot preload
                new TransferGamePiece(subShooter, subTurret, subTransfer, subPitch)
                    .until(() -> !subTransfer.hasGamePiece))),

        // W1
        new PathPlannerAuto("PsW1sW2sW3s.1"),
        Commands.parallel(new Shoot(subShooter, subLEDs).repeatedly(),
            // shoot W1
            new TransferGamePiece(subShooter, subTurret, subTransfer, subPitch)
                .until(() -> !subTransfer.hasGamePiece)),

        // W2
        new PathPlannerAuto("PsW1sW2sW3s.2"),
        Commands.parallel(new Shoot(subShooter, subLEDs).repeatedly(),
            // shoot W2
            new TransferGamePiece(subShooter, subTurret, subTransfer, subPitch)
                .until(() -> !subTransfer.hasGamePiece)),
        // W3
        new PathPlannerAuto("PsW1sW2sW3s.3"),
        Commands.parallel(new Shoot(subShooter, subLEDs).repeatedly(),
            // shoot W3
            new TransferGamePiece(subShooter, subTurret, subTransfer, subPitch)
                .until(() -> !subTransfer.hasGamePiece)));

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
