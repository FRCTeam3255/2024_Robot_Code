// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.WingOnly;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.LockedLocation;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.Shoot;
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

  PathPlannerPath PsW1sW2sW3s = PathPlannerPath.fromChoreoTrajectory("PsW1sW2sW3s.1");
  PathPlannerPath PsW1sW2sW3sFlipped = PathPlannerPath.fromChoreoTrajectory("PsW1sW2sW3s.1").flipPath();

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
        // Shoot preloaded game piece
        Commands.runOnce(() -> RobotContainer.setLockedLocation(LockedLocation.SPEAKER)),
        new Shoot(subShooter, subLEDs).until(() -> !subTransfer.calcGamePieceCollected()),

        RobotContainer.zeroPitch(), // I have no clue if this will work.. i think autobuilder will override it
        AutoBuilder.followPath(PsW1sW2sW3s),
        new Shoot(subShooter, subLEDs).until(() -> !subTransfer.calcGamePieceCollected()),

        AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("PsW1sW2sW3s.2")),
        new Shoot(subShooter, subLEDs).until(() -> !subTransfer.calcGamePieceCollected()),

        AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("PsW1sW2sW3s.3")),
        new Shoot(subShooter, subLEDs).until(() -> !subTransfer.calcGamePieceCollected()));
  }

  public Supplier<Pose2d> getInitialPose() {
    return () -> (FieldConstants.isRedAlliance())
        ? PsW1sW2sW3sFlipped.getStartingDifferentialPose()
        : PsW1sW2sW3s.getStartingDifferentialPose();
  }

  public Command getAutonomousCommand() {
    return this;
  }
}
